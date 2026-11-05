"""运行固定的双臂对称伸展与回收任务并输出姿态和末端误差。"""

import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from centaur_control.controllers.pd import apply_joint_targets
from centaur_control.controllers.pd import current_joint_targets
from centaur_control.controllers.pd import support_posture_targets
from centaur_control.models.definitions import HAND_SITES
from centaur_control.paths import MODEL_PATH
from centaur_control.models.definitions import actuator_dofs
from centaur_control.models.definitions import object_id
from centaur_control.kinematics.velocity_qp import solve


from centaur_control.paths import PROJECT_ROOT as ROOT
DT = 0.002
DURATION = 4.0


def set_initial_state(model, data, targets):
    data.qpos[2] = 0.4559
    for leg in ("FL", "FR", "RL", "RR"):
        for suffix, value in (("hip", 0.0), ("thigh", 0.2), ("calf", -0.9), ("foot", 0.0)):
            joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_{suffix}")
            data.qpos[model.jnt_qposadr[joint_id]] = value
    mujoco.mj_forward(model, data)
    targets[:] = current_joint_targets(model, data)
    apply_joint_targets(model, data, targets)


def euler_roll_pitch(quaternion):
    w, x, y, z = quaternion
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    return roll, pitch


def wheel_contacts(model, data):
    wheels = set()
    for contact in data.contact[:data.ncon]:
        first = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or ""
        second = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or ""
        if "floor" in (first, second) and "_wheel" in (first + second):
            wheels.add(first if "_wheel" in first else second)
    return len(wheels)


def run(on_frame=None):
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    model.opt.timestep = DT
    data = mujoco.MjData(model)
    targets = np.zeros(model.nu)
    set_initial_state(model, data, targets)
    initial_wheel_count = wheel_contacts(model, data)
    if initial_wheel_count < 4:
        raise RuntimeError("初始状态没有建立四轮地面接触，请检查轮端几何或站立姿态")

    static_data = mujoco.MjData(model)
    static_targets = np.zeros(model.nu)
    set_initial_state(model, static_data, static_targets)
    static_max_tilt = 0.0
    static_max_drift = 0.0
    print("阶段 1/2：静止 PD 保持，仿真时间 2 秒。", flush=True)
    if on_frame is not None:
        on_frame(model, static_data, 0.0)
    for step in range(round(2.0 / DT)):
        apply_joint_targets(model, static_data, support_posture_targets(model, static_data, static_targets))
        mujoco.mj_step(model, static_data)
        roll, pitch = euler_roll_pitch(static_data.qpos[3:7])
        static_max_tilt = max(static_max_tilt, float(np.hypot(roll, pitch)))
        static_max_drift = max(static_max_drift, float(np.linalg.norm(static_data.qpos[:2])))
        if wheel_contacts(model, static_data) < 4:
            raise RuntimeError("静止 PD 基线失去轮地接触，暂停双臂测试")
        if on_frame is not None and (step + 1) % 8 == 0:
            on_frame(model, static_data, (step + 1) * DT)
    if static_max_tilt > 0.08 or static_max_drift > 0.08:
        raise RuntimeError("静止 PD 基线的机身姿态或位置漂移超过测试阈值")

    print("阶段 2/2：重置为相同初始状态，执行双臂伸展与回收，仿真时间 4 秒。", flush=True)
    if on_frame is not None:
        on_frame(model, data, 2.0)
    start = np.array([data.site_xpos[object_id(model, mujoco.mjtObj.mjOBJ_SITE, name)].copy() for name in HAND_SITES])
    max_tilt = 0.0
    squared_error = 0.0
    samples = 0
    minimum_contacts = 4
    steps = round(DURATION / DT)
    actuator_joint_dofs = actuator_dofs(model)
    active = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id)
              for actuator_id in range(model.nu)
              if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id) or "").startswith(
                  ("left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist"))}
    for step in range(steps):
        phase = step * DT / DURATION
        envelope = np.sin(np.pi * phase) ** 2
        offset = envelope * np.array([[0.045, 0.02, 0.02], [0.045, -0.02, 0.02]])
        goals = start + offset
        qdot = solve(model, data, goals, active_actuators=active)
        for actuator_id, dof in enumerate(actuator_joint_dofs):
            joint_id = int(model.actuator_trnid[actuator_id, 0])
            qadr = model.jnt_qposadr[joint_id]
            targets[actuator_id] += qdot[actuator_id] * DT
            if model.jnt_limited[joint_id]:
                targets[actuator_id] = np.clip(targets[actuator_id], *model.jnt_range[joint_id])
        apply_joint_targets(model, data, support_posture_targets(model, data, targets))
        mujoco.mj_step(model, data)

        roll, pitch = euler_roll_pitch(data.qpos[3:7])
        max_tilt = max(max_tilt, float(np.hypot(roll, pitch)))
        minimum_contacts = min(minimum_contacts, wheel_contacts(model, data))
        mujoco.mj_forward(model, data)
        errors = [np.linalg.norm(data.site_xpos[object_id(model, mujoco.mjtObj.mjOBJ_SITE, name)] - goal)
                  for name, goal in zip(HAND_SITES, goals)]
        squared_error += sum(error * error for error in errors)
        samples += len(errors)
        if on_frame is not None and (step + 1) % 8 == 0:
            on_frame(model, data, 2.0 + (step + 1) * DT)

    result = {
        "task": "双手对称伸展并回收，4 秒，2 ms 步长",
        "steps": steps,
        "actuated_joints_pd_controlled": int(model.nu),
        "initial_wheels_in_contact": int(initial_wheel_count),
        "static_pd_baseline_2s": {
            "maximum_base_tilt_rad": static_max_tilt,
            "maximum_xy_drift_m": static_max_drift,
            "minimum_wheels_in_contact": 4,
        },
        "minimum_wheels_in_contact_during_task": int(minimum_contacts),
        "maximum_base_tilt_rad": max_tilt,
        "hand_position_rmse_m": float(np.sqrt(squared_error / samples)),
        "final_base_position_world_m": data.qpos[:3].tolist(),
        "final_hand_positions_world_m": [data.site_xpos[object_id(model, mujoco.mjtObj.mjOBJ_SITE, name)].tolist() for name in HAND_SITES],
        "result": "通过：静止 PD 基线和双臂 IK-QP 任务均完成，四轮持续接触" if minimum_contacts >= 4 and np.isfinite(max_tilt) else "失败",
    }
    if minimum_contacts < 4 or not np.isfinite(max_tilt):
        raise RuntimeError(json.dumps(result, ensure_ascii=False, indent=2))
    result_path = ROOT / "centaur_control" / "bilateral_test_results.json"
    result_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


def run_with_viewer():
    """在同一测试执行期间显示仿真状态，完成后保持最终画面供检查。"""
    import mujoco.viewer

    with ExitStack() as stack:
        viewer = None
        display_data = None
        start_time = None

        def show_frame(model, data, elapsed):
            nonlocal viewer, display_data, start_time
            if viewer is None:
                # 使用独立显示数据，避免查看器交互改变验收测试的物理状态。
                display_data = mujoco.MjData(model)
                display_data.qpos[:] = data.qpos
                mujoco.mj_forward(model, display_data)
                viewer = stack.enter_context(mujoco.viewer.launch_passive(model, display_data))
                with viewer.lock():
                    viewer.cam.lookat[:] = [0.05, 0.0, 0.50]
                    viewer.cam.distance = 1.9
                    viewer.cam.azimuth = 135
                    viewer.cam.elevation = -20
                    viewer.opt.geomgroup[3] = 0
                start_time = time.perf_counter()
                print("MuJoCo 可视化已启动；关闭窗口可结束检查。", flush=True)
            if not viewer.is_running():
                raise InterruptedError("可视化窗口已关闭，测试中止；本次不写入通过结果。")
            with viewer.lock():
                display_data.qpos[:] = data.qpos
                display_data.qvel[:] = data.qvel
                display_data.ctrl[:] = data.ctrl
                display_data.time = data.time
                mujoco.mj_forward(model, display_data)
            viewer.sync()
            remaining = start_time + elapsed - time.perf_counter()
            if remaining > 0:
                time.sleep(min(remaining, 0.05))

        run(on_frame=show_frame)
        print("测试完成，窗口保持最终姿态；可旋转、缩放检查，关闭窗口后退出。", flush=True)
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(description="双臂 PD 与 IK-QP 测试及 MuJoCo 可视化检查")
    parser.add_argument("--viewer", action="store_true", help="显示静止保持和双臂动作过程，完成后保持窗口")
    args = parser.parse_args()
    try:
        run_with_viewer() if args.viewer else run()
    except (InterruptedError, KeyboardInterrupt) as error:
        print(str(error) or "检查已由用户中止。", flush=True)
        raise SystemExit(130)


if __name__ == "__main__":
    main()
