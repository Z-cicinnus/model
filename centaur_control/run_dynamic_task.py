"""平地四轮支撑。运行 python -m centaur_control.run_dynamic_task --angle 5"""

import argparse
import csv
import time
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

from .models.state import nominal_state
from .controllers.pd import current_joint_targets

LEGS = ("FL", "FR", "RL", "RR")


def reference(t, angle, turn_rate, turn_speed):
    # vx, yaw rate, roll (positive = right lean), pitch (positive = forward).
    stages = [
        (0, 0, 0, 0),
        (0.25, 0, 0, 0),
        (turn_speed, turn_rate, -angle, 0),
        (0, 0, 0, 0),
        (-0.20, 0, 0, 0),
        (0, 0, 0, angle),
        (0, 0, 0, -angle),
        (0, 0, angle, 0),
        (0, 0, 0, 0),
    ]
    starts = (0, 4, 8, 20, 24, 28, 32, 36, 40)
    i = np.searchsorted(starts, t, side="right") - 1
    u = min((t - starts[i]) / 1.5, 1.0)
    return np.array(stages[max(i - 1, 0)]) + u * u * (3 - 2 * u) * (
        np.array(stages[i]) - stages[max(i - 1, 0)]
    )


def support_loads(xy, weight, moment):
    # Vertical slice of the centroidal wrench QP. Gravity and roll/pitch only.
    A = np.array([np.ones(4), xy[:, 1], -xy[:, 0]])
    target = np.r_[weight, moment]
    H = A.T @ A + 0.0001 * np.eye(4)
    b = A.T @ target + 0.0001 * weight / 4
    f = np.linalg.solve(H, b)
    if f.min() < 0:
        result = minimize(
            lambda x: 0.5 * x @ H @ x - b @ x,
            np.maximum(f, 0),
            jac=lambda x: H @ x - b,
            bounds=[(0, None)] * 4,
            method="L-BFGS-B",
        )
        if not result.success:
            raise RuntimeError(result.message)
        f = result.x
    return f


def run(
    angle,
    output,
    view=False,
    turn_rate=0.05,
    turn_speed=0.25,
    command=None,
    duration=44.0,
    diagnostics=False,
    command_ramp=1.5,
    task=None,
    playback_speed=1.0,
    physics_hz=100,
):
    if not np.isfinite(playback_speed) or playback_speed <= 0:
        raise ValueError("playback_speed must be finite and positive")
    if command_ramp <= 0 or not np.isfinite(command_ramp):
        raise ValueError("command_ramp must be finite and positive")
    if duration <= 0 or not np.isfinite(duration):
        raise ValueError("duration must be finite and positive")
    if command is not None:
        command = np.asarray(command, dtype=float)
        if command.shape != (4,) or not np.isfinite(command).all():
            raise ValueError("command must contain finite vx, yaw_rate, roll, pitch")
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    m = mj.MjModel.from_xml_path(str(Path(__file__).with_name("scene.xml")))
    if physics_hz not in (100, 200, 500):
        raise ValueError("physics_hz must be 100, 200 or 500; default is 100")
    m.opt.timestep = 1.0 / physics_hz
    control_stride = max(1, round(0.01 / m.opt.timestep))
    control_dt = control_stride * m.opt.timestep
    d = nominal_state(m)
    target = current_joint_targets(m, d)
    d.ctrl[:] = target
    root = m.body("base_link").id
    gids = [m.geom(k + "_foot_wheel").id for k in LEGS]
    floor = m.geom("floor").id
    legs = np.array(
        [
            [
                m.actuator(k + "_" + j + "_pd").id
                for j in ("hip", "thigh", "calf", "foot")
            ]
            for k in LEGS
        ]
    )
    joints = m.actuator_trnid[legs, 0]
    dofs, pos = m.jnt_dofadr[joints], m.jnt_qposadr[joints]
    centers = d.geom_xpos[gids].copy()
    offsets = centers - d.xpos[root]
    radius = m.geom_size[gids[0], 0]
    weight = m.body_mass.sum() * abs(m.opt.gravity[2])
    yaw_i = 0.0
    speed_i = 0.0
    tilt_i = np.zeros(2)
    wheel_speeds = np.zeros(4)
    # Four wheel drives use torque commands in this simulation instance.
    wheels = legs[:, 3]
    m.actuator_biasprm[wheels, :] = 0
    m.actuator_gainprm[wheels, 0] = 1
    m.actuator_ctrlrange[wheels] = m.actuator_forcerange[wheels]
    if task is not None:
        task.setup(m, d, root)
    viewer = None
    if view:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(m, d)
    wall_start = time.perf_counter()
    with open(output, "w", newline="") as file:
        writer = csv.writer(file)
        columns = "t vx_ref yaw_ref roll_ref_deg pitch_ref_deg vx yaw_rate roll_deg pitch_deg FL_N FR_N RL_N RR_N peak_leg_wheel_util".split()
        if diagnostics:
            columns += ["base_z", "x", "y"] + [f"{leg}_slip_mps" for leg in LEGS]
            columns += ["min_leg_limit_margin_rad", "other_ground_contacts"]
        writer.writerow(columns)
        for step in range(round(duration / m.opt.timestep)):
            mj.mj_forward(m, d)
            R = d.xmat[root].reshape(3, 3)
            roll, pitch, yaw = Rotation.from_matrix(R).as_euler("xyz")
            heading = Rotation.from_euler("z", yaw).as_matrix()
            vx = heading[:, 0] @ d.qvel[:3]
            wz = (R @ d.qvel[3:6])[2]
            requested = reference(d.time, angle, turn_rate, turn_speed)
            if command is not None:
                u = np.clip((d.time - 4.0) / command_ramp, 0.0, 1.0)
                requested = command * u * u * (3 - 2 * u)
            if task is not None:
                requested = task.update(m, d, requested)
            v, w, r, p = requested
            tilt = np.array([r, p])
            if step % control_stride == 0:
                tilt_i = np.clip(tilt_i + control_dt * (tilt - [roll, pitch]), -0.12, 0.12)
            desired = Rotation.from_euler("xyz", [*(tilt + tilt_i), yaw]).as_matrix()
            if step % control_stride == 0 and not getattr(task, "owns_control", False):
                # Preserve stance width and fore/aft wheel offsets in the yaw frame.
                goal = offsets @ heading.T
                goal[:, 2] += 0.09 * min(d.time / 2, 1)
                if task is not None:
                    goal[:, 2] = radius - task.height - 0.5 * (task.height - d.xpos[root, 2])
                goal = goal @ desired
                err = (goal - (d.geom_xpos[gids] - d.xpos[root]) @ R) @ R.T
                moment = heading[:2, :2] @ (
                    120 * np.array([r - roll, p - pitch]) - 12 * d.qvel[3:5]
                )
                loads = support_loads(
                    d.geom_xpos[gids, :2] - d.subtree_com[root, :2], weight, moment
                )
                speed_i = np.clip(speed_i + control_dt * (v - vx), -0.3, 0.3)
                yaw_i = np.clip(yaw_i + control_dt * (w - wz), -1.5, 1.5)
                for i, g in enumerate(gids):
                    jac = np.zeros((3, m.nv))
                    mj.mj_jacGeom(m, d, jac, None, g)
                    J = jac[:, dofs[i, :3]]
                    dq = np.linalg.solve(J.T @ J + 0.0003 * np.eye(3), J.T @ err[i])
                    a = legs[i, :3]
                    feed = d.qfrc_bias[dofs[i, :3]] - J[2] * loads[i]
                    d.ctrl[a] = np.clip(
                        d.qpos[pos[i, :3]] + dq + feed / m.actuator_gainprm[a, 0],
                        *m.actuator_ctrlrange[a].T,
                    )
                    turn = w + 2 * (w - wz) + 24 * yaw_i
                    speed = (
                        v + 0.5 * (v - vx) + 3 * speed_i - turn * offsets[i, 1]
                    ) / radius
                    wheel_speeds[i] = speed
            # Wheel speed feedback in torque mode; retain the XML torque limit.
            if step % control_stride == 0:
                if not getattr(task, "owns_control", False):
                    torque = 1.6 * (wheel_speeds - d.qvel[dofs[:, 3]])
                    d.ctrl[wheels] = np.clip(torque, *m.actuator_ctrlrange[wheels].T)
                if task is not None:
                    task.apply(m, d)
            force = np.zeros(4)
            slip_squared = np.zeros(4)
            for k, c in enumerate(d.contact):
                pair = (c.geom1, c.geom2)
                if (
                    floor in pair
                    and (wheel := next((g for g in pair if g in gids), None))
                    is not None
                ):
                    wrench = np.zeros(6)
                    mj.mj_contactForce(m, d, k, wrench)
                    wheel_index = gids.index(wheel)
                    normal_force = max(wrench[0], 0.0)
                    force[wheel_index] += normal_force
                    if diagnostics:
                        # Velocity of the wheel material point at the actual contact.
                        # All joint velocities, including wheel spin, are included.
                        contact_jac = np.zeros((3, m.nv))
                        mj.mj_jac(
                            m, d, contact_jac, None, c.pos, int(m.geom_bodyid[wheel])
                        )
                        velocity = contact_jac @ d.qvel
                        normal = c.frame[:3]
                        tangential = velocity - normal * (normal @ velocity)
                        slip_squared[wheel_index] += normal_force * (
                            tangential @ tangential
                        )
            util = np.max(abs(d.actuator_force[legs]) / m.actuator_forcerange[legs, 1])
            row = [
                d.time,
                v,
                w,
                *np.rad2deg([r, p]),
                vx,
                wz,
                *np.rad2deg([roll, pitch]),
                *force,
                util,
            ]
            if diagnostics:
                slip = np.sqrt(
                    np.divide(
                        slip_squared, force, out=np.full(4, np.nan), where=force > 1e-6
                    )
                )
                row += [d.xpos[root, 2], d.xpos[root, 0], d.xpos[root, 1], *slip]
                bounds = m.jnt_range[joints[:, :3]]
                angles = d.qpos[pos[:, :3]]
                margin = np.minimum(
                    angles - bounds[:, :, 0], bounds[:, :, 1] - angles
                ).min()
                other_ground = sum(
                    floor in (c.geom1, c.geom2)
                    and not any(g in gids for g in (c.geom1, c.geom2))
                    for c in d.contact
                )
                row += [margin, other_ground]
            writer.writerow(row)
            if task is not None:
                task.record(m, d, force)
            if not np.isfinite(d.qpos).all() or max(abs(roll), abs(pitch)) > np.deg2rad(60):
                raise RuntimeError(
                    "Simulation stopped: nonfinite state or attitude exceeds 60 degrees"
                )
            if viewer:
                if not viewer.is_running():
                    break
                viewer.sync()
                remaining = wall_start + d.time / playback_speed - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
            mj.mj_step(m, d)
    if viewer:
        viewer.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--angle", type=float, default=5)
    parser.add_argument("--output", default="dynamic_trace.csv")
    parser.add_argument(
        "--turn-rate", type=float, default=0.05, help="转弯角速度 rad/s"
    )
    parser.add_argument("--view", action="store_true")
    parser.add_argument(
        "--turn-speed", type=float, default=0.25, help="弧线段前向速度 m/s"
    )
    args = parser.parse_args()
    if not 0 <= args.angle <= 60:
        parser.error("--angle must be in [0,60] degrees")
    if not np.all(np.isfinite([args.turn_rate, args.turn_speed])):
        parser.error("--turn-rate and --turn-speed must be finite")
    run(np.deg2rad(args.angle), args.output, args.view, args.turn_rate, args.turn_speed)
