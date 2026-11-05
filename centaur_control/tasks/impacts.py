"""半正弦冲击的闭环对照、局部失稳区间扫描与可视化轨迹记录。"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from centaur_control.paths import MODEL_PATH
from centaur_control.models.state import nominal_state


from centaur_control.paths import IMPACTS_DIR as OUT
DIRECTIONS = {"forward": [1., 0., 0.], "backward": [-1., 0., 0.],
              "left": [0., 1., 0.], "right": [0., -1., 0.]}
LABELS = {"forward": "向前", "backward": "向后", "left": "向左", "right": "向右",
          "free": "自由滚动", "active": "主动抗扰", "base": "底盘", "chest": "胸部"}


from centaur_control.models.orientation import roll_pitch


def pulse(t, duration, peak):
    return peak*np.sin(np.pi*t/duration) if 0 <= t < duration else 0.


class Experiment:
    """两种模式共用质量、接触、力矩上限和关节 PD，仅改变轮/姿态反馈。"""

    def __init__(self, mode):
        if mode not in ("free", "active"):
            raise ValueError("模式只能为 free 或 active")
        self.mode = mode
        self.model = m = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
        self.dt = float(m.opt.timestep)
        self.data = nominal_state(m)
        self.data.qpos[2] -= 2e-5
        self.joints = m.actuator_trnid[:, 0].astype(int)
        self.qa = m.jnt_qposadr[self.joints]
        self.va = m.jnt_dofadr[self.joints]
        names = [m.joint(int(j)).name for j in self.joints]
        self.wheels = np.array([i for i, n in enumerate(names) if n.endswith("_foot")])
        self.hips = np.array([i for i, n in enumerate(names) if n.endswith("_hip")])
        self.thighs = np.array([i for i, n in enumerate(names) if n.endswith("_thigh")])
        self.nominal = self.data.qpos[self.qa].copy()
        self.floor = m.geom("floor").id
        self.wheel_geoms = {m.geom(f"{leg}_foot_wheel").id for leg in ("FL", "FR", "RL", "RR")}
        self.base = m.body("base_link").id
        self.points = {"base": self.base, "chest": m.body("waist_yaw_link").id}
        # 只在本次实验模型内替换轮位置伺服，源 MJCF 与历史双臂基线不改动。
        m.actuator_gainprm[self.wheels] = 0
        m.actuator_biasprm[self.wheels] = 0
        if mode == "active":
            m.actuator_gainprm[self.wheels, 0] = 1.
        m.actuator_ctrlrange[self.wheels] = [-15., 15.]
        self.data.ctrl[:] = self.nominal
        self.reference_x = 0.
        for _ in range(round(2./self.dt)):
            self.control(self.data)
            mujoco.mj_step(m, self.data)
        mujoco.mj_forward(m, self.data)
        self.reference_x = float(self.data.xpos[self.base, 0])
        self.seed = mujoco.MjData(m)
        mujoco.mj_copyData(self.seed, m, self.data)

    def control(self, data):
        ctrl = self.nominal.copy()
        if self.mode == "active":
            roll, pitch = roll_pitch(data)
            ctrl[self.hips] += np.clip(.5*roll + .04*data.qvel[3], -.15, .15)
            ctrl[self.thighs] -= np.clip(.5*pitch + .04*data.qvel[4], -.15, .15)
            # 基础抗扰反馈：前后位置/速度、俯仰与轮速阻尼；没有横向全向驱动。
            rotation = data.xmat[self.base].reshape(3, 3)
            vx = float(rotation[:, 0] @ data.qvel[:3])
            dx = float(data.xpos[self.base, 0] - self.reference_x)
            torque = 12.*pitch + 2.*data.qvel[4] - 6.*vx - 3.*dx
            ctrl[self.wheels] = np.clip(torque - .8*data.qvel[self.va[self.wheels]], -15., 15.)
        else:
            ctrl[self.wheels] = 0.
        data.ctrl[:] = np.clip(ctrl, self.model.actuator_ctrlrange[:, 0], self.model.actuator_ctrlrange[:, 1])

    def contact_state(self, data):
        wheels = set()
        other = set()
        for contact in data.contact:
            if self.floor in (contact.geom1, contact.geom2):
                gid = int(contact.geom2 if contact.geom1 == self.floor else contact.geom1)
                if gid in self.wheel_geoms:
                    wheels.add(gid)
                elif contact.dist <= 0:
                    other.add(self.model.body(int(self.model.geom_bodyid[gid])).name)
        return len(wheels), sorted(other)

    def wheel_mask(self, data):
        touching = set()
        for contact in data.contact:
            if self.floor in (contact.geom1, contact.geom2):
                touching.add(int(contact.geom2 if contact.geom1 == self.floor else contact.geom1))
        return [self.model.geom(f"{leg}_foot_wheel").id in touching for leg in ("FL", "FR", "RL", "RR")]

    def run(self, point, direction, duration, peak, record=False, observation=5.):
        if point not in self.points or direction not in DIRECTIONS or duration <= 0 or peak < 0 or observation < 0:
            raise ValueError("冲击参数不合法")
        m, d = self.model, self.data
        mujoco.mj_copyData(d, m, self.seed)
        axis = np.array(DIRECTIONS[direction])
        body = self.points[point]
        initial = d.qpos[:3].copy()
        count, other = self.contact_state(d)
        height = float(d.xipos[body, 2])
        result = {"mode": self.mode, "point": point, "body": m.body(body).name,
                  "direction": direction, "duration_s": duration, "peak_force_N": peak,
                  "observation_after_pulse_s": observation, "initial_force_height_m": height,
                  "analytic_impulse_Ns": 2.*peak*duration/np.pi, "numerical_impulse_Ns": 0.,
                  "max_abs_roll_pitch_deg": [0., 0.], "peak_ground_moment_xyz_Nm": [0., 0., 0.],
                  "peak_com_moment_xyz_Nm": [0., 0., 0.], "angular_impulse_ground_xyz_Nms": [0., 0., 0.],
                  "min_wheels": count, "first_lift": None, "first_ground_contact": None,
                  "fall": False, "fall_event": None, "numerical_failure": False,
                  "maximum_xy_displacement_m": 0., "torque_saturated_steps": 0}
        traces = {key: [] for key in ("t", "qpos", "qvel", "force", "point", "roll_pitch_deg", "wheels", "torque", "wheel_mask", "force_t")}
        timer = 0.
        tail = []
        impulse_m = np.zeros(3)
        for step in range(round((duration+observation)/self.dt)):
            t = step*self.dt
            self.control(d)
            force = axis*pulse(t, duration, peak)
            position = d.xipos[body].copy()
            d.qfrc_applied[:] = 0
            mujoco.mj_applyFT(m, d, force, np.zeros(3), position, body, d.qfrc_applied)
            ground_reference = np.r_[d.xpos[self.base, :2], 0.]
            moment = np.cross(position-ground_reference, force)
            com_moment = np.cross(position-d.subtree_com[self.base], force)
            result["peak_ground_moment_xyz_Nm"] = np.maximum(result["peak_ground_moment_xyz_Nm"], abs(moment)).tolist()
            result["peak_com_moment_xyz_Nm"] = np.maximum(result["peak_com_moment_xyz_Nm"], abs(com_moment)).tolist()
            result["numerical_impulse_Ns"] += float(np.linalg.norm(force))*self.dt
            impulse_m += moment*self.dt
            mujoco.mj_step(m, d)
            mujoco.mj_forward(m, d)
            angles = np.degrees(roll_pitch(d))
            count, other = self.contact_state(d)
            result["min_wheels"] = min(result["min_wheels"], count)
            result["max_abs_roll_pitch_deg"] = np.maximum(result["max_abs_roll_pitch_deg"], abs(angles)).tolist()
            drift = float(np.linalg.norm(d.qpos[:2]-initial[:2]))
            result["maximum_xy_displacement_m"] = max(result["maximum_xy_displacement_m"], drift)
            event = {"time_s": t+self.dt, "roll_pitch_deg": angles.tolist(), "wheels": count, "other_floor_bodies": other}
            if count < 4 and result["first_lift"] is None:
                event["wheel_mask_FL_FR_RL_RR"] = self.wheel_mask(d)
                result["first_lift"] = event
            if other and result["first_ground_contact"] is None:
                result["first_ground_contact"] = event
            # 接触丧失不等于倾覆；用持续异常着地或大倾角标记本实验的倒地失效。
            timer = timer+self.dt if other or abs(angles).max() > 60. else 0.
            if timer >= .10 and not result["fall"]:
                result["fall"], result["fall_event"] = True, event
            if t >= duration+observation-1.:
                tail.append([float(np.linalg.norm(d.qvel[:3])), float(abs(angles).max()), count])
            limited = m.actuator_forcelimited.astype(bool)
            if np.any(limited & ((d.actuator_force <= m.actuator_forcerange[:, 0]+.01) |
                                  (d.actuator_force >= m.actuator_forcerange[:, 1]-.01))):
                result["torque_saturated_steps"] += 1
            if record and step % 5 == 0:
                for key, value in zip(traces, (t+self.dt, d.qpos.copy(), d.qvel.copy(), force, position, angles, count, d.actuator_force.copy(), self.wheel_mask(d), t)):
                    traces[key].append(value)
            if not np.isfinite(d.qpos).all() or np.any(d.warning.number):
                result["numerical_failure"] = True
                break
        result["angular_impulse_ground_xyz_Nms"] = impulse_m.tolist()
        result["final_roll_pitch_deg"] = np.degrees(roll_pitch(d)).tolist()
        result["final_velocity_m_s"] = d.qvel[:3].tolist()
        result["final_base_position_m"] = d.qpos[:3].tolist()
        result["final_other_floor_bodies"] = self.contact_state(d)[1]
        result["recovered"] = bool(not result["fall"] and not result["numerical_failure"] and tail and
                                   max(x[0] for x in tail) < .10 and max(x[1] for x in tail) < 10 and min(x[2] for x in tail) == 4)
        result["status"] = "数值失败" if result["numerical_failure"] else "倒地失效" if result["fall"] else "恢复" if result["recovered"] else "观察期末未恢复"
        return result, {key: np.array(value) for key, value in traces.items()}


def save_trace(experiment, point, direction, duration, force, name):
    result, trace = experiment.run(point, direction, duration, force, record=True)
    np.savez_compressed(OUT / f"{name}.npz", **trace)
    (OUT / f"{name}.json").write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    return result


def sweep(max_force=3200., refinements=5):
    OUT.mkdir(parents=True, exist_ok=True)
    summary = {"model_sha256": hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest(),
               "mujoco_version": mujoco.__version__, "max_force_N": max_force,
               "definition": "倒地失效=非轮部件触地或倾角超过60度持续0.10秒；观察至脉冲结束后5秒；仅局部首次观测区间，不假定全局单调",
               "modes": {}, "groups": [], "trials": []}
    started = time.perf_counter()
    for mode in ("free", "active"):
        exp = Experiment(mode)
        baseline = save_trace(exp, "base", "forward", .1, 0., f"baseline_{mode}")
        summary["modes"][mode] = baseline
        print(f"{LABELS[mode]}无冲击基线：{baseline['status']}，位移 {baseline['maximum_xy_displacement_m']:.3f} m", flush=True)
        if baseline["fall"] or baseline["numerical_failure"]:
            print("该模式无冲击已失效，不报告有效冲击阈值。", flush=True)
            continue
        for point in ("base", "chest"):
            for direction in DIRECTIONS:
                for duration in (.05, .10, .20):
                    trial_results = []
                    lower, upper = 0., None
                    levels = [float(x) for x in (25, 50, 100, 200, 400, 800, 1600, 3200) if x < max_force]+[max_force]
                    for force in levels:
                        result, _ = exp.run(point, direction, duration, force)
                        trial_results.append(result)
                        if result["numerical_failure"]:
                            break
                        if result["fall"]:
                            upper = force
                            break
                        lower = force
                    if upper is not None:
                        for _ in range(refinements):
                            force = .5*(lower+upper)
                            result, _ = exp.run(point, direction, duration, force)
                            trial_results.append(result)
                            if result["numerical_failure"]:
                                break
                            if result["fall"]:
                                upper = force
                            else:
                                lower = force
                    name = f"{mode}_{point}_{direction}_{int(duration*1000)}ms"
                    passing = save_trace(exp, point, direction, duration, lower, name+"_lower")
                    failing = save_trace(exp, point, direction, duration, upper, name+"_upper") if upper else None
                    group = {"id": name, "mode": mode, "point": point, "direction": direction, "duration_s": duration,
                             "lower_no_fall_N": lower, "upper_fall_N": upper, "lower": passing, "upper": failing,
                             "valid_bracket": bool(upper is not None and not passing["fall"] and failing["fall"] and not passing["numerical_failure"] and not failing["numerical_failure"])}
                    summary["groups"].append(group)
                    summary["trials"].extend(trial_results)
                    (OUT / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
                    print(f"{len(summary['groups'])}/48 {name}：未倒地 {lower:.2f} N / 倒地 {upper} N；{time.perf_counter()-started:.0f} s", flush=True)
    report(summary)
    return summary


def report(summary):
    rows = ["# 平地短时冲击：自由滚动与主动抗扰", "", "## 条件与判据", "",
            "用户确认：空载、原站立/双臂姿态、推底盘或胸部、前后左右半正弦脉冲，持续50/100/200 ms；两种模式均允许轮轴转动。",
            "作用点分别为 base_link 和 waist_yaw_link 刚体的质心，随刚体运动；胸部不是整个上部子树的合成质心。力方向固定在世界系；施力高度随时间变化。",
            "先静置2秒，再施加脉冲并观察5秒。自由模式轮电机输出零，模型原轮轴阻尼/摩擦保留，腿/上部保持PD。主动模式轮轴为±15 N·m力矩控制，前后位置/速度与俯仰反馈，腿部roll/pitch反馈；这是当前基础控制器比较，不代表最优抗扰能力。",
            "倒地失效：非轮部件触地或|roll/pitch|>60°持续0.10秒；轻微抬轮单独记录。恢复：最后1秒四轮接地、速度<0.10m/s且倾角<10°。这些是明确的仿真判据，不是实物安全标准。",
            "扫描先按25、50、100…N倍增寻找首次倒地，再局部二分；非线性碰撞不保证力幅与倒地单调，表中只是已测试的局部区间，不声称全局唯一临界值。",
            "矩形表中的力矩为冲击期间 |F×力臂| 的轴分量峰值，参考点为随底盘移动的地面投影；另存绕整机质心的力矩和地面参考角冲量。允许滚动时支撑边变化，因此不能把此峰值解释为固定边的静态抗倾覆力矩。", "", "## 无冲击基线", ""]
    for mode, b in summary["modes"].items():
        rows.append(f"- {LABELS[mode]}：{b['status']}，最大水平位移 {b['maximum_xy_displacement_m']:.4f} m，最大roll/pitch {np.round(b['max_abs_roll_pitch_deg'],3).tolist()}°。")
    rows += ["", "## 逐工况倒地边界", "", "| 模式 | 点 | 方向 | ms | 未倒地/倒地峰值力 N | 倒地侧冲量 N·s | 峰值地面轴力矩 N·m | 倒地首次判定roll/pitch ° |", "|---|---|---|---:|---:|---:|---:|---|"]
    with (OUT / "boundaries.csv").open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.writer(file)
        writer.writerow(["模式", "冲击点", "方向", "持续秒", "未倒地峰值N", "倒地峰值N", "初始施力高度m", "峰值地面力矩Nm", "冲量Ns", "roll度", "pitch度", "有效局部区间"])
        for g in summary["groups"]:
            r = g["upper"] or g["lower"]
            moment = max(r["peak_ground_moment_xyz_Nm"][:2])
            angle = r["fall_event"]["roll_pitch_deg"] if r["fall_event"] else [None, None]
            upper = f"{g['upper_fall_N']:.2f}" if g["upper_fall_N"] else "未找到"
            rows.append(f"| {LABELS[g['mode']]} | {LABELS[g['point']]} | {LABELS[g['direction']]} | {int(g['duration_s']*1000)} | {g['lower_no_fall_N']:.2f} / {upper} | {r['analytic_impulse_Ns']:.2f} | {moment:.2f} | {angle} |")
            writer.writerow([g["mode"],g["point"],g["direction"],g["duration_s"],g["lower_no_fall_N"],g["upper_fall_N"],r["initial_force_height_m"],moment,r["analytic_impulse_Ns"],*angle,g["valid_bracket"]])
    rows += ["", "## 家庭场景的选择", "", "比较需要同时看是否倒地、恢复时间、滚动位移和碰撞风险；较高冲击力阈值不自动代表更适合室内。此实验是无限平地，未加入家具、儿童或墙体，不能据此证明家庭安全。自由滚动若持续漂移，抓取时不能直接沿用；主动抗扰若轮胎侧滑或控制饱和，也需要改进控制与碰撞检测。", "",
             "每个工况下/上边界均有NPZ回放，保存时间、广义状态、实际施力、施力点、姿态、轮接触数与执行器力。JSON保存所有试验和判定事件。模型原先排除的装配碰撞仍然排除；质量与接触参数未实物校准。", "",
             "可视化：`python centaur_control/view_robot_analysis.py --mode impact --case active_chest_forward_100ms_upper`。该模式回放实际积分的受力轨迹；窗口显示英文短标签，详细中文解释在终端与报告中。", "",
             f"MuJoCo {summary['mujoco_version']}；模型SHA256 `{summary['model_sha256']}`。"]
    (OUT / "impact_report.md").write_text("\n".join(rows)+"\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="两模式半正弦碰撞冲击对照")
    parser.add_argument("--max-force", type=float, default=3200, help="数值搜索峰值力上限，不代表家庭常见碰撞力")
    parser.add_argument("--refinements", type=int, default=5, help="局部区间二分次数")
    parser.add_argument("--baseline-only", action="store_true", help="只检查两模式无冲击状态")
    args = parser.parse_args()
    if args.max_force <= 0 or args.refinements < 0:
        parser.error("扫描参数必须非负且力上限为正")
    if args.baseline_only:
        for mode in ("free", "active"):
            exp = Experiment(mode)
            print(json.dumps(exp.run("base", "forward", .1, 0)[0], ensure_ascii=False, indent=2))
    else:
        sweep(args.max_force, args.refinements)


if __name__ == "__main__":
    main()
