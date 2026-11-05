"""整机运动学、动力学与固定支撑工作空间分析，输出可复现的中文报告。"""

import argparse
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from centaur_control.paths import MODEL_PATH
from centaur_control.models.definitions import HAND_SITES
from centaur_control.models.definitions import WHEEL_SITES
from centaur_control.kinematics.jacobians import wheel_contact_jacobian
from centaur_control.dynamics.rigid_body import dynamics
from centaur_control.kinematics.forward import forward_kinematics
from centaur_control.kinematics.inverse import inverse_kinematics
from centaur_control.kinematics.jacobians import jac_site
from centaur_control.dynamics.stability import stability
from centaur_control.workspace.grounded import GroundedWorkspace
from centaur_control.workspace.grounded import rotation

from centaur_control.paths import analysis_output
from centaur_control.tasks.provenance import manifest


def analyze(samples=20000, seed=20261104, height_samples=5, output_dir=None):
    if samples < 100:
        raise ValueError("至少采样100次")
    if height_samples != 5:
        raise ValueError("本项验收固定为五个高度样本")
    OUTPUT = analysis_output(output_dir, formal=samples >= 20000)
    parameters = dict(samples=samples, tilted_samples=samples, seed=seed, height_samples=height_samples)
    manifest(OUTPUT, "analysis", parameters, "running")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    workspace = GroundedWorkspace(model)
    data = workspace.data
    nominal = workspace.nominal
    props = dynamics(model, data)
    static = stability(props["mass"], props["com"], workspace.points)
    fk = forward_kinematics(model, nominal)
    goals = {s: fk[s]["world"].copy() for s in HAND_SITES}
    for goal in goals.values():
        goal[:3, 3] += [.02, 0., .015]
    base_goal = np.eye(4)
    base_goal[:3, 3] = nominal[:3]
    base_goal[:3, :3] = rotation(pitch=np.radians(2))
    ik = inverse_kinematics(model, nominal, goals, base_goal)
    print(f"双手6D逆解：{ik['success']}，位置误差 {ik['position_error_m']:.6g} m", flush=True)
    hand_jac = np.array([jac_site(model, data, s) for s in HAND_SITES])
    contact_jac = wheel_contact_jacobian(model, data)
    axes, frames = [], []
    for axis in ("x", "y", "z", "roll", "pitch"):
        for sign in (-1, 1):
            r = workspace.scan_axis(axis, sign)
            frames.extend(r.pop("frames"))
            axes.append(r)
            print(f"路径 {axis} {sign:+d}：{r['feasible']:.5f} {r['unit']}", flush=True)
    height_bounds = [r for r in axes if r["axis"] == "z"]
    min_height = nominal[2]+min(r["feasible"] for r in height_bounds)
    max_height = nominal[2]+max(r["feasible"] for r in height_bounds)
    height_results = []
    height_frames = []
    for index, height in enumerate(np.linspace(min_height,max_height,height_samples)):
        position = nominal[:3].copy()
        position[2] = height
        q = workspace.geometry(position,np.eye(3))
        base = workspace.evaluate(q) if q is not None else None
        if base is None:
            raise RuntimeError(f"五个高度中的第{index+1}个无法通过重力/力矩筛选")
        extremes = []
        row = {"sample":index+1,"base_link_height_m":float(height),"body_roll_deg":0.,"body_pitch_deg":0.,
               "leg_joints_rad":q[workspace.address].tolist(),"limits":{}}
        for axis, direction in (("pitch",1),("pitch",-1),("roll",1),("roll",-1)):
            result = workspace.scan_axis(axis,direction,q)
            row["limits"][f"{axis}_{'positive' if direction>0 else 'negative'}"] = {
                "last_feasible_deg":result["feasible"],"first_rejected_deg":result["first_rejected"]}
            extremes.append(result["frames"][-1])
        height_frames.append(np.array([q.copy(),*extremes]))
        height_results.append(row)
        print(f"高度样本 {index+1}/{height_samples}：{height:.4f} m，前后/左右倾边界已计算",flush=True)
    rows, legs, stats = workspace.sample(samples, seed)
    tilted_rows, tilted_legs, tilted_stats = workspace.sample(samples, seed+1, tilted=True)
    level_count = len(rows)
    rows.extend(tilted_rows[1:])
    tips = np.array([r["hands"] for r in rows])
    qpos = np.array([r["qpos"] for r in rows])
    legq = np.array([r["qpos"] for r in legs])
    local = tips-nominal[:3]
    hands = [{"hand": name, "aabb_initial_base_m": [local[:, i].min(axis=0).tolist(), local[:, i].max(axis=0).tolist()],
              "occupied_25mm_voxels": int(len(np.unique(np.floor(local[:, i]/.025), axis=0)))} for i, name in enumerate(("左手", "右手"))]
    result = {"model_sha256": hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest(), "mujoco_version": mujoco.__version__,
              "seed": seed, "sampling": stats, "tilted_sampling": tilted_stats, "hands": hands, "axis_paths": axes,
              "height_tilt_analysis":height_results,
              "level_body_sampled_world_aabb_m": [legq[:, :3].min(axis=0).tolist(), legq[:, :3].max(axis=0).tolist()],
              "static_locked_comparator": static, "mass_kg": props["mass"], "com_world_m": props["com"].tolist(),
              "mass_matrix_shape": list(props["M"].shape), "mass_matrix_min_eigenvalue": float(np.linalg.eigvalsh(props["M"]).min()),
              "arm_leg_schur_norm": float(np.linalg.norm(props["coupling"])),
              "upper_gravity_moment_at_mount_world_Nm": props["mount_gravity_moment_world"].tolist(),
              "ik": {k:v for k,v in ik.items() if k not in ("qpos", "trajectory")},
              "fk": {s:{k:v.tolist() for k,v in transforms.items()} for s,transforms in fk.items()}}
    np.savez_compressed(OUTPUT/"analysis_data.npz", nominal_qpos=nominal, support_points=workspace.points,
                        ik_trajectory=ik["trajectory"], ik_goals=np.array(list(goals.values())), posture_qpos=np.array(frames),
                        workspace_qpos=qpos, workspace_tips_world=tips, level_leg_qpos=legq,
                        workspace_level_mask=np.arange(len(rows)) < level_count,
                        tilted_initial_upper_qpos=np.array([r["qpos"] for r in tilted_legs]),
                        height_tilt_qpos=np.array(height_frames),
                        workspace_normal_forces=np.array([r["normal_forces"] for r in rows]),
                        workspace_torque=np.array([r["torque"] for r in rows]),
                        M=props["M"], bias=props["bias"], inertia_com_world=props["inertia_com_world"],
                        base_mediated_arm_leg_coupling=props["coupling"], hand_jacobians=hand_jac, wheel_contact_jacobian=contact_jac)
    (OUTPUT/"analysis_summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2),encoding="utf-8")
    lines = ["# 整机运动学、动力学与固定支撑工作空间", "",
             "用户确认：空手、初始站姿、固定初始四轮地面支撑点。机身允许平移/升降。腰部只有yaw关节，因此躯干水平的工作空间与机身倾斜路径分开计算。冲击实验另见 impacts/impact_report.md。", "",
             f"整机质量 {props['mass']:.6f} kg；初始质心世界位置 {np.round(props['com'],6).tolist()} m。",
             f"质量矩阵 {props['M'].shape}，最小特征值 {result['mass_matrix_min_eigenvalue']:.6g}；双臂/腿部经浮动基消元后的耦合矩阵范数 {result['arm_leg_schur_norm']:.6g}。",
             "NPZ保存完整质量矩阵、零速偏置力、刚体质心惯量、左右手6D雅可比、轮地接触雅可比和耦合矩阵。轮地雅可比使用实际圆柱最低点的全链速度，包含浮动基与所有腿/轮角速度；它不是只减去轮轴转速。",
             "动力学方程：M·v̇+h=τ执行器+τ被动+τ接触+τ外力。质量矩阵含原文件armature；刚体质心惯量不含转子等效项。", "",
             "## 正逆运动学", "", "正解输出双手、四轮中心相对世界与底盘的4×4位姿。逆解以全部nv速度为变量，通过局部阻尼最小二乘与KKT等式保持四轮中心和手指耦合；缩放步长维持关节限位。它不是全局规划器，也不能证明任意目标不可达。",
             f"验证任务：双手保持朝向，前移20mm、升高15mm，底盘俯仰2°。结果：{result['ik']}。窗口ik回放为几何迭代，不是闭环动态控制。", "",
             "## 四轮接地的机身路径范围", "", "以下从初始站姿沿单一轴连续扫描，其他机身坐标不变，手臂保持初始姿态。平移步长2mm、角度步长0.5°，首个不通过区间再二分10次；小数位不代表模型测量精度。",
             "| 轴 | 方向 | 最后通过值 | 首个不通过值 | 单位 |", "|---|---:|---:|---:|---|"]
    for r in axes:
        lines.append(f"| {r['axis']} | {r['sign']} | {r['feasible']:.5f} | {r['first_rejected']:.5f} | {r['unit']} |")
    lines += ["", "平移值为相对初始底盘的增量。z路径可换算底盘绝对高度；x路径为固定脚下支撑时的机身前后移动，等价于腿端相对底盘反向移动，不能解释为抬轮高度。roll/pitch是主动构型路径角，不是滚动冲击倾覆角。该局部路径不允许同时下蹲以扩大倾角，不能当作全局最大调姿幅度。", "",
              "## 腿、腰、双臂协同工作空间", "",
              f"随机种子 {seed}；联合采样 {samples} 次，通过腿几何 {stats['geometry_solutions']} 次，保留同步双手构型 {stats['accepted']} 个（含初始构型）；上部保持初始姿态的水平腿部样本 {stats['level_leg_accepted']} 个。",
              f"另以种子{seed+1}采样{samples}次倾斜机身，共通过{tilted_stats['accepted']-1}个额外双手构型。两个集合合并用于下表和点云；NPZ的workspace_level_mask区分水平/倾斜构型。倾斜时上部初始姿态的样本另存tilted_initial_upper_qpos。",
              "机身三维平移搜索盒由两段腿长和髋侧偏距推导；水平集合固定机身roll/pitch/yaw=0；倾斜集合在保持机身向上的roll/pitch各±90°范围随机搜索，yaw仍为0，这只是搜索外包范围。腰yaw、双臂各关节在源文件限位内采样。四轮实际接触点固定，有限宽圆柱在水平轮轴时允许接触位于胎面线段内，倾斜时使用最低边缘点。",
              "筛选：关节范围、当前启用碰撞规则、四个非负竖直接触力的重力平衡，以及源文件执行器/关节力矩限幅。手指耦合的重力通过约束传递。该静力筛选不用水平地面力，是可行域的保守子集；未验证PD跟踪或动态过渡，也不覆盖被排除的装配碰撞。",
              "| 手 | 初始底盘坐标xyz下界m | 上界m | 25mm占用体素 |", "|---|---|---|---:|"]
    for r in hands:
        a,b=r["aabb_initial_base_m"]
        lines.append(f"| {r['hand']} | {np.round(a,4).tolist()} | {np.round(b,4).tolist()} | {r['occupied_25mm_voxels']} |")
    lines += ["", "## 五种腿高下的重力倾斜边界", "",
              f"按可行底盘高度范围等间距取{height_samples}点，最低/最高端点包含在内。每个高度固定初始四轮地面支撑位置，腰/双臂保持零位，只调四条腿；逐一增加单轴俯仰或横滚，静力平衡需存在四个非负轮法向力，且所有电机关节力矩不超源模型上限。首个失败区间用二分细化；仅报告单轴角度，前后与左右组合可能更早失效。",
              "| 样本 | base_link高度m | 前倾pitch+° | 后倾pitch−° | roll+° | roll−° |", "|---:|---:|---:|---:|---:|---:|"]
    for h in height_results:
        l=h["limits"]
        lines.append(f"| {h['sample']} | {h['base_link_height_m']:.5f} | {l['pitch_positive']['last_feasible_deg']:.3f} | {l['pitch_negative']['last_feasible_deg']:.3f} | {l['roll_positive']['last_feasible_deg']:.3f} | {l['roll_negative']['last_feasible_deg']:.3f} |")
    lines += ["", "限制角度由固定接触点的圆柱胎面/边缘几何、关节范围、支撑力与重力矩、执行器力矩共同决定，不等于移动底盘的动态防倾覆角。最低/最高为初始x/y不变的升降路径端点，不是允许平移后全局最低/最高高度。端点接近零的角度表示固定高度已没有调姿余量，不表示一倾斜就发生重力倾覆。analysis_data.npz中的height_tilt_qpos包含每个高度的水平状态和四个极限状态，可通过height_sweep模式逐一检查。",
              f"上部初始姿态下，水平底盘的有限样本世界位置包围盒：{result['level_body_sampled_world_aabb_m']} m。",
              "点云是有限采样的内近似；包围盒内不保证可达，两只手的边际包络不能任意组合。每个点均保存对应完整构型、接触力和关节力矩。连续路径安全性尚未验证。", "",
              "## 锁定刚体的静态对照", "", "仅作解释：车轮与关节锁定、绕固定边缘倾斜，无滑移。M=m·g·d，θ=atan(d/h)。这与已确认的自由滚动/主动抗扰冲击不同。", "",
              "| 方向 | 附加纯力偶N·m | 刚性绕边临界角° |", "|---|---:|---:|"]
    for r in static["directions"].values():
        lines.append(f"| {r['label']} | {r['additional_moment_Nm']:.3f} | {r['critical_angle_deg']:.3f} |")
    lines += ["", "## 复现与可视化", "", "在仓库根目录执行（完整PowerShell启动方式见README）：", "", "```powershell",
              "& $python centaur_control/test_robot_analysis.py", "& $python centaur_control/analyze_robot.py",
              "& $python centaur_control/view_robot_analysis.py --mode ik",
              "& $python centaur_control/view_robot_analysis.py --mode posture",
              "& $python centaur_control/view_robot_analysis.py --mode workspace", "```", "",
              "质量/惯量沿用源文件；pelvis缺少显式惯量，约0.56446kg由网格与默认密度推算。装配碰撞排除仍存在，不能据此证明机械无干涉。", f"模型SHA256：{result['model_sha256']}；MuJoCo {mujoco.__version__}。"]
    (OUTPUT/"analysis_report.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    manifest(OUTPUT, "analysis", parameters, "complete")
    print(json.dumps(stats, ensure_ascii=False),flush=True)
    return result


def main():
    parser=argparse.ArgumentParser(description="固定四轮支撑的整机分析")
    parser.add_argument("--samples",type=int,default=20000,help="联合采样次数")
    parser.add_argument("--seed",type=int,default=20261104,help="随机种子")
    parser.add_argument("--height-samples",type=int,default=5,help="高度样本数，固定为五个")
    parser.add_argument("--output-dir", type=Path, help="自定义结果目录；默认正式分析写 formal，低采样写 checks")
    args=parser.parse_args()
    analyze(args.samples,args.seed,args.height_samples,args.output_dir)


if __name__ == "__main__":
    main()
