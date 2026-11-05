"""加载统一模型、验证动力学数据，并导出首轮整机分析结果。"""

import json
from pathlib import Path

import mujoco
import numpy as np


from centaur_control.paths import PROJECT_ROOT as ROOT
MODEL_PATH = ROOT / "centaur_control" / "centaur_unified.xml"
OUT = ROOT / "centaur_control" / "model_dynamics.npz"
HAND_SITES = ("lh_hand_base_link_site", "rh_hand_base_link_site")
WHEEL_SITES = tuple(f"{leg}_foot_site" for leg in ("FL", "FR", "RL", "RR"))


def name_id(model, kind, name):
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise ValueError(f"找不到名为 {name!r} 的 {kind.name}")
    return result


def site_jacobian(model, data, name):
    sid = name_id(model, mujoco.mjtObj.mjOBJ_SITE, name)
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jp, jr, sid)
    return np.vstack((jp, jr))


from centaur_control.kinematics.jacobians import wheel_contact_jacobian


def analyze():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    pelvis_id = name_id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    base_id = name_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if model.body_parentid[pelvis_id] != base_id:
        raise RuntimeError("R1 pelvis 未直接连接到 AS2-W base_link")
    if not np.allclose(model.body_pos[pelvis_id], [-0.015, 0.0, 0.1], atol=1e-8):
        raise RuntimeError("R1 pelvis 安装位置与指定值不符")
    if not np.allclose(model.body_quat[pelvis_id], [1.0, 0.0, 0.0, 0.0], atol=1e-8):
        raise RuntimeError("R1 pelvis 安装四元数与指定值不符")
    data.qpos[2] = 0.50
    for leg in ("FL", "FR", "RL", "RR"):
        for suffix, value in (("hip", 0.0), ("thigh", 0.2), ("calf", -0.9), ("foot", 0.0)):
            jid = name_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{leg}_{suffix}")
            data.qpos[model.jnt_qposadr[jid]] = value
    mujoco.mj_forward(model, data)

    mass = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, mass, data.qM)
    hand_jacs = [site_jacobian(model, data, site) for site in HAND_SITES]
    contact_jac = wheel_contact_jacobian(model, data)
    upper_dofs, leg_dofs, base_dofs = [], [], []
    for dof, jid in enumerate(model.dof_jntid):
        joint = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(jid)) or ""
        if joint.startswith(("left_shoulder", "right_shoulder", "left_elbow", "right_elbow")):
            upper_dofs.append(dof)
        if joint.startswith(("FL_", "FR_", "RL_", "RR_")):
            leg_dofs.append(dof)
        if joint == "base_free":
            base_dofs.append(dof)
    # 不同肢体之间没有直接的关节惯性块；消去共享浮动基，
    # 以计算经基座传递的手臂与腿部等效动态耦合。
    m_uu = mass[np.ix_(upper_dofs, upper_dofs)]
    m_ll = mass[np.ix_(leg_dofs, leg_dofs)]
    m_bb = mass[np.ix_(base_dofs, base_dofs)]
    m_ub = mass[np.ix_(upper_dofs, base_dofs)]
    m_lb = mass[np.ix_(leg_dofs, base_dofs)]
    coupling = -m_ub @ np.linalg.solve(m_bb, m_lb.T)
    if not np.all(np.isfinite(mass)) or not np.all(np.linalg.eigvalsh(mass) > 0):
        raise RuntimeError("验证姿态下的质量矩阵不是有限正定矩阵")
    if contact_jac.shape != (12, model.nv) or any(jac.shape != (6, model.nv) for jac in hand_jacs):
        raise RuntimeError("手部或车轮雅可比矩阵的尺寸不符合预期")
    smoke_data = mujoco.MjData(model)
    smoke_data.qpos[:] = data.qpos
    for _ in range(10):
        mujoco.mj_step(model, smoke_data)
    if not np.all(np.isfinite(smoke_data.qpos)) or not np.all(np.isfinite(smoke_data.qvel)):
        raise RuntimeError("短时仿真自检中出现非有限状态")

    np.savez_compressed(OUT, mass_matrix=mass, left_hand_jacobian=hand_jacs[0],
                        right_hand_jacobian=hand_jacs[1], wheel_contact_jacobian=contact_jac)
    result = {
        "model_file": str(MODEL_PATH),
        "mujoco_version": mujoco.__version__,
        "model_parameters": {
            "pelvis_mount_relative_to_base_link_pos_m": [-0.015, 0.0, 0.1],
            "pelvis_mount_relative_to_base_link_quat_wxyz": [1.0, 0.0, 0.0, 0.0],
            "wheel_contact_radius_m": 0.086,
            "mass_and_inertia": "按源 URDF/MJCF 原值保留",
        },
        "dimensions": {"nbody": model.nbody, "njnt": model.njnt, "nq": model.nq, "nv": model.nv, "nu": model.nu, "ngeom": model.ngeom},
        "total_model_mass_kg": float(model.body_subtreemass[1]),
        "com_world_m": data.subtree_com[1].tolist(),
        "mass_matrix_shape": list(mass.shape),
        "mass_matrix_min_eigenvalue": float(np.linalg.eigvalsh(mass).min()),
        "mass_matrix_condition_number": float(np.linalg.cond(mass)),
        "hand_jacobian_shapes": [list(jac.shape) for jac in hand_jacs],
        "wheel_contact_jacobian_shape": list(contact_jac.shape),
        "arm_leg_direct_joint_block_frobenius": float(np.linalg.norm(mass[np.ix_(upper_dofs, leg_dofs)])),
        "arm_leg_base_mediated_schur_frobenius": float(np.linalg.norm(coupling)),
        "arm_leg_base_mediated_schur_relative": float(np.linalg.norm(coupling) / max(np.sqrt(np.linalg.norm(m_uu) * np.linalg.norm(m_ll)), 1e-12)),
        "matrix_archive": str(OUT),
        "short_step_smoke_test": "通过：执行 10 个 MuJoCo 步后 qpos/qvel 均为有限值",
        "result": "通过：安装变换、质量矩阵正定性、雅可比尺寸和 10 步仿真均符合检查要求（不代表已通过平衡测试）",
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))


def main():
    analyze()

if __name__ == "__main__":
    main()
