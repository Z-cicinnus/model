"""整机质量矩阵、质心惯量、安装力矩和臂腿耦合。"""

import mujoco
import numpy as np


def mass_properties(model, data, body_ids=None):
    """按质量加权求质心，以平行轴定理求世界系质心转动惯量。"""
    ids = np.arange(1, model.nbody) if body_ids is None else np.asarray(body_ids, dtype=int)
    mass = model.body_mass[ids]
    total = float(mass.sum())
    com = np.sum(mass[:, None]*data.xipos[ids], axis=0)/total
    inertia = np.zeros((3, 3))
    for bid, value in zip(ids, mass):
        rotation = data.ximat[bid].reshape(3, 3)
        r = data.xipos[bid]-com
        inertia += rotation @ np.diag(model.body_inertia[bid]) @ rotation.T
        inertia += value*((r@r)*np.eye(3)-np.outer(r, r))
    return total, com, inertia


def upper_body_ids(model):
    pelvis = model.body("pelvis").id
    ids = []
    for bid in range(1, model.nbody):
        ancestor = bid
        while ancestor and ancestor != pelvis:
            ancestor = int(model.body_parentid[ancestor])
        if ancestor == pelvis:
            ids.append(bid)
    return ids


def dynamics(model, data):
    """导出 M、偏置力、刚体质心惯量与消去浮动基后的双臂/腿部耦合。"""
    mujoco.mj_forward(model, data)
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, M, data.qM)
    arm, leg = [], []
    for j in range(1, model.njnt):
        name = model.joint(j).name
        if name.startswith(("left_", "right_")):
            arm.append(int(model.jnt_dofadr[j]))
        if name.startswith(("FL_", "FR_", "RL_", "RR_")):
            leg.append(int(model.jnt_dofadr[j]))
    coupling = M[np.ix_(arm, leg)] - M[np.ix_(arm, range(6))] @ np.linalg.solve(M[:6, :6], M[np.ix_(range(6), leg)])
    mass, com, inertia = mass_properties(model, data)
    upper = upper_body_ids(model)
    upper_mass, upper_com, _ = mass_properties(model, data, upper)
    mount = data.xpos[model.body("pelvis").id]
    force = upper_mass*model.opt.gravity
    moment = np.cross(upper_com-mount, force)
    return {"M": M, "bias": data.qfrc_bias.copy(), "com": com, "inertia_com_world": inertia,
            "coupling": coupling, "mass": mass, "upper_mass": upper_mass,
            "mount_gravity_force_world": force, "mount_gravity_moment_world": moment}
