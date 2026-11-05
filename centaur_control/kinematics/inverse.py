"""固定轮端和手指耦合的局部六维逆解。"""

import mujoco
import numpy as np
from centaur_control.models.definitions import HAND_SITES, WHEEL_SITES
from centaur_control.kinematics.forward import forward_kinematics, pose_error
from centaur_control.kinematics.jacobians import jac_site


def inverse_kinematics(model, initial, goals, base_goal=None, max_iterations=160):
    """全 nv 变量的局部阻尼逆解，保持四轮中心和 O6 耦合，受关节限位约束。

    这是准静态几何逆解；不承诺全局可达性，也不代替碰撞规划或动力学控制。
    """
    if max_iterations < 1:
        raise ValueError("迭代次数必须为正")
    forward_kinematics(model, initial, ())
    if not goals or any(name not in HAND_SITES for name in goals):
        raise ValueError("需指定左手或右手的 4×4 世界系目标位姿")
    for target in goals.values():
        if np.shape(target) != (4, 4) or not np.isfinite(target).all():
            raise ValueError("末端目标位姿必须为有限的 4×4 矩阵")
        if not np.allclose(target[3], [0, 0, 0, 1]) or not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-6) or not np.isclose(np.linalg.det(target[:3, :3]), 1.):
            raise ValueError("末端目标必须是有效的刚体变换")
    data = mujoco.MjData(model)
    data.qpos[:] = initial
    mujoco.mj_forward(model, data)
    wheel_goals = np.array([data.site_xpos[model.site(s).id].copy() for s in WHEEL_SITES])
    trajectory = [data.qpos.copy()]
    weights = np.ones(model.nv) * 0.003
    weights[:6] = 0.03
    success = False
    for iteration in range(max_iterations):
        tasks, errors, hand_errors = [], [], []
        for name, target in goals.items():
            sid = model.site(name).id
            error = pose_error(target, data.site_xpos[sid], data.site_xmat[sid].reshape(3, 3))
            hand_errors.append(error)
            scale = np.array([1., 1., 1., .35, .35, .35])
            tasks.append(scale[:, None] * jac_site(model, data, name))
            errors.append(scale * error)
        base_error = np.zeros(6)
        if base_goal is not None:
            bid = model.body("base_link").id
            base_error = pose_error(base_goal, data.xpos[bid], data.xmat[bid].reshape(3, 3))
            jac = np.zeros((6, model.nv))
            mujoco.mj_jacBody(model, data, jac[:3], jac[3:], bid)
            tasks.append(2. * jac)
            errors.append(2. * base_error)
        constraints, correction = [], []
        for name, goal in zip(WHEEL_SITES, wheel_goals):
            constraints.append(jac_site(model, data, name)[:3])
            correction.extend(goal - data.site_xpos[model.site(name).id])
        for eq in range(model.neq):
            if model.eq_type[eq] != mujoco.mjtEq.mjEQ_JOINT:
                continue
            j1, j2 = int(model.eq_obj1id[eq]), int(model.eq_obj2id[eq])
            coefficients = model.eq_data[eq, :5]
            q2 = data.qpos[model.jnt_qposadr[j2]]
            row = np.zeros(model.nv)
            row[model.jnt_dofadr[j1]] = 1.
            row[model.jnt_dofadr[j2]] = -sum(k * coefficients[k] * q2 ** (k-1) for k in range(1, 5))
            constraints.append(row[None, :])
            correction.append(np.polynomial.polynomial.polyval(q2, coefficients) - data.qpos[model.jnt_qposadr[j1]])
        contact_error = np.max(np.abs(correction))
        position_error = max(np.linalg.norm(e[:3]) for e in hand_errors)
        rotation_error = max(np.linalg.norm(e[3:]) for e in hand_errors)
        if position_error < 2e-4 and rotation_error < 1e-3 and contact_error < 2e-5 and np.linalg.norm(base_error) < 1e-3:
            success = True
            break
        A, b = np.vstack(tasks), np.concatenate(errors)
        C, c = np.vstack(constraints), np.array(correction)
        H = A.T @ A + np.diag(weights)
        # KKT 等式保留轮端与手指耦合；步长缩放确保有限关节限位。
        kkt = np.block([[H, C.T], [C, np.zeros((len(c), len(c)))]])
        step = np.linalg.lstsq(kkt, np.r_[A.T @ b, c], rcond=1e-10)[0][:model.nv]
        factor = min(1., .12 / max(np.max(np.abs(step)), 1e-12))
        for j in range(1, model.njnt):
            if model.jnt_limited[j]:
                velocity = step[model.jnt_dofadr[j]]
                q = data.qpos[model.jnt_qposadr[j]]
                if abs(velocity) > 1e-12:
                    bound = model.jnt_range[j, 1 if velocity > 0 else 0]
                    factor = min(factor, max(0., (bound-q) / velocity))
        if factor < 1e-8:
            break
        mujoco.mj_integratePos(model, data.qpos, step, factor)
        mujoco.mj_forward(model, data)
        trajectory.append(data.qpos.copy())
    # 最后一步可能耗尽迭代次数，指标必须对应返回的状态。
    hand_errors = [pose_error(target, data.site_xpos[model.site(name).id],
                             data.site_xmat[model.site(name).id].reshape(3, 3)) for name, target in goals.items()]
    position_error = max(np.linalg.norm(e[:3]) for e in hand_errors)
    rotation_error = max(np.linalg.norm(e[3:]) for e in hand_errors)
    contact_error = max(np.max(abs(goal-data.site_xpos[model.site(name).id])) for name, goal in zip(WHEEL_SITES, wheel_goals))
    for eq in range(model.neq):
        if model.eq_type[eq] == mujoco.mjtEq.mjEQ_JOINT:
            j1, j2 = int(model.eq_obj1id[eq]), int(model.eq_obj2id[eq])
            error = data.qpos[model.jnt_qposadr[j1]] - np.polynomial.polynomial.polyval(data.qpos[model.jnt_qposadr[j2]], model.eq_data[eq, :5])
            contact_error = max(contact_error, abs(error))
    if base_goal is not None:
        bid = model.body("base_link").id
        base_error = pose_error(base_goal, data.xpos[bid], data.xmat[bid].reshape(3, 3))
    success = bool(position_error < 2e-4 and rotation_error < 1e-3 and contact_error < 2e-5 and np.linalg.norm(base_error) < 1e-3)
    return {"success": success, "iterations": iteration+1,
            "position_error_m": float(position_error), "rotation_error_rad": float(rotation_error),
            "wheel_and_coupling_error": float(contact_error), "base_pose_error": float(np.linalg.norm(base_error)),
            "qpos": data.qpos.copy(), "trajectory": np.array(trajectory)}
