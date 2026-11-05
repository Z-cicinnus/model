"""带关节限位的双手任务速度二次规划。"""

import mujoco
import numpy as np
from centaur_control.models.definitions import HAND_SITES, actuator_dofs, object_id
from centaur_control.kinematics.jacobians import site_linear_jacobian, wheel_contact_jacobian


def solve(model, data, hand_targets, active_actuators=None, max_velocity=1.2):
    """求解双手跟踪与轮地速度约束的有界最小二乘 QP。"""
    dofs = actuator_dofs(model)
    passive = np.ones(model.nv, dtype=bool)
    passive[dofs] = False
    free_dofs = np.flatnonzero(passive)
    blocks, targets = [], []

    for site_name, goal in zip(HAND_SITES, hand_targets):
        site_id = object_id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        jacobian = site_linear_jacobian(model, data, site_id)
        weight = 5.0
        blocks.append(weight * jacobian[:, dofs])
        desired = 2.0 * (goal - data.site_xpos[site_id])
        targets.append(weight * (desired - jacobian[:, free_dofs] @ data.qvel[free_dofs]))

    contact = wheel_contact_jacobian(model, data)
    blocks.append(8.0 * contact[:, dofs])
    targets.append(-8.0 * contact[:, free_dofs] @ data.qvel[free_dofs])

    wheel_indices = [index for index, actuator_id in enumerate(range(model.nu))
                     if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id) or "").endswith("_foot_pd")]
    if wheel_indices:
        brake = np.zeros((len(wheel_indices), len(dofs)))
        brake[np.arange(len(wheel_indices)), wheel_indices] = 12.0
        blocks.append(brake)
        targets.append(np.zeros(len(wheel_indices)))

    blocks.append(2.0 * np.eye(len(dofs)))
    targets.append(np.zeros(len(dofs)))
    matrix = np.vstack(blocks)
    target = np.concatenate(targets)

    lower, upper = np.full(len(dofs), -max_velocity), np.full(len(dofs), max_velocity)
    dt = model.opt.timestep
    for index, actuator_id in enumerate(range(model.nu)):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id) or ""
        if active_actuators is not None and actuator_name not in active_actuators:
            lower[index] = 0.0
            upper[index] = 0.0
            continue
        if model.jnt_limited[joint_id]:
            q = data.qpos[model.jnt_qposadr[joint_id]]
            lower[index] = max(lower[index], (model.jnt_range[joint_id, 0] - q) / dt)
            upper[index] = min(upper[index], (model.jnt_range[joint_id, 1] - q) / dt)

    hessian = matrix.T @ matrix + 1e-5 * np.eye(len(dofs))
    gradient = matrix.T @ target
    lipschitz = float(np.linalg.eigvalsh(hessian)[-1])
    if not np.isfinite(lipschitz) or lipschitz <= 0:
        raise RuntimeError("全身 IK-QP 的 Hessian 无法求解")
    solution = np.clip(np.zeros(len(dofs)), lower, upper)
    extrapolated = solution.copy()
    momentum = 1.0
    for _ in range(50):
        updated = np.clip(extrapolated + (gradient - hessian @ extrapolated) / lipschitz, lower, upper)
        next_momentum = 0.5 * (1.0 + np.sqrt(1.0 + 4.0 * momentum * momentum))
        extrapolated = updated + ((momentum - 1.0) / next_momentum) * (updated - solution)
        solution, momentum = updated, next_momentum
    if not np.all(np.isfinite(solution)):
        raise RuntimeError("全身 IK-QP 输出了非有限关节速度")
    return solution
