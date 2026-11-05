"""末端正运动学与位姿误差。"""

import mujoco
import numpy as np
from centaur_control.models.definitions import HAND_SITES, WHEEL_SITES


def forward_kinematics(model, qpos, sites=HAND_SITES + WHEEL_SITES):
    """由 nq 维广义位置返回世界系位姿和相对底盘位姿。"""
    qpos = np.asarray(qpos, dtype=float)
    if qpos.shape != (model.nq,) or not np.isfinite(qpos).all():
        raise ValueError(f"qpos 必须为长度 {model.nq} 的有限向量")
    if not np.isclose(np.linalg.norm(qpos[3:7]), 1., atol=1e-6):
        raise ValueError("浮动基四元数必须归一化")
    data = mujoco.MjData(model)
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    base = model.body("base_link").id
    rotation = data.xmat[base].reshape(3, 3)
    result = {}
    for name in sites:
        sid = model.site(name).id
        world = np.eye(4)
        world[:3, :3] = data.site_xmat[sid].reshape(3, 3)
        world[:3, 3] = data.site_xpos[sid]
        relative = np.eye(4)
        relative[:3, :3] = rotation.T @ world[:3, :3]
        relative[:3, 3] = rotation.T @ (world[:3, 3] - data.xpos[base])
        result[name] = {"world": world, "base": relative}
    return result


def pose_error(target, position, rotation):
    """位置差与世界系旋转向量，顺序为平移、转动。"""
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(target[:3, :3] @ rotation.T).ravel())
    if quat[0] < 0:
        quat *= -1
    angular = np.empty(3)
    mujoco.mju_quat2Vel(angular, quat, 1.)
    return np.r_[target[:3, 3] - position, angular]
