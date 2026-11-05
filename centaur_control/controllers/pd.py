"""为模型中的全部位置执行器提供统一的 PD 目标接口。"""

import mujoco
import numpy as np


def current_joint_targets(model, data):
    """读取每个执行器所控关节的当前位置，作为安全保持目标。"""
    targets = np.empty(model.nu)
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        qpos_address = model.jnt_qposadr[joint_id]
        targets[actuator_id] = data.qpos[qpos_address]
    return targets


def apply_joint_targets(model, data, targets):
    """将全部执行器的位置目标限幅后写入 MuJoCo 控制向量。"""
    targets = np.asarray(targets, dtype=float)
    if targets.shape != (model.nu,) or not np.all(np.isfinite(targets)):
        raise ValueError(f"执行器目标必须是长度为 {model.nu} 的有限数值数组")
    for actuator_id in range(model.nu):
        control = targets[actuator_id]
        if model.actuator_ctrllimited[actuator_id]:
            control = np.clip(control, *model.actuator_ctrlrange[actuator_id])
        data.ctrl[actuator_id] = control


def actuator_joint_names(model):
    """返回按执行器顺序排列的受控关节名。"""
    return [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(joint_id))
            for joint_id in model.actuator_trnid[:, 0]]


def support_posture_targets(model, data, nominal_targets, roll_gain=0.5, pitch_gain=0.5):
    """依据机身横滚/俯仰误差微调四腿髋、髋俯仰 PD 目标。"""
    w, x, y, z = data.qpos[3:7]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    targets = np.asarray(nominal_targets, dtype=float).copy()
    joint_names = actuator_joint_names(model)
    for actuator_id, joint_name in enumerate(joint_names):
        if joint_name.endswith("_hip"):
            targets[actuator_id] += roll_gain * roll
        elif joint_name.endswith("_thigh"):
            targets[actuator_id] -= pitch_gain * pitch
    return targets
