"""模型对象命名与执行器自由度映射。"""

import mujoco
import numpy as np

HAND_SITES = ("lh_hand_base_link_site", "rh_hand_base_link_site")
WHEEL_SITES = tuple(f"{leg}_foot_site" for leg in ("FL", "FR", "RL", "RR"))


def object_id(model, kind, name):
    object_id = mujoco.mj_name2id(model, kind, name)
    if object_id < 0:
        raise ValueError(f"找不到模型对象：{name}")
    return object_id


def actuator_dofs(model):
    return np.asarray([model.jnt_dofadr[int(joint)] for joint in model.actuator_trnid[:, 0]], dtype=int)
