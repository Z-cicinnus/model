"""名义站姿与单关节位置设置。"""

import mujoco
from centaur_control.models.definitions import WHEEL_SITES


def set_joint(model, data, name, value):
    joint = model.joint(name)
    if joint.limited[0] and not joint.range[0] <= value <= joint.range[1]:
        raise ValueError(f"关节 {name} 的目标 {value} 超出限位")
    data.qpos[joint.qposadr[0]] = value


def nominal_state(model, thigh=0.2, calf=-0.9):
    """四轮轴水平，按圆柱半径调整浮动基高度，轮端恰好接地。"""
    data = mujoco.MjData(model)
    for leg in ("FL", "FR", "RL", "RR"):
        for suffix, value in (("hip", 0.), ("thigh", thigh), ("calf", calf)):
            set_joint(model, data, f"{leg}_{suffix}", value)
    mujoco.mj_forward(model, data)
    sid = model.site(WHEEL_SITES[0]).id
    radius = model.geom("FL_foot_wheel").size[0]
    data.qpos[2] += radius - data.site_xpos[sid, 2]
    mujoco.mj_forward(model, data)
    return data
