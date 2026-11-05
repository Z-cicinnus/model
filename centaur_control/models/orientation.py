"""从整机浮动基状态读取横滚和俯仰角。"""

import numpy as np


def roll_pitch(data):
    w, x, y, z = data.qpos[3:7]
    return np.array([np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                     np.arcsin(np.clip(2*(w*y-z*x), -1., 1.))])
