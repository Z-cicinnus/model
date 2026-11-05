"""末端及轮地接触点的完整运动链雅可比。"""

import mujoco
import numpy as np
from centaur_control.models.definitions import WHEEL_SITES, object_id


def jac_site(model, data, name):
    jac = np.zeros((6, model.nv))
    mujoco.mj_jacSite(model, data, jac[:3], jac[3:], model.site(name).id)
    return jac


def site_linear_jacobian(model, data, site_id):
    jacobian = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacobian, None, site_id)
    return jacobian


def wheel_contact_jacobian(model, data):
    """计算圆柱最低接触点的法向、侧向和滚动速度，包含全链角速度。"""
    rows = []
    for site_name in WHEEL_SITES:
        site_id = object_id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        body_id = model.site_bodyid[site_id]
        axle = data.xmat[body_id].reshape(3, 3) @ np.array([0.0, 1.0, 0.0])
        axle /= np.linalg.norm(axle)
        normal = np.array([0.0, 0.0, 1.0])
        rolling = np.cross(axle, normal)
        rolling /= np.linalg.norm(rolling)
        geom = model.geom(site_name.replace("_site", "_wheel"))
        radial = normal - axle * axle[2]
        radial /= np.linalg.norm(radial)
        point = data.site_xpos[site_id] - geom.size[0] * radial
        if abs(axle[2]) > 1e-8:
            point -= np.sign(axle[2]) * geom.size[1] * axle
        jacobian = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jacobian, None, point, body_id)
        lateral = np.cross(normal, rolling)
        rows.extend((normal @ jacobian, lateral @ jacobian, rolling @ jacobian))
    return np.vstack(rows)
