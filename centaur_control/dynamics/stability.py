"""支撑多边形和锁定刚体的静态倾覆对照。"""

import numpy as np
from centaur_control.models.definitions import WHEEL_SITES

DIRECTIONS = {
    "roll_positive": ("向右侧倾（roll 正）", np.array([0., -1.]), 0, 1),
    "roll_negative": ("向左侧倾（roll 负）", np.array([0., 1.]), 0, -1),
    "pitch_positive": ("向前倾（pitch 正）", np.array([1., 0.]), 1, 1),
    "pitch_negative": ("向后倾（pitch 负）", np.array([-1., 0.]), 1, -1),
}


def support_points(model, data, tire_width=False):
    """仅用于四轮轴水平且同一地面的站立构型，返回名义支撑点。"""
    points = []
    for name in WHEEL_SITES:
        sid = model.site(name).id
        geom = model.geom(name.replace("_site", "_wheel"))
        axle = data.xmat[model.site_bodyid[sid]].reshape(3, 3)[:, 1]
        if abs(axle[2]) > 1e-7:
            raise ValueError("静态支撑分析要求轮轴水平；倾斜轮胎需重新求接触边界")
        center = data.site_xpos[sid].copy()
        center[2] -= geom.size[0]
        if tire_width:
            points.extend((center-axle*geom.size[1], center+axle*geom.size[1]))
        else:
            points.append(center)
    points = np.array(points)
    if np.ptp(points[:, 2]) > 1e-6:
        raise ValueError("四轮必须位于同一支撑平面")
    return points


def convex_hull(points):
    """二维单调链凸包，顶点逆时针排列。"""
    points = sorted(set(map(tuple, np.asarray(points)[:, :2])))
    def cross(o, a, b):
        return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    halves = []
    for order in (points, points[::-1]):
        half = []
        for point in order:
            while len(half) >= 2 and cross(half[-2], half[-1], point) <= 0:
                half.pop()
            half.append(point)
        halves.extend(half[:-1])
    if len(halves) < 3:
        raise ValueError("支撑点不能构成面积非零的支撑多边形")
    return np.array(halves)


def stability(mass, com, points, gravity=9.81):
    """计算四方向额外纯力偶上限与刚性整体倾斜临界角，不含动载荷。"""
    polygon = convex_hull(points)
    edges = np.roll(polygon, -1, axis=0)-polygon
    inward = np.column_stack((-edges[:, 1], edges[:, 0]))
    inward /= np.linalg.norm(inward, axis=1)[:, None]
    margins = np.sum(inward*(np.asarray(com)[:2]-polygon), axis=1)
    height = float(com[2]-np.mean(points[:, 2]))
    if height <= 0 or mass <= 0:
        raise ValueError("质量与质心离支撑平面的高度必须为正")
    directions = {}
    for key, (label, direction, axis, sign) in DIRECTIONS.items():
        projection = inward @ direction
        candidates = np.where(projection < -1e-9)[0]
        index = int(candidates[np.argmin(margins[candidates]/(-projection[candidates]))])
        distance = float(margins[index]/(-projection[index]))
        angle = float(np.degrees(np.arctan2(distance, height)))
        directions[key] = {"label": label, "margin_m": distance,
                           "additional_moment_Nm": mass*gravity*distance,
                           "critical_angle_deg": sign*angle,
                           "equivalent_horizontal_force_at_com_N": mass*gravity*distance/height,
                           "equivalent_com_acceleration_m_s2": gravity*distance/height,
                           "edge_xy": [polygon[index].tolist(), polygon[(index+1) % len(polygon)].tolist()]}
    return {"mass_kg": float(mass), "com_world_m": np.asarray(com).tolist(),
            "com_height_m": height, "polygon_xy_m": polygon.tolist(),
            "minimum_margin_m": float(margins.min()), "initially_inside": bool(margins.min() >= 0),
            "directions": directions}
