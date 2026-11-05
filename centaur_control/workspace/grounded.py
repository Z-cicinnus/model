"""固定初始支撑点的圆柱轮接地逆解和有限工作空间采样。"""

import mujoco
import numpy as np

from centaur_control.models.collision import collision_pairs
from centaur_control.models.definitions import HAND_SITES, WHEEL_SITES
from centaur_control.models.state import nominal_state
from centaur_control.dynamics.stability import convex_hull

LEGS = ("FL", "FR", "RL", "RR")


def rotation(roll=0., pitch=0.):
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    return np.array([[cp, sp*sr, sp*cr], [0., cr, -sr], [-sp, cp*sr, cp*cr]])


class GroundedWorkspace:
    """沿初始屈膝分支求解，不将有限采样包络声称为全局最大工作空间。"""

    def __init__(self, model):
        self.model = m = model
        self.data = nominal_state(m)
        self.nominal = self.data.qpos.copy()
        self.sites = [m.site(s).id for s in WHEEL_SITES]
        self.origins = np.array([m.body(leg+"_hip").pos for leg in LEGS])
        self.address = np.array([[m.joint(leg+"_"+j).qposadr[0] for j in ("hip", "thigh", "calf")] for leg in LEGS])
        self.ranges = np.array([[m.joint(leg+"_"+j).range for j in ("hip", "thigh", "calf")] for leg in LEGS])
        self.l1 = np.array([-m.body(leg+"_calf").pos[2] for leg in LEGS])
        self.l2 = np.array([-m.body(leg+"_foot").pos[2] for leg in LEGS])
        self.offset = np.array([m.body(leg+"_thigh").pos[1] + m.site(leg+"_foot_site").pos[1] for leg in LEGS])
        self.radius = np.array([m.geom(leg+"_foot_wheel").size[0] for leg in LEGS])
        self.width = np.array([m.geom(leg+"_foot_wheel").size[1] for leg in LEGS])
        self.points = self.data.site_xpos[self.sites].copy()
        self.points[:, 2] -= self.radius
        self.polygon = convex_hull(self.points)
        self.actdof = m.jnt_dofadr[m.actuator_trnid[:, 0]]

    def geometry(self, position, R):
        """圆柱接触点固定；水平轮轴允许接触点位于胎面宽度内。"""
        q = self.nominal.copy()
        q[:3] = position
        mujoco.mju_mat2Quat(q[3:7], R.ravel())
        for i in range(4):
            anchor = position+R @ self.origins[i]
            target = R.T @ (self.points[i]-anchor)
            candidates = []
            horizontal = np.arctan2(-R[2, 1], R[2, 2])
            lateral = target[1]*np.cos(horizontal)+target[2]*np.sin(horizontal)-self.offset[i]
            if abs(lateral) <= self.width[i]+1e-10:
                candidates.append((horizontal, -lateral))
            amplitude = np.hypot(target[1], target[2])
            phase = np.arctan2(target[2], target[1])
            for side in (-1., 1.):
                ratio = (self.offset[i]-self.width[i]*side)/max(amplitude, 1e-12)
                if abs(ratio) <= 1:
                    for branch in (-1., 1.):
                        hip = (phase+branch*np.arccos(ratio)+np.pi) % (2*np.pi)-np.pi
                        axle = R @ np.array([0., np.cos(hip), np.sin(hip)])
                        if side*axle[2] > 1e-10:
                            candidates.append((hip, side*self.width[i]))
            found = False
            for hip, offset in sorted(candidates, key=lambda pair: abs(pair[0])):
                axle = R @ np.array([0., np.cos(hip), np.sin(hip)])
                radial = np.array([0., 0., 1.])-axle*axle[2]
                radial /= np.linalg.norm(radial)
                center = self.points[i]+self.radius[i]*radial+offset*axle
                local = R.T @ (center-anchor)
                x = local[0]
                z = -np.sin(hip)*local[1]+np.cos(hip)*local[2]
                cosine = (x*x+z*z-self.l1[i]**2-self.l2[i]**2)/(2*self.l1[i]*self.l2[i])
                if abs(cosine) > 1.+1e-10:
                    continue
                calf = -np.arccos(np.clip(cosine, -1, 1))
                thigh = np.arctan2(-x, -z)-np.arctan2(self.l2[i]*np.sin(calf), self.l1[i]+self.l2[i]*np.cos(calf))
                angles = np.array([hip, thigh, calf])
                if np.all(angles >= self.ranges[i, :, 0]-1e-9) and np.all(angles <= self.ranges[i, :, 1]+1e-9):
                    q[self.address[i]] = angles
                    found = True
                    break
            if not found:
                return None
        return q

    def evaluate(self, q):
        """碰撞、支撑多边形和仅竖直法向载荷的静力执行器筛选。"""
        m, d = self.model, self.data
        d.qpos[:] = q
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        if collision_pairs(m, d):
            return None
        com = d.subtree_com[m.body("base_link").id]
        A = np.vstack((np.ones(4), self.points[:, :2].T))
        target = m.body_mass.sum()*9.81*np.r_[1., com[:2]]
        forces = np.linalg.lstsq(A, target, rcond=None)[0]
        null = np.linalg.svd(A, full_matrices=True)[2][-1]
        lo, hi = -np.inf, np.inf
        for f, n in zip(forces, null):
            if n > 1e-10:
                lo = max(lo, -f/n)
            elif n < -1e-10:
                hi = min(hi, -f/n)
            elif f < 0:
                return None
        if lo > hi:
            return None
        J = np.zeros((4, m.nv))
        for i, sid in enumerate(self.sites):
            jac = np.zeros((3, m.nv))
            mujoco.mj_jac(m, d, jac, None, self.points[i], int(m.site_bodyid[sid]))
            J[i] = jac[2]
        required = d.qfrc_bias-J.T @ forces
        for eq in range(m.neq):
            if m.eq_type[eq] == mujoco.mjtEq.mjEQ_JOINT:
                j1, j2 = int(m.eq_obj1id[eq]), int(m.eq_obj2id[eq])
                required[m.jnt_dofadr[j2]] += m.eq_data[eq, 1]*required[m.jnt_dofadr[j1]]
        tau = required[self.actdof]
        slope = -(J.T @ null)[self.actdof]
        joints = m.actuator_trnid[:, 0]
        lower = np.where(m.actuator_forcelimited, m.actuator_forcerange[:, 0], -np.inf)
        upper = np.where(m.actuator_forcelimited, m.actuator_forcerange[:, 1], np.inf)
        lower = np.maximum(lower, np.where(m.jnt_actfrclimited[joints], m.jnt_actfrcrange[joints, 0], -np.inf))
        upper = np.minimum(upper, np.where(m.jnt_actfrclimited[joints], m.jnt_actfrcrange[joints, 1], np.inf))
        for t, s, low, high in zip(tau, slope, lower, upper):
            if abs(s) < 1e-10:
                if t < low-1e-6 or t > high+1e-6:
                    return None
            else:
                a, b = sorted(((low-t)/s, (high-t)/s))
                lo, hi = max(lo, a), min(hi, b)
        if lo > hi:
            return None
        parameter = np.clip(-tau@slope/max(slope@slope, 1e-12), lo, hi)
        forces += parameter*null
        return {"qpos": q.copy(), "com": com.copy(), "normal_forces": forces,
                "torque": tau+parameter*slope,
                "hands": d.site_xpos[[m.site(s).id for s in HAND_SITES]].copy()}

    def scan_axis(self, axis, sign, start_q=None):
        """从给定可行姿态连续扫描，得到本路径首个不通过区间。"""
        start_q = self.nominal if start_q is None else np.asarray(start_q, dtype=float)
        if start_q.shape != (self.model.nq,) or self.evaluate(start_q) is None:
            raise ValueError("扫描起始姿态必须是通过静力筛选的整机状态")
        is_angle = axis in ("roll", "pitch")
        step, cap = (np.radians(.5), np.radians(80.)) if is_angle else (.002, .7)
        def candidate(value):
            position = start_q[:3].copy()
            R = np.eye(3)
            if is_angle:
                R = rotation(**{axis: sign*value})
            else:
                position[{"x": 0, "y": 1, "z": 2}[axis]] += sign*value
            q = self.geometry(position, R)
            return self.evaluate(q) if q is not None else None
        lower, upper = 0., cap
        frames = [start_q.copy()]
        for value in np.arange(step, cap+step/2, step):
            result = candidate(value)
            if result is None:
                upper = value
                break
            lower = value
            frames.append(result["qpos"])
        for _ in range(10):
            mid = (lower+upper)/2
            result = candidate(mid)
            if result is None:
                upper = mid
            else:
                lower = mid
        end = candidate(lower)
        if end is not None:
            frames.append(end["qpos"])
        convert = np.degrees if is_angle else float
        return {"axis": axis, "sign": sign, "feasible": sign*convert(lower),
                "first_rejected": sign*convert(upper), "unit": "deg" if is_angle else "m",
                "base_height_m": float(frames[-1][2]), "frames": np.array(frames)}

    def sample(self, samples, seed, tilted=False):
        """由腿长推导搜索盒，联合采样机身位置、腰和双臂，可加入倾斜。"""
        rng = np.random.default_rng(seed)
        reach = float(np.max(self.l1+self.l2))
        reach3 = np.array([reach, reach+max(abs(self.offset)), reach+max(abs(self.offset))])
        lower = np.max(self.points-self.origins-reach3, axis=0)
        upper = np.min(self.points-self.origins+reach3, axis=0)
        lower[2] = max(0., lower[2])
        varying = [j for j in range(1, self.model.njnt) if self.model.joint(j).name.startswith(("left_", "right_", "waist_"))]
        qa = self.model.jnt_qposadr[varying]
        ranges = self.model.jnt_range[varying]
        rows, leg_rows, geometry_count = [], [], 0
        base = self.evaluate(self.nominal)
        if base is None:
            raise RuntimeError("初始站姿未通过接地工作空间静力筛选")
        rows.append(base)
        leg_rows.append(base)
        for _ in range(samples):
            R = rotation(*rng.uniform(-np.pi/2, np.pi/2, 2)) if tilted else np.eye(3)
            if tilted:
                extent = np.sqrt((self.l1+self.l2)**2+self.offset**2)+self.radius+self.width
                anchors = self.origins @ R.T
                current_lower = np.max(self.points-anchors-extent[:, None], axis=0)
                current_upper = np.min(self.points-anchors+extent[:, None], axis=0)
                current_lower[2] = max(0., current_lower[2])
                if np.any(current_upper <= current_lower):
                    continue
                position = rng.uniform(current_lower, current_upper)
            else:
                position = rng.uniform(lower, upper)
            q = self.geometry(position, R)
            if q is None:
                continue
            geometry_count += 1
            legs = self.evaluate(q)
            if legs is not None:
                leg_rows.append(legs)
            q[qa] = rng.uniform(ranges[:, 0], ranges[:, 1])
            row = self.evaluate(q)
            if row is not None:
                rows.append(row)
        return rows, leg_rows, {"attempts": samples, "geometry_solutions": geometry_count,
                               "accepted": len(rows), "level_leg_accepted": len(leg_rows),
                               "search_box_world_m": [lower.tolist(), upper.tolist()]}
