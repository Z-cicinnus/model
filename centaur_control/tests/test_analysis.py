"""运动学有限差分、动力学残差、接地几何与冲击协议的可运行核验。"""

import json
from pathlib import Path
import mujoco
import numpy as np
from centaur_control.paths import MODEL_PATH
from centaur_control.models.definitions import HAND_SITES
from centaur_control.models.definitions import WHEEL_SITES
from centaur_control.kinematics.jacobians import wheel_contact_jacobian
from centaur_control.models.state import nominal_state
from centaur_control.kinematics.forward import forward_kinematics
from centaur_control.kinematics.inverse import inverse_kinematics
from centaur_control.kinematics.jacobians import jac_site
from centaur_control.dynamics.rigid_body import dynamics
from centaur_control.dynamics.rigid_body import mass_properties
from centaur_control.kinematics.forward import pose_error
from centaur_control.workspace.grounded import GroundedWorkspace
from centaur_control.workspace.grounded import rotation
from centaur_control.tasks.impacts import Experiment
from centaur_control.tasks.impacts import pulse


def check():
    m = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    d = nominal_state(m)
    initial = d.qpos.copy()
    rng = np.random.default_rng(20261104)
    velocity = rng.normal(size=m.nv)
    eps = 1e-7
    displaced = initial.copy()
    mujoco.mj_integratePos(m, displaced, velocity, eps)
    a = forward_kinematics(m, initial)
    b = forward_kinematics(m, displaced)
    errors = []
    for name in HAND_SITES+WHEEL_SITES:
        numerical = pose_error(b[name]["world"], a[name]["world"][:3, 3], a[name]["world"][:3, :3])/eps
        errors.append(np.max(abs(numerical-jac_site(m, d, name)@velocity)))
    assert max(errors) < 2e-6, errors
    # 轮地雅可比必须包含底盘和大腿角速度，按固定材料点有限差分核验。
    jd = wheel_contact_jacobian(m, d)
    d2 = mujoco.MjData(m)
    d2.qpos[:] = displaced
    mujoco.mj_forward(m, d2)
    contact_errors = []
    for i, name in enumerate(WHEEL_SITES):
        sid = m.site(name).id
        bid = int(m.site_bodyid[sid])
        point = d.site_xpos[sid]-[0., 0., .086]
        local = d.xmat[bid].reshape(3, 3).T @ (point-d.xpos[bid])
        moved = d2.xpos[bid]+d2.xmat[bid].reshape(3, 3)@local
        numerical = (moved-point)/eps
        contact_errors.append(np.max(abs(numerical[[2, 1, 0]]-jd[i*3:i*3+3]@velocity)))
    assert max(contact_errors) < 2e-6, contact_errors
    props = dynamics(m, d)
    assert np.allclose(props["M"], props["M"].T, atol=1e-12)
    assert np.linalg.eigvalsh(props["M"]).min() > 0
    assert np.allclose(props["com"], d.subtree_com[m.body("base_link").id], atol=1e-12)
    d.qvel[:] = .1*velocity
    d.ctrl[:] = initial[m.jnt_qposadr[m.actuator_trnid[:, 0]]]
    mujoco.mj_forward(m, d)
    M = np.zeros((m.nv,m.nv))
    mujoco.mj_fullM(m,M,d.qM)
    residual = M@d.qacc+d.qfrc_bias-d.qfrc_actuator-d.qfrc_passive-d.qfrc_constraint-d.qfrc_applied
    assert np.max(abs(residual)) < 1e-6, np.max(abs(residual))
    goals = {s:a[s]["world"].copy() for s in HAND_SITES}
    for target in goals.values():
        target[:3,3] += [.02,0,.015]
    base = np.eye(4)
    base[:3,3] = initial[:3]
    base[:3,:3] = rotation(pitch=np.radians(2))
    ik = inverse_kinematics(m, initial, goals, base)
    assert ik["success"], ik
    unreachable = {HAND_SITES[0]:goals[HAND_SITES[0]].copy()}
    unreachable[HAND_SITES[0]][:3,3] += [100,100,100]
    bad = inverse_kinematics(m, initial, unreachable, max_iterations=4)
    assert not bad["success"] and np.isfinite(bad["qpos"]).all()
    actual = forward_kinematics(m,bad["qpos"])[HAND_SITES[0]]["world"]
    assert abs(np.linalg.norm(unreachable[HAND_SITES[0]][:3,3]-actual[:3,3])-bad["position_error_m"]) < 1e-10
    workspace = GroundedWorkspace(m)
    geometry_errors = []
    for x,z,r,p in ((0,0,0,0),(.02,-.10,0,0),(0,0,.10,0),(0,0,-.10,0),(0,0,0,.04)):
        q = workspace.geometry(initial[:3]+[x,0,z],rotation(r,p))
        assert q is not None
        d.qpos[:] = q
        mujoco.mj_forward(m,d)
        for i,sid in enumerate(workspace.sites):
            bid = m.site_bodyid[sid]
            axle = d.xmat[bid].reshape(3,3)[:,1]
            delta = workspace.points[i]-d.site_xpos[sid]
            lateral = delta@axle
            radial = delta-lateral*axle
            geometry_errors.extend([abs(np.linalg.norm(radial)-workspace.radius[i]),
                                    max(0,abs(lateral)-workspace.width[i])])
            bottom = d.site_xpos[sid,2]-workspace.radius[i]*np.sqrt(1-axle[2]**2)-workspace.width[i]*abs(axle[2])
            geometry_errors.append(abs(bottom))
        assert workspace.evaluate(q) is not None
    assert max(geometry_errors) < 1e-8, geometry_errors
    baselines = {}
    for mode in ("free","active"):
        exp = Experiment(mode)
        result, trace = exp.run("base","forward",.1,0,record=True)
        assert result["recovered"] and result["min_wheels"] == 4, result
        if mode == "free":
            assert np.max(abs(trace["torque"][:,exp.wheels])) == 0
        else:
            assert np.max(abs(trace["torque"][:,exp.wheels])) <= 15.00001
        baselines[mode] = {k:result[k] for k in ("status","maximum_xy_displacement_m","max_abs_roll_pitch_deg")}
    exp = Experiment("free")
    result, trace = exp.run("chest","left",.05,100,True,observation=.2)
    assert abs(result["numerical_impulse_Ns"]/result["analytic_impulse_Ns"]-1) < .002
    assert np.all(trace["force"][:,0] == 0) and np.all(trace["force"][:,1] >= 0)
    assert pulse(-1,.1,100) == 0 and pulse(.1,.1,100) == 0
    output = {"passed":True,"site_jacobian_max_error":max(errors),"contact_jacobian_max_error":max(contact_errors),
              "dynamics_residual_max":float(np.max(abs(residual))),"contact_geometry_max_error_m":max(geometry_errors),
              "ik_position_error_m":ik["position_error_m"],"baselines":baselines}
    from centaur_control.paths import CHECKS_DIR
    path = CHECKS_DIR/"verification.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(output,ensure_ascii=False,indent=2),encoding="utf-8")
    print(json.dumps(output,ensure_ascii=False,indent=2))


def main():
    check()


if __name__ == "__main__":
    main()
