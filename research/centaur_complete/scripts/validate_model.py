"""Validate assembly fidelity and actual single-system dynamic coupling."""
from pathlib import Path
import json
import xml.etree.ElementTree as E
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[1]

def full_inertia(m,b):
    R=np.zeros(9);mujoco.mju_quat2Mat(R,m.body_iquat[b]);R=R.reshape(3,3)
    return R@np.diag(m.body_inertia[b])@R.T

def source_model(filename,part):
    r=E.parse(ROOT/'source'/filename).getroot()
    r.find('compiler').attrib.pop('meshdir',None)
    for x in r.findall('asset/mesh'):
        x.set('file',str((ROOT/'meshes'/part/Path(x.get('file')).name).resolve()))
    out=ROOT/f'_reference_{part}.xml'
    E.ElementTree(r).write(out,encoding='utf-8')
    try:return mujoco.MjModel.from_xml_path(str(out))
    finally:out.unlink(missing_ok=True)

def validate():
    cfg=json.loads((ROOT/'assembly_config.json').read_text())
    m=mujoco.MjModel.from_xml_path(str(ROOT/'centaur.xml'));d=mujoco.MjData(m)
    a=source_model('AS2W.xml','as2w');u=source_model('r1_a7.xml','r1a7')
    mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
    assert (m.nq,m.nv,m.nu,m.nbody)==(40,39,33,38)
    free=np.flatnonzero(m.jnt_type==mujoco.mjtJoint.mjJNT_FREE)
    assert len(free)==1
    assert m.joint(int(free[0])).name=='as2w_base_freejoint'
    base=m.body('as2w_base_link').id;pelvis=m.body('r1a7_pelvis_link').id
    assert m.body_parentid[pelvis]==base
    assert abs(m.body_mass.sum()-a.body_mass.sum()-u.body_mass.sum())<1e-12
    err_mass=err_com=err_I=0.
    for ref,prefix in [(a,'as2w_'),(u,'')]:
        for i in range(1,ref.nbody):
            b=m.body(prefix+ref.body(i).name).id
            err_mass=max(err_mass,float(abs(m.body_mass[b]-ref.body_mass[i])))
            err_com=max(err_com,float(np.max(np.abs(m.body_ipos[b]-ref.body_ipos[i]))))
            err_I=max(err_I,float(np.max(np.abs(full_inertia(m,b)-full_inertia(ref,i)))))
    assert max(err_mass,err_com,err_I)<1e-12
    for j in range(m.njnt):
        if m.jnt_limited[j]:
            v=d.qpos[m.jnt_qposadr[j]]
            assert m.jnt_range[j,0]<=v<=m.jnt_range[j,1],m.joint(j).name
    expected=d.xpos[base]+d.xmat[base].reshape(3,3)@np.array(cfg['mount_position_in_as2w_base_m'])
    assert np.allclose(d.xpos[pelvis],expected,atol=1e-14)
    M=np.zeros((m.nv,m.nv));mujoco.mj_fullM(m,d,M)
    eigen=float(np.linalg.eigvalsh(M).min());assert eigen>0
    upper_dofs=[m.joint(x.get('joint')).dofadr[0] for x in E.parse(ROOT/'source/r1_a7.xml').getroot().find('actuator')]
    coupling=float(np.linalg.norm(M[np.ix_(range(6),upper_dofs)]))
    assert coupling>1e-3
    # All motor commands are direct joint output torques in N*m.
    rng=np.random.default_rng(9)
    d.ctrl[:]=rng.uniform(m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
    mujoco.mj_forward(m,d)
    for i in range(m.nu):
        j=m.actuator_trnid[i,0];v=m.jnt_dofadr[j]
        assert abs(d.qfrc_actuator[v]-d.ctrl[i])<1e-10
    # Conserved momentum under an internal arm torque in free space.
    old_g=m.opt.gravity.copy();old_masks=(m.geom_contype.copy(),m.geom_conaffinity.copy())
    old_integrator=m.opt.integrator
    # RK4 isolates momentum conservation from first-order integration drift.
    m.opt.integrator=mujoco.mjtIntegrator.mjINT_RK4
    m.opt.gravity[:]=0;m.geom_contype[:]=0;m.geom_conaffinity[:]=0
    mujoco.mj_resetDataKeyframe(m,d,0)
    d.ctrl[m.actuator('r1a7_left_shoulder_pitch_joint_motor').id]=1.0
    mujoco.mj_forward(m,d)
    for _ in range(20):mujoco.mj_step(m,d)
    mujoco.mj_forward(m,d);mujoco.mj_subtreeVel(m,d)
    vbase=d.qvel[:6].copy();angmom=d.subtree_angmom[base].copy()
    linmom=d.subtree_linvel[base]*m.body_subtreemass[base]
    assert np.linalg.norm(vbase)>1e-5
    assert np.linalg.norm(linmom)<1e-5 and np.linalg.norm(angmom)<1e-5
    m.opt.gravity[:]=old_g;m.geom_contype[:]=old_masks[0];m.geom_conaffinity[:]=old_masks[1]
    m.opt.integrator=old_integrator
    # A short real-contact dynamic execution smoke test, not a walking test.
    mujoco.mj_resetDataKeyframe(m,d,0)
    for _ in range(100):mujoco.mj_step(m,d)
    assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
    assert not any(x.number for x in d.warning)
    # Inspect/hold variant: exact gravity compensation is sufficient at rest.
    fm=mujoco.MjModel.from_xml_path(str(ROOT/'fixed_base.xml'));fd=mujoco.MjData(fm)
    mujoco.mj_resetDataKeyframe(fm,fd,0);target=fd.qpos.copy()
    for _ in range(1000):
        mujoco.mj_forward(fm,fd)
        fd.ctrl[:]=fd.qfrc_bias
        mujoco.mj_step(fm,fd)
    drift=float(np.max(np.abs(fd.qpos-target)))
    assert drift<1e-8
    mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
    # Separation of the two convex collision hulls at the provisional interface.
    interface_gap=float(mujoco.mj_geomDistance(m,d,m.geom('as2w_base_link_collision_1').id,
                          m.geom('r1a7_pelvis_link_collision0').id,1.0,None))
    assert interface_gap>=0
    report={'model':'四轮足半人马','mujoco_version':mujoco.__version__,'status':'passed',
            'mass_kg':{'as2w':float(a.body_mass.sum()),'r1_a7':float(u.body_mass.sum()),'combined':float(m.body_mass.sum()),'additional_mount':0},
            'physical_bodies':m.nbody-1,'active_joint_count':33,'actuators':m.nu,'freejoint_count':len(free),'nq':m.nq,'nv':m.nv,
            'max_body_mass_error_kg':err_mass,'max_body_com_error_m':err_com,'max_body_inertia_error_kg_m2':err_I,
            'mount_position_in_base_m':cfg['mount_position_in_as2w_base_m'],
            'home_com_world_m':d.subtree_com[base].tolist(),
            'home_com_in_base_m':(d.xmat[base].reshape(3,3).T@(d.subtree_com[base]-d.xpos[base])).tolist(),
            'home_mass_matrix_min_eigenvalue':eigen,'base_upper_mass_matrix_block_norm':coupling,
            'internal_arm_torque_test_s':0.02,'internal_arm_test_integrator':'RK4','base_reaction_velocity':vbase.tolist(),
            'free_space_linear_momentum_residual':float(np.linalg.norm(linmom)),
            'free_space_angular_momentum_residual':float(np.linalg.norm(angmom)),
            'fixed_inspection_hold_drift_rad':drift,'floating_contact_execution_s':0.1,
            'base_pelvis_convex_hull_separation_m':interface_gap,
            'limitations':['Mount pose provisional; no added hardware connector mass or inertia.',
                           'Self-collision disabled; convex mesh environment contacts.',
                           'No locomotion stability claim; old AS2-W controller requires interface adaptation.',
                           'Source camera inertia retained pending CAD verification.',
                           'IMU and joint sensors are ideal simulation signals without noise.']}
    (ROOT/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':validate()
