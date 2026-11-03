from pathlib import Path
import json
import numpy as np
import mujoco
R=Path(__file__).resolve().parents[1]
m=mujoco.MjModel.from_xml_path(str(R/'centaur_self_collision.xml'));d=mujoco.MjData(m)
a=mujoco.MjModel.from_xml_path(str(R/'centaur.xml'))
for field in ['body_mass','body_ipos','body_inertia','body_iquat','body_pos','body_quat','jnt_pos','jnt_axis','jnt_range','actuator_ctrlrange']:
 assert np.array_equal(getattr(m,field),getattr(a,field)),field
assert m.nexclude==len(json.loads((R/'collision_config.json').read_text())['manual_excludes'])
assert np.array_equal(m.key_qpos[1],a.key_qpos[0])
changed=np.flatnonzero(abs(m.key_qpos[0]-a.key_qpos[0])>1e-14)
assert set(changed)=={int(m.joint('r1a7_'+side+'_shoulder_roll_joint').qposadr[0]) for side in ['left','right']}

def home():mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
def contacts():
 out=[]
 for i,c in enumerate(d.contact):
  if m.geom_bodyid[c.geom1]==0 or m.geom_bodyid[c.geom2]==0:continue
  force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force)
  out.append({'a':m.geom(c.geom1).name,'b':m.geom(c.geom2).name,'gap_m':float(c.dist),'normal_force_N':float(force[0])})
 return out
home();initial=contacts();print('home',initial,flush=True)
# actual collision pipeline includes body-level masks and default welded-parent filter
sweep=[]
for j in range(m.njnt):
 if not m.jnt_limited[j]:continue
 for f in [.1,.5,.9]:
  home();d.qpos[m.jnt_qposadr[j]]=(1-f)*m.jnt_range[j,0]+f*m.jnt_range[j,1];mujoco.mj_forward(m,d)
  c=contacts()
  if c:sweep.append({'joint':m.joint(j).name,'fraction':f,'contacts':c})
home();d.qpos[2]-=.0006;mujoco.mj_forward(m,d)
ground=[{'a':m.geom(c.geom1).name,'b':m.geom(c.geom2).name,'gap_m':float(c.dist)} for c in d.contact if m.geom_bodyid[c.geom1]==0 or m.geom_bodyid[c.geom2]==0]
home()
for _ in range(100):mujoco.mj_step(m,d)
finite=bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all());warnings=[int(x.number) for x in d.warning]
fm=mujoco.MjModel.from_xml_path(str(R/'fixed_base_self_collision.xml'));fd=mujoco.MjData(fm);mujoco.mj_resetDataKeyframe(fm,fd,0);q=fd.qpos.copy()
for _ in range(500):
 mujoco.mj_forward(fm,fd);fd.ctrl[:]=np.clip(fd.qfrc_bias,fm.actuator_ctrlrange[:,0],fm.actuator_ctrlrange[:,1]);mujoco.mj_step(fm,fd)
drift=float(np.max(abs(fd.qpos-q)))
report={'version':mujoco.__version__,'unchanged_inertial_joint_actuator_and_mount':True,'home_change':'shoulder roll left +3deg right -3deg; original stored as assembly_reference','manual_excludes':m.nexclude,'home_internal_contacts':initial,'ground_contacts_after_0_6mm_lowering':ground,'sampled_poses':87,'sampled_poses_with_contacts':len(sweep),'sweep':sweep,'floating_0_1s_finite':finite,'floating_warnings':warnings,'fixed_gravity_hold_0_5s_max_drift_rad':drift,'collision_meshes':m.nmesh-a.nmesh,'status':'passed_home_and_smoke' if not initial and finite and not any(warnings) and drift<1e-7 else 'needs_review','limitations':['Approximate convex decomposition, not CAD interference verification.','Joint sweep is sparse, not a collision-free operating envelope.','No floating standing controller validation.']}
(R/'collision_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print({k:v for k,v in report.items() if k!='sweep'},flush=True)
