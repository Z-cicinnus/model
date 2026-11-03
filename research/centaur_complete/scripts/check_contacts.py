"""Stage 2 diagnostics; does not edit either baseline XML."""
from pathlib import Path
import json,csv
import numpy as np
import mujoco
R=Path(__file__).resolve().parents[1];O=R/'checks_step2';O.mkdir(exist_ok=True)
m=mujoco.MjModel.from_xml_path(str(R/'centaur.xml'));d=mujoco.MjData(m)
def home():
 mujoco.mj_resetDataKeyframe(m,d,0);mujoco.mj_forward(m,d)
def distance(a,b):return float(mujoco.mj_geomDistance(m,d,a,b,2,None))
home();gs=[g for g in range(m.ngeom) if m.geom_bodyid[g]>0 and m.geom_contype[g]]
floor=next(g for g in range(m.ngeom) if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_PLANE)
wheels=[next(g for g in gs if m.body(m.geom_bodyid[g]).name==f'as2w_{leg}_foot') for leg in ['FL','FR','RL','RR']]
pairs=[]
for i,a in enumerate(gs):
 for b in gs[i+1:]:
  ba,bb=m.geom_bodyid[a],m.geom_bodyid[b]
  relation='same_weld' if m.body_weldid[ba]==m.body_weldid[bb] else ('adjacent' if m.body_parentid[ba]==bb or m.body_parentid[bb]==ba else 'nonadjacent')
  pairs.append((a,b,relation))
overlaps=[]
for a,b,rel in pairs:
 gap=distance(a,b)
 if gap< -1e-5:overlaps.append({'a':m.geom(a).name,'b':m.geom(b).name,'relation':rel,'signed_convex_distance_m':gap})
joints=[];acts=[]
for j in range(m.njnt):
 if m.jnt_type[j]==mujoco.mjtJoint.mjJNT_FREE:continue
 q=float(d.qpos[m.jnt_qposadr[j]]);lim=bool(m.jnt_limited[j]);lo,hi=m.jnt_range[j]
 joints.append({'name':m.joint(j).name,'home_rad':q,'limited':lim,'range_rad':[float(lo),float(hi)] if lim else None,'home_legal':bool(not lim or lo<=q<=hi),'axis_local':m.jnt_axis[j].tolist(),'axis_world_home':d.xaxis[j].tolist()})
for i in range(m.nu):
 home();j=m.actuator_trnid[i,0];v=m.jnt_dofadr[j];d.ctrl[i]=.1;mujoco.mj_forward(m,d)
 expected=np.zeros(m.nv);expected[v]=.1
 acts.append({'actuator':m.actuator(i).name,'joint':m.joint(j).name,'ctrl_Nm_test':.1,'generalized_force_max_error':float(np.max(abs(d.qfrc_actuator-expected))),'limits_Nm':m.actuator_ctrlrange[i].tolist()})
home();gaps=[distance(floor,g) for g in wheels];ground=[]
for g in gs:ground.append({'geom':m.geom(g).name,'ground_gap_m':distance(floor,g)})
# Moving the entire robot vertically for a diagnostic contact query only.
d.qpos[2]-=max(gaps)+.0001;mujoco.mj_forward(m,d)
touch=[{'a':m.geom(c.geom1).name,'b':m.geom(c.geom2).name,'distance_m':float(c.dist)} for c in d.contact]
# Home-pose wheel mesh radius variation over a full turn; no dynamics.
spin={}
for g in wheels:
 j=next(j for j in range(m.njnt) if m.jnt_bodyid[j]==m.geom_bodyid[g]);vals=[]
 for theta in np.linspace(0,2*np.pi,73):
  home();d.qpos[m.jnt_qposadr[j]]+=theta;mujoco.mj_forward(m,d);vals.append(distance(floor,g))
 spin[m.joint(j).name]={'ground_gap_min_m':min(vals),'ground_gap_max_m':max(vals),'peak_to_peak_m':max(vals)-min(vals),'positive_rolling_direction':'base +X at home (axis +Y, ground normal +Z)'}
# One-joint-at-a-time samples: convex screening, not an exhaustive collision-free envelope.
baseline_pairs={(x['a'],x['b']) for x in overlaps if x['relation']=='nonadjacent'}
sweep=[]
for j in range(m.njnt):
 if not m.jnt_limited[j]:continue
 for fraction in [.1,.5,.9]:
  home();lo,hi=m.jnt_range[j];d.qpos[m.jnt_qposadr[j]]=lo+fraction*(hi-lo);mujoco.mj_forward(m,d)
  bad=[]
  for a,b,rel in pairs:
   if rel!='nonadjacent':continue
   gap=distance(a,b)
   if gap<-.001:bad.append([m.geom(a).name,m.geom(b).name,gap])
  if bad:sweep.append({'joint':m.joint(j).name,'angle_rad':float(d.qpos[m.jnt_qposadr[j]]),'overlaps':bad,'new_pairs_relative_to_home':[x for x in bad if tuple(x[:2]) not in baseline_pairs]})
home();mass=float(m.body_mass.sum());base=m.body('as2w_base_link').id
report={'mujoco_version':mujoco.__version__,'scope':'geometric and actuator diagnostic, not standing validation','mass_kg':mass,'home_com_base_m':(d.subtree_com[base]-d.xpos[base]).tolist(),'home_contact_count':d.ncon,'wheel_gaps_m':dict(zip(['FL','FR','RL','RR'],gaps)),'diagnostic_lowering_m':max(gaps)+.0001,'contacts_after_diagnostic_lowering':touch,'home_convex_overlaps':overlaps,'ground_clearances':ground,'joints':joints,'actuators':acts,'wheel_rotation_screen':spin,'joint_sweep_samples':len([x for x in joints if x['limited']])*3,'joint_sweep_with_nonadjacent_overlaps':sweep,'self_collision_enabled':False}
(O/'diagnostics.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps({k:report[k] for k in ['mass_kg','home_com_base_m','home_contact_count','wheel_gaps_m','contacts_after_diagnostic_lowering','home_convex_overlaps','wheel_rotation_screen']},ensure_ascii=False,indent=2));print('sweep flagged',len(sweep));assert all(x['home_legal'] for x in joints);assert all(x['generalized_force_max_error']<1e-10 for x in acts)
