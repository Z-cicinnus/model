"""Rigid point-contact static equilibrium; not a soft-contact standing controller."""
from pathlib import Path
import json,hashlib
import numpy as np
import mujoco
from scipy.spatial import ConvexHull
R=Path(__file__).resolve().parents[1];OUT=R/'static_support';OUT.mkdir(exist_ok=True)
FILE=R/'centaur_self_collision.xml'
m=mujoco.MjModel.from_xml_path(str(FILE));d=mujoco.MjData(m)
legs=['FL','FR','RL','RR'];wheel_geoms=[m.geom(f'as2w_{leg}_foot_collision_1').id for leg in legs]
floor=m.geom('as2w_floor').id;base=m.body('as2w_base_link').id

def solve(label,thigh):
 mujoco.mj_resetDataKeyframe(m,d,m.key('home').id)
 for leg in legs:
  for suffix,angle in [('thigh',thigh),('calf',-2*thigh),('foot',thigh)]:d.qpos[m.joint(f'as2w_{leg}_{suffix}').qposadr[0]]=angle
 mujoco.mj_forward(m,d)
 gaps=[float(mujoco.mj_geomDistance(m,d,floor,w,2,None)) for w in wheel_geoms]
 d.qpos[2]-=max(gaps);q=d.qpos.copy()
 # Query contact points with a tiny probe penetration; restore exact tangent pose.
 d.qpos[2]-=.0001;mujoco.mj_forward(m,d)
 points={}
 for c in d.contact:
  for w in wheel_geoms:
   if {int(c.geom1),int(c.geom2)}=={floor,w}:
    p=c.pos.copy();p[2]=0;points[w]=p
 assert len(points)==4
 d.qpos[:]=q;mujoco.mj_forward(m,d)
 internal=[{'a':m.geom(c.geom1).name,'b':m.geom(c.geom2).name,'gap_m':float(c.dist)} for c in d.contact if m.geom_bodyid[c.geom1]>0 and m.geom_bodyid[c.geom2]>0 and c.dist< -1e-6]
 legal=all(not m.jnt_limited[j] or m.jnt_range[j,0]<=q[m.jnt_qposadr[j]]<=m.jnt_range[j,1] for j in range(m.njnt))
 p=np.array([points[w] for w in wheel_geoms]);J=[]
 for w,point in zip(wheel_geoms,p):
  jp=np.zeros((3,m.nv));jr=jp.copy();mujoco.mj_jac(m,d,jp,jr,point,m.geom_bodyid[w]);J.append(jp)
 J=np.array(J);V=J[:,2,:].T;g=d.qfrc_bias.copy();A=V[:6];b=g[:6]
 # Closest load vector to equal loading satisfying floating-base equilibrium.
 equal=np.ones(4)*m.body_mass.sum()*(-m.opt.gravity[2])/4
 f=equal+np.linalg.pinv(A)@(b-A@equal)
 residual=A@f-b
 if np.max(abs(residual))>1e-7:raise RuntimeError('Vertical-only equilibrium inconsistent')
 tau_g=g-V@f
 ctrl=np.zeros(m.nu);torques=[]
 for i in range(m.nu):
  j=m.actuator_trnid[i,0];v=m.jnt_dofadr[j];gear=m.actuator_gear[i,0];ctrl[i]=tau_g[v]/gear
  lo,hi=m.actuator_ctrlrange[i];limit=hi if ctrl[i]>=0 else -lo
  torques.append({'actuator':m.actuator(i).name,'joint':m.joint(j).name,'holding_torque_Nm':float(tau_g[v]),'ctrl':float(ctrl[i]),'ctrl_limits':[float(lo),float(hi)],'utilization':float(abs(ctrl[i])/limit),'within_limit':bool(lo<=ctrl[i]<=hi)})
 com=d.subtree_com[base].copy();hull=ConvexHull(p[:,:2]);margin=float(np.min(-(hull.equations[:,:2]@com[:2]+hull.equations[:,2])))
 force=np.column_stack([np.zeros(4),np.zeros(4),f]);moment=np.cross(p-com,force).sum(axis=0)
 # Independently feed ideal contact forces and computed actuator torques through forward dynamics.
 old_disable=m.opt.disableflags;m.opt.disableflags=int(old_disable)|int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
 d.qfrc_applied[:]=V@f;d.ctrl[:]=ctrl;mujoco.mj_forward(m,d);acc=float(np.max(abs(d.qacc)))
 m.opt.disableflags=old_disable;d.qfrc_applied[:]=0;d.ctrl[:]=0
 assert np.linalg.norm(moment)<1e-6
 feasible=bool(np.min(f)>0 and margin>0 and not internal and legal and all(x['within_limit'] for x in torques))
 if feasible:assert acc<1e-6
 return {'label':label,'thigh_rad':thigh,'knee_rad':-2*thigh,'base_height_m':float(q[2]),'qpos':q.tolist(),'com_world_m':com.tolist(),'com_in_base_m':(com-np.array([0,0,q[2]])).tolist(),'contact_positions_world_m':dict(zip(legs,p.tolist())),'vertical_loads_N':dict(zip(legs,f.tolist())),'total_weight_N':float(f.sum()),'support_polygon_margin_m':margin,'equilibrium_residual_max':float(max(abs(residual))),'global_moment_residual_Nm':moment.tolist(),'ideal_forward_dynamics_max_acceleration':acc,'joint_angles_legal':bool(legal),'internal_contacts':internal,'feasible_under_assumptions':feasible,'max_utilization':max(x['utilization'] for x in torques),'torques':torques}
results=[solve('home',.65),solve('higher_body',.55),solve('lower_body',.75)]
report={'model_sha256':hashlib.sha256(FILE.read_bytes()).hexdigest(),'mujoco_version':mujoco.__version__,'mass_kg':float(m.body_mass.sum()),'gravity_m_s2':m.opt.gravity.tolist(),'assumptions':['Unloaded, horizontal rigid ground; four unilateral point contacts.','Only vertical contact forces; zero tangential forces and zero contact moments.','Minimum squared deviation from equal loading; redundant support load is not unique.','Static qvel=qacc=0; source torque limits, not verified continuous motor ratings.','Contact points are geometric estimates from a 0.1mm query probe, then projected to z=0.','Ideal-force forward dynamics disables collision solver only in that verification; does not test soft-contact standing.'],'cases':results}
(OUT/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps({k:report[k] for k in ['mass_kg','mujoco_version']},ensure_ascii=False))
for x in results:print(json.dumps({k:v for k,v in x.items() if k not in ['qpos','torques']},ensure_ascii=False,indent=2));print('peak',max(x['torques'],key=lambda z:z['utilization']))
