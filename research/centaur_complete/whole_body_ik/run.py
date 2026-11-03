from pathlib import Path
import sys,json,hashlib,argparse,time
import numpy as np
import mujoco
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parents[1];sys.path.insert(0,str(R/'standing'))
from controller import StandController
OUT=Path(__file__).resolve().parent/'results';OUT.mkdir(exist_ok=True)
def run(case,viewer=False):
 m=mujoco.MjModel.from_xml_path(str(R/'centaur_self_collision.xml'));d=mujoco.MjData(m);c=StandController(m,d);c.reset(d)
 assert np.all(m.body_gravcomp==0) and not np.any(d.qfrc_applied) and not np.any(d.xfrc_applied)
 static=json.loads((R/'static_support/results.json').read_text());assert hashlib.sha256((R/'centaur_self_collision.xml').read_bytes()).hexdigest()==static['model_sha256']
 ref=mujoco.MjData(m);ref.qpos[:]=d.qpos;mujoco.mj_forward(m,ref);q0=d.qpos.copy()
 hands=[m.body('r1a7_'+s+'_wrist_yaw_link').id for s in ['left','right']];p0=np.array([d.xpos[b].copy() for b in hands]);rots=[d.xmat[b].reshape(3,3).copy() for b in hands]
 legs=['FL','FR','RL','RR'];wg=[m.geom('as2w_'+l+'_foot_collision_1').id for l in legs];wb=[int(m.geom_bodyid[g]) for g in wg]
 cp=np.array([static['cases'][0]['contact_positions_world_m'][l] for l in legs]);local=[d.xmat[b].reshape(3,3).T@(p-d.xpos[b]) for b,p in zip(wb,cp)]
 wheelv=[int(m.joint('as2w_'+l+'_foot').dofadr[0]) for l in legs]
 def jac(data,b,p):
  a=np.zeros((3,m.nv));bjac=a.copy();mujoco.mj_jac(m,data,a,bjac,p,b);return a,bjac
 duration=12;dt=.02;refs=[];speeds=[];ffs=[];targets=[];maxeq=0.;refcollision=0;qp_res=[]
 for t in np.arange(0,duration+dt/2,dt):
  mujoco.mj_forward(m,ref);phase=np.clip((t-2)/8,0,1);s=np.sin(np.pi*phase)**2
  if case=='hold':s=0
  pitch=np.deg2rad(1)*s if case=='coordinated' else 0
  target=p0+np.array([.015*s,0,.015*s]);A=[];y=[]
  for i,b in enumerate(hands):
   jp,jr=jac(ref,b,ref.xpos[b]);A.extend([10*jp,2*jr]);y.extend([10*4*(target[i]-ref.xpos[b]),2*4*Rotation.from_matrix(rots[i]@ref.xmat[b].reshape(3,3).T).as_rotvec()])
  base=m.body('as2w_base_link').id;jp,jr=jac(ref,base,ref.xpos[base]);Rd=Rotation.from_euler('y',pitch).as_matrix()
  A.extend([10*jp,10*jr,.03*np.eye(m.nv)]);y.extend([10*4*(q0[:3]-ref.qpos[:3]),10*4*Rotation.from_matrix(Rd@ref.xmat[base].reshape(3,3).T).as_rotvec(),np.zeros(m.nv)])
  A=np.vstack(A);y=np.concatenate(y);J=[];e=[]
  for b,p,goal in zip(wb,local,cp):
   point=ref.xpos[b]+ref.xmat[b].reshape(3,3)@p;j,_=jac(ref,b,point);J.append(j);e.extend(5*(goal-point))
  J=np.vstack(J);E=np.vstack([J,np.eye(m.nv)[wheelv]]);rhs=np.r_[e,np.zeros(4)]
  lo=np.full(m.nv,-.5);hi=-lo;lo[:3]=-.04;hi[:3]=.04;lo[3:6]=-.15;hi[3:6]=.15
  for j in range(1,m.njnt):
   if m.jnt_limited[j]:
    v=m.jnt_dofadr[j];pos=ref.qpos[m.jnt_qposadr[j]];lo[v]=max(lo[v],(m.jnt_range[j,0]+1e-4-pos)/dt);hi[v]=min(hi[v],(m.jnt_range[j,1]-1e-4-pos)/dt)
  H=A.T@A;g=-A.T@y
  sol=minimize(lambda v:.5*v@H@v+g@v,np.zeros(m.nv),jac=lambda v:H@v+g,bounds=list(zip(lo,hi)),constraints={'type':'eq','fun':lambda v:E@v-rhs,'jac':lambda v:E},method='SLSQP',options={'ftol':1e-10,'maxiter':150})
  if not sol.success:raise RuntimeError(sol.message)
  v=sol.x;maxeq=max(maxeq,float(abs(E@v-rhs).max()));qp_res.append(sol.nit)
  refcollision+=int(any(m.geom_bodyid[x.geom1]>0 and m.geom_bodyid[x.geom2]>0 and x.dist< -1e-6 for x in ref.contact))
  # Quasistatic gravity compensation, four vertical load allocation.
  V=J.reshape(4,3,m.nv)[:,2,:].T;grav=ref.qfrc_bias.copy();equal=np.full(4,m.body_mass.sum()*9.81/4);f=equal+np.linalg.pinv(V[:6])@(grav[:6]-V[:6]@equal)
  ff=(grav-V@f)[c.vadr]/m.actuator_gear[:,0]
  refs.append(ref.qpos.copy());speeds.append(v);ffs.append(ff);targets.append(np.r_[target.flatten(),pitch]);mujoco.mj_integratePos(m,ref.qpos,v,dt)
 refs=np.array(refs);speeds=np.array(speeds);ffs=np.array(ffs);targets=np.array(targets)
 np.savez_compressed(OUT/(case+'_reference.npz'),qpos=refs,qvel=speeds,feedforward=ffs,targets=targets,dt=dt)
 print('reference',case,'collision',refcollision,'eq',maxeq,flush=True)
 # Execute reference using motor torques only, with a freely moving base.
 view=None
 if viewer:
  import mujoco.viewer as mv
  view=mv.launch_passive(m,d)
 actual_q=[];rows=[];collisions=0;sat=0;warnings=0;minload=1e9;maxutil=0
 for k in range(round(duration/m.opt.timestep)):
  wall=time.perf_counter();idx=min(int(d.time/dt),len(refs)-1);qr=refs[idx];vr=speeds[idx]
  raw=ffs[idx]+c.kp*(qr[c.qadr]-d.qpos[c.qadr])+c.kd*(vr[c.vadr]-d.qvel[c.vadr]);d.ctrl[:]=np.clip(raw,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1]);sat+=int(np.max(abs(raw-d.ctrl))>1e-9);maxutil=max(maxutil,float(np.max(abs(d.ctrl)/m.actuator_ctrlrange[:,1])))
  mujoco.mj_step(m,d);mujoco.mj_forward(m,d)
  collisions+=int(any(m.geom_bodyid[x.geom1]>0 and m.geom_bodyid[x.geom2]>0 and x.dist< -1e-6 for x in d.contact))
  if k%10==0:
   loads=np.zeros(4)
   for ci,x in enumerate(d.contact):
    for i,g in enumerate(wg):
     if g in [x.geom1,x.geom2] and (m.geom_bodyid[x.geom1]==0 or m.geom_bodyid[x.geom2]==0):
      force=np.zeros(6);mujoco.mj_contactForce(m,d,ci,force);loads[i]+=(x.frame.reshape(3,3).T@force[:3])[2]*(1 if x.geom2==g else -1)
   minload=min(minload,float(loads.min()));rpy=Rotation.from_quat(d.qpos[[4,5,6,3]]).as_euler('xyz');actual=np.array([d.xpos[b] for b in hands]);err=np.linalg.norm(actual-targets[idx,:6].reshape(2,3),axis=1)
   actual_q.append(d.qpos.copy());rows.append(np.r_[d.time,err,rpy,targets[idx,6],d.qpos[:3],loads]);
  if view:
   if not view.is_running():break
   if k%10==0:view.sync()
   time.sleep(max(0,m.opt.timestep-(time.perf_counter()-wall)))
 if view:view.close()
 a=np.array(rows);metrics=dict(case=case,seconds=float(d.time),reference_contact_steps=refcollision,qp_equality_residual=maxeq,hand_max_error_m=a[:,1:3].max(axis=0).tolist(),pitch_tracking_max_deg=float(np.rad2deg(abs(a[:,4]-a[:,6]).max())),pitch_range_deg=np.rad2deg([a[:,4].min(),a[:,4].max()]).tolist(),roll_max_deg=float(np.rad2deg(abs(a[:,3]).max())),xy_max_m=float(np.linalg.norm(a[:,7:9],axis=1).max()),height_error_max_m=float(abs(a[:,9]-q0[2]).max()),internal_contact_steps=collisions,saturated_steps=sat,max_torque_fraction=maxutil,min_sampled_wheel_load_N=minload,engine_warnings=[int(w.number) for w in d.warning])
 metrics['passed']=bool(not refcollision and not collisions and not sat and not any(metrics['engine_warnings']) and max(metrics['hand_max_error_m'])<.02 and metrics['pitch_tracking_max_deg']<.5 and metrics['roll_max_deg']<1 and metrics['xy_max_m']<.03 and metrics['height_error_max_m']<.01 and minload>1)
 np.savez_compressed(OUT/(case+'_actual.npz'),samples=a,qpos=np.array(actual_q),columns=['time','left_error','right_error','roll','pitch','yaw','target_pitch','x','y','z','FL_load','FR_load','RL_load','RR_load'])
 (OUT/(case+'.json')).write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics),flush=True)
 return metrics['passed']
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--case',choices=['hold','arms','coordinated','all'],default='coordinated');p.add_argument('--viewer',action='store_true');args=p.parse_args()
 ok=[run(case,args.viewer) for case in (['hold','arms','coordinated'] if args.case=='all' else [args.case])]
 if not all(ok):raise SystemExit('Acceptance checks failed; inspect results.')
