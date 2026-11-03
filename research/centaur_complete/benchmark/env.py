"""Deterministic torque-level whole-robot benchmark; no Gym dependency."""
from pathlib import Path
import hashlib,json
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1]
class CentaurEnv:
 def __init__(self,task='hold'):
  self.config=json.loads((Path(__file__).parent/'tasks.json').read_text());self.task_name=task;self.task=self.config['tasks'][task]
  path=ROOT/'centaur_self_collision.xml';self.model_hash=hashlib.sha256(path.read_bytes()).hexdigest()
  self.m=mujoco.MjModel.from_xml_path(str(path));self.m.opt.timestep=self.config['physics_dt_s'];self.d=mujoco.MjData(self.m)
  self.dt=self.config['control_dt_s'];self.substeps=round(self.dt/self.m.opt.timestep)
  assert abs(self.substeps*self.m.opt.timestep-self.dt)<1e-12
  static=json.loads((ROOT/'static_support/results.json').read_text());assert static['model_sha256']==self.model_hash
  self.q0=np.array(static['cases'][0]['qpos']);self.base=self.m.body('as2w_base_link').id;self.hands=[self.m.body('r1a7_'+s+'_wrist_yaw_link').id for s in ['left','right']]
  self.vadr=np.array([self.m.jnt_dofadr[self.m.actuator_trnid[i,0]] for i in range(self.m.nu)])
  self.low=self.m.actuator_ctrlrange[:,0].copy();self.high=self.m.actuator_ctrlrange[:,1].copy()
  self.config_hash=hashlib.sha256((Path(__file__).parent/'tasks.json').read_bytes()).hexdigest();self.reset()
 def reset(self,seed=42):
  self.seed=int(seed);self.rng=np.random.default_rng(seed);mujoco.mj_resetData(self.m,self.d);self.d.qpos[:]=self.q0;mujoco.mj_forward(self.m,self.d)
  self.hand0=np.array([self.d.xpos[b].copy() for b in self.hands]);self.rot0=[self.d.xmat[b].reshape(3,3).copy() for b in self.hands]
  self.rows=[];self.qhistory=[];self.actions=[];self.internal_steps=0;self.energy=0.;self.clipped=0;self.max_torque_fraction=0.;self.done=False
  return self.observe(),self.info()
 def targets(self,t=None):
  t=self.d.time if t is None else t;u=np.clip((t-2)/6,0,1);s=u*u*u*(10-15*u+6*u*u)
  yaw=np.deg2rad(self.task['yaw_deg'])*s;pitch=np.deg2rad(self.task['pitch_deg'])*s
  rot=Rotation.from_euler('zy',[yaw,pitch]).as_matrix();yawrot=Rotation.from_euler('z',yaw).as_matrix()
  base=self.q0[:3]+[self.task['translation_m']*s,0,0]
  hp=np.array([base+yawrot@(p-self.q0[:3]+np.array([self.task['reach_m']*s,0,self.task['reach_m']*s])) for p in self.hand0])
  return base,rot,hp,[yawrot@r for r in self.rot0]
 def observe(self):
  bp,br,hp,hr=self.targets();quat=lambda r:Rotation.from_matrix(r).as_quat()[[3,0,1,2]]
  # 101 elements: 40 qpos + 39 qvel + 1 time + 3 base xyz + 4 quat + 6 wrist xyz + 8 wrist quat.
  return np.r_[self.d.qpos,self.d.qvel,self.d.time/self.config['duration_s'],bp,quat(br),hp.ravel(),np.concatenate([quat(r) for r in hr])].astype(np.float64)
 def info(self):
  return dict(task=self.task_name,seed=self.seed,model_sha256=self.model_hash,config_sha256=self.config_hash,benchmark_version=self.config['version'])
 def step(self,action):
  if self.done:raise RuntimeError('Call reset after termination/truncation')
  action=np.asarray(action,dtype=float)
  if action.shape!=(self.m.nu,) or not np.isfinite(action).all():raise ValueError('action must contain 33 finite motor torques')
  ctrl=np.clip(action,self.low,self.high);self.clipped+=int(np.any(ctrl!=action));self.d.ctrl[:]=ctrl
  terminated=False;reason=''
  for _ in range(self.substeps):
   self.d.xfrc_applied[:]=0
   if self.task.get('push_start_s',np.inf)<=self.d.time<self.task.get('push_end_s',-np.inf):self.d.xfrc_applied[self.base,:3]=self.task['push_world_N']
   assert np.all(self.d.qfrc_applied==0) and np.all(self.m.body_gravcomp==0)
   mujoco.mj_step(self.m,self.d);mujoco.mj_forward(self.m,self.d)
   self.energy+=float(np.sum(abs(self.d.qfrc_actuator[self.vadr]*self.d.qvel[self.vadr])))*self.m.opt.timestep
   self.internal_steps+=int(any(self.m.geom_bodyid[c.geom1]>0 and self.m.geom_bodyid[c.geom2]>0 and c.dist< -1e-6 for c in self.d.contact))
   if not np.isfinite(self.d.qpos).all() or any(w.number for w in self.d.warning):terminated=True;reason='numerical_failure';break
   rpy=Rotation.from_matrix(self.d.xmat[self.base].reshape(3,3)).as_euler('xyz')
   if max(abs(rpy[:2]))>np.deg2rad(self.config['termination']['roll_pitch_deg']) or self.d.qpos[2]<self.config['termination']['minimum_base_height_m']:terminated=True;reason='fallen';break
  self.max_torque_fraction=max(self.max_torque_fraction,float(np.max(abs(ctrl)/np.maximum(abs(self.low),abs(self.high)))))
  bp,br,hp,hr=self.targets();be=float(np.linalg.norm(self.d.qpos[:3]-bp));he=np.array([np.linalg.norm(self.d.xpos[b]-p) for b,p in zip(self.hands,hp)])
  ae=float(Rotation.from_matrix(br@self.d.xmat[self.base].reshape(3,3).T).magnitude());ha=np.array([Rotation.from_matrix(r@self.d.xmat[b].reshape(3,3).T).magnitude() for b,r in zip(self.hands,hr)])
  cost=(be/.03)**2+np.mean((he/.012)**2)+(ae/np.deg2rad(.5))**2+np.mean((ha/np.deg2rad(5))**2)+.001*np.mean((ctrl/self.high)**2)
  reward=-float(cost)*self.dt
  self.rows.append(np.r_[self.d.time,be,he,ae,ha,reward]);self.qhistory.append(self.d.qpos.copy());self.actions.append(ctrl.copy())
  truncated=self.d.time>=self.config['duration_s']-1e-9;self.done=terminated or truncated
  info=self.info();info['termination_reason']=reason
  if self.done:info['metrics']=self.metrics(terminated)
  return self.observe(),reward,terminated,truncated,info
 def metrics(self,terminated=False):
  a=np.array(self.rows);rms=np.sqrt(np.mean(a[:,1:7]**2,axis=0));last=a[-1,1:7];lim=self.config['success_limits']
  checks=dict(completed=not terminated and self.d.time>=self.config['duration_s']-1e-9,base_rmse=rms[0]<lim['base_position_rmse_m'],hands_rmse=bool(np.all(rms[1:3]<lim['hand_position_rmse_m'])),base_orientation=np.rad2deg(rms[3])<lim['base_orientation_rmse_deg'],hand_orientation=bool(np.all(np.rad2deg(rms[4:])<lim['hand_orientation_rmse_deg'])),final_base=last[0]<lim['final_base_position_m'],final_hands=bool(np.all(last[1:3]<lim['final_hand_position_m'])),final_base_orientation=np.rad2deg(last[3])<lim['final_base_orientation_deg'],no_internal_contacts=self.internal_steps==0)
  return dict(**self.info(),seconds=float(self.d.time),rmse_base_m=float(rms[0]),rmse_hands_m=rms[1:3].tolist(),rmse_base_orientation_deg=float(np.rad2deg(rms[3])),rmse_hand_orientation_deg=np.rad2deg(rms[4:]).tolist(),final_errors=last.tolist(),absolute_joint_mechanical_work_J=self.energy,clipped_action_steps=self.clipped,max_torque_fraction=self.max_torque_fraction,internal_contact_steps=self.internal_steps,total_reward=float(a[:,-1].sum()),checks={k:bool(v) for k,v in checks.items()},success=bool(all(checks.values())))
