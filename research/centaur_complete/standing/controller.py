from pathlib import Path
import json
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parents[1]
class StandController:
 def __init__(self,m,d):
  case=json.loads((ROOT/'static_support/results.json').read_text())['cases'][0]
  self.m=m;self.qref=np.array(case['qpos']);self.qadr=np.array([m.jnt_qposadr[m.actuator_trnid[i,0]] for i in range(m.nu)]);self.vadr=np.array([m.jnt_dofadr[m.actuator_trnid[i,0]] for i in range(m.nu)])
  assert m.nq==40 and m.nu==33 and sum(m.jnt_type==mujoco.mjtJoint.mjJNT_FREE)==1
  force_by_name={x['actuator']:x['ctrl'] for x in case['torques']}
  self.ff=np.array([force_by_name[m.actuator(i).name] for i in range(m.nu)]);self.kp=np.zeros(m.nu);self.kd=self.kp.copy()
  for i in range(m.nu):
   n=m.actuator(i).name
   if n.startswith('as2w_'):kp,kd=(5,0.3) if '_foot' in n else (250,8)
   elif 'wrist' in n:kp,kd=1,.025
   elif 'head' in n:kp,kd=2,.05
   elif 'waist' in n:kp,kd=40,2
   else:kp,kd=25,1
   self.kp[i]=kp;self.kd[i]=kd
  self.last_raw=np.zeros(m.nu)
 def reset(self,d):
  mujoco.mj_resetData(self.m,d);d.qpos[:]=self.qref;mujoco.mj_forward(self.m,d)
 def target(self,t,arm_motion=False):
  target=self.qref[self.qadr].copy()
  if arm_motion:
   t=max(0,t-2);a=.15*(1-np.cos(2*np.pi*.1*t)) if t<10 else 0
   for i in range(self.m.nu):
    if 'shoulder_pitch_joint' in self.m.actuator(i).name:target[i]-=a
  return target
 def command(self,d,arm_motion=False):
  target=self.target(d.time,arm_motion)
  self.last_raw=self.ff+self.kp*(target-d.qpos[self.qadr])-self.kd*d.qvel[self.vadr]
  return np.clip(self.last_raw,self.m.actuator_ctrlrange[:,0],self.m.actuator_ctrlrange[:,1])
