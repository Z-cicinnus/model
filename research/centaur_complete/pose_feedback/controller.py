"""Simulation-only base longitudinal/yaw feedback added to whole-robot PD."""
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from standing.controller import StandController
class PoseController(StandController):
 def __init__(self,m,d):
  super().__init__(m,d)
  self.wheels=[i for i in range(m.nu) if m.actuator(i).name.startswith('as2w_') and '_foot' in m.actuator(i).name]
  for i in self.wheels:self.kp[i]=0
 def command(self,d,arm_motion=False):
  super().command(d,arm_motion)
  w,x,y,z=d.qpos[3:7];yaw=np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
  forward=np.cos(yaw)*d.qpos[0]+np.sin(yaw)*d.qpos[1]
  vforward=np.cos(yaw)*d.qvel[0]+np.sin(yaw)*d.qvel[1]
  straight=np.clip(-100*forward-14*vforward,-4,4)
  yawterm=np.clip(40*yaw+3*d.qvel[5],-3,3)
  for i in self.wheels:
   side=1 if '_FL_' in self.m.actuator(i).name or '_RL_' in self.m.actuator(i).name else -1
   self.last_raw[i]+=straight+side*yawterm
  return np.clip(self.last_raw,self.m.actuator_ctrlrange[:,0],self.m.actuator_ctrlrange[:,1])
