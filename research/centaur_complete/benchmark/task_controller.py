"""Whole-model velocity QP reference plus motor PD and wheel pose feedback."""
from pathlib import Path
import sys
import numpy as np
import mujoco
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from env import ROOT
sys.path.insert(0,str(ROOT))
from standing.controller import StandController
class TaskController(StandController):
 def __init__(self,env):
  super().__init__(env.m,env.d);self.env=env;self.dt=.02;self.yaw_integral=0.;self.x_integral=0.
  self.wheels=[i for i in range(self.m.nu) if self.m.actuator(i).name.startswith('as2w_') and '_foot' in self.m.actuator(i).name]
  self.leg_act=[i for i in range(self.m.nu) if self.m.actuator(i).name.startswith('as2w_') and i not in self.wheels]
  self.leg_dof=self.vadr[self.leg_act];self.correction=np.zeros(len(self.leg_act));self.next_feedback=0.
  self.jlo=np.full(self.m.nu,-np.inf);self.jhi=np.full(self.m.nu,np.inf)
  for i in range(self.m.nu):
   j=self.m.actuator_trnid[i,0]
   if self.m.jnt_limited[j]:self.jlo[i],self.jhi[i]=self.m.jnt_range[j]+[1e-4,-1e-4]
  self.plan()
 def plan(self):
  e=self.env;m=self.m;ref=mujoco.MjData(m);ref.qpos[:]=self.qref;mujoco.mj_forward(m,ref)
  wg=[m.geom('as2w_'+l+'_foot_collision_1').id for l in ['FL','FR','RL','RR']];wb=[int(m.geom_bodyid[g]) for g in wg]
  centers0=np.array([ref.xpos[b].copy() for b in wb]);self.ref_collision=0;self.eq_error=0;self.min_load=1e9;self.ff_residual=0
  self.q=[];self.v=[];self.ff=[]
  def jac(b,p):
   j=np.zeros((3,m.nv));r=j.copy();mujoco.mj_jac(m,ref,j,r,p,b);return j,r
  for t in np.arange(0,e.config['duration_s']+self.dt/2,self.dt):
   mujoco.mj_forward(m,ref);bp,br,hp,hr=e.targets(t);A=[];y=[]
   for b,p,r in zip(e.hands,hp,hr):
    jp,jr=jac(b,ref.xpos[b]);A.extend([20*jp,3*jr]);y.extend([20*5*(p-ref.xpos[b]),3*5*Rotation.from_matrix(r@ref.xmat[b].reshape(3,3).T).as_rotvec()])
   jp,jr=jac(e.base,ref.xpos[e.base]);A.extend([40*jp,40*jr]);y.extend([40*5*(bp-ref.xpos[e.base]),40*5*Rotation.from_matrix(br@ref.xmat[e.base].reshape(3,3).T).as_rotvec()])
   posture=np.zeros(m.nv);mujoco.mj_differentiatePos(m,posture,1,ref.qpos,self.qref)
   reg=np.ones(m.nv)*.1;reg[:6]=.01
   for i in self.wheels:reg[self.vadr[i]]=.001;posture[self.vadr[i]]=0
   A.append(np.diag(reg));y.append(reg*posture)
   # Instantaneous rolling contact: normal and longitudinal no slip, lateral slip permitted.
   yaw=Rotation.from_matrix(ref.xmat[e.base].reshape(3,3)).as_euler('xyz')[2];forward=np.array([np.cos(yaw),np.sin(yaw),0.]);lateral=np.array([-np.sin(yaw),np.cos(yaw),0.]);J=[];E=[];rhs=[]
   for b,c0 in zip(wb,centers0):
    center=ref.xpos[b];point=center+np.array([0,0,-.086]);j,_=jac(b,point);J.append(j)
    E.extend([j[2],forward@j]);rhs.extend([5*(c0[2]-center[2]),0.]);A.append((.5*lateral@j)[None,:]);y.append(np.zeros(1))
   A=np.vstack(A);y=np.concatenate(y);E=np.array(E);rhs=np.array(rhs);J=np.array(J)
   lo=np.full(m.nv,-.5);hi=-lo;lo[:3]=-.15;hi[:3]=.15;lo[3:6]=-.3;hi[3:6]=.3
   for i in self.wheels:lo[self.vadr[i]]=-4;hi[self.vadr[i]]=4
   for j in range(1,m.njnt):
    if m.jnt_limited[j]:
     k=m.jnt_dofadr[j];p=ref.qpos[m.jnt_qposadr[j]];lo[k]=max(lo[k],(m.jnt_range[j,0]+1e-4-p)/self.dt);hi[k]=min(hi[k],(m.jnt_range[j,1]-1e-4-p)/self.dt)
   H=A.T@A;g=-A.T@y;sol=minimize(lambda v:.5*v@H@v+g@v,np.zeros(m.nv),jac=lambda v:H@v+g,bounds=list(zip(lo,hi)),constraints={'type':'eq','fun':lambda v:E@v-rhs,'jac':lambda v:E},method='SLSQP',options={'ftol':1e-10,'maxiter':150})
   if not sol.success:raise RuntimeError('QP: '+sol.message)
   self.eq_error=max(self.eq_error,float(abs(E@sol.x-rhs).max()));self.ref_collision+=int(any(m.geom_bodyid[c.geom1]>0 and m.geom_bodyid[c.geom2]>0 and c.dist< -1e-6 for c in ref.contact))
   V=J[:,2,:].T;g=ref.qfrc_bias.copy();f0=np.full(4,m.body_mass.sum()*9.81/4);f=f0+np.linalg.pinv(V[:6])@(g[:6]-V[:6]@f0);self.min_load=min(self.min_load,float(f.min()));self.ff_residual=max(self.ff_residual,float(abs(V[:6]@f-g[:6]).max()))
   self.q.append(ref.qpos.copy());self.v.append(sol.x);self.ff.append((g-V@f)[self.vadr]/m.actuator_gear[:,0]);mujoco.mj_integratePos(m,ref.qpos,sol.x,self.dt)
  self.q=np.array(self.q);self.v=np.array(self.v);self.ff=np.array(self.ff)
  if self.ref_collision or self.min_load<=0:raise RuntimeError('Invalid reference: collision or negative wheel load')
 def command(self,d):
  index=min(int(d.time/self.dt),len(self.q)-1);q=self.q[index];v=self.v[index]
  if d.time>=self.next_feedback-1e-9:
   bp,br,_,_=self.env.targets();R=d.xmat[self.env.base].reshape(3,3);angular=R.T@Rotation.from_matrix(br@R.T).as_rotvec();angular[2]=0
   delta=np.r_[bp-d.qpos[:3],angular];delta[0]=0
   J=[]
   for l in ['FL','FR','RL','RR']:
    b=int(self.m.geom_bodyid[self.m.geom('as2w_'+l+'_foot_collision_1').id]);j=np.zeros((3,self.m.nv));r=j.copy();mujoco.mj_jac(self.m,d,j,r,d.xpos[b]+[0,0,-.086],b);J.append(j)
   J=np.vstack(J);self.correction=np.clip(-3.0*np.linalg.pinv(J[:,self.leg_dof])@J[:,:6]@delta,-.12,.12);self.next_feedback=d.time+.02
  joint_target=q[self.qadr].copy();joint_target[self.leg_act]+=self.correction; joint_target=np.clip(joint_target,self.jlo,self.jhi)
  raw=self.ff[index]+self.kp*(joint_target-d.qpos[self.qadr])+self.kd*(v[self.vadr]-d.qvel[self.vadr])
  bp,br,_,_=self.env.targets();future=self.env.targets(d.time+.001);desired_v=(future[0]-bp)/.001
  yaw=Rotation.from_matrix(d.xmat[self.env.base].reshape(3,3)).as_euler('xyz')[2];yd=Rotation.from_matrix(br).as_euler('xyz')[2];yd_next=Rotation.from_matrix(future[1]).as_euler('xyz')[2];wd=(yd_next-yd)/.001
  xerr=np.array([np.cos(yaw),np.sin(yaw),0.])@(bp-d.qpos[:3]);verr=np.array([np.cos(yaw),np.sin(yaw),0.])@(desired_v-d.qvel[:3]);yawerr=np.arctan2(np.sin(yaw-yd),np.cos(yaw-yd))
  self.x_integral=np.clip(self.x_integral+xerr*.001,-.1,.1);self.yaw_integral=np.clip(self.yaw_integral+yawerr*.001,-.2,.2)
  straight=np.clip(100*xerr+14*verr+10*self.x_integral,-4,4);turn=np.clip(200*yawerr+8*(d.qvel[5]-wd)+20*self.yaw_integral,-6,6)
  for i in self.wheels:
   side=1 if '_FL_' in self.m.actuator(i).name or '_RL_' in self.m.actuator(i).name else -1
   raw[i]=self.ff[index,i]+self.kd[i]*(v[self.vadr[i]]-d.qvel[self.vadr[i]])+straight+side*turn
  return raw
