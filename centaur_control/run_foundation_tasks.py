"""Three repeatable foundation tasks, using the existing centroidal/IK baseline."""
import argparse
import json
from pathlib import Path
from contextlib import ExitStack
import numpy as np
from scipy.spatial.transform import Rotation
from .run_dynamic_task import run


def blend(t):
    u = np.clip(t, 0, 1)
    return u*u*(3-2*u)


class FoundationTask:
    def __init__(self, args, log):
        self.args, self.log = args, log
        self.manual = {}
        self.previous_force = None
        self.replay = []
        if args.replay:
            with open(args.replay, encoding='utf-8') as f:
                self.replay = [json.loads(line) for line in f if line.strip()]
            times = [r['t'] for r in self.replay]
            if not times or any(b <= a for a,b in zip(times,times[1:])):
                raise ValueError('Replay timestamps must increase')
            self.times = times

    def setup(self, m, d, root):
        self.root = root
        self.xyz = d.xpos[root].copy()
        self.h0 = self.args.height if self.args.height is not None else self.xyz[2] - .09
        if not np.isfinite(self.h0) or self.h0 <= 0:
            raise ValueError('height must be finite and positive')
        self.height = self.xyz[2]
        self.actuators = {m.joint(int(j)).name: i for i,j in enumerate(m.actuator_trnid[:,0])}
        self.nominal = {name: float(d.qpos[m.jnt_qposadr[m.actuator_trnid[i,0]]]) for name,i in self.actuators.items()}
        self.previous_force = np.zeros_like(d.xfrc_applied)
        self.yaw0 = Rotation.from_matrix(d.xmat[root].reshape(3,3)).as_euler('xyz')[2]

    def preset(self, t):
        a = np.deg2rad(self.args.angle)
        z = self.args.lift
        # Each stage ramps for two seconds and holds for four.
        if self.args.task == 'move':
            stages = [(0,0,0,0,0,0,0),(.2,0,0,0,0,0,0),(0,0,0,0,0,0,0),(-.2,0,0,0,0,0,0),(0,0,0,0,0,0,0),(.05,.05,0,0,0,0,0),(0,0,0,0,0,0,0),(.05,-.05,0,0,0,0,0),(0,0,0,0,0,0,0)]
        elif self.args.task == 'posture':
            stages = [(0,0,0,0,0,0,0),(0,0,0,a,0,0,0),(0,0,0,0,0,0,0),(0,0,0,-a,0,0,0),(0,0,0,0,0,0,0),(0,0,-a,0,0,0,0),(0,0,0,0,0,0,0),(0,0,a,0,0,0,0),(0,0,0,0,0,0,0)]
        else:
            # height, shoulder pitch, shoulder abduction; exploratory amplitudes.
            stages = [(0,0,0,0,0,0,0),(0,0,0,0,z,0,0),(0,0,0,0,-z,0,0),(0,0,0,0,0,0,0),(0,0,0,0,0,-.4,0),(0,0,0,0,0,0,0),(0,0,0,0,0,0,.3),(0,0,0,0,0,0,0),(0,0,0,0,z,-.4,0),(0,0,0,0,0,0,0),(0,0,0,0,-z,0,.3),(0,0,0,0,0,0,0)]
        i = min(int(t/6),len(stages)-1)
        return np.array(stages[max(i-1,0)]) + blend((t-i*6)/2)*(np.array(stages[i])-stages[max(i-1,0)])

    def update(self, m, d, unused):
        v,w,r,p,z,reach,spread = self.preset(d.time)
        self.height = float(self.xyz[2] + blend(d.time/3)*(self.h0-self.xyz[2]) + z)
        self.joints = {f'{side}_shoulder_pitch_joint':reach for side in ('left','right')}
        self.joints.update(left_shoulder_roll_joint=spread,right_shoulder_roll_joint=-spread)
        self.wrenches = []
        if self.args.manual and int(round(d.time/m.opt.timestep)) % max(1, round(0.1 / m.opt.timestep)) == 0:
            try:
                self.manual = json.loads(Path(self.args.manual).read_text(encoding='utf-8-sig'))
            except (OSError, json.JSONDecodeError):
                pass  # Atomic saves preferred; retain last valid snapshot during editing.
        manual = self.manual
        self.command = [float(v),float(w),float(r),float(p)]
        if self.replay:
            idx = np.searchsorted(self.times,d.time+1e-8,side='right')-1
            if idx < 0 or d.time > self.times[-1]+m.opt.timestep:
                raise ValueError('Replay does not cover requested simulation time')
            row = self.replay[idx]
            self.command, self.height = row['command'], row['height_ref']
            self.joints, self.wrenches = row['joints'], row['wrenches']
        else:
            self.joints.update(manual.get('joints',{}))
            self.height = manual.get('height',self.height)
            self.wrenches = manual.get('wrenches',[])
            self.command = manual.get('command',self.command)
        if len(self.command)!=4 or not np.isfinite([*self.command,self.height]).all() or self.height<=0:
            raise ValueError('Invalid command or height')
        for name,value in self.joints.items():
            if name not in self.actuators or not np.isfinite(value):
                raise ValueError(f'Invalid joint target: {name}')
            j=m.actuator_trnid[self.actuators[name],0]
            if m.jnt_limited[j] and not m.jnt_range[j,0]<=value<=m.jnt_range[j,1]:
                raise ValueError(f'Joint target exceeds limit: {name}')
        desired = np.array(self.command,dtype=float)
        if self.args.task != 'move':
            yaw = Rotation.from_matrix(d.xmat[self.root].reshape(3,3)).as_euler('xyz')[2]
            forward = np.array([np.cos(yaw),np.sin(yaw)])
            desired[0] += np.clip(1.2*(self.xyz[:2]-d.xpos[self.root,:2])@forward,-.15,.15)
            desired[1] += np.clip(1.5*np.arctan2(np.sin(self.yaw0-yaw),np.cos(self.yaw0-yaw)),-.15,.15)
        return desired

    def apply(self,m,d):
        for name,value in self.joints.items():
            i=self.actuators[name]
            j=m.actuator_trnid[i,0]
            # Leg overrides deliberately supersede IK; wheel commands remain torque mode.
            if name.endswith('_foot'):
                raise ValueError('Use motion commands for wheels, not joint targets')
            bias=0 if name.startswith(('FL_','FR_','RL_','RR_')) else d.qfrc_bias[m.jnt_dofadr[j]]/m.actuator_gainprm[i,0]
            d.ctrl[i]=np.clip(value+bias,*m.actuator_ctrlrange[i])
        d.xfrc_applied[:] -= self.previous_force
        self.previous_force[:]=0
        for item in self.wrenches:
            body=m.body(item['body']).id
            wrench=np.asarray(item['world_wrench'],dtype=float)
            if body==0 or wrench.shape!=(6,) or not np.isfinite(wrench).all():
                raise ValueError('Wrench requires a body and six finite world-frame components')
            self.previous_force[body]+=wrench
        d.xfrc_applied[:]+=self.previous_force

    def record(self,m,d,force):
        actual={name:float(d.qpos[m.jnt_qposadr[m.actuator_trnid[self.actuators[name],0]]]) for name in self.joints}
        self.log.write(json.dumps(dict(t=float(d.time),command=self.command,height_ref=float(self.height),joints=self.joints,wrenches=self.wrenches,base_xyz=d.xpos[self.root].tolist(),joint_actual=actual,wheel_load=force.tolist()),ensure_ascii=False)+'\n')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task',choices=['move','posture','upper'],required=True)
    p.add_argument('--view',action='store_true')
    p.add_argument('--angle',type=float,default=5)
    p.add_argument('--height',type=float)
    p.add_argument('--lift',type=float,default=.02)
    p.add_argument('--duration',type=float)
    p.add_argument('--manual',help='Live JSON command file, reread every 0.1 simulation second')
    p.add_argument('--replay',help='Previously recorded commands.jsonl')
    p.add_argument('--output',default='foundation_results')
    p.add_argument('--speed', type=float, default=1.0, help='Playback speed')
    args=p.parse_args()
    if not np.isfinite([args.angle,args.lift]).all() or not 0<=args.angle<=50 or not 0<=args.lift<=.15:
        p.error('angle must be 0..50 degrees; lift 0..0.15 m (not a validated safe range)')
    if args.manual and args.replay:
        p.error('manual and replay are mutually exclusive')
    folder=Path(args.output);folder.mkdir(parents=True,exist_ok=True)
    path=folder/'commands.jsonl'
    if args.replay and Path(args.replay).resolve()==path.resolve():
        p.error('Replay output must use a different folder')
    with path.open('w',encoding='utf-8') as f:
        task=FoundationTask(args,f)
        run(0,folder/'trace.csv',view=args.view,diagnostics=True,duration=args.duration or (72 if args.task=='upper' else 54),task=task,playback_speed=args.speed)

if __name__=='__main__':
    main()
