"""Frozen foundation comparison suite; default physics/control 100 Hz."""
import argparse
from argparse import Namespace
import hashlib
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from .run_dynamic_task import run
from .run_foundation_tasks import FoundationTask
from .benchmark_foundation import pulse

HERE = Path(__file__).parent

def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def source_hashes():
    files=[HERE/f for f in ("scene.xml","centaur_unified.xml","run_dynamic_task.py","run_foundation_tasks.py","run_comparison_suite.py","benchmark_foundation.py")]
    xml=ET.parse(HERE/"centaur_unified.xml").getroot()
    meshdir=HERE/xml.find("compiler").get("meshdir", ".")
    files += [meshdir/m.get("file") for m in xml.findall("./asset/mesh")]
    return {str(f.relative_to(HERE)) if f.is_relative_to(HERE) else str(f):digest(f) for f in files}

def protocol():
    cases=[]
    for task in ('move','posture','upper'):
        cases.append(dict(id=task+'_nominal',task=task,force=[0,0,0],arm='both'))
    for task in ('move','posture'):
        for force in (40,120):
            for axis in (0,1):
                for sign in (-1,1):
                    vec=[0,0,0];vec[axis]=force*sign
                    cases.append(dict(id=f'{task}_{force}N_{"xy"[axis]}{"p" if sign>0 else "m"}',task=task,force=vec,arm='both'))
    for side in ('left','right'):
        cases.append(dict(id='upper_'+side,task='upper',force=[0,0,0],arm=side))
    return dict(version='foundation-v5.0',physics_hz=100,control_hz=100,
                angle_deg=5,lift_m=.02,height=None,discard_initial_s=4,
                duration_s=dict(move=54,posture=54,upper=72),
                pulse=dict(start_s=10,duration_s=.5,body='base_link',frame='world',point='body COM'),
                thresholds=dict(height_rmse_m=.01,roll_rmse_deg=1,pitch_rmse_deg=1,
                    speed_rmse_mps=.05,yaw_rate_rmse_radps=.03,upper_joint_rmse_rad=.1,
                    peak_attitude_error_deg=5,max_low_load_run_s=.1,
                    wheel_load_threshold_N=1,max_saturation_run_s=.2,saturation_fraction=.99,
                    recovery_angle_deg=.5,recovery_height_m=.005,recovery_speed_mps=.02,
                    recovery_yaw_radps=.02,recovery_xy_m=.02,recovery_hold_s=1,recovery_deadline_s=3),
                cases=cases)

class SuiteTask(FoundationTask):
    def update(self,m,d,unused):
        requested=super().update(m,d,unused)
        for name in self.joints:
            if self.args.arm != 'both' and not name.startswith(self.args.arm+'_'):
                self.joints[name]=self.nominal[name]
        if pulse(d.time,10,.5) and any(self.args.force):
            self.wrenches=[dict(body='base_link',world_wrench=[*self.args.force,0,0,0])]
        return requested

    def record(self,m,d,force):
        # Capture all actuator loads, including the upper body omitted by v4.
        limits=np.maximum(abs(m.actuator_forcerange).max(axis=1),1e-12)
        util=np.where(m.actuator_forcelimited,abs(d.actuator_force)/limits,0)
        actual={name:float(d.qpos[m.jnt_qposadr[m.actuator_trnid[self.actuators[name],0]]]) for name in self.joints}
        self.log.write(json.dumps(dict(t=float(d.time),command=self.command,height_ref=float(self.height),
            joints=self.joints,wrenches=self.wrenches,base_xyz=d.xpos[self.root].tolist(),joint_actual=actual,
            wheel_load=force.tolist(),peak_all_actuator_util=float(util.max()),
            actuator_power_abs_W=float(np.sum(abs(d.actuator_force*d.actuator_velocity))),
            warnings=[int(w.number) for w in d.warning]))+'\n')

def run_case(case,output,hz=100,view=False,speed=1):
    folder=Path(output)/case['id'];folder.mkdir(parents=True,exist_ok=True)
    if any(folder.iterdir()):raise FileExistsError(f'{folder}: use a fresh output or --resume')
    cfg=dict(case,angle=5,height=None,lift=.02,manual=None,replay=None)
    meta=dict(case,controller='centroidal_ik_baseline_v5',protocol_version='foundation-v5.0',
              physics_hz=hz,control_hz=100,mujoco_version=mujoco.__version__,status='running',
              duration=72 if case['task']=='upper' else 54,
              sha256=source_hashes())
    start=time.perf_counter()
    try:
        with (folder/'commands.jsonl').open('w') as f:
            run(0,folder/'trace.csv',duration=meta['duration'],diagnostics=True,
                task=SuiteTask(Namespace(**cfg),f),physics_hz=hz,view=view,playback_speed=speed)
        meta['status']='completed'
        with (folder/'commands.jsonl').open() as f:
            for line in f: last=json.loads(line)['t']
        meta['last_time']=last
        if last < meta['duration']-1/hz-1e-6:meta['status']='interrupted'
    except Exception as e:
        meta['status']='failed';meta['error']=repr(e)
    finally:
        meta['wall_seconds']=time.perf_counter()-start
        (folder/'config.json').write_text(json.dumps(meta,indent=2))
    print(case['id'],hz,meta['status'],round(meta['wall_seconds'],1),flush=True)
    return meta

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',default='comparison_baseline_v5')
    p.add_argument('--task',choices=['all','move','posture','upper'],default='all')
    p.add_argument('--case',help='Exact case ID from protocol.json')
    p.add_argument('--physics-hz',type=int,choices=[100,200,500],default=100)
    p.add_argument('--resume',action='store_true')
    p.add_argument('--view',action='store_true');p.add_argument('--speed',type=float,default=1)
    args=p.parse_args()
    if not np.isfinite(args.speed) or args.speed<=0:p.error('speed must be finite and positive')
    spec=protocol();root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    manifest=root/'protocol.json'
    if manifest.exists() and json.loads(manifest.read_text())!=spec:p.error('Protocol differs; use a fresh output')
    manifest.write_text(json.dumps(spec,indent=2))
    cases=[c for c in spec['cases'] if (args.task=='all' or c['task']==args.task) and (not args.case or c['id']==args.case)]
    if not cases:p.error('No matching case')
    for case in cases:
        config=root/case['id']/'config.json'
        if args.resume and config.exists():
            old=json.loads(config.read_text())
            expected=source_hashes()
            if old['physics_hz']!=args.physics_hz or old['sha256']!=expected:p.error('Resume configuration/code mismatch')
            print(case['id'],'retained',old['status'],flush=True);continue
        run_case(case,root,args.physics_hz,args.view,args.speed)

if __name__=='__main__':main()
