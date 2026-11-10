"""Six matched foundation experiments: deterministic pulses and unilateral motion."""
import argparse
from argparse import Namespace
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from .run_dynamic_task import run
from .run_foundation_tasks import FoundationTask


def pulse(t, start, duration):
    # Quantization removes accumulated simulation-time error at pulse boundaries.
    return start <= round(float(t), 9) < start + duration


class Experiment(FoundationTask):
    def update(self, m, d, unused):
        requested = super().update(m, d, unused)
        if self.args.condition == 'disturbed':
            if self.args.task == 'upper':
                for name in list(self.joints):
                    if name.startswith('right_'):
                        self.joints[name] = self.nominal[name]
            elif pulse(d.time, self.args.force_start, self.args.force_duration):
                self.wrenches = [{'body': 'base_link', 'world_wrench': [0, self.args.force, 0, 0, 0, 0]}]
        return requested


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--task', choices=['move','posture','upper','all'], default='all')
    p.add_argument('--condition', choices=['baseline','disturbed','both'], default='both')
    p.add_argument('--view', action='store_true')
    p.add_argument('--speed', type=float, default=1)
    p.add_argument('--force', type=float, default=5, help='World +Y force in N; negative reverses direction')
    p.add_argument('--force-start', type=float, default=10)
    p.add_argument('--force-duration', type=float, default=.5)
    p.add_argument('--angle', type=float, default=5)
    p.add_argument('--height', type=float)
    p.add_argument('--lift', type=float, default=.02)
    p.add_argument('--output', default='foundation_benchmark')
    args=p.parse_args()
    if not np.isfinite([args.speed,args.force,args.force_start,args.force_duration,args.angle,args.lift]).all() or args.speed<=0 or args.force_start<4 or args.force_duration<=0 or args.force_start+args.force_duration>50 or not 0<=args.angle<=50 or not 0<=args.lift<=.15:
        p.error('Invalid parameters; pulse must lie in [4,50] simulation seconds')
    if args.height is not None and (not np.isfinite(args.height) or args.height<=0):
        p.error('height must be positive and finite')
    tasks=['move','posture','upper'] if args.task=='all' else [args.task]
    conditions=['baseline','disturbed'] if args.condition=='both' else [args.condition]
    for name in tasks:
        for condition in conditions:
            folder=Path(args.output)/name/condition
            folder.mkdir(parents=True,exist_ok=True)
            # Protect existing experiment records against accidental replacement.
            if (folder/'commands.jsonl').exists():
                p.error(f'{folder} already contains results; choose a new --output directory')
            config=Namespace(**vars(args));config.task=name;config.condition=condition
            config.manual=None;config.replay=None
            duration=72 if name=='upper' else 54
            metadata=vars(config).copy();metadata['duration']=duration
            metadata['mujoco_version']=mujoco.__version__
            metadata['physics_hz']=100
            metadata['control_hz']=100
            metadata['timestep_s']=.01
            metadata['sha256']={f:hashlib.sha256(Path(__file__).with_name(f).read_bytes()).hexdigest() for f in ['scene.xml','centaur_unified.xml','run_dynamic_task.py','run_foundation_tasks.py','benchmark_foundation.py']}
            metadata['status']='running'
            try:
                with (folder/'commands.jsonl').open('w',encoding='utf-8') as log:
                    experiment=Experiment(config,log)
                    run(0,folder/'trace.csv',view=args.view,diagnostics=True,duration=duration,task=experiment,playback_speed=args.speed)
                last=json.loads((folder/'commands.jsonl').read_text(encoding='utf-8').splitlines()[-1])['t']
                metadata['last_time']=last
                metadata['status']='completed' if last>=duration-.010001 else 'interrupted'
            except Exception as exc:
                metadata['status']='failed';metadata['error']=repr(exc)
                raise
            finally:
                (folder/'config.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
            print(name,condition,metadata['status'],flush=True)

if __name__=='__main__':main()
