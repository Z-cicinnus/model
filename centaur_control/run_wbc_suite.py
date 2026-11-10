"""WBC adapter for the unchanged foundation-v5.0 task and evaluator."""
import argparse
from argparse import Namespace
import json
from pathlib import Path
import time
import mujoco
import numpy as np
import clarabel
import scipy
from .run_comparison_suite import SuiteTask, protocol, source_hashes, digest, HERE
from .run_dynamic_task import run
from .controllers.wbc import WholeBodyController


def hashes():
    h=source_hashes()
    for name in ('controllers/wbc.py','run_wbc_suite.py','models/state.py','controllers/pd.py'):
        h[name]=digest(HERE/name)
    return h


class WBCTask(SuiteTask):
    owns_control=True

    def setup(self,m,d,root):
        super().setup(m,d,root)
        self.controller=WholeBodyController(m,d)

    def update(self,m,d,unused):
        self.requested=super().update(m,d,unused)
        return self.requested

    def apply(self,m,d):
        # Preserve original force timing, then replace all actuator commands.
        # The QP does not read the temporary output of FoundationTask.apply.
        super().apply(m,d)
        try:
            self.controller.update(m,d,self.requested,self.height,self.joints)
        finally:
            self.qplog.write(json.dumps(dict(t=float(d.time),**self.controller.stats))+'\n')


def run_case(case,output,hz=100,view=False,speed=1):
    folder=Path(output)/case['id'];folder.mkdir(parents=True,exist_ok=True)
    if any(folder.iterdir()):raise FileExistsError(str(folder)+' is not empty')
    meta=dict(case,controller='whole_body_inverse_dynamics_qp_v1',protocol_version='foundation-v5.0',
              physics_hz=hz,control_hz=100,mujoco_version=mujoco.__version__,
              clarabel_version=clarabel.__version__,numpy_version=np.__version__,scipy_version=scipy.__version__,
              status='running',duration=72 if case['task']=='upper' else 54,sha256=hashes())
    start=time.perf_counter()
    try:
        with (folder/'commands.jsonl').open('w') as log, (folder/'wbc.jsonl').open('w') as qplog:
            task=WBCTask(Namespace(**dict(case,angle=5,height=None,lift=.02,manual=None,replay=None)),log)
            task.qplog=qplog
            run(0,folder/'trace.csv',duration=meta['duration'],diagnostics=True,task=task,
                physics_hz=hz,view=view,playback_speed=speed)
        meta['status']='completed'
        with (folder/'commands.jsonl').open() as log:
            for line in log:meta['last_time']=json.loads(line)['t']
        if meta.get('last_time',0)<meta['duration']-1/hz-1e-6:meta['status']='interrupted'
    except Exception as exc:
        meta.update(status='failed',error=repr(exc))
    finally:
        meta['wall_seconds']=time.perf_counter()-start
        (folder/'config.json').write_text(json.dumps(meta,indent=2))
    print(case['id'],hz,meta['status'],round(meta['wall_seconds'],1),meta.get('error',''),flush=True)
    return meta


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',default='comparison_wbc_v1')
    p.add_argument('--case');p.add_argument('--task',choices=['all','move','posture','upper'],default='all')
    p.add_argument('--physics-hz',type=int,choices=[100,200,500],default=100)
    p.add_argument('--view',action='store_true');p.add_argument('--speed',type=float,default=1)
    p.add_argument('--resume',action='store_true');args=p.parse_args()
    if not np.isfinite(args.speed) or args.speed<=0:p.error('speed must be finite and positive')
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True);spec=protocol()
    manifest=root/'protocol.json'
    if manifest.exists() and json.loads(manifest.read_text())!=spec:p.error('Protocol mismatch')
    manifest.write_text(json.dumps(spec,indent=2))
    cases=[c for c in spec['cases'] if (args.task=='all' or args.task==c['task']) and (not args.case or args.case==c['id'])]
    if not cases:p.error('No matching case')
    for case in cases:
        config=root/case['id']/'config.json'
        if args.resume and config.exists():
            old=json.loads(config.read_text())
            if old['sha256']!=hashes() or old['physics_hz']!=args.physics_hz or old['mujoco_version']!=mujoco.__version__ or old.get('clarabel_version')!=clarabel.__version__:
                p.error('Resume code/environment mismatch')
            print(case['id'],'retained',old['status'],flush=True);continue
        run_case(case,root,args.physics_hz,args.view,args.speed)

if __name__=='__main__':main()
