from pathlib import Path
import json,argparse,hashlib
import numpy as np
import mujoco
from env import CentaurEnv
from task_controller import TaskController

def run(task):
 e=CentaurEnv(task);e.reset(42);c=TaskController(e);out=Path(__file__).parent/'results/task_qp_v1';out.mkdir(parents=True,exist_ok=True)
 print(task,'reference ready',c.eq_error,flush=True)
 while True:
  _,_,term,trunc,info=e.step(c.command(e.d))
  if term or trunc:break
 result=info['metrics'];result.update(controller_sha256=hashlib.sha256((Path(__file__).parent/'task_controller.py').read_bytes()).hexdigest(),policy='whole_body_reference_qp_with_pd_and_wheel_feedback',mujoco_version=mujoco.__version__,qp_reference_internal_contacts=c.ref_collision,qp_max_constraint_residual=c.eq_error,minimum_reference_wheel_load_N=c.min_load,static_base_residual=c.ff_residual)
 (out/(task+'.json')).write_text(json.dumps(result,indent=2));np.savez_compressed(out/(task+'.npz'),samples=np.array(e.rows)[::10],qpos=np.array(e.qhistory)[::10],action=np.array(e.actions)[::10],reference_qpos=c.q,reference_qvel=c.v,reference_dt=c.dt)
 print(json.dumps(result),flush=True);return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--task',default='all');args=p.parse_args();tasks=CentaurEnv().config['tasks'];results=[run(t) for t in (tasks if args.task=='all' else [args.task])]
 out=Path(__file__).parent/'results/task_qp_v1';(out/('summary.json' if args.task=='all' else args.task+'_summary.json')).write_text(json.dumps(results,indent=2))
