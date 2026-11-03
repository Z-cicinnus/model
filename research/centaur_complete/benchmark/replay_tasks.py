"""Replay recorded states; this is visualization, not another physics test."""
from pathlib import Path
import argparse,time
import numpy as np
import mujoco
import mujoco.viewer as mv
from env import ROOT
TASKS=['hold','dual_reach','posture_reach','straight_reach','turn_reach','disturbed_reach']
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--task',metavar='TASK',choices=TASKS+['all'],default='straight_reach',help='回放任务（'+'/'.join(TASKS+['all'])+'）；all 依次回放全部任务');a=p.parse_args()
 m=mujoco.MjModel.from_xml_path(str(ROOT/'centaur_self_collision.xml'));d=mujoco.MjData(m)
 for task in (TASKS if a.task=='all' else [a.task]):
  path=Path(__file__).parent/'results/task_qp_v1'/(task+'.npz');saved=np.load(path);print(task,flush=True)
  with mv.launch_passive(m,d) as view:
   view.cam.distance=2;view.cam.azimuth=140;view.cam.elevation=-15;view.cam.lookat[:]=[0,0,.5];view.opt.geomgroup[3]=0
   for q in saved['qpos']:
    if not view.is_running():break
    start=time.perf_counter();d.qpos[:]=q;mujoco.mj_forward(m,d);view.sync();time.sleep(max(0,.01-(time.perf_counter()-start)))