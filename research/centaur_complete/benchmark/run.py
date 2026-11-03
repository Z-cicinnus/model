from pathlib import Path
import sys,json,argparse
import numpy as np
from env import CentaurEnv,ROOT
sys.path.insert(0,str(ROOT))
from pose_feedback.controller import PoseController

def run(task):
 env=CentaurEnv(task);obs,info=env.reset(42);controller=PoseController(env.m,env.d)
 # Existing holding controller, intentionally unchanged; it does not follow benchmark task targets.
 while True:
  action=controller.command(env.d,False);obs,reward,terminated,truncated,info=env.step(action)
  if terminated or truncated:break
 out=Path(__file__).parent/'results'/'pose_hold_baseline';out.mkdir(parents=True,exist_ok=True)
 result=info['metrics'];result['policy']='existing_pose_hold_no_task_tracking';result['mujoco_version']=__import__('mujoco').__version__
 (out/(task+'.json')).write_text(json.dumps(result,ensure_ascii=False,indent=2))
 np.savez_compressed(out/(task+'.npz'),samples=np.array(env.rows)[::10],qpos=np.array(env.qhistory)[::10],action=np.array(env.actions)[::10],columns=['time','base_error_m','left_error_m','right_error_m','base_orientation_rad','left_orientation_rad','right_orientation_rad','reward'])
 print(task,'success',result['success'],'base rmse',result['rmse_base_m'],'hand rmse',result['rmse_hands_m'],flush=True)
 return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--task',default='all');a=p.parse_args();tasks=CentaurEnv().config['tasks']
 results=[run(t) for t in (tasks if a.task=='all' else [a.task])]
 out=Path(__file__).parent/'results'/'pose_hold_baseline';(out/'summary.json').write_text(json.dumps(results,indent=2))
