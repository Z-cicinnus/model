"""Summarize quality metrics without assigning uncalibrated pass/fail thresholds."""
import argparse
import json
from pathlib import Path
import numpy as np

def summarize(folder):
    folder=Path(folder)
    data=np.genfromtxt(folder/'trace.csv',delimiter=',',names=True)
    records=[json.loads(line) for line in (folder/'commands.jsonl').read_text(encoding='utf-8').splitlines()]
    keep=data['t']>=4
    if not keep.any():raise ValueError('Need more than four seconds of data')
    xyz=np.array([r['base_xyz'] for r in records])
    ref=np.array([r['height_ref'] for r in records])
    height=xyz[:,2]-ref
    joint=np.array([r['joint_actual'][n]-v for r in records if r['t']>=4 for n,v in r['joints'].items()])
    result={'samples':len(data),'last_time_s':float(data['t'][-1]),'height_rmse_m':float(np.sqrt(np.mean(height[keep]**2))), 'height_peak_error_m':float(np.max(abs(height[keep]))),'max_xy_displacement_m':float(np.linalg.norm(xyz[keep,:2]-xyz[0,:2],axis=1).max()),'joint_rmse_rad':float(np.sqrt(np.mean(joint**2))),'min_wheel_load_N':float(min(data[f'{leg}_N'][keep].min() for leg in ['FL','FR','RL','RR'])),'max_nonwheel_ground_contacts':int(data['other_ground_contacts'][keep].max()),'peak_leg_wheel_util':float(data['peak_leg_wheel_util'][keep].max())}
    for actual,target in [('vx','vx_ref'),('yaw_rate','yaw_ref'),('roll_deg','roll_ref_deg'),('pitch_deg','pitch_ref_deg')]:
        result[actual+'_rmse']=float(np.sqrt(np.mean((data[actual][keep]-data[target][keep])**2)))
    (folder/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');a=p.parse_args();print(json.dumps(summarize(a.folder),indent=2))
