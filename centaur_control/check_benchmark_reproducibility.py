"""Numerical trajectory check for a repeated or refined-physics trial."""
import argparse
import json
from pathlib import Path
import numpy as np
from .compare_foundation import read_trial,summarize


def check(reference,candidate,output):
    reference,candidate,output=map(Path,(reference,candidate,output))
    if output.exists():raise FileExistsError('Use a fresh report path')
    a,ma=read_trial(reference);b,mb=read_trial(candidate)
    for key in ['id','task','arm','force','duration','control_hz','mujoco_version','sha256']:
        if ma[key]!=mb[key]:raise ValueError(f'Mismatched {key}')
    keep=a['t']>=4;difference={}
    for key in ['roll_deg','pitch_deg','base_z','vx','yaw_rate','x','y']:
        sampled=np.interp(a['t'],b['t'],b[key])
        difference[key]=float(np.max(abs(a[key][keep]-sampled[keep])))
    same=ma['physics_hz']==mb['physics_hz']
    report=dict(case_id=ma['id'],reference_hz=ma['physics_hz'],candidate_hz=mb['physics_hz'],
        check_type='deterministic_repeat' if same else 'physics_step_sensitivity',
        max_absolute_trajectory_difference=difference,reference_metrics=summarize(a),candidate_metrics=summarize(b))
    if same:report['trajectory_equal_at_1e_9']=all(v<=1e-9 for v in difference.values())
    else:
        report['interpretation']='Selected case only; differences do not establish convergence or a tipping boundary.'
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2,allow_nan=False));print(json.dumps(report,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('reference');p.add_argument('candidate');p.add_argument('--output',required=True);a=p.parse_args();check(a.reference,a.candidate,a.output)
