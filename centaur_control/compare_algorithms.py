"""Join two evaluated suites after checking the frozen physical experiment."""
import argparse
import csv
import json
from pathlib import Path
from .evaluate_comparison_suite import evaluate


def compare(left,right,output):
    left,right,output=map(Path,(left,right,output))
    if output.exists():raise FileExistsError('Choose a fresh comparison CSV')
    pa=json.loads((left/'protocol.json').read_text());pb=json.loads((right/'protocol.json').read_text())
    if pa!=pb:raise ValueError('Different protocols cannot be compared')
    a=evaluate(left);b=evaluate(right)
    if min(a['recorded_cases'],b['recorded_cases'])!=a['expected_cases']:
        raise ValueError('Both algorithms must attempt every frozen case; failed attempts count')
    rows=[]
    keys=['height_error_rmse','roll_error_rmse','pitch_error_rmse','speed_error_rmse',
          'yaw_rate_error_rmse','upper_rmse_rmse','peak_attitude_error_deg',
          'min_wheel_load_N','max_low_load_run_s','recovery_s','peak_paired_xy_m',
          'absolute_actuator_work_J','contact_slip_rms_mps']
    for x,y in zip(a['results'],b['results']):
        case=x['case_id'];ma=json.loads((left/case/'config.json').read_text());mb=json.loads((right/case/'config.json').read_text())
        for key in ['id','task','force','arm','duration','physics_hz','control_hz','mujoco_version']:
            if ma[key]!=mb[key]:raise ValueError(f'{case}: mismatched {key}')
        ha={k.replace('\\','/'):v for k,v in ma['sha256'].items()};hb={k.replace('\\','/'):v for k,v in mb['sha256'].items()}
        # Controller implementation may differ; task generator and physical assets may not.
        frozen=[k for k in ha if not k.endswith('.py') or k in ['run_foundation_tasks.py','run_comparison_suite.py','benchmark_foundation.py']]
        if any(ha[k]!=hb.get(k) for k in frozen):raise ValueError('Changed model/assets/task generator')
        row=dict(case_id=case,A_status=x['status'],B_status=y['status'],A_pass=x['pass_gates'],B_pass=y['pass_gates'])
        for k in keys:row.update({f'A_{k}':x.get(k),f'B_{k}':y.get(k)})
        rows.append(row)
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    print(f"A {a['passed_cases']}/{a['expected_cases']}; B {b['passed_cases']}/{b['expected_cases']}; {output}")

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('baseline');p.add_argument('candidate');p.add_argument('--output',default='algorithm_comparison.csv');args=p.parse_args();compare(args.baseline,args.candidate,args.output)
