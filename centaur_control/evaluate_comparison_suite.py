"""Evaluate frozen suite; failed/missing trials stay in the denominator."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from .compare_foundation import read_trial, summarize, recovery_time
from .run_comparison_suite import protocol


def longest_run(mask,dt):
    best=current=0
    for flag in mask:
        current=current+1 if flag else 0
        best=max(best,current)
    return round(best*dt,6)


def evaluate(root):
    root=Path(root)
    spec=json.loads((root/'protocol.json').read_text())
    if spec!=protocol():raise ValueError('Unknown/modified protocol; use its matching evaluator')
    th=spec['thresholds'];results=[];cache={};identity=None
    for case in spec['cases']:
        folder=root/case['id'];row=dict(case_id=case['id'],task=case['task'],status='missing',pass_gates=False)
        if not (folder/'config.json').exists():results.append(row);continue
        meta=json.loads((folder/'config.json').read_text());row['status']=meta['status']
        signature=(meta['physics_hz'],meta['control_hz'],meta['mujoco_version'],meta['sha256'])
        if identity is None:identity=signature
        elif identity!=signature:raise ValueError('Mixed solver, frequencies, model or controller code in one suite')
        row['physics_hz']=meta['physics_hz'];row['wall_seconds']=meta['wall_seconds']
        if row['status']!='completed':
            row['failure_reasons']=[meta.get('error',row['status'])];results.append(row);continue
        a,_=read_trial(folder)
        records=[json.loads(s) for s in (folder/'commands.jsonl').read_text().splitlines()]
        t=a['t'];keep=t>=spec['discard_initial_s'];dt=1/meta['physics_hz']
        if not np.allclose(t,[r['t'] for r in records],atol=1e-8):raise ValueError('Mismatched trace/reference timestamps')
        critical=['t','roll_deg','pitch_deg','base_z','vx','yaw_rate','load','upper_rmse']
        if not all(np.isfinite(a[n]).all() for n in critical):
            row.update(status='nonfinite',failure_reasons=['nonfinite state/metric']);results.append(row);continue
        if len(t)!=round(meta['duration']/dt) or not np.allclose(np.diff(t),dt,atol=1e-8):raise ValueError('Missing/nonuniform samples')
        row.update(summarize(a))
        row['peak_attitude_error_deg']=float(max(abs(a['roll_error'][keep]).max(),abs(a['pitch_error'][keep]).max()))
        row['max_low_load_run_s']=longest_run(a['load'][keep]<th['wheel_load_threshold_N'],dt)
        util=np.array([r['peak_all_actuator_util'] for r in records])
        row['peak_all_actuator_util']=float(util[keep].max())
        row['max_saturation_run_s']=longest_run(util[keep]>=th['saturation_fraction'],dt)
        row['absolute_actuator_work_J']=float(sum(r['actuator_power_abs_W'] for r,k in zip(records,keep) if k)*dt)
        row['solver_warning_count']=sum(records[-1]['warnings'])
        row['min_leg_limit_margin_rad']=float(a['min_leg_limit_margin_rad'][keep].min())
        slip=np.stack([a[f'{leg}_slip_mps'] for leg in ['FL','FR','RL','RR']],axis=1)[keep]
        finite=slip[np.isfinite(slip)]
        row['contact_slip_rms_mps']=float(np.sqrt(np.mean(finite**2))) if len(finite) else None
        metrics={'height_error_rmse':'height_rmse_m','roll_error_rmse':'roll_rmse_deg',
                 'pitch_error_rmse':'pitch_rmse_deg','speed_error_rmse':'speed_rmse_mps',
                 'yaw_rate_error_rmse':'yaw_rate_rmse_radps','upper_rmse_rmse':'upper_joint_rmse_rad',
                 'peak_attitude_error_deg':'peak_attitude_error_deg',
                 'max_low_load_run_s':'max_low_load_run_s','max_saturation_run_s':'max_saturation_run_s'}
        reasons=[m for m,limit in metrics.items() if row[m]>th[limit]+1e-9]
        if row['nonwheel_contact_samples']:reasons.append('nonwheel_ground_contact')
        if row['solver_warning_count']:reasons.append('solver_warning')
        if row['min_leg_limit_margin_rad']<-.01:reasons.append('leg_joint_limit_violation')
        nominal=case['task']+'_nominal'
        if case['id']==nominal:cache[nominal]=a
        elif nominal in cache:
            b=cache[nominal]
            xy=np.hypot(a['x']-b['x'],a['y']-b['y'])
            row['peak_paired_xy_m']=float(xy[keep].max())
            if any(case['force']):
                near=(abs(a['roll_deg']-b['roll_deg'])<=th['recovery_angle_deg']) & (abs(a['pitch_deg']-b['pitch_deg'])<=th['recovery_angle_deg']) & (abs(a['base_z']-b['base_z'])<=th['recovery_height_m']) & (abs(a['vx']-b['vx'])<=th['recovery_speed_mps']) & (abs(a['yaw_rate']-b['yaw_rate'])<=th['recovery_yaw_radps'])
                end=10.5;window=(t>=10-1e-8)&(t<end-1e-8)
                row['left_recovery_band']=bool(np.any(~near[window]))
                rt=recovery_time(t,t>=end-1e-8,near,end,th['recovery_hold_s'])
                row['recovery_s']=rt
                row['recovery_with_xy_s']=recovery_time(t,t>=end-1e-8,near&(xy<=th['recovery_xy_m']),end,th['recovery_hold_s'])
                row['impulse_vector_Ns']=(np.array(case['force'])*.5).tolist()
                actual=np.array([np.sum([w['world_wrench'][:3] for w in r['wrenches']],axis=0) if r['wrenches'] else np.zeros(3) for r in records]).sum(axis=0)*dt
                if not np.allclose(actual,row['impulse_vector_Ns'],atol=1e-6):raise ValueError('Incorrect applied pulse impulse')
                if rt is None or rt>th['recovery_deadline_s']:reasons.append('recovery_deadline')
        else:reasons.append('missing_nominal_reference')
        row['failure_reasons']=reasons;row['pass_gates']=not reasons
        results.append(row)
    report=dict(protocol_version=spec['version'],expected_cases=len(spec['cases']),
                recorded_cases=sum(r['status']!='missing' for r in results),
                completed_cases=sum(r['status']=='completed' for r in results),
                passed_cases=sum(r['pass_gates'] for r in results),results=results)
    (root/'evaluation.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    columns=['case_id','task','status','pass_gates','failure_reasons','height_error_rmse','roll_error_rmse','pitch_error_rmse','speed_error_rmse','yaw_rate_error_rmse','upper_rmse_rmse','peak_attitude_error_deg','min_wheel_load_N','max_low_load_run_s','peak_all_actuator_util','max_saturation_run_s','recovery_s','recovery_with_xy_s','peak_paired_xy_m','absolute_actuator_work_J','contact_slip_rms_mps','solver_warning_count']
    with (root/'scores.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=columns,extrasaction='ignore');writer.writeheader();writer.writerows(results)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(3,1,figsize=(13,11),sharex=True)
    for ax,key,label in zip(axs,['peak_attitude_error_deg','min_wheel_load_N','height_error_rmse'],['Peak attitude error (deg)','Min wheel load (N)','Height RMSE (m)']):
        values=[r.get(key,np.nan) for r in results]
        ax.bar(range(len(results)),values,color=['#27866e' if r['pass_gates'] else '#c35a44' for r in results]);ax.set_ylabel(label);ax.grid(axis='y',alpha=.2)
    axs[-1].set_xticks(range(len(results)),[r['case_id'] for r in results],rotation=65,ha='right')
    fig.suptitle('Frozen foundation suite: green = all gates passed; red = failed gate')
    fig.tight_layout();fig.savefig(root/'overview.png',dpi=150);plt.close(fig)
    print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');args=p.parse_args();evaluate(args.folder)
