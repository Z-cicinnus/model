"""Compare completed paired trials; recovery means return near the no-pulse trajectory."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def recovery_time(t, eligible, within, end, hold=1.):
    dt=float(np.median(np.diff(t)))
    count=max(1,int(np.ceil(hold/dt)))
    good=np.asarray(eligible)&np.asarray(within)&(t>=end-1e-8)
    length=0
    for i, value in enumerate(good):
        length=length+1 if value else 0
        if length>=count:
            return round(max(0.,float(t[i-count+1]-end)),6)
    return None


def read_trial(folder):
    config=json.loads((folder/'config.json').read_text(encoding='utf-8'))
    if config['status']!='completed':raise ValueError(f'{folder}: incomplete trial')
    a=np.genfromtxt(folder/'trace.csv',delimiter=',',names=True)
    rows=[json.loads(s) for s in (folder/'commands.jsonl').read_text(encoding='utf-8').splitlines()]
    if len(a)!=len(rows):raise ValueError('Trace and reference logs have different lengths')
    a=dict((n,a[n]) for n in a.dtype.names)
    a['height_error']=a['base_z']-np.array([r['height_ref'] for r in rows])
    a['roll_error']=a['roll_deg']-a['roll_ref_deg']
    a['pitch_error']=a['pitch_deg']-a['pitch_ref_deg']
    a['speed_error']=a['vx']-a['vx_ref']
    a['yaw_rate_error']=a['yaw_rate']-a['yaw_ref']
    a['load']=np.min([a[f'{leg}_N'] for leg in ['FL','FR','RL','RR']],axis=0)
    a['upper_rmse']=np.array([np.sqrt(np.mean([(r['joint_actual'][n]-v)**2 for n,v in r['joints'].items()])) for r in rows])
    a['force_y']=np.array([sum(w['world_wrench'][1] for w in r['wrenches']) for r in rows])
    return a, config


def summarize(a):
    keep=a['t']>=4
    dt=float(np.median(np.diff(a['t'])))
    return {**{n+'_rmse':float(np.sqrt(np.mean(a[n][keep]**2))) for n in ['height_error','roll_error','pitch_error','speed_error','yaw_rate_error','upper_rmse']},'min_wheel_load_N':float(a['load'][keep].min()),'below_1N_duration_s':float(np.sum(a['load'][keep]<1)*dt),'nonwheel_contact_samples':int(np.sum(a['other_ground_contacts'][keep]>0)),'peak_leg_wheel_util':float(a['peak_leg_wheel_util'][keep].max())}


def compare(root):
    root=Path(root); report={}
    for task in ['move','posture','upper']:
        a,ca=read_trial(root/task/'baseline');b,cb=read_trial(root/task/'disturbed')
        if len(a['t'])!=len(b['t']) or not np.allclose(a['t'],b['t']):raise ValueError('Unmatched timestamps')
        for key in ['angle','height','lift','duration','mujoco_version','sha256','physics_hz','control_hz','timestep_s']:
            if ca[key]!=cb[key]:raise ValueError(f'Unmatched {key}')
        t=a['t']; keep=t>=4
        delta_xy=np.hypot(b['x']-a['x'],b['y']-a['y'])
        result={'baseline':summarize(a),'disturbed':summarize(b),'peak_xy_difference_m':float(delta_xy[keep].max()),'final_xy_difference_m':float(delta_xy[-1])}
        if task!='upper':
            end=cb['force_start']+cb['force_duration']
            near=(abs(b['roll_deg']-a['roll_deg'])<=.5)&(abs(b['pitch_deg']-a['pitch_deg'])<=.5)&(abs(b['base_z']-a['base_z'])<=.005)&(abs(b['vx']-a['vx'])<=.02)&(abs(b['yaw_rate']-a['yaw_rate'])<=.02)
            eligible=t>=end-1e-8
            result['left_recovery_tolerance_during_pulse']=bool(np.any(~near[(t>=cb['force_start']-1e-8)&(t<end-1e-8)]))
            result['paired_recovery_without_position_s']=recovery_time(t,eligible,near,end)
            result['paired_recovery_with_2cm_position_s']=recovery_time(t,eligible,near&(delta_xy<=.02),end)
            result['pulse_impulse_Ns']=float(np.sum(b['force_y'])*np.median(np.diff(t)))
        report[task]=result
        if task!='upper':
            fig,axs=plt.subplots(3,1,figsize=(10,7),sharex=True)
            window=(t>=cb['force_start']-2)&(t<=end+4)
            for ax,(values,label) in zip(axs,[(b['roll_deg']-a['roll_deg'],'Roll difference (deg)'),(b['pitch_deg']-a['pitch_deg'],'Pitch difference (deg)'),(delta_xy*1000,'XY difference (mm)')]):
                ax.plot(t[window],values[window]);ax.set_ylabel(label);ax.grid(alpha=.25);ax.axvspan(cb['force_start'],end,color='orange',alpha=.2)
            axs[-1].set_xlabel('Simulation time (s)');fig.suptitle(f'{task}: pulse response relative to baseline');fig.tight_layout();fig.savefig(root/f'{task}_pulse_detail.png',dpi=160);plt.close(fig)
        fig,axs=plt.subplots(3,2,figsize=(12,10),sharex=True)
        for ax,(key,label) in zip(axs.flat,[('speed_error','Speed error (m/s)'),('height_error','Height error (m)'),('roll_error','Roll error (deg)'),('pitch_error','Pitch error (deg)'),('load','Minimum wheel load (N)')]):
            ax.plot(t,a[key],label='Baseline',lw=1)
            ax.plot(t,b[key],label='Single left arm' if task=='upper' else f"{cb['force']:g} N pulse",lw=1)
            ax.set_ylabel(label);ax.grid(alpha=.25)
        axs[2,1].plot(t,delta_xy,label='XY difference',color='tab:purple');axs[2,1].set_ylabel('Paired XY difference (m)')
        for ax in axs.flat:
            ax.axvspan(0,4,color='gray',alpha=.08)
            if task!='upper':ax.axvspan(cb['force_start'],cb['force_start']+cb['force_duration'],color='orange',alpha=.2)
            ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.25)
        axs[0,0].legend();fig.suptitle(f'{task}: matched reference experiments');fig.tight_layout();fig.savefig(root/f'{task}_comparison.png',dpi=160);plt.close(fig)
    (root/'comparison.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps(report,indent=2))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder',nargs='?',default='foundation_benchmark');args=p.parse_args();compare(args.folder)
