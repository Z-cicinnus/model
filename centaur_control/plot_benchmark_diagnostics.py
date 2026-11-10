"""Plot failed-case dynamics and physics-step sensitivity on a common time axis."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from .compare_foundation import read_trial

def plot(root,fine):
    root=Path(root)
    report=json.loads((root/'evaluation.json').read_text())
    for case in report['results']:
        if case['pass_gates'] or case['status']!='completed':continue
        fig,axs=plt.subplots(3,1,figsize=(10,9),sharex=True)
        for folder in [root,*map(Path,fine)]:
            trial=folder/case['case_id']
            if not (trial/'config.json').exists():continue
            a,c=read_trial(trial);t=a['t'];keep=(t>=8)&(t<=18)
            for ax,key in zip(axs,['pitch_error','load','peak_leg_wheel_util']):
                ax.plot(t[keep],a[key][keep],label=f"Physics {c['physics_hz']} Hz",lw=1.2)
        for ax,label in zip(axs,['Pitch tracking error (deg)','Min wheel load (N)','Peak leg/wheel utilization']):
            ax.set_ylabel(label);ax.grid(alpha=.2);ax.axvspan(10,10.5,color='orange',alpha=.2)
        axs[0].axhline(5,color='gray',ls='--',lw=.8);axs[0].axhline(-5,color='gray',ls='--',lw=.8)
        axs[1].axhline(1,color='gray',ls='--',lw=.8)
        axs[0].legend();axs[-1].set_xlabel('Simulation time (s)')
        fig.suptitle(case['case_id']+'; all controllers update at 100 Hz')
        fig.tight_layout();fig.savefig(root/(case['case_id']+'_detail.png'),dpi=150);plt.close(fig)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');p.add_argument('--fine',nargs='*',default=[]);a=p.parse_args();plot(a.folder,a.fine)
