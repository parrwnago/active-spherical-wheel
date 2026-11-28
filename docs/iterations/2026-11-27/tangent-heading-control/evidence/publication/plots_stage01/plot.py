"""I75 presentation plots from sealed science/signals; never reruns inference."""
from pathlib import Path
from datetime import datetime, timezone
import argparse, hashlib, importlib.util, json, shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

BASE=Path(__file__).resolve().parents[2]
COLORS={'N':'#555555','T':'#0072b2'}
LABELS={'N':'Native','T':'Trajectory-facing heading'}
PHASES=(('m100',-1.),('p030',.3),('p090',.9))


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,indent=2,sort_keys=True)+'\n')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--evaluation',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();evaluation=args.evaluation.resolve();output=args.output.resolve()
    inputs=[evaluation/'science.json',evaluation/'artifact_manifest.json',BASE/'campaign/outputs_sealed.json',
            BASE/'protocol/protocol.json',BASE/'runtime/route.py',Path(__file__)]
    inputs += [evaluation/f'{phase}_{policy}_signals.npz' for phase,_ in PHASES for policy in COLORS]
    inputs += [BASE/f'campaign/runs/{phase}_{policy}/precision.npz' for phase,_ in PHASES for policy in COLORS]
    bindings={str(path):dict(sha256=sha(path),bytes=path.stat().st_size) for path in inputs}
    output.mkdir(parents=True,exist_ok=False)
    save(output/'registered_before_plot.json',dict(utc=datetime.now(timezone.utc).isoformat(),
         files=bindings,no_new_physics=True,no_new_inference=True,descriptive_only=True))
    shutil.copy2(Path(__file__),output/'plot.py')
    science=json.loads((evaluation/'science.json').read_text())
    spec=importlib.util.spec_from_file_location('i75_plot_route',BASE/'runtime/route.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    path=module.SmoothSPath()
    full_reference=np.array([path(float(t))[0] for t in np.arange(9001)*.005])
    signals={};poses={}
    for phase,_ in PHASES:
        for policy in COLORS:
            case=f'{phase}_{policy}'
            with np.load(evaluation/f'{case}_signals.npz',allow_pickle=False) as z:
                signals[case]={k:z[k] for k in z.files}
            with np.load(BASE/f'campaign/runs/{case}/precision.npz',allow_pickle=False) as z:
                poses[case]=(z['exact_qpos'][:,:2].copy(),z['exact_reference_xy'].copy())
    plt.rcParams.update({'font.size':10,'axes.spines.top':False,'axes.spines.right':False})
    figure,axes=plt.subplots(4,3,figsize=(13.5,12),constrained_layout=True)
    for col,(phase,angle) in enumerate(PHASES):
        anchor=path.origin
        ref=full_reference-anchor
        axes[0,col].plot(ref[:,0],ref[:,1],color='black',ls='--',lw=2,label='Reference')
        axes[0,col].scatter(*ref[-1],marker='*',c='black',s=65,zorder=5)
        for policy,color in COLORS.items():
            case=f'{phase}_{policy}';s=signals[case];xy=poses[case][0]-anchor
            axes[0,col].plot(xy[:,0],xy[:,1],color=color,lw=1.5,label=LABELS[policy])
            axes[1,col].plot(s['time_s'],1000*s['timed_xy_error_m'],color=color,lw=1.1)
            axes[2,col].plot(s['time_s'],s['heading_error_abs_deg'],color=color,lw=1.1)
            axes[3,col].plot(s['time_s'],s['raw_euler_heading_rate_error_rad_s'],color=color,lw=1.1)
        axes[0,col].set(title=f'Start phase {angle:+.1f} rad',xlabel='World x from reference start [m]',ylabel='World y from reference start [m]')
        axes[0,col].set_aspect('equal',adjustable='datalim')
        axes[1,col].set(ylabel='Timed XY error [mm]',xlabel='Time [s]',xlim=(0,45))
        axes[2,col].set(ylabel='Raw tangent heading error [deg]',xlabel='Time [s]',xlim=(0,45))
        axes[2,col].hlines(10,35,45,color='black',ls=':',lw=1)
        axes[3,col].set(ylabel='Euler heading-rate tracking error [rad/s]',xlabel='Time [s]',xlim=(0,45))
        for row in (1,2,3):
            axes[row,col].axvspan(35,45,color='#eeeeee',zorder=-2)
            axes[row,col].axvline(3,color='#aaaaaa',ls=':',lw=.8)
            axes[row,col].axvline(35,color='#aaaaaa',ls=':',lw=.8)
        for row in range(4):axes[row,col].grid(alpha=.2)
    axes[0,0].legend(fontsize=8,loc='best')
    figure.suptitle('I-75 — native versus trajectory-facing heading\nGazebo / DART; shaded interval = 10 s goal hold',fontsize=15)
    figure.savefig(output/'motion_comparison.png',dpi=160)
    figure.savefig(output/'motion_comparison.pdf')
    holdfig,holdaxes=plt.subplots(4,3,figsize=(13.5,10),sharex=True,constrained_layout=True)
    quantities=(('goal_distance_m',1000.,20.,'Goal distance [mm]'),
                ('body_xy_speed_m_s',1.,.02,'Body XY speed [m/s]'),
                ('world_z_angular_speed_abs_rad_s',1.,.05,'|World vertical angular speed| [rad/s]'),
                ('reference_up_error_deg',1.,2.,'Raw-reference attitude error [deg]'))
    for col,(phase,angle) in enumerate(PHASES):
        holdaxes[0,col].set_title(f'Start phase {angle:+.1f} rad')
        for row,(key,scale,bound,label) in enumerate(quantities):
            ax=holdaxes[row,col]
            for policy,color in COLORS.items():
                s=signals[f'{phase}_{policy}'];mask=s['time_s']>=35-1e-9
                ax.plot(s['time_s'][mask],scale*s[key][mask],color=color,lw=1.2,label=LABELS[policy])
            ax.axhline(bound,color='black',ls=':',lw=1)
            ax.axvspan(43,45,color='#eeeeee',zorder=-2)
            ax.set(ylabel=label,xlim=(35,45));ax.grid(alpha=.2)
        holdaxes[-1,col].set_xlabel('Time [s]')
    holdaxes[0,0].legend(fontsize=8)
    holdfig.suptitle('Settling requires all four conditions together\nDotted: criterion; shaded: required final 2 s. These are physical-state samples.',fontsize=15)
    holdfig.savefig(output/'goal_hold.png',dpi=160);holdfig.savefig(output/'goal_hold.pdf')
    with PdfPages(output/'tangent_heading_comparison.pdf') as pdf:
        pdf.savefig(figure);pdf.savefig(holdfig)
    plt.close('all')
    if any(sha(Path(path))!=binding['sha256'] for path,binding in bindings.items()):
        raise ValueError('Plot inputs changed')
    save(output/'artifact_manifest.json',dict(files={p.name:dict(sha256=sha(p),bytes=p.stat().st_size)
         for p in sorted(output.iterdir()) if p.is_file()},science_status=science['decision']['status']))
    print(json.dumps({'output':str(output),'status':science['decision']['status']}))


if __name__=='__main__':main()
