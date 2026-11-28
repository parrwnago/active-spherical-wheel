"""I76 fixed route/phase plots from sealed outcomes; no new inference/physics."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,importlib.util,json,shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
BASE=Path(__file__).resolve().parents[2]
COLORS={'N':'#555555','T':'#0072b2'}
LABELS={'N':'Native','T':'Trajectory-facing heading'}
PHASES=(('m100',-1.),('p030',.3),('p090',.9))
ROUTES=('straight','arc','eight')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True)+'\n')
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--evaluation',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    evaluation=args.evaluation.resolve();out=args.output.resolve()
    cases=[f'{r}_{p}_{a}'for r in ROUTES for p,_ in PHASES for a in COLORS]
    files=[Path(__file__),BASE/'runtime/route.py',BASE/'protocol/protocol.json',BASE/'campaign/outputs_sealed.json',evaluation/'science.json',evaluation/'artifact_manifest.json']
    files += [evaluation/f'{c}_signals.npz' for c in cases]+[BASE/f'campaign/runs/{c}/precision.npz' for c in cases]
    bindings={str(p):dict(sha256=sha(p),bytes=p.stat().st_size)for p in files}
    out.mkdir(parents=True,exist_ok=False);save(out/'registered_before_plot.json',dict(files=bindings,utc=datetime.now(timezone.utc).isoformat(),no_new_physics=True,no_new_inference=True,descriptive_only=True))
    shutil.copy2(Path(__file__),out/'plot.py')
    spec=importlib.util.spec_from_file_location('_i76_plotroute',BASE/'runtime/route.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    science=json.loads((evaluation/'science.json').read_text())
    plt.rcParams.update({'font.size':9,'axes.spines.top':False,'axes.spines.right':False})
    with PdfPages(out/'tangent_routes_comparison.pdf') as pdf:
        for route_id in ROUTES:
            route=mod.make_route(route_id);holdstart=route.start_s+route.duration_s;end=route.total_duration_s
            fullref=np.array([route.origin+route.rotation@route.shape_derivatives(float(u))[0] for u in np.linspace(0,1,4097)])
            signals={};poses={}
            for phase,_ in PHASES:
                for policy in COLORS:
                    c=f'{route_id}_{phase}_{policy}'
                    with np.load(evaluation/f'{c}_signals.npz',allow_pickle=False) as z:signals[c]={k:z[k]for k in z.files}
                    with np.load(BASE/f'campaign/runs/{c}/precision.npz',allow_pickle=False)as z:poses[c]=z['exact_qpos'][:,:2].copy()
            fig,axes=plt.subplots(5,3,figsize=(13.5,14),constrained_layout=True)
            for col,(phase,angle) in enumerate(PHASES):
                ref=fullref-route.origin;axes[0,col].plot(ref[:,0],ref[:,1],color='black',ls='--',lw=2,label='Full planned path')
                axes[0,col].scatter(*ref[-1],marker='*',c='black',s=65,zorder=5)
                for policy,color in COLORS.items():
                    c=f'{route_id}_{phase}_{policy}';s=signals[c];xy=poses[c]-route.origin
                    axes[0,col].plot(xy[:,0],xy[:,1],color=color,lw=1.3,label=LABELS[policy])
                    axes[1,col].plot(s['time_s'],1000*s['timed_xy_error_m'],color=color,lw=1.)
                    axes[2,col].plot(s['time_s'],1000*s['geometric_cross_track_m'],color=color,lw=1.)
                    axes[3,col].plot(s['time_s'],s['heading_error_abs_deg'],color=color,lw=1.)
                    axes[4,col].plot(s['time_s'],s['raw_euler_heading_rate_error_rad_s'],color=color,lw=1.)
                axes[0,col].set(title=f'Start phase {angle:+.1f} rad',xlabel='World x from planned start [m]',ylabel='World y from planned start [m]');axes[0,col].set_aspect('equal',adjustable='datalim')
                for row,label in [(1,'Timed XY error [mm]'),(2,'Geometric cross-track [mm]'),(3,'Raw tangent heading error [deg]'),(4,'Euler heading-rate error [rad/s]')]:
                    ax=axes[row,col];ax.set(xlabel='Time [s]',ylabel=label,xlim=(0,end));ax.axvspan(holdstart,end,color='#eeeeee',zorder=-2);ax.axvline(3,color='#aaaaaa',ls=':',lw=.8)
                axes[3,col].hlines(10,holdstart,end,color='black',ls=':',lw=1)
                for row in range(5):axes[row,col].grid(alpha=.2)
            axes[0,0].legend(fontsize=8,loc='best')
            fig.suptitle(f'I-76 — {route_id}: native versus trajectory-facing heading\nGazebo / DART; matched starts, fixed full path; shaded = 10 s goal hold',fontsize=14)
            fig.savefig(out/f'{route_id}_motion.png',dpi=150);fig.savefig(out/f'{route_id}_motion.pdf');pdf.savefig(fig);plt.close(fig)
            fig,axes=plt.subplots(4,3,figsize=(13.5,10),sharex=True,constrained_layout=True)
            quantities=(('goal_distance_m',1000.,20.,'Goal distance [mm]'),('body_xy_speed_m_s',1.,.02,'Body XY speed [m/s]'),('world_z_angular_speed_abs_rad_s',1.,.05,'|Physical world-z rate| [rad/s]'),('reference_up_error_deg',1.,2.,'Raw-reference up error [deg]'))
            for col,(phase,angle)in enumerate(PHASES):
                axes[0,col].set_title(f'Start phase {angle:+.1f} rad')
                for row,(key,scale,bound,label)in enumerate(quantities):
                    ax=axes[row,col]
                    for policy,color in COLORS.items():
                        s=signals[f'{route_id}_{phase}_{policy}'];mask=s['time_s']>=holdstart-1e-9
                        ax.plot(s['time_s'][mask],scale*s[key][mask],color=color,lw=1.1,label=LABELS[policy])
                    ax.axhline(bound,color='black',ls=':',lw=1);ax.axvspan(end-2,end,color='#eeeeee',zorder=-2);ax.set(ylabel=label,xlim=(holdstart,end));ax.grid(alpha=.2)
                axes[-1,col].set_xlabel('Time [s]')
            axes[0,0].legend(fontsize=8)
            fig.suptitle(f'{route_id}: all four settling conditions must hold together\nDotted = criterion; shaded = required final 2 s; physical-state samples',fontsize=14)
            fig.savefig(out/f'{route_id}_goal_hold.png',dpi=150);fig.savefig(out/f'{route_id}_goal_hold.pdf');pdf.savefig(fig);plt.close(fig)
    if any(sha(Path(p))!=v['sha256']for p,v in bindings.items()):raise ValueError('Frozen plot input changed')
    save(out/'artifact_manifest.json',dict(files={p.name:dict(sha256=sha(p),bytes=p.stat().st_size)for p in sorted(out.iterdir())if p.is_file()},science_status=science['decision']['status']))
    print(json.dumps({'output':str(out),'status':science['decision']['status']}))
if __name__=='__main__':main()
