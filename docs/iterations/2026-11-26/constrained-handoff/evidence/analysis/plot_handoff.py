"""Scientific I56 comparison from the exact full-rate portable projections."""
from pathlib import Path
import argparse
from datetime import datetime, timezone
import hashlib
import json
import importlib.util
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def draw(stage, output):
    output.mkdir(parents=True,exist_ok=False)
    index=json.loads((stage/'portable_inputs.json').read_text())
    reducer_path=stage/index['sources']['reducer']
    spec=importlib.util.spec_from_file_location('_i56_plot_metrics',reducer_path)
    metrics=importlib.util.module_from_spec(spec);spec.loader.exec_module(metrics)
    inputs=[stage/'portable_inputs.json',stage/'report.json',reducer_path]+[stage/index['cases'][c]['projection'] for c in ('m100_B','m100_A','m100_H')]
    (output/'registered_before_render.json').write_text(json.dumps(dict(registered_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(Path(__file__)),input_sha256={str(p):sha(p) for p in inputs},
        scope='Plot only; source arrays kept at5ms. Lines connect every saved point; PDF rasterizes dense lines. No new controller/model/physics.'),indent=2)+'\n')
    fig,axes=plt.subplots(4,2,figsize=(12,13),layout='constrained')
    colors={'m100_B':'#2166ac','m100_A':'#6b6b6b','m100_H':'#b2182b'}
    labels={'m100_B':'B: native LQR','m100_A':'A: bounded, no yaw/momentum','m100_H':'H: full handoff'}
    for case in ('m100_B','m100_A','m100_H'):
        d=metrics.load(stage/index['cases'][case]['projection']);t=d['time_s'];sig=metrics.signals(d)
        q=d['exact_qpos'];r=d['exact_reference_qpos']
        up=metrics.rotation(q[:,3:7])[:,:,2];rup=metrics.rotation(r[:,3:7])[:,:,2]
        tilt=np.degrees(np.arccos(np.clip(up[:,2],-1,1)))
        lean=np.degrees(np.arccos(np.clip(np.sum(up*rup,axis=1),-1,1)))
        a=dict(color=colors[case],lw=.9,alpha=.9,rasterized=True)
        axes[0,0].plot(q[:,0],q[:,1],label=labels[case],**a)
        axes[0,1].plot(t,1000*np.linalg.norm(q[:,:2]-d['exact_reference_xy'],axis=1),**a)
        axes[1,0].plot(t,tilt,**a);axes[1,1].plot(t,lean,**a)
        axes[2,0].plot(t,sig['world_omega_rad_s'][:,2],**a)
        h=sig['heading_rad'];valid=sig['heading_valid'];incr=np.where(valid[1:]&valid[:-1],abs(np.diff(h)),0)
        axes[2,1].plot(t,np.r_[0,np.cumsum(incr)]/(2*np.pi),**a)
        axes[3,0].plot(t[:-1],np.max(abs(d['exact_applied'][:-1])/d['exact_input_bounds'][:-1],axis=1),**a)
        dt=np.r_[.005,np.diff(t)]
        slew=np.array([60,60,60,6,6])
        normalized=np.max(abs(d['exact_applied']-d['exact_previous_applied'])/(dt[:,None]*slew),axis=1)
        axes[3,1].plot(t[:-1],normalized[:-1],**a)
        if case=='m100_B':
            axes[0,0].plot(d['exact_reference_xy'][:,0],d['exact_reference_xy'][:,1],color='black',ls='--',lw=1,label='Timed reference')
    axes[0,0].set(xlabel='World x (m)',ylabel='World y (m)',title='Measured route');axes[0,0].set_aspect('equal',adjustable='datalim');axes[0,0].legend(fontsize=8)
    axes[0,1].set(ylabel='Timed position error (mm)',title='Position versus current timed reference')
    axes[1,0].set(ylabel='Absolute tilt (deg)',title='Body up relative to gravity')
    axes[1,1].set(ylabel='Raw-reference lean error (deg)',title='Body up relative to native observer target')
    axes[2,0].set(ylabel='World vertical angular rate (rad/s)',title='Measured body angular velocity projected to world Z')
    axes[2,1].set(ylabel='Absolute heading travel (turns)',title='Cumulative |unwrapped heading increment|')
    axes[3,0].set(ylabel='max |actual command| / cap',title='Total final actual command, held samples')
    axes[3,1].set(ylabel='max |actual command step| / slew allowance',title='Actual command rates versus declared slew limits')
    axes[3,0].axhline(1,color='black',ls=':',lw=.8);axes[3,1].axhline(1,color='black',ls=':',lw=.8)
    for i,ax in enumerate(axes.flat):
        ax.grid(alpha=.2)
        if i:
            ax.set_xlim(0,44);ax.set_xlabel('Simulation time (s)');ax.axvspan(28,44,color='black',alpha=.035)
    fig.suptitle('I-56 | Mars figure-eight, initial phase −1.0 rad | 44 s, same frozen LQR gains\nHard command feasibility did not ensure better balance or motion',fontsize=13)
    fig.savefig(output/'comparison.png',dpi=170)
    fig.savefig(output/'comparison.pdf',dpi=170)
    plt.close(fig)
    (output/'artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=p.name,bytes=p.stat().st_size,sha256=sha(p)) for p in output.iterdir() if p.is_file()]),indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args();draw(a.data_source.resolve(),a.output.resolve())
