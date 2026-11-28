"""I78 all-three-phase publication from closed, sealed canonical arrays only."""
from datetime import datetime,timezone
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
PHASES=(-1.,.3,.9)
LABELS={-1.:'−1.0 rad',.3:'+0.3 rad',.9:'+0.9 rad'}
COLORS=dict(N='#315da8',G='#128774')

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p,v):
    with Path(p).open('x')as f:json.dump(v,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def load_fields(p,keys):
    with np.load(p,allow_pickle=False)as z:return{k:z[k].copy()for k in keys}
def fmt(x,scale=1.,digits=1):return 'incomplete'if x is None else f'{scale*x:.{digits}f}'

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--evaluation',type=Path,required=True)
    ap.add_argument('--protocol',type=Path,default=BASE/'protocol/protocol.json')
    ap.add_argument('--output',type=Path,default=BASE/'evaluation/publication_stage01')
    args=ap.parse_args();evaluation=args.evaluation.resolve();out=args.output.resolve()
    report_path=evaluation/'report.json';report=json.loads(report_path.read_text())
    protocol=json.loads(args.protocol.read_text());base=args.protocol.resolve().parent.parent
    if report['iteration']!='I-78' or report.get('canonical_analysis_stage')!='stage02':
        raise ValueError('Canonical stage02 evaluation required')
    jobs=protocol['cases']
    if {(j['phase_rad'],j['policy'])for j in jobs}!={(s,p)for s in PHASES for p in ('N','G')}:
        raise ValueError('All six declared cases must be published')
    paths={};inputs=[Path(__file__).resolve(),report_path,evaluation/'artifact_manifest.json',args.protocol.resolve()]
    for j in jobs:
        cid=j['case_id'];case=base/protocol['run_root']/cid;native=cid.rsplit('_',1)[0]+'_N'
        paths[cid]=dict(precision=case/'child/campaign/runs'/native/'precision.npz',
            sidecar=case/'governor_diagnostics.npz',signals=evaluation/(cid+'_signals.npz'))
        inputs.extend(paths[cid].values())
    bindings={str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in inputs}
    out.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),out/'plot_comparison.py')
    write(out/'registered_before_arrays.json',dict(iteration='I-78',utc=datetime.now(timezone.utc).isoformat(),
        files=bindings,no_new_arrays_read_before_registration=True,no_new_physics=True,
        plot_plan='Allthreephases, wholeprimaryepisodes. Sharedgeometricpath/goal; physicalXYanderrorversusreferenceprogress, physicalclock/progress/rate/clipping. No outcomeselectedlocalwindows.',
        canonical_decision_unchanged=True))
    data={}
    for j in jobs:
        cid=j['case_id'];p=paths[cid]
        physical=load_fields(p['precision'],['time_s','exact_qpos'])
        side=load_fields(p['sidecar'],['time_s','gov__clock_rate','gov__raw_target_rate'])
        signals=load_fields(p['signals'],['time_s','progress','original_progress','geometric_error_m',
            'normalized_clipping_excess','primary_episode_mask','traversal_mask'])
        np.testing.assert_array_equal(physical['time_s'],side['time_s'])
        np.testing.assert_array_equal(physical['time_s'],signals['time_s'])
        data[(j['phase_rad'],j['policy'])]=dict(physical=physical,side=side,signals=signals,row=report['cases'][cid])
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages
    angle=np.deg2rad(113.);origin=np.array([-2.4,10.]);goal=origin+1.2*np.array([np.cos(angle),np.sin(angle)])
    all_xy=np.concatenate([d['physical']['exact_qpos'][d['signals']['primary_episode_mask'],:2]for d in data.values()])
    lo=np.minimum(all_xy.min(axis=0),np.minimum(origin,goal))-.04
    hi=np.maximum(all_xy.max(axis=0),np.maximum(origin,goal))+.04
    figs=[]
    fig,axes=plt.subplots(2,3,figsize=(15,10),constrained_layout=True)
    for col,phase in enumerate(PHASES):
        ax,er=axes[:,col]
        ax.plot([origin[0],goal[0]],[origin[1],goal[1]],':',color='#555555',label='Planned path')
        ax.scatter(*goal,marker='x',color='k',s=25,zorder=5)
        for policy in ('N','G'):
            d=data[(phase,policy)];s=d['signals'];m=s['primary_episode_mask'];xy=d['physical']['exact_qpos'][m,:2]
            name='Native N'if policy=='N'else'Progress G';r=d['row']
            ax.plot(xy[:,0],xy[:,1],color=COLORS[policy],label=name,lw=1.4)
            ax.scatter(xy[-1,0],xy[-1,1],color=COLORS[policy],marker='s',s=22)
            er.plot(s['progress'][s['traversal_mask']],1000*s['geometric_error_m'][s['traversal_mask']],
                color=COLORS[policy],label=name,lw=1.3)
        ax.set_title('Starting phase '+LABELS[phase]);ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_aspect('equal')
        ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)');ax.grid(alpha=.2)
        er.set_xlabel('Reference geometric progress u');er.set_ylabel('Distance to finite planned path (mm)');er.set_xlim(0,1);er.grid(alpha=.2)
        n,g=(data[(phase,p)]['row']for p in ('N','G'))
        er.set_title('Peak N/G: '+fmt(n['tracking']['geometric_traversal']['maximum'],1000)+' / '+fmt(g['tracking']['geometric_traversal']['maximum'],1000)+' mm')
        note='Reference arrival N/G: '+fmt(n['declared_reference_arrival_s'],digits=3)+' / '+fmt(g['declared_reference_arrival_s'],digits=3)+' s'
        ax.text(.02,.02,note,transform=ax.transAxes,fontsize=8,bbox=dict(facecolor='white',alpha=.8,edgecolor='none'))
        if col==0:ax.legend(fontsize=8);er.legend(fontsize=8)
    fig.suptitle('I-78: same geometric straight path, managed progress versus original clock\n'+report['decision']['status']+' — no automatic controller adoption',fontsize=15)
    fig.supxlabel('Full primary episodes shown. Squares: episode end. Error curves retain repeated progress during pauses; time RMS, stalls and goal settling remain separate checks.',fontsize=9)
    fig.savefig(out/'motion_comparison.png',dpi=150);figs.append(fig)
    fig2,axes=plt.subplots(3,3,figsize=(15,10),sharex='col',constrained_layout=True)
    for col,phase in enumerate(PHASES):
        top,rate,clip=axes[:,col]
        for policy in ('N','G'):
            d=data[(phase,policy)];s=d['signals'];t=s['time_s'];m=s['primary_episode_mask'];r=d['row']
            name='Native N'if policy=='N'else'Progress G'
            top.plot(t[m],s['progress'][m],color=COLORS[policy],label=name)
            clip.plot(t[m],s['normalized_clipping_excess'][m],color=COLORS[policy],label=name,lw=1.)
            arrival=r['declared_reference_arrival_s']
            if arrival is not None:top.axvline(arrival,color=COLORS[policy],ls=':',lw=.8)
        d=data[(phase,'G')];s=d['signals'];t=s['time_s'];m=s['traversal_mask'];arrival=d['row']['declared_reference_arrival_s']
        if arrival is not None:m=m&(t<arrival-1e-9)
        rate.plot(t[m],d['side']['gov__clock_rate'][m],color=COLORS['G'],label='Actual clock rate λ')
        rate.plot(t[m],d['side']['gov__raw_target_rate'][m],color='#cc8933',ls='--',label='Observed demand target')
        top.set_title('Starting phase '+LABELS[phase]);top.set_ylabel('Path progress u');top.set_ylim(-.02,1.04)
        rate.set_ylabel('Clock rate / target');rate.set_ylim(-.02,1.04)
        clip.set_ylabel('Normalized requested clipping excess');clip.set_xlabel('Physical simulation time (s)')
        for ax in axes[:,col]:ax.grid(alpha=.2)
        if col==0:top.legend(fontsize=8);rate.legend(fontsize=8);clip.legend(fontsize=8)
    fig2.suptitle('I-78: physical time, online progress response and actual command demand',fontsize=15)
    fig2.supxlabel('Rate panel excludes the completed goal hold. Clipping shows final total native requests; the governor does not certify future feasibility or create missing drive authority.',fontsize=9)
    fig2.savefig(out/'timing_commands.png',dpi=150);figs.append(fig2)
    with PdfPages(out/'progress_comparison.pdf')as pdf:
        for f in figs:pdf.savefig(f)
    for f in figs:plt.close(f)
    lines=['# I-78 physical progress comparison','',report['decision']['status'],
           '', '| Phase | Policy | Reference arrival (s) | Added arrival time (s) | Geometric peak (mm) | Progress RMS (mm) | Physical goal convergence |',
           '| --- | --- | ---: | ---: | ---: | ---: | --- |']
    for phase in PHASES:
        for policy in ('N','G'):
            r=data[(phase,policy)]['row'];lines.append('| '+LABELS[phase]+' | '+policy+' | '+fmt(r['declared_reference_arrival_s'],digits=3)+' | '+fmt(r['arrival_added_time_s'],digits=3)+' | '+fmt(r['tracking']['geometric_traversal']['maximum'],1000)+' | '+fmt(r['tracking']['geometric_progress_rms']['rms'],1000)+' | '+str(r['settling']['physical_position_passed'])+' |')
    lines+=['','Original-clock tracking is descriptive schedule lag. Each episode uses its first reference arrival plus 10 s; no later easy tail or incomplete route is substituted. All three phases remain in the frozen screen.']
    (out/'comparison.md').write_text('\n'.join(lines)+'\n')
    for rel,bound in bindings.items():
        if sha(ROOT/rel)!=bound['sha256']:raise AssertionError('Source/input changed during publication: '+rel)
    write(out/'artifact_manifest.json',dict(files={str(p.relative_to(out)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in sorted(out.rglob('*'))if p.is_file()}))
    print(json.dumps(dict(completed=True,no_new_physics=True,all_three_phases=True,status=report['decision']['status'],outputs=['motion_comparison.png','timing_commands.png','progress_comparison.pdf','comparison.md'])),flush=True)

if __name__=='__main__':main()
