"""I77 aligned contact-wrench/geometry diagnostics; no physics or tuning."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import numpy as np
from spherical_wheel.contact_sensing import intrinsic_normal

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
RADIUS = .12

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write(p, value):
    with Path(p).open('x') as f:
        json.dump(value, f, indent=2, sort_keys=True, allow_nan=False)
        f.write('\n')
def unit(v):
    n = float(np.linalg.norm(v))
    return np.asarray(v)/n if n>1e-12 else np.full(3, np.nan)
def angle(a,b):
    if a is None or b is None: return None
    if not np.isfinite(np.r_[a,b]).all(): return None
    return float(np.degrees(np.arccos(np.clip(unit(a)@unit(b),-1.,1.))))
def clean(x):
    if isinstance(x,dict): return {k:clean(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)): return [clean(v) for v in x]
    if isinstance(x,np.ndarray): return clean(x.tolist())
    if isinstance(x,np.generic): return clean(x.item())
    if isinstance(x,float) and not np.isfinite(x): return None
    return x
def load(p):
    with np.load(p,allow_pickle=False) as z: return {k:z[k].copy() for k in z.files}
def up(quat):
    w,x,y,z=quat
    return np.array([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    case=args.run.resolve(); out=args.output.resolve()
    trace=case/'substeps.npz'
    precision=case/'child/campaign/runs/eight_m100_N/precision.npz'
    rollout=case/'child/campaign/runs/eight_m100_N/rollout.npz'
    parity=case/'parity_receipt.json'
    if not json.loads(parity.read_text())['passed']: raise ValueError('Native parity required')
    paths=[Path(__file__).resolve(),trace,precision,rollout,parity,
           ROOT/'src/spherical_wheel/contact_sensing.py',ROOT/'config/model.yaml']
    bindings={str(p.relative_to(ROOT)):sha(p) for p in paths}
    out.mkdir(parents=True,exist_ok=False)
    shutil.copy2(Path(__file__),out/'analyze_trace.py')
    write(out/'registered_before_arithmetic.json',dict(iteration='I-77',
        utc=datetime.now(timezone.utc).isoformat(),inputs=bindings,radius_m=RADIUS,
        no_new_physics=True,no_control_changes=True,cutoff_rad_s=100.,
        interval='Five completed1ms solves following a held command are matched to the NEXT5ms observer update.',
        signed_wrench='Sum every exported raw force; retain admitted-only and native-contact-filter sums separately. Torque about wheel centre from pre-solve pose, with post-solve sensitivity.',
        geometry='Pressure-weighted exported normals and centre-to-pressure-point directions are descriptive aggregates, not unique terrain normals.',
        diagnostic_filter='Physical interval wrench follows the same100rad/s filter, anchored to saved native filtered wrench at96s; no fitted delay/parameters.',
        selected_times_s=[96.15,96.2,96.35,96.4,96.5,96.8,97.,97.5],
        limitations=['Exported solved forces are sampled forces, not a solver-impulse API.','Moment centre/frame quadrature and model mismatch can affect observer comparison.','Observed target motion and matched wrenches do not independently establish its physical causal contribution.']))
    s,p,r=load(trace),load(precision),load(rollout)
    np.testing.assert_array_equal(r['time_s'],p['time_s'].astype(np.float32))
    if len(s['time_s'])!=1500 or not np.all(s['hold_completed']):
        raise AssertionError('Expected 1500 samples from completed holds')
    if sorted(set(s['hold_index'].tolist()))!=list(range(19200,19500)):
        raise AssertionError('The diagnostic filter requires contiguous holds96.000–97.500s')
    for hold in range(19200,19500):
        ids=np.flatnonzero(s['hold_index']==hold)
        np.testing.assert_array_equal(s['held_input'][ids],np.tile(p['exact_applied'][hold],(5,1)))
        np.testing.assert_array_equal(p['exact_previous_applied'][hold+1],p['exact_applied'][hold])
    sf={name:i for i,name in enumerate(s['state_fields'])}
    assert sf['x']==2 and sf['z']==4
    rows=[]; sub=[]
    for j,t in enumerate(s['time_s']):
        c=s['contact_rows'][s['contact_offsets'][j]:s['contact_offsets'][j+1]]
        pos=c[:,1:4]; n=c[:,4:7]; f=c[:,7:10]
        fn=np.einsum('ij,ij->i',n,f)
        admitted=c[:,12]>=.5
        native=admitted&((c[:,10]>0.)|(fn>0.))
        pre=s['bridge_state_before'][j,2:5]; post=s['bridge_state'][j,2:5]
        positive=admitted&(fn>1e-9)
        F=f.sum(axis=0); M=np.cross(pos-pre,f).sum(axis=0)
        Mpost=np.cross(pos-post,f).sum(axis=0)
        P=float(fn[positive].sum())
        normal_sum=(fn[positive,None]*n[positive]).sum(axis=0)
        radial_sum=(fn[positive,None]*(pre-pos[positive])).sum(axis=0)
        sub.append(dict(time_s=float(t),hold_index=int(s['hold_index'][j]),
            substep_index=int(s['substep_index'][j]),force=F,moment=M,moment_post=Mpost,
            native_force=f[native].sum(axis=0),native_moment=np.cross(pos[native]-pre,f[native]).sum(axis=0),
            admitted_force=f[admitted].sum(axis=0),rejected_force=f[~admitted].sum(axis=0),
            positive_pressure_N=P,normal_sum=normal_sum,radial_sum=radial_sum,
            low_normal_force=f[admitted&(fn<=1e-9)].sum(axis=0),
            pressure_family_N=np.array([fn[positive&(c[:,0]==k)].sum() for k in range(7)])))
    times=p['time_s']; first=int(round(96./.005))
    physical_filtered=p['exact_observer_wrench'][first].copy()
    reference_up=np.array([up(q) for q in p['exact_reference_quat']])
    target_rate=np.r_[0.,np.degrees(np.arccos(np.clip(np.sum(reference_up[1:]*reference_up[:-1],axis=1),-1,1)))/.005]
    for hold in sorted(set(x['hold_index'] for x in sub)):
        a=[x for x in sub if x['hold_index']==hold]
        if len(a)!=5 or [x['substep_index'] for x in a]!=[1,2,3,4,5]: raise AssertionError('Incomplete held interval')
        i=hold+1
        if abs(float(times[i])-a[-1]['time_s'])>1e-8: raise AssertionError('Interval endpoint mismatch')
        F=np.mean([x['force'] for x in a],axis=0); M=np.mean([x['moment'] for x in a],axis=0)
        Mpost=np.mean([x['moment_post'] for x in a],axis=0)
        W=np.r_[F,M]
        physical_filtered+=(1-np.exp(-100*.005))*(W-physical_filtered)
        raw_n=intrinsic_normal(F,M,RADIUS)
        post_n=intrinsic_normal(F,Mpost,RADIUS)
        filtered_n=intrinsic_normal(physical_filtered[:3],physical_filtered[3:],RADIUS)
        observer_n=intrinsic_normal(p['exact_observer_wrench'][i,:3],p['exact_observer_wrench'][i,3:],RADIUS)
        pressure_n=unit(np.sum([x['normal_sum'] for x in a],axis=0))
        radial_n=unit(np.sum([x['radial_sum'] for x in a],axis=0))
        est=p['exact_estimated_normal'][i]
        rows.append(dict(time_s=float(times[i]),source_row=i,prior_hold_index=hold,
            physical_interval_wrench=W,observer_interval_wrench=p['exact_observer_interval_mean'][i,:6],
            interval_force_error_N=np.linalg.norm(F-p['exact_observer_interval_mean'][i,:3]),
            interval_moment_error_Nm=np.linalg.norm(M-p['exact_observer_interval_mean'][i,3:6]),
            filtered_physical_wrench=physical_filtered.copy(),native_filtered_wrench=p['exact_observer_wrench'][i].copy(),
            physical_interval_intrinsic_normal=raw_n,physical_filtered_intrinsic_normal=filtered_n,
            native_filtered_intrinsic_normal=observer_n,estimated_normal=est.copy(),
            pressure_weighted_solver_normal=pressure_n,pressure_point_radial_direction=radial_n,
            saved_DEM_proxy=r['ground_normal'][i].copy(),
            observer_vs_filtered_physical_intrinsic_deg=angle(observer_n,filtered_n),
            physical_intrinsic_vs_pressure_radial_deg=angle(raw_n,radial_n),
            estimated_vs_pressure_normal_deg=angle(est,pressure_n),
            physical_moment_pre_post_direction_sensitivity_deg=angle(raw_n,post_n),
            pressure_family_N=np.mean([x['pressure_family_N'] for x in a],axis=0),
            low_normal_force_N=np.mean([x['low_normal_force'] for x in a],axis=0),
            rejected_force_N=np.mean([x['rejected_force'] for x in a],axis=0),
            native_contact_filter_wrench=np.r_[np.mean([x['native_force'] for x in a],axis=0),np.mean([x['native_moment'] for x in a],axis=0)],
            next_requested=p['exact_requested'][i].copy(),next_applied=p['exact_applied'][i].copy(),
            held_input_completed_interval=p['exact_previous_applied'][i].copy(),
            joint_rate_reference=p['exact_feedback_reference_qvel'][i,6:9].copy(),
            target_up_rate_deg_s=float(target_rate[i]),target_up=reference_up[i].copy(),
            body_up=up(p['exact_qpos'][i,3:7]),
            velocity_error_m_s=p['exact_qvel'][i,:2]-p['exact_reference_velocity_m_s'][i]))
    selected=[]
    for t in [96.15,96.2,96.35,96.4,96.5,96.8,97.,97.5]:
        selected.append(min(rows,key=lambda x:abs(x['time_s']-t)))
    def stats(key):
        v=np.array([x[key] for x in rows if x[key] is not None])
        return dict(max=float(v.max()),rms=float(np.sqrt(np.mean(v*v))),median=float(np.median(v)))
    summary=dict(interval_force_error_N=stats('interval_force_error_N'),
        interval_moment_error_Nm=stats('interval_moment_error_Nm'),
        observer_vs_filtered_physical_intrinsic_deg=stats('observer_vs_filtered_physical_intrinsic_deg'),
        physical_intrinsic_vs_pressure_radial_deg=stats('physical_intrinsic_vs_pressure_radial_deg'),
        estimated_vs_pressure_normal_deg=stats('estimated_vs_pressure_normal_deg'),
        moment_centre_sensitivity_deg=stats('physical_moment_pre_post_direction_sensitivity_deg'))
    for relative,bound in bindings.items():
        if sha(ROOT/relative)!=bound: raise AssertionError('Input/source changed')
    write(out/'report.json',clean(dict(iteration='I-77',intervals=len(rows),substeps=len(sub),
        prefix_bit_exact=True,no_controller_intervention=True,summary=summary,
        selected_rows=selected,rows=rows,source_inputs_unchanged=True,
        qualification='Aligned exported contact-force diagnostics, not an exact force-impulse oracle or closed-loop intervention. Pressure aggregates, radial equivalent contact and ground tangent are different quantities.')))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t=np.array([x['time_s'] for x in rows]);fig,ax=plt.subplots(4,1,figsize=(11,10),sharex=True,constrained_layout=True)
    for key,label in [('estimated_normal','used normal'),('native_filtered_intrinsic_normal','observer intrinsic'),('physical_filtered_intrinsic_normal','filtered exported wrench'),('pressure_weighted_solver_normal','pressure-weighted solver normal')]:
        vals=np.array([x[key] for x in rows]);ax[0].plot(t,np.degrees(np.arccos(np.clip(vals[:,2],-1,1))),label=label)
    ax[0].set_ylabel('Inclination (deg)');ax[0].legend(fontsize=8,ncol=2)
    ax[1].plot(t,[x['target_up_rate_deg_s'] for x in rows],label='desired-up rate');ax[1].set_ylabel('Target rate (deg/s)');ax[1].legend()
    for k,label in enumerate(['vx error','vy error']):ax[2].plot(t,[1000*x['velocity_error_m_s'][k] for x in rows],label=label)
    ax[2].set_ylabel('Velocity error (mm/s)');ax[2].legend()
    for k,label in enumerate(['shaft','shell A','shell B']):ax[3].plot(t,[x['held_input_completed_interval'][k] for x in rows],label=label)
    ax[3].set_ylabel('Prior hold torque (N m)');ax[3].set_xlabel('Recorded interval-end time (s)');ax[3].legend(ncol=3)
    for a in ax:a.axvline(96.15,color='k',ls=':',lw=.7);a.grid(alpha=.25)
    fig.suptitle('I-77 native eight: aligned contacts and early reference/motion response\nNo controller or physics-parameter intervention; all control-prefix arrays reproduce exactly')
    fig.savefig(out/'aligned_contacts.png',dpi=150);plt.close(fig)
    write(out/'artifact_manifest.json',dict(files={q.name:dict(bytes=q.stat().st_size,sha256=sha(q)) for q in sorted(out.iterdir()) if q.is_file()}))
    print(json.dumps(clean(summary)),flush=True)

if __name__=='__main__': main()
