"""Post-outcome independent I78 scalar closure and descriptive diagnostics.

No physics, controller, governor or canonical metric imports. Fixed protocol gates
are recomputed unchanged. Context offsets are descriptive, never scoring windows.
The earlier spoken never-retimed claim for G+.9 was erroneous: saved stage01
proofs correctly identify first changed reference row3963 and dense35.00463s.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import shutil
import numpy as np

ROOT=Path(__file__).resolve().parents[4]
BASE=ROOT/'results/path_progress_governor_20261128'
ORIGIN=np.array([-2.4,10.])
D=np.array([np.cos(np.deg2rad(113.)),np.sin(np.deg2rad(113.))])
GOAL=ORIGIN+1.2*D
DT=.005

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,dict):return {str(k):safe(v) for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [safe(v) for v in x]
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return safe(x.item())
    if isinstance(x,float)and not np.isfinite(x):return None
    return x
def write(p,x):
    with Path(p).open('x')as f:json.dump(safe(x),f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def load(p):
    with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def changed(a,b):
    a,b=np.asarray(a),np.asarray(b)
    return np.any(np.ascontiguousarray(a).view(np.uint8).reshape(len(a),-1)!=
                  np.ascontiguousarray(b).view(np.uint8).reshape(len(b),-1),axis=1)
def first_time(mask,t):
    rows=np.flatnonzero(mask);return None if not len(rows)else float(t[rows[0]])
def case_paths(cid):
    case=BASE/'campaign/runs'/cid
    return case,case/'child/campaign/runs'/(cid[:-1]+'N')/'precision.npz'
def quaternion_signals(q,qref,v):
    q=q/np.linalg.norm(q,axis=1)[:,None]
    qr=qref/np.linalg.norm(qref,axis=1)[:,None]
    w,x,y,z=q.T;rw,rx,ry,rz=qr.T
    up=np.column_stack((2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)))
    rup=np.column_stack((2*(rx*rz+ry*rw),2*(ry*rz-rx*rw),1-2*(rx*rx+ry*ry)))
    tilt=np.rad2deg(np.arccos(np.clip(up[:,2],-1,1)))
    lean=np.rad2deg(np.arccos(np.clip(np.sum(up*rup,axis=1),-1,1)))
    third=np.column_stack((2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))
    wz=np.sum(third*v[:,3:6],axis=1)
    return tilt,lean,wz,up,rup

def reduce_case(p,s,o):
    t=p['time_s'];arrival=o['first_complete_sample_s'];end=arrival+10.
    ep=t<=end+1e-9;tr=(t>=3.-1e-9)&(t<=arrival+1e-9)
    held=p['command_held_after_row'].astype(bool)&(t<end-1e-9)
    xy=p['exact_qpos'][:,:2];v=p['exact_qvel']
    along=np.sum((xy-ORIGIN)*D,axis=1)
    closest=ORIGIN+np.clip(along,0,1.2)[:,None]*D
    e=np.sqrt(np.sum((xy-closest)**2,axis=1))
    speed=np.sqrt(np.sum(v[:,:2]**2,axis=1))
    goal=np.sqrt(np.sum((xy-GOAL)**2,axis=1))
    tilt,lean,wz,up,rup=quaternion_signals(p['exact_qpos'][:,3:7],p['exact_reference_quat'],v)
    caps=p['exact_input_bounds'];request=p['exact_requested'];applied=p['exact_applied']
    clipped=abs(request)>caps+1e-12
    excess=np.sqrt(np.sum(((request-np.clip(request,-caps,caps))/caps)**2,axis=1))
    step=np.sqrt(np.sum(((applied-p['exact_previous_applied'])/caps)**2,axis=1))
    us=s['gov__progress_u'][tr];es=e[tr];weights=np.maximum(np.diff(us),0)
    progress_rms=np.sqrt(np.sum((es[:-1]**2+es[1:]**2)*weights/2)/np.sum(weights))
    accel=np.sqrt(np.sum(((v[10:,:3]-v[:-10,:3])/.05)**2,axis=1))
    vals=dict(absolute_tilt_peak_deg=np.max(tilt[ep]),
        acceleration_50ms_peak_m_s2=np.max(accel[ep[10:]&ep[:-10]]),
        any_channel_clipping_fraction=np.mean(clipped[held].any(axis=1)),
        any_channel_clipping_seconds=np.sum(clipped[held].any(axis=1))*DT,
        geometric_peak_m=np.max(e[tr]),normalized_applied_command_step_peak=np.max(step[held]),
        normalized_clipping_excess_rms=np.sqrt(np.mean(excess[held]**2)),
        own_raw_target_up_error_peak_deg=np.max(lean[ep]),
        own_raw_target_up_error_rms_deg=np.sqrt(np.mean(lean[ep]**2)),
        planar_speed_peak_m_s=np.max(speed[ep]),progress_weighted_geometric_rms_m=progress_rms,
        time_weighted_geometric_rms_m=np.sqrt(np.mean(e[tr]**2)),
        world_z_angular_rate_peak_rad_s=np.max(abs(wz[ep])))
    final=(t>=end-2.-1e-9)&(t<=end+1e-9)
    position_good=(goal<=.02+1e-12)&(speed<=.02+1e-12)
    four_good=position_good&(abs(wz)<=.05+1e-12)&(lean<=2.+1e-12)
    endpoints=dict(actual_end_s=t[-1],reference_first_complete_sample_s=arrival,
        reference_dense_completion_s=o['finished_at_s'],first10s_end_s=end,
        physical_goal_error_at_end_m=goal[np.flatnonzero(ep)[-1]],
        final2s_samples=final.sum(),final2s_goal_peak_m=np.max(goal[final]),
        final2s_speed_peak_m_s=np.max(speed[final]),
        final2s_position_convergence=bool(final.sum()==401 and position_good[final].all()),
        final2s_four_condition_settling=bool(final.sum()==401 and four_good[final].all()))
    diag=dict(joint_reference_max_abs_rad_s=np.max(abs(p['exact_allocation_drive_reference'][held,:3])),
        joint_reference_channel_max_abs_rad_s=np.max(abs(p['exact_allocation_drive_reference'][held,:3]),axis=0),
        actual_joint_velocity_channel_max_abs_rad_s=np.max(abs(v[ep,6:9]),axis=0),
        joint_reference_over20_rows=int(np.sum(np.max(abs(p['exact_allocation_drive_reference'][held,:3]),axis=1)>20)),
        joint_reference_over20_seconds=float(DT*np.sum(np.max(abs(p['exact_allocation_drive_reference'][held,:3]),axis=1)>20)),
        requested_channel_max_abs=np.max(abs(request[held]),axis=0),
        requested_max_bound_fraction=np.max(abs(request[held])/caps[held]),
        applied_channel_max_abs=np.max(abs(applied[held]),axis=0))
    if o['policy']=='G':
        active=(t>3.)&~s['gov__reference_complete'].astype(bool)&held
        factors={name:s['gov__'+name]for name in ('authority_factor','joint_reference_factor','command_factor')}
        raw=np.min(np.column_stack(list(factors.values())),axis=1)
        slowdown=active&(raw<1.)
        actual_reference_diff=changed(s['governed_xy'],s['nominal_clock_xy'])
        for a,b in [('governed_velocity_m_s','nominal_clock_velocity_m_s'),('governed_acceleration_m_s2','nominal_clock_acceleration_m_s2')]:
            actual_reference_diff|=changed(s[a],s[b])
        diag['clock']=dict(minimum_traversal_lambda=np.min(s['gov__clock_rate'][active]),
            minimum_target_gamma=np.min(s['gov__new_target_rate'][active]),
            first_gamma_below1_s=first_time(slowdown,t),
            first_coherent_reference_difference_from_original_clock_s=first_time(actual_reference_diff,t),
            maximum_traversal_virtual_delay_s=np.max((t-s['gov__virtual_time_s'])[active]),
            slowdown_target_rows=int(slowdown.sum()),slowdown_target_seconds=float(slowdown.sum()*DT),
            factor_activity={name:dict(minimum=float(np.min(value[active])),
                below1_rows=int(np.sum(active&(value<1.))),
                achieves_min_below1_rows=int(np.sum(slowdown&(abs(value-raw)<=1e-13))))for name,value in factors.items()})
    sig=dict(t=t,ep=ep,tr=tr,held=held,e=e,speed=speed,goal=goal,tilt=tilt,lean=lean,wz=wz,up=up,rup=rup)
    return vals,endpoints,diag,sig

def contexts(p,s,sig):
    answer={}
    for name,key in [('speed_peak','speed'),('tilt_peak','tilt')]:
        ids=np.flatnonzero(sig['ep']);i=int(ids[np.argmax(sig[key][ids])]);rows=[]
        for delta in (-1.,-.5,-.25,-.1,-.05,0.,.05,.1,.25,.5,1.):
            j=i+int(round(delta/DT))
            if j<0 or j>=len(sig['t']):continue
            rows.append(dict(offset_s=delta,row=j,time_s=sig['t'][j],
                xy=p['exact_qpos'][j,:2],body_velocity=p['exact_qvel'][j,:3],
                speed_m_s=sig['speed'][j],tilt_deg=sig['tilt'][j],up_error_deg=sig['lean'][j],
                geometric_error_m=sig['e'][j],world_z_rate_rad_s=sig['wz'][j],
                tau_s=s['gov__virtual_time_s'][j],lambda_rate=s['gov__clock_rate'][j],
                lambda_dot_s_inv=s['gov__clock_acceleration_s_inv'][j],
                new_gamma=s['gov__new_target_rate'][j],
                factors=[s['gov__'+k][j]for k in ('authority_factor','joint_reference_factor','command_factor')],
                reference_velocity=p['exact_reference_velocity_m_s'][j],
                reference_acceleration=p['exact_reference_acceleration_m_s2'][j],
                actual_up=sig['up'][j],desired_up=sig['rup'][j],estimated_normal=p['exact_estimated_normal'][j],
                support_loads_N=p['exact_estimated_support_loads_N'][j],
                drive_references=p['exact_allocation_drive_reference'][j],
                requested=p['exact_requested'][j],applied=p['exact_applied'][j],
                lqr_command=p['exact_lqr_command'][j],rolling_feedforward=p['exact_rolling_feedforward'][j]))
        answer[name]=dict(peak_row=i,peak_time_s=sig['t'][i],peak_value=sig[key][i],context=rows)
    return answer

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    out=args.output.resolve();protocol_path=BASE/'protocol/protocol.json';seal_path=BASE/'campaign/outputs_sealed.json'
    report_path=BASE/'evaluation/completed_stage01/report.json'
    proto=json.loads(protocol_path.read_text());seal=json.loads(seal_path.read_text())
    canonical=json.loads(report_path.read_text());paths=[Path(__file__).resolve(),protocol_path,seal_path,report_path,
        BASE/'evaluation/completed_stage01/artifact_manifest.json',
        BASE/'evaluation/evaluate_stage02.py',BASE/'evaluation/metrics_stage02.py']
    for job in proto['cases']:
        case,precision=case_paths(job['case_id'])
        paths.extend((precision,case/'governor_diagnostics.npz',case/'i78_outcome.json',case/'artifact_manifest.json',
            BASE/'governor/verification'/(job['case_id']+'_stage01')/'report.json'))
        for p in (precision,case/'governor_diagnostics.npz',case/'i78_outcome.json'):
            bound=seal['files'][str(p.relative_to(BASE))]
            assert sha(p)==bound['sha256']and p.stat().st_size==bound['bytes']
    bindings={str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in set(paths)}
    out.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),out/'crosscheck_and_diagnose.py')
    write(out/'registered_before_new_reductions.json',dict(utc=datetime.now(timezone.utc).isoformat(),files=bindings,
        no_new_physics=True,post_outcome_descriptive=True,score_or_policy_changes=False,
        prior_reporting_correction='G+.9 stage01 report always said row3963 divergence and dense35.0046309899s. Earlier never-retimed/full-through45 prose was an erroneous human summary; original reports/source retained.',
        context_offsets_s=[-1.,-.5,-.25,-.1,-.05,0.,.05,.1,.25,.5,1.],
        comparisons='Original fixed canonical scalar/gate definitions independently coded. All6 held command rows for bursts; terminal hold excluded from slowdown. No new adoption thresholds.'))
    reduced={};raw={};side={};signals={};checks=[]
    for job in proto['cases']:
        cid=job['case_id'];case,precision=case_paths(cid)
        p,s,o=load(precision),load(case/'governor_diagnostics.npz'),json.loads((case/'i78_outcome.json').read_text())
        assert o['case']['case_id']==cid and o['policy']==job['policy']
        vals,endpoint,diag,sig=reduce_case(p,s,o);raw[cid]=p;side[cid]=s;signals[cid]=sig
        row=canonical['cases'][cid];res={}
        for name,value in vals.items():
            cfg=proto['evaluation']['nonregression'][name];target=row
            for key in cfg['path']:target=target[key]
            error=float(abs(value-target));res[name]=error;assert error<1e-11,(cid,name,value,target)
        assert abs(endpoint['physical_goal_error_at_end_m']-row['tracking']['goal_error_at_primary_end_m'])<1e-12
        assert endpoint['final2s_position_convergence']==row['tracking']['physical_goal_position_converged']
        assert endpoint['final2s_four_condition_settling']==row['settling']['four_condition_passed']
        reduced[cid]=dict(scalars=vals,endpoints=endpoint,diagnostics=diag,canonical_max_abs_residuals=res)
    prefix={}
    for stem in ('straight_m100','straight_p030','straight_p090'):
        n,g=raw[stem+'_N'],raw[stem+'_G'];common=min(len(n['time_s']),len(g['time_s']))
        divergence={}
        for label,keys in dict(reference=['exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2'],
                               qpos=['exact_qpos'],qvel=['exact_qvel'],applied=['exact_applied']).items():
            diff=np.zeros(common,bool)
            for key in keys:diff|=changed(g[key][:common],n[key][:common])
            idx=np.flatnonzero(diff);divergence[label]=dict(first_row=None if not len(idx)else int(idx[0]),
                first_time_s=first_time(diff,n['time_s'][:common]))
        proof=json.loads((BASE/'governor/verification'/(stem+'_G_stage01')/'report.json').read_text())
        assert divergence['reference']['first_row']==proof['first_changed_reference_row']
        prefix[stem]=divergence
        for metric,cfg in proto['evaluation']['nonregression'].items():
            nv=reduced[stem+'_N']['scalars'][metric];gv=reduced[stem+'_G']['scalars'][metric]
            limit=cfg['ratio']*nv+cfg['add'];passed=bool(gv<=limit+1e-12)
            named=str(float(proto['cases'][[j['case_id']for j in proto['cases']].index(stem+'_G')]['phase_rad']))+'_'+metric
            orig=next(c for c in canonical['decision']['checks']if c['name']==named)
            assert orig['passed']==passed and abs(orig['limit']-limit)<1e-11
            checks.append(dict(name=named,actual=gv,native=nv,limit=limit,passed=passed))
    context={cid:contexts(raw[cid],side[cid],signals[cid])for cid in ('straight_p030_N','straight_p030_G')}
    for path,b in bindings.items():assert sha(ROOT/path)==b['sha256']and(ROOT/path).stat().st_size==b['bytes']
    write(out/'report.json',dict(iteration='I-78',passed=True,source_inputs_unchanged=True,cases=reduced,
        raw_pair_divergence=prefix,independent_nonregression_checks=checks,
        failed_nonregression_checks=[c for c in checks if not c['passed']],p030_peak_context=context,
        causal_qualification='Context is temporal correlation from changed closed-loop trajectories. It does not prove the original root cause, uniquely isolate acceleration/clock/contact effects, or demonstrate hard feasibility.',
        reporting_correction='G+.9 did retime starting19.815s; saved stage01 math/prefix proof was correct, earlier prose summary was incorrect.'))
    write(out/'artifact_manifest.json',dict(files={p.name:dict(bytes=p.stat().st_size,sha256=sha(p))for p in out.iterdir()if p.is_file()}))
    print(json.dumps(safe(dict(passed=True,failed_checks=[c for c in checks if not c['passed']],
        case_summary={cid:dict(scalars=x['scalars'],endpoints=x['endpoints'],diagnostics=x['diagnostics'])for cid,x in reduced.items()},
        divergence=prefix)),indent=2))

if __name__=='__main__':main()
