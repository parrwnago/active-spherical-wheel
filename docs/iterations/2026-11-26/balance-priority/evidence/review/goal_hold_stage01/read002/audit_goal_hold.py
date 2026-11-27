"""Post-primary exploratory fixed goal-hold audit of saved I58 B/P logs.

No dynamics/model/controller/observer calls, fitting or changed acceptance gates.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];STUDY=ROOT/'results/balance_priority_20261126';SUPPORTS=('shell_A','shell_B','belt_A','belt_B')


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return x.item()
    if isinstance(x,dict):return {str(k):safe(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [safe(v) for v in x]
    if isinstance(x,float) and not np.isfinite(x):return None
    return x

def save(p,x):Path(p).write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}
def rotation(q):
    q=q/np.linalg.norm(q,axis=1)[:,None];w,x,y,z=q.T
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],[2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],[2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]]).transpose(2,0,1)
def scalar(x):
    x=np.asarray(x,float);x=x[np.isfinite(x)]
    return dict(samples=len(x),mean=float(x.mean()),RMS=float(np.sqrt(np.mean(x*x))),minimum=float(x.min()),maximum=float(x.max()),p95_abs=float(np.quantile(abs(x),.95))) if len(x) else dict(samples=0)
def vectors(x):
    x=np.asarray(x,float);return dict(norm=scalar(np.linalg.norm(x,axis=1)),RMS_by_component=np.sqrt(np.mean(x*x,axis=0)),max_abs_by_component=np.max(abs(x),axis=0),final=x[-1]) if len(x) else dict(samples=0)
def support_names(loads):
    masks=loads>.05;return [','.join(name for name,active in zip(SUPPORTS,row) if active) or 'none' for row in masks]
def dominant(loads):return np.where(loads.sum(axis=1)>.05,np.argmax(loads,axis=1),-1)
def seal(output):save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size) for p in sorted(output.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']))


def run(output):
    if output.exists():raise FileExistsError('Preserve prior exploratory reads')
    output.mkdir(parents=True);protocol=json.loads((STUDY/'protocol.json').read_text());jobs=[j for j in protocol['cases'] if j['variant'] in ('native','balance_priority')];available={};missing=[]
    for job in jobs:
        folder=STUDY/'campaign/runs'/job['case_id']
        if (folder/'artifact_manifest.json').exists() and all((folder/n).exists() for n in ('precision.npz','summary.json','provenance.json','governor_diagnostics.json')):available[job['case_id']]=(job,folder)
        else:missing.append(job['case_id'])
    inputs=[Path(__file__),HERE/'registered_before_exploratory_arithmetic.json',STUDY/'protocol.json']+[folder/name for _,folder in available.values() for name in ('precision.npz','summary.json','provenance.json','governor_diagnostics.json','artifact_manifest.json')]
    hashes={str(p.relative_to(ROOT)):sha(p) for p in inputs}
    save(output/'registered_before_arithmetic.json',dict(registered_utc=datetime.now(timezone.utc).isoformat(),input_sha256=hashes,available=list(available),missing=missing,scope='Fixed41-44s post-primary exploratory diagnosis; no changedgates ornewphysics.'))
    shutil.copyfile(Path(__file__),output/'audit_goal_hold.py');report=dict(iteration='I-58 post-primary exploratory',window_s=[41,44],last_window_s=[43.5,44],cases={},missing_cases=missing,phase_comparisons={},no_physics=True,
        interpretation='Physical radialvelocity positive meansmovingtowardfixedworldgoal. Gtravelradialdelta is held20msmodelcounterfactual relativeclippednative, not absoluteforecast orcausalphysicaleffect. Endpointsupportcomparisons areevaluation-only; no within5mscontactcontinuity claim.')
    for cid,(job,folder) in available.items():
        for row in json.loads((folder/'artifact_manifest.json').read_text())['files']:assert sha(folder/row['path'])==row['sha256']
        raw=load(folder/'precision.npz');summary=json.loads((folder/'summary.json').read_text());provenance=json.loads((folder/'provenance.json').read_text());governor=json.loads((folder/'governor_diagnostics.json').read_text())
        required=('time_s','exact_qpos','exact_qvel','exact_reference_xy','exact_reference_qpos','exact_native_requested','exact_native_applied','exact_applied','exact_previous_applied','exact_input_bounds','exact_estimated_support_loads_N','truth_patch_loads_N')
        assert all(k in raw for k in required);assert len(governor)==len(raw['time_s']);assert np.array_equal(raw['exact_previous_applied'][1:],raw['exact_applied'][:-1]);assert provenance['source_hashes_unchanged_after']
        t=raw['time_s'];selected=(t>=41-1e-9)&(t<=44+1e-9);ids=np.flatnonzero(selected)
        if not len(ids):report['cases'][cid]=dict(available=True,window_available=False,observed_end_s=float(t[-1]),completed=summary['completed']);continue
        goal=raw['exact_reference_xy'][-1];references=raw['exact_reference_xy'][selected];constant_goal=np.max(abs(references-goal))<=1e-10;assert constant_goal,(cid,'referenceisnotconstantgoal')
        q=raw['exact_qpos'][selected];v=raw['exact_qvel'][selected];tt=t[selected];toward=goal-q[:,:2];error=np.linalg.norm(toward,axis=1);unit=np.divide(toward,error[:,None],out=np.full_like(toward,np.nan),where=error[:,None]>1e-9);radial=np.sum(unit*v[:,:2],axis=1);speed=np.linalg.norm(v[:,:2],axis=1);tangential=np.sqrt(np.maximum(0,speed*speed-radial*radial));R=rotation(q[:,3:7]);omega=np.einsum('nij,nj->ni',R,v[:,3:6]);Rref=rotation(raw['exact_reference_qpos'][selected,3:7]);lean=np.rad2deg(np.arccos(np.clip(np.sum(R[:,:,2]*Rref[:,:,2],axis=1),-1,1)));tilt=np.rad2deg(np.arccos(np.clip(R[:,2,2],-1,1)))
        caps=raw['exact_input_bounds'][selected];actual=raw['exact_applied'][selected];native=raw['exact_native_applied'][selected];request=raw['exact_native_requested'][selected];previous=raw['exact_previous_applied'][selected];delta=actual-native;step=actual-previous;rates=step/.005;exceeded=abs(step)>np.array([60,60,60,6,6])*.005+1e-12;clipped=abs(request)>caps+1e-12;held=ids<len(t)-1
        proxy=np.zeros((len(ids),2));proxy_available=np.zeros(len(ids),bool)
        for local,index in enumerate(ids):
            d=governor[index]['diagnostics']
            if d:
                proxy[local]=d['travel_response_delta_m_s'];proxy_available[local]=True
                np.testing.assert_allclose(d['input_delta'],delta[local],atol=1e-12,rtol=0)
                assert d['horizon_s']==.02
        if job['variant']=='balance_priority':assert proxy_available.all()
        proxy_radial=np.sum(proxy*unit,axis=1);est=raw['exact_estimated_support_loads_N'][selected];truth=raw['truth_patch_loads_N'][selected];mask_mismatch=np.any((est>.05)!=(truth>.05),axis=1);dominant_mismatch=dominant(est)!=dominant(truth)
        def window_stats(lo):
            s=tt>=lo-1e-9;hs=s&held;ii=np.flatnonzero(s)
            trend=float(np.polyfit(tt[s]-tt[s][0],error[s],1)[0]) if s.sum()>=2 else None
            return dict(start_s=float(tt[ii[0]]),end_s=float(tt[ii[-1]]),samples=int(s.sum()),held_commands=int(hs.sum()),error_start_m=float(error[ii[0]]),error_end_m=float(error[ii[-1]]),error_change_m=float(error[ii[-1]]-error[ii[0]]),error_min_m=float(error[s].min()),error_min_time_s=float(tt[ii[np.argmin(error[s])]]),error_max_m=float(error[s].max()),error_max_time_s=float(tt[ii[np.argmax(error[s])]]),error_linear_trend_m_s=trend,actual_toward_goal_radial_velocity_m_s=scalar(radial[s]),actual_radial_final_m_s=float(radial[ii[-1]]),radial_toward_above1mm_s_fraction=float(np.mean(radial[s]>.001)),radial_away_below_minus1mm_s_fraction=float(np.mean(radial[s]<-.001)),radial_quiet_or_unresolved_fraction=float(np.mean(~np.isfinite(radial[s])|(abs(radial[s])<=.001))),tangential_speed_m_s=scalar(tangential[s]),model_G20_travel_radial_delta_m_s=scalar(proxy_radial[hs]),model_G20_radial_opposes_goal_below_minus1mm_s_fraction=float(np.mean(proxy_radial[hs]<-.001)) if hs.any() else None,final_minus_native_command=vectors(delta[hs]),actual_applied_command=vectors(actual[hs]),native_amplitude_clip_held_fraction=float(np.mean(clipped[hs].any(axis=1))) if hs.any() else None,native_amplitude_clip_held_seconds_by_input=clipped[hs].sum(axis=0)*.005,optional_old_rate_exceed_held_fraction=float(np.mean(exceeded[hs].any(axis=1))) if hs.any() else None,optional_old_rate_exceed_seconds_by_input=exceeded[hs].sum(axis=0)*.005,max_abs_actual_command_rate_by_input=np.max(abs(rates[hs]),axis=0) if hs.any() else None,support_mask_disagreement_held_fraction=float(np.mean(mask_mismatch[hs])) if hs.any() else None,dominant_support_disagreement_held_fraction=float(np.mean(dominant_mismatch[hs])) if hs.any() else None,actual_planar_speed_m_s=scalar(speed[s]),actual_world_angular_rate_rad_s=vectors(omega[s]),actual_relative_joint_rates_rad_s=vectors(v[s,6:9]),raw_reference_lean_deg=scalar(lean[s]),absolute_tilt_deg=scalar(tilt[s]))
        original_tail=t>=t[-1]-1;Rt=rotation(raw['exact_qpos'][original_tail,3:7]);Rrt=rotation(raw['exact_reference_qpos'][original_tail,3:7]);lt=np.rad2deg(np.arccos(np.clip(np.sum(Rt[:,:,2]*Rrt[:,:,2],axis=1),-1,1)));vt=raw['exact_qvel'][original_tail];settling=dict(completed=bool(summary['completed']),relative_lean_below1deg=bool(np.all(lt<1)),planar_speed_below030=bool(np.all(np.linalg.norm(vt[:,:2],axis=1)<.03)),angular_rate_below080=bool(np.all(np.linalg.norm(vt[:,3:6],axis=1)<.08)))
        payload=dict(time_s=tt,source_indices=ids,world_goal_xy_m=goal,error_m=error,toward_goal_unit_xy=unit,actual_toward_goal_radial_velocity_m_s=radial,actual_tangential_speed_m_s=tangential,actual_planar_speed_m_s=speed,actual_world_angular_velocity_rad_s=omega,actual_joint_rates_rad_s=v[:,6:9],actual_applied=actual,native_applied=native,native_requested=request,final_minus_native_command=delta,model_G20_travel_response_delta_m_s=proxy,model_G20_radial_delta_m_s=proxy_radial,model_proxy_logged=proxy_available,held_command=held,native_amplitude_clipped=clipped,diagnostic_optional_rate_exceeded=exceeded,estimated_loads_N=est,truth_loads_N=truth,support_mask_discrepancy=mask_mismatch,dominant_support_discrepancy=dominant_mismatch,raw_reference_lean_deg=lean,absolute_tilt_deg=tilt)
        np.savez_compressed(output/(cid+'_goal_hold.npz'),**payload)
        report['cases'][cid]=dict(available=True,completed=bool(summary['completed']),failure=summary['failure'],observed_end_s=float(t[-1]),covers_fixed_window=bool(tt[0]<=41+1e-9 and tt[-1]>=44-1e-9),goal_world_xy_m=goal,constant_reference_goal_verified=constant_goal,whole_hold=window_stats(41),last_half_second=window_stats(43.5),terminal_command_unheld=actual[-1],terminal_minus_native_command_unheld=delta[-1],final_estimated_support=support_names(est[-1:])[0],final_truth_support=support_names(truth[-1:])[0],all_endpoint_support_labels=dict(estimated_counts={x:support_names(est).count(x) for x in set(support_names(est))},truth_counts={x:support_names(truth).count(x) for x in set(support_names(truth))}),original_settling=dict(passed=all(settling.values()),checks=settling,failed_checks=[k for k,vv in settling.items() if not vv],tail_start_s=float(t[original_tail][0]),tail_end_s=float(t[-1]),max_relative_lean_deg=float(lt.max()),max_planar_speed_m_s=float(np.linalg.norm(vt[:,:2],axis=1).max()),max_angular_rate_rad_s=float(np.linalg.norm(vt[:,3:6],axis=1).max())),model_scope='Saved frozenI56G20 differential atcurrentstate; twoI57anchorsvalidatedlocally, not everygoalholdstate orabsolutevelocitybaseline. Bdeltaiszero bydefinition, not a modelaccuracyresult.')
    for phase in ('m100','p030','p090'):
        b,p=report['cases'].get(phase+'_B'),report['cases'].get(phase+'_P')
        if b is None or p is None:report['phase_comparisons'][phase]=dict(available=False,reason='requiredcaseunavailable');continue
        np.testing.assert_array_equal(b['goal_world_xy_m'],p['goal_world_xy_m']);report['phase_comparisons'][phase]=dict(available=True,P_minus_B_error_at41_m=p['whole_hold']['error_start_m']-b['whole_hold']['error_start_m'],P_minus_B_error_at44_m=p['whole_hold']['error_end_m']-b['whole_hold']['error_end_m'],P_error_change41_to44_m=p['whole_hold']['error_change_m'],B_error_change41_to44_m=b['whole_hold']['error_change_m'],P_error_change_last_half_second_m=p['last_half_second']['error_change_m'],B_error_change_last_half_second_m=b['last_half_second']['error_change_m'],P_mean_toward_radial_last_half_m_s=p['last_half_second']['actual_toward_goal_radial_velocity_m_s'].get('mean'),P_final_toward_radial_m_s=p['last_half_second']['actual_radial_final_m_s'])
    save(output/'report.json',report)
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(11,9),constrained_layout=True)
    for row,phase in enumerate(('m100','p030','p090')):
        for suffix,label,color in (('B','NativeB','#2166ac'),('P','PriorityP','#b2182b')):
            path=output/(phase+'_'+suffix+'_goal_hold.npz')
            if not path.exists():continue
            a=load(path);axes[row,0].plot(a['time_s'],a['error_m']*1000,label=label,color=color);axes[row,1].plot(a['time_s'],a['actual_toward_goal_radial_velocity_m_s'],label=label+' actual',color=color)
            if suffix=='P':axes[row,1].plot(a['time_s'][a['held_command']],a['model_G20_radial_delta_m_s'][a['held_command']],ls=':',color='#777777',lw=1,label='G20 candidate−native proxy')
        axes[row,0].set_ylabel('Goal distance,mm');axes[row,1].set_ylabel('Radial velocity,m/s');axes[row,1].axhline(0,lw=.5,color='black')
        for ax in axes[row]:ax.set_title(phase+(' — MISSING PAIR' if not report['phase_comparisons'][phase]['available'] else ''));ax.set_xlabel('Simulation time,s');ax.set_xlim(41,44);ax.legend(fontsize=7)
    fig.suptitle('Post-primary exploratory goalhold41–44s\nPositive radialvelocity approachesfixedgoal. Dotted G20delta is a modelcounterfactual, not causalobservedvelocity.',fontsize=11)
    fig.savefig(output/'goal_hold.png',dpi=180);fig.savefig(output/'goal_hold.pdf');plt.close(fig)
    assert hashes=={str(p.relative_to(ROOT)):sha(p) for p in inputs};seal(output)
    print(json.dumps(safe(dict(available=list(report['cases']),missing=missing,comparisons=report['phase_comparisons'])),indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.output.resolve())
