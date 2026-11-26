"""I50 descriptive synchronization audit, saved I49 data only.

Fixed windows: recorded17–19s and30–34s (video11s and17–19s respectively).
No controller/observer/predictor/preview or simulator execution is permitted.
"""
from datetime import datetime,timezone
from pathlib import Path
import csv
import hashlib
import json

import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
INPUT=ROOT/'results/belt_capacity_sweep_20261126'
CASES=('mars_cap030','mars_cap060','mars_cap090')
LABELS=('shell_A','shell_B','belt_A','belt_B')
WINDOWS={'recorded18':[17.,19.],'playback18_recorded32':[30.,34.]}


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def safe(x):
    if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [safe(v)for v in x]
    if isinstance(x,np.ndarray):return safe(x.tolist())
    if isinstance(x,np.generic):return safe(x.item())
    if isinstance(x,float)and not np.isfinite(x):return None
    return x
def save(path,x):path.write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def dominant(loads,threshold):
    return np.where(loads.max(axis=1)>=threshold,np.argmax(loads,axis=1),4)
def state_runs(t,values,mask):
    ii=np.flatnonzero(mask);out=[]
    if not len(ii):return out
    start=ii[0];last=start
    for i in ii[1:]:
        if i!=last+1 or values[i]!=values[last]:
            out.append(dict(start_s=t[start],last_s=t[last],span_s=t[last]-t[start],samples=last-start+1,label=(*LABELS,'none')[values[start]]))
            start=i
        last=i
    out.append(dict(start_s=t[start],last_s=t[last],span_s=t[last]-t[start],samples=last-start+1,label=(*LABELS,'none')[values[start]]))
    return out


def main():
    output=HERE/'stage01';output.mkdir(exist_ok=False)
    sources=[Path(__file__).resolve(),ROOT/'src/spherical_wheel/synchronization.py',ROOT/'src/spherical_wheel/offroad_control.py',
        ROOT/'src/spherical_wheel/experiments.py',ROOT/'src/spherical_wheel/contact_sensing.py',ROOT/'src/spherical_wheel/model.py',
        ROOT/'src/spherical_wheel/gazebo/plant.py',ROOT/'gazebo/plugin/SphericalWheelBridge.cc']
    records=[]
    for path in sources:
        target=output/'source_snapshot'/path.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(path.read_bytes())
        records.append(dict(origin=str(path.relative_to(ROOT)),snapshot=str(target.relative_to(output)),sha256=sha(path)))
    inputs={}
    for case in CASES:
        for name in ('precision.npz','rollout.npz','provenance.json','summary.json'):
            p=INPUT/'campaign/runs'/case/name;inputs[str(p.relative_to(ROOT))]=sha(p)
    for name in ('video_metadata.json','video_display_schedule.json'):
        p=INPUT/'videos'/name;inputs[str(p.relative_to(ROOT))]=sha(p)
    save(output/'before_arithmetic.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),sources=records,input_sha256=inputs,
        fixed_windows_s=WINDOWS,scope='Secondary descriptive saved-data audit only; no new physics, controller replay or kinematic forecast. Event peaks are maximum normalized successive applied-command steps within these fixed windows; all5ms rows preserved.'))
    schedule=json.loads((INPUT/'videos/video_display_schedule.json').read_text())
    video_row=next(r for r in schedule if r['frame']==360)
    result=dict(video18s_frame360=video_row,windows_s=WINDOWS,cases={},limitations=[
        'Full _sync target, lookahead chosen, margin/gap and predicted mismatch were not saved by I49; weights and command delta were.',
        'Current solver truth is last1ms snapshot at5ms endpoints, not an interval mean or all substeps.',
        'Estimated5% synchronization cutoff is a fraction of inferredTOTALnormal load; truth dominant display uses5%bodyweight and is explicitly different.',
        'A temporal association of estimated contact or synchronization with a twitch does not establish its cause.'])
    for case in CASES:
        directory=INPUT/'campaign/runs'/case;p=load(directory/'precision.npz');log=load(directory/'rollout.npz')
        t=p['time_s'];bounds=p['exact_input_bounds'];weight=json.loads((directory/'provenance.json').read_text())['weight_N'];threshold=.05*weight
        truth=p['truth_patch_loads_N'];estimated=p['exact_estimated_support_loads_N'];totals=estimated.sum(axis=1)
        shares=estimated/np.maximum(totals[:,None],1e-12);truth_dom=dominant(truth,threshold);obs_dom=dominant(estimated,threshold)
        step=np.zeros((len(t),5));step[1:]=p['exact_applied'][1:]-p['exact_applied'][:-1]
        requested_step=np.zeros_like(step);requested_step[1:]=np.diff(p['exact_native_requested'],axis=0)
        nominal_rate=np.full((len(t),3),np.nan);nominal_rate[:-1]=np.diff(p['exact_nominal_joints_before'],axis=0)/.005
        nominal_extra=nominal_rate-p['exact_reference_qvel'][:,6:9]
        windows={};csv_rows=[]
        for name,(lo,hi)in WINDOWS.items():
            mask=(t>=lo-1e-9)&(t<=hi+1e-9);ii=np.flatnonzero(mask)
            peak=ii[np.argmax(np.max(abs(step[ii])/bounds[ii],axis=1))]
            delta=p['exact_transition_command_delta'];weights=p['exact_sync_weights']
            before_sync=sum(p[key]for key in ('exact_equilibrium_command','exact_rolling_feedforward','exact_support_command','exact_lqr_command'))
            scope=dict(window_s=[lo,hi],samples=len(ii),max_sync_weight=np.max(weights[ii],axis=0),
                nonzero_sync_samples=np.sum(weights[ii]>1e-12,axis=0),max_abs_sync_command_delta=np.max(abs(delta[ii]),axis=0),
                peak_applied_step=dict(index=int(peak),time_s=t[peak],step=step[peak],requested_step=requested_step[peak],
                    normalized_by_own_bounds=step[peak]/bounds[peak],applied=p['exact_applied'][peak],requested=p['exact_native_requested'][peak],
                    observer_patch_loads_N=estimated[peak],truth_patch_loads_N=truth[peak],observer_shares=shares[peak],
                    sync_weights=weights[peak],transition_delta=delta[peak],lqr=p['exact_lqr_command'][peak],
                    lqr_step=p['exact_lqr_command'][peak]-p['exact_lqr_command'][peak-1],
                    rolling=p['exact_rolling_feedforward'][peak],support=p['exact_support_command'][peak],
                    actual_joint_rates_rad_s=p['exact_qvel'][peak,6:9],drive_references=p['exact_allocation_drive_reference'][peak],
                    slip_m_s=log['slip_speed_m_s'][peak],utilization=log['utilization'][peak]),
                truth_dominant_runs=state_runs(t,truth_dom,mask),observer_dominant_runs=state_runs(t,obs_dom,mask),
                estimated_contact_vs_truth_dominant_disagree_samples=int(np.sum(truth_dom[ii]!=obs_dom[ii])),
                max_abs_nominal_extra_joint_rate=np.nanmax(abs(nominal_extra[ii]),axis=0),
                max_abs_requested=np.max(abs(p['exact_native_requested'][ii]),axis=0),
                max_abs_applied=np.max(abs(p['exact_applied'][ii]),axis=0),max_slip_m_s=np.max(log['slip_speed_m_s'][ii]))
            windows[name]=scope
            for i in ii:
                row=dict(window=name,index=int(i),time_s=t[i],truth_dominant=(*LABELS,'none')[truth_dom[i]],
                    observer_dominant=(*LABELS,'none')[obs_dom[i]],slip_m_s=log['slip_speed_m_s'][i],utilization=log['utilization'][i],
                    maximum_normalized_applied_step=np.max(abs(step[i])/bounds[i]),
                    maximum_normalized_requested_step=np.max(abs(requested_step[i])/bounds[i]),
                    inferred_total_N=totals[i],planar_tracking_error_m=np.linalg.norm(p['exact_qpos'][i,:2]-p['exact_reference_xy'][i]))
                for j,label in enumerate(LABELS):
                    row['truth_load_'+label+'_N']=truth[i,j];row['observer_load_'+label+'_N']=estimated[i,j];row['observer_share_'+label]=shares[i,j]
                for key,values in [('sync_weight',weights[i]),('transition_delta',delta[i]),('pre_sync_command',before_sync[i]),
                    ('requested',p['exact_native_requested'][i]),('applied',p['exact_applied'][i]),('applied_step',step[i]),
                    ('lqr',p['exact_lqr_command'][i]),('rolling',p['exact_rolling_feedforward'][i]),('support',p['exact_support_command'][i]),
                    ('drive_reference',p['exact_allocation_drive_reference'][i])]:
                    for j,value in enumerate(values):row[key+'_'+str(j)]=value
                for j,label in enumerate(('shaft','shell_A','shell_B')):
                    row['actual_rate_'+label+'_rad_s']=p['exact_qvel'][i,6+j]
                    row['reference_rate_'+label+'_rad_s']=p['exact_reference_qvel'][i,6+j]
                    row['nominal_extra_rate_'+label+'_rad_s']=nominal_extra[i,j]
                    row['nominal_angle_'+label+'_rad']=p['exact_nominal_joints_before'][i,j]
                csv_rows.append(safe(row))
        with(output/(case+'_windows.csv')).open('w',newline='')as stream:
            writer=csv.DictWriter(stream,fieldnames=list(csv_rows[0]));writer.writeheader();writer.writerows(csv_rows)
        result['cases'][case]=dict(windows=windows,weight_N=weight,
            loaded_sync_overlap_samples_perdrive=np.sum((p['exact_sync_weights'][:,1:]>1e-12)&(shares>=.05-1e-12),axis=0))
    save(output/'report.json',result)
    for rel,expected in inputs.items():assert sha(ROOT/rel)==expected
    for row in records:assert sha(ROOT/row['origin'])==row['sha256']
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),bytes=p.stat().st_size,sha256=sha(p))
        for p in sorted(output.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))
    print(json.dumps(safe({case:{name:scope['peak_applied_step']for name,scope in detail['windows'].items()}
        for case,detail in result['cases'].items()}),indent=2))


if __name__=='__main__':main()
