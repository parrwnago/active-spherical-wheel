"""Independent formula audit of fixed I56 portable arrays and saved full diagnostics.
No imported controller/model/reducer functions, no simulation calls.
"""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,gzip,numpy as np
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
STAGE=ROOT/'results/constrained_handoff_20261126/analysis/stage01'
canonical=json.loads((STAGE/'report.json').read_text());index=json.loads((STAGE/'portable_inputs.json').read_text());protocol=json.loads((ROOT/'results/constrained_handoff_20261126/protocol.json').read_text());limits=protocol['acceptance'];events=json.loads((STAGE/'portable_inputs/events.json').read_text())
inputs={}
def readjson(p):inputs[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest();return json.loads(p.read_text())
def arrays(p):inputs[str(p.relative_to(ROOT))]=hashlib.sha256(p.read_bytes()).hexdigest();z=np.load(p,allow_pickle=False);return {k:z[k] for k in z.files}
def rotation(q):
    q=q/np.linalg.norm(q,axis=1)[:,None];w,x,y,z=q.T
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)], [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)], [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]]).transpose(2,0,1)
def first_run(t,mask,duration=.1):
    ids=np.flatnonzero(mask)
    for run in np.split(ids,np.where(np.diff(ids)>1)[0]+1):
        if len(run) and t[run[-1]]-t[run[0]]>=duration-1e-9:return dict(index=int(run[0]),start_s=float(t[run[0]]),end_s=float(t[run[-1]]),duration_s=float(t[run[-1]]-t[run[0]]))
    return None
raws={};derived={};metrics={};metadata={};diagnostics={};comparisons={}
for case,spec in index['cases'].items():
    raw=arrays(STAGE/spec['projection']);raws[case]=raw;t=raw['time_s'];r=rotation(raw['exact_qpos'][:,3:7]);rr=rotation(raw['exact_reference_qpos'][:,3:7]);up=r[:,:,2];rup=rr[:,:,2]
    lean=np.rad2deg(np.arccos(np.clip(np.sum(up*rup,axis=1),-1,1)));tilt=np.rad2deg(np.arccos(np.clip(up[:,2],-1,1)));xyerror=np.linalg.norm(raw['exact_qpos'][:,:2]-raw['exact_reference_xy'],axis=1);worldw=np.einsum('nij,nj->ni',r,raw['exact_qvel'][:,3:6]);heading=np.unwrap(np.arctan2(r[:,1,0],r[:,0,0]));moving=(t>=1)&(t<=41);late=(t>=28-1e-9)&(t<=44+1e-9);il=np.flatnonzero(late);interval=np.r_[.005,np.diff(t)];u=raw['exact_applied'];caps=raw['exact_input_bounds'];prev=raw['exact_previous_applied'];rates=np.array(protocol['candidate_settings']['allocator']['slew_limits']);step=abs(u-prev);slew=max(0.,float((step-interval[:,None]*rates).max()));amp=max(0.,float((abs(u)-caps).max()));request=max(0.,float((abs(raw['exact_requested'])-caps).max()));tail=t>=t[-1]-1
    md=readjson(STAGE/spec['metadata']);metadata[case]=md
    dgpath=STAGE/spec['diagnostics'];inputs[str(dgpath.relative_to(ROOT))]=hashlib.sha256(dgpath.read_bytes()).hexdigest();dg=json.loads(gzip.decompress(dgpath.read_bytes()));diagnostics[case]=[row['diagnostics'] for row in dg]
    eq=max([float(d['equality_residual_max']) for d in diagnostics[case] if d] or [0]);secondary=max([float(np.max(np.abs(d['secondary_balance_delta']))) for d in diagnostics[case] if d] or [0]);primary=np.array([d['first_stage_balance_delta'] for d in diagnostics[case] if d])
    turns=[]
    for ev in events:
        selected=(t>=ev['window_s'][0]-1e-9)&(t<=ev['window_s'][1]+1e-9);value=ev['incoming_sign']*(raw['exact_qpos'][:,ev['axis_index']]-ev['reference_component_extremum_m']);turns.append(float(max(0,value[selected].max())))
    m=dict(rms=float(np.sqrt(np.mean(xyerror[moving]**2))),peak=float(xyerror.max()),final=float(xyerror[-1]),lean_peak=float(lean.max()),lean_rms=float(np.sqrt(np.mean(lean**2))),tilt_peak=float(tilt.max()),yaw_peak=float(abs(worldw[late,2]).max()),heading_rad=float(abs(np.diff(heading[il])).sum()),slew=slew,amp=amp,request=request,equality=eq,secondary=secondary,turns=turns,completed=bool(md['summary']['completed'] and md['summary']['failure'] is None and t[-1]>=44-1e-9),settled=bool(np.all(lean[tail]<1)&np.all(np.linalg.norm(raw['exact_qvel'][tail,:2],axis=1)<.03)&np.all(np.linalg.norm(raw['exact_qvel'][tail,3:6],axis=1)<.08)),prior_actual=bool(np.array_equal(prev[1:],u[:-1])))
    expected=canonical['cases'][case]
    pairs=[(m['rms'],expected['timed_tracking']['moving_rms_m']),(m['peak'],expected['timed_tracking']['full_peak_m']),(m['final'],expected['timed_tracking']['final_m']),(m['lean_peak'],expected['attitude']['peak_raw_reference_error_deg']),(m['lean_rms'],expected['attitude']['rms_raw_reference_error_deg']),(m['tilt_peak'],expected['attitude']['peak_absolute_deg']),(m['yaw_peak'],expected['motion']['late']['world_z_peak_abs_rad_s']),(m['heading_rad'],expected['motion']['late']['heading_absolute_travel_rad']),(m['slew'],expected['commands']['slew_max_violation']),(m['equality'],expected['optimizer'].get('equality_residual_max',0))]
    assert all(np.isclose(a,b,rtol=1e-10,atol=1e-10) for a,b in pairs),(case,pairs)
    np.testing.assert_allclose(turns,[a['geometric_component_overshoot_m'] for a in expected['turns']],atol=1e-12,rtol=0)
    assert m['settled']==expected['original_settling']['passed']
    m['all_independent_core_metrics_agree']=True;metrics[case]=m;derived[case]=dict(t=t,lean=lean,tilt=tilt,xyerror=xyerror,worldw=worldw,primary=primary)
b=metrics['m100_B']
for case in ('m100_H','m100_A'):
    c=metrics[case];can=canonical['comparisons'][case+'_vs_m100_B'];checks=dict(full44_no_failure=c['completed'],baseline_full44_no_failure=b['completed'],baseline_historical_exact=metadata['m100_B']['historical_parity']['all_shared_fields_exact'],moving_window_complete=True,late_window_complete=True,late_heading_valid=True,same_frozen_numeric_bank=metadata[case]['provenance']['fixed_bank_numeric_sha256']==metadata['m100_B']['provenance']['fixed_bank_numeric_sha256'],commands_finite_and_previous_actual=c['prior_actual'] and np.isfinite(raws[case]['exact_applied']).all(),final_command_box=c['amp']<=1e-10 and c['request']<=1e-10,final_command_slew=c['slew']<=1e-10,secondary_balance_equality=c['equality']<=1e-8 and c['secondary']<=1e-8,late_yaw_peak=c['yaw_peak']<=.9*b['yaw_peak']+1e-12,late_heading_absolute_travel=c['heading_rad']<=.8*b['heading_rad']+1e-12,moving_timed_xy_RMS=c['rms']<=1.1*b['rms']+.001+1e-12,full_timed_xy_peak=c['peak']<=b['peak']+.01+1e-12,final_timed_xy=c['final']<=b['final']+.01+1e-12,raw_reference_lean_peak=c['lean_peak']<=b['lean_peak']+1+1e-12,raw_reference_lean_RMS=c['lean_rms']<=1.1*b['lean_rms']+.1+1e-12,absolute_tilt_peak=c['tilt_peak']<=b['tilt_peak']+1+1e-12,all_six_turn_overshoots=all(a<=z+.01+1e-12 for a,z in zip(c['turns'],b['turns'])),conditional_original_settling=not b['settled'] or c['settled'])
    checks={k:bool(v) for k,v in checks.items()};assert checks==can['checks'],(case,checks,can['checks']);comparisons[case]=dict(all_gates_agree=True,checks=checks,failed=[k for k,v in checks.items() if not v])
# Event selection is exploratory saved-log analysis, after campaign outcomes, not preregistered causal evidence.
h=derived['m100_H'];t=h['t'];d=diagnostics['m100_H'];primary_norm=np.linalg.norm(h['primary'],axis=1);delta_error=h['xyerror']-derived['m100_B']['xyerror']
event_selection=dict(first_sustained_raw_lean_over5deg=first_run(t,h['lean']>5),first_sustained_absolute_tilt_over10deg=first_run(t,h['tilt']>10),first_sustained_timed_error_over20mm=first_run(t,h['xyerror']>.02),first_sustained_extra_timed_error_over_B20mm=first_run(t,delta_error>.02),first_sustained_primary_loss_over_01rad_s=first_run(t,primary_norm>.01,.01))
event_selection['peak_primary_loss']=dict(index=int(np.argmax(primary_norm)),time_s=float(t[np.argmax(primary_norm)]),norm_rad_s=float(primary_norm.max()))
event_selection['peak_tilt']=dict(index=int(np.argmax(h['tilt'])),time_s=float(t[np.argmax(h['tilt'])]),deg=float(h['tilt'].max()))
event_selection['peak_xy']=dict(index=int(np.argmax(h['xyerror'])),time_s=float(t[np.argmax(h['xyerror'])]),m=float(h['xyerror'].max()))
fullpath=ROOT/'results/constrained_handoff_20261126/campaign/runs/m100_H/governor_diagnostics.json';full=readjson(fullpath);fullraw=arrays(ROOT/'results/constrained_handoff_20261126/campaign/runs/m100_H/precision.npz')
windows={}
for name,event in event_selection.items():
    if event is None:continue
    i=event['index'];lo=max(0,i-20);hi=min(len(t)-1,i+20);sel=slice(lo,hi+1);fd=full[i]['diagnostics'];wm=h['worldw'][i];j=min(i+1,len(t)-1)
    windows[name]=dict(**event,window_s=[float(t[lo]),float(t[hi])],raw_lean_deg=float(h['lean'][i]),absolute_tilt_deg=float(h['tilt'][i]),timed_error_m=float(h['xyerror'][i]),baseline_timed_error_m=float(derived['m100_B']['xyerror'][i]),world_yaw_rate_rad_s=float(wm[2]),proxy_yaw_rad_s=float(fd['proxy_world_yaw_rate_rad_s']),observed_next_yaw_rad_s=float(h['worldw'][j,2]),first_stage_balance_delta=fd['first_stage_balance_delta'],secondary_balance_delta=fd['secondary_balance_delta'],peak_primary_loss_window=float(primary_norm[sel].max()),native_slew_violation=fd['native_slew_violation'],active_slew=fd['active_slew'],input_delta=fd['input_delta'],contact_names=fd['contact_names'],tangent_confidence_proxy=fd['tangent_confidence_proxy'],mismatch_m_s=fd['estimated_material_velocity_mismatch_m_s'],estimated_loads_N=fullraw['exact_estimated_support_loads_N'][i].tolist(),truth_loads_N=fullraw['truth_patch_loads_N'][i].tolist())
report=dict(completed_utc=datetime.now(timezone.utc).isoformat(),independent_core_metrics=metrics,independent_gate_checks=comparisons,event_selection=event_selection,event_windows=windows,physics_executions=0,causal_scope='Exploratory timing/coincidence only. Gdelta is a 20ms frozen-contact counterfactual difference relative to native-at-this-state; actual next5ms includes native action, common drift, contact impulses and geometry change. Unpaired actual change cannot validate/refute Gdelta. No unique root cause assigned.',input_sha256=inputs)
(HERE/'audit_result.json').write_text(json.dumps(report,indent=2,default=lambda x:bool(x) if isinstance(x,np.bool_) else x,allow_nan=False)+'\n')
print(json.dumps(dict(metric_audit='agree',H_failed=comparisons['m100_H']['failed'],A_failed=comparisons['m100_A']['failed'],events=event_selection),indent=2))
