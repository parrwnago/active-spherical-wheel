"""I77 baseline state/command timelines. Algebraic ordering, not causal proof."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,importlib.util,json,shutil
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
DATA=ROOT/'results/tangent_heading_routes_20261127'
OUT=HERE/'completed_stage01'
FAMILIES=('shell_A','shell_B','belt_A','belt_B')
TERMS=('equilibrium_command','rolling_feedforward','lqr_command','support_command','transition_command_delta','heading_command','fade_command_delta','clipping_command_delta')
BLOCKS=(('XY_position',0,2),('height',2,3),('roll_pitch_attitude',3,5),('heading_position',5,6),('derived_rolling_shaft_error',6,7),('cyclic_shell_position',7,9),('body_linear_velocity',9,12),('body_angular_velocity',12,15),('shaft_velocity',15,16),('shell_velocities',16,18))

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def safe(x):
 if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [safe(v)for v in x]
 if isinstance(x,np.ndarray):return safe(x.tolist())
 if isinstance(x,np.generic):return safe(x.item())
 if isinstance(x,float)and not np.isfinite(x):return None
 return x
def write(p,v):
 with p.open('x')as f:json.dump(safe(v),f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def angle(a,b):
 a=a/np.linalg.norm(a,axis=1)[:,None];b=b/np.linalg.norm(b,axis=1)[:,None]
 return np.degrees(np.arccos(np.clip(np.sum(a*b,axis=1),-1,1)))
def support(loads):
 total=loads.sum(axis=1);valid=np.isfinite(loads).all(axis=1)&(loads>=0).all(axis=1)&(total>=1.)
 shares=np.divide(loads,total[:,None],out=np.full_like(loads,np.nan),where=total[:,None]>0)
 dominant=np.where(valid,np.argmax(loads,axis=1),-1)
 mask=np.sum((shares>=.1)*(1<<np.arange(4)),axis=1);mask[~valid]=0
 return shares,dominant,mask,valid

def peak(t,a,idx,absolute=True):
 v=np.asarray(a);v=v[:,None]if v.ndim==1 else v
 out=[]
 for c in range(v.shape[1]):
  finite=idx[np.isfinite(v[idx,c])]
  if not len(finite):out.append(None);continue
  j=int(finite[np.argmax(abs(v[finite,c])if absolute else v[finite,c])]);out.append(dict(channel=c,source_row=j,time_s=float(t[j]),value=float(v[j,c])))
 return out

def changes(t,a,idx):
 ids=idx[(idx>0)&(a[idx]!=a[np.maximum(idx-1,0)])]
 return [dict(source_row=int(i),time_s=float(t[i]),before=int(a[i-1]),after=int(a[i]))for i in ids]

def recovered_interval(t,timed,geo,moving,peakidx):
 good=moving&(np.arange(len(t))<peakidx)&(timed<=.020)&(geo<=.010)&np.isfinite(timed)&np.isfinite(geo)
 groups=[];start=None
 for i,okay in enumerate(good):
  if okay and(start is None or(i and abs(t[i]-t[i-1]-.005)>1e-9)):start=i
  if start is not None and(not okay or i==len(t)-1):
   end=i-1 if not okay else i
   if t[end]-t[start]>=.1-1e-9:groups.append((start,end))
   start=None
 return None if not groups else groups[-1]

def first_after(t,y,bound,low,high):
 start=None
 for i in range(low,high+1):
  if not np.isfinite(y[i])or y[i]<bound:start=None;continue
  if start is None:start=i
  if t[i]-t[start]>=.1-1e-9:return start,i
 return None

def snapshot(raw,a,index):
 t=a['time_s'];i=int(index)
 return dict(source_row=i,time_s=float(t[i]),state_before_command=True,command_held_after=bool(raw['command_held_after_row'][i]),timed_error_m=a['timed_error_m'][i],geometric_error_m=a['geometric_error_m'][i],offset_xy_m=a['offset_xy_m'][i],velocity_actual_world_m_s=raw['exact_qvel'][i,:3],velocity_reference_world_xy_m_s=raw['exact_reference_velocity_m_s'][i],velocity_error_world_xy_m_s=a['velocity_error_world_xy_m_s'][i],reference_acceleration_world_xy_m_s2=raw['exact_reference_acceleration_m_s2'][i],actual_backward50ms_acceleration_world_m_s2=a['actual_backward50ms_acceleration_world_m_s2'][i],tilt_deg=a['absolute_tilt_deg'][i],reference_up_error_deg=a['reference_up_error_deg'][i],normal_estimate=raw['exact_estimated_normal'][i],saved_local_DEM_proxy_normal=a['saved_ground_normal_float32'][i],normal_estimate_vs_DEM_proxy_deg=a['normal_estimate_vs_DEM_proxy_deg'][i],estimated_loads_N=raw['exact_estimated_support_loads_N'][i],instantaneous_positive_pressure_loads_N=raw['truth_patch_loads_N'][i],estimated_dominant=int(a['estimated_dominant'][i]),pressure_dominant=int(a['pressure_dominant'][i]),estimated_loadedset_bits=int(a['estimated_loadedset_bits'][i]),pressure_loadedset_bits=int(a['pressure_loadedset_bits'][i]),effective_phase_rad=raw['exact_effective_phase_rad'][i],joint_rate_actual_rad_s=raw['exact_qvel'][i,6:9],joint_rate_reference_rad_s=raw['exact_reference_qvel'][i,6:9],feedback_joint_rate_reference_rad_s=raw['exact_feedback_reference_qvel'][i,6:9],allocation_drive_reference_mixed=raw['exact_allocation_drive_reference'][i],requested_command_mixed=raw['exact_requested'][i],applied_next_interval_mixed=raw['exact_applied'][i],previous_applied_observed_interval_mixed=raw['exact_previous_applied'][i],input_bounds_mixed=raw['exact_input_bounds'][i],sync_weights=raw['exact_sync_weights'][i],drive_fade=raw['exact_drive_fade'][i],command_terms={k:raw['exact_'+k][i]for k in TERMS},lqr_blocks={k:a['lqr_'+k][i]for k,_,_ in BLOCKS},gain_change_contribution_mixed=a['gain_change_contribution_mixed'][i],error_change_contribution_mixed=a['error_change_contribution_mixed'][i],normalized_requested_excess=a['normalized_requested_excess'][i])

def window_summary(raw,a,center,span):
 t=a['time_s'];idx=np.flatnonzero((t>=center-span-1e-10)&(t<=center+span+1e-10));held=idx[raw['command_held_after_row'][idx].astype(bool)]
 clipped=held[a['any_requested_clipping'][held]]
 return dict(center_s=center,half_width_s=span,first_s=t[idx[0]],last_s=t[idx[-1]],samples=len(idx),first_clipping_time_s=None if not len(clipped)else t[clipped[0]],clipped_held_samples=len(clipped),estimated_loadedset_changes=changes(t,a['estimated_loadedset_bits'],idx),instantaneous_pressure_loadedset_changes=changes(t,a['pressure_loadedset_bits'],idx),estimated_dominant_changes=changes(t,a['estimated_dominant'],idx),pressure_dominant_changes=changes(t,a['pressure_dominant'],idx),normal_proxy_step_peaks_deg=peak(t,a['ground_normal_step_deg'],idx),estimated_normal_step_peaks_deg=peak(t,a['estimated_normal_step_deg'],idx),normal_mismatch_peak_deg=peak(t,a['normal_estimate_vs_DEM_proxy_deg'],idx),joint_reference_abs_peaks_rad_s=peak(t,raw['exact_reference_qvel'][:,6:9],held),joint_reference_step_abs_peaks_rad_s=peak(t,a['joint_reference_step_rad_s'],held),actual_joint_rate_abs_peaks_rad_s=peak(t,raw['exact_qvel'][:,6:9],idx),requested_abs_peaks_mixed=peak(t,raw['exact_requested'],held),applied_abs_peaks_mixed=peak(t,raw['exact_applied'],held),normalized_excess_peak=peak(t,a['normalized_requested_excess'],held),gain_change_effect_peak_normalized=peak(t,a['gain_change_contribution_norm_by_caps'],held),error_change_effect_peak_normalized=peak(t,a['error_change_contribution_norm_by_caps'],held),snapshots=[snapshot(raw,a,int(np.argmin(abs(t-(center+offset)))))for offset in (-span,-.25,-.1,-.05,0.,.05,.1,.25,span)if abs(offset)<=span+1e-12])

def main():
 protocol_path=DATA/'protocol/protocol.json';protocol=json.loads(protocol_path.read_text());jobs=[j for j in protocol['cases']if j['policy']=='N'];events_path=HERE/'events_stage01/report.json';events=json.loads(events_path.read_text())
 sources=[Path(__file__),DATA/'evaluation/metrics.py',ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/contact_sensing.py',ROOT/'src/spherical_wheel/offroad.py',ROOT/'src/spherical_wheel/offroad_experiments.py',ROOT/'src/spherical_wheel/gazebo/plant.py']
 files=sources+[protocol_path,DATA/'campaign/outputs_sealed.json',DATA/'evaluation/completed_stage01/science.json',events_path,HERE/'events_stage01/artifact_manifest.json']
 for j in jobs:
  files += [DATA/f"campaign/runs/{j['case_id']}/{n}"for n in ('precision.npz','rollout.npz','summary.json','provenance.json')]+[DATA/f"evaluation/completed_stage01/{j['case_id']}_signals.npz"]
 bindings={str(p.relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size)for p in files}
 OUT.mkdir(parents=True,exist_ok=False)
 for p in sources:
  dest=OUT/'source'/p.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
 write(OUT/'registered_before_array_reads.json',dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),files=bindings,no_new_physics=True,no_control_changes=True,events='Fixed selector report. Contexts±1s and±.25s at every event.',local_eight_rule='For eight_m100_N, latest prepeak contiguous100ms recovered interval with timed<=20mm AND geometric<=10mm; report exit and subsequent first sustainedgeo30mm/timed50mm. Ifnone reportnone, neverretune.',gain_decomposition='DeltaLQR=-(Ki-Kprev)e_prev-Ki(ei-e_prev), reported in actual command units and cap-normalized norm; algebraic, not independent causal interventions.',truth_limits='Positive-pressure truth is last1ms only, not prior5ms interval or full tangential force routing. Saved ground normal is float32 local DEM-nearest-centre tangent proxy, not DART per-contact normals.',timing='State atrowi precedes rowi generated command; previous_applied held over observed preceding interval. Backward50ms acceleration uses past/current velocities only.'))
 spec=importlib.util.spec_from_file_location('_i77_metrics',DATA/'evaluation/metrics.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 reports={};local_report=None
 for job in jobs:
  cid=job['case_id'];folder=DATA/'campaign/runs'/cid
  with np.load(folder/'precision.npz',allow_pickle=False)as z:raw={k:z[k].copy()for k in z.files}
  with np.load(folder/'rollout.npz',allow_pickle=False)as z:roll={k:z[k].copy()for k in ('time_s','qpos','qvel','ground_normal','estimated_normal','ground_height_m','slip_speed_m_s')}
  with np.load(DATA/f'evaluation/completed_stage01/{cid}_signals.npz',allow_pickle=False)as z:s={k:z[k].copy()for k in ('time_s','timed_xy_error_m','geometric_cross_track_m','absolute_tilt_deg','reference_up_error_deg','world_z_angular_speed_rad_s')}
  t=raw['time_s'];n=len(t);caps=raw['exact_input_bounds'];held=raw['command_held_after_row'].astype(bool)
  np.testing.assert_array_equal(t,s['time_s'])
  for k in ('time_s','qpos','qvel'):np.testing.assert_array_equal(roll[k],raw[{'qpos':'exact_qpos','qvel':'exact_qvel'}.get(k,k)].astype(np.float32))
  np.testing.assert_array_equal(roll['estimated_normal'],raw['exact_estimated_normal'].astype(np.float32))
  K,e=raw['exact_feedback_gain'],raw['exact_error'];lqr=-np.einsum('nij,nj->ni',K,e)
  gain_delta=np.zeros((n,5));err_delta=np.zeros((n,5));gain_delta[1:]=-np.einsum('nij,nj->ni',K[1:]-K[:-1],e[:-1]);err_delta[1:]=-np.einsum('nij,nj->ni',K[1:],e[1:]-e[:-1])
  blocks={name:-np.einsum('nij,nj->ni',K[:,:,lo:hi],e[:,lo:hi])for name,lo,hi in BLOCKS}
  before=raw['exact_equilibrium_command']+raw['exact_rolling_feedforward']+raw['exact_lqr_command']+raw['exact_support_command'];requested=before+raw['exact_transition_command_delta']+raw['exact_heading_command']+raw['exact_fade_command_delta']
  checks=dict(lqr=float(np.max(abs(lqr-raw['exact_lqr_command']))),blocks=float(np.max(abs(sum(blocks.values())-lqr))),delta_lqr=float(np.max(abs(gain_delta[1:]+err_delta[1:]-np.diff(lqr,axis=0)))),requested_ledger=float(np.max(abs(requested-raw['exact_requested']))),applied_ledger=float(np.max(abs(requested+raw['exact_clipping_command_delta']-raw['exact_applied']))))
  if max(checks.values())>1e-9:raise ValueError('Algebraic ledger failed '+cid)
  estshares,estdom,estmask,estvalid=support(raw['exact_estimated_support_loads_N']);pshares,pdom,pmask,pvalid=support(raw['truth_patch_loads_N'])
  estnormal=raw['exact_estimated_normal'];ground=roll['ground_normal'].astype(float);normalerror=angle(estnormal,ground)
  eststep=np.r_[np.nan,angle(estnormal[1:],estnormal[:-1])];groundstep=np.r_[np.nan,angle(ground[1:],ground[:-1])]
  acc=np.full((n,3),np.nan);acc[10:]=(raw['exact_qvel'][10:,:3]-raw['exact_qvel'][:-10,:3])/.05
  excess=np.linalg.norm((raw['exact_requested']-np.clip(raw['exact_requested'],-caps,caps))/caps,axis=1)
  a=dict(time_s=t,timed_error_m=s['timed_xy_error_m'],geometric_error_m=s['geometric_cross_track_m'],offset_xy_m=raw['exact_qpos'][:,:2]-raw['exact_reference_xy'],velocity_error_world_xy_m_s=raw['exact_qvel'][:,:2]-raw['exact_reference_velocity_m_s'],actual_backward50ms_acceleration_world_m_s2=acc,absolute_tilt_deg=s['absolute_tilt_deg'],reference_up_error_deg=s['reference_up_error_deg'],world_z_angular_speed_rad_s=s['world_z_angular_speed_rad_s'],saved_ground_normal_float32=ground,normal_estimate_vs_DEM_proxy_deg=normalerror,estimated_normal_step_deg=eststep,ground_normal_step_deg=groundstep,estimated_loadshares=estshares,pressure_loadshares=pshares,estimated_dominant=estdom,pressure_dominant=pdom,estimated_loadedset_bits=estmask,pressure_loadedset_bits=pmask,estimated_load_valid=estvalid,pressure_load_valid=pvalid,gain_change_contribution_mixed=gain_delta,error_change_contribution_mixed=err_delta,gain_change_contribution_norm_by_caps=np.linalg.norm(gain_delta/caps,axis=1),error_change_contribution_norm_by_caps=np.linalg.norm(err_delta/caps,axis=1),joint_reference_step_rad_s=np.vstack((np.zeros(3),np.diff(raw['exact_reference_qvel'][:,6:9],axis=0))),normalized_requested_excess=excess,any_requested_clipping=np.any(abs(raw['exact_requested'])>caps+1e-12,axis=1),estimated_vs_pressure_equivalent_point_distance_m=np.linalg.norm(raw['exact_estimated_contact_point_m']-raw['truth_equivalent_point_m'],axis=1),saved_DEM_ground_height_float32_m=roll['ground_height_m'].astype(float),saved_instantaneous_slip_speed_float32_m_s=roll['slip_speed_m_s'].astype(float))
  a.update({'lqr_'+k:v for k,v in blocks.items()})
  a['lqr_joint_reference_component_mixed']=np.einsum('nij,nj->ni',K[:,:,15:18],raw['exact_feedback_reference_qvel'][:,6:9]);a['lqr_actual_joint_rate_component_mixed']=-np.einsum('nij,nj->ni',K[:,:,15:18],raw['exact_qvel'][:,6:9])
  selected=list(events['cases'][cid]['events'])
  clock=protocol['evaluation']['clock_by_route'][job['route_id']];low,high=clock['moving_s'];moving=(t>=low-1e-9)&(t<high-1e-9)
  if cid=='eight_m100_N':
   pe=next(v for v in selected if v['kind']=='largest_moving_geometric_peak');pi=pe['source_row'];recovery=recovered_interval(t,a['timed_error_m'],a['geometric_error_m'],moving,pi)
   local_report=dict(case_id=cid,peak_time_s=t[pi],rule='Last>=100ms timed<=20mm AND geometric<=10mm before selected moving geometricpeak; no retuning.',recovered_interval=None)
   if recovery is not None:
    start,end=recovery;local_report['recovered_interval']=dict(first_source_row=start,last_source_row=end,first_s=t[start],last_s=t[end],duration_s=t[end]-t[start]);exitrow=min(end+1,n-1)
    for kind,i in [('late_recovered_interval_start',start),('late_recovered_interval_end',end),('late_recovered_interval_exit',exitrow)]:selected.append(dict(kind=kind,found=True,source_row=i,time_s=t[i]))
    local_report['post_recovery_thresholds']={}
    for kind,key,bound in [('late_geometric_30mm_100ms','geometric_error_m',.030),('late_timed_50mm_100ms','timed_error_m',.050)]:
     found=first_after(t,a[key],bound,exitrow,pi);local_report['post_recovery_thresholds'][kind]=None if found is None else dict(source_row=found[0],time_s=t[found[0]],confirmation_time_s=t[found[1]])
     if found:selected.append(dict(kind=kind,found=True,source_row=found[0],time_s=t[found[0]]))
  event_reports=[]
  context_fields=['time_s','exact_qpos','exact_qvel','exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2','exact_reference_qvel','exact_feedback_reference_qvel','exact_requested','exact_applied','exact_previous_applied','exact_input_bounds','exact_sync_weights','exact_drive_fade','exact_estimated_normal','exact_estimated_support_loads_N','exact_estimated_contact_point_m','truth_patch_loads_N','truth_patch_points_m','truth_equivalent_point_m','exact_effective_phase_rad','exact_observer_wrench','exact_observer_generalized','exact_observer_interval_mean']+['exact_'+k for k in TERMS]
  for j,event in enumerate(selected):
   if not event['found']:event_reports.append(event);continue
   center=float(event['time_s']);item=dict(event=event,snapshot=snapshot(raw,a,event['source_row']),contexts=[window_summary(raw,a,center,w)for w in (1.,.25)])
   ids=np.flatnonzero((t>=center-1-1e-10)&(t<=center+1+1e-10));payload={k:raw[k][ids]for k in context_fields};payload.update({'derived_'+k:v[ids]for k,v in a.items()})
   name=f'{cid}_event{j:02d}_context.npz';np.savez_compressed(OUT/name,**payload);item['context_file']=name;event_reports.append(item)
  full={**a,'requested_command_mixed':raw['exact_requested'],'applied_command_mixed':raw['exact_applied'],'previous_applied_mixed':raw['exact_previous_applied'],'joint_rate_reference_rad_s':raw['exact_reference_qvel'][:,6:9],'joint_rate_actual_rad_s':raw['exact_qvel'][:,6:9],'reference_xy_m':raw['exact_reference_xy'],'actual_xy_m':raw['exact_qpos'][:,:2],'reference_velocity_xy_m_s':raw['exact_reference_velocity_m_s'],'actual_velocity_world_m_s':raw['exact_qvel'][:,:3],'reference_acceleration_xy_m_s2':raw['exact_reference_acceleration_m_s2'],'sync_weights':raw['exact_sync_weights'],'drive_fade':raw['exact_drive_fade'],'estimated_normal':estnormal,'estimated_support_loads_N':raw['exact_estimated_support_loads_N'],'instantaneous_pressure_loads_N':raw['truth_patch_loads_N'],'effective_phase_rad':raw['exact_effective_phase_rad'],'input_bounds_mixed':caps};full.update({'term_'+k:raw['exact_'+k]for k in TERMS});np.savez_compressed(OUT/f'{cid}_timeline.npz',**full)
  reports[cid]=dict(case=job,initial=events['cases'][cid]['initial'],moving_start=events['cases'][cid]['moving_start'],algebraic_max_residuals=checks,rollout_float32_cast_alignment_verified=True,events=event_reports)
 if not all(sha(ROOT/p)==v['sha256']for p,v in bindings.items()):raise ValueError('Frozen input/source changed')
 write(OUT/'report.json',dict(iteration='I-77',cases=reports,late_eight=local_report,source_inputs_unchanged=True,no_new_physics=True,no_control_changes=True,thresholds_are_diagnostics_not_new_acceptance_gates=True,pressure_truth_qualification='Instantaneous last1ms positive normal-pressure aggregate, not full 5ms integrated contact wrench/tangential force routing.',ground_normal_qualification='Savedfloat32 localDEM tangentnearestwheelcentre, rowalignedbyexactfloat32cast; not DART per-contact normals.',same_row_command_is_response_to_recorded_state=True))
 write(OUT/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(OUT)),sha256=sha(p),bytes=p.stat().st_size)for p in sorted(OUT.rglob('*'))if p.is_file()]))
 print(json.dumps(safe(dict(cases=len(reports),late_eight=local_report,all_algebraic_ledgers_pass=True))))
if __name__=='__main__':main()
