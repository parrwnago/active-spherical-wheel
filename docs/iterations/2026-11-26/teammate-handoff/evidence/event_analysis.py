"""I53 saved I52 m100 event audit: no controller/model/physics calls."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil,csv
import numpy as np
H=Path(__file__).resolve().parent;ROOT=H.parents[2];OLD=ROOT/'results/contact_scheduling_joint_tracking_20261126';CASES=tuple('m100_'+x for x in('B','R','N','NR'));LABELS=('shell_A','shell_B','belt_A','belt_B','none');BOUNDS=np.array([1.5,.6,.6,.3,.3]);SCALE=np.array([5.,5.,5.,.3,.3]);WINDOWS={'playback9_11':[28.,36.],'recorded9_11_alternative':[9.,11.]}
TERMS=('equilibrium_command','rolling_feedforward','support_command','lqr_command','transition_command_delta','heading_command','fade_command_delta');GROUPS={'position':slice(0,3),'attitude':slice(3,6),'joint_position':slice(6,9),'linear_velocity':slice(9,12),'angular_velocity':slice(12,15),'joint_velocity':slice(15,18)}

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return {str(k):safe(x)for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 if isinstance(v,float)and not np.isfinite(v):return None
 return v

def save(p,v):p.write_text(json.dumps(safe(v),indent=2,allow_nan=False)+'\n')
def load(p):
 with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def rotate(q,v):
 q=q/np.linalg.norm(q,axis=1)[:,None];c=np.cross(q[:,1:],v);return v+2*q[:,:1]*c+2*np.cross(q[:,1:],c)
def derivative(t,x,lag):
 out=np.full_like(x,np.nan,dtype=float);out[lag:]=(x[lag:]-x[:-lag])/(t[lag:]-t[:-lag])[:,None];return out

def main():
 out=H/'stage01';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'audit_acceleration.py');mapping=OLD/'campaign/resolved_case_paths.json';paths=json.loads(mapping.read_text())['cases'];folders={c:ROOT/paths[c]for c in CASES}
 inputs=[mapping,OLD/'protocol.json',OLD/'videos/video_metadata.json',OLD/'videos/video_display_schedule.json',OLD/'analysis/stage01/report.json',ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/experiments.py',OLD/'candidate/scheduling_reference.py',ROOT/'results/allocation_handoff_isolation_20261126/reference_candidate/joint_reference_guard.py']
 for c in CASES:inputs.extend([folders[c]/f for f in('precision.npz','rollout.npz','provenance.json','summary.json','candidate_diagnostics.json','artifact_manifest.json')]);inputs.append(OLD/'analysis/stage01'/(c+'_geometry.npz'))
 hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs};save(out/'registered_before_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),input_sha256=hashes,windows_s=WINDOWS,selection='Allfourm100arms andbothfixedwindows; objectivelyselectmaxspeed,bodyangularspeed,5ms/50msplanaracceleration,jointpercomponentacceleration,normalizedcommandstep,timed/localerror,referenceguarddelta. Reportfulltraceerrormaxseparately. Peaksareobservational,notcausalidentification.',derivatives='Backwarddifferencesofsaved5msvelocity, over1or10holds(5/50ms), endingatthecurrentrow. Bodyangularvectorsrotatedtoworldbeforederivative. Rawjerkisseconddifferenceofvelocity, notinstantforce. Commandatrowi−1drivestheintervalendingi; currentcommandicanonlyaffectfuturemotion.',references='20rad/s and100rad/s² areperjointREFERENCEbounds. Noactualjointaccelerationlimitisimplementedbythisguard. First3inputsNm,last2m/s. Allcomparedcaps.30.',geometry='CurrentestimatedpointanalyticjointJacobian only, nativestackedunweightedXYconstraints/scales. Singleglobalpointfloat64,pluralperpatchfloat32; noactualforceprediction/contactquery.'))
 schedule=json.loads((OLD/'videos/video_display_schedule.json').read_text());frames=[next(x for x in schedule if x['frame']==i)for i in(180,200,220)];result={'frame_mapping':frames,'registered_windows_s':WINDOWS,'cases':{},'limits':['Measuredaccelerationisintervalaveragefrom5mslogs, notinstantwithin1mssolveracceleration.','Currentcommand/gainatendpointcannotcausethealreadyobservedvelocityincrement; priorheldcommandisexplicit.','Endpointtruthislast1mssolvercontact, notwholeholdmean.','ReferenceslewboundsdonotboundactualaccelerationorLQRtorque.','Crossarmhistories/configurationsdiffer; temporalassociationdoesnotidentifyuniquephysicalcause.','Nvisualsmoothnessrequiresseparateobjectivechecks; whole-route/turnregressionsremain.']};csvrows=[]
 for case in CASES:
  folder=folders[case];p=load(folder/'precision.npz');l=load(folder/'rollout.npz');g=load(OLD/'analysis/stage01'/(case+'_geometry.npz'));t=p['time_s'];n=len(t);q=p['exact_qpos'];v=p['exact_qvel'];K=p['exact_feedback_gain'];e=p['exact_error'];weight=json.loads((folder/'provenance.json').read_text())['weight_N'];est=p['exact_estimated_support_loads_N'];truth=p['truth_patch_loads_N'];req=p['exact_native_requested'];applied=p['exact_applied']
  omega_world=rotate(q[:,3:7],v[:,3:6]);acc5=derivative(t,v,1);acc50=derivative(t,v,10);alpha5=derivative(t,omega_world,1);jerk5=derivative(t,acc5,1);step=np.vstack([np.zeros(5),np.diff(applied,axis=0)]);rstep=np.vstack([np.zeros(5),np.diff(req,axis=0)]);refacc=derivative(t,p['exact_guarded_joint_reference'],1)
  timed=np.linalg.norm(q[:,:2]-p['exact_reference_xy'],axis=1);local=g['local_path_distance_m'];yaw=np.unwrap(np.arctan2(2*(q[:,3]*q[:,6]+q[:,4]*q[:,5]),1-2*(q[:,5]**2+q[:,6]**2)));ua=rotate(q[:,3:7],np.tile([0.,0.,1.],(n,1)));ur=rotate(p['exact_reference_quat'],np.tile([0.,0.,1.],(n,1)));lean=np.degrees(np.arccos(np.clip(np.sum(ua*ur,axis=1),-1,1)))
  axisY=rotate(q[:,3:7],np.tile([0.,1.,0.],(n,1)));axisX=rotate(q[:,3:7],np.column_stack([np.cos(q[:,7]),np.zeros(n),-np.sin(q[:,7])]))
  assert np.max(abs(sum(p['exact_'+z]for z in TERMS)-req))<1e-9
  groups={name:-np.einsum('nij,nj->ni',K[:,:,cols],e[:,cols])for name,cols in GROUPS.items()};assert np.max(abs(sum(groups.values())-p['exact_lqr_command']))<1e-9
  gj=np.zeros((n,5));ej=gj.copy();gj[1:]=-np.einsum('nij,nj->ni',np.diff(K,axis=0),(e[1:]+e[:-1])/2);ej[1:]=-np.einsum('nij,nj->ni',(K[1:]+K[:-1])/2,np.diff(e,axis=0));assert np.max(abs(gj[1:]+ej[1:]-np.diff(p['exact_lqr_command'],axis=0)))<1e-9
  def dominant(a):return LABELS[int(np.argmax(a))]if a.sum()>=max(.05,.05*weight)else'none'
  def control(i):
   b=max(0,i-1);return dict(time_s=t[i],requested=req[i],applied=applied[i],requested_step=rstep[i],applied_step=step[i],terms={z:p['exact_'+z][i]for z in TERMS},term_steps={z:p['exact_'+z][i]-p['exact_'+z][b]for z in TERMS},LQRgroups={name:x[i]for name,x in groups.items()},LQR_gain_jump=gj[i],LQR_error_jump=ej[i],sync_weights=p['exact_sync_weights'][i],gain_relative_step=np.linalg.norm(K[i]-K[b])/max(np.linalg.norm(K[b]),1e-30),actual_joint_rates_rad_s=v[i,6:9],native_joint_reference_rad_s=p['exact_native_joint_reference'][i],used_joint_reference_rad_s=p['exact_guarded_joint_reference'][i],reference_guard_delta_rad_s=p['exact_reference_guard_delta'][i],used_reference_acceleration_rad_s2=refacc[i],estimated_loads_N=est[i],truth_loads_N=truth[i],estimated_patch=dominant(est[i]),truth_patch=dominant(truth[i]),worldZ_phase_rad=p['exact_phase_world_vertical_rad'][i],normal_phase_rad=p['exact_phase_estimated_normal_rad'][i],used_phase_rad=p['exact_effective_phase_rad'][i],feedback_error=e[i])
  def geometry(i):
   loaded=np.flatnonzero(est[i]>0);rows=[];quality='exact_single'if len(loaded)==1 else'plural_float32'
   if not len(loaded):return dict(available=False,reason='No currentestimatedloadedpatch')
   for j in loaded:
    point=p['exact_estimated_contact_point_m'][i]if len(loaded)==1 else l['estimated_contact_points_m'][i,j].astype(float)
    if not np.isfinite(point).all():return dict(available=False,reason='Nonfinitecurrentestimatedpoint')
    r=point-q[i,:3];J=np.zeros((2,5));J[:,0]=np.cross(axisY[i],r)[:2]
    if j<2:J[:,j+1]=np.cross(axisX[i],r)[:2]
    else:J[:,j+1]=axisY[i,:2]
    rows.extend(J)
   A=np.asarray(rows);sv=np.linalg.svd(A*SCALE,compute_uv=False);rank=int(np.sum(sv>max(sv[0]*1e-7,1e-30)));native=np.r_[p['exact_native_joint_reference'][i],p['exact_rolling_feedforward'][i,3:]];used=np.r_[p['exact_guarded_joint_reference'][i],p['exact_rolling_feedforward'][i,3:]];rhs=np.tile(-p['exact_reference_velocity_m_s'][i],len(loaded))
   return dict(available=True,point_quality=quality,loaded_patches=[LABELS[j]for j in loaded],scaled_singular_values=sv,numerical_rank_at_native_rcond=rank,retained_subspace_condition=sv[0]/max(sv[rank-1],1e-30)if rank else None,native_reference_residual_norm_m_s=np.linalg.norm(A@native-rhs),used_reference_residual_norm_m_s=np.linalg.norm(A@used-rhs),interpretation='StackedcurrentestimatedXYconstraintgeometry, notactualdynamicfeasibility. Multi-patchretainedconditionisnotidenticaltoatwo-dimensionalbodyauthoritymetric.')
  def snapshot(i):
   b=max(0,i-1);return dict(index=int(i),time_s=t[i],interval_start_s=t[b],timed_error_m=timed[i],local_path_distance_m=local[i],local_projection_boundary=g['artificial_projection_window_boundary'][i],actual_xy=q[i,:2],reference_xy=p['exact_reference_xy'][i],desired_velocity_m_s=p['exact_reference_velocity_m_s'][i],desired_acceleration_m_s2=p['exact_reference_acceleration_m_s2'][i],actual_linear_velocity_m_s=v[i,:3],actual_angular_velocity_body_rad_s=v[i,3:6],actual_joint_rates_rad_s=v[i,6:9],measured_acceleration5ms=acc5[i],measured_acceleration50ms=acc50[i],angular_acceleration_world5ms_rad_s2=alpha5[i],measured_jerk5ms=jerk5[i],reference_lean_error_deg=lean[i],yaw_unwrapped_deg=np.degrees(yaw[i]),slip_m_s=l['slip_speed_m_s'][i],normal_force_N=l['normal_force_N'][i],contact_force_N=l['contact_force_N'][i],estimated_normal=p['exact_estimated_normal'][i],current_control=control(i),previous_hold_control=control(b),geometry=geometry(i),clock='Measuredvelocitychangeendingatthisrowfollowedprevious_hold_control; current_controlactsnext.')
  windows={};selected=set(int(np.argmin(abs(t-x)))for x in[9,10,11,28,29,30,30.225,31,31.5,32,32.5,33,34,35,36])
  for name,(lo,hi)in WINDOWS.items():
   ii=np.flatnonzero((t>=lo-1e-9)&(t<=hi+1e-9));events={}
   signals={'max_planar_speed':np.linalg.norm(v[:,:2],axis=1),'max_body_angular_speed':np.linalg.norm(v[:,3:6],axis=1),'max_planar_acceleration5ms':np.linalg.norm(acc5[:,:2],axis=1),'max_planar_acceleration50ms':np.linalg.norm(acc50[:,:2],axis=1),'max_joint_component_acceleration5ms':np.max(abs(acc5[:,6:9]),axis=1),'max_joint_component_acceleration50ms':np.max(abs(acc50[:,6:9]),axis=1),'max_applied_normalized_step':np.max(abs(step)/BOUNDS,axis=1),'max_requested_normalized_step':np.max(abs(rstep)/BOUNDS,axis=1),'max_local_error':local,'max_timed_error':timed,'max_reference_guard_delta':np.max(abs(p['exact_reference_guard_delta']),axis=1),'max_used_joint_reference':np.max(abs(p['exact_guarded_joint_reference']),axis=1)}
   for label,signal in signals.items():
    ix=int(ii[np.nanargmax(signal[ii])]);selected.add(ix);events[label]=snapshot(ix)
    for offset in(-20,-10,-1,1,10,20):selected.add(max(0,min(n-1,ix+offset)))
   windows[name]=dict(samples=len(ii),window_s=[lo,hi],timed_error_rms_m=np.sqrt(np.mean(timed[ii]**2)),local_distance_rms_m=np.sqrt(np.mean(local[ii]**2)),actual_planar_speed_max_m_s=np.linalg.norm(v[ii,:2],axis=1).max(),planar_acceleration5ms_rms_m_s2=np.sqrt(np.mean(np.sum(acc5[ii,:2]**2,axis=1))),planar_acceleration50ms_rms_m_s2=np.sqrt(np.mean(np.sum(acc50[ii,:2]**2,axis=1))),actual_joint_acceleration5ms_max_abs_by_joint_rad_s2=np.max(abs(acc5[ii,6:9]),axis=0),actual_joint_acceleration50ms_max_abs_by_joint_rad_s2=np.max(abs(acc50[ii,6:9]),axis=0),used_reference_acceleration_max_abs_by_joint_rad_s2=np.max(abs(refacc[ii]),axis=0),requested_saturation_samples_by_drive=np.sum(abs(req[ii])>BOUNDS+1e-12,axis=0),max_reference_lean_error_deg=lean[ii].max(),events=events)
   for i in ii:
    row=dict(case=case,window=name,time_s=t[i],timed_error_m=timed[i],local_error_m=local[i],speed_m_s=np.linalg.norm(v[i,:2]),angular_speed_rad_s=np.linalg.norm(v[i,3:6]),acc5_norm_m_s2=np.linalg.norm(acc5[i,:2]),acc50_norm_m_s2=np.linalg.norm(acc50[i,:2]),estimated_patch=dominant(est[i]),truth_patch=dominant(truth[i]),slip_m_s=l['slip_speed_m_s'][i],relative_lean_deg=lean[i])
    for field,vec in [('joint_rate',v[i,6:9]),('joint_acc5',acc5[i,6:9]),('native_ref',p['exact_native_joint_reference'][i]),('used_ref',p['exact_guarded_joint_reference'][i]),('reference_acc',refacc[i])]:row.update({field+'_'+str(j):x for j,x in enumerate(vec)})
    for field,vec in [('requested',req[i]),('applied',applied[i]),('LQR',p['exact_lqr_command'][i]),('sync_delta',p['exact_transition_command_delta'][i])]:row.update({field+'_'+str(j):x for j,x in enumerate(vec)})
    csvrows.append(safe(row))
  peak=int(np.argmax(timed));selected.add(peak);result['cases'][case]=dict(windows=windows,whole_trace_max_timed_error=snapshot(peak),selected_snapshots=[snapshot(i)for i in sorted(selected)])
 save(out/'report.json',result)
 with(out/'window_samples.csv').open('w',newline='')as f:w=csv.DictWriter(f,fieldnames=csvrows[0].keys());w.writeheader();w.writerows(csvrows)
 for rel,h in hashes.items():assert sha(ROOT/rel)==h
 save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
 print('I53eventauditcomplete')
if __name__=='__main__':main()
