"""I50 saved-array event audit, no controller/model/physics calls."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil,csv
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
OLD=ROOT/'results/belt_capacity_sweep_20261126'
CASES=('mars_cap030','mars_cap060','mars_cap090')
LABELS=('shell_A','shell_B','belt_A','belt_B','none')
WINDOWS={'early_departure':[6.4,7.4],'recorded18_alternative':[17.,19.],'playback18_context':[30.,34.],'handoff_narrow':[31.8,32.1],'late_deviation':[39.,44.]}
FIXED_TIMES=[6.6,6.7,6.725,6.8,6.825,6.85,6.86,6.9,7.,7.4,17.,18.,19.,30.,31.,31.6,31.7,31.8,31.85,31.9,31.95,32.,32.05,32.1,32.2,33.,34.,39.,40.,41.,42.,43.,43.5,43.8,44.]
GROUPS={'position':slice(0,3),'attitude':slice(3,6),'joint_position':slice(6,9),'linear_velocity':slice(9,12),'angular_velocity':slice(12,15),'joint_velocity':slice(15,18)}
TERMS=('equilibrium_command','rolling_feedforward','support_command','lqr_command','transition_command_delta','heading_command','fade_command_delta')
BOUNDS=np.array([1.5,.6,.6,.3,.3]);R=.12;CAP=.12*np.cos(np.radians(15.46600995));GAP=.01

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
 q=q/np.linalg.norm(q,axis=1)[:,None];c=np.cross(q[:,1:],v)
 return v+2*q[:,:1]*c+2*np.cross(q[:,1:],c)
def up(q):return rotate(q,np.broadcast_to([0.,0.,1.],(len(q),3)))
def labels(loads,th):return np.where(loads.max(axis=1)>=th,np.argmax(loads,axis=1),4)
def margin(point,center,axis,label):
 vec=point-center;length=np.linalg.norm(vec,axis=1);coord=np.einsum('ij,ij->i',axis,vec)
 value=np.where(np.isin(label,[0,2]),1.,-1.)*coord*R/np.maximum(length,1e-30)
 lower=np.where(label<2,GAP,CAP);upper=np.where(label<2,CAP,R)
 out=np.minimum(value-lower,upper-value);out[(label==4)|(~np.all(np.isfinite(point),axis=1))]=np.nan
 return out

def main():
 out=HERE/'stage01';out.mkdir(exist_ok=False)
 inputs=[OLD/'videos/video_display_schedule.json',OLD/'analysis/stage01/report.json',ROOT/'config/model.yaml',ROOT/'src/spherical_wheel/offroad_control.py',ROOT/'src/spherical_wheel/contact_sensing.py',ROOT/'src/spherical_wheel/synchronization.py',ROOT/'src/spherical_wheel/model.py']
 for case in CASES:
  inputs +=[OLD/'campaign/runs'/case/f for f in ('precision.npz','rollout.npz','provenance.json','summary.json')]
  inputs +=[OLD/'analysis/stage01'/(case+'_geometry.npz')]
 src=out/'source';src.mkdir();shutil.copyfile(__file__,src/'analyze_deviation.py')
 input_hash={str(p.relative_to(ROOT)):sha(p)for p in inputs}
 save(out/'registered_before_arithmetic.json',dict(registered_utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),input_sha256=input_hash,windows_s=WINDOWS,fixed_snapshot_times_s=FIXED_TIMES,selection='All3caps/allfixedwindows; wholetrace max timed+local error and first sustained20mm local distance >=100ms after1s. Within each fixed window identify max normalized applied/requested step, body angular rate and one-step angular-rate change. Observed peak selections descriptive, not causal estimates.',contact_labels='Rawargmax if maxload>=5%bodyweight; unique>=80%total separately; no de-chatter.',gain_jump_split='Symmetric exact LQR jump decomposition -dK*(e_new+e_old)/2 -(K_new+K_old)/2*de.',truth_clock='Stored last1ms solver contact at5ms endpoint; not full held-interval force. Current point/pose geometry diagnostic with solver-state-clock caveat.',scope='Savedarray only, no model/controller/observer/physics/forecast calls.'))
 schedule=json.loads((OLD/'videos/video_display_schedule.json').read_text());frame=next(x for x in schedule if x['frame']==360)
 result={'video_frame360':frame,'windows_s':WINDOWS,'cases':{},'limitations':['Endpoint1ms contacttruth is not5msmean.','Points are load-weightedmeans not individualcontacts. Radialband margins have geometry+clock caveats.','Chronology does not uniquely identify physicalcause.','No percommand wallclock/sensordelivery/actuatorresponse timestamps.','Full synchronizer target/gap/lookahead notsaved; weights+commanddelta available.']}
 csvrows=[]
 for case in CASES:
  folder=OLD/'campaign/runs'/case;p=load(folder/'precision.npz');l=load(folder/'rollout.npz');g=load(OLD/'analysis/stage01'/(case+'_geometry.npz'))
  t=p['time_s'];n=len(t);weight=json.loads((folder/'provenance.json').read_text())['weight_N'];th=.05*weight
  q=p['exact_qpos'];v=p['exact_qvel'];est=p['exact_estimated_support_loads_N'];truth=p['truth_patch_loads_N'];ed=labels(est,th);td=labels(truth,th)
  e=p['exact_error'];K=p['exact_feedback_gain'];requested=p['exact_native_requested'];applied=p['exact_applied'];bounds=p['exact_input_bounds']
  timed=np.linalg.norm(q[:,:2]-p['exact_reference_xy'],axis=1);local=g['local_path_distance_m'];lag=t-g['closest_reference_time_s'];ua=up(q[:,3:7]);ur=up(p['exact_reference_quat'])
  lean=np.degrees(np.arccos(np.clip(np.einsum('ij,ij->i',ua,ur),-1,1)));absolute=np.degrees(np.arccos(np.clip(ua[:,2],-1,1)))
  axis=rotate(q[:,3:7],np.column_stack([np.cos(q[:,7]),np.zeros(n),-np.sin(q[:,7])]))
  emargin=margin(p['exact_estimated_contact_point_m'],q[:,:3],axis,ed)
  tp=np.full((n,3),np.nan);valid=td<4;tp[valid]=p['truth_patch_points_m'][np.flatnonzero(valid),td[valid]]
  tmargin=margin(tp,q[:,:3],axis,td);pgap=np.linalg.norm(p['exact_estimated_contact_point_m']-p['truth_equivalent_point_m'],axis=1)
  step=np.vstack([np.zeros(5),np.diff(applied,axis=0)]);rstep=np.vstack([np.zeros(5),np.diff(requested,axis=0)]);dw=np.vstack([np.zeros(3),np.diff(v[:,3:6],axis=0)])
  assert np.max(abs(sum(p['exact_'+term]for term in TERMS)-requested))<1e-12
  groups={name:-np.einsum('nij,nj->ni',K[:,:,cols],e[:,cols])for name,cols in GROUPS.items()}
  assert np.max(abs(sum(groups.values())-p['exact_lqr_command']))<1e-12
  gj=np.zeros((n,5));ej=np.zeros((n,5));gj[1:]=-np.einsum('nij,nj->ni',np.diff(K,axis=0),(e[1:]+e[:-1])/2);ej[1:]=-np.einsum('nij,nj->ni',(K[1:]+K[:-1])/2,np.diff(e,axis=0))
  assert np.max(abs(gj[1:]+ej[1:]-np.diff(p['exact_lqr_command'],axis=0)))<1e-12
  def snap(i):
   prev=max(0,i-1)
   d=dict(index=int(i),time_s=t[i],timed_error_m=timed[i],local_distance_m=local[i],lag_s_positive_behind=lag[i],projection_window_boundary=g['artificial_projection_window_boundary'][i],actual_xy=q[i,:2],reference_xy=p['exact_reference_xy'][i],actual_speed_m_s=np.linalg.norm(v[i,:2]),desired_velocity_m_s=p['exact_reference_velocity_m_s'][i],desired_acceleration_m_s2=p['exact_reference_acceleration_m_s2'][i],body_angular_velocity_rad_s=v[i,3:6],body_angular_speed_rad_s=np.linalg.norm(v[i,3:6]),body_angular_velocity_step_rad_s=dw[i],actual_joint_rates_rad_s=v[i,6:9],shaft_angle_rad=q[i,7],effective_phase_rad=l['effective_phase_rad'][i],absolute_tilt_deg=absolute[i],reference_lean_error_deg=lean[i],estimated_normal=p['exact_estimated_normal'][i],logged_dem_normal=l['ground_normal'][i],estimated_patch=LABELS[ed[i]],truth_patch=LABELS[td[i]],estimated_loads_N=est[i],truth_loads_N=truth[i],estimated_unique80=bool(est[i].max()>=.8*est[i].sum()and est[i].max()>=th),truth_unique80=bool(truth[i].max()>=.8*truth[i].sum()and truth[i].max()>=th),point_gap_m=pgap[i],estimated_radial_band_margin_m=emargin[i],truth_patch_mean_radial_band_margin_m=tmargin[i],requested=requested[i],applied=applied[i],applied_step=step[i],requested_step=rstep[i],requested_step_oldbounds=rstep[i]/BOUNDS,applied_step_oldbounds=step[i]/BOUNDS,own_bounds=bounds[i],saturated=abs(requested[i])>bounds[i]+1e-12,sync_weights=p['exact_sync_weights'][i],lqr_gain_change_contribution=gj[i],lqr_error_change_contribution=ej[i],gain_relative_step=np.linalg.norm(K[i]-K[prev])/max(np.linalg.norm(K[prev]),1e-30),feedback_error=e[i],feedback_error_step=e[i]-e[prev],drive_reference=p['exact_allocation_drive_reference'][i],slip_m_s=l['slip_speed_m_s'][i],utilization=l['utilization'][i],normal_force_N=l['normal_force_N'][i],tangential_force_N=l['tangential_force_N'][i],contact_force_N=l['contact_force_N'][i])
   d.update(terms={term:p['exact_'+term][i]for term in TERMS},term_steps={term:p['exact_'+term][i]-p['exact_'+term][prev]for term in TERMS},lqr_groups={name:a[i]for name,a in groups.items()},lqr_group_steps={name:a[i]-a[prev]for name,a in groups.items()})
   return d
  windows={};selected=set(np.argmin(abs(t-at))for at in FIXED_TIMES)
  for name,(lo,hi)in WINDOWS.items():
   ii=np.flatnonzero((t>=lo-1e-9)&(t<=hi+1e-9));events={}
   for label,signal in [('max_timed_error',timed),('max_local_distance',local),('max_applied_step',np.max(abs(step)/BOUNDS,axis=1)),('max_requested_step',np.max(abs(rstep)/BOUNDS,axis=1)),('max_body_angular_speed',np.linalg.norm(v[:,3:6],axis=1)),('max_body_angular_rate_step',np.linalg.norm(dw,axis=1))]:
    ix=int(ii[np.argmax(signal[ii])]);selected.add(ix);events[label]=snap(ix)
   changes=ii[(td[ii]!=td[np.maximum(ii-1,0)])|(ed[ii]!=ed[np.maximum(ii-1,0)])];selected.update(changes.tolist())
   windows[name]=dict(samples=len(ii),timed_rms_m=np.sqrt(np.mean(timed[ii]**2)),local_rms_m=np.sqrt(np.mean(local[ii]**2)),local_window_edge_samples=np.sum(g['artificial_projection_window_boundary'][ii]),max_relative_lean_deg=lean[ii].max(),request_saturation_duration_s=np.sum(abs(requested[ii])>bounds[ii]+1e-12,axis=0)*.005,maximum_sync_weight=np.max(p['exact_sync_weights'][ii],axis=0),max_abs_sync_delta=np.max(abs(p['exact_transition_command_delta'][ii]),axis=0),contact_mode_changes=[snap(int(i))for i in changes],events=events)
   for i in ii:
    row=dict(case=case,window=name,index=int(i),time_s=t[i],timed_error_m=timed[i],local_distance_m=local[i],lag_s=lag[i],projection_boundary=bool(g['artificial_projection_window_boundary'][i]),absolute_tilt_deg=absolute[i],relative_lean_deg=lean[i],angular_speed_rad_s=np.linalg.norm(v[i,3:6]),angular_step_rad_s=np.linalg.norm(dw[i]),estimated_patch=LABELS[ed[i]],truth_patch=LABELS[td[i]],point_gap_m=pgap[i],estimated_margin_m=emargin[i],truth_mean_margin_m=tmargin[i],slip_m_s=l['slip_speed_m_s'][i],utilization=l['utilization'][i])
    for field,arr in [('requested',requested),('applied',applied),('step',step),('sync_weights',p['exact_sync_weights']),('lqr',p['exact_lqr_command']),('rolling',p['exact_rolling_feedforward']),('support',p['exact_support_command']),('sync_delta',p['exact_transition_command_delta']),('gain_change_lqr',gj),('error_change_lqr',ej)]:row.update({field+'_'+str(j):arr[i,j]for j in range(5)})
    for j,label in enumerate(LABELS[:4]):row['truth_'+label+'_N']=truth[i,j];row['estimated_'+label+'_N']=est[i,j]
    csvrows.append(safe(row))
  mask=(t>=1.-1e-9)&(local>.02);starts=np.flatnonzero(mask&~np.r_[False,mask[:-1]]);ends=np.flatnonzero(mask&~np.r_[mask[1:],False]);runs=[(int(a),int(b))for a,b in zip(starts,ends)if t[b]-t[a]>=.1-1e-10]
  first=None if not runs else {'onset':snap(runs[0][0]),'last_s':t[runs[0][1]],'confirmed_at_s':t[runs[0][0]]+.1}
  il=int(np.argmax(local));it=int(np.argmax(timed));selected.update([il,it])
  result['cases'][case]=dict(fulltrace_max_local=snap(il),fulltrace_max_timed=snap(it),first_sustained_local20mm=first,windows=windows,fixed_and_change_snapshots=[snap(int(i))for i in sorted(selected)])
 with(out/'all_window_samples.csv').open('w',newline='')as f:
  writer=csv.DictWriter(f,fieldnames=csvrows[0].keys());writer.writeheader();writer.writerows(csvrows)
 save(out/'report.json',result)
 for rel,h in input_hash.items():assert sha(ROOT/rel)==h
 assert sha(__file__)==sha(src/'analyze_deviation.py')
 save(out/'artifact_manifest.json',{'files':[{'path':str(f.relative_to(out)),'bytes':f.stat().st_size,'sha256':sha(f)}for f in sorted(out.rglob('*'))if f.is_file()and f.name!='artifact_manifest.json']})
 print('I50 deviation audit complete')
if __name__=='__main__':main()
