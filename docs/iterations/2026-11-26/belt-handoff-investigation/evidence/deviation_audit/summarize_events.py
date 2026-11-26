"""I50 compact saved-data event account; explicit supplemental current geometry."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil,importlib.util,csv
import numpy as np
H=Path(__file__).resolve().parent;ROOT=H.parents[2];OLD=ROOT/'results/belt_capacity_sweep_20261126'
CASES=('mars_cap030','mars_cap060','mars_cap090');TIMES=[6.7,6.725,6.825,6.85,6.86,29.9,30.,30.01,30.04,30.085,31.6,31.65,31.655,31.8,31.815,32.,32.17,32.19,38.64,39.63,40.625,41.,42.11,42.915,43.,43.3,43.725,43.865,43.985,44.]
GROUPS={'position':slice(0,3),'attitude':slice(3,6),'joint_position':slice(6,9),'linear_velocity':slice(9,12),'angular_velocity':slice(12,15),'joint_velocity':slice(15,18)}
LABELS=('shell_A','shell_B','belt_A','belt_B','none');TERMS=('equilibrium_command','rolling_feedforward','support_command','lqr_command','transition_command_delta','heading_command','fade_command_delta')
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
 with np.load(p)as z:return {k:z[k].copy()for k in z.files}
def rotate(q,v):
 q=q/np.linalg.norm(q);c=np.cross(q[1:],v);return v+2*q[0]*c+2*np.cross(q[1:],c)
def angle(a,b):return np.degrees(np.arccos(np.clip(a@b/max(np.linalg.norm(a)*np.linalg.norm(b),1e-30),-1,1)))
def main():
 out=H/'compact_stage03';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'summarize_events.py')
 inputs=[H/'stage01/report.json',H/'stage02_focused/report.json',ROOT/'src/spherical_wheel/experiments.py',ROOT/'src/spherical_wheel/model.py',ROOT/'src/spherical_wheel/offroad_control.py']
 for c in CASES:inputs.extend([OLD/'campaign/runs'/c/f for f in('precision.npz','rollout.npz','provenance.json')]);inputs.append(OLD/'analysis/stage01'/(c+'_geometry.npz'))
 hashes={str(p.relative_to(ROOT)):sha(p)for p in inputs}
 save(out/'declared_before_arithmetic.json',dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(__file__),input_sha256=hashes,fixed_times_s=TIMES,supplement='Current singlepatch planar rollingJacobian from saved exactpose+estimatedpoint. Known co-centeredshaft aboutforkY, shell aboutcarrierX. No model/controller or physics invocation. Verify this analyticmatrix maps SAVEDjointreference to -desiredXY; reportresidual. Scales[5,5,5,.3,.3] areallocationpreferences notlimits. Report nonzero singularvalues/angle andrank for availablepair. First rate>5/10/50/100rads-run containing31.65 toresolve30sleftcensor; no retuneselection.',limits='LoggedDEMnormal is a terrain-referenceproxy, not forceweightedphysicalcollisionnormal. Cancellations andgain/error splits are algebra, notcausalintervention.'))
 a=json.loads((H/'stage01/report.json').read_text());b=json.loads((H/'stage02_focused/report.json').read_text())
 report={'input_sha256':hashes,'scope':'Read-only I49 eventdiagnostic; no new physics/defaults/controller replay.','video18s':a['video_frame360'],'cases':{},'conclusions':{},'limitations':a['limitations']}
 csvrows=[]
 for c in CASES:
  p=load(OLD/'campaign/runs'/c/'precision.npz');l=load(OLD/'campaign/runs'/c/'rollout.npz');g=load(OLD/'analysis/stage01'/(c+'_geometry.npz'));t=p['time_s'];weight=json.loads((OLD/'campaign/runs'/c/'provenance.json').read_text())['weight_N']
  def label(x):return int(np.argmax(x))if x.max()>=.05*weight else 4
  rows=[]
  for at in TIMES:
   i=int(np.argmin(abs(t-at)));j=max(0,i-1);q=p['exact_qpos'][i];v=p['exact_qvel'][i];K=p['exact_feedback_gain'][i];e=p['exact_error'][i];oldK=p['exact_feedback_gain'][j];olde=p['exact_error'][j]
   ed=label(p['exact_estimated_support_loads_N'][i]);td=label(p['truth_patch_loads_N'][i]);normal=p['exact_estimated_normal'][i];actualup=rotate(q[3:7],np.array([0.,0.,1.]));refup=rotate(p['exact_reference_quat'][i],np.array([0.,0.,1.]));den=p['exact_input_bounds'][i]
   row=dict(time_s=t[i],index=i,local_error_m=g['local_path_distance_m'][i],timed_error_m=np.linalg.norm(q[:2]-p['exact_reference_xy'][i]),lag_s=t[i]-g['closest_reference_time_s'][i],projection_boundary=bool(g['artificial_projection_window_boundary'][i]),position_error_heading_m=e[:2],actual_velocity_m_s=v[:3],reference_velocity_m_s=p['exact_reference_velocity_m_s'][i],desired_acceleration_m_s2=p['exact_reference_acceleration_m_s2'][i],angular_velocity_rad_s=v[3:6],reference_joint_rates_rad_s=p['exact_reference_qvel'][i,6:9],actual_joint_rates_rad_s=v[6:9],actual_up=actualup,reference_up=refup,estimated_normal=normal,logged_DEM_normal=l['ground_normal'][i],estimated_vs_DEM_deg=angle(normal,l['ground_normal'][i]),actual_tilt_deg=angle(actualup,np.array([0.,0.,1.])),reference_tilt_deg=angle(refup,np.array([0.,0.,1.])),relative_lean_deg=angle(actualup,refup),estimated_patch=LABELS[ed],true_patch=LABELS[td],estimated_loads_N=p['exact_estimated_support_loads_N'][i],true_loads_N=p['truth_patch_loads_N'][i],requested=p['exact_native_requested'][i],applied=p['exact_applied'][i],capped=abs(p['exact_native_requested'][i])>den+1e-12,sync_weights=p['exact_sync_weights'][i],terms={term:p['exact_'+term][i]for term in TERMS},LQRgroups={name:-K[:,s]@e[s]for name,s in GROUPS.items()},LQRgain_jump=-(K-oldK)@((e+olde)/2),LQRerror_jump=-((K+oldK)/2)@(e-olde),slip_m_s=l['slip_speed_m_s'][i])
   # Analytic rotational pointJacobian for single co-centered smoothsupport.
   axisY=rotate(q[3:7],np.array([0.,1.,0.]));axisX=rotate(q[3:7],np.array([np.cos(q[7]),0.,-np.sin(q[7])]))
   r=p['exact_estimated_contact_point_m'][i]-q[:3];J=np.zeros((2,5));J[:,0]=np.cross(axisY,r)[:2]
   if ed<2:J[:,1+ed]=np.cross(axisX,r)[:2]
   elif ed<4:J[:,1+ed]=axisY[:2]
   else:J[:]=np.nan
   if ed<4:
    A=J*np.array([5.,5.,5.,.3,.3]);s=np.linalg.svd(A,compute_uv=False);cols=np.flatnonzero(np.linalg.norm(A,axis=0)>1e-15)
    geometric_angle=angle(A[:,cols[0]],A[:,cols[1]])if len(cols)==2 else None
    drive=np.r_[p['exact_reference_qvel'][i,6:9],p['exact_rolling_feedforward'][i,3:5]]
    row['planar_rolling_geometry']=dict(matrix_J=J,scaled_singular_values=s,scaled_sigma_min=s[-1],scaled_condition=s[0]/max(s[-1],1e-30),two_active_column_angle_deg=geometric_angle,saved_allocation_planar_residual_m_s=J@drive+p['exact_reference_velocity_m_s'][i],assumption='Current exactpose+globalestimatedpoint; twoactiveanalyticdrives. Singlepatchonly. No terrain/force prediction.')
   rows.append(row)
   flat={'case':c,'time_s':t[i],'local_error_mm':row['local_error_m']*1e3,'timed_error_mm':row['timed_error_m']*1e3,'relative_lean_deg':row['relative_lean_deg'],'estimated_patch':row['estimated_patch'],'true_patch':row['true_patch'],'normal_DEM_delta_deg':row['estimated_vs_DEM_deg']}
   for key in ('actual_velocity_m_s','reference_velocity_m_s','requested','applied','sync_weights','position_error_heading_m'):
    flat.update({key+'_'+str(k):x for k,x in enumerate(row[key])})
   for key,vec in row['LQRgroups'].items():flat.update({'LQR_'+key+'_'+str(k):x for k,x in enumerate(vec)})
   csvrows.append(flat)
  magnitude=abs(p['exact_reference_qvel'][:,6:9]).max(axis=1);peak=int(np.argmin(abs(t-31.65)));rate_runs={}
  for threshold in (5.,10.,50.,100.):
   if magnitude[peak]<=threshold:rate_runs[str(threshold)]=None;continue
   start=peak
   while start>0 and magnitude[start-1]>threshold:start-=1
   rate_runs[str(threshold)]=dict(start_s=t[start],left_censored_at_trace_start=bool(start==0),magnitude_at_start_rad_s=magnitude[start])
  old=a['cases'][c];focus=b[c]
  # Keep only compact fixed summaries, never duplicate megabytes of rows.
  report['cases'][c]=dict(full_max_local={k:old['fulltrace_max_local'][k]for k in('time_s','local_distance_m','timed_error_m','projection_window_boundary')},first_sustained20mm_s=old['first_sustained_local20mm']['onset']['time_s'],late_threshold_onsets_s={k:None if v is None else v['onset']['time_s']for k,v in focus['late_threshold_run_containing_peak'].items()},rate_runs_containing31_65=rate_runs,windows={name:{k:z[k]for k in('timed_rms_m','local_rms_m','local_window_edge_samples','max_relative_lean_deg','request_saturation_duration_s','maximum_sync_weight','max_abs_sync_delta')}for name,z in old['windows'].items()},snapshots=rows)
 report['conclusions']={'command_origin':'Baseline31.650huge torque request is overwhelmingly jointvelocityfeedback fromhuge rollingallocation references; syncdeltaandrollingdirectFF arezero atthatrow.31.655reversal ismostly errorchange, notgainchange.','separate_late_event':'Baseline lateerror alreadycrosses50mm40.625 and100mm42.110 onaccurately identifiedshellB beforefirstsignificantbeltB sample42.915. Laterbeltlimit+modechattercanaggravate butdo notstartthatgrowth.','limits':'Accurate current identity doesnotvalidatecontactnormal/effectivepoint/dynamics/controlpriority. Algebraic cancellationofposition/velocitycorrectionwithattitude/angular/jointfeedback doesnotbyitselfprovewhichtermshouldchange. No exactuniquephysicalcauseidentified.'}
 save(out/'summary.json',report)
 with(out/'event_table.csv').open('w',newline='')as f:w=csv.DictWriter(f,fieldnames=csvrows[0].keys());w.writeheader();w.writerows(csvrows)
 for rel,h in hashes.items():assert sha(ROOT/rel)==h
 save(out/'artifact_manifest.json',{'files':[{'path':str(f.relative_to(out)),'bytes':f.stat().st_size,'sha256':sha(f)}for f in sorted(out.rglob('*'))if f.is_file()and f.name!='artifact_manifest.json']})
 print('I50 compact stage complete')
if __name__=='__main__':main()
