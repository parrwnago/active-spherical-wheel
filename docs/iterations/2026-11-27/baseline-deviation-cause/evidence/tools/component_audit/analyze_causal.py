"""I77 three fixed straight causal arms; source/input seal precedes arrays/math."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,importlib.util,json,shutil
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
RUNS=HERE/'causal_branches_stage01'
OUT=HERE/'causal_reduction_stage02'
ARMS=('native24','one_pulse','cluster_cap5')
CASE='straight_m100_N'
ENDPOINTS=(4130,4527,4800)

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return{k:safe(x)for k,x in v.items()}
 if isinstance(v,(tuple,list)):return[safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 if isinstance(v,float)and not np.isfinite(v):return None
 return v
def save(p,v):p.write_text(json.dumps(safe(v),indent=2,sort_keys=True,allow_nan=False)+'\n')
def stats(a,t,indices):
 x=a[indices];j=int(np.argmax(x));duration=float(t[indices[-1]]-t[indices[0]])
 return dict(rows=len(indices),first_index=int(indices[0]),last_index=int(indices[-1]),peak_m=float(x[j]),peak_index=int(indices[j]),peak_time_s=float(t[indices[j]]),sample_rms_m=float(np.sqrt(np.mean(x*x))),time_weighted_rms_m=float(np.sqrt(np.trapz(x*x,t[indices])/duration)))
def rows_changed(delta,eligible=None):
 mask=np.any(delta!=0,axis=1)
 if eligible is not None:mask&=eligible
 return np.flatnonzero(mask)

def main():
 OUT.mkdir(exist_ok=False)
 route=ROOT/'results/tangent_heading_routes_20261127/runtime/route.py'
 sources=[Path(__file__),HERE/'causal_branch.py',route]
 inputs=[RUNS/'protocol.json',RUNS/'observed_exits.jsonl',HERE/'causal_reduction_stage01/report.json']
 paths={}
 for arm in ARMS:
  base=RUNS/arm;manifest_path=base/'artifact_manifest.json';manifest=json.loads(manifest_path.read_text())
  paths[arm]=dict(precision=base/'child/campaign/runs'/CASE/'precision.npz',sidecar=base/'sidecar.npz',metadata=base/'diagnostic_metadata.json')
  for p in paths[arm].values():
   binding=manifest['files'][str(p.relative_to(base))]
   if p.stat().st_size!=binding['bytes']or sha(p)!=binding['sha256']:raise AssertionError('Final arm input differs:'+str(p))
  inputs.extend([manifest_path,*paths[arm].values(),base/'registered_before_run.json'])
 for p in sources:
  q=OUT/'source'/p.relative_to(ROOT);q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
 registration=dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),sources={str(p.relative_to(ROOT)):sha(p)for p in sources},inputs={str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in inputs},no_case_arrays_loaded_before_registration=True,window_indices=[4080,4800],window_policy='Inclusive state rows20.400–24.000; all moving on the nominal32s route; final24s command is unheld.',endpoint_indices=list(ENDPOINTS),geometric_metric='Exact nearest point to the full1.2m finite straight segment; timedXYerror separate.',causal_scope='One fixed pulse and one fixed250ms referenceclip; no tuning/adoption. Branch state differences after first applied change are physical consequences, not isolated same-state algebra.')
 save(OUT/'registered_before_arrays.json',registration)
 spec=importlib.util.spec_from_file_location('_i77_causal_analysis_route',route);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);path=m.make_route('straight')
 origin=np.asarray(path.origin);direction=np.asarray(path.direction);length=1.2
 all_data={};all_side={};reports={}
 for arm in ARMS:
  with np.load(paths[arm]['precision'],allow_pickle=False)as z:d={k:z[k].copy()for k in z.files}
  with np.load(paths[arm]['sidecar'],allow_pickle=False)as z:s={k:z[k].copy()for k in z.files}
  t=d['time_s'];p=d['exact_qpos'][:,:2];ref=d['exact_reference_xy']
  if len(t)!=4801 or len(s['time_s'])!=4801 or t.tobytes()!=s['time_s'].tobytes():raise AssertionError('Unexpected causal output clock')
  projection=(p-origin)@direction;closest=origin+np.clip(projection,0,length)[:,None]*direction
  geo=np.linalg.norm(p-closest,axis=1);timed=np.linalg.norm(p-ref,axis=1);cross=(p-origin)@np.array([-direction[1],direction[0]]);along=(p-ref)@direction
  q=d['exact_qpos'][:,3:7];tilt=np.degrees(np.arccos(np.clip(1.-2.*(q[:,1]**2+q[:,2]**2),-1.,1.)))
  valid_changed=s['reference_changed'].astype(bool)
  refrows=np.flatnonzero(valid_changed);requestedrows=rows_changed(s['requested_treatment_delta'],valid_changed);appliedrows=rows_changed(s['applied_treatment_delta'],valid_changed)
  exposure=[]
  for i in appliedrows:
   exposure.append(dict(index=int(i),time_s=t[i],raw_feedback_reference_before=s['native_feedback_reference_qvel'][i,6:9],used_feedback_reference=s['used_feedback_reference_qvel'][i,6:9],requested_delta=s['requested_treatment_delta'][i],applied_delta=s['applied_treatment_delta'][i],same_state_native_applied=s['same_state_native_applied'][i],actual_applied=s['actual_applied'][i]))
  endpoints={}
  for i in ENDPOINTS:
   endpoints[str(i)]=dict(index=i,time_s=t[i],geometric_error_m=geo[i],timed_error_m=timed[i],signed_cross_track_m=cross[i],signed_along_timed_error_m=along[i],xy_m=p[i],reference_xy_m=ref[i],qvel=d['exact_qvel'][i],tilt_deg=tilt[i],command_held_after_row=bool(d['command_held_after_row'][i]),requested=d['exact_requested'][i],applied=d['exact_applied'][i],reference_changed=bool(valid_changed[i]),command_interpretation='State precedes this row command; at4130 the intervention is released, at4800 emitted command is not held.')
  windows={}
  for label,lo,hi in [('full_20p4_to24',4080,4800),('armed_window_before_release',4080,4130),('post_release',4130,4800)]:
   indices=np.arange(lo,hi+1)
   windows[label]=dict(geometric=stats(geo,t,indices),timed=stats(timed,t,indices),peak_tilt_deg=float(np.max(tilt[indices])),peak_abs_body_angular_rate_rad_s=np.max(np.abs(d['exact_qvel'][indices,3:6]),axis=0),peak_abs_requested=np.max(np.abs(d['exact_requested'][indices]),axis=0),peak_abs_applied=np.max(np.abs(d['exact_applied'][indices]),axis=0))
  first=None if not len(appliedrows)else int(appliedrows[0]);immediate=None
  if first is not None:
   immediate=dict(changed_command_index=first,time_s=t[first],delta_at_current_state_qpos=d['exact_qpos'][first]-all_data['native24']['exact_qpos'][first],delta_at_current_state_qvel=d['exact_qvel'][first]-all_data['native24']['exact_qvel'][first],next_index=first+1,next_time_s=t[first+1],next_state_delta_qpos=d['exact_qpos'][first+1]-all_data['native24']['exact_qpos'][first+1],next_state_delta_qvel=d['exact_qvel'][first+1]-all_data['native24']['exact_qvel'][first+1])
  release=None
  if arm!='native24':
   release_index=4118 if arm=='one_pulse' else 4130
   j=release_index;i=j-1
   release=dict(last_armed_index=i,last_armed_time_s=t[i],last_armed_raw_joint_reference=s['raw_reference_qvel'][i,6:9],last_armed_used_joint_reference=s['used_feedback_reference_qvel'][i,6:9],last_armed_reference_actually_changed=bool(valid_changed[i]),last_armed_applied=d['exact_applied'][i],last_actual_reference_changed_index=int(refrows[-1]),first_released_index=j,first_released_time_s=t[j],released_joint_reference=s['used_feedback_reference_qvel'][j,6:9],released_applied=d['exact_applied'][j],released_command_step=d['exact_applied'][j]-d['exact_applied'][i],release_state_delta_qpos=d['exact_qpos'][j]-all_data['native24']['exact_qpos'][j],release_state_delta_qvel=d['exact_qvel'][j]-all_data['native24']['exact_qvel'][j],first_post_release_state_index=j+1,first_post_release_state_delta_qpos=d['exact_qpos'][j+1]-all_data['native24']['exact_qpos'][j+1],first_post_release_state_delta_qvel=d['exact_qvel'][j+1]-all_data['native24']['exact_qvel'][j+1],note='A return to native reference feedback at an already branched state, not separately randomized against continuing the intervention. For cluster, the preceding row was already unchanged by the cap; the nominal window boundary does not itself switch the command formula.')
  reports[arm]=dict(windows=windows,endpoints=endpoints,reference_changed_rows=refrows,requested_changed_rows=requestedrows,applied_changed_rows=appliedrows,applied_exposures=exposure,first_applied_immediate_response=immediate,release=release)
  all_data[arm]=d;all_side[arm]=s
 comparisons={}
 for arm in ARMS[1:]:
  comparisons[arm]={}
  for i in ENDPOINTS:
   a=reports[arm]['endpoints'][str(i)];b=reports['native24']['endpoints'][str(i)]
   comparisons[arm][str(i)]=dict(geometric_error_delta_m=a['geometric_error_m']-b['geometric_error_m'],timed_error_delta_m=a['timed_error_m']-b['timed_error_m'],xy_delta_m=np.asarray(a['xy_m'])-np.asarray(b['xy_m']),tilt_delta_deg=a['tilt_deg']-b['tilt_deg'])
 report=dict(iteration='I-77',registered_before_arrays_sha256=sha(OUT/'registered_before_arrays.json'),arms=reports,paired_endpoint_differences=comparisons,limitations=['Three fixed diagnosticarms of one start/route, not robustness or acceptance evidence.','A changed reference may be attenuated by preparation/fade/clipping; actual exposure is explicitly measured.','Postrelease outcomes combine prior physical effects and return to native feedback; release is not an isolated causal factor.','No finite-gain tuning, observer replacement, contactoracle or new controller was tested.'])
 prior=json.loads((HERE/'causal_reduction_stage01/report.json').read_text())
 for arm in ARMS:
  for key in ('windows','endpoints','reference_changed_rows','requested_changed_rows','applied_changed_rows','applied_exposures','first_applied_immediate_response'):
   if safe(reports[arm][key])!=prior['arms'][arm][key]:raise AssertionError('Expandedreleasecontext changed previous primary reduction:'+arm+'/'+key)
 report['prior_reduction_primary_metrics_bit_equivalent']=True
 report['followup_scope']='Extends explicit release context to both treatments, preserving all stage01 primary values and rawinputs.'
 save(OUT/'report.json',report)
 for name,h in registration['sources'].items():
  if sha(ROOT/name)!=h:raise AssertionError('Analysis source changed')
 save(OUT/'artifact_manifest.json',dict(files={str(p.relative_to(OUT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in sorted(OUT.rglob('*'))if p.is_file()}))
 print(json.dumps(safe({arm:{str(i):reports[arm]['endpoints'][str(i)]['geometric_error_m']for i in ENDPOINTS}for arm in ARMS}),indent=2))

if __name__=='__main__':main()
