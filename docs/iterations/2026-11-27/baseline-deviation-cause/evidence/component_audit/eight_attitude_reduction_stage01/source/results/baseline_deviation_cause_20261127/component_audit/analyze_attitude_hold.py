"""I77 one preregistered eight attitude hold: fixed-target physical comparison."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,importlib.util,json,shutil
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
BRANCH=HERE/'eight_attitude_hold103_stage01'
OLD=ROOT/'results/tangent_heading_routes_20261127'
CASE='eight_m100_N'
OUT=HERE/'eight_attitude_reduction_stage01'
START,STOP,SOURCE,END=19280,19380,19279,20600
ENDPOINTS=(STOP,19500,20427,END)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return{k:safe(x)for k,x in v.items()}
 if isinstance(v,(list,tuple)):return[safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 if isinstance(v,float)and not np.isfinite(v):return None
 return v
def save(p,v):p.write_text(json.dumps(safe(v),indent=2,sort_keys=True,allow_nan=False)+'\n')
def angle(a,b):return np.degrees(np.arccos(np.clip(np.sum(a*b,axis=-1),-1.,1.)))
def main():
 OUT.mkdir(exist_ok=False)
 metric=OLD/'evaluation/metrics.py';source=Path(__file__)
 branch_precision=BRANCH/'child/campaign/runs'/CASE/'precision.npz';native_precision=OLD/'campaign/runs'/CASE/'precision.npz'
 inputs=[branch_precision,BRANCH/'sidecar.npz',BRANCH/'diagnostic_metadata.json',BRANCH/'artifact_manifest.json',BRANCH/'full_precision_prefix_receipt.json',BRANCH/'registered_before_arrays_and_physics.json',native_precision,OLD/'campaign/outputs_sealed.json',HERE/'attitude_hold_protocol.json']
 manifest=json.loads((BRANCH/'artifact_manifest.json').read_text())['files']
 for p in inputs[:3]:
  bound=manifest[str(p.relative_to(BRANCH))]
  if sha(p)!=bound['sha256']or p.stat().st_size!=bound['bytes']:raise AssertionError('Finalbranchinputchanged')
 seal=json.loads((OLD/'campaign/outputs_sealed.json').read_text())['files'][str(native_precision.relative_to(OLD))]
 if sha(native_precision)!=seal['sha256']or native_precision.stat().st_size!=seal['bytes']:raise AssertionError('Originalnativeinputchanged')
 for p in (source,metric,HERE/'attitude_hold_branch.py'):
  dest=OUT/'source'/p.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dest)
 registration=dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),sources={str(p.relative_to(ROOT)):sha(p)for p in (source,metric,HERE/'attitude_hold_branch.py')},inputs={str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in inputs},no_case_arrays_loaded_before_registration=True,hold_window_indices=[START,STOP],endpoint_indices=list(ENDPOINTS),comparison='Fixed preregistered physical hold versus sealedoriginalnative; all79prefields matched before19280. Exactsame savednative targetup used to evaluate both physicalstates; fixedheldtarget alsoevaluated at96.900 before releasecommandacts.',geometric_metric='UnchangedI76 nearest4097uniform-u polyline; timedXYseparate.',scope='One causal diagnostic, not a newcontroller or robustcandidate acceptance; stopping103s is not133s routecompletion.')
 save(OUT/'registered_before_arrays.json',registration)
 spec=importlib.util.spec_from_file_location('_i77_attitude_metrics',metric);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
 fields=('time_s','exact_qpos','exact_qvel','exact_reference_xy','exact_reference_quat','exact_requested','exact_applied','exact_input_bounds','command_held_after_row')
 data={}
 for name,p in [('native',native_precision),('held',branch_precision)]:
  with np.load(p,allow_pickle=False)as z:data[name]={k:z[k][:END+1].copy()for k in fields}
 with np.load(BRANCH/'sidecar.npz',allow_pickle=False)as z:s={k:z[k].copy()for k in z.files}
 if data['native']['time_s'].tobytes()!=data['held']['time_s'].tobytes()or len(s['time_s'])!=END+1:raise AssertionError('Diagnosticclockmismatch')
 fixed_native_up=m.rotation(data['native']['exact_reference_quat'])[:,:,2]
 fixed_held_up=fixed_native_up[SOURCE]
 path=m.geometric_polyline('eight',4097)
 reports={}
 for name,d in data.items():
  t=d['time_s'];xy=d['exact_qpos'][:,:2];indices=np.arange(START,END+1)
  geo=np.full(len(t),np.nan);geo[indices]=m.polyline_distance(xy[indices],path)
  timed=np.linalg.norm(xy-d['exact_reference_xy'],axis=1)
  actual_up=m.rotation(d['exact_qpos'][:,3:7])[:,:,2]
  current_untreated_up=m.rotation(d['exact_reference_quat'])[:,:,2]
  original_target_error=angle(actual_up,fixed_native_up);held_target_error=angle(actual_up,np.broadcast_to(fixed_held_up,actual_up.shape));current_target_error=angle(actual_up,current_untreated_up)
  endpoints={}
  for i in ENDPOINTS:
   endpoints[str(i)]=dict(time_s=t[i],geometric_error_m=geo[i],timed_error_m=timed[i],up_error_to_original_native_target_deg=original_target_error[i],up_error_to_fixed_held_target_deg=held_target_error[i],up_error_to_current_untreated_target_deg=current_target_error[i],xy_m=xy[i],qvel=d['exact_qvel'][i],actual_up=actual_up[i],original_native_target_up=fixed_native_up[i],fixed_held_target_up=fixed_held_up,command_held_after_row=bool(d['command_held_after_row'][i]),state_timing='State precedes newlyemittedcommand;96.900 isbeforereleasecommandacts;103.000finalcommandunheld inbranch.')
  windows={}
  for label,lo,hi in [('intervention_before_release',START,STOP),('post_release',STOP,END),('full96p4_to103',START,END)]:
   ix=np.arange(lo,hi+1);j=int(ix[np.argmax(geo[ix])]);jt=int(ix[np.argmax(timed[ix])])
   windows[label]=dict(first_index=lo,last_index=hi,rows=len(ix),geometric=m.stats(geo[ix]),geometric_peak_index=j,geometric_peak_time_s=t[j],timed=m.stats(timed[ix]),timed_peak_index=jt,timed_peak_time_s=t[jt],up_error_to_original_native_target_deg=m.stats(original_target_error[ix]),up_error_to_fixed_held_target_deg=m.stats(held_target_error[ix]),peak_abs_requested=np.max(np.abs(d['exact_requested'][ix]),axis=0),peak_abs_applied=np.max(np.abs(d['exact_applied'][ix]),axis=0),saturated_command_rows=np.sum(np.abs(d['exact_requested'][ix])>d['exact_input_bounds'][ix]+1e-12,axis=0))
  reports[name]=dict(endpoints=endpoints,windows=windows)
 a=data['held'];b=data['native'];first=START
 immediate=dict(first_index=first,time_s=a['time_s'][first],first_state_delta_qpos=a['exact_qpos'][first]-b['exact_qpos'][first],first_state_delta_qvel=a['exact_qvel'][first]-b['exact_qvel'][first],raw_reference_quat=s['raw_reference_quat'][first],used_reference_quat=s['used_reference_quat'][first],requested_delta_vs_exact_same_state_native=a['exact_requested'][first]-b['exact_requested'][first],applied_delta_vs_exact_same_state_native=a['exact_applied'][first]-b['exact_applied'][first],next_index=first+1,next_state_delta_qpos=a['exact_qpos'][first+1]-b['exact_qpos'][first+1],next_state_delta_qvel=a['exact_qvel'][first+1]-b['exact_qvel'][first+1])
 if np.any(immediate['first_state_delta_qpos'])or np.any(immediate['first_state_delta_qvel']):raise AssertionError('Firsttreatmentstatenotnative')
 release=dict(index=STOP,time_s=a['time_s'][STOP],held_target_up=fixed_held_up,released_current_untreated_target_up=s['used_target_up'][STOP],last_held_used_reference_quat=s['used_reference_quat'][STOP-1],released_used_reference_quat=s['used_reference_quat'][STOP],used_reference_up_jump_deg=angle(s['used_target_up'][STOP],s['used_target_up'][STOP-1]),requested_command_step=a['exact_requested'][STOP]-a['exact_requested'][STOP-1],applied_command_step=a['exact_applied'][STOP]-a['exact_applied'][STOP-1],interpretation='AtSTOPstate theprecedingheldtargetisstillprimaryevaluationtarget; usedtargetrecordedatSTOPbelongsnewreleasecommand. Lateroutcomesincludeinterventionandreturneffects; no continuinghold counterfactualwasrun.')
 report=dict(iteration='I-77',arms=reports,first_applied_intervention=immediate,release=release,changed_reference_rows=np.flatnonzero(s['reference_changed']),limitations=['One fixedhalfsecondhold ofoneeightstart; no candidateadoption/generalization.','Holdingreference isadiagnostic toseparatecontrollerresponsefromterrainforcing, notaproposaltoignoreterrain.','Bothstatesareevaluatedagainstthe SAME saveduntreatednative targetup; heldtargeterrorisreported separately.','Postreleaseeffectsarenotindependentlyrandomized;103sdiagnosticprefixisnotfull133sroutecompletion.'])
 save(OUT/'report.json',report)
 for n,h in registration['sources'].items():
  if sha(ROOT/n)!=h:raise AssertionError('Analysissourcechanged')
 save(OUT/'artifact_manifest.json',dict(files={str(p.relative_to(OUT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in sorted(OUT.rglob('*'))if p.is_file()}))
 print(json.dumps(safe({name:{str(i):{'geo_mm':v['geometric_error_m']*1000,'timed_mm':v['timed_error_m']*1000,'original_target_error_deg':v['up_error_to_original_native_target_deg'],'held_target_error_deg':v['up_error_to_fixed_held_target_deg']}for i,v in r['endpoints'].items()}for name,r in reports.items()}),indent=2))
if __name__=='__main__':main()
