"""Independent I77 recorded-field prefix and branch-isolation verification.

Native24 is compared with original N through24s, not through the full45s route.
The final emitted command at24s is unheld here but held in the original45s run.
Candidate commands match before their first actual changed reference; physical
state/native feedback at that first row must still match inclusively.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
OLD=ROOT/'results/tangent_heading_routes_20261127'
CASE='straight_m100_N'
FIRST={'native24':None,'one_pulse':4117,'cluster_cap5':4083}
STATE_FIELDS=('time_s','exact_qpos','exact_qvel','exact_previous_applied','exact_nominal_joints_before','exact_observer_wrench','exact_observer_generalized','exact_observer_interval_mean','exact_estimated_normal','exact_estimated_contact_point_m','exact_estimated_support_loads_N','truth_time_s','truth_patch_loads_N','truth_patch_points_m','truth_patch_contact_counts','truth_equivalent_point_m','exact_reference_qpos','exact_reference_qvel','exact_feedback_gain','exact_support_command','exact_rolling_feedforward','exact_effective_phase_rad')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return {str(k):safe(x)for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 if isinstance(v,float)and not np.isfinite(v):return None
 return v
def save(p,v):
 with p.open('x')as f:json.dump(safe(v),f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def exact(a,b):return a.dtype==b.dtype and a.shape==b.shape and np.ascontiguousarray(a).tobytes()==np.ascontiguousarray(b).tobytes()
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--arm',choices=FIRST,required=True);parser.add_argument('--runs-root',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
 arm=args.arm;folder=args.runs_root.resolve()/arm;out=args.output.resolve();produced=folder/'child/campaign/runs'/CASE/'precision.npz';baseline=OLD/'campaign/runs'/CASE/'precision.npz'
 manifest=json.loads((folder/'artifact_manifest.json').read_text())
 for rel,b in manifest['files'].items():
  p=folder/rel
  if p.stat().st_size!=b['bytes']or sha(p)!=b['sha256']:raise ValueError('Arm manifest differs:'+rel)
 oldseal=json.loads((OLD/'campaign/outputs_sealed.json').read_text());binding=oldseal['files'][str(baseline.relative_to(OLD))]
 if baseline.stat().st_size!=binding['bytes']or sha(baseline)!=binding['sha256']:raise ValueError('Original precision changed')
 inputs=[Path(__file__).resolve(),baseline,OLD/'campaign/outputs_sealed.json',produced,folder/'artifact_manifest.json',folder/'diagnostic_metadata.json',folder/'sidecar.npz']
 bindings={str(p):dict(sha256=sha(p),bytes=p.stat().st_size)for p in inputs}
 out.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),out/'verify_causal_precision.py')
 save(out/'registered_before_array_reads.json',dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),arm=arm,inputs=bindings,expected_horizon_s=24.,expected_rows=4801,first_actual_reference_change=FIRST[arm],native_terminalmask_exception='Original row4800 heldtrue; diagnostic row4800 is emitted-unheldfalse. All preceding held flags and other fields must match.',no_new_physics=True,no_control_change=True))
 with np.load(baseline,allow_pickle=False)as z:old={k:z[k].copy()for k in z.files}
 with np.load(produced,allow_pickle=False)as z:new={k:z[k].copy()for k in z.files}
 with np.load(folder/'sidecar.npz',allow_pickle=False)as z:side={k:z[k].copy()for k in z.files}
 if set(old)!=set(new):raise ValueError('Shared precision field sets differ')
 t=new['time_s'];n=len(t);first=FIRST[arm]
 if n!=4801 or abs(float(t[-1])-24.)>1e-8:raise ValueError('Wrong actual24s prefix')
 end=n if first is None else first
 checks=[]
 for key in sorted(new):
  expected=old[key][:end].copy()
  exception=first is None and key=='command_held_after_row'
  if exception:
   if not bool(expected[-1])or bool(new[key][-1]):raise ValueError('Wrong explicit24s terminal-mask exception')
   expected[-1]=False
  checks.append(dict(field=key,exact=exact(new[key][:end],expected),rows=end,terminal_mask_exception=exception))
 statechecks=[]
 if first is not None:
  for key in STATE_FIELDS:statechecks.append(dict(field=key,rows=first+1,exact=exact(new[key][:first+1],old[key][:first+1])))
  for key,a,b in [('native_error',side['native_error'][first],old['exact_error'][first]),('native_feedback_reference',side['native_feedback_reference_qvel'][first],old['exact_feedback_reference_qvel'][first]),('native_feedback_command',side['native_feedback_command'][first],old['exact_equilibrium_command'][first]+old['exact_rolling_feedforward'][first]+old['exact_support_command'][first]-old['exact_feedback_gain'][first]@old['exact_error'][first])]:
   statechecks.append(dict(field=key,rows=1,exact=exact(a,b)))
 expected_ref=side['native_feedback_reference_qvel'].copy()
 if arm=='one_pulse':expected_ref[4117,6:9]=old['exact_feedback_reference_qvel'][4116,6:9]
 elif arm=='cluster_cap5':expected_ref[4080:4130,6:9]=np.clip(expected_ref[4080:4130,6:9],-5.,5.)
 expected_error=side['native_error'].copy();changed=np.any(expected_ref!=side['native_feedback_reference_qvel'],axis=1)
 expected_error[changed,15:18]=new['exact_qvel'][changed,6:9]-expected_ref[changed,6:9]
 isolation=dict(used_reference_matches_fixed_rule=exact(new['exact_feedback_reference_qvel'],expected_ref),used_error_matches_fixed_rule=exact(new['exact_error'],expected_error),error_outside_joint_rate_indices_unchanged=exact(new['exact_error'][:,:15],side['native_error'][:,:15]),reference_outside_joint_rate_indices_unchanged=exact(new['exact_feedback_reference_qvel'][:,:6],side['native_feedback_reference_qvel'][:,:6]),history_unchanged_by_override=exact(side['history_after_native_feedback_sha256'],side['history_after_override_sha256']),raw_reference_matches_sidecar=exact(new['exact_reference_qvel'],side['raw_reference_qvel']),no_heading_command=bool(np.all(new['exact_heading_command']==0.)),exactly_one_observer_update=bool(np.all(new['native_observer_update_count']==1)),no_truth_or_terrain_command_read=bool(np.all(new['no_truth_or_terrain_read_during_command'])))
 indices=np.flatnonzero(changed);actualfirst=None if not len(indices)else int(indices[0])
 isolation['first_actual_reference_change_matches_frozen_rule']=actualfirst==first
 delta=side['actual_applied']-side['same_state_native_applied'];exposure=np.flatnonzero(np.max(abs(delta),axis=1)>1e-12)
 requested_delta=side['actual_requested']-side['same_state_native_requested'];request_exposure=np.flatnonzero(np.max(abs(requested_delta),axis=1)>1e-12)
 rawpoint=new['exact_qpos'][:,:2];reference=new['exact_reference_xy'];timed=np.linalg.norm(rawpoint-reference,axis=1)
 start,goal=old['exact_reference_xy'][0],old['exact_reference_xy'][-1];segment=goal-start
 u=np.clip((rawpoint-start)@segment/(segment@segment),0.,1.);geometric=np.linalg.norm(rawpoint-(start+u[:,None]*segment),axis=1)
 endpoints={}
 for index in (4130,4527,4800):
  endpoints[f'{t[index]:.3f}']=dict(index=index,time_s=t[index],timed_error_m=timed[index],geometric_error_m=geometric[index],point_xy_m=rawpoint[index],reference_xy_m=reference[index],state_before_current_command=True,goal_completion_claim=False)
 scope=np.arange(n)>=4083;ids=np.flatnonzero(scope);peak=int(ids[np.argmax(geometric[ids])])
 passed=all(c['exact']for c in checks+statechecks)and all(isolation.values())
 result=dict(iteration='I-77',arm=arm,passed=passed,rows=n,precision_field_count=len(new),complete_command_prefix_stop_exclusive=end,complete_prefix_field_checks=checks,first_change_inclusive_state_and_native_feedback_checks=statechecks,isolation=isolation,actual_first_reference_change_index=actualfirst,reference_changed_rows=len(indices),requested_effect_rows_over1e_12=len(request_exposure),applied_effect_rows_over1e_12=len(exposure),first_applied_effect_index=None if not len(exposure)else int(exposure[0]),reference_change_times_s=t[indices],actual_applied_change_times_s=t[exposure],maximum_applied_delta_by_channel=np.max(abs(delta),axis=0),endpoints=endpoints,post4083_geometric_peak=dict(time_s=t[peak],error_m=geometric[peak]),no_new_physics=True,full45s_reproduction_claim=False,scope='Measured-state and declared precision-field prefix plus intervention algebra only; current-state native counterfactual is not the historical native trajectory after branch.')
 if any(sha(Path(p))!=v['sha256']for p,v in bindings.items()):raise ValueError('Verification source/input changed')
 save(out/'report.json',result);save(out/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)for p in sorted(out.iterdir())if p.is_file()and p.name!='artifact_manifest.json']))
 print(json.dumps(safe({k:result[k]for k in ['arm','passed','precision_field_count','actual_first_reference_change_index','reference_changed_rows','applied_effect_rows_over1e_12','first_applied_effect_index','maximum_applied_delta_by_channel','endpoints','post4083_geometric_peak']})))
 if not passed:raise SystemExit(2)
if __name__=='__main__':main()
