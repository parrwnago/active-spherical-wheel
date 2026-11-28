"""I77 exact-prefix, short Gazebo branches of the original I76 native N.

No production source is changed. Inactive feedback returns native bytes.
A treatment changes only selected feedback joint-rate references/errors;
raw allocation, its caches, observer and all remaining command stages persist.
"""
from pathlib import Path
from datetime import datetime,timezone
from contextlib import ExitStack
from unittest.mock import patch
import argparse,copy,hashlib,importlib.util,json,os,shutil,subprocess,sys,traceback
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
ORIGINAL=ROOT/'results/tangent_heading_routes_20261127'
CASE='straight_m100_N'
ARMS=('native24','one_pulse','cluster_cap5')
ARM_SPECS=dict(native24=dict(kind='native'),one_pulse=dict(kind='hold_saved_previous_feedback_joint_reference',start_index=4117,stop_index_exclusive=4118,source_index=4116,joint_velocity_dofs=[6,7,8]),cluster_cap5=dict(kind='clip_feedback_joint_reference',start_index=4080,stop_index_exclusive=4130,cap_rad_s=5.,joint_velocity_dofs=[6,7,8]))
PREFIX_FIELDS=('exact_requested','exact_applied','exact_reference_qpos','exact_reference_qvel','exact_feedback_reference_qvel','exact_feedback_gain','exact_error','exact_lqr_command','exact_equilibrium_command','exact_rolling_feedforward','exact_support_command','exact_transition_command_delta','exact_heading_command','exact_fade_command_delta','exact_clipping_command_delta','exact_sync_weights','exact_estimated_support_loads_N','exact_estimated_contact_point_m','exact_observer_wrench','exact_observer_generalized','exact_observer_interval_mean','exact_estimated_normal','exact_effective_phase_rad','exact_drive_fade')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(v):
 if isinstance(v,dict):return{str(k):safe(x)for k,x in v.items()}
 if isinstance(v,(list,tuple)):return[safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 if isinstance(v,Path):return str(v)
 if isinstance(v,float)and not np.isfinite(v):return None
 return v

def save(p,v):Path(p).write_text(json.dumps(safe(v),indent=2,sort_keys=True,allow_nan=False)+'\n')
def token(a):
 a=np.asarray(a);return a.dtype.str,a.shape,np.ascontiguousarray(a).tobytes()
def digest(v):return hashlib.sha256(repr(snapshot(v)).encode()).hexdigest()
def snapshot(v):
 if isinstance(v,dict):return tuple((k,snapshot(v[k]))for k in sorted(v))
 if isinstance(v,(tuple,list)):return tuple(snapshot(x)for x in v)
 if v is None or isinstance(v,(str,bool,int,float)):return v
 return token(v)
def history(c):
 names=('nominal_joints','_vref','_held','_sync','heading_reference','_slow_normal','_fall_end','_yaw_error','_plane')
 h={k:getattr(c,k)for k in names}
 h['estimator']=vars(c.estimator).copy()
 h['observer']={k:getattr(c.observer,k)for k in ('generalized','interval_mean','_previous')}
 h['synchronizer_plane']=c.synchronizer._plane
 return digest(h)
def load(p,name):
 spec=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m

def treatment(arm,index,native_ref,held):
 if arm=='native24':return None
 if arm=='one_pulse':return np.array(held,copy=True)if index==4117 else None
 if arm=='cluster_cap5':return np.clip(native_ref,-5.,5.)if 4080<=index<4130 else None
 raise ValueError('Only preregistered arms are allowed')

def replace_feedback_reference(controller,native_command,native_fb,feedforward,replacement):
 if replacement is None or np.array_equal(np.asarray(replacement),native_fb['reference_qvel'][6:9]):return native_command,native_fb
 used_ref=np.array(native_fb['reference_qvel'],copy=True);used_ref[6:9]=replacement
 used_error=np.array(native_fb['error'],copy=True);used_error[15:18]=controller.plant.data.qvel[6:9]-used_ref[6:9]
 used_command=(np.asarray(controller.ordinary['equilibrium']['u'])+np.asarray(feedforward)+np.asarray(native_fb['support_torque'])-native_fb['K']@used_error)
 used_fb=dict(native_fb);used_fb['error']=used_error;used_fb['reference_qvel']=used_ref
 return used_command,used_fb

def native_final_counterfactual(controller,native_feedback_command,info):
 # Preview target velocities/weights depend on state, records and plane; primary
 # command only initializes zero-weight entries, so reuse the current targets.
 sync=controller._sync;weights=np.asarray(sync['blend_weights'])
 post_sync=(1.-weights)*native_feedback_command+weights*np.asarray(sync['commands'])
 post_heading=post_sync+np.asarray(info['heading_command'])
 requested,fade=controller._fade_when_unloaded(post_heading,info['estimate'],info['reference_qvel'])
 return requested,np.clip(requested,-controller.plant.bounds,controller.plant.bounds)

def register(protocol_path,stage):
 protocol=json.loads(protocol_path.read_text())
 if protocol['iteration']!='I-77' or protocol['case_id']!=CASE or protocol['horizon_s']!=24. or tuple(protocol['arms'])!=ARMS or protocol['interventions']!=ARM_SPECS:raise ValueError('Frozen I77 diagnostic schema differs')
 original_protocol=ORIGINAL/'protocol/protocol.json'
 if sha(original_protocol)!='53106cdb758d76436e04b747c9483c4081abcc7ec1f2a163ace90b58b82085c2':raise ValueError('I76 source protocol changed')
 p=json.loads(original_protocol.read_text());seal_path=ORIGINAL/'campaign/outputs_sealed.json';seal=json.loads(seal_path.read_text())
 baseline=ORIGINAL/'campaign/runs'/CASE/'precision.npz';expected=seal['files'][str(baseline.relative_to(ORIGINAL))]
 if baseline.stat().st_size!=expected['bytes']or sha(baseline)!=expected['sha256']:raise ValueError('Native saved recording differs from I76 seal')
 sourcefiles=[Path(__file__),protocol_path,ORIGINAL/'campaign_tools/run_case.py']+[ROOT/name for name in p['source_sha256']]
 inputs=[baseline,original_protocol,seal_path,ROOT/p['gain_cache']['path']]
 for name,h in p['source_sha256'].items():
  if sha(ROOT/name)!=h:raise ValueError('Native registered source changed:'+name)
 for name,h in p['input_sha256'].items():
  if sha(ROOT/name)!=h:raise ValueError('Native registered input changed:'+name)
 stage.mkdir(parents=True,exist_ok=False)
 for f in sorted(set(sourcefiles)):
  dest=stage/'source'/f.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dest)
 registered=dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),protocol_sha256=sha(protocol_path),sources={str(f.relative_to(ROOT)):sha(f)for f in sorted(set(sourcefiles))},inputs={str(f.relative_to(ROOT)):sha(f)for f in inputs},original_I76_protocol_sha256=sha(original_protocol),original_I76_output_seal_sha256=sha(seal_path),no_arrays_loaded_before_registration=True,actual_requested_horizon_s=24.,nominal_path_horizon_s=45.,arms=protocol['arms'],arm_specs=ARM_SPECS,no_production_changes=True)
 save(stage/'registered_before_run.json',registered);return protocol,p,registered

def run_arm(arm,protocol_path,stage):
 protocol,p,registered=register(protocol_path,stage)
 child=stage/'child';child.mkdir()
 for folder in ('adapter','runtime','protocol'):
  (child/folder).mkdir()
 for folder,names in {'adapter':['tangent_heading_adapter.py'],'runtime':['route.py','heading_reference.py'],'protocol':['protocol.json']}.items():
  for name in names:
   source=ORIGINAL/folder/name;dest=child/folder/name;shutil.copy2(source,dest)
   if sha(dest)!=sha(source):raise AssertionError('Executed child source copy differs')
 copied={str(q.relative_to(stage)):sha(q)for q in child.rglob('*')if q.is_file()}
 save(stage/'child_execution_bindings.json',dict(files=copied,original_control_protocol='I76 source/config frozen; I77 sidecar overrides inherited iteration/full-horizon claims.'))
 with np.load(ORIGINAL/'campaign/runs'/CASE/'precision.npz',allow_pickle=False)as z:
  required=('time_s','exact_qpos','exact_qvel','exact_previous_applied','exact_nominal_joints_before',*PREFIX_FIELDS)
  expected={key:z[key].copy()for key in required}
 held=expected['exact_feedback_reference_qvel'][4116,6:9].copy()
 prefix_end=4801 if arm=='native24'else 4117 if arm=='one_pulse'else 4080
 rows=[];capture={};prefix_checks=0;state_prefix_checks=0;native_feedback_prefix_checks=0
 first_active=None if arm=='native24' else 4117 if arm=='one_pulse' else 4080
 state_prefix_end_inclusive=4800 if first_active is None else first_active
 import spherical_wheel.gazebo.experiments as gx
 from spherical_wheel.offroad_control import OffroadController
 original=load(ORIGINAL/'campaign_tools/run_case.py','_i77_original_i76_runner')
 true_command=OffroadController.command;true_feedback=OffroadController._terrain_feedback;true_run=gx.run_gazebo
 def feedback(c,qref,vref,ff,normal,measurement,touching):
  nonlocal native_feedback_prefix_checks
  index=int(round(float(c.plant.data.time)/c.plant.dt))
  native_command,native_fb=true_feedback(c,qref,vref,ff,normal,measurement,touching)
  before=history(c);native_error=native_fb['error'].copy();native_ref=native_fb['reference_qvel'].copy()
  if index<=state_prefix_end_inclusive:
   native_expected=(expected['exact_equilibrium_command'][index]+expected['exact_rolling_feedforward'][index]+expected['exact_support_command'][index]-expected['exact_feedback_gain'][index]@expected['exact_error'][index])
   for key,a,b in [('native_error',native_error,expected['exact_error'][index]),('native_feedback_ref',native_ref,expected['exact_feedback_reference_qvel'][index]),('native_K',native_fb['K'],expected['exact_feedback_gain'][index]),('native_feedback_command',native_command,native_expected)]:
    if token(a)!=token(b):raise AssertionError(f'First-active-inclusive native feedback differs at{index}/{key}')
   native_feedback_prefix_checks+=1
  replacement=treatment(arm,index,native_ref[6:9],held)
  used_command,used_fb=replace_feedback_reference(c,native_command,native_fb,ff,replacement)
  after=history(c)
  if after!=before:raise AssertionError('Feedback-only treatment mutated native history/cache')
  capture.clear();capture.update(native_feedback_command=np.array(native_command,copy=True),used_feedback_command=np.array(used_command,copy=True),native_error=native_error,used_error=np.array(used_fb['error'],copy=True),native_feedback_reference_qvel=native_ref,used_feedback_reference_qvel=np.array(used_fb['reference_qvel'],copy=True),feedback_override_active=replacement is not None,reference_changed=not np.array_equal(native_ref,used_fb['reference_qvel']),history_after_native_feedback_sha256=before,history_after_override_sha256=after,held_raw_vref=np.zeros(9)if c._held is None else c._held['vref'].copy())
  return used_command,used_fb
 def command(c,xy,velocity,acceleration):
  nonlocal prefix_checks,state_prefix_checks
  index=int(round(float(c.plant.data.time)/c.plant.dt));before=history(c)
  if index<=state_prefix_end_inclusive:
   if token(np.asarray(float(c.plant.data.time)))!=token(expected['time_s'][index]):raise AssertionError('Native prefix clock differs')
   for key,value in [('exact_qpos',c.plant.data.qpos),('exact_qvel',c.plant.data.qvel),('exact_previous_applied',c.plant.last_input),('exact_nominal_joints_before',c.nominal_joints)]:
    if token(value)!=token(expected[key][index]):raise AssertionError(f'Native state/history prefix differs at{index}/{key}')
   state_prefix_checks+=1
  applied,info=true_command(c,xy,velocity,acceleration)
  if index<prefix_end:
   current={**{'exact_'+key:np.asarray(info[key])for key in ('requested','reference_qpos','reference_qvel','feedback_reference_qvel','feedback_gain','error','lqr_command','equilibrium_command','rolling_feedforward','support_command','transition_command_delta','heading_command','fade_command_delta','clipping_command_delta','sync_weights','estimated_support_loads_N','estimated_contact_point_m','effective_phase_rad','drive_fade')},'exact_applied':applied,'exact_observer_wrench':np.r_[c.observer.wrench()],'exact_observer_generalized':c.observer.generalized,'exact_observer_interval_mean':c.observer.interval_mean,'exact_estimated_normal':info['estimate']['normal']}
   for key in PREFIX_FIELDS:
    if token(current[key])!=token(expected[key][index]):raise AssertionError(f'Native command prefix differs at{index}/{key}')
   prefix_checks+=1
  counterfactual_history_before=history(c)
  native_request,native_applied=native_final_counterfactual(c,capture['native_feedback_command'],info)
  if history(c)!=counterfactual_history_before:raise AssertionError('Stateless command-stage counterfactual mutated history')
  if not capture['reference_changed']:
   np.testing.assert_allclose(native_request,info['requested'],rtol=0,atol=2e-14)
   np.testing.assert_allclose(native_applied,applied,rtol=0,atol=2e-14)
  rows.append(dict(time_s=float(c.plant.data.time),control_index=index,history_before_command_sha256=before,history_after_command_sha256=history(c),**{k:np.asarray(v).copy()for k,v in capture.items()},actual_requested=np.asarray(info['requested']).copy(),actual_applied=np.asarray(applied).copy(),same_state_native_requested=native_request,same_state_native_applied=native_applied,applied_treatment_delta=np.asarray(applied)-native_applied,requested_treatment_delta=np.asarray(info['requested'])-native_request,raw_reference_qvel=np.asarray(info['reference_qvel']).copy()))
  return applied,info
 def diagnostic_run(workspace,terrain,path,params=None,**kwargs):
  kwargs['duration_s']=24.;return true_run(workspace,terrain,path,params,**kwargs)
 success=False
 try:
  with patch.object(gx,'run_gazebo',diagnostic_run),patch.object(OffroadController,'command',command),patch.object(OffroadController,'_terrain_feedback',feedback):original.run_case(child,CASE)
  success=True
 except BaseException:
  save(stage/'failure.json',dict(traceback=traceback.format_exc(),rows=len(rows),arm=arm,prefix_rows_verified=prefix_checks));raise
 finally:
  if rows:np.savez_compressed(stage/'sidecar.npz',**{key:np.asarray([r[key]for r in rows])for key in rows[0]})
  for name,h in registered['sources'].items():
   if sha(ROOT/name)!=h:raise AssertionError('Frozen executed source changed:'+name)
  save(stage/'diagnostic_metadata.json',dict(iteration='I-77',arm=arm,success=success,complete_command_prefix_rows_verified=prefix_checks,complete_command_prefix_stop_exclusive=prefix_end,state_prefix_rows_verified=state_prefix_checks,state_prefix_end_inclusive=state_prefix_end_inclusive,native_feedback_prefix_rows_verified=native_feedback_prefix_checks,actual_requested_duration_s=24.,nominal_reference_route_duration_s=45.,full_route_completion_claim=False,source_protocol_sha256=registered['original_I76_protocol_sha256'],parent_I76_case=CASE,no_trajectory_facing_heading_actuation=True,intervention_specification=ARM_SPECS[arm],cap_interpretation='The cluster cap5rad/s matches the native allocation normalization scale, not an asserted motor-speed rating; actual commands retain original torque/surface-speed caps.',primary_endpoint_indices=[4130,4527,4800],endpoint_4130_interpretation='State at20.650s before the native release command acts; release effects afterward reported separately.',intervention_scope='Only feedback joint-rate reference/error rows; raw allocation/cache/normal/observer/gains/support/preparation/fade/clipping algorithms remain native. Their future values may change through actual physical state.',inherited_metadata_override='Child I76 wrapper iteration/case duration fields are inherited source metadata, not an I76 campaign or full-route completion claim. This I77 diagnostic metadata is authoritative.',history_fingerprint_scope='Selected numeric native-controller history; full physical solver prefix is replayed, not checkpoint restored.',same_state_native_counterfactual_scope='Current actual state/history with native feedback reference, propagated through the unchanged state-based preparation/fade/clipping; not the historical native trajectory after branch.'))
 if success:
  if prefix_checks!=prefix_end or state_prefix_checks!=state_prefix_end_inclusive+1 or native_feedback_prefix_checks!=state_prefix_end_inclusive+1:raise AssertionError('Required exact prefix incomplete')
  manifest={str(q.relative_to(stage)):dict(bytes=q.stat().st_size,sha256=sha(q))for q in sorted(stage.rglob('*'))if q.is_file()}
  save(stage/'artifact_manifest.json',dict(iteration='I-77',files=manifest))
  print(json.dumps(dict(arm=arm,success=True,frames=len(rows),prefix_rows_verified=prefix_checks)),flush=True)

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--arm',choices=ARMS,required=True);ap.add_argument('--protocol',type=Path,required=True);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--run',action='store_true');a=ap.parse_args()
 if not a.run:raise SystemExit('Explicit --run required for new Gazebo integration')
 run_arm(a.arm,a.protocol.resolve(),a.output.resolve())
if __name__=='__main__':main()
