"""I77 one fixed eight attitude-reference causal hold, without production edits."""
from pathlib import Path
from datetime import datetime,timezone
from contextlib import ExitStack
from unittest.mock import patch
import argparse,hashlib,importlib.util,json,shutil,sys,traceback
import numpy as np
import mujoco

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
OLD=ROOT/'results/tangent_heading_routes_20261127'
CASE='eight_m100_N'
START,STOP,SOURCE,HORIZON=19280,19380,19279,103.
PREFIX_FIELDS=('exact_requested','exact_applied','exact_reference_qpos','exact_reference_qvel','exact_feedback_reference_qvel','exact_feedback_gain','exact_error','exact_lqr_command','exact_equilibrium_command','exact_rolling_feedforward','exact_support_command','exact_transition_command_delta','exact_heading_command','exact_fade_command_delta','exact_clipping_command_delta','exact_sync_weights','exact_estimated_support_loads_N','exact_estimated_contact_point_m','exact_observer_wrench','exact_observer_generalized','exact_observer_interval_mean','exact_estimated_normal','exact_effective_phase_rad','exact_drive_fade')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True,allow_nan=False)+'\n')
def module(p,name):
 spec=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m
def token(v):
 a=np.asarray(v);return a.dtype.str,a.shape,np.ascontiguousarray(a).tobytes()
def raw_native_error(c,qref,vref,normal):
 """Stateless exact source algebra; never call native feedback a second time."""
 yaw=float(np.arctan2(2*(c.plant.data.qpos[3]*c.plant.data.qpos[6]+c.plant.data.qpos[4]*c.plant.data.qpos[5]),1.-2*(c.plant.data.qpos[5]**2+c.plant.data.qpos[6]**2)))
 cs,sn=np.cos(yaw),np.sin(yaw);rotation=np.array([[cs,sn],[-sn,cs]])
 target=np.array(qref,copy=True)
 mujoco.mju_mulQuat(target[3:7],np.array(qref[3:7]),np.array([np.cos(yaw/2),0.,0.,np.sin(yaw/2)]))
 target[2]=c.plant.data.qpos[2];velocity=np.array(vref,copy=True)
 velocity[2]=-float(normal[:2]@velocity[:2])/max(normal[2],.2)
 error=c.plant.error(target,velocity);error[:2]=rotation@error[:2];error[9:11]=rotation@error[9:11]
 error[6:9]=(error[6:9]+np.pi)%(2*np.pi)-np.pi;error[7:9]=0.
 if c.params.shaft_error=='free':error[6]=0.
 elif c.params.shaft_error=='rolling':error[6]=error[0]/c.radius-error[4]
 if c._yaw_error is not None and c.params.heading_actuation=='lqr':error[5]=c._yaw_error
 return error
def up(q):
 rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,np.asarray(q));return rotation.reshape(3,3)[:,2]
def angle(a,b):return float(np.degrees(np.arccos(np.clip(a@b,-1.,1.))))

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--protocol',type=Path,required=True);ap.add_argument('--run',action='store_true');args=ap.parse_args()
 if not args.run:raise SystemExit('Explicit --run required for new physics')
 out=args.output.resolve();out.mkdir(exist_ok=False)
 protocol=json.loads(args.protocol.read_text())
 if protocol['case_id']!=CASE or protocol['hold_indices']!=[START,STOP]or protocol['saved_reference_index']!=SOURCE or protocol['horizon_s']!=HORIZON:raise ValueError('Fixed diagnostic protocol differs')
 old_protocol=OLD/'protocol/protocol.json';p=json.loads(old_protocol.read_text())
 if sha(old_protocol)!='53106cdb758d76436e04b747c9483c4081abcc7ec1f2a163ace90b58b82085c2':raise ValueError('Original protocol changed')
 for n,h in {**p['source_sha256'],**p['input_sha256']}.items():
  if sha(ROOT/n)!=h:raise ValueError('Registered native source/input changed:'+n)
 native=ROOT/'results/baseline_deviation_cause_20261127/contact_trace/native97p5_stage01'
 native_prefix=native/'child/campaign/runs'/CASE/'precision.npz';manifest=json.loads((native/'artifact_manifest.json').read_text());bound=manifest['files'][str(native_prefix.relative_to(native))]
 if sha(native_prefix)!=bound['sha256']or native_prefix.stat().st_size!=bound['bytes']:raise ValueError('Validated native prefix changed')
 baseline=OLD/'campaign/runs'/CASE/'precision.npz';output_seal=OLD/'campaign/outputs_sealed.json';seal=json.loads(output_seal.read_text());bound=seal['files'][str(baseline.relative_to(OLD))]
 if sha(baseline)!=bound['sha256']or baseline.stat().st_size!=bound['bytes']:raise ValueError('Sealed full original native precision changed')
 parity=json.loads((native/'parity_receipt.json').read_text())
 if not parity['passed']or parity['prefix_rows']!=19501:raise ValueError('Native baseline parity prerequisite absent')
 sources=[Path(__file__),HERE/'causal_branch.py',OLD/'campaign_tools/run_case.py',args.protocol.resolve()]+[ROOT/n for n in p['source_sha256']]
 inputs=[baseline,output_seal,HERE/'attitude_hold_prephysics_horizon_amendment.json',native_prefix,native/'artifact_manifest.json',native/'parity_receipt.json',native/'registered_before_physics.json',old_protocol,ROOT/p['gain_cache']['path']]
 for f in set(sources):
  dest=out/'source'/f.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dest)
 bindings={str(f.relative_to(ROOT)):sha(f)for f in set(sources)}
 save(out/'registered_before_arrays_and_physics.json',dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),sources=bindings,inputs={str(f.relative_to(ROOT)):dict(bytes=f.stat().st_size,sha256=sha(f))for f in inputs},protocol_sha256=sha(args.protocol),no_case_arrays_loaded_before_registration=True,intervention='Hold only raw qref quaternion passed to native terrain feedback at savednative96.395s during96.400–96.900s; copycallerqref; nativefeedbackonce.',actual_horizon_s=HORIZON,nominal_route_horizon_s=133.,full_route_completion_claim=False))
 with np.load(baseline,allow_pickle=False)as z:expected={k:z[k].copy()for k in z.files}
 held=expected['exact_reference_quat'][SOURCE].copy()
 if token(held)!=token(expected['exact_reference_qpos'][SOURCE,3:7]):raise AssertionError('Raw saved quaternion fields disagree')
 helper=module(HERE/'causal_branch.py','_i77_held_history_helper')
 parent=module(OLD/'campaign_tools/run_case.py','_i77_held_eight_parent')
 child=out/'child'
 for folder,names in dict(adapter=['tangent_heading_adapter.py'],runtime=['route.py','heading_reference.py'],protocol=['protocol.json']).items():
  (child/folder).mkdir(parents=True)
  for name in names:
   src=OLD/folder/name;dst=child/folder/name;shutil.copy2(src,dst)
   if sha(src)!=sha(dst):raise AssertionError('Executed childcopy differs')
 import spherical_wheel.gazebo.experiments as gx
 from spherical_wheel.offroad_control import OffroadController
 native_feedback=OffroadController._terrain_feedback;native_command=OffroadController.command;native_run=gx.run_gazebo
 rows=[];capture={};prefix_checks=0;state_checks=0;firstrow_stage_checks=0;inactive_error_checks=0
 def feedback(c,qref,vref,ff,normal,measurement,touching):
  nonlocal firstrow_stage_checks,inactive_error_checks
  i=int(round(c.plant.data.time/c.plant.dt));before=helper.history(c);rawq=np.array(qref,copy=True)
  untreated=raw_native_error(c,qref,vref,normal)
  if helper.history(c)!=before:raise AssertionError('Stateless rawreference error reconstruction mutated history')
  active=START<=i<STOP;usedq=np.array(qref,copy=True)if active else qref
  if active:usedq[3:7]=held
  if i==START:
   checks=[('rawqref',qref,expected['exact_reference_qpos'][i]),('rawvref',vref,expected['exact_reference_qvel'][i]),('rollingFF',ff,expected['exact_rolling_feedforward'][i]),('observergeneralized',c.observer.generalized,expected['exact_observer_generalized'][i]),('observerintervalmean',c.observer.interval_mean,expected['exact_observer_interval_mean'][i]),('observerwrench',np.r_[c.observer.wrench()],expected['exact_observer_wrench'][i]),('estimatednormal',normal,expected['exact_estimated_normal'][i]),('untreatederror',untreated,expected['exact_error'][i])]
   for key,a,b in checks:
    if token(a)!=token(b):raise AssertionError('Inclusive firstactive stage mismatch:'+key)
  command,fb=native_feedback(c,usedq,vref,ff,normal,measurement,touching)
  if token(rawq)!=token(qref):raise AssertionError('Caller raw qref mutated')
  if not active:
   if token(untreated)!=token(fb['error']):raise AssertionError('Stateless source error reconstruction disagrees oninactive row')
   inactive_error_checks+=1
  if i==START:
   for key,a,b in [('gain',fb['K'],expected['exact_feedback_gain'][i]),('support',fb['support_torque'],expected['exact_support_command'][i]),('feedbackvref',fb['reference_qvel'],expected['exact_feedback_reference_qvel'][i])]:
    if token(a)!=token(b):raise AssertionError('Inclusive firstactive feedback mismatch:'+key)
   firstrow_stage_checks+=1
  actualup=up(c.plant.data.qpos[3:7]);nativeup=up(expected['exact_reference_quat'][i]);rawup=up(rawq[3:7]);usedup=up(usedq[3:7])
  capture.clear();capture.update(raw_reference_quat=rawq[3:7].copy(),used_reference_quat=np.array(usedq[3:7],copy=True),original_native_reference_quat=expected['exact_reference_quat'][i].copy(),raw_native_error=untreated.copy(),used_error=fb['error'].copy(),feedback_gain=fb['K'].copy(),raw_reference_qvel=np.array(vref,copy=True),held_raw_vref=np.zeros(9)if c._held is None else c._held['vref'].copy(),history_before_native_feedback_sha256=before,history_after_native_feedback_sha256=helper.history(c),hold_window_active=active,reference_changed=not np.array_equal(rawq[3:7],usedq[3:7]),actual_up=actualup,original_native_target_up=nativeup,current_untreated_target_up=rawup,used_target_up=usedup,up_error_to_original_native_target_deg=angle(actualup,nativeup),up_error_to_current_untreated_target_deg=angle(actualup,rawup),up_error_to_used_target_deg=angle(actualup,usedup))
  return command,fb
 def command(c,xy,velocity,acceleration):
  nonlocal prefix_checks,state_checks
  i=int(round(c.plant.data.time/c.plant.dt));before=helper.history(c)
  if i<=START:
   for key,a in [('time_s',float(c.plant.data.time)),('exact_qpos',c.plant.data.qpos),('exact_qvel',c.plant.data.qvel),('exact_previous_applied',c.plant.last_input),('exact_nominal_joints_before',c.nominal_joints)]:
    if token(a)!=token(expected[key][i]):raise AssertionError('Firstactiveinclusive physical/history prefix mismatch:'+key)
   state_checks+=1
  applied,info=native_command(c,xy,velocity,acceleration)
  if i<START:
   current={**{'exact_'+k:np.asarray(info[k])for k in ('requested','reference_qpos','reference_qvel','feedback_reference_qvel','feedback_gain','error','lqr_command','equilibrium_command','rolling_feedforward','support_command','transition_command_delta','heading_command','fade_command_delta','clipping_command_delta','sync_weights','estimated_support_loads_N','estimated_contact_point_m','effective_phase_rad','drive_fade')},'exact_applied':applied,'exact_observer_wrench':np.r_[c.observer.wrench()],'exact_observer_generalized':c.observer.generalized,'exact_observer_interval_mean':c.observer.interval_mean,'exact_estimated_normal':info['estimate']['normal']}
   for key in PREFIX_FIELDS:
    if token(current[key])!=token(expected[key][i]):raise AssertionError('Native emittedcommand prefix mismatch:'+key)
   prefix_checks+=1
  rows.append(dict(time_s=float(c.plant.data.time),control_index=i,history_before_command_sha256=before,history_after_command_sha256=helper.history(c),actual_requested=np.array(info['requested'],copy=True),actual_applied=np.array(applied,copy=True),**{k:np.asarray(v).copy()for k,v in capture.items()}))
  return applied,info
 def run(workspace,terrain,path,params=None,**kwargs):kwargs['duration_s']=HORIZON;return native_run(workspace,terrain,path,params,**kwargs)
 success=False
 try:
  with patch.object(gx,'run_gazebo',run),patch.object(OffroadController,'command',command),patch.object(OffroadController,'_terrain_feedback',feedback):parent.run_case(child,CASE)
  success=True
 except BaseException:
  save(out/'failure.json',dict(traceback=traceback.format_exc(),rows=len(rows)));raise
 finally:
  if rows:np.savez_compressed(out/'sidecar.npz',**{k:np.asarray([r[k]for r in rows])for k in rows[0]})
  save(out/'diagnostic_metadata.json',dict(iteration='I-77',success=success,case_id=CASE,actual_horizon_s=HORIZON,nominal_route_horizon_s=133.,full_route_completion_claim=False,complete_command_prefix_stop_exclusive=START,complete_command_prefix_rows_verified=prefix_checks,state_prefix_end_inclusive=START,state_prefix_rows_verified=state_checks,first_active_stage_checks=firstrow_stage_checks,inactive_native_error_reconstruction_checks=inactive_error_checks,hold_indices=[START,STOP],saved_reference_index=SOURCE,primary_endpoint_index=STOP,secondary_endpoint_index=19500,additional_endpoint_indices=[20427,20600],prephysics_horizon_amendment='97.5s proposal preserved; root requested103s before any candidate physics/outcome to connect earlydisturbance to originallatepeak102.135s.',inherited_metadata_override='I76/full133s childjob labels are inherited runner metadata; this I77 truncated-horizon diagnostic is authoritative.',scope='Only rawattitude quaternion entering feedback is held. Bodyyaw handling and derivedshaft pitchcoupling remain native; no normal/support/observer/gain/preparation/fade/caps changes. Later values may change via physical state.',error_comparison='Sidecar compares actualbodyup to saveduntreated native targetup, currentuntreated targetup, and usedheld targetup separately; full rawnative/used feedback error also preserved.'))
  for n,h in bindings.items():
   if sha(ROOT/n)!=h:raise AssertionError('Frozen source changed:'+n)
 if prefix_checks!=START or state_checks!=START+1 or firstrow_stage_checks!=1:raise AssertionError('Incomplete prefix checks')
 produced=child/'campaign/runs'/CASE/'precision.npz';checks={}
 with np.load(produced,allow_pickle=False)as z:
  if set(z.files)!=set(expected)or len(z['time_s'])!=20601:raise AssertionError('Branch precision schema/horizon differs')
  for key in sorted(z.files):
   good=token(z[key][:START])==token(expected[key][:START]);checks[key]=good
  if not all(checks.values()):raise AssertionError('Full sharedprecision prefix mismatch')
 save(out/'full_precision_prefix_receipt.json',dict(passed=True,shared_fields=len(checks),prefix_stop_exclusive=START,fields=checks,scope='All shared emitted/state precision fields before intervention; no fullbranch equality claimed.'))
 save(out/'artifact_manifest.json',dict(files={str(f.relative_to(out)):dict(bytes=f.stat().st_size,sha256=sha(f))for f in sorted(out.rglob('*'))if f.is_file()and '__pycache__'not in f.parts}))
 print(json.dumps(dict(passed=True,rows=len(rows),prefix_rows=prefix_checks,state_prefix_rows=state_checks)),flush=True)
if __name__=='__main__':main()
