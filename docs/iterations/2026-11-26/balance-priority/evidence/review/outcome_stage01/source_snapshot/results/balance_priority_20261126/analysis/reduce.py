"""I58 sealed saved-array reduction. No controller, observer, model or physics calls.

Uses root-owned I58 metric policy plus immutable I56/I49/I41 pure metrics.
P rate values remain optional diagnostics; all gates come from frozen protocol.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse,csv,gzip,hashlib,json,shutil,types
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
SHARED=ROOT/'results/constrained_handoff_20261126/analysis/reduce_handoff.py'
SHARED_SHA='036f805e4dd2944f4be5f85bf811065afb3e00cf08f9d870c33351d69826493c'
LEGACY=ROOT/'results/belt_capacity_sweep_20261126/analyze_capacity.py'
LEGACY_SHA='cbaf7ae18989e1a07e9dfaf4167dd2eaa75fd59d064c4259a7b75f79d05c3a6d'
OLD_PROTOCOL=ROOT/'results/contact_scheduling_joint_tracking_20261126/protocol.json'
PHASES=('m100','p030','p090')
REFERENCE_FIELDS=('time_s','exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2')
PROJECTION_FIELDS=('time_s','exact_qpos','exact_qvel','exact_reference_qpos','exact_reference_xy',
 'exact_reference_velocity_m_s','exact_reference_acceleration_m_s2','exact_applied','exact_previous_applied',
 'exact_input_bounds','exact_requested','exact_native_requested')
EVENT_FIELDS=('event_id','time_s','axis','axis_index','incoming_sign','outgoing_sign','window_s','reference_component_extremum_m')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def safe(v):
 if isinstance(v,dict):return {str(k):safe(x)for k,x in v.items()}
 if isinstance(v,(list,tuple)):return [safe(x)for x in v]
 if isinstance(v,np.ndarray):return safe(v.tolist())
 if isinstance(v,np.generic):return safe(v.item())
 if isinstance(v,Path):return str(v)
 if isinstance(v,float)and not np.isfinite(v):return None
 return v
def save(path,v):Path(path).write_text(json.dumps(safe(v),indent=2,allow_nan=False)+'\n')
def load(path):
 with np.load(path,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def module(path,name):
 m=types.ModuleType(name);m.__file__=str(path);exec(compile(path.read_text(),str(path),'exec'),m.__dict__);return m

def dependencies():
 if sha(SHARED)!=SHARED_SHA or sha(LEGACY)!=LEGACY_SHA:raise ValueError('Immutable I56/I49 pure metric source changed')
 old=read(OLD_PROTOCOL);helper=ROOT/old['inputs']['event_helper'];events=ROOT/old['inputs']['event_turns']
 for key,path in(('event_helper',helper),('event_turns',events)):
  if sha(path)!=old['input_sha256'][key]:raise ValueError('Corrected fixed turn definitions/helper changed')
 return helper,events

def verify_manifest(folder):
 for row in read(folder/'artifact_manifest.json')['files']:
  if sha(folder/row['path'])!=row['sha256']:raise ValueError('Sealed case artifact changed: '+str(folder/row['path']))

def compare_cases(cases,protocol,metrics):
 comparisons={}
 for phase in PHASES:
  p,b=phase+'_P',phase+'_B'
  if p in cases and b in cases:
   old=cases.get('m100_A',dict(available=False))if phase=='m100'else None
   result=metrics.gate_comparison(cases[p],cases[b],protocol['acceptance'],hard_slew=old)
   result.update(candidate=p,baseline=b,old_hard_slew='m100_A'if phase=='m100'else None)
   comparisons[p+'_vs_'+b]=result
 return comparisons

def write_tables(output,report):
 fields=['case_id','available','completed','duration_s','moving_XY_RMS_mm','peak_XY_mm','final_XY_mm',
  'raw_lean_peak_deg','raw_lean_RMS_deg','absolute_tilt_peak_deg','late_yaw_peak_rad_s','late_abs_heading_turns',
  'priority_max_abs_rad_s','priority_held_RMS_rad_s','command_step_RMS','command_step_p99','old_slew_diagnostic_excess',
  'historical_all_shared_exact']
 with(output/'cases.csv').open('w',newline='')as stream:
  w=csv.DictWriter(stream,fieldnames=fields);w.writeheader()
  for case,r in report['cases'].items():
   v=dict(case_id=case,available=r.get('available',False))
   if v['available']:
    t=r['timed_tracking'];p=r['priority'];c=r['continuity'];y=r['motion']['late'];a=r['attitude']
    v.update(completed=r['completed'],duration_s=r['observed_duration_s'],moving_XY_RMS_mm=None if t['moving_rms_m']is None else 1000*t['moving_rms_m'],
     peak_XY_mm=1000*t['full_peak_m'],final_XY_mm=1000*t['final_m'],raw_lean_peak_deg=a['peak_raw_reference_error_deg'],
     raw_lean_RMS_deg=a['rms_raw_reference_error_deg'],absolute_tilt_peak_deg=a['peak_absolute_deg'],
     late_yaw_peak_rad_s=y.get('world_z_peak_abs_rad_s'),late_abs_heading_turns=y.get('heading_absolute_travel_turns'),
     priority_max_abs_rad_s=p.get('total_response_max_abs'),priority_held_RMS_rad_s=p.get('total_response_held_RMS_norm'),
     command_step_RMS=c['normalized_step_norm_RMS'],command_step_p99=c['normalized_step_norm_p99'],
     old_slew_diagnostic_excess=r['commands']['slew_max_violation'],historical_all_shared_exact=(r.get('historical_parity')or{}).get('all_shared_fields_exact'))
   w.writerow(v)
 with(output/'turns.csv').open('w',newline='')as stream:
  fields=['case_id','event_id','window_complete','geometric_component_overshoot_m','actual_reversal_lag_s','actual_reversal_missing_in_window']
  w=csv.DictWriter(stream,fieldnames=fields);w.writeheader()
  for case,row in report['cases'].items():
   for turn in row.get('turns',[]):w.writerow(dict(case_id=case,**{k:turn[k]for k in fields if k!='case_id'}))

def reduce(output):
 helper_path,events_path=dependencies();protocol_path=BASE/'protocol.json';protocol=read(protocol_path)
 if protocol['iteration']!='I-58':raise ValueError('I58 protocol required')
 output.mkdir(parents=True,exist_ok=False)
 sources=[Path(__file__),BASE/'analysis/metrics.py',BASE/'analysis/replay.py',SHARED,LEGACY,OLD_PROTOCOL,helper_path,events_path,protocol_path]
 records=[]
 for source in sources:
  target=output/'source_snapshot'/source.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,target)
  records.append(dict(origin=str(source.relative_to(ROOT)),snapshot=str(target.relative_to(output)),sha256=sha(source)))
 folders={};availability={};inputs=[]
 program_path=ROOT/protocol['historical_baseline_paths']['m100_B'];inputs.append(program_path)
 required=('precision.npz','rollout.npz','summary.json','provenance.json','governor_diagnostics.json','artifact_manifest.json')
 for job in protocol['cases']:
  case=job['case_id'];folder=BASE/'campaign/runs'/case;folders[case]=folder
  present=all((folder/name).is_file()for name in required)
  availability[case]=dict(available=present,reason=None if present else'execution_failure'if(folder/'failure.json').is_file()else'not_run_or_not_sealed',
   case_id=case,phase_rad=job['phase_rad'],variant=job['variant'],case_source=str(BASE.relative_to(ROOT)))
  if present:
   verify_manifest(folder);inputs.extend(folder/name for name in required)
   if(folder/'historical_precision_comparison.json').is_file():inputs.append(folder/'historical_precision_comparison.json')
  elif(folder/'failure.json').is_file():inputs.append(folder/'failure.json')
 declaration=dict(iteration='I-58',registered_before_arithmetic_utc=datetime.now(timezone.utc).isoformat(),sources=records,
  input_sha256={str(p.relative_to(ROOT)):sha(p)for p in inputs},availability_before_reading=availability,
  frozen_acceptance=protocol['acceptance'],metric_conventions=dict(attitude_RMS_scope='All measured0–observedend states, including terminal; same legacy convention.',
   XY_moving_window_s=protocol['acceptance']['moving_window_s'],yaw_window_s=protocol['acceptance']['yaw_window_s'],
   commands='All emitted amplitude/priority checks. Held exposure and continuity omit terminal. First emission uses actual prior input.',
   priority='Sum first-stage+secondary estimated transverse deltas; not secondary-only equality.',
   rates='60/6old assumed rates are hard constraints only for A. P/B rate comparisons are diagnostics.',
   events='Exactly six immutable corrected reference turns, excluding their stored old outcomes.',
   motion='World-angular rates from endpoint quaternion and local qvel; acceleration5/50ms interval-average differences.'),
  no_physics_model_controller_observer_calls=True)
 save(output/'registered_before_arithmetic.json',declaration)
 shared=module(SHARED,'_i58_fixed_i56_metrics');legacy=module(LEGACY,'_i58_fixed_i49_metrics')
 helper=module(helper_path,'_i58_fixed_i41_helper');metrics=module(BASE/'analysis/metrics.py','_i58_root_metrics')
 events=[{k:r[k]for k in EVENT_FIELDS}for r in read(events_path)['rows']]
 if len(events)!=6:raise ValueError('Expected all six fixed reference turns')
 with np.load(program_path,allow_pickle=False)as z:program={k:z[k].copy()for k in REFERENCE_FIELDS}
 portable=output/'portable_inputs';portable.mkdir();np.savez_compressed(portable/'reference_program.npz',**program);save(portable/'events.json',events)
 index=dict(iteration='I-58',timestamps='All original5ms samples, without downsampling.',
  sources={name:str((output/'source_snapshot'/path.relative_to(ROOT)).relative_to(output))for name,path in
   dict(reducer=Path(__file__),metrics=BASE/'analysis/metrics.py',shared=SHARED,legacy=LEGACY,helper=helper_path).items()},
  reference_program='portable_inputs/reference_program.npz',events='portable_inputs/events.json',
  reference_program_original_precision_sha256=sha(program_path),cases={},unavailable_cases={},protocol=protocol)
 report=dict(iteration='I-58',cases={},comparisons={},definitions=declaration['metric_conventions'],
  raw_target_unmodified=True,no_new_physics_model_controller_or_observer_calls=True,default_adoption=False)
 for job in protocol['cases']:
  case=job['case_id'];folder=folders[case]
  if not availability[case]['available']:
   report['cases'][case]=availability[case];index['unavailable_cases'][case]=availability[case]
   if(folder/'failure.json').is_file():report['cases'][case]['failure_evidence']=read(folder/'failure.json')
   continue
  raw=load(folder/'precision.npz');summary=read(folder/'summary.json');provenance=read(folder/'provenance.json');governor=read(folder/'governor_diagnostics.json')
  row,signals,geometry=metrics.case_metrics(raw,summary,provenance,governor,protocol,program,events,shared,legacy,helper)
  parity=read(folder/'historical_precision_comparison.json')if(folder/'historical_precision_comparison.json').is_file()else None
  row.update(case_id=case,phase_rad=job['phase_rad'],variant=job['variant'],case_source=str(BASE.relative_to(ROOT)),historical_parity=parity)
  report['cases'][case]=row;np.savez_compressed(output/(case+'_signals.npz'),**signals);np.savez_compressed(output/(case+'_geometry.npz'),**geometry)
  projection=portable/(case+'_precision_projection.npz');fields={k:raw[k]for k in PROJECTION_FIELDS};np.savez_compressed(projection,**fields)
  projected=load(projection)
  projection_exact=all(v.dtype==projected[k].dtype and v.shape==projected[k].shape and v.tobytes()==projected[k].tobytes()for k,v in fields.items())
  if not projection_exact:raise ValueError('Portable metric projection lost precision')
  compressed=portable/(case+'_diagnostics.json.gz')
  with gzip.open(compressed,'wt',encoding='utf8')as stream:
   json.dump(safe([dict(time_s=r['time_s'],diagnostics=r['diagnostics'])for r in governor]),stream,allow_nan=False,separators=(',',':'))
  metadata=portable/(case+'_metadata.json');save(metadata,dict(summary=summary,provenance=provenance,historical_parity=parity,job=job,
   original_precision_path=str((folder/'precision.npz').relative_to(ROOT)),original_precision_sha256=sha(folder/'precision.npz'),
   projection_fields=PROJECTION_FIELDS,projection_byte_exact=projection_exact,actual_metadata_preserved=True))
  index['cases'][case]={name:str(path.relative_to(output))for name,path in dict(projection=projection,diagnostics=compressed,metadata=metadata).items()}
  index['cases'][case].update({name+'_sha256':sha(path)for name,path in dict(projection=projection,diagnostics=compressed,metadata=metadata).items()})
  index['cases'][case]['original_precision_sha256']=sha(folder/'precision.npz')
 report['comparisons']=compare_cases(report['cases'],protocol,metrics)
 initial=report['comparisons'].get('m100_P_vs_m100_B',{})
 report['broader_campaign_eligible']=bool(initial.get('available')and initial.get('passes_all_conditions'))
 report['all_three_phase_screens_pass']=all(report['comparisons'].get(p+'_P_vs_'+p+'_B',{}).get('passes_all_conditions',False)for p in PHASES)
 report['all_registered_cases_available']=all(r.get('available')for r in report['cases'].values())
 save(output/'report.json',report);save(output/'portable_inputs.json',index);write_tables(output,report)
 for path,expected in declaration['input_sha256'].items():
  if sha(ROOT/path)!=expected:raise ValueError('Registered input changed during reduction: '+path)
 for source in records:
  if sha(ROOT/source['origin'])!=source['sha256']:raise ValueError('Registered source changed during reduction: '+source['origin'])
 save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
  for p in sorted(output.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))
 print(json.dumps(safe(dict(available_cases=[c for c,r in report['cases'].items()if r.get('available')],broader_campaign_eligible=report['broader_campaign_eligible'],
  comparisons={k:dict(available=v.get('available'),passes=v.get('passes_all_conditions'),failed=v.get('failed_checks'))for k,v in report['comparisons'].items()})),indent=2))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args();reduce(a.output.resolve())
