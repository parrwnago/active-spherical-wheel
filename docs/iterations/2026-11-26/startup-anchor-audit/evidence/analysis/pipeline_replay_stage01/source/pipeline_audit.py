"""I59 supplementary pure-NumPy fixed-state native pipeline and portable replay.

P commands are native-policy shadows, not reoptimized final P commands.
No reset, observer/reference update, dynamics evaluation, physics or trajectory inference.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil,types
import numpy as np
FIELDS=('time_s','exact_qpos','exact_qvel','exact_error','exact_feedback_gain','exact_equilibrium_command',
 'exact_rolling_feedforward','exact_support_command','exact_lqr_command','exact_transition_command_delta',
 'exact_heading_command','exact_fade_command_delta','exact_reference_qpos','exact_reference_qvel',
 'exact_feedback_reference_qvel','exact_sync_weights','exact_input_bounds','exact_native_requested',
 'exact_native_applied','exact_native_clipping_command_delta','exact_applied','exact_previous_applied',
 'exact_estimated_support_loads_N','exact_estimated_normal','exact_estimated_contact_point_m')
MODES=('gain_row_only','trim_only','combined_diagnostic')


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(x):
 if isinstance(x,dict):return {str(k):safe(v)for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [safe(v)for v in x]
 if isinstance(x,np.ndarray):return safe(x.tolist())
 if isinstance(x,np.generic):return safe(x.item())
 return x
def save(p,x):Path(p).write_text(json.dumps(safe(x),indent=2,allow_nan=False)+'\n')
def load(p):
 with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def exact(a,b):
 a,b=np.asarray(a),np.asarray(b);return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()
def module(p):
 m=types.ModuleType('_i59_native_pipeline');m.__file__=str(p);exec(compile(p.read_text(),str(p),'exec'),m.__dict__);return m

def statistics(t,caps,base,requested,clipped,lower,upper,cliplo,cliphi,window,p):
 lo,hi=window;mask=(t>=lo-1e-9)&(t<=hi+1e-9);weight=np.maximum(0.,np.minimum(t[1:],hi)-np.maximum(t[:-1],lo))
 d=requested-base['request'];a=clipped-base['clipped'];absolute=np.asarray(p['absolute_material_thresholds']);normalized=p['normalized_material_threshold']
 def duration(condition):return np.sum(condition[:-1]*weight[:,None],axis=0)
 masked=(abs(d)>absolute)&(abs(a)<=absolute)
 return dict(samples=int(mask.sum()),held_exposure_s=float(weight.sum()),
  raw_max_abs_by_input=np.max(abs(d[mask]),axis=0),clipped_max_abs_by_input=np.max(abs(a[mask]),axis=0),
  raw_RMS_by_input=np.sqrt(np.mean(d[mask]**2,axis=0)),clipped_RMS_by_input=np.sqrt(np.mean(a[mask]**2,axis=0)),
  raw_material_duration_by_input_s=duration(abs(d)>absolute),clipped_material_duration_by_input_s=duration(abs(a)>absolute),
  raw_normalized_material_duration_s=float(np.sum(weight*(np.max(abs(d[:-1])/caps[:-1],axis=1)>normalized))),
  clipped_normalized_material_duration_s=float(np.sum(weight*(np.max(abs(a[:-1])/caps[:-1],axis=1)>normalized))),
  baseline_native_outside_duration_by_input_s=duration(abs(base['request'])>caps+1e-12),
  alternative_native_outside_duration_by_input_s=duration(abs(requested)>caps+1e-12),masked_material_duration_by_input_s=duration(masked),
  raw_interval_max_width_by_input=np.max((upper-lower)[mask],axis=0),clipped_interval_max_width_by_input=np.max((cliphi-cliplo)[mask],axis=0),
  worst_raw_abs_upper_bound_by_input=np.max(np.maximum(abs(lower-base['request']),abs(upper-base['request']))[mask],axis=0),
  worst_clipped_abs_upper_bound_by_input=np.max(np.maximum(abs(cliplo-base['clipped']),abs(cliphi-base['clipped']))[mask],axis=0))


def compute(inputs,protocol,helper,out):
 anchors={name:dict(K0=inputs[name][0]['exact_feedback_gain'][0,0],trim=inputs[name][0]['exact_equilibrium_command'][0])for name in protocol['anchor_cases']}
 pairs=[dict(actual=a,alternative=b,K0_byte_exact=exact(x['K0'],y['K0']),trim_byte_exact=exact(x['trim'],y['trim']),
  K0_max_abs_difference=float(np.max(abs(x['K0']-y['K0']))),trim_max_abs_difference=float(np.max(abs(x['trim']-y['trim']))))
  for a,x in anchors.items()for b,y in anchors.items()]
 report=dict(iteration='I-59',new_physics_runs=0,coefficient_pairs=pairs,anchors=anchors,cases={},
  boundary=protocol['boundary'],thresholds=dict(absolute=protocol['absolute_material_thresholds'],normalized=protocol['normalized_material_threshold']),
  inference='Fixed-state command arithmetic only; neither altered trajectories nor better stability are established.')
 windows={**protocol['windows_s'],**protocol['event_windows_s']}
 for case,(raw,meta)in inputs.items():
  t=raw['time_s'];n=len(t);caps=raw['exact_input_bounds'];own=case[:-1]+'B';K0=raw['exact_feedback_gain'][0,0];trim=raw['exact_equilibrium_command'][0]
  controls=dict(constant_K0=exact(raw['exact_feedback_gain'][:,0,:],np.broadcast_to(K0,(n,len(K0)))),
   constant_trim=exact(raw['exact_equilibrium_command'],np.broadcast_to(trim,(n,5))),same_phase_B_K0=exact(K0,anchors[own]['K0']),
   same_phase_B_trim=exact(trim,anchors[own]['trim']),shaft_sync_zero=bool(np.all(raw['exact_sync_weights'][:,0]==0)),
   full_5ms_trace=bool(n==8801 and abs(t[0])<1e-12 and abs(t[-1]-44)<1e-8 and np.all(abs(np.diff(t)-.005)<1e-9)))
  if not all(controls.values()):raise ValueError('Fixed-state control failed: '+case+' '+str(controls))
  fade=np.empty((n,3));identified=np.zeros(n,bool);residuals={};fadeclosure=0.
  for i in range(n):
   row={k:raw[k][i]for k in FIELDS};stages=helper.native_stages(row,atol=protocol['numerical_closure_max_abs'])
   f=helper.infer_fade(stages,raw['exact_qvel'][i],raw['exact_reference_qvel'][i],meta['parameters']['airborne_damping_Nms'],
    raw['recorded_fade_float32'][i],closure_atol=protocol['numerical_closure_max_abs'])
   fade[i]=f['estimate'],f['lower'],f['upper'];identified[i]=f['identifiable'];fadeclosure=max(fadeclosure,f['closure_residual'])
   for key,value in stages['residuals'].items():residuals[key]=max(residuals.get(key,0.),value)
  attenuation=np.ones((n,5));attenuation[:,:3]=fade[:,0,None];low=attenuation.copy();high=attenuation.copy();low[:,:3]=fade[:,1,None];high[:,:3]=fade[:,2,None]
  base=dict(request=raw['exact_native_requested'],clipped=raw['exact_native_applied'])
  result=dict(controls=controls,baseline_closure_max=residuals,fade=dict(identifiable_samples=int(identified.sum()),unidentifiable_samples=int((~identified).sum()),
   estimate_min=float(fade[:,0].min()),estimate_max=float(fade[:,0].max()),interval_width_max=float(np.max(fade[:,2]-fade[:,1])),closure_max=fadeclosure,
   precision='All reconstructed values retain intervals; float32 telemetry is not called exact.'),alternatives={})
  derivatives=dict(time_s=t,fade_estimate_lower_upper=fade,fade_identifiable=identified)
  for alternative,a in anchors.items():
   delta_feedback=np.zeros((n,5));contribution=-(a['K0']-K0)[None,:]*raw['exact_error'];delta_feedback[:,0]=contribution.sum(axis=1)
   delta_trim=np.broadcast_to(a['trim']-trim,(n,5));modes={}
   for mode,pre in zip(MODES,(delta_feedback,delta_trim,delta_feedback+delta_trim)):
    post=(1.-raw['exact_sync_weights'])*pre;d=attenuation*post;v1,v2=low*post,high*post;dl,dh=np.minimum(v1,v2),np.maximum(v1,v2)
    def shifted(delta):
     value=base['request']+delta;value[delta==0]=base['request'][delta==0];return value
    request,lower,upper=shifted(d),shifted(dl),shifted(dh)
    clipped=np.clip(request,-caps,caps);cliplo=np.clip(lower,-caps,caps);cliphi=np.clip(upper,-caps,caps)
    identity=alternative==own
    if identity:
     controls[mode+'_actual_anchor_native_exact']=exact(request,base['request'])and exact(clipped,base['clipped'])
     if not controls[mode+'_actual_anchor_native_exact']:raise ValueError('Actual-anchor reproduction failed')
    # Cross-check the vectorized stage map with the independently source-audited scalar helper at fixed rows.
    for i in(0,971,7185,n-1):
     stages=helper.native_stages({k:raw[k][i]for k in FIELDS},atol=protocol['numerical_closure_max_abs'])
     f=dict(estimate=fade[i,0],lower=fade[i,1],upper=fade[i,2]);check=helper.propagate_fixed_state(stages,pre[i],f)
     if max(np.max(abs(check['native_request']-request[i])),np.max(abs(check['native_clipped']-clipped[i])))>protocol['numerical_closure_max_abs']:raise ValueError('Scalar/vector stage map mismatch')
    modes[mode]={name:statistics(t,caps,base,request,clipped,lower,upper,cliplo,cliphi,window,protocol)for name,window in windows.items()}
    key=alternative+'_'+mode;derivatives[key+'_delta_request']=d;derivatives[key+'_delta_native_clipped']=clipped-base['clipped']
   result['alternatives'][alternative]=modes
  np.savez_compressed(out/(case+'_derivatives.npz'),**derivatives);report['cases'][case]=result
 report['all_controls_pass']=all(all(c['controls'].values())for c in report['cases'].values())
 return safe(report)


def run(protocol_path,data_source,output):
 output.mkdir(parents=True,exist_ok=False);source=output/'source';source.mkdir();shutil.copyfile(Path(__file__),source/'pipeline_audit.py')
 if data_source:
  index=json.loads((data_source/'inputs/index.json').read_text());protocol=index['protocol'];helper_path=data_source/'source/native_pipeline.py'
  for row in index['files']:
   if sha(data_source/row['path'])!=row['sha256']:raise ValueError('Changed portable input: '+row['path'])
  inputs={case:(load(data_source/item['npz']),json.loads((data_source/item['json']).read_text()))for case,item in index['cases'].items()}
  registration=dict(mode='portable NumPy replay',input_index_sha256=sha(data_source/'inputs/index.json'),known_prior='Original I59 null coefficient bound/report may be known; arithmetic/criteria unchanged.')
 else:
  base=protocol_path.parent.parent;root=base.parents[1];protocol=json.loads(protocol_path.read_text());seal=json.loads((protocol_path.parent/'input_source_seal.json').read_text())
  for row in seal['inputs']+seal['sources']:
   if not row.get('missing')and sha(root/row['path'])!=row['sha256']:raise ValueError('Sealed source changed: '+row['path'])
  helper_path=base/'command_tools/native_pipeline.py';registration=dict(mode='original saved-data audit',protocol_sha256=sha(protocol_path),
   input_source_seal_sha256=sha(protocol_path.parent/'input_source_seal.json'),input_sha256={r['path']:r['sha256']for r in seal['inputs']},
   known_prior='Minimal coefficient-stage bound<=2.5274356e-8Nm already established. Supplementary full-stage code is now frozen before its arithmetic; root protocol was frozen before any coefficient outcome.')
  inputs={};pack=output/'inputs';pack.mkdir();index=dict(protocol=protocol,cases={},files=[])
  registration.update(captured_before_pipeline_arithmetic_utc=datetime.now(timezone.utc).isoformat(),analysis_source_sha256=sha(Path(__file__)),helper_sha256=sha(helper_path))
  save(output/'registered_before_arithmetic.json',registration);shutil.copyfile(helper_path,source/'native_pipeline.py')
  for case in protocol['datasets']:
   def item(name):return next(r for r in seal['inputs']if r['case']==case and Path(r['path']).name==name)
   with np.load(root/item('precision.npz')['path'],allow_pickle=False)as z:raw={k:z[k].copy()for k in FIELDS}
   with np.load(root/item('rollout.npz')['path'],allow_pickle=False)as z:raw['recorded_fade_float32']=z['drive_fade'].copy()
   meta=json.loads((root/item('provenance.json')['path']).read_text());inputs[case]=(raw,meta)
   npz=pack/(case+'.npz');metadata=pack/(case+'.json');np.savez_compressed(npz,**raw);save(metadata,dict(case=meta['case'],parameters=meta['parameters'],
    original_hashes={name:item(name)['sha256']for name in('precision.npz','rollout.npz','provenance.json')}))
   index['cases'][case]=dict(npz=str(npz.relative_to(output)),json=str(metadata.relative_to(output)))
   index['files']+= [dict(path=str(p.relative_to(output)),sha256=sha(p))for p in(npz,metadata)]
  index['files'].append(dict(path='source/native_pipeline.py',sha256=sha(source/'native_pipeline.py')));save(pack/'index.json',index)
 if data_source:
  registration.update(captured_before_pipeline_arithmetic_utc=datetime.now(timezone.utc).isoformat(),analysis_source_sha256=sha(Path(__file__)),helper_sha256=sha(helper_path));save(output/'registered_before_arithmetic.json',registration)
  shutil.copyfile(helper_path,source/'native_pipeline.py')
 helper=module(source/'native_pipeline.py');science=compute(inputs,protocol,helper,output);save(output/'science.json',science)
 if data_source:
  same=science==json.loads((data_source/'science.json').read_text());save(output/'replay_comparison.json',dict(science_exact=same,source_science_sha256=sha(data_source/'science.json')))
  if not same:raise ValueError('Portable pipeline replay differs')
 save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
  for p in output.rglob('*')if p.is_file()and p.name!='artifact_manifest.json']))
 print(json.dumps(dict(all_controls_pass=science['all_controls_pass'],cases=len(science['cases']),new_physics_runs=0)))

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);g=p.add_mutually_exclusive_group(required=True);g.add_argument('--protocol',type=Path);g.add_argument('--data-source',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 run(a.protocol.resolve()if a.protocol else None,a.data_source.resolve()if a.data_source else None,a.output.resolve())
