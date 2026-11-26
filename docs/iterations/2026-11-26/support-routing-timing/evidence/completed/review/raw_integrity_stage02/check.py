from pathlib import Path
import hashlib,json
import numpy as np
from spherical_wheel.support_routing_audit import bridge_contact_truth

ROOT=Path.cwd();FAMILY=ROOT/'results/support_routing_timing_20261126';OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
inputs={};cases={}
for name,folder in [('mars_compact_outback','mars_compact_outback'),('flat_compact_outback','flat_compact_outback_setup_corrected')]:
 p=FAMILY/'campaign/runs'/folder
 for f in ['precision.npz','truth_substeps.npz','rollout.npz','report.json','provenance.json']:
  inputs[str((p/f).relative_to(ROOT))]=sha(p/f)
 with np.load(p/'precision.npz') as z:r={k:z[k] for k in z.files}
 with np.load(p/'truth_substeps.npz') as z:s={k:z[k] for k in z.files}
 with np.load(p/'rollout.npz') as z:l={k:z[k] for k in z.files}
 provenance=json.loads((p/'provenance.json').read_text());report=json.loads((p/'report.json').read_text())
 checks={'completed':report['completed'],'one_feedback_call':bool(np.all(r['shadow_feedback_call_count']==1)),'shadow_immutable':bool(np.all(r['shadow_history_unchanged'])),'full_16s':len(r['time_s'])==3201 and abs(r['time_s'][-1]-16)<1e-8,'zero_interval_at_t0':int(r['truth_interval_sample_count'][0])==0 and not bool(r['truth_interval_valid'][0]),'five_samples_each_interval':bool(np.all(r['truth_interval_sample_count'][1:]==5)),'substep_count':len(s['time_s'])==16000,'substep_spacing':bool(np.allclose(np.diff(s['time_s']),.001,rtol=0,atol=1e-10)),'relative_interval_end_alignment':bool(np.allclose(s['time_s'][4::5],r['time_s'][1:],rtol=0,atol=1e-10)),'state_plugin_step_iterations':bool(np.all(np.diff(s['bridge_state'][:,1])==1)),'applied_inputs_match':bool(np.array_equal(l['input'],r['exact_applied'].astype(l['input'].dtype))),'commands_match':bool(np.array_equal(l['command'],r['exact_native_requested'].astype(l['command'].dtype))),'qpos_match':bool(np.array_equal(l['qpos'],r['exact_qpos'].astype(l['qpos'].dtype)))}
 gaps={}
 raw=[]
 for k,st in enumerate(s['bridge_state']):
  a,b=s['contact_offsets'][k:k+2];raw.append(bridge_contact_truth(s['contacts'][a:b],st[2:5]))
 for rawkey,savedkey in [('loads_N','truth_interval_mean_loads_N'),('wrench_world','truth_interval_mean_wrench_world'),('all_external_wrench_world','truth_interval_mean_all_external_wrench_world'),('body_wrench_world','truth_interval_mean_body_wrench_world'),('body_load_N','truth_interval_mean_body_load_N')]:
  values=np.asarray([v[rawkey] for v in raw]);values=values.reshape((3200,5)+values.shape[1:]).mean(axis=1)
  gap=float(np.max(np.abs(values-r[savedkey][1:])));gaps[savedkey]=gap;checks[savedkey+'_reconstructed']=gap<1e-12
 checks['sources_unchanged']=all(sha(ROOT/v['origin'])==v['sha256']==sha(p/v['snapshot']) for v in provenance['sources'])
 checks['bridge_binary_unchanged']=sha(ROOT/provenance['bridge_binary']['path'])==provenance['bridge_binary']['sha256']==sha(p/provenance['bridge_binary']['snapshot'])
 cases[name]={'passed':all(checks.values()),'checks':checks,'max_absolute_gaps':gaps,'path':str(p.relative_to(ROOT)),'physics_substeps':len(raw)}
result={'iteration':'I-46','previous_attempt':'../raw_integrity_stage01 (report serialization failed on numpy.bool_; unchanged data/checks rerun)','passed':all(c['passed'] for c in cases.values()),'cases':cases,'inputs_sha256':inputs,'checker_sha256':sha(Path(__file__)),'scope':'Independent reconstruction of saved mean truth from each raw1ms solve about its matching moving center. No controller/physics executed; stored native applied/requested values checked after normal rollout dtype cast (float32 versus full precision), and all snapshots checked.'}
(OUT/'report.json').write_text(json.dumps(result,indent=2,default=lambda value:value.item() if isinstance(value,np.generic) else str(value))+'\n');print(json.dumps(result,indent=2,default=lambda value:value.item() if isinstance(value,np.generic) else str(value)));assert result['passed']
