"""Independent raw scope check, no controller/model execution."""
from pathlib import Path
import json,hashlib
import numpy as np
ROOT=Path.cwd();F=ROOT/'results/belt_capacity_sweep_20261126';OUT=F/'review/raw_stage01'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def load(p):
 with np.load(p) as z:return {k:z[k].copy() for k in z.files}

def main():
 OUT.mkdir(exist_ok=False)
 before={'source_sha256':sha(Path(__file__)),'inputs':{}}
 cases={};checks={};arrays={};provs={}
 for ident,cap in [('mars_cap030',.3),('mars_cap060',.6),('mars_cap090',.9)]:
  p=F/'campaign/runs'/ident
  for name in ['precision.npz','rollout.npz','summary.json','provenance.json','execution_result.json']:
   before['inputs'][str((p/name).relative_to(ROOT))]=sha(p/name)
  r,l=load(p/'precision.npz'),load(p/'rollout.npz');pr=json.loads((p/'provenance.json').read_text());su=json.loads((p/'summary.json').read_text());arrays[ident]=r;provs[ident]=pr
  bounds=np.array([1.5,.6,.6,cap,cap]);n=len(r['time_s'])
  c={'sources_unchanged':all(sha(ROOT/v['origin'])==v['sha256']==sha(p/v['snapshot']) for v in pr['sources']),'bounds':bool(np.array_equal(pr['input_bounds'],bounds) and np.all(r['exact_input_bounds']==bounds)),'applied_clipping':bool(np.array_equal(r['exact_applied'],np.clip(r['exact_native_requested'],-bounds,bounds))),'prior_alignment':bool(np.array_equal(r['exact_previous_applied'][1:],r['exact_applied'][:-1])),'rollout_qpos':bool(np.array_equal(l['qpos'],r['exact_qpos'].astype(l['qpos'].dtype))),'rollout_applied':bool(np.array_equal(l['input'],r['exact_applied'].astype(l['input'].dtype))),'rollout_requested':bool(np.array_equal(l['command'],r['exact_native_requested'].astype(l['command'].dtype))),'one_per_5ms':bool(np.allclose(np.diff(r['time_s']),.005,atol=1e-10,rtol=0)),'no_input_exceeds_own_cap':bool(np.all(np.abs(r['exact_applied'])<=bounds+1e-12)),'design_bounds':pr['design']['design_bounds']==[1.5,.6,.6,.3,.3]}
  cases[ident]={'cap_m_s':cap,'samples':n,'completed':su['completed'],'failure':su['failure'],'passed':all(c.values()),'checks':c}
 hpath=ROOT/'results/lqr_contact_forecast_20261125/campaign/runs/mars_figure8_p090/precision.npz';before['inputs'][str(hpath.relative_to(ROOT))]=sha(hpath);hist=load(hpath);base=arrays['mars_cap030'];parity={}
 for key in hist:
  if key in base:
   same=np.array_equal(hist[key],base[key],equal_nan=True);parity[key]={'bit_identical':bool(same),'maximum_absolute_gap':float(np.nanmax(abs(hist[key]-base[key]))) if hist[key].size else 0.}
 checks['baseline_historical_all_common_arrays_identical']=all(v['bit_identical'] for v in parity.values())
 checks['fixed_gain_bank']=len({p['design']['gain_bank_sha256'] for p in provs.values()})==1
 checks['fixed_ordinary_design']=len({p['ordinary_design_sha256'] for p in provs.values()})==1
 for ident,r in arrays.items():
  n=len(r['time_s']);checks[ident+'_reference_identical']=all(np.array_equal(r[k],base[k][:n]) for k in ['time_s','exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2'])
  checks[ident+'_initial_state_identical']=all(np.array_equal(r[k][0],base[k][0]) for k in ['exact_qpos','exact_qvel','exact_observer_wrench','exact_estimated_normal','exact_nominal_joints_before'])
 checks['input_hashes_stable']=all(sha(ROOT/rel)==digest for rel,digest in before['inputs'].items())
 before['capture_note']='Hashes obtained before saved-array comparisons and rechecked afterward; no physics or control called.'
 (OUT/'inputs.json').write_text(json.dumps(before,indent=2)+'\n')
 result={'iteration':'I-49','passed':all(c['passed'] for c in cases.values()) and all(checks.values()),'cases':cases,'checks':checks,'baseline_history_parity':parity,'scope':'Capacity bounds/total clipping/measurement history/gain-bank/reference/source checks. Completion/fall is an outcome, not a harness-pass requirement.','source_sha256':before['source_sha256']}
 (OUT/'report.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));assert result['passed']
if __name__=='__main__':main()
