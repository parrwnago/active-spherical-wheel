"""One explicit synthetic partial-trace reduction+portable-replay contract.

Does not read any I58 run. Fabricated rows are marked SYNTHETIC and remain
inside a distinct fixture directory; they are not scientific run evidence.
"""
from pathlib import Path
from datetime import datetime,timezone
import importlib.util,json,shutil
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]

def module(path,name):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def main():
 reducer=module(HERE/'reduce.py','_synthetic_reducer');replay=module(HERE/'replay.py','_synthetic_replay')
 base=HERE/'synthetic_fixture_stage01';base.mkdir(exist_ok=False);(base/'analysis').mkdir()
 for name in('metrics.py','replay.py'):shutil.copyfile(HERE/name,base/'analysis'/name)
 protocol=json.loads((HERE.parent/'protocol.json').read_text())
 times=np.arange(8801)*.005
 reference=dict(time_s=times,exact_reference_xy=np.zeros((8801,2)),exact_reference_velocity_m_s=np.zeros((8801,2)),exact_reference_acceleration_m_s2=np.zeros((8801,2)))
 np.savez_compressed(base/'reference_program.npz',**reference)
 protocol['historical_baseline_paths']['m100_B']=str((base/'reference_program.npz').relative_to(ROOT))
 (base/'protocol.json').write_text(json.dumps(protocol))
 for job in protocol['cases']:
  if job['case_id']not in('m100_B','m100_A','m100_P'):continue
  n=9;t=times[:n];q=np.zeros((n,10));q[:,3]=1;u=np.zeros((n,5));v=np.zeros((n,9))
  raw=dict(time_s=t,exact_qpos=q,exact_qvel=v,exact_reference_qpos=q.copy(),exact_reference_xy=np.zeros((n,2)),
   exact_reference_velocity_m_s=np.zeros((n,2)),exact_reference_acceleration_m_s2=np.zeros((n,2)),exact_applied=u.copy(),
   exact_previous_applied=u.copy(),exact_input_bounds=np.tile([1.5,.6,.6,.3,.3],(n,1)),exact_requested=u.copy(),exact_native_requested=u.copy())
  folder=base/'campaign/runs'/job['case_id'];folder.mkdir(parents=True)
  np.savez_compressed(folder/'precision.npz',**raw);np.savez_compressed(folder/'rollout.npz',synthetic=np.array(True))
  summary=dict(completed=False,failure='SYNTHETIC early termination fixture')
  provenance=dict(case=job,source_hashes_unchanged_after=True,frozen_bank_loaded=True,
   fixed_bank_bytes_sha256=protocol['gain_cache']['sha256'],fixed_bank_numeric_sha256='SYNTHETIC-FIXTURE-NOT-A-RUN')
  diag=dict(equality_residual_max=0.,secondary_balance_delta=[0.,0.],first_stage_balance_delta=[0.,0.],
   tangent_confidence_proxy=1.,estimated_material_velocity_mismatch_m_s=0.,native_contact_velocity_residual_norm_m_s=0.,
   governed_contact_velocity_residual_norm_m_s=0.,model_seconds=0.,allocation_seconds=0.,total_seconds=0.,
   active_amplitude=[False]*5,active_slew=[False]*5,secondary_recovery_used=False,secondary_primary_restore_used=False,
   solver_warnings=[],relative_rotor_vertical_momentum_Nms=[0.,0.],proxy_relative_rotor_vertical_momentum_Nms=[0.,0.])
  governor=[dict(time_s=float(at),diagnostics=diag if job['variant']!='native'else{})for at in t]
  for name,payload in(('summary.json',summary),('provenance.json',provenance),('governor_diagnostics.json',governor)):
   reducer.save(folder/name,payload)
  if job['variant']in('native','hard_slew'):reducer.save(folder/'historical_precision_comparison.json',dict(all_shared_fields_exact=True,scope='SYNTHETIC flag for metric branch coverage, not a parity measurement.'))
  reducer.save(folder/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=reducer.sha(p))for p in folder.iterdir()if p.is_file()]))
 reducer.BASE=base
 stage=base/'reduced';reducer.reduce(stage)
 report=reducer.read(stage/'report.json')
 assert not report['broader_campaign_eligible']and not report['comparisons']['m100_P_vs_m100_B']['passes_all_conditions']
 assert 'full44_no_failure'in report['comparisons']['m100_P_vs_m100_B']['failed_checks']
 replay.replay(stage,base/'replay.json')
 reducer.save(base/'contract_result.json',dict(passed=True,scope=__doc__,completed_utc=datetime.now(timezone.utc).isoformat(),
  partial_case_rows=9,missing_cases_retained=4,portable_replay_exact=True,short_failed_screen_preserved=True,
  source_sha256={p.name:reducer.sha(p)for p in(HERE/'reduce.py',HERE/'metrics.py',HERE/'replay.py',Path(__file__))}))
 print('Synthetic partial-trace reduction and portable replay passed.')

if __name__=='__main__':main()
