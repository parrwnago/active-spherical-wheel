"""Freeze I77 source/config audit before model or data arithmetic."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,shutil,subprocess
ROOT=Path(__file__).resolve().parents[3]
STAGE=Path(__file__).resolve().parent
BASELINE=ROOT/'results/tangent_heading_routes_20261127'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
 p=STAGE/'registered_before_source_analysis.json'
 if p.exists():raise FileExistsError('Preserve earlier audit registration')
 sources=list((ROOT/'src/spherical_wheel').rglob('*.py'))+list((ROOT/'scripts').glob('*.py'))+[ROOT/'config/model.yaml',Path(__file__).resolve()]
 for folder in ('results/causal_replay_20261126','results/handoff_factorial_20261125','results/feedback_recovery_20261125','results/belt_handoff_investigation_20261126','results/support_routing_timing_20261126'):
  directory=ROOT/folder
  if directory.exists():
   sources += [q for q in directory.rglob('*.py')if not any(v in q.parts for v in ('source','source_snapshot','source_before','__pycache__'))]
 sources=sorted(set(sources))
 protocol=BASELINE/'protocol/protocol.json';value=json.loads(protocol.read_text())
 if sha(protocol)!='53106cdb758d76436e04b747c9483c4081abcc7ec1f2a163ace90b58b82085c2':raise ValueError('I76 baseline protocol changed')
 inputs=[protocol,BASELINE/'campaign/outputs_sealed.json',ROOT/value['gain_cache']['path']]
 inputs += [BASELINE/'campaign/runs'/job['case_id']/'provenance.json'for job in value['cases']if job['policy']=='N']
 for source in sources:
  target=STAGE/'source'/source.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
 for source in inputs:
  if source.suffix=='.json':
   target=STAGE/'input_metadata'/source.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
 registered=dict(iteration='I-77',utc=datetime.now(timezone.utc).isoformat(),question='Inventory the ACTIVE original baseline command chain, distinguish known algebraic pathways from physical-cause hypotheses, and identify minimal same-prefix interventions.',git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),source_sha256={str(q.relative_to(ROOT)):sha(q)for q in sources},input_sha256={str(q.relative_to(ROOT)):sha(q)for q in inputs},baseline_protocol_sha256=sha(protocol),bound_gain_cache=value['gain_cache'],no_new_physics=True,no_case_array_contents_loaded=True,audit_boundary='Source and existing provenance/configuration only. No new rollout, gain design or contact oracle in this audit. Literature-like comments are not treated as I76 causal evidence.')
 p.write_text(json.dumps(registered,indent=2,sort_keys=True)+'\n');print(json.dumps(dict(sources=len(sources),inputs=len(inputs),baseline_protocol_sha256=sha(protocol))))
if __name__=='__main__':main()
