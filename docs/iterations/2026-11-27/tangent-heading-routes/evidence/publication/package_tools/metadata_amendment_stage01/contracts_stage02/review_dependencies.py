"""Source-registration review of result-owned inputs; no array/physics reads."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json
ROOT=Path(__file__).resolve().parents[5]
BASE=ROOT/'results/tangent_heading_routes_20261127'
STAGE=Path(__file__).resolve().parent
EVIDENCE=ROOT/'docs/iterations/2026-11-27/tangent-heading-routes/evidence'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb')as stream:
  for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
def binding(p):return dict(bytes=Path(p).stat().st_size,sha256=sha(p))
def main():
 manifest=json.loads((EVIDENCE/'compact_record_manifest.json').read_text());inventory=json.loads((EVIDENCE/manifest['bundle_inventory']).read_text())
 available=set(manifest['files'])
 for item in inventory['files']:
  p=ROOT/item['origin']
  if p.is_relative_to(BASE):available.add(str(p.relative_to(BASE)))
 paths=[BASE/'evaluation/completed_stage01/registered_before_metric_arithmetic.json',BASE/'publication/plots_stage01/registered_before_plot.json',BASE/'evaluation/command_audit_stage01/registered_before_arithmetic.json',BASE/'evaluation/retry_numeric_replay_stage01/registered_before_array_reads.json']
 paths+=sorted((BASE/'publication/videos_stage01').glob('*/*/registered_before_render.json'))
 required={};external=set();producer_groups={}
 def add(name,expected,group):
  p=Path(name);p=p if p.is_absolute()else ROOT/p
  if p.is_relative_to(BASE):
   relative=str(p.relative_to(BASE));actual=binding(p)
   if actual['sha256']!=expected['sha256']or('bytes'in expected and actual['bytes']!=expected['bytes']):raise AssertionError('Registered producer input changed:'+relative)
   if relative in required and required[relative]!=actual:raise AssertionError('Conflicting producer input binding')
   required[relative]=actual;producer_groups.setdefault(group,[]).append(relative)
  else:external.add(str(p))
 for path in paths:
  d=json.loads(path.read_text());values=d.get('inputs',d.get('files'))
  rows=values if isinstance(values,list)else[dict(path=k,**v)for k,v in values.items()]
  for row in rows:add(row['path'],row,str(path.relative_to(BASE)))
 protocol=json.loads((BASE/'protocol/protocol.json').read_text())
 for group in ('source_sha256','input_sha256'):
  for name,digest in protocol[group].items():add(name,dict(sha256=digest),'physics_protocol/'+group)
 proposed={str(p.relative_to(BASE))for p in(BASE/'review').rglob('*.jsonl')}
 proposed|={f'campaign/runs/{job["case_id"]}/source_snapshot/config/model.yaml'for job in protocol['cases']}
 missing=sorted(set(required)-available);after=sorted(set(required)-available-proposed)
 if after:raise AssertionError('Additional omitted result-owned producer inputs:'+str(after))
 output=dict(iteration='I-76',review_utc=datetime.now(timezone.utc).isoformat(),passed=True,result_owned_input_count=len(required),result_owned_inputs=required,source_registration_bindings={str(p.relative_to(BASE)):binding(p)for p in paths},producer_groups=producer_groups,initial_missing_registered_result_inputs=missing,missing_after_explicit_amendment=after,expected_metadata_amendment_files=sorted(proposed),external_source_asset_runtime_prerequisites=sorted(external),external_scope='External project sources/assets/binaries remain repo/runtime dependencies declared by the physics protocol; this amendment restores result-owned prerequisites.',source_review='Reviewed unchanged evaluation/plot/audit/replay/render result-owned input registrations and renderer direct model-snapshot reads. Final sealed-output closure additionally covers recovery JSONL referenced indirectly through the seal.',no_array_contents_loaded=True,no_metric_scoring=True,no_physics=True)
 with(STAGE/'dependency_review.json').open('x')as f:json.dump(output,f,indent=2,sort_keys=True);f.write('\n')
 print(json.dumps(dict(result_owned_inputs=len(required),initial_missing=len(missing),missing_after_amendment=len(after))))
if __name__=='__main__':main()
