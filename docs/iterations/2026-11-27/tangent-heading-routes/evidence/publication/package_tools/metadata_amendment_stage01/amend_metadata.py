"""Documented post-collection metadata coverage amendment; never rewrite raw tar.

The tested initial builder omitted .jsonl from its compact suffix set. Preserve
that initial manifest, add missing recovery JSONL and model snapshots, and retain
this correction's source/regression receipts before regenerating the manifest.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil

STAGE=Path(__file__).resolve().parent
ROOT=STAGE.parents[4]
BASE=ROOT/'results/tangent_heading_routes_20261127'
SMALL={'.py','.json','.md','.log','.txt'}
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb')as stream:
  for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
def record(p):return dict(bytes=Path(p).stat().st_size,sha256=sha(p))
def save(p,v):
 with Path(p).open('x')as f:json.dump(v,f,indent=2,sort_keys=True);f.write('\n')
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--evidence',type=Path,required=True);parser.add_argument('--bundle',type=Path,required=True);args=parser.parse_args()
 evidence=args.evidence.resolve();bundle=args.bundle.resolve();manifest_path=evidence/'compact_record_manifest.json'
 oldbytes=manifest_path.read_bytes();old=json.loads(oldbytes)
 expected=set(old['files'])|{'compact_record_manifest.json'}
 actual={str(p.relative_to(evidence))for p in evidence.rglob('*')if p.is_file()}
 if actual!=expected:raise AssertionError('Initial compact field set changed before amendment')
 for relative,expected_record in old['files'].items():
  if record(evidence/relative)!=expected_record:raise AssertionError('Initial compact bytes changed:'+relative)
 inventory=json.loads((evidence/old['bundle_inventory']).read_text())
 raw_before=record(bundle)
 if raw_before!={k:inventory[k]for k in ('bytes','sha256')}:raise AssertionError('Initial raw bundle changed')
 packaging=json.loads((evidence/'publication/package_tools/registered_before_packaging.json').read_text())
 builder=BASE/'publication/package_tools/build_record.py'
 if record(builder)!=packaging['files'][str(builder.relative_to(ROOT))]:raise AssertionError('Original tested builder bytes changed')
 seal_path=BASE/'campaign/outputs_sealed.json';seal=json.loads(seal_path.read_text())
 jsonl_sources=sorted((BASE/'review').rglob('*.jsonl'))
 if not jsonl_sources:raise AssertionError('Expected recovery JSONL sources')
 protocol=json.loads((BASE/'protocol/protocol.json').read_text())
 protocol_config_sha=protocol['source_sha256']['config/model.yaml'];model_sources=[];model_provenance={}
 for case in protocol['cases']:
  folder=BASE/'campaign/runs'/case['case_id'];provenance=folder/'provenance_before.json';relative=str(provenance.relative_to(BASE))
  if record(provenance)!=seal['files'][relative]:raise AssertionError('Prephysics provenance differs from root seal')
  entries=[entry for entry in json.loads(provenance.read_text())['sources']if entry['origin']=='config/model.yaml']
  if len(entries)!=1:raise AssertionError('One original model config source required')
  entry=entries[0];source=folder/entry['snapshot'];value=record(source)
  if value['sha256']!=entry['sha256']or value['sha256']!=protocol_config_sha:raise AssertionError('Model snapshot does not match sealed prephysics provenance/protocol')
  model_sources.append(source);model_provenance[str(source.relative_to(BASE))]=dict(case_id=case['case_id'],sealed_prephysics_provenance_path=relative,sealed_prephysics_provenance=record(provenance),provenance_config_sha256=entry['sha256'],protocol_config_sha256=protocol_config_sha)
 if len(model_sources)!=18:raise AssertionError('All18 model snapshots required')
 sources=sorted(jsonl_sources+model_sources)
 bindings={str(p.relative_to(BASE)):record(p)for p in sources}
 for relative,value in bindings.items():
  if relative in seal['files']and value!=seal['files'][relative]:raise AssertionError('Recovery JSONL differs from root seal')
 save(STAGE/'registered_before_amendment.json',dict(iteration='I-76',utc=datetime.now(timezone.utc).isoformat(),initial_compact_manifest=record(manifest_path),unchanged_raw_bundle=raw_before,unchanged_tested_builder=record(builder),root_output_seal=record(seal_path),metadata_sources=bindings,model_config_provenance=model_provenance,source_sha256=sha(Path(__file__)),cause='Initial tested collector omitted .jsonl from suffix selection and did not recurse into canonical per-case source_snapshot/config; recovery JSONL and renderer model snapshots were absent.',operation='Explicit metadata-only coverage amendment, with initial manifest preserved; no physics, metric, simulation or video changes.'))
 initial=STAGE/'initial_compact_record_manifest.json'
 with initial.open('xb')as f:f.write(oldbytes)
 copied={}
 for p in sources:
  relative=p.relative_to(BASE);target=evidence/relative
  if target.exists():
   if record(target)!=bindings[str(relative)]:raise AssertionError('Conflicting recovery metadata in compact tree')
  else:target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
  copied[str(relative)]=record(target)
 if any(record(BASE/relative)!=binding for relative,binding in bindings.items()):raise AssertionError('Recovery source changed')
 if record(bundle)!=raw_before:raise AssertionError('Raw tar bytes changed during metadata amendment')
 save(STAGE/'receipt.json',dict(iteration='I-76',status='PASS',initial_compact_manifest_sha256=hashlib.sha256(oldbytes).hexdigest(),initial_compact_manifest_preserved='publication/package_tools/metadata_amendment_stage01/initial_compact_record_manifest.json',added_metadata_files=copied,added_metadata_bytes=sum(v['bytes']for v in copied.values()),added_jsonl_files=[str(p.relative_to(BASE))for p in jsonl_sources],added_model_config_files=[str(p.relative_to(BASE))for p in model_sources],model_config_provenance=model_provenance,root_sealed_jsonl=[p for p in copied if p.endswith('.jsonl')and p in seal['files']],raw_bundle_unchanged=raw_before,builder_bytes_unchanged=True,no_raw_recompression=True,no_physics=True,no_metric_scoring=True,no_video_rerender=True,final_actual_restore_closure_required=True,closure_tool='publication/package_tools/metadata_amendment_stage01/check_restore_closure.py'))
 # Correction sources/history are a distinct post-collection stage, not passed
 # off as files selected by the original constructor or its pre-pack binding.
 destination=evidence/'publication/package_tools/metadata_amendment_stage01'
 for p in sorted(STAGE.rglob('*')):
  if p.is_file()and'__pycache__'not in p.parts and p.suffix in SMALL:
   q=destination/p.relative_to(STAGE);q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
 final=dict(old)
 final['files']={str(p.relative_to(evidence)):record(p)for p in sorted(evidence.rglob('*'))if p.is_file()and p!=manifest_path}
 final['metadata_coverage_amendment']=dict(stage='publication/package_tools/metadata_amendment_stage01',initial_compact_manifest_sha256=hashlib.sha256(oldbytes).hexdigest(),reason='Recovery JSONL and canonical renderer model snapshots omitted by original compact selection',jsonl_files=len(jsonl_sources),model_config_snapshots=len(model_sources),raw_bundle_sha256=raw_before['sha256'],raw_bundle_bytes_unchanged=True,requires_actual_restore_all_sealed_outputs=True)
 manifest_path.write_text(json.dumps(final,indent=2,sort_keys=True)+'\n')
 print(json.dumps(dict(status='PASS',initial_files=len(old['files']),final_files=len(final['files']),bound_bytes=sum(v['bytes']for v in final['files'].values()),added_jsonl=len(jsonl_sources),added_model_configs=len(model_sources),final_manifest_sha256=sha(manifest_path),raw_bundle_sha256=raw_before['sha256'])))
if __name__=='__main__':main()
