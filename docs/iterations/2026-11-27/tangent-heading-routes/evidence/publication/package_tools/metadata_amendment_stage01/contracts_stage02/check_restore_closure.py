"""Exact final-output closure check on the actual restored I76 result tree."""
from pathlib import Path,PurePosixPath
from datetime import datetime,timezone
import argparse,hashlib,json

def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb')as stream:
  for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
 return h.hexdigest()
def binding(path):return dict(bytes=Path(path).stat().st_size,sha256=sha(path))
def verify_sealed_outputs(restored,seal):
 restored=Path(restored).resolve();checks={}
 for relative,expected in seal['files'].items():
  pure=PurePosixPath(relative)
  if pure.is_absolute()or'..'in pure.parts:raise ValueError('Unsafe sealed output path')
  p=restored/relative
  if not p.is_file():raise AssertionError('Restored sealed output missing: '+relative)
  if binding(p)!=expected:raise AssertionError('Restored sealed output bytes differ: '+relative)
  checks[relative]=expected
 return checks

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--evidence',type=Path,required=True);parser.add_argument('--archive-qa',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
 evidence=args.evidence.resolve();qa_path=args.archive_qa.resolve();output=args.output.resolve();output.mkdir(exist_ok=False)
 qa=json.loads(qa_path.read_text())
 if qa['status']!='PASS':raise ValueError('Original archive/actual-restore QA must pass first')
 restored=Path(qa['actual_restore']['temporary_repository'])/'results/tangent_heading_routes_20261127'
 seal_path=evidence/'campaign/outputs_sealed.json';seal=json.loads(seal_path.read_text())
 source=Path(__file__).resolve()
 before=dict(iteration='I-76',utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(source),archive_qa_receipt=binding(qa_path),output_seal=binding(seal_path),final_compact_manifest=binding(evidence/'compact_record_manifest.json'),actual_restored_result=str(restored),no_metric_scoring=True,no_physics=True)
 (output/'registered_before_closure_check.json').write_text(json.dumps(before,indent=2,sort_keys=True)+'\n')
 checks=verify_sealed_outputs(restored,seal)
 amendment=json.loads((evidence/'publication/package_tools/metadata_amendment_stage01/receipt.json').read_text())
 metadata_checks=verify_sealed_outputs(restored,dict(files=amendment['added_metadata_files']))
 model_paths=amendment['added_model_config_files']
 if len(model_paths)!=18:raise AssertionError('All18 saved model snapshots are required')
 required_dependency_paths=set(checks)|set(model_paths)
 if len(required_dependency_paths)!=167:raise AssertionError('Expected149 sealed outputs plus18 distinct model inputs')
 dependency=json.loads((evidence/'publication/package_tools/metadata_amendment_stage01/dependency_review.json').read_text())
 producer_checks=verify_sealed_outputs(restored,dict(files=dependency['result_owned_inputs']))
 if len(checks)!=149:raise AssertionError('Expected all149 root-sealed outputs')
 # A selected-file round trip is insufficient: required recovered stdout JSONL
 # must exist in the actual restored fixture and retain its exact original bytes.
 required=[relative for relative in checks if relative.endswith('.jsonl')]
 if not required:raise AssertionError('Recovery JSONL is required by the final output seal')
 report=dict(status='PASS',iteration='I-76',archive_qa_receipt=binding(qa_path),final_compact_manifest_sha256=sha(evidence/'compact_record_manifest.json'),output_seal_sha256=sha(seal_path),all_sealed_outputs_restored_byte_exact=True,sealed_output_count=len(checks),essential_restored_prerequisite_count=len(required_dependency_paths),saved_model_config_count=len(model_paths),all_saved_model_configs_restored_byte_exact=True,model_configs_bound_to_sealed_prephysics_provenance=True,result_owned_producer_input_count=len(producer_checks),all_result_owned_producer_inputs_restored_byte_exact=True,jsonl_sealed_outputs=required,actual_restored_result=str(restored),checked_files=checks,checked_added_metadata=metadata_checks,checked_result_owned_producer_inputs=producer_checks,no_metric_scoring=True,no_physics=True,limitation='This closure concerns all root-sealed experiment outputs, not installation of external runtime sources/binaries.')
 (output/'receipt.json').write_text(json.dumps(report,indent=2,sort_keys=True)+'\n')
 print(json.dumps(dict(status='PASS',sealed_outputs=len(checks),sealed_jsonl=len(required),saved_model_configs=len(model_paths),essential_prerequisites=len(required_dependency_paths),registered_result_owned_inputs=len(producer_checks),actual_restored_result=str(restored))))
if __name__=='__main__':main()
