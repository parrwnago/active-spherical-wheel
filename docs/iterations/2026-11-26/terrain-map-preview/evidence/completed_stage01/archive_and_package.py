"""Seal I44 compact research evidence and a review/reproduction bundle.

Large derived map snapshots stay local; raw depth and exact input states are
bundled so they can be regenerated. Prior iterations are referenced by hashes.
"""
from pathlib import Path
import hashlib,json,shutil,zipfile

ROOT=Path(__file__).resolve().parents[2]
FAMILY=ROOT/'results/terrain_memory_20261126'
DOC=ROOT/'docs/iterations/2026-11-26/terrain-map-preview'
COMPACT=DOC/'evidence/completed_stage01'
BUNDLE=FAMILY/'presentation-pack.zip'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def write(path,value):path.write_text(json.dumps(value,indent=2)+'\n')

def main():
 assert not BUNDLE.exists() and not COMPACT.exists(),'Preserve prior packaging; choose a fresh stage.'
 analysis=FAMILY/'campaign/analyze_stage01'
 assert (analysis/'artifact_manifest.json').exists()
 assert (FAMILY/'videos/terrain_memory_shadow.mp4').exists()
 required_qa=[FAMILY/'review/depth_raw_qa_stage02/report.json',FAMILY/'review/memory_raw_qa_stage01/report.json',FAMILY/'tests/stage01_sealed/report.json']
 for p in required_qa:assert p.exists(),p
 pre=json.loads((FAMILY/'pre_change/manifest.json').read_text())
 originals=[r for r in pre['source'] if r['path'].startswith(('src/','scripts/','tests/','config/'))or r['path']=='pyproject.toml']
 for r in originals:assert sha(ROOT/r['path'])==r['sha256'],r['path']
 for r in pre['prior_immutable_evidence']:assert sha(ROOT/r['path'])==r['sha256'],r['path']
 # Current native/new-source state must match the final selected-test snapshot.
 sealed=json.loads((FAMILY/'tests/stage01_sealed/source_manifest.json').read_text())
 for r in sealed['sources']:assert sha(ROOT/r['path'])==r['sha256'],r['path']
 compact_records=[]
 COMPACT.mkdir(parents=True)
 # Keep complete small development sources/logs, including failures. Never copy
 # scene meshes, arrays or video into the tracked compact-evidence directory.
 for p in sorted(FAMILY.rglob('*')):
  if not p.is_file():continue
  rel=p.relative_to(FAMILY)
  if any(x in rel.parts for x in ('scene','evaluation_scene','robot','_render_scenes','_scene')):continue
  if p.suffix not in ('.json','.py','.log','.txt','.png','.jpg','.pdf','.svg'):continue
  if 'source' in rel.parts and not rel.parts[0] in ('memory_development','adapter_development'):continue
  if p.name=='predictions.json' or p.stat().st_size>2_000_000:continue
  target=COMPACT/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
  compact_records.append({'origin':str(p.relative_to(ROOT)),'copy':str(target.relative_to(DOC)),'sha256':sha(p),'bytes':p.stat().st_size})
 # One final full Python/config snapshot; preserve path context.
 snapshot=COMPACT/'final_source'
 for p in [ROOT/'scripts/run-terrain-memory.py',ROOT/'pyproject.toml',ROOT/'config/model.yaml',
           *sorted((ROOT/'src/spherical_wheel').rglob('*.py')),
           *sorted((ROOT/'tests').glob('test_terrain*.py'))]:
  target=snapshot/p.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
  compact_records.append({'origin':str(p.relative_to(ROOT)),'copy':str(target.relative_to(DOC)),'sha256':sha(p),'bytes':p.stat().st_size})
 predictions=json.loads((analysis/'predictions.json').read_text())
 examples=[r for r in predictions if 'dense_bump'in r['strata']or abs(r['anchor_time_s']-30.)<1e-8 or abs(r['anchor_time_s']-43.6)<1e-8]
 write(COMPACT/'prediction_examples.json',{'scope':'Dense bump rows plus fixed30s/43.6s examples, not all forecast slots','rows':examples})
 write(COMPACT/'manifest.json',{'base_for_origin':'repository root','base_for_copy':'iteration directory','files':compact_records,'omissions':'Full depth arrays, full forecasts, large derived map snapshots, optical scene assets and MP4 remain local; selected originals are included in the review bundle.'})
 paths={}
 def add(p):
  p=p.resolve();assert p.is_file();paths[str(p.relative_to(ROOT))]=p
 # Full campaign/development evidence, excluding only the very large regenerable
 # map-cell snapshot and partial packaging metadata.
 omitted=[]
 for p in FAMILY.rglob('*'):
  if not p.is_file():continue
  if p.name in ('presentation-pack.zip','presentation-pack.zip.sha256','presentation_manifest.json','presentation_verification.json'):continue
  if p.name=='memory_snapshots.npz':
   omitted.append({'path':str(p.relative_to(ROOT)),'sha256':sha(p),'bytes':p.stat().st_size,'reason':'Derived full map snapshots; regenerate from bundled raw depth and source. Video and compact reports included.'});continue
  if p.suffix=='.pyc' or '__pycache__'in p.parts:continue
  add(p)
 for folder in [ROOT/'docs/iterations/2026-11-26',ROOT/'generated/offroad/flat',ROOT/'assets/cad/visuals']:
  for p in folder.rglob('*'):
   if p.is_file():add(p)
 for p in [ROOT/'ITERATION_LOG.md',ROOT/'AGENTS.md',ROOT/'pyproject.toml',ROOT/'config/model.yaml',ROOT/'scripts/run-terrain-memory.py',ROOT/'GAZEBO.md',ROOT/'scripts/fetch-marsyard.sh']:
  add(p)
 for p in (ROOT/'src/spherical_wheel').rglob('*.py'):add(p)
 for p in (ROOT/'tests').glob('test_terrain*.py'):add(p)
 protocol=json.loads((FAMILY/'protocol.json').read_text())
 for name in ('precision','rollout','provenance'):add(ROOT/protocol[name])
 metadata=json.loads((FAMILY/'campaign/depth_stage01/scene_metadata.json').read_text())
 for item in metadata['source_assets']:add(Path(item['path']))
 payload=[{'path':name,'sha256':sha(p),'bytes':p.stat().st_size}for name,p in sorted(paths.items())]
 manifest={'iteration':'I-44','scope':'Review/reproduction inputs for one reused trajectory, not additional robot trials','files':payload,'omitted_derived_outputs':omitted,'preexisting_sources_unchanged':len(originals),'prior_artifacts_unchanged':len(pre['prior_immutable_evidence']),'compact_manifest':str((COMPACT/'manifest.json').relative_to(ROOT)),'reproduction':'Restore source at original repo paths in isolated checkout, install pinned dependencies, then run documented fresh stage tags. XML absolute paths remain historical provenance; scene factories regenerate them for the current root.'}
 write(FAMILY/'presentation_manifest.json',manifest)
 with zipfile.ZipFile(BUNDLE,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6)as archive:
  for name,p in sorted(paths.items()):archive.write(p,name)
  archive.write(FAMILY/'presentation_manifest.json','results/terrain_memory_20261126/presentation_manifest.json')
 with zipfile.ZipFile(BUNDLE)as archive:
  assert archive.testzip()is None
  for item in payload:assert hashlib.sha256(archive.read(item['path'])).hexdigest()==item['sha256'],item['path']
 (FAMILY/'presentation-pack.zip.sha256').write_text(sha(BUNDLE)+'  presentation-pack.zip\n')
 verification={'archive':str(BUNDLE.relative_to(ROOT)),'sha256':sha(BUNDLE),'bytes':BUNDLE.stat().st_size,'payload_files_verified':len(payload),'zip_members':len(payload)+1,'crc_passed':True,'compact_files':len(compact_records),'compact_bytes':sum(r['bytes']for r in compact_records),'excluded_large_derived':omitted,'prior_artifacts_unchanged':len(pre['prior_immutable_evidence']),'existing_sources_unchanged':len(originals)}
 write(FAMILY/'presentation_verification.json',verification)
 write(DOC/'final_verification.json',verification)
 print(json.dumps(verification,indent=2))

if __name__=='__main__':main()
