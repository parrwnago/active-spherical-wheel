"""Seal I45 evidence, preserving prior results and excluding active packaging logs."""
from pathlib import Path
import hashlib,json,shutil,zipfile
ROOT=Path(__file__).resolve().parents[2]
FAMILY=ROOT/'results/contact_preview_pose_20261126'
DOC=ROOT/'docs/iterations/2026-11-26/contact-preview-pose-isolation'
COMPACT=DOC/'evidence/completed_stage01'
ZIP=FAMILY/'presentation-pack.zip'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,value):p.write_text(json.dumps(value,indent=2)+'\n')
def main():
 assert not ZIP.exists()and not COMPACT.exists(),'Never replace a prior packaging stage'
 stage=FAMILY/'campaign/stage01';assert(stage/'artifact_manifest.json').exists()
 assert(FAMILY/'videos/contact_pose_diagnostic.mp4').exists()
 assert(FAMILY/'review/final_report.json').exists()
 pre=json.loads((FAMILY/'pre_change/manifest.json').read_text())
 old=[r for r in pre['source']if r['path'].startswith(('src/','scripts/','tests/','config/'))or r['path']=='pyproject.toml']
 for r in old:assert sha(ROOT/r['path'])==r['sha256'],r['path']
 for r in pre['prior_archives']:assert sha(ROOT/r['path'])==r['sha256'],r['path']
 current=json.loads((FAMILY/'tests/final_selected/source_manifest.json').read_text())
 for r in current['sources']:assert sha(ROOT/r['path'])==r['sha256'],r['path']
 compact=[];COMPACT.mkdir(parents=True)
 skip_logs={'archive_and_package.log'}
 for p in sorted(FAMILY.rglob('*')):
  if not p.is_file()or p.name in skip_logs or '__pycache__'in p.parts:continue
  rel=p.relative_to(FAMILY)
  if p.suffix not in('.json','.py','.log','.txt','.png','.jpg','.svg','.pdf'):continue
  if p.name=='predictions.json'or p.stat().st_size>2_000_000:continue
  if 'source'in rel.parts:continue
  target=COMPACT/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
  compact.append({'origin':str(p.relative_to(ROOT)),'copy':str(target.relative_to(DOC)),'sha256':sha(p),'bytes':p.stat().st_size})
 sources=[ROOT/'scripts/diagnose-contact-preview-pose.py',ROOT/'scripts/run-terrain-memory.py',ROOT/'pyproject.toml',ROOT/'config/model.yaml',*sorted((ROOT/'src/spherical_wheel').rglob('*.py')),*sorted((ROOT/'tests').glob('test_contact_pose*.py')),*sorted((ROOT/'tests').glob('test_terrain*.py'))]
 for p in sources:
  target=COMPACT/'final_source'/p.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
  compact.append({'origin':str(p.relative_to(ROOT)),'copy':str(target.relative_to(DOC)),'sha256':sha(p),'bytes':p.stat().st_size})
 write(COMPACT/'manifest.json',{'origin_base':'repository root','copy_base':'I45 iteration directory','files':compact,'note':'Full per-query output/raw depth remains local and in selected review bundle inputs. Prior I44 snapshot remains immutable.'})
 files={}
 def add(p):
  p=p.resolve();assert p.is_file();files[str(p.relative_to(ROOT))]=p
 for p in FAMILY.rglob('*'):
  if not p.is_file()or '__pycache__'in p.parts or p.suffix=='.pyc':continue
  if p.name in skip_logs or p.name in('presentation-pack.zip','presentation-pack.zip.sha256','presentation_manifest.json','presentation_verification.json','post_seal_verification.json'):continue
  add(p)
 for p in DOC.rglob('*'):
  if p.is_file()and p.name not in('final_verification.json','post_seal_verification.json'):add(p)
 for p in [DOC.parent/'README.md',ROOT/'ITERATION_LOG.md',ROOT/'AGENTS.md',ROOT/'pyproject.toml',ROOT/'config/model.yaml',ROOT/'scripts/run-terrain-memory.py',ROOT/'scripts/diagnose-contact-preview-pose.py']:
  add(p)
 for p in(ROOT/'src/spherical_wheel').rglob('*.py'):add(p)
 for p in(ROOT/'tests').glob('test_contact_pose*.py'):add(p)
 for p in(ROOT/'tests').glob('test_terrain*.py'):add(p)
 for rel in ['docs/iterations/2026-11-26/terrain-map-preview/README.md','docs/iterations/2026-11-25/figure8-event-audit/README.md','docs/iterations/2026-11-25/handoff-factorial/README.md','docs/iterations/2026-11-25/tilt-reference/README.md']:
  add(ROOT/rel)
 protocol=json.loads((FAMILY/'protocol.json').read_text());external=[]
 for name,rel in protocol['inputs'].items():
  p=ROOT/rel
  if name in('I44_map_snapshots','I44_archive'):
   external.append({'name':name,'path':rel,'sha256':sha(p),'bytes':p.stat().st_size,'note':'Prior immutable dependency; large derived map can be regenerated from I44 raw inputs/source, then I45 rerun. Original remains local.'})
  else:add(p)
 for p in(ROOT/'generated/offroad/flat').rglob('*'):
  if p.is_file():add(p)
 payload=[{'path':name,'sha256':sha(p),'bytes':p.stat().st_size}for name,p in sorted(files.items())]
 manifest={'iteration':'I-45','scope':'One hindsight pose diagnostic, not a live policy or new robot trial','files':payload,'external_prior_dependencies':external,'reproduction':'Restore exact source paths in isolated checkout, use declared numerical environment and documented fresh stage name. Large prior snapshot/archive are referenced by hash and not nested in this bundle.','preexisting_sources_unchanged':len(old),'prior_archives_unchanged':len(pre['prior_archives'])}
 write(FAMILY/'presentation_manifest.json',manifest)
 with zipfile.ZipFile(ZIP,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6)as z:
  for name,p in sorted(files.items()):z.write(p,name)
  z.write(FAMILY/'presentation_manifest.json','results/contact_preview_pose_20261126/presentation_manifest.json')
 with zipfile.ZipFile(ZIP)as z:
  assert z.testzip()is None
  for item in payload:assert hashlib.sha256(z.read(item['path'])).hexdigest()==item['sha256'],item['path']
 result={'sha256':sha(ZIP),'bytes':ZIP.stat().st_size,'payload_files_verified':len(payload),'zip_members':len(payload)+1,'crc_passed':True,'compact_files':len(compact),'compact_bytes':sum(x['bytes']for x in compact),'prior_archives_unchanged':len(pre['prior_archives']),'preexisting_sources_unchanged':len(old),'external_large_prior_dependencies':external,'packaging_log_excluded_until_process_completion':True}
 write(FAMILY/'presentation_verification.json',result);write(DOC/'final_verification.json',result)
 (FAMILY/'presentation-pack.zip.sha256').write_text(result['sha256']+'  presentation-pack.zip\n')
 print(json.dumps(result,indent=2))
if __name__=='__main__':main()
