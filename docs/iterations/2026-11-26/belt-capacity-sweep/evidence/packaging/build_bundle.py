from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,zipfile

ROOT=Path.cwd();FAMILY=ROOT/'results/belt_capacity_sweep_20261126';OUT=FAMILY/'presentation-pack.zip'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
paths=[]
for p in sorted(FAMILY.rglob('*')):
 if not p.is_file():continue
 rel=p.relative_to(FAMILY)
 if any(v in ('_workspace','_workspaces','__pycache__','.pytest_cache') for v in rel.parts):continue
 if rel.parts[0]=='packaging' and (p.suffix=='.log' or p.name=='bundle_report.json'):continue
 if p.suffix=='.zip' or p==OUT:continue
 paths.append((p,Path('local-evidence')/rel))
# Compact evidence and living research notes are the final reviewed text, not
# the source of earlier scientific executions (which have separate snapshots).
docs=ROOT/'docs/iterations/2026-11-26/belt-capacity-sweep'
for p in sorted(docs.rglob('*')):
 if p.is_file():paths.append((p,p.relative_to(ROOT)))
for p in [ROOT/'ITERATION_LOG.md',ROOT/'docs/iterations/2026-11-26/README.md']:
 paths.append((p,p.relative_to(ROOT)))
records=[{'path':str(rel),'origin':str(p.relative_to(ROOT)),'bytes':p.stat().st_size,'sha256':sha(p)} for p,rel in paths]
manifest={'iteration':'I-49','created_utc':datetime.now(timezone.utc).isoformat(),'payload_files':len(records),'payload_bytes':sum(v['bytes'] for v in records),'scope':'Three fresh native44s Marsfigure-eight capacityarms(.30/.60/.90), original fixedgain identities, exact state/command/observer/evaluator arrays, protocol, prework/source snapshots, targeted validation/independent numerical QA and recorded negative results, plots and corrected actual-state comparison video. Generated workspaces and prior archives are excluded. This is evidence, not a self-contained simulator installation or motor validation.','files':records}
assert not OUT.exists(), 'Preserve existing archive rather than overwrite'
with zipfile.ZipFile(OUT,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6,allowZip64=True) as z:
 for p,rel in paths:z.write(p,str(rel))
 z.writestr('bundle_manifest.json',json.dumps(manifest,indent=2)+'\n')
with zipfile.ZipFile(OUT) as z:
 assert z.testzip() is None
 assert len(z.namelist())==len(records)+1
 for item in records:
  assert hashlib.sha256(z.read(item['path'])).hexdigest()==item['sha256'],item['path']
report={'iteration':'I-49','passed':True,'archive':str(OUT.relative_to(ROOT)),'bytes':OUT.stat().st_size,'sha256':sha(OUT),'payload_files':len(records),'zip_members':len(records)+1,'payload_bytes':manifest['payload_bytes'],'all_payload_hashes_verified':True,'crc_passed':True,'packaging_source_sha256':sha(Path(__file__)),'excluded_active_packaging_logs':True}
(FAMILY/'packaging/bundle_report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
