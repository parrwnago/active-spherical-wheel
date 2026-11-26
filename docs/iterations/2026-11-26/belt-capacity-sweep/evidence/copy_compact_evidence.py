"""Select/copy compact I49 evidence, retaining failures and exact-source lineage.

Default previews selection only. --execute requires final independent science,
source and media QA, then atomically creates a fresh evidence directory.
"""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import shutil

FAMILY=Path(__file__).resolve().parent
ROOT=FAMILY.parents[1]
TARGET=ROOT/'docs/iterations/2026-11-26/belt-capacity-sweep/evidence'
SMALL={'.json','.csv','.log','.txt','.patch','.py','.png','.jpg','.md','.pdf'}
EXCLUDED_PARTS={'__pycache__','_workspace','_workspaces','_render_scenes'}
NOTES=('ITERATION_LOG.md','docs/iterations/2026-11-26/README.md','docs/iterations/2026-11-26/belt-capacity-sweep/README.md')


def sha(path):
    h=hashlib.sha256()
    with path.open('rb')as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def save(path,value):path.write_text(json.dumps(value,indent=2)+'\n')


def select():
    rows={};omitted=[]
    def add(path,destination,kind):
        if path.is_file():rows[str(destination)]=dict(origin=str(path.relative_to(ROOT)),evidence_path=str(destination),
            kind=kind,bytes=path.stat().st_size,sha256=sha(path))
    for rel in ('protocol.json','campaign/jobs.json','pre_change/manifest.json','run_capacity_case.py',
                'verify_capacity_runner.py','analyze_capacity.py','copy_compact_evidence.py','packaging/build_bundle.py'):
        add(FAMILY/rel,Path(rel),'registered settings, source or packaging lineage')
    for group in ('analysis','review','plots','videos'):
        base=FAMILY/group
        if not base.exists():continue
        for path in sorted(base.rglob('*')):
            if not path.is_file():continue
            relative=path.relative_to(FAMILY)
            if any(part in EXCLUDED_PARTS for part in relative.parts)or path.suffix not in SMALL:continue
            if path.name.startswith('compact_copy_'):continue
            if path.stat().st_size>4*1024*1024:
                omitted.append(dict(origin=str(path.relative_to(ROOT)),reason='Over4MiB compact limit; retained in local family/bundle.'));continue
            add(path,relative,'retained report, review, failed stage, exact staged source or plot/still')
    for path in sorted((FAMILY/'campaign').glob('*.log')):
        add(path,path.relative_to(FAMILY),'native execution log')
    for case in ('mars_cap030','mars_cap060','mars_cap090'):
        folder=FAMILY/'campaign/runs'/case
        for name in ('provenance.json','summary.json','terrain.json','execution_result.json','execution_failure.json','artifact_manifest.json'):
            add(folder/name,Path('campaign/runs')/case/name,'native case metadata; large raw arrays stay local/in bundle')
    for relative in NOTES:
        add(ROOT/relative,Path('research_notes_snapshot')/relative,'Final research narrative snapshot, not original experiment source or unchanged-origin claim')
    return [rows[k]for k in sorted(rows)],omitted


def readiness():
    flags=('review/core_checks_stage01/report.json','review/campaign_checks_stage01/report.json',
        'review/raw_stage01/report.json','review/metrics_numeric_stage02/report.json',
        'review/reducer_independent_stage01/report.json','review/protected_final/report.json','videos/qa.json')
    result=[]
    for rel in flags:
        path=FAMILY/rel
        if not path.is_file():raise RuntimeError('Missing required final QA: '+rel)
        if json.loads(path.read_text()).get('passed')is not True:raise RuntimeError('QA did not pass: '+rel)
        result.append(dict(origin=str(path.relative_to(ROOT)),sha256=sha(path),passed=True))
    for rel in ('analysis/stage01/report.json','videos/belt_capacity_comparison.mp4'):
        path=FAMILY/rel
        if not path.is_file():raise RuntimeError('Missing canonical result/media: '+rel)
        result.append(dict(origin=str(path.relative_to(ROOT)),sha256=sha(path)))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    rows,omitted=select()
    if not args.execute:
        print(json.dumps(dict(executed=False,target=str(TARGET),files=len(rows),bytes=sum(r['bytes']for r in rows),
            pending_guard='Execution requires all independent QA and final videos/qa.json passed',retained_local=omitted),indent=2));return
    guards=readiness()
    if TARGET.exists():raise FileExistsError('Preserve existing evidence: '+str(TARGET))
    stage=TARGET.with_name(TARGET.name+'_staging')
    stage.mkdir(parents=True,exist_ok=False)
    for row in rows:
        origin=ROOT/row['origin'];destination=stage/row['evidence_path']
        if sha(origin)!=row['sha256']:raise RuntimeError('Origin changed before copy: '+row['origin'])
        destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(origin,destination)
        if sha(destination)!=row['sha256']:raise RuntimeError('Copy hash mismatch: '+row['evidence_path'])
    for row in rows:
        if sha(ROOT/row['origin'])!=row['sha256']:raise RuntimeError('Origin changed during copy: '+row['origin'])
    manifest=dict(iteration='I-49',captured_utc=datetime.now(timezone.utc).isoformat(),files=rows,
        count=len(rows),bytes=sum(r['bytes']for r in rows),readiness_guards=guards,retained_local=omitted,
        boundaries='No rawNPZ, MP4, generated workspaces, plugin binaries or older archive payloads. Final media stays in explicitly linked local bundle. Negative checks, syntax failures and interpretation clarification retained.',
        self_hash='manifest.json is intentionally not self-hashed; outer bundle/final QA may bind it.')
    save(stage/'manifest.json',manifest)
    stage.rename(TARGET)
    print(json.dumps(dict(executed=True,target=str(TARGET),files=len(rows),bytes=manifest['bytes'],manifest_sha256=sha(TARGET/'manifest.json')),indent=2))


if __name__=='__main__':main()
