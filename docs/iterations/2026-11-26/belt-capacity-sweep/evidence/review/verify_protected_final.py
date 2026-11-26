"""I49 read-only final protected-origin/archive verification; no tests/physics."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json

FAMILY=Path(__file__).resolve().parents[1]
ROOT=FAMILY.parents[1]
OUTPUT=FAMILY/'review/protected_final'
EXCLUDED={'ITERATION_LOG.md','docs/iterations/2026-11-26/README.md'}


def sha(path):
    h=hashlib.sha256()
    with path.open('rb')as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def save(path,value):path.write_text(json.dumps(value,indent=2)+'\n')


def main():
    OUTPUT.mkdir(exist_ok=False)
    source=Path(__file__).resolve();(OUTPUT/source.name).write_bytes(source.read_bytes())
    before=FAMILY/'pre_change/manifest.json'
    manifest=json.loads(before.read_text())
    save(OUTPUT/'before_verification.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(source),pre_change_manifest_sha256=sha(before),excluded_living_notes=sorted(EXCLUDED),
        scope='Read-only current origin/snapshot/archive hashing; no tests, controller or physics.'))
    snapshots=[];origins=[];excluded=[]
    for row in manifest['sources']:
        snap=FAMILY/row['snapshot'];origin=ROOT/row['path']
        snapshots.append(dict(path=row['snapshot'],passed=sha(snap)==row['sha256'],sha256=sha(snap)))
        result=dict(path=row['path'],pre_change_sha256=row['sha256'],current_sha256=sha(origin))
        if row['path']in EXCLUDED:
            result['reason']='Authorized living research note update; not represented as unchanged experiment source'
            excluded.append(result)
        else:
            result['passed']=result['current_sha256']==row['sha256'];origins.append(result)
    archives=[dict(path=row['path'],passed=sha(ROOT/row['path'])==row['sha256'] and (ROOT/row['path']).stat().st_size==row['bytes'],
        sha256=sha(ROOT/row['path']),bytes=(ROOT/row['path']).stat().st_size)for row in manifest['prior_archives']]
    runner=FAMILY/'run_capacity_case.py';verify=FAMILY/'verify_capacity_runner.py'
    final_core=json.loads((FAMILY/'review/core_checks_stage01/report.json').read_text())
    frozen={row['origin']:row['sha256']for row in final_core['sources']}
    new_sources=[dict(path=str(p.relative_to(ROOT)),passed=sha(p)==frozen[str(p.relative_to(ROOT))],sha256=sha(p))for p in (runner,verify,FAMILY/'protocol.json',FAMILY/'campaign/jobs.json')]
    case_sources=[]
    for case in ('mars_cap030','mars_cap060','mars_cap090'):
        p=FAMILY/'campaign/runs'/case/'provenance.json';provenance=json.loads(p.read_text())
        case_sources.append(dict(case_id=case,all_run_sources_unchanged=all(sha(ROOT/r['origin'])==r['sha256']for r in provenance['sources']),
            bridge_binary_unchanged=sha(ROOT/provenance['bridge_binary']['path'])==provenance['bridge_binary']['sha256']))
    report=dict(passed=all(r['passed']for r in snapshots+origins+archives+new_sources)and all(all(v for k,v in r.items()if k!='case_id')for r in case_sources),
        pre_change_source_count=len(manifest['sources']),protected_origin_count=len(origins),snapshot_count=len(snapshots),
        prior_archive_count=len(archives),excluded_living_notes=excluded,protected_origins=origins,pre_change_snapshots=snapshots,
        prior_archives=archives,final_frozen_runner_verifier_protocol_jobs=new_sources,executed_case_sources=case_sources,
        scope='Final protected-source/archive integrity only; no existing runtime/config/test files edited by I49 and no additional test or simulation execution.')
    assert report['pre_change_source_count']==152 and report['protected_origin_count']==150 and len(excluded)==2 and len(archives)==17
    save(OUTPUT/'report.json',report)
    assert report['passed']
    save(OUTPUT/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)for p in sorted(OUTPUT.iterdir())if p.is_file()and p.name!='artifact_manifest.json']))
    print(json.dumps({k:report[k]for k in ('passed','pre_change_source_count','protected_origin_count','snapshot_count','prior_archive_count')},indent=2))


if __name__=='__main__':main()
