"""Copy small I46 evidence with origin/hash lineage; never copy raw or video data.

Default is a read-only selection preview.  --execute requires sealed science,
independent QA and final media QA, then atomically creates a fresh evidence dir.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import shutil

FAMILY = Path(__file__).resolve().parent
ROOT = FAMILY.parents[1]
TARGET = ROOT/'docs/iterations/2026-11-26/support-routing-timing/evidence'
SMALL = {'.json', '.csv', '.log', '.txt', '.patch', '.py', '.png', '.jpg', '.svg', '.pdf'}
AUDIT_NAMES = {'support_routing_audit.py', 'audit-support-routing.py',
               'test_support_routing_audit.py', 'test_support_routing_campaign.py',
               'reduce_routing.py', 'audit-support-routing-flat.py'}
NOTES = ['ITERATION_LOG.md', 'docs/iterations/2026-11-26/README.md',
         'docs/iterations/2026-11-26/support-routing-timing/README.md']


def sha(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


def select():
    selected = {}
    skipped = []
    def add(path, relative, kind):
        if path.is_file():
            selected[str(relative)] = dict(origin=str(path.relative_to(ROOT)), evidence_path=str(relative),
                bytes=path.stat().st_size, sha256=sha(path), kind=kind)
    for name in ('protocol.json', 'protocol_clarifications.json', 'pre_change/manifest.json',
                 'campaign/jobs.json', 'copy_compact_evidence.py', 'packaging/build_bundle.py'):
        add(FAMILY/name, Path(name), 'registered settings / lineage / evidence-copy tool')
    for group in ('site_selection', 'core_development', 'review', 'tests', 'analysis',
                  'setup_correction_stage02', 'videos', 'plots'):
        base = FAMILY/group
        if not base.exists():
            continue
        for path in sorted(base.rglob('*')):
            if not path.is_file():
                continue
            rel = path.relative_to(FAMILY)
            parts = rel.parts
            if path.name.startswith('compact_copy_'):
                continue
            if group == 'videos' and path.suffix == '.jpg' and path.name not in {'video_site_intro.jpg', 'video_resultcard.jpg', 'decoded_frame_0220.jpg', 'decoded_frame_0440.jpg'}:
                continue
            if path.suffix not in SMALL or any(p in ('_workspace', '_workspaces', '_render_scenes', '__pycache__') for p in parts):
                continue
            if path.name == 'candidates.json':
                skipped.append(dict(origin=str(path.relative_to(ROOT)), reason='Full2091-site candidate arrays stay in the local bundle; strict/revised summary and maps retained.'))
                continue
            if 'source_snapshot' in parts and group != 'videos':
                continue
            if 'source' in parts and path.name not in AUDIT_NAMES and group != 'videos':
                continue
            if path.stat().st_size > 3*1024*1024:
                skipped.append(dict(origin=str(path.relative_to(ROOT)), reason='Above3MiB compact evidence limit; stays in local bundle.'))
                continue
            add(path, rel, 'retained report / review / failed stage / source / figure')
    for base in (FAMILY/'campaign/historical_stage01', FAMILY/'campaign/runs'):
        if not base.exists():
            continue
        for path in sorted(base.rglob('*.json')):
            if 'source_snapshot' in path.parts or path.name not in {
                'provenance.json', 'report.json', 'summary.json', 'terrain.json',
                'failure.json', 'execution_result.json', 'artifact_manifest.json'}:
                continue
            add(path, path.relative_to(FAMILY), 'native/replay outcome or retained setup failure, compact metadata only')
    for rel in ('src/spherical_wheel/support_routing_audit.py', 'scripts/audit-support-routing.py',
                'tests/test_support_routing_audit.py', 'tests/test_support_routing_campaign.py'):
        add(ROOT/rel, Path('source')/rel, 'new audit implementation / selected tests, final frozen source')
    for rel in NOTES:
        add(ROOT/rel, Path('research_notes_snapshot')/rel,
            'Final research-note snapshot; documentation, not the experiment source or an unchanged-origin claim')
    rows = [selected[key] for key in sorted(selected)]
    return rows, skipped


def readiness():
    required = {'analysis/stage01/report.json': None,
                'review/protected_origins_stage01/report.json': 'passed',
                'review/reduction_numeric_stage01/report.json': 'passed',
                'review/raw_integrity_stage02/report.json': 'passed',
                'videos/qa.json': 'passed'}
    results = []
    for rel, flag in required.items():
        path = FAMILY/rel
        if not path.is_file():
            raise RuntimeError('Evidence is not ready; missing '+rel)
        report = json.loads(path.read_text())
        if flag is not None and report.get(flag) is not True:
            raise RuntimeError('Evidence QA has not passed: '+rel)
        results.append(dict(path=str(path.relative_to(ROOT)), sha256=sha(path), required_pass_flag=flag))
    if not (FAMILY/'videos/support_routing_timing.mp4').is_file():
        raise RuntimeError('Expected final local video is absent')
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if args.execute:
        guards = readiness()
        if TARGET.exists():
            raise FileExistsError('Preserve existing evidence: '+str(TARGET))
    rows, skipped = select()
    if not args.execute:
        print(json.dumps(dict(executed=False, target=str(TARGET), files=len(rows),
            bytes=sum(r['bytes'] for r in rows), pending_media_guard='--execute requires videos/qa.json and final MP4',
            selected_origins=[r['origin'] for r in rows], retained_local=skipped), indent=2))
        return
    staging = TARGET.with_name(TARGET.name+'_staging')
    staging.mkdir(parents=True, exist_ok=False)
    try:
        for row in rows:
            origin, destination = ROOT/row['origin'], staging/row['evidence_path']
            if sha(origin) != row['sha256']:
                raise RuntimeError('Source changed during evidence preparation: '+str(origin))
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(origin, destination)
            if sha(destination) != row['sha256']:
                raise RuntimeError('Evidence copy hash mismatch')
        for row in rows:
            if sha(ROOT/row['origin']) != row['sha256']:
                raise RuntimeError('Source changed before evidence seal')
        manifest = dict(iteration='I-46', copied_utc=datetime.now(timezone.utc).isoformat(),
            files=len(rows), bytes=sum(r['bytes'] for r in rows), entries=rows, readiness_guards=guards,
            retained_local=skipped,
            excludes='RawNPZ,MP4,pluginbinaries,generatedworkspaces,sceneassets,oldarchives andfullsitecandidatearrays. These remain in the explicitly identified local presentation bundle.',
            self_manifest='This manifest is not included in its own entries; no recursive self-hash.',
            documentation_note='Research-note snapshots are final narrative text, accurately separate from tested/experiment source snapshots.')
        (staging/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
        staging.rename(TARGET)
        print(json.dumps(dict(copied=True, target=str(TARGET), files=len(rows), bytes=manifest['bytes'],
                             manifest_sha256=sha(TARGET/'manifest.json'))))
    except Exception as error:
        (staging/'copy_failure.json').write_text(json.dumps(dict(error=str(error)),indent=2)+'\n')
        raise


if __name__ == '__main__':
    main()
