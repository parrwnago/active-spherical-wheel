"""Read-only post-freeze source/archive verification, separate from mutable notes."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json

STAGE = Path(__file__).resolve().parent
FAMILY = STAGE.parents[1]
ROOT = FAMILY.parents[1]


def sha(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


def main():
    pre = json.loads((FAMILY/'pre_change/manifest.json').read_text())
    mutable = {'ITERATION_LOG.md', 'docs/iterations/2026-11-26/README.md',
               'docs/iterations/2026-11-26/support-routing-timing/README.md'}
    protected, notes, snapshots = [], [], []
    for item in pre['source']:
        row = dict(path=item['path'], expected_sha256=item['sha256'], actual_sha256=sha(ROOT/item['path']))
        row['unchanged'] = row['actual_sha256'] == row['expected_sha256']
        (notes if item['path'] in mutable else protected).append(row)
        path = FAMILY/item['snapshot']
        snapshots.append(dict(path=item['snapshot'], unchanged=sha(path) == item['sha256']))
    archives = []
    for item in pre['prior_archives']:
        path = ROOT/item['path']
        archives.append(dict(path=item['path'], expected_sha256=item['sha256'], actual_sha256=sha(path),
            expected_bytes=item['bytes'], actual_bytes=path.stat().st_size))
    for row in archives:
        row['unchanged'] = row['expected_sha256'] == row['actual_sha256'] and row['expected_bytes'] == row['actual_bytes']
    final = []
    core_stage = FAMILY/'core_development/stage02'
    for item in json.loads((core_stage/'source_manifest.json').read_text())['sources']:
        final.append(dict(path=item['path'], expected_sha256=item['sha256'],
            current_matches=sha(ROOT/item['path']) == item['sha256'],
            tested_snapshot_matches=sha(core_stage/item['snapshot']) == item['sha256']))
    campaign_stage = FAMILY/'review/campaign_checks_stage02'
    for item in json.loads((campaign_stage/'source_manifest.json').read_text())['files']:
        snapshot = campaign_stage/'source'/item['path']
        final.append(dict(path=item['path'], expected_sha256=item['sha256'],
            current_matches=sha(ROOT/item['path']) == item['sha256'],
            tested_snapshot_matches=sha(snapshot) == item['sha256'] if snapshot.is_file() else None,
            snapshot_note='Campaign source snapshot exists for owned test and driver; other sources also preserved by core stage.' if not snapshot.is_file() else None))
    correction = json.loads((FAMILY/'setup_correction_stage02/declaration_before_checks.json').read_text())
    correction_source = next(r for r in correction['sources_before_checks'] if r['path'].endswith('audit-support-routing-flat.py'))
    final.append(dict(path=correction_source['path'], expected_sha256=correction_source['sha256'],
        current_matches=sha(ROOT/correction_source['path']) == correction_source['sha256'], tested_snapshot_matches=None,
        snapshot_note='Separately preserved correction copy; exact diff/AST contract and pure setup checks retained.'))
    clarification = json.loads((FAMILY/'protocol_clarifications.json').read_text())
    plugin = [dict(path=path, expected_sha256=value, actual_sha256=sha(ROOT/path))
              for path, value in clarification['additional_pre_execution_source_sha256'].items()]
    for row in plugin:
        row['unchanged'] = row['actual_sha256'] == row['expected_sha256']
    result = dict(iteration='I-46', captured_utc=datetime.now(timezone.utc).isoformat(),
        passed=all(r['unchanged'] for r in protected+snapshots+archives+plugin)
            and all(r['current_matches'] and r['tested_snapshot_matches'] is not False for r in final),
        protected_origins_count=len(protected), protected_origins=protected,
        retained_prechange_snapshots=snapshots, prior_archive_count=len(archives), prior_archives=archives,
        final_tested_sources=final, plugin_and_registered_extra_sources=plugin,
        excluded_mutable_research_notes=notes,
        exclusion_reason='These three current research-note files are expected to change during I46 documentation. Their original pre-change snapshots remain verified.',
        scope='Read-only file and hash checks; no tests, raw reductions, control or simulation executed.')
    (STAGE/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],protected_origins=len(protected),prior_archives=len(archives))))


if __name__ == '__main__':
    main()
