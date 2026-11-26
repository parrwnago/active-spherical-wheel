"""Consolidate completed I44 independent QA without changing earlier stages."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
FAMILY = next(p for p in HERE.parents if p.name == 'terrain_memory_20261126')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')


def metadata(path, relative_to):
    return dict(path=str(path.relative_to(relative_to)), bytes=path.stat().st_size, sha256=sha(path))


def main():
    if (BASE / 'report.json').exists():
        raise FileExistsError('Preserve final report')
    source = HERE / 'source'
    source.mkdir(exist_ok=False)
    shutil.copyfile(__file__, source / 'seal_final.py')
    save(HERE / 'declaration_before_consolidation.json', dict(
        declared_utc=datetime.now(timezone.utc).isoformat(),
        scope='Report consolidation and post-process seal only; no new numeric campaign or changed primary artifacts.',
        source=metadata(source / 'seal_final.py', HERE),
        inputs=[metadata(BASE / name, BASE) for name in (
            'corrected_stage02/report.json', 'supplement_stage03/report.json',
            'corrected_stage02/artifact_manifest.json', 'supplement_stage03/artifact_manifest.json')]))
    primary_path = BASE / 'corrected_stage02/report.json'
    pair_path = BASE / 'supplement_stage03/report.json'
    primary = json.loads(primary_path.read_text())
    pairs = json.loads(pair_path.read_text())
    assert primary['passed'] and pairs['passed']
    assert sha(FAMILY / 'campaign/analyze_stage01/report.json') == primary['analysis_report_sha256']
    assert sha(FAMILY / 'analysis/matched_pairs_stage01/report.json') == pairs['supplement_report_sha256']
    legacy_seal_issues = []
    for stage in ('corrected_stage02', 'supplement_stage03'):
        folder = BASE / stage
        mismatch = []
        for item in json.loads((folder / 'artifact_manifest.json').read_text())['files']:
            if sha(folder / item['path']) != item['sha256']:
                mismatch.append(item['path'])
        assert mismatch == ['execution.log']
        legacy_seal_issues.append(dict(stage=stage, stale_digest_paths=mismatch,
                                      cause='Manifest was written before final report print completed in redirected log.',
                                      correction='Earlier manifest retained; final_artifact_manifest.json seals the completed logs and all preserved stages.'))
    uniform = pairs['independent_strata']['uniform']
    report = dict(
        iteration='I-44', passed=True,
        scope='Authoritative aggregate of independent primary saved-output QA and post-registered paired-supplement QA; no robot dynamics or rendering.',
        primary_saved_data_qa=metadata(primary_path, BASE),
        matched_supplement_qa=metadata(pair_path, BASE),
        primary_analysis_report_sha256=primary['analysis_report_sha256'],
        matched_supplement_report_sha256=pairs['supplement_report_sha256'],
        primary_checks=dict(rows=primary['raw_query_rows'], unique_shared_query_slots=primary['unique_shared_query_slots'],
                            anchor_prefixes=primary['anchor_prefixes_checked'], first_prefix_snapshot_exact=primary['first_prefix_snapshot_exact'],
                            capture_transform_max_gap=primary['capture_transform_max_gap'], utility_gates=primary['utility_gates']),
        matched_uniform_summary=uniform,
        matched_dense_summary=pairs['independent_strata']['dense_bump'],
        preserved_failures=[dict(path='failure_note.json', kind='QA parser assumption: ObserverPlane does not export mapper-only at_time_s; corrected stage02 accepts absence only for that arm.'),
                            dict(path=str((FAMILY / 'memory_development/stage02_pole_failure').relative_to(FAMILY)),
                                 kind='Implementation sloped-pole roundoff failure before main map analysis; minimal normalization/pole fix passed 17 owned tests in stage03.')],
        preserved_seal_issues=legacy_seal_issues,
        interpretation=pairs['interpretation'] + primary['limitations'],
        no_simulations_or_primary_parameter_changes=True,
        authoritative_seal='final_artifact_manifest.json')
    save(BASE / 'report.json', report)
    own = [p for p in HERE.rglob('*') if p.is_file()]
    save(HERE / 'artifact_manifest.json', dict(files=[metadata(p, HERE) for p in sorted(own)]))
    all_paths = [p for p in BASE.rglob('*') if p.is_file() and p != BASE / 'final_artifact_manifest.json']
    save(BASE / 'final_artifact_manifest.json', dict(
        status='Final post-process seal; preserved earlier manifests can contain the explicitly documented stale log digest.',
        files=[metadata(p, BASE) for p in sorted(all_paths)]))


if __name__ == '__main__':
    main()
