"""Consolidate I45 independent review and seal only after raw QA has ended."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
REVIEW = HERE.parent
FAMILY = REVIEW.parent


def sha(path):
    d = hashlib.sha256()
    with path.open('rb') as stream:
        for piece in iter(lambda: stream.read(1024 * 1024), b''):
            d.update(piece)
    return d.hexdigest()


def metadata(path, relative):
    return dict(path=str(path.relative_to(relative)), sha256=sha(path), bytes=path.stat().st_size)


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def main():
    if (REVIEW / 'final_report.json').exists():
        raise FileExistsError('Preserve sealed review')
    source = HERE / 'source'
    source.mkdir(exist_ok=False)
    shutil.copyfile(__file__, source / 'finalize.py')
    raw_path = REVIEW / 'raw_qa_stage01/report.json'
    campaign_path = FAMILY / 'campaign/stage01/report.json'
    save(HERE / 'declaration_before_consolidation.json', dict(
        declared_utc=datetime.now(timezone.utc).isoformat(),
        scope='Aggregate and post-process seal only; no new scoring, rendering, map, controller or physics.',
        source=metadata(source / 'finalize.py', HERE),
        raw_qa_report=metadata(raw_path, REVIEW), campaign_report=metadata(campaign_path, FAMILY)))
    raw = json.loads(raw_path.read_text())
    campaign = json.loads(campaign_path.read_text())
    assert raw['passed'] and campaign['integrity_passed']
    assert sha(campaign_path) == raw['campaign_report_sha256']
    raw_stage = raw_path.parent
    seal = json.loads((raw_stage / 'artifact_manifest.json').read_text())
    for item in seal['files']:
        actual = metadata(raw_stage / item['path'], raw_stage)
        assert actual == item
    uniform = raw['independent_primary_tables']['uniform']['all_horizons']
    coverage = {stratum: {arm: {key: campaign['summary'][stratum][arm]['all_horizons'][key]
                              for key in ('query_slots', 'surface_supported', 'complete_preview', 'complete_nonempty', 'complete_nonempty_coverage', 'unknown_surface', 'partial_contact_region')}
                         for arm in campaign['arms']}
                for stratum in ('uniform', 'dense_bump')}
    report = dict(iteration='I-45', passed=True,
        scope='Authoritative independent protocol/source and saved-output QA aggregate; scientific result remains diagnostic and mixed.',
        raw_qa=metadata(raw_path, REVIEW), campaign_report_sha256=sha(campaign_path),
        protocol_sha256=sha(FAMILY / 'protocol.json'),
        verified=dict(raw_rows=raw['raw_rows'], shared_query_slots=raw['shared_query_slots'], prefixes=raw['prefixes_verified'],
                      CV_parity_rows=raw['CV_parity_rows'], actual_pose_rows=raw['actual_pose_rows'],
                      all_registered_input_hashes=True, all_truth_classes_and_targets=True,
                      all_strata_horizon_matches_and_gates=True, map_clocks_ages_delivery_prefix=True,
                      max_orientation_matrix_gap=raw['max_orientation_matrix_gap'],
                      max_candidate_radius_gap_m=raw['max_candidate_radius_gap_m']),
        primary_uniform_fourway=uniform, coverage=coverage,
        registered_decision=raw['independently_verified_decision'],
        source_audit=[
            'All four conditions generated before truth classification/scoring.',
            'Actual-pose generation receives future configuration only, zero motion horizon and velocity, and anchor-frozen observer/map providers.',
            'No future observations, contacts, forces, normals, scene or controller enter generation.',
            'Wrong predicted identities remain in nearest-ANY geometric-candidate scoring; identity and ambiguity use the same four-way denominator.',
            'Unknown/empty/incomplete previews remain in all-query coverage and all-eligible identity reporting.',
            'Geometric-candidate count and unique-patch count are separately exposed.'],
        interpretation=[
            'The registered branch parks current map-to-contact integration: oracle memory RMS is slightly worse and fails both material RMS improvement gates.',
            'This is mixed evidence: oracle memory improves p95 and maximum error despite worse RMS/median and slightly lower matched identity recall.',
            'Removing CV motion error narrows the memory disadvantage substantially; it does not establish useful overall contact improvement.',
            'Conditional resolved-contact patch means are not exact individual collision points; saved1ms timing and ideal-rim geometry remain.',
            'One historical execution, correlated anchors/horizons, hindsight configuration and unchanged native driving; no live improvement demonstrated.'],
        new_physics_runs=0, new_depth_renders=0, no_runtime_controller_change=True,
        failures_in_this_independent_QA_stage=0,
        authoritative_postprocess_seal='final_artifact_manifest.json')
    save(REVIEW / 'final_report.json', report)
    files = [p for p in HERE.rglob('*') if p.is_file()]
    save(HERE / 'artifact_manifest.json', dict(files=[metadata(p, HERE) for p in sorted(files)]))
    # Logs are closed: raw QA has returned and this finalizer writes no stdout.
    paths = [p for p in REVIEW.rglob('*') if p.is_file() and p != REVIEW / 'final_artifact_manifest.json']
    save(REVIEW / 'final_artifact_manifest.json', dict(files=[metadata(p, REVIEW) for p in sorted(paths)]))


if __name__ == '__main__':
    main()
