"""Independent I45 saved-array QA; does not import the diagnostic implementation."""
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
FAMILY = next(p for p in HERE.parents if p.name == 'contact_preview_pose_20261126')
REPO = FAMILY.parents[1]
ARMS = ('observer_cv', 'memory_cv', 'observer_actual_pose', 'memory_actual_pose')
LABELS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')


def sha(path):
    d = hashlib.sha256()
    with path.open('rb') as stream:
        for piece in iter(lambda: stream.read(1024 * 1024), b''):
            d.update(piece)
    return d.hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def stats(values):
    v = np.array([x for x in values if x is not None and np.isfinite(x)], float)
    if not len(v):
        return dict(n=0, rmse=None, median_abs=None, p90_abs=None, p95_abs=None, max_abs=None)
    absolute = np.abs(v)
    return dict(n=len(v), rmse=float(np.linalg.norm(v) / np.sqrt(len(v))),
                median_abs=float(np.quantile(absolute, .5)), p90_abs=float(np.quantile(absolute, .9)),
                p95_abs=float(np.quantile(absolute, .95)), max_abs=float(max(absolute)))


def assert_match(actual, expected, path=''):
    if isinstance(actual, dict):
        for key, value in actual.items():
            assert key in expected, (path, key)
            assert_match(value, expected[key], path + '/' + key)
    elif actual is None:
        assert expected is None, path
    elif isinstance(actual, (int, bool, str)):
        assert actual == expected, (path, actual, expected)
    elif isinstance(actual, (list, tuple)):
        assert len(actual) == len(expected), path
        for i, (a, b) in enumerate(zip(actual, expected)):
            assert_match(a, b, path + '/' + str(i))
    else:
        assert abs(actual - expected) < 1e-11, (path, actual, expected)


def raw_truth(data, index, weight):
    loads = data['truth_patch_loads_N'][index]
    counts = data['truth_patch_contact_counts'][index]
    points = data['truth_patch_points_m'][index]
    loaded = np.flatnonzero(loads >= max(.05, .05 * weight))
    patch = int(loaded[0]) if len(loaded) == 1 else None
    other = None if patch is None else float(sum(loads[k] for k in range(4) if k != patch))
    primary = bool(patch is not None and sum(counts) == 1 and counts[patch] == 1
                   and np.isfinite(points[patch]).all() and other <= .05 + 1e-12)
    equivalent = data['truth_equivalent_point_m'][index]
    legacy = bool(len(loaded) and np.isfinite(equivalent).all())
    if primary:
        category = 'single_resolved_patch_mean'
    elif len(loaded) > 1:
        category = 'multiple_loaded_patches'
    elif patch is not None and sum(counts) > 1:
        category = 'single_patch_multiple_resolved'
    elif patch is not None and other > .05 + 1e-12:
        category = 'weak_or_unresolved'
    elif patch is not None:
        category = 'invalid_truth' if not np.isfinite(points[patch]).all() else 'weak_or_unresolved'
    else:
        category = 'weak_or_unresolved' if np.any(loads > 0) else 'unloaded'
    return dict(primary_eligible=primary, primary_patch=LABELS[patch] if primary else None,
                primary_target_m=points[patch].tolist() if primary else None,
                legacy_equivalent_eligible=legacy, legacy_target_m=equivalent.tolist() if legacy else None,
                loaded_patches=[LABELS[k] for k in loaded], category=category)


def metric_table(groups, primary=True):
    errors_key = 'primary_error' if primary else 'legacy_error'
    result = {}
    for arm in ARMS:
        selected = [group[arm] for group in groups]
        errors = stats([r[errors_key] for r in selected])
        result[arm] = dict(distance_error_m=errors, rmse_m=errors['rmse'], p95_m=errors['p95_abs'],
                           identity_recall=sum(r['identity'] for r in selected) / len(selected) if selected and primary else None,
                           ambiguity_fraction=sum(r['ambiguous'] for r in selected) / len(selected) if selected else None,
                           candidate_cardinality=stats([r['patch_count'] for r in selected]))
    return result


def main():
    if (HERE / 'report.json').exists():
        raise FileExistsError('Preserve completed QA stage')
    stage = FAMILY / 'campaign/stage01'
    sources = [Path(__file__), FAMILY / 'protocol.json', REPO / 'scripts/diagnose-contact-preview-pose.py',
               REPO / 'src/spherical_wheel/contact_pose_diagnostic.py', REPO / 'src/spherical_wheel/terrain_memory.py']
    source_dir = HERE / 'source'
    source_dir.mkdir(exist_ok=False)
    records = []
    for source in sources:
        target = source_dir / source.name
        shutil.copyfile(source, target)
        records.append(dict(origin=str(source), snapshot=str(target.relative_to(HERE)), sha256=sha(source)))
    save(HERE / 'declaration_before_qa.json', dict(declared_utc=datetime.now(timezone.utc).isoformat(),
        scope='Independent saved-array arithmetic/pose/prefix QA only; no scene, renderer, controller, map regeneration or dynamics.',
        sources=records, campaign_report_sha256=sha(stage / 'report.json'),
        campaign_predictions_sha256=sha(stage / 'predictions.json')))
    protocol = json.loads((FAMILY / 'protocol.json').read_text())
    for key, path in protocol['inputs'].items():
        assert sha(REPO / path) == protocol['input_sha256'][key], key
    for path, metadata in json.loads((stage / 'artifact_manifest.json').read_text()).items():
        assert sha(stage / path) == metadata['sha256'], path
    for item in json.loads((stage / 'source_manifest.json').read_text())['sources']:
        assert sha(stage / item['snapshot']) == item['sha256'], item['snapshot']
        assert sha(REPO / item['path']) == item['sha256'], item['path']
    report = json.loads((stage / 'report.json').read_text())
    assert report['integrity_passed']
    rows = json.loads((stage / 'predictions.json').read_text())
    anchors = json.loads((stage / 'anchor_records.json').read_text())
    old_anchors = json.loads((REPO / protocol['inputs']['I44_anchor_records']).read_text())
    old_rows = json.loads((REPO / protocol['inputs']['I44_predictions']).read_text())
    old = {(r['anchor_index'], r['horizon_s'], r['arm']): r for r in old_rows}
    with np.load(REPO / protocol['inputs']['precision']) as z:
        data = {k: z[k] for k in z.files}
    with np.load(REPO / protocol['inputs']['depth']) as z:
        deliveries, captures = z['delivered_at_s'], z['capture_time_s']
    weight = json.loads((REPO / protocol['inputs']['native_provenance']).read_text())['weight_N']
    assert len(rows) == 1428 and len(anchors) == 119
    anchor_by_index = {a['anchor_index']: a for a in anchors}
    for anchor, previous in zip(anchors, old_anchors):
        now = anchor['anchor_time_s']
        available = np.flatnonzero(deliveries <= now + 1e-9)
        assert anchor['ingested_batches'] == len(available)
        assert anchor['latest_batch_index'] == int(available[-1])
        assert anchor['maximum_ingested_delivery_s'] == float(deliveries[available[-1]])
        assert anchor['maximum_ingested_capture_s'] == float(captures[available[-1]])
        assert anchor['map_knowledge_time_s'] <= now + 1e-9
        assert anchor['map_locked_for_four_conditions'] and anchor['prefix_matches_I44']
        for key in ('anchor_index', 'anchor_time_s', 'strata', 'ingested_batches', 'memory_cell_count'):
            assert anchor[key] == previous[key]
    grouped = {}
    max_rotation_gap = max_radius_gap = 0.
    for row in rows:
        ai, h, arm = row['anchor_index'], row['horizon_s'], row['arm']
        assert arm in ARMS
        now = float(data['time_s'][ai])
        fi = int(np.argmin(np.abs(data['time_s'] - (now + h))))
        assert fi == row['future_index'] and abs(data['time_s'][fi] - now - h) <= 1e-8
        truth = raw_truth(data, fi, weight)
        assert_match(truth, row['truth'], arm + '/truth')
        preview, score = row['contact_preview'], row['contact_score']
        predicted = np.asarray(preview['predicted_qpos'])
        if arm.endswith('actual_pose'):
            expected = data['exact_qpos'][fi]
            np.testing.assert_allclose(predicted[:3], expected[:3], atol=0, rtol=0)
            np.testing.assert_allclose(predicted[7:], expected[7:], atol=0, rtol=0)
            target_rotation = Rotation.from_quat(expected[[4, 5, 6, 3]])
            assert row['uses_future_configuration'] and not preview['deployable_input_condition']
        else:
            q, v = data['exact_qpos'][ai], data['exact_qvel'][ai]
            np.testing.assert_allclose(predicted[:3], q[:3] + h * v[:3], atol=1e-14, rtol=0)
            np.testing.assert_allclose(predicted[7:], q[7:] + h * v[6:], atol=1e-14, rtol=0)
            target_rotation = Rotation.from_quat(q[[4, 5, 6, 3]]) * Rotation.from_rotvec(v[3:6] * h)
            old_arm = 'observer_plane' if arm == 'observer_cv' else 'memory_four'
            prior = old[(ai, h, old_arm)]
            assert_match(predicted.tolist(), prior['shared_cv_qpos'])
            assert_match(row['surface_query'], prior['surface_query'])
            raw_preview = {k: v for k, v in preview.items() if k not in ('pose_source', 'uses_future_configuration', 'deployable_input_condition')}
            assert_match(raw_preview, prior['contact_preview'])
            assert not row['uses_future_configuration']
        got_rotation = Rotation.from_quat(predicted[[4, 5, 6, 3]])
        max_rotation_gap = max(max_rotation_gap, float(np.max(np.abs(got_rotation.as_matrix() - target_rotation.as_matrix()))))
        assert preview['time_s'] == now + h
        query = row['surface_query']
        def audit_query(query):
            if arm.startswith('memory'):
                assert query['at_time_s'] == now
                if query['last_capture_s'] is not None:
                    last = query['last_capture_s']
                    assert query['first_capture_s'] <= last <= anchor_by_index[ai]['maximum_ingested_capture_s'] + 1e-9
                    assert abs(query['age_s'] - (now - last)) < 1e-12
                if query.get('support_newest_last_capture_s') is not None:
                    assert query['support_oldest_last_capture_s'] <= query['support_newest_last_capture_s']
                    assert query['support_newest_last_capture_s'] <= anchor_by_index[ai]['maximum_ingested_capture_s'] + 1e-9
            elif 'at_time_s' in query:
                assert query['at_time_s'] == now
        audit_query(query)
        candidates = preview['candidates']
        patches = sorted({c['patch'] for c in candidates})
        complete = bool(preview['contact_prediction_supported'])
        def nearest(target):
            return None if target is None or not candidates else min(float(np.linalg.norm(np.asarray(c['point_m']) - target)) for c in candidates)
        primary_error = nearest(truth['primary_target_m']) if truth['primary_eligible'] else None
        legacy_error = nearest(truth['legacy_target_m']) if truth['legacy_equivalent_eligible'] else None
        identity = bool(truth['primary_eligible'] and complete and truth['primary_patch'] in patches)
        assert_match(dict(primary_nearest_candidate_error_m=primary_error, legacy_nearest_candidate_error_m=legacy_error,
                          primary_patch_in_candidates=identity, geometric_candidate_count=len(candidates),
                          unique_patch_count=len(patches), contact_preview_complete=complete), score)
        for candidate in candidates:
            max_radius_gap = max(max_radius_gap, abs(float(np.linalg.norm(np.asarray(candidate['point_m']) - predicted[:3])) - .12))
            cq = candidate['contact_surface_query']
            audit_query(cq)
        group = grouped.setdefault((ai, h), {})
        assert arm not in group
        group[arm] = dict(raw=row, truth=truth, complete=complete, nonempty=bool(candidates),
                          primary_error=primary_error, legacy_error=legacy_error,
                          identity=identity, ambiguous=bool(preview['ambiguous']), patch_count=len(patches))
    assert max_rotation_gap < 1e-12 and max_radius_gap < 1e-12
    assert len(grouped) == 357 and all(set(g) == set(ARMS) for g in grouped.values())
    primary_tables = {}
    for stratum in ('all_unique', 'uniform', 'dense_bump'):
        groups = {k: g for k, g in grouped.items() if stratum == 'all_unique' or stratum in g[ARMS[0]]['raw']['strata']}
        primary_tables[stratum] = {}
        for horizon in (None, .05, .1, .2):
            name = 'all_horizons' if horizon is None else f'{horizon:.2f}s'
            sub = {k: g for k, g in groups.items() if horizon is None or k[1] == horizon}
            for arm in ARMS:
                selected = [g[arm] for g in sub.values()]
                total = len(selected)
                primary = [r for r in selected if r['truth']['primary_eligible']]
                legacy = [r for r in selected if r['truth']['legacy_equivalent_eligible']]
                counts = dict(query_slots=total,
                    surface_supported=sum(r['raw']['contact_preview']['supported_surface'] for r in selected),
                    complete_preview=sum(r['complete'] for r in selected),
                    complete_nonempty=sum(r['complete'] and r['nonempty'] for r in selected),
                    known_no_near_contact=sum(r['raw']['contact_preview'].get('known_no_near_contact', False) for r in selected),
                    unknown_surface=sum(not r['raw']['contact_preview']['supported_surface'] for r in selected),
                    partial_contact_region=sum(r['raw']['contact_preview']['supported_surface'] and not r['complete'] for r in selected),
                    primary_all_eligible=dict(n=len(primary), identity_hits=sum(r['identity'] for r in primary),
                        identity_recall=sum(r['identity'] for r in primary) / len(primary) if primary else None),
                    truth_categories=dict(Counter(r['truth']['category'] for r in selected)),
                    legacy_all_eligible=dict(n=len(legacy), nearest_candidate_error_m=stats([r['legacy_error'] for r in legacy])))
                counts['complete_nonempty_coverage'] = counts['complete_nonempty'] / total if total else None
                assert_match(counts, report['summary'][stratum][arm][name], 'summary/' + stratum + '/' + arm + '/' + name)
            for target, eligibility in (('primary', 'primary_eligible'), ('legacy', 'legacy_equivalent_eligible')):
                common, common_keys, excluded = [], [], Counter()
                for key, group in sub.items():
                    if not group[ARMS[0]]['truth'][eligibility]:
                        excluded['truth_ineligible'] += 1
                    elif not all(r['complete'] for r in group.values()):
                        excluded['incomplete_preview'] += 1
                    elif not all(r['nonempty'] for r in group.values()):
                        excluded['empty_candidates'] += 1
                    else:
                        common.append(group)
                        common_keys.append(list(key))
                table = dict(query_slots=len(sub), matched_slots=len(common), exclusions=dict(excluded),
                    matched_keys=common_keys,
                    fixed_truth_eligible_slots=sum(g[ARMS[0]]['truth'][eligibility] for g in sub.values()),
                    arms=metric_table(common, primary=target == 'primary'))
                assert_match(table, report[target + '_common_fourway'][stratum][name], target + '/' + stratum + '/' + name)
                if target == 'primary':
                    primary_tables[stratum][name] = table
    uniform = primary_tables['uniform']['all_horizons']
    limits = protocol['registered_decision_thresholds']
    def quality(observer, memory):
        valid = all(x[k] is not None for x in (observer, memory) for k in ('rmse_m', 'p95_m', 'identity_recall', 'ambiguity_fraction'))
        absolute = observer['rmse_m'] - memory['rmse_m'] if valid else None
        relative = absolute / observer['rmse_m'] if valid and observer['rmse_m'] > 0 else None
        checks = dict(RMS_relative_improvement=bool(relative is not None and relative >= limits['RMS_relative_improvement_min'] - 1e-14),
            RMS_absolute_improvement=bool(valid and absolute >= limits['RMS_absolute_improvement_m_min'] - 1e-14),
            p95_error=bool(valid and memory['p95_m'] <= observer['p95_m'] + limits['p95_error_increase_m_max'] + 1e-14),
            identity_recall=bool(valid and memory['identity_recall'] >= observer['identity_recall'] - limits['matched_identity_recall_loss_max'] - 1e-14),
            ambiguity=bool(valid and memory['ambiguity_fraction'] <= observer['ambiguity_fraction'] + limits['matched_ambiguity_fraction_increase_max'] + 1e-14))
        return dict(checks=checks, passed=all(checks.values()), absolute_RMS_improvement_m=absolute, relative_RMS_improvement=relative)
    arms = uniform['arms']
    cv = quality(arms['observer_cv'], arms['memory_cv'])
    oracle = quality(arms['observer_actual_pose'], arms['memory_actual_pose'])
    eligible = uniform['fixed_truth_eligible_slots']
    fraction = uniform['matched_slots'] / eligible if eligible else None
    cv_coverage = report['summary']['uniform']['memory_cv']['all_horizons']['complete_nonempty_coverage']
    oracle_coverage = report['summary']['uniform']['memory_actual_pose']['all_horizons']['complete_nonempty_coverage']
    support = dict(common_count=uniform['matched_slots'] >= limits['primary_common_min_n'],
        common_fraction=fraction is not None and fraction >= limits['primary_common_fraction_min'],
        memory_oracle_coverage=oracle_coverage >= limits['uniform_memory_complete_nonempty_coverage_min'] - 1e-14,
        memory_coverage_loss=oracle_coverage >= cv_coverage - limits['oracle_memory_coverage_loss_vs_CV_memory_max'] - 1e-14)
    if not support['common_count'] or not support['common_fraction']:
        branch = 'inconclusive_insufficient_common_primary'
    elif arms['memory_actual_pose']['rmse_m'] > arms['observer_actual_pose']['rmse_m']:
        branch = 'park_current_map_to_contact_integration_oracle_memory_worse'
    elif oracle['passed'] and all(support.values()):
        branch = 'audit_metric_aggregation_and_stratification' if cv['passed'] else 'investigate_motion_prediction_keep_LQR_unchanged'
    else:
        branch = 'insufficient_useful_advantage_no_integration'
    decision = dict(common_primary_n=uniform['matched_slots'], fixed_primary_eligible_n=eligible,
                    common_primary_fraction=fraction, support_checks=support, cv_quality=cv, oracle_quality=oracle, decision_branch=branch)
    assert_match(decision, report['registered_decision'], 'registered_decision')
    for key, path in protocol['inputs'].items():
        assert sha(REPO / path) == protocol['input_sha256'][key], key
    save(HERE / 'report.json', dict(iteration='I-45', passed=True,
        scope='Independent saved-array QA; no renderer, scene, controller, map-regeneration or dynamics calls.',
        campaign_report_sha256=sha(stage / 'report.json'), raw_rows=len(rows), shared_query_slots=len(grouped),
        prefixes_verified=len(anchors), CV_parity_rows=714, actual_pose_rows=714,
        max_orientation_matrix_gap=max_rotation_gap, max_candidate_radius_gap_m=max_radius_gap,
        independent_primary_tables=primary_tables, independently_verified_decision=decision,
        limitations=['Targets are resolved-contact patch means, not guaranteed individual physical contacts.',
                    'Saved-state/contact1ms convention and ideal-rim geometry remain.',
                    'One historical trajectory; nested anchors/horizons are not independent trials.',
                    'Future configuration is hindsight; no live improvement or integration authorization.']))
    # No stdout follows this seal, avoiding a late redirected-log mutation.
    files = [p for p in HERE.rglob('*') if p.is_file() and p.name != 'artifact_manifest.json']
    save(HERE / 'artifact_manifest.json', dict(files=[dict(path=str(p.relative_to(HERE)), bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(files)]))


if __name__ == '__main__':
    main()
