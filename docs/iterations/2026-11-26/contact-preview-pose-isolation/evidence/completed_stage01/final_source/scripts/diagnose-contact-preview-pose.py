"""I-45: frozen terrain provider by CV / hindsight saved-configuration diagnostic.

Only saved depth and saved native states are consumed.  No scene is built, no
controller is invoked, and no dynamics or camera rendering is executed.  The
I-44 preview algorithm and settings are reused without modification.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
from time import perf_counter
import traceback
import types
import zipfile

import numpy as np

from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.contact_pose_diagnostic import (
    ARMS, classify_saved_contact_truth, common_fourway_rows, compare_values,
    diagnostic_decision, endpoint_indices, generate_pose_conditions,
    score_contact_preview,
)
from spherical_wheel.terrain_depth import DepthFrame
from spherical_wheel.terrain_memory import MemorySettings, TerrainMemory, WheelGeometry


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def safe(value):
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return safe(value.tolist())
    if isinstance(value, np.generic):
        return safe(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def save_json(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def load_I44_helpers():
    """Compile the preserved runner text without writing into its sealed tree."""
    path = REPO_ROOT/'scripts/run-terrain-memory.py'
    module = types.ModuleType('_I44_unchanged_helpers')
    module.__file__ = str(path)
    exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
    return module


def dependency_paths(output):
    paths = [Path(__file__).resolve(), REPO_ROOT/'scripts/run-terrain-memory.py',
             REPO_ROOT/'config/model.yaml', REPO_ROOT/'pyproject.toml', output/'protocol.json']
    paths += sorted((REPO_ROOT/'src/spherical_wheel').rglob('*.py'))
    paths += sorted((REPO_ROOT/'tests').glob('test_contact_pose*.py'))
    return sorted(set(paths))


def snapshot_sources(stage, output):
    rows = []
    for source in dependency_paths(output):
        rel = source.relative_to(REPO_ROOT)
        target = stage/'source'/rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        rows.append(dict(path=str(rel), sha256=sha(source), snapshot=str(target.relative_to(stage))))
    result = dict(captured_utc=datetime.now(timezone.utc).isoformat(), sources=rows)
    save_json(stage/'source_manifest.json', result)
    return result


def verify_inputs(protocol):
    actual = {name: sha(REPO_ROOT/path) for name, path in protocol['inputs'].items()}
    changed = [name for name, value in actual.items() if value != protocol['input_sha256'][name]]
    if changed:
        raise RuntimeError('Registered inputs changed: '+', '.join(changed))
    return actual


def npz_array_prefix(path, key, rows):
    """Read just the first rows of one compressed npy member, not the full map."""
    with zipfile.ZipFile(path) as archive, archive.open(key+'.npy') as stream:
        version = np.lib.format.read_magic(stream)
        reader = (np.lib.format.read_array_header_1_0 if version == (1, 0)
                  else np.lib.format.read_array_header_2_0)
        shape, fortran, dtype = reader(stream)
        if fortran or dtype.hasobject or not shape or rows > shape[0]:
            raise ValueError('Expected numeric row-major saved map prefix')
        count = rows*int(np.prod(shape[1:], dtype=int))
        payload = stream.read(count*dtype.itemsize)
        if len(payload) != count*dtype.itemsize:
            raise ValueError('Incomplete saved map prefix')
        return np.frombuffer(payload, dtype=dtype).copy().reshape((rows,)+shape[1:])


def verify_first_map_prefix(memory, anchor, snapshot_path):
    with np.load(snapshot_path) as z:
        offsets = z['offsets'].copy()
        first_time = float(z['anchor_time_s'][0])
    cells = memory.observed_cells(at_time_s=anchor['anchor_time_s'])
    count = int(offsets[1])
    checks = dict(anchor_time=abs(first_time-anchor['anchor_time_s']) <= 1e-10,
                  cell_count=len(cells['centers_m']) == count)
    for key in ('centers_m', 'count', 'first_capture_s', 'last_capture_s', 'age_s', 'height_spread_m'):
        original = npz_array_prefix(snapshot_path, key, count)
        checks[key] = bool(np.array_equal(cells[key], original, equal_nan=True))
    return dict(checks=checks, passed=all(checks.values()), prefix_cells=count,
                loading='Only first-prefix rows streamed; no full-map copy or rehydration.')


def memory_lock_token(memory):
    """Epoch and container identity; source-verified query() is read-only."""
    return dict(available_at_s=float(memory._available_at_s),
                cells=len(memory._cells), frames=len(memory._frames),
                cells_identity=id(memory._cells), frames_identity=id(memory._frames))


def rows_summary(rows, statistics):
    result = {}
    for stratum in ('all_unique', 'uniform', 'dense_bump'):
        selected = [r for r in rows if stratum == 'all_unique' or stratum in r['strata']]
        result[stratum] = {}
        for arm in ARMS:
            group = [r for r in selected if r['arm'] == arm]
            tables = {}
            for horizon in (None, .05, .1, .2):
                sub = [r for r in group if horizon is None or abs(r['horizon_s']-horizon) < 1e-10]
                primary = [r for r in sub if r['truth']['primary_eligible']]
                legacy = [r for r in sub if r['truth']['legacy_equivalent_eligible']]
                complete_nonempty = sum(r['contact_score']['contact_preview_complete'] and r['contact_score']['nonempty_candidates'] for r in sub)
                tables['all_horizons' if horizon is None else f'{horizon:.2f}s'] = dict(
                    query_slots=len(sub), surface_supported=sum(r['contact_preview']['supported_surface'] for r in sub),
                    complete_preview=sum(r['contact_score']['contact_preview_complete'] for r in sub),
                    complete_nonempty=complete_nonempty,
                    complete_nonempty_coverage=complete_nonempty/len(sub) if sub else None,
                    known_no_near_contact=sum(r['contact_preview'].get('known_no_near_contact', False) for r in sub),
                    unknown_surface=sum(not r['contact_preview']['supported_surface'] for r in sub),
                    partial_contact_region=sum(r['contact_preview']['supported_surface'] and not r['contact_score']['contact_preview_complete'] for r in sub),
                    surface_reasons=dict(Counter(r['surface_query'].get('reason') for r in sub)),
                    contact_reasons=dict(Counter(r['contact_preview'].get('reason') for r in sub)),
                    truth_categories=dict(Counter(r['truth']['category'] for r in sub)),
                    ambiguous=sum(r['contact_score']['candidate_ambiguous'] for r in sub),
                    unique_patch_count=statistics([r['contact_score']['unique_patch_count'] for r in sub]),
                    geometric_candidate_count=statistics([r['contact_score']['geometric_candidate_count'] for r in sub]),
                    primary_all_eligible=dict(n=len(primary),
                        identity_recall=sum(r['contact_score']['primary_patch_in_candidates'] for r in primary)/len(primary) if primary else None,
                        identity_hits=sum(r['contact_score']['primary_patch_in_candidates'] for r in primary),
                        nearest_error_any_available_candidates_including_partial_m=statistics([r['contact_score']['primary_nearest_candidate_error_m'] for r in primary]),
                        nearest_error_complete_nonempty_only_m=statistics([r['contact_score']['primary_nearest_candidate_error_m'] for r in primary if r['contact_score']['contact_preview_complete'] and r['contact_score']['nonempty_candidates']])),
                    legacy_all_eligible=dict(n=len(legacy),
                        nearest_candidate_error_m=statistics([r['contact_score']['legacy_nearest_candidate_error_m'] for r in legacy])))
            result[stratum][arm] = tables
    return result


def matched_tables(rows, statistics, target='primary'):
    result = {}
    error_key = 'primary_nearest_candidate_error_m' if target == 'primary' else 'legacy_nearest_candidate_error_m'
    eligible_key = 'primary_eligible' if target == 'primary' else 'legacy_equivalent_eligible'
    for stratum in ('all_unique', 'uniform', 'dense_bump'):
        selected = [r for r in rows if stratum == 'all_unique' or stratum in r['strata']]
        result[stratum] = {}
        for horizon in (None, .05, .1, .2):
            sub = [r for r in selected if horizon is None or abs(r['horizon_s']-horizon) < 1e-10]
            common = common_fourway_rows(sub, target=target)
            groups = common.pop('groups')
            common['fixed_truth_eligible_slots'] = sum(r['truth'][eligible_key] for r in sub if r['arm'] == ARMS[0])
            common['matched_keys'] = [list(key) for key, _ in groups]
            common['arms'] = {}
            for arm in ARMS:
                armrows = [group[arm] for _, group in groups]
                errors = statistics([r['contact_score'][error_key] for r in armrows])
                common['arms'][arm] = dict(distance_error_m=errors,
                    rmse_m=errors['rmse'], p95_m=errors['p95_abs'],
                    identity_recall=sum(r['contact_score']['primary_patch_in_candidates'] for r in armrows)/len(armrows) if armrows and target == 'primary' else None,
                    ambiguity_fraction=sum(r['contact_score']['candidate_ambiguous'] for r in armrows)/len(armrows) if armrows else None,
                    candidate_cardinality=statistics([r['contact_score']['candidate_cardinality'] for r in armrows]))
            result[stratum]['all_horizons' if horizon is None else f'{horizon:.2f}s'] = common
    return result


def run_diagnostic(output, stage, protocol):
    started = perf_counter()
    inputs_before = verify_inputs(protocol)
    helper = load_I44_helpers()
    old_protocol = json.loads((REPO_ROOT/protocol['inputs']['I44_protocol']).read_text())
    old_rows = json.loads((REPO_ROOT/protocol['inputs']['I44_predictions']).read_text())
    old_anchors = json.loads((REPO_ROOT/protocol['inputs']['I44_anchor_records']).read_text())
    old_by_key = {(r['anchor_index'], r['horizon_s'], r['arm']): r for r in old_rows}
    provenance = json.loads((REPO_ROOT/protocol['inputs']['native_provenance']).read_text())
    with np.load(REPO_ROOT/protocol['inputs']['precision']) as z:
        exact = {k: z[k].copy() for k in z.files}
    with np.load(REPO_ROOT/protocol['inputs']['depth']) as z:
        depth = {k: z[k].copy() for k in z.files}
    times, qpos = exact['time_s'], exact['exact_qpos']
    anchors = helper.make_anchors(times)
    if len(anchors) != protocol['anchors']['unique']:
        raise AssertionError('Anchor count differs from protocol')
    horizons = tuple(protocol['anchors']['horizons_s'])
    settings = MemorySettings(**{k: v for k, v in old_protocol['map'].items()
                                 if k not in ('unknown_rule', 'static_world_assumption')})
    geometry = WheelGeometry.from_config(load_config())
    memory = TerrainMemory(settings)
    pointer, rows, anchor_records = 0, [], []
    parity_failures, prefix_result = [], None
    map_wall, query_wall = [], []
    cv_meta = {'pose_source', 'uses_future_configuration', 'deployable_input_condition'}
    for number, anchor in enumerate(anchors):
        index, now = anchor['anchor_index'], anchor['anchor_time_s']
        map_start = perf_counter()
        while pointer < len(depth['capture_time_s']) and depth['delivered_at_s'][pointer] <= now+1e-9:
            frames = tuple(DepthFrame(camera_name=str(depth['camera_names'][k]),
                depth_m=depth['depth_m'][pointer, k], valid_mask=depth['valid_mask'][pointer, k],
                intrinsics=depth['intrinsics'][k], body_from_camera=depth['body_from_camera'][k],
                world_from_camera=depth['world_from_camera'][pointer, k],
                capture_time_s=float(depth['capture_time_s'][pointer]),
                delivered_at_s=float(depth['delivered_at_s'][pointer])) for k in range(4))
            memory.ingest(frames, available_at_s=now)
            pointer += 1
        map_wall.append(perf_counter()-map_start)
        if number == 0:
            prefix_result = verify_first_map_prefix(memory, anchor, REPO_ROOT/protocol['inputs']['I44_map_snapshots'])
        original_anchor = old_anchors[number]
        prefix = dict(latest_batch_index=pointer-1,
            maximum_ingested_capture_s=None if not pointer else float(depth['capture_time_s'][pointer-1]),
            maximum_ingested_delivery_s=None if not pointer else float(depth['delivered_at_s'][pointer-1]),
            ingested_batches=pointer, memory_cell_count=len(memory._cells))
        anchor_failures = compare_values({**anchor, **prefix},
            {k: original_anchor[k] for k in (*anchor, *prefix)}, atol=1e-10)
        future_indices = endpoint_indices(times, now, horizons, tolerance_s=1e-8)
        providers = dict(observer_plane=helper.ObserverPlane(qpos[index, :3],
            exact['exact_estimated_normal'][index], geometry.radius_m, now), memory_four=memory)
        before = memory_lock_token(memory)
        query_start = perf_counter()
        # No truth classifier/scorer runs until ALL FOUR conditions are complete.
        predictions = generate_pose_conditions(providers, geometry, qpos[index],
            exact['exact_qvel'][index], now,
            {h: qpos[future_indices[h]] for h in horizons}, horizons_s=horizons,
            band_tolerance_m=old_protocol['contact_geometry']['band_tolerance_m'],
            max_gap_m=old_protocol['contact_geometry']['max_geometric_gap_m'])
        query_wall.append(perf_counter()-query_start)
        after = memory_lock_token(memory)
        if before != after:
            raise AssertionError('Memory changed while generating the four anchored conditions')
        anchor_records.append(dict(**anchor, **prefix,
            exact_qpos=qpos[index].copy(), estimated_normal=exact['exact_estimated_normal'][index].copy(),
            future_indices={str(h): idx for h, idx in future_indices.items()},
            future_time_gaps_s={str(h): float(times[idx]-now-h) for h, idx in future_indices.items()},
            map_knowledge_time_s=before['available_at_s'], map_locked_for_four_conditions=True,
            prefix_matches_I44=not anchor_failures, prefix_failures=anchor_failures))
        if anchor_failures:
            parity_failures.extend(f'anchor/{index}/{x}' for x in anchor_failures)
        for hi, horizon in enumerate(horizons):
            future_index = future_indices[horizon]
            truth = classify_saved_contact_truth(exact['truth_patch_loads_N'][future_index],
                exact['truth_patch_contact_counts'][future_index], exact['truth_patch_points_m'][future_index],
                exact['truth_equivalent_point_m'][future_index], provenance['weight_N'])
            for arm in ARMS:
                preview = predictions[arm][hi]
                contact_score = score_contact_preview(preview, truth)
                row = dict(**anchor, arm=arm, horizon_s=horizon, future_index=future_index,
                    future_time_s=float(times[future_index]), pose_source=preview['pose_source'],
                    uses_future_configuration=preview['uses_future_configuration'],
                    surface_query=preview['surface'], contact_preview=preview,
                    truth=truth, contact_score=contact_score)
                rows.append(row)
                if arm in ('observer_cv', 'memory_cv'):
                    old_arm = 'observer_plane' if arm == 'observer_cv' else 'memory_four'
                    old = old_by_key[(index, horizon, old_arm)]
                    # Compare raw original fields, including all status/reason/candidate fields.
                    actual = dict(shared_cv_qpos=preview['predicted_qpos'],
                        surface_query=row['surface_query'],
                        contact_preview={k: v for k, v in preview.items() if k not in cv_meta},
                        candidate_patches=contact_score['candidate_patches'],
                        contact_preview_complete=contact_score['contact_preview_complete'],
                        candidate_ambiguous=contact_score['candidate_ambiguous'],
                        future_point_eligible=truth['legacy_equivalent_eligible'],
                        nearest_candidate_error_m=contact_score['legacy_nearest_candidate_error_m'])
                    differences = compare_values(actual, {k: old[k] for k in actual}, atol=1e-10)
                    parity_failures.extend(f'{arm}/{index}/{horizon}/{x}' for x in differences)
        if number % 15 == 0:
            print(json.dumps(dict(stage='I45_diagnostic', anchors=number+1, total=len(anchors),
                time_s=now, ingested_batches=pointer, memory_cells=len(memory._cells),
                elapsed_s=perf_counter()-started)), flush=True)
    summary = rows_summary(rows, helper.statistics)
    primary = matched_tables(rows, helper.statistics, target='primary')
    legacy = matched_tables(rows, helper.statistics, target='legacy')
    uniform = primary['uniform']['all_horizons']
    armstats = uniform['arms']
    coverage = summary['uniform']
    decision = diagnostic_decision(uniform['matched_slots'], uniform['fixed_truth_eligible_slots'],
        cv=dict(observer=armstats['observer_cv'], memory=armstats['memory_cv']),
        oracle=dict(observer=armstats['observer_actual_pose'], memory=armstats['memory_actual_pose']),
        memory_cv_coverage=coverage['memory_cv']['all_horizons']['complete_nonempty_coverage'],
        memory_oracle_coverage=coverage['memory_actual_pose']['all_horizons']['complete_nonempty_coverage'],
        thresholds=protocol['registered_decision_thresholds'])
    inputs_after = verify_inputs(protocol)
    sources = json.loads((stage/'source_manifest.json').read_text())
    changed_sources = [r['path'] for r in sources['sources'] if sha(REPO_ROOT/r['path']) != r['sha256']]
    integrity = dict(CV_reproduction_passed=not parity_failures,
        CV_conditions=2, CV_compared_rows=len(anchors)*len(horizons)*2,
        absolute_numeric_tolerance=1e-10, parity_failure_count=len(parity_failures),
        parity_failures=parity_failures, first_map_prefix=prefix_result,
        all_anchor_prefixes_match=all(a['prefix_matches_I44'] for a in anchor_records),
        causal_deliveries=all(a['maximum_ingested_delivery_s'] is None or a['maximum_ingested_delivery_s'] <= a['anchor_time_s']+1e-9 for a in anchor_records),
        all_maps_locked=all(a['map_locked_for_four_conditions'] for a in anchor_records),
        sources_unchanged=not changed_sources, changed_sources=changed_sources,
        inputs_before_sha256=inputs_before, inputs_after_sha256=inputs_after,
        input_hashes_unchanged=inputs_before == inputs_after,
        controller_invoked=False, new_physics_runs=0, new_depth_renders=0, scene_evaluation=False)
    integrity_passed = bool(not parity_failures and prefix_result['passed']
        and integrity['all_anchor_prefixes_match'] and integrity['causal_deliveries']
        and integrity['all_maps_locked'] and integrity['sources_unchanged'] and integrity['input_hashes_unchanged'])
    if not integrity_passed:
        decision = dict(decision_branch='integrity_failed_no_scientific_interpretation')
    report = dict(iteration='I-45', scope=protocol['scope'], anchors=len(anchors),
        unique_query_slots=len(anchors)*len(horizons), method_rows=len(rows), arms=ARMS,
        integrity=integrity, integrity_passed=integrity_passed, summary=summary,
        primary_common_fourway=primary, legacy_common_fourway=legacy, registered_decision=decision,
        strata_overlap='6.8s belongs to uniform and dense_bump. These are nested within one historical run, not independent trials.',
        runtime=dict(total_elapsed_s=perf_counter()-started, map_update_wall_s=helper.statistics(map_wall),
            four_condition_query_wall_s=helper.statistics(query_wall), interpretation='Offline runtime; not a real-time claim.'),
        limitations=[protocol['truth']['count_caveat'], protocol['truth']['timing'],
            'Actual-pose conditions receive future configuration as hindsight, not a deployable prediction.',
            'Known ideal geometry and frozen local heightfield planes do not predict force, load share or slip.',
            'No scene/terrain normal truth, future depth or future observer update enters generation.',
            'No new controller commands, dynamics, camera capture or changed driving behavior.'])
    save_json(stage/'predictions.json', rows)
    save_json(stage/'anchor_records.json', anchor_records)
    save_json(stage/'report.json', report)
    save_json(stage/'artifact_manifest.json', {name:dict(sha256=sha(stage/name), bytes=(stage/name).stat().st_size)
        for name in ('report.json', 'predictions.json', 'anchor_records.json', 'source_manifest.json')})
    if not integrity_passed:
        raise RuntimeError('I45 integrity failed; preserved stage must not be scientifically interpreted')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--tag', default='stage01')
    parser.add_argument('--run', action='store_true', help='Execute saved-data diagnostic after preflight')
    args = parser.parse_args()
    if not args.tag.replace('_', '').isalnum():
        parser.error('Stage tag must be alphanumeric with optional underscores')
    output = args.output.resolve()
    protocol = json.loads((output/'protocol.json').read_text())
    if not args.run:
        hashes = verify_inputs(protocol)
        print(json.dumps(dict(preflight_passed=True, inputs=hashes,
                             future_stage=str(output/'campaign'/args.tag))), flush=True)
        return
    stage = output/'campaign'/args.tag
    stage.mkdir(parents=True, exist_ok=False)
    snapshot_sources(stage, output)
    try:
        run_diagnostic(output, stage, protocol)
    except Exception as error:
        save_json(stage/'failure.json', dict(error=str(error), traceback=traceback.format_exc()))
        raise


if __name__ == '__main__':
    main()
