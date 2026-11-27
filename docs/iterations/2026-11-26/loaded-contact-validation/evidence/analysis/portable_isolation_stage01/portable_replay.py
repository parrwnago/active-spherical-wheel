"""Replay I57 response metrics using only NumPy and the compact saved arrays.

Usage: python portable_replay.py --dataset DATASET --output NEW_DIRECTORY

DATASET contains protocol.json, report.json, portable_inputs/*.npz and its
artifact_manifest.json. No project imports, original full I56 log, Gazebo,
MuJoCo, controller or model execution is needed. Pre-anchor replay parity is
certified metadata, not independently reproduced by these endpoint arrays.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

GROUPS = {
    'planar': ((0, 1), 2e-4, 'm/s'),
    'transverse': ((3, 4), 2e-3, 'rad/s'),
    'world_yaw': ((9,), 2e-3, 'rad/s'),
    'shaft': ((6,), 2e-2, 'rad/s'),
    'shell_A': ((7,), 2e-2, 'rad/s'),
    'shell_B': ((8,), 2e-2, 'rad/s'),
}
ROLES = {
    'native': 'native_center_amplitude_only_counterfactual',
    'governed': 'governed_center',
    'duplicate': 'deterministic_duplicate_control',
    'midpoint': 'native_governed_midpoint_amplitude_only_counterfactual',
    'continuation': 'exact_recorded_policy_control',
    'column': 'governed_center_column_probe',
}


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, np.ndarray):
        return safe(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {key: safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(item) for item in value]
    return value


def save(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def load(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


def exact(a, b):
    a, b = np.asarray(a), np.asarray(b)
    return a.dtype == b.dtype and a.shape == b.shape and a.tobytes() == b.tobytes()


def rotation(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > 1e-6:
        raise ValueError('A finite unit wxyz quaternion is required')
    w, x, y, z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def observed_difference(a, b, index, initial_rotation):
    va, vb = a['exact_qvel'][index], b['exact_qvel'][index]
    world_a = rotation(a['exact_qpos'][index, 3:7])@va[3:6]
    world_b = rotation(b['exact_qpos'][index, 3:7])@vb[3:6]
    world_delta = world_a-world_b
    return np.r_[va[:3]-vb[:3], initial_rotation.T@world_delta,
                 va[6:]-vb[6:], world_delta[2]]


def predicted_difference(response, delta, initial_rotation):
    generalized = response@delta
    return np.r_[generalized, (initial_rotation@generalized[3:6])[2]]


def group_scores(predicted, observed):
    result = {}
    for name, (indices, floor, units) in GROUPS.items():
        p, o = predicted[list(indices)], observed[list(indices)]
        pn, on = float(np.linalg.norm(p)), float(np.linalg.norm(o))
        error = float(np.linalg.norm(o-p))
        presolved, oresolved = pn > floor, on > floor
        cosine = float(np.clip(p@o/(pn*on), -1, 1)) if presolved and oresolved else None
        gain = on/pn if presolved else None
        relative = error/max(on, floor)
        if not presolved and not oresolved:
            status, passed = 'unresolved', None
        elif not presolved:
            status, passed = 'false_negligible_authority', False
        elif not oresolved:
            status, passed = 'overpredicted_authority', False
        else:
            passed = bool(relative <= .5 and cosine >= .8 and .5 <= gain <= 2.)
            status = 'resolved_pass' if passed else 'resolved_fail'
        result[name] = dict(predicted=p.tolist(), observed=o.tolist(), floor=floor,
            predicted_norm=pn, observed_norm=on, absolute_error=error,
            relative_vector_error=relative, directional_cosine=cosine,
            observed_over_predicted_gain=gain, status=status, passes_screen=passed,
            transverse_nearzero_absolute_pass=bool(on <= floor) if name == 'transverse' and not presolved else None,
            units=units)
    return result


def linearity_scores(full, half):
    result = {}
    for name, (indices, floor, units) in GROUPS.items():
        a, b = full[list(indices)], 2*half[list(indices)]
        norm, half_norm = float(np.linalg.norm(a)), float(np.linalg.norm(b))
        error = float(np.linalg.norm(a-b))
        ratio = error/max(norm, floor)
        unresolved = norm <= floor and half_norm <= floor
        result[name] = dict(full_observed=a.tolist(), twice_half_observed=b.tolist(),
            absolute_nonlinearity=error, relative_nonlinearity=ratio, floor=floor,
            unresolved=unresolved, passes_screen=None if unresolved else bool(ratio <= .25), units=units)
    return result


def compare(actual, expected, label, differences):
    """Compare complete metric structures with portable floating tolerances."""
    if isinstance(expected, dict):
        if not isinstance(actual, dict) or set(actual) != set(expected):
            differences.append(dict(path=label, issue='dictionary keys differ'))
            return
        for key in expected:
            compare(actual[key], expected[key], label+'/'+key, differences)
    elif isinstance(expected, (list, tuple)):
        if not isinstance(actual, (list, tuple, np.ndarray)) or len(actual) != len(expected):
            differences.append(dict(path=label, issue='sequence lengths differ'))
            return
        for index, item in enumerate(expected):
            compare(actual[index], item, label+f'/{index}', differences)
    elif isinstance(expected, bool) or expected is None or isinstance(expected, str):
        if actual != expected:
            differences.append(dict(path=label, actual=safe(actual), expected=expected))
    elif isinstance(expected, (int, float)):
        if not np.isclose(actual, expected, rtol=1e-10, atol=1e-12):
            differences.append(dict(path=label, actual=float(actual), expected=expected))
    elif actual != expected:
        differences.append(dict(path=label, issue='values differ'))


def direct_controls(branches, original):
    def role(name):
        items = [value for value in branches.values() if value['job']['role'] == ROLES[name]]
        if len(items) > 1:
            raise ValueError('Repeated branch role: '+name)
        return items[0]['arrays'] if items else None
    continuation, governed, duplicate = (role(name) for name in ('continuation', 'governed', 'duplicate'))
    state_fields = ('time_s', 'exact_qpos', 'exact_qvel', 'exact_previous_applied')
    continuation_check = None if continuation is None else all(exact(continuation[key], original[key]) for key in state_fields)
    duplicate_fields = state_fields + ('actual_applied', 'bridge_state', 'contact_rows', 'contact_offsets', 'command_sequence')
    duplicate_check = None if governed is None or duplicate is None else all(exact(governed[key], duplicate[key]) for key in duplicate_fields)
    anchors = {key: all(exact(value['arrays'][field][0], original[field][0]) for field in state_fields) for key, value in branches.items()}
    input_history = {key: bool(exact(value['arrays']['actual_applied'], value['arrays']['exact_previous_applied'])
        and exact(value['arrays']['actual_applied'][1:], value['arrays']['command_sequence'])) for key, value in branches.items()}
    continuation_commands = None if continuation is None else exact(continuation['command_sequence'], original['exact_applied'][:4])
    return dict(continue_saved_exact=continuation_check, governed_duplicate_exact=duplicate_check,
                archived_anchor_exact=anchors, actual_previous_input_history_exact=input_history,
                continuation_saved_command_sequence_exact=continuation_commands)


def replay(dataset, output):
    if output.exists():
        raise FileExistsError('Preserve earlier portable replay attempts')
    output.mkdir(parents=True)
    files = [dataset/'protocol.json', dataset/'report.json', dataset/'artifact_manifest.json']
    files += sorted((dataset/'portable_inputs').glob('*.npz'))
    if not files[3:]:
        raise ValueError('The compact portable_inputs arrays are required')
    inputs = {str(p.relative_to(dataset)): sha(p) for p in files}
    registered = datetime.now(timezone.utc).isoformat()
    save(output/'registered_before_arithmetic.json', dict(registered_utc=registered,
        source_sha256=sha(__file__), dataset_input_sha256=inputs,
        dependencies='Python standard library and NumPy only',
        no_physics=True, no_original_full_I56_log=True,
        scope='Recompute endpoint controls, all paired loaded/free metrics and full/half linearity from compact full-rate arrays. Pre-anchor parity remains certified metadata.'))
    shutil.copy2(__file__, output/'portable_replay.py')
    protocol, canonical = read(dataset/'protocol.json'), read(dataset/'report.json')
    manifest = {item['path']: item['sha256'] for item in read(dataset/'artifact_manifest.json')['files']}
    for path in files:
        relative = str(path.relative_to(dataset))
        if relative == 'artifact_manifest.json':
            continue
        if relative not in manifest or sha(path) != manifest[relative]:
            raise ValueError('Dataset artifact hash mismatch: '+relative)
    differences, csv_rows = [], []
    result = dict(iteration='I-57', dataset_report_sha256=inputs['report.json'],
        source_sha256=sha(__file__), events={}, paired_score_group_count=0,
        linearity_group_count=0, no_physics=True,
        pre_anchor_prefix_scope='Original full prefix arrays are not in the compact dataset. Their parity flags and source-prefix hashes are archived canonical certifications; they are not independently recomputed here.',
        direct_control_scope='All archived anchor states, five-state saved continuations, duplicate endpoint/input/contact arrays and four held-command histories are directly checked from portable arrays.')
    available = []
    for event in protocol['events']:
        event_id = event['event_id']
        original = load(dataset/'portable_inputs'/(event_id+'_original_continuation.npz'))
        model = load(dataset/'portable_inputs'/(event_id+'_model.npz'))
        branches = {}
        for job in protocol['cases']:
            if job['event_id'] != event_id:
                continue
            path = dataset/'portable_inputs'/(job['case_id']+'_endpoints.npz')
            if not path.exists():
                continue
            arrays = load(path)
            stored_job = json.loads(arrays['job_json_utf8'].tobytes().decode())
            if stored_job != job:
                raise ValueError('Portable endpoint job differs from protocol: '+job['case_id'])
            branches[job['case_id']] = dict(job=job, arrays=arrays)
            available.append(job['case_id'])
        controls = direct_controls(branches, original)
        old = canonical['events'][event_id]
        for field in ('continue_saved_exact', 'governed_duplicate_exact'):
            compare(controls[field], old['controls'][field], event_id+'/controls/'+field, differences)
        if not all(controls['archived_anchor_exact'].values()) or not all(controls['actual_previous_input_history_exact'].values()):
            raise ValueError('An archived anchor or applied-input history failed direct equality')
        certified = dict(all_available_prefixes_exact=old['controls']['all_available_prefixes_exact'],
                         prefix_checks=old['controls']['prefix_checks'], independently_recomputed=False)
        def role(name):
            values = [b for b in branches.values() if b['job']['role'] == ROLES[name]]
            return values[0] if values else None
        native, governed, midpoint = (role(name) for name in ('native', 'governed', 'midpoint'))
        pairs = []
        if native is not None and governed is not None:
            pairs.append(('native_to_governed', governed, native))
        if native is not None and midpoint is not None:
            pairs.append(('native_to_midpoint', midpoint, native))
        for branch in branches.values():
            job = branch['job']
            if job['role'] == ROLES['column'] and governed is not None:
                pairs.append((f'u{job["input_column"]}_{job["amplitude"]}', branch, governed))
        observed, comparisons = {}, []
        canonical_comparisons = {(row['pair_id'], row['horizon_s']): row for row in old['comparisons']}
        for pair_id, a, b in pairs:
            ua, ub = a['arrays']['command_sequence'], b['arrays']['command_sequence']
            if not np.all(ua == ua[0]) or not np.all(ub == ub[0]):
                raise ValueError('A paired command is not a constant hold')
            delta = ua[0]-ub[0]
            for index, key in ((1, 'h005ms'), (4, 'h020ms')):
                R0 = model[key+'_physical_map'][3:6, 3:6]
                measured = observed_difference(a['arrays'], b['arrays'], index, R0)
                loaded = predicted_difference(model[key+'_response'], delta, R0)
                free = predicted_difference(model[key+'_free_response'], delta, R0)
                horizon = index*.005
                row = dict(pair_id=pair_id, horizon_s=horizon, actual_delta_input=delta.tolist(),
                    predicted=loaded.tolist(), free_predicted=free.tolist(), observed=measured.tolist(),
                    groups=group_scores(loaded, measured), free_groups=group_scores(free, measured))
                expected = canonical_comparisons.pop((pair_id, horizon))
                for field in row:
                    compare(row[field], expected[field], event_id+'/'+pair_id+'/'+key+'/'+field, differences)
                comparisons.append(row)
                observed[pair_id, key] = measured
                result['paired_score_group_count'] += 2*len(GROUPS)
                for variant, scores in (('loaded', row['groups']), ('free', row['free_groups'])):
                    for group, score in scores.items():
                        csv_rows.append(dict(event_id=event_id, pair_id=pair_id, horizon_ms=round(horizon*1000),
                            model=variant, group=group, **{name: value for name, value in score.items() if name not in ('predicted', 'observed')}))
        if canonical_comparisons:
            raise ValueError('Some canonical response pairs were not independently recomputed')
        linearity = []
        old_linearity = {(row['full_pair'], row['half_pair'], row['horizon_s']): row for row in old['linearity']}
        for full_id, half_id in [('native_to_governed', 'native_to_midpoint')]+[(f'u{k}_full', f'u{k}_half') for k in range(5)]:
            for key, horizon in (('h005ms', .005), ('h020ms', .02)):
                if (full_id, key) not in observed or (half_id, key) not in observed:
                    continue
                row = dict(full_pair=full_id, half_pair=half_id, horizon_s=horizon,
                    groups=linearity_scores(observed[full_id, key], observed[half_id, key]))
                expected = old_linearity.pop((full_id, half_id, horizon))
                compare(row['groups'], expected['groups'], event_id+'/'+full_id+'/'+key+'/linearity', differences)
                linearity.append(row)
                result['linearity_group_count'] += len(GROUPS)
        if old_linearity:
            raise ValueError('Some canonical full/half comparisons were not independently recomputed')
        result['events'][event_id] = dict(direct_endpoint_controls=controls,
            certified_pre_anchor_prefix=certified, comparisons=comparisons, linearity=linearity,
            canonical_interpretation_valid_using_certified_prefix=old['controls']['valid_for_interpretation'])
    compare(sorted(available), sorted(canonical['available_cases']), 'available_cases', differences)
    result['available_cases'] = sorted(available)
    result['missing_registered_cases'] = sorted(set(job['case_id'] for job in protocol['cases'])-set(available))
    result['canonical_numeric_comparison'] = dict(passed=not differences, differences=differences,
        relative_tolerance=1e-10, absolute_tolerance=1e-12,
        scope='Every observed/loaded/free response vector, every group score and every full/half group metric; direct continuation/duplicate controls and available case identities')
    if inputs != {str(path.relative_to(dataset)): sha(path) for path in files}:
        raise ValueError('Portable dataset inputs changed during replay')
    result['input_hashes_unchanged'] = True
    result['completed_utc'] = datetime.now(timezone.utc).isoformat()
    save(output/'report.json', result)
    with (output/'response_scores.csv').open('w', newline='') as stream:
        if csv_rows:
            writer = csv.DictWriter(stream, fieldnames=list(csv_rows[0]))
            writer.writeheader()
            writer.writerows(csv_rows)
    save(output/'artifact_manifest.json', dict(files=[dict(path=path.name, sha256=sha(path), bytes=path.stat().st_size)
        for path in sorted(output.iterdir()) if path.is_file() and path.name != 'artifact_manifest.json']))
    print(json.dumps({name: result[name] for name in ('paired_score_group_count', 'linearity_group_count', 'canonical_numeric_comparison', 'input_hashes_unchanged')}, indent=2))
    if differences:
        raise SystemExit('Portable metrics differ from the canonical report; failed attempt retained')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    replay(args.dataset.resolve(), args.output.resolve())


if __name__ == '__main__':
    main()
