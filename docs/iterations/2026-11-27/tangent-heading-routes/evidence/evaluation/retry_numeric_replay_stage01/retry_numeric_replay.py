"""I76 exact saved-array replay diagnostic; never a controller adoption gate.

The plan was fixed after attempt01's endpoint-reference exception, before the
corrected retry. Compare every stored field of twelve full recordings and
three saved failure prefixes; do not score their motion performance. The three
explicit aborts have no saved precision and are excluded openly. This source
must be sealed with its inputs before any NPZ-array load and executed only
after root releases all eighteen corrected outputs.
"""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import shutil

import numpy as np

BASE = Path(__file__).resolve().parents[1]
ROOT = BASE.parents[1]
PRIOR_SEAL_SHA = 'd7da4005b306359d1ae031ec657fb94e62390aafab469d2e3336d3c9279ca81c'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def seal_bindings(seal, base):
    """Verify original container bytes, not inferred numeric equality."""
    result = {}
    for relative, binding in seal['files'].items():
        path = (base / relative).resolve()
        path.relative_to(base.resolve())
        if not path.is_file() or path.stat().st_size != binding['bytes'] or sha(path) != binding['sha256']:
            raise ValueError('Sealed artifact changed: ' + str(path))
        result[path] = binding
    return result


def release(protocol_path, seal_path, runs_root, prior_root, plan_path):
    """Source/container/metadata checks only; no NumPy-array reads."""
    protocol = json.loads(protocol_path.read_text())
    jobs = protocol['cases']
    expected = {(route, policy, phase) for route in ('straight', 'arc', 'eight')
                for policy in ('N', 'T') for phase in (-1., .3, .9)}
    if protocol.get('iteration') != 'I-76' or len(jobs) != 18 or len({j['case_id'] for j in jobs}) != 18:
        raise ValueError('All eighteen unique corrected I76 jobs required')
    if {(j['route_id'], j['policy'], float(j['phase_rad'])) for j in jobs} != expected:
        raise ValueError('Registered route/phase/policy coverage differs')
    current = json.loads(seal_path.read_text())
    processes = current.get('processes', [])
    if (current.get('protocol_sha256') != sha(protocol_path) or not current.get('all_processes_successful')
        or len(processes) != 18 or {p['case_id'] for p in processes} != {j['case_id'] for j in jobs}
        or any(p.get('returncode') != 0 for p in processes)):
        raise ValueError('Successful all-eighteen corrected output seal required')
    current_bindings = seal_bindings(current, BASE)
    for job in jobs:
        if (runs_root/job['case_id']/'precision.npz').resolve() not in current_bindings:
            raise ValueError('Corrected precision missing from all-eighteen seal')
    old_seal_path = prior_root/'campaign/outputs_sealed.json'
    if sha(old_seal_path) != PRIOR_SEAL_SHA:
        raise ValueError('Preserved first-attempt output seal differs')
    old = json.loads(old_seal_path.read_text())
    old_protocol_path = prior_root/'protocol/protocol.json'
    if old['protocol_sha256'] != sha(old_protocol_path):
        raise ValueError('Preserved first-attempt protocol differs')
    old_bindings = seal_bindings(old, prior_root)
    plan = json.loads(plan_path.read_text())
    if plan.get('prior_output_seal_sha256') != PRIOR_SEAL_SHA:
        raise ValueError('Predeclared replay plan differs')
    full = plan['completed_comparisons']; partial = plan['partial_prefix_comparisons']; excluded = plan['excluded_explicit_aborts']
    if len(full) != 12 or len(partial) != 3 or len(excluded) != 3:
        raise ValueError('Predeclared twelve full / three prefixes / three exclusions required')
    if len(set(full+partial+excluded)) != 18 or set(full+partial+excluded) != {j['case_id'] for j in jobs}:
        raise ValueError('Predeclared case partitions differ from corrected campaign')
    items = []
    for case in full:
        source = prior_root/'campaign/runs'/case/'precision.npz'
        if source.resolve() not in old_bindings:
            raise ValueError('Completed historical precision absent from original seal')
        items.append(dict(case_id=case, kind='completed_full_precision', prior=source,
                          corrected=runs_root/case/'precision.npz', prior_bound_in_original_seal=True))
    for case in partial:
        source = prior_root/'campaign/runs'/case/'partial_precision.npz'
        if not source.is_file():
            raise FileNotFoundError('Recorded failure prefix is absent: '+case)
        if source.resolve() in old_bindings:
            raise ValueError('Partial provenance assumption changed; inspect before relabeling')
        items.append(dict(case_id=case, kind='exception_partial_precision', prior=source,
                          corrected=runs_root/case/'precision.npz', prior_bound_in_original_seal=False))
        truth = source.with_name('partial_truth.npz')
        if truth.exists():
            items.append(dict(case_id=case, kind='exception_partial_truth', prior=truth,
                              corrected=runs_root/case/'precision.npz', prior_bound_in_original_seal=truth.resolve() in old_bindings))
    return protocol, plan, items, old_seal_path, old_protocol_path


def compare(item):
    full = item['kind'] == 'completed_full_precision'
    with np.load(item['prior'], allow_pickle=False) as original, np.load(item['corrected'], allow_pickle=False) as corrected:
        original_keys, corrected_keys = set(original.files), set(corrected.files)
        missing = sorted(original_keys-corrected_keys)
        extras = sorted(corrected_keys-original_keys)
        time_key = 'time_s' if 'time_s' in original_keys else 'truth_time_s'
        saved_time = original[time_key]
        n = len(saved_time)
        fields = {}
        for key in sorted(original_keys):
            left = original[key]
            if left.dtype.hasobject:
                raise TypeError('Object fields are not numeric replay inputs')
            if not left.ndim or left.shape[0] != n:
                raise ValueError('Stored field lacks the declared saved-row clock: '+key)
            if key not in corrected_keys:
                fields[key] = dict(exact=False, reason='missing_corrected_field', prior_dtype=left.dtype.str, prior_shape=list(left.shape))
                continue
            entire = corrected[key]
            right = entire if full else entire[:n]
            dtype_ok = left.dtype == right.dtype
            shape_ok = left.shape == right.shape
            a, b = np.ascontiguousarray(left), np.ascontiguousarray(right)
            exact = bool(dtype_ok and shape_ok and a.tobytes() == b.tobytes())
            first = None
            if not exact and dtype_ok and shape_ok and n:
                row_bytes = a.dtype.itemsize * int(np.prod(a.shape[1:], dtype=int))
                if row_bytes:
                    different = np.flatnonzero(np.any(a.view(np.uint8).reshape(n, row_bytes) != b.view(np.uint8).reshape(n, row_bytes), axis=1))
                    first = int(different[0]) if len(different) else None
            elif not exact and dtype_ok and not full and entire.shape[0] < n:
                first = int(entire.shape[0])
            fields[key] = dict(exact=exact, dtype_equal=bool(dtype_ok), shape_equal=bool(shape_ok),
                               prior_dtype=left.dtype.str, corrected_dtype=right.dtype.str,
                               prior_shape=list(left.shape), compared_corrected_shape=list(right.shape),
                               prior_array_sha256=hashlib.sha256(a.tobytes()).hexdigest(),
                               corrected_compared_array_sha256=hashlib.sha256(b.tobytes()).hexdigest(),
                               first_different_saved_row=first,
                               first_different_saved_time_s=None if first is None or first >= n else float(saved_time[first]))
        key_policy_ok = not missing and (not full or not extras)
        passed = bool(key_policy_ok and all(v['exact'] for v in fields.values()))
        final_n = len(corrected['time_s'])
        return dict(case_id=item['case_id'], kind=item['kind'], passed=passed,
                    saved_rows=n, saved_terminal_time_s=float(saved_time[-1]) if n else None,
                    corrected_total_rows=final_n, corrected_extra_rows_beyond_saved_prefix=0 if full else max(0, final_n-n),
                    prior_bound_in_original_launcher_seal=item['prior_bound_in_original_seal'],
                    prior_partial_provenance_note=None if item['prior_bound_in_original_seal'] else
                        'Not listed by the original launcher seal. Preserved original failure output, explicitly bound in this diagnostic before comparison loads; earlier diagnosis read its time field only.',
                    field_count=len(fields), exact_fields=sum(v['exact'] for v in fields.values()),
                    missing_corrected_fields=missing, corrected_only_fields=extras,
                    key_policy='Full recordings require equal key sets; partial recordings require every stored field and explicitly report corrected-only fields.',
                    container_bytes_are_not_array_replay_target=True, fields=fields)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('protocol', 'runs-root', 'all-runs-seal', 'prior-root', 'plan', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    protocol_path, seal_path = args.protocol.resolve(), args.all_runs_seal.resolve()
    runs_root, prior_root = args.runs_root.resolve(), args.prior_root.resolve()
    plan_path, output = args.plan.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError('Preserve previous replay stages')
    protocol, plan, items, old_seal_path, old_protocol_path = release(protocol_path, seal_path, runs_root, prior_root, plan_path)
    paths = [Path(__file__).resolve(), protocol_path, seal_path, plan_path, old_seal_path, old_protocol_path,
             prior_root/'attempt_record.json', BASE/'runtime/route.py', prior_root/'pre_fix_source/runtime/route.py']
    paths += [item[k] for item in items for k in ('prior', 'corrected')]
    paths = sorted(set(paths))
    bindings = [dict(path=str(p), bytes=p.stat().st_size, sha256=sha(p)) for p in paths]
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(Path(__file__), output/'retry_numeric_replay.py')
    shutil.copy2(plan_path, output/'predeclared_plan.json')
    write(output/'registered_before_array_reads.json', dict(iteration='I-76', utc=datetime.now(timezone.utc).isoformat(),
          inputs=bindings, no_new_physics=True, no_controller_performance_scoring=True,
          not_an_adoption_or_acceptance_gate=True, completed_cases=plan['completed_comparisons'],
          partial_prefix_cases=plan['partial_prefix_comparisons'], excluded_explicit_aborts=plan['excluded_explicit_aborts'],
          original_seal_provenance='Twelve completed precision containers are in the original launcher seal. Three partial precision and optional partial truth containers are later bound here, before comparison arrays are loaded; no retroactive seal is claimed.',
          equality_policy='All stored fields: key policy, dtype, shape and contiguous raw array bytes (including NaN payloads). NPZ ZIP bytes and changed metadata are not numeric replay targets.'))
    reports = [compare(item) for item in items]
    unchanged = all(p.stat().st_size == item['bytes'] and sha(p) == item['sha256'] for item in bindings for p in [Path(item['path'])])
    if not unchanged:
        raise ValueError('Replay source/input changed during comparison')
    full = [r for r in reports if r['kind'] == 'completed_full_precision']
    partial = [r for r in reports if r['kind'] == 'exception_partial_precision']
    truth = [r for r in reports if r['kind'] == 'exception_partial_truth']
    passed = all(r['passed'] for r in reports)
    write(output/'report.json', dict(iteration='I-76', status='PASS_NUMERIC_REPLAY' if passed else 'NUMERIC_REPLAY_MISMATCH',
          all_available_stored_arrays_byte_exact=passed, input_source_unchanged=True,
          completed_recordings=len(full), completed_exact=sum(r['passed'] for r in full),
          partial_precision_recordings=len(partial), partial_precision_exact=sum(r['passed'] for r in partial),
          partial_truth_recordings=len(truth), partial_truth_exact=sum(r['passed'] for r in truth),
          excluded_explicit_aborts=plan['excluded_explicit_aborts'], cases=reports,
          no_new_physics=True, no_controller_performance_scoring=True, no_tuning_or_gate_changes=True,
          interpretation='Correction reproducibility diagnostic only. A mismatch is preserved, not used to tune the controller; equality does not establish tracking or heading acceptance. Three explicit aborted cases have no saved numeric comparison.'))
    write(output/'artifact_manifest.json', dict(files=[dict(path=p.name, bytes=p.stat().st_size, sha256=sha(p))
          for p in sorted(output.iterdir()) if p.is_file() and p.name != 'artifact_manifest.json']))
    print(json.dumps(dict(status='PASS_NUMERIC_REPLAY' if passed else 'NUMERIC_REPLAY_MISMATCH',
                         completed_exact=sum(r['passed'] for r in full), completed_total=len(full),
                         partial_precision_exact=sum(r['passed'] for r in partial), partial_precision_total=len(partial),
                         partial_truth_exact=sum(r['passed'] for r in truth), partial_truth_total=len(truth))))


if __name__ == '__main__':
    main()
