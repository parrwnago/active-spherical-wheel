"""Lossless I68 storage adapter; scientific equations are unchanged.

load_trace(package) restores every original capture field and checks dtype,
shape and SHA256 of its bytes. Consecutive before-states/contact packets are
stored once, with explicit exceptions if continuity does not hold. Offline
q/v arrays are regenerated with the unchanged frozen collector function.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np


PRECISION_KEYS = ('time_s', 'exact_qpos', 'exact_qvel', 'exact_applied',
    'exact_previous_applied', 'exact_observer_interval_mean',
    'exact_observer_generalized', 'exact_estimated_normal',
    'exact_estimated_support_loads_N', 'exact_estimated_contact_point_m')
REMOVED = ('bridge_state_before', 'contact_rows_before', 'contact_offsets_before',
           'qpos_before', 'qvel_before', 'qpos_after', 'qvel_after')
PREFIX = '__archive_'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def binding(a):
    a = np.asarray(a)
    return dict(dtype=a.dtype.str, shape=list(a.shape),
                sha256=hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest())


def read(path):
    with np.load(path, allow_pickle=False) as packet:
        return {key: packet[key] for key in packet.files}


def equal(a, b):
    return binding(a) == binding(b)


def validate(arrays, expected):
    if set(arrays) != set(expected):
        raise AssertionError('Restored array field set differs')
    differences = {key: dict(expected=expected[key], actual=binding(arrays[key]))
                   for key in expected if binding(arrays[key]) != expected[key]}
    if differences:
        raise AssertionError('Lossless restoration mismatch: '+json.dumps(differences, sort_keys=True))
    return dict(all_arrays_byte_exact=True, array_count=len(expected))


def collector(path):
    spec = importlib.util.spec_from_file_location('_frozen_i68_capture', path)
    out = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(out)
    return out


def encode(trace):
    if any(key.startswith(PREFIX) for key in trace):
        raise ValueError('Reserved archive key prefix already present')
    n = len(trace['bridge_state'])
    if n == 0:
        raise ValueError('Registered full trace must be nonempty')
    payload = {key: value for key, value in trace.items() if key not in REMOVED}
    expected_before = np.concatenate((trace['bridge_state_before'][:1], trace['bridge_state'][:-1]), axis=0)
    exceptions = np.array([i for i in range(n) if not equal(expected_before[i], trace['bridge_state_before'][i])], np.int64)
    payload[PREFIX+'first_state_before'] = trace['bridge_state_before'][:1]
    payload[PREFIX+'state_exception_index'] = exceptions
    payload[PREFIX+'state_exception_rows'] = trace['bridge_state_before'][exceptions]
    old_offsets, post_offsets = trace['contact_offsets_before'], trace['contact_offsets']
    pre = trace['contact_rows_before']
    payload[PREFIX+'first_contacts_before'] = pre[old_offsets[0]:old_offsets[1]]
    indices, packets = [], []
    for i in range(1, n):
        previous = trace['contact_rows'][post_offsets[i-1]:post_offsets[i]]
        actual = pre[old_offsets[i]:old_offsets[i+1]]
        if not equal(previous, actual):
            indices.append(i)
            packets.append(actual)
    payload[PREFIX+'contact_exception_index'] = np.asarray(indices, np.int64)
    payload[PREFIX+'contact_exception_offsets'] = np.r_[0, np.cumsum([len(x) for x in packets], dtype=np.int64)]
    payload[PREFIX+'contact_exception_rows'] = np.concatenate(packets, axis=0) if packets else np.empty((0, 16), pre.dtype)
    return payload


def decode(payload, converter):
    out = {key: value for key, value in payload.items() if not key.startswith(PREFIX)}
    post = out['bridge_state']
    before = np.concatenate((payload[PREFIX+'first_state_before'], post[:-1]), axis=0)
    before[payload[PREFIX+'state_exception_index']] = payload[PREFIX+'state_exception_rows']
    out['bridge_state_before'] = before
    indices = payload[PREFIX+'contact_exception_index']
    positions = {int(value): index for index, value in enumerate(indices)}
    packets = [payload[PREFIX+'first_contacts_before']]
    exceptions, extra_offsets = payload[PREFIX+'contact_exception_rows'], payload[PREFIX+'contact_exception_offsets']
    for i in range(1, len(post)):
        if i in positions:
            j = positions[i]
            packet = exceptions[extra_offsets[j]:extra_offsets[j+1]]
        else:
            packet = out['contact_rows'][out['contact_offsets'][i-1]:out['contact_offsets'][i]]
        packets.append(packet)
    out['contact_rows_before'] = np.concatenate(packets, axis=0)
    out['contact_offsets_before'] = np.r_[0, np.cumsum([len(x) for x in packets], dtype=np.int64)]
    out['qpos_before'], out['qvel_before'] = converter(out['bridge_state_before'])
    out['qpos_after'], out['qvel_after'] = converter(out['bridge_state'])
    return out


def manifest(package):
    package = Path(package)
    data = json.loads((package/'capture_manifest.json').read_text())
    for row in data['files']:
        if sha(package/row['path']) != row['sha256']:
            raise AssertionError('Portable dependency changed: '+row['path'])
    return data


def load_trace(package):
    package = Path(package)
    data = manifest(package)
    conversion = collector(package.parent/'portable_tools/frozen_substep_trace.py').bridge_state_to_qv
    out = decode(read(package/'capture_payload.npz'), conversion)
    validate(out, data['trace_array_bindings'])
    return out


def load_precision(package, old=False):
    package = Path(package)
    data = manifest(package)
    out = read(package/'essential_precision.npz')
    validate(out, data['old_precision_array_bindings' if old else 'precision_array_bindings'])
    return out


def pack(args):
    package = Path(args.output)
    if package.exists():
        raise FileExistsError(package)
    seal = json.loads(Path(args.seal).read_text())
    for row in seal['files']:
        if sha(row['path']) != row['sha256']:
            raise AssertionError('Pre-packing dependency changed: '+row['path'])
    source, new, old = read(args.trace), read(args.precision), read(args.old_precision)
    new = {key: new[key] for key in PRECISION_KEYS}
    old = {key: old[key] for key in PRECISION_KEYS}
    if not all(equal(new[key], old[key]) for key in PRECISION_KEYS):
        raise AssertionError('Old/new essential precision differs; cannot deduplicate')
    payload = encode(source)
    package.mkdir(parents=True)
    np.savez_compressed(package/'capture_payload.npz', **payload)
    np.savez_compressed(package/'essential_precision.npz', **new)
    fields = {key: binding(value) for key, value in source.items()}
    data = dict(schema_version=1, numpy_version=np.__version__,
        trace_original=dict(path=str(args.trace), sha256=sha(args.trace)),
        precision_original=dict(path=str(args.precision), sha256=sha(args.precision)),
        old_precision_original=dict(path=str(args.old_precision), sha256=sha(args.old_precision)),
        trace_array_bindings=fields,
        precision_array_bindings={key: binding(value) for key, value in new.items()},
        old_precision_array_bindings={key: binding(value) for key, value in old.items()},
        precision_scope='Only ten original fields used by frozen offline analysis; original complete59-field parity remains a separate campaign receipt.',
        precision_deduplication='Old and new each independently bound to their original file and per-array bytes; shared identical essential packet only after comparison.',
        exception_counts=dict(state_rows=len(payload[PREFIX+'state_exception_index']),
                              contact_packets=len(payload[PREFIX+'contact_exception_index'])),
        reconstruction='Original frozen collector converts cached raw24 to q/v. Earlier before packets use previous post packets, except stored explicit discontinuities.',
        files=[])
    for path in (package/'capture_payload.npz', package/'essential_precision.npz',
                 *sorted((package.parent/'portable_tools').glob('*.py'))):
        data['files'].append(dict(path=str(Path(__import__('os').path.relpath(path, package))),
                                  sha256=sha(path), bytes=path.stat().st_size))
    (package/'capture_manifest.json').write_text(json.dumps(data, indent=2, sort_keys=True)+'\n')
    restored = load_trace(package)
    checks = dict(trace=validate(restored, fields), precision=validate(load_precision(package), data['precision_array_bindings']),
                  old_precision=validate(load_precision(package, old=True), data['old_precision_array_bindings']),
                  exception_counts=data['exception_counts'], numpy_version=np.__version__,
                  compact_bytes=sum((package/name).stat().st_size for name in ('capture_payload.npz', 'essential_precision.npz')),
                  original_trace_bytes=Path(args.trace).stat().st_size)
    (package/'packing_verification.json').write_text(json.dumps(checks, indent=2, sort_keys=True)+'\n')
    print(json.dumps(checks, sort_keys=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action', required=True)
    build = actions.add_parser('pack')
    for name in ('trace', 'precision', 'old-precision', 'seal', 'output'):
        build.add_argument('--'+name, type=Path, required=True)
    expand = actions.add_parser('expand')
    expand.add_argument('--package', type=Path, required=True)
    expand.add_argument('--output', type=Path, required=True)
    check = actions.add_parser('check')
    check.add_argument('--package', type=Path, required=True)
    args = parser.parse_args()
    if args.action == 'pack':
        pack(args)
    elif args.action == 'expand':
        if args.output.exists():
            raise FileExistsError(args.output)
        np.savez_compressed(args.output, **load_trace(args.package))
        print(json.dumps(dict(restored_array_bytes_exact=True, output=str(args.output))))
    else:
        data = manifest(args.package)
        result = dict(trace=validate(load_trace(args.package), data['trace_array_bindings']),
                      precision=validate(load_precision(args.package), data['precision_array_bindings']),
                      old_precision=validate(load_precision(args.package, old=True), data['old_precision_array_bindings']),
                      numpy_version=np.__version__)
        print(json.dumps(result, sort_keys=True))


if __name__ == '__main__':
    main()
