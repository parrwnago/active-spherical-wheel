"""Isolated NumPy-only I72 inference replay from a published evidence tree.

Calls the byte-preserved infer_case API, never the repository-bound CLI or
seal loader. Every canonical output array is bound by dtype, shape and bytes.
Only input/output plumbing is added; scientific arithmetic is unchanged.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time


CASES = ('m100_B', 'p030_B', 'p090_B')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n')


def verify_package(package):
    package = Path(package)
    seal = json.loads((package / 'publication/portable_tools/replay_source_input_seal.json').read_text())
    for relative, expected in seal['files'].items():
        path = Path(relative)
        if path.is_absolute() or '..' in path.parts:
            raise ValueError('Nonportable seal path: ' + relative)
        if sha(package / path) != expected:
            raise ValueError('Sealed portable source/input changed: ' + relative)
    return seal


def array_binding(np, value):
    value = np.asarray(value)
    return dict(dtype=value.dtype.str, shape=list(value.shape),
                sha256=hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest())


def nested_bindings(np, value, prefix=''):
    """Include every decoded ranking/fallback array in the mutation guard."""
    if isinstance(value, dict):
        result = {}
        for key in sorted(value):
            result.update(nested_bindings(np, value[key], prefix + '/' + key))
        return result
    return {prefix: array_binding(np, value)}


def worker(args):
    import numpy as np
    package = args.package.resolve()
    verify_package(package)
    names = ('geometry_first_contact', 'frozen_i60_rank', 'packet_io')
    if any(name in sys.modules for name in names):
        raise RuntimeError('Scientific modules loaded before isolated setup')
    tools = package / 'inference_tools'
    inputs = package / 'inputs'
    sys.path.insert(0, str(inputs))
    sys.path.insert(0, str(tools))
    import packet_io
    import geometry_first_contact
    import frozen_i60_rank
    spec = importlib.util.spec_from_file_location('_i72_packet_runner', tools / 'run_case.py')
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    modules = dict(packet_io=(packet_io, inputs / 'packet_io.py'),
        geometry_first_contact=(geometry_first_contact, tools / 'geometry_first_contact.py'),
        frozen_i60_rank=(frozen_i60_rank, tools / 'frozen_i60_rank.py'),
        frozen_runner=(runner, tools / 'run_case.py'))
    imports = {}
    for name, (module, path) in modules.items():
        actual = Path(module.__file__).resolve()
        if actual != path.resolve():
            raise RuntimeError('Unexpected scientific import: ' + name)
        imports[name] = dict(path=str(actual), sha256=sha(actual))
    if any(name == 'spherical_wheel' or name.startswith('spherical_wheel.') for name in sys.modules):
        raise RuntimeError('Live repository package leaked into isolated replay')
    output = args.output / args.worker
    output.mkdir(parents=True, exist_ok=False)
    save(output / 'registered_before_replay.json', dict(case=args.worker,
        source_input_seal_sha256=sha(package / 'publication/portable_tools/replay_source_input_seal.json'),
        python=sys.version, executable=sys.executable, numpy=np.__version__, cwd=os.getcwd(), imports=imports,
        boundary='Sealed original rank features, physical candidate coordinates and native fallback only; no truth, NNLS, dynamics or physics.',
        invoked_api='Exact copied run_case.infer_case(packet_io.load_case(...)); repository-bound run and verify_seal are not called.'))
    packet = packet_io.load_case(package / 'inputs/stage01', args.worker)
    before = nested_bindings(np, packet)
    reference = json.loads((package / 'publication/portable_tools/reference_array_bindings.json').read_text())['cases'][args.worker]
    started = time.monotonic()
    arrays = runner.infer_case(packet)
    actual = {key: array_binding(np, value) for key, value in arrays.items()}
    input_unchanged = before == nested_bindings(np, packet)
    expected = reference['arrays']
    exact = actual == expected
    np.savez_compressed(output / 'predictions.npz', **arrays)
    verify_package(package)
    report = dict(case=args.worker, frames=len(arrays['time_s']), completed=True,
        all_prediction_arrays_byte_exact=exact, array_field_set_exact=set(actual) == set(expected),
        input_arrays_unchanged=input_unchanged, source_input_seals_unchanged=True,
        differing_arrays=sorted(key for key in set(actual) | set(expected) if actual.get(key) != expected.get(key)),
        actual_array_bindings=actual, verified_array_count=len(actual),
        decoded_input_array_count=len(before), imports=imports,
        elapsed_s=time.monotonic() - started, numpy=np.__version__,
        truth_read=False, nnls_refit=False, physics_run=False, live_package_imported=False)
    save(output / 'replay_report.json', report)
    print(json.dumps(dict(case=args.worker, completed=True, all_exact=exact, input_unchanged=input_unchanged)), flush=True)
    if not exact or not input_unchanged:
        raise RuntimeError('Strict isolated replay mismatch; original output values preserved')


def parent(args):
    package = args.package.resolve()
    verify_package(package)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    children = []
    for case in CASES:
        command = [sys.executable, '-I', str(Path(__file__).resolve()), '--worker', case,
                   '--package', str(package), '--output', str(output)]
        environment = os.environ.copy()
        for key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
                    'COLCON_PREFIX_PATH', 'LD_PRELOAD'):
            environment.pop(key, None)
        environment.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
        log = (output / (case + '.log')).open('x')
        child = subprocess.Popen(command, cwd='/tmp', env=environment, stdout=log, stderr=subprocess.STDOUT)
        children.append((case, child, log, command))
    statuses = []
    for case, child, log, command in children:
        code = child.wait()
        log.close()
        statuses.append(dict(case=case, returncode=code, command=command))
    reports = {case: json.loads((output / case / 'replay_report.json').read_text()) for case in CASES
               if (output / case / 'replay_report.json').exists()}
    verify_package(package)
    all_exact = len(reports) == 3 and all(r['all_prediction_arrays_byte_exact'] and r['input_arrays_unchanged'] for r in reports.values())
    receipt = dict(children=statuses, all_three_completed=len(reports) == 3, all_arrays_byte_exact=all_exact,
        frames=sum(r['frames'] for r in reports.values()),
        verified_prediction_arrays=sum(r['verified_array_count'] for r in reports.values()),
        source_input_seals_unchanged=True,
        cases={case: dict(frames=r['frames'], all_arrays_byte_exact=r['all_prediction_arrays_byte_exact'],
                         input_arrays_unchanged=r['input_arrays_unchanged'], verified_array_count=r['verified_array_count'],
                         decoded_input_array_count=r['decoded_input_array_count'],
                         elapsed_s=r['elapsed_s'], imports=r['imports'], numpy=r['numpy']) for case, r in reports.items()},
        scope='Cached NumPy inference only; all source/parameters unchanged; no labels, NNLS, dynamics or physics.')
    save(output / 'receipt.json', receipt)
    if not all_exact or any(status['returncode'] for status in statuses):
        raise RuntimeError('One or more replay workers failed; inspect preserved logs')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--worker', choices=CASES, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.worker:
        worker(args)
    else:
        parent(args)
