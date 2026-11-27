"""Isolated, truth-free I-70 full inference replay from compact primitives.

Calls the byte-preserved scientific modules and ShadowInference class. This
wrapper changes input loading, output plumbing and verification only.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
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
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def verify_packet(tools, inputs):
    seal = json.loads((tools/'replay_source_seal.json').read_text())
    for relative, expected in seal['tool_files'].items():
        if sha(tools/relative) != expected:
            raise ValueError('sealed replay tool/source changed: '+relative)
    if sha(inputs/'package_manifest.json') != seal['primitive_package_manifest_sha256']:
        raise ValueError('primitive package manifest differs from frozen replay input')
    return seal


def isolated_imports(tools, inputs):
    """Fresh worker; the empty copied package must shadow the installed repo."""
    if any(name == 'spherical_wheel' or name.startswith('spherical_wheel.') for name in sys.modules):
        raise RuntimeError('scientific package already imported before isolated setup')
    source, measured = tools/'source', inputs/'source'
    sys.path.insert(0, str(measured)); sys.path.insert(0, str(source))
    from portable_inputs import load_primitives
    from input_features import frame_features
    import bank_points
    import spherical_wheel
    import spherical_wheel.shadow_contact_models as models
    import spherical_wheel.temporal_contact_filter as temporal
    spec = importlib.util.spec_from_file_location('i70_frozen_shadow_runner', source/'run_shadow_case.py')
    runner = importlib.util.module_from_spec(spec); spec.loader.exec_module(runner)
    expected = {'spherical_wheel': source/'spherical_wheel/__init__.py',
        'spherical_wheel.shadow_contact_models': source/'spherical_wheel/shadow_contact_models.py',
        'spherical_wheel.temporal_contact_filter': source/'spherical_wheel/temporal_contact_filter.py',
        'input_features': measured/'input_features.py', 'bank_points': measured/'bank_points.py',
        'portable_inputs': measured/'portable_inputs.py'}
    imported = {}
    for name, path in expected.items():
        actual = Path(sys.modules[name].__file__).resolve()
        if actual != path.resolve(): raise RuntimeError('non-packet scientific import: '+name)
        imported[name] = dict(path=str(actual), sha256=sha(actual))
    for name, module in sys.modules.items():
        if name == 'spherical_wheel' or name.startswith('spherical_wheel.'):
            path = Path(module.__file__).resolve()
            if not path.is_relative_to(source.resolve()): raise RuntimeError('live package import leaked: '+name)
    imported['frozen_runner'] = dict(path=str(Path(runner.__file__).resolve()), sha256=sha(runner.__file__))
    return load_primitives, frame_features, models, temporal, runner, imported


def array_binding(np, value):
    a = np.asarray(value)
    return dict(dtype=a.dtype.str, shape=list(a.shape),
                sha256=hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest())


def run_worker(args):
    import numpy as np
    import scipy
    tools = Path(__file__).resolve().parent; inputs = args.inputs.resolve()
    seal = verify_packet(tools, inputs)
    loader, frame_features, models, temporal, runner, imports = isolated_imports(tools, inputs)
    out = args.output/args.worker
    out.mkdir(parents=True, exist_ok=False)
    primitives = loader(inputs, args.worker)
    expected_features = json.loads((inputs/args.worker/'local_feature_array_bindings.json').read_text())
    expected_predictions = json.loads((tools/'reference_array_bindings.json').read_text())['cases'][args.worker]
    n = len(primitives['time_s'])
    source_context = dict(status='registered before replay inference', case=args.worker,
        source_seal_sha256=sha(tools/'replay_source_seal.json'), imports=imports,
        python=sys.version, executable=sys.executable, numpy=np.__version__, scipy=scipy.__version__,
        input_boundary='Compact whitelisted measured primitives only; no evaluation labels, terrain, desired trajectory or native family prior.',
        feature_cache_used=False, empty_package_initializer='Packaging-only empty initializer; original live __init__ is never imported.')
    save(out/'registered_before_replay.json', source_context)
    hashes = {key: hashlib.sha256() for key in expected_features}
    shapes, dtypes = {}, {}
    rows, evidence = [], []
    estimator = runner.ShadowInference(); started = time.monotonic()
    for i, t in enumerate(primitives['time_s']):
        frame = frame_features(primitives, i)  # deliberately UNCACHED
        for key in hashes:
            value = np.asarray(frame[key])
            hashes[key].update(np.ascontiguousarray(value).tobytes())
            shapes[key] = [n]+list(value.shape); dtypes[key] = value.dtype.str
        dt = .005 if i == 0 else float(t-primitives['time_s'][i-1])
        model = models.evaluate_frame(frame['observation9'], frame['points_m'], frame['force_normals'],
            frame['material_velocity_m_s'], frame['force_maps'], frame['family_index'],
            frame['valid'], frame['normal_hint'], frame['candidate_index'],
            dt_s=dt, observation_valid=frame['interval_valid'])
        row, ev = estimator.update(float(t), model); rows.append(row); evidence.append(ev)
        if i % 1000 == 0:
            print(json.dumps(dict(case=args.worker, frames=i+1, total=n, elapsed_s=time.monotonic()-started)), flush=True)
    computed_features = {key: dict(dtype=dtypes[key], shape=shapes[key], sha256=value.hexdigest())
                         for key, value in hashes.items()}
    feature_exact = computed_features == expected_features
    predictions = {key: np.asarray([row[key] for row in rows]) for key in rows[0]}
    ev_arrays = {key: np.asarray([row[key] for row in evidence]) for key in evidence[0]}
    np.savez_compressed(out/'predictions.npz', **predictions)
    np.savez_compressed(out/'model_evidence.npz', **ev_arrays)
    comparisons = {}
    for filename, arrays in (('predictions.npz', predictions), ('model_evidence.npz', ev_arrays)):
        actual = {key: array_binding(np, value) for key, value in arrays.items()}
        expected = expected_predictions[filename]['arrays']
        comparisons[filename] = dict(field_set_exact=set(actual)==set(expected),
            all_arrays_exact=actual == expected, actual=actual,
            differing_arrays=[key for key in set(actual)|set(expected) if actual.get(key) != expected.get(key)])
    verify_packet(tools, inputs)
    all_exact = bool(feature_exact and all(c['all_arrays_exact'] for c in comparisons.values()))
    report = dict(case=args.worker, frames=n, completed=True, all_exact=all_exact,
        cache_vs_fresh_all_feature_arrays_exact=feature_exact,
        feature_bindings=computed_features,
        differing_feature_arrays=[key for key in computed_features if computed_features[key] != expected_features[key]],
        output_comparisons=comparisons, imports=imports, elapsed_s=time.monotonic()-started,
        source_and_primitive_seals_unchanged=True, truth_read=False, physics_run=False,
        inference_api='Copied ShadowInference.update + copied evaluate_frame + copied frame_features(primitives,i) uncached.')
    save(out/'replay_report.json', report)
    print(json.dumps(dict(case=args.worker, completed=True, all_exact=all_exact, seconds=report['elapsed_s'])), flush=True)
    if not all_exact: raise RuntimeError('strict cached/fresh or full inference array parity failed')


def run_parent(args):
    tools = Path(__file__).resolve().parent
    verify_packet(tools, args.inputs.resolve())
    args.output.mkdir(parents=True, exist_ok=False)
    cases = (args.case,) if args.case else CASES
    children = []
    for case in cases:
        log = (args.output/(case+'.log')).open('x')
        command = [sys.executable, '-I', str(Path(__file__).resolve()), '--worker', case,
                   '--inputs', str(args.inputs.resolve()), '--output', str(args.output.resolve())]
        environment = os.environ.copy()
        for key in ('PYTHONPATH', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
                    'COLCON_PREFIX_PATH', 'LD_PRELOAD'):
            environment.pop(key, None)
        environment.update(OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=environment, cwd='/tmp')
        children.append((case, child, log, command))
    status = []
    for case, child, log, command in children:
        code = child.wait(); log.close()
        status.append(dict(case=case, returncode=code, command=command))
    reports = {case: json.loads((args.output/case/'replay_report.json').read_text())
               for case in cases if (args.output/case/'replay_report.json').exists()}
    save(args.output/'replay_summary.json', dict(children=status,
        all_cases_completed=len(reports)==len(cases),
        all_exact=len(reports)==len(cases) and all(r['all_exact'] for r in reports.values()),
        cases={case: dict(frames=r['frames'], all_exact=r['all_exact'], elapsed_s=r['elapsed_s'],
                cache_vs_fresh_all_feature_arrays_exact=r['cache_vs_fresh_all_feature_arrays_exact'],
                verified_prediction_arrays=sum(len(x['actual']) for x in r['output_comparisons'].values()))
               for case, r in reports.items()}))
    if any(x['returncode'] for x in status): raise RuntimeError('one or more isolated workers failed; inspect preserved logs')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--inputs', type=Path, default=Path(__file__).resolve().parents[1]/'portable_inputs')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--case', choices=CASES)
    p.add_argument('--worker', choices=CASES, help=argparse.SUPPRESS)
    args = p.parse_args()
    if args.worker: run_worker(args)
    else: run_parent(args)


if __name__ == '__main__': main()
