"""I-60: truth-blind, zero-physics contact-candidate feature extraction.

Uses exact recorded state, previous input, estimated wrench and an inferred
normal only. Historical solver labels are not loaded by this process.
"""
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch
import argparse
import hashlib
import importlib.util
import inspect
import json
import shutil
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
CASES = ('m100_B', 'p030_B', 'p090_B')
ALLOWED_FIELDS = ('time_s', 'exact_qpos', 'exact_qvel', 'exact_previous_applied',
                  'exact_observer_wrench', 'exact_observer_generalized',
                  'exact_observer_interval_mean', 'exact_estimated_normal',
                  'exact_estimated_support_loads_N', 'exact_estimated_contact_point_m')
MODES = ('geometry', 'wrench', 'velocity', 'fused')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


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
    return value


def save(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def module(path):
    spec = importlib.util.spec_from_file_location('_i60_contact_hypotheses', path)
    value = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = value
    spec.loader.exec_module(value)
    return value


def flatten_scores(scores):
    result = {}
    for mode in MODES:
        if mode not in scores:
            raise ValueError('Missing fixed ranking arm: '+mode)
        for key, value in scores[mode].items():
            array = np.asarray(value)
            if array.dtype.kind == 'O':
                raise TypeError('Object result cannot be portable: '+mode+'/'+key)
            result[mode+'__'+key] = array
    return result


def run(protocol_path, case, output):
    if case not in CASES:
        raise ValueError('Only the three preselected native B recordings are allowed')
    protocol = json.loads(protocol_path.read_text())
    if protocol.get('iteration') != 'I-60':
        raise ValueError('Expected a sealed I-60 protocol')
    seal_path = protocol_path.parent/'input_source_seal.json'
    seal = json.loads(seal_path.read_text())
    for item in seal['sources']+seal['inputs']:
        original_path = Path(item['path'])
        if not original_path.is_absolute():
            original_path = ROOT/original_path
        if sha(original_path) != item['sha256']:
            raise ValueError('Root registration changed: '+str(original_path))
    clarification = json.loads((protocol_path.parent/'window_clarification_before_execution.json').read_text())
    if sha(protocol_path) != clarification['protocol_sha256']:
        raise ValueError('Root protocol differs from its pre-execution seal')
    feature_options = dict(cone_rad=np.deg2rad(protocol['candidate_search']['normal_cone_deg']),
        minimum_force_N=protocol['candidate_search']['minimum_compressive_force_N'])
    rank_options = dict(velocity_scale_m_s=protocol['scales']['velocity_m_s'],
        normal_scale_rad=np.deg2rad(protocol['scales']['normal_angle_deg']),
        minimum_excitation_m_s=protocol['confidence']['minimum_excitation_m_s'],
        minimum_margin=protocol['confidence']['minimum_runnerup_score_gap'],
        maximum_residual_m_s=protocol['confidence']['maximum_best_velocity_residual_m_s'],
        minimum_separation_m_s=protocol['confidence']['minimum_best_runnerup_velocity_vector_separation_m_s'])
    output.mkdir(parents=True, exist_ok=False)
    source = output/'source'
    source.mkdir()
    original = ROOT/'results/balance_priority_20261126/campaign/runs'/case
    precision, provenance_path = original/'precision.npz', original/'provenance.json'
    provenance = json.loads(provenance_path.read_text())
    if provenance['case']['variant'] != 'native':
        raise ValueError('This feature audit is restricted to native B states')
    xml = Path(provenance['models']['mirror_xml'])
    config = original/'source_snapshot/config/model.yaml'
    candidate_source = ROOT/'src/spherical_wheel/contact_hypotheses.py'
    intrinsic_source = ROOT/'src/spherical_wheel/contact_sensing.py'
    bound = [precision, provenance_path, xml, config, candidate_source,
             intrinsic_source, Path(__file__).resolve(), protocol_path]
    for path in bound:
        if not path.is_file():
            raise FileNotFoundError(path)
    registration = dict(iteration='I-60', case=case,
        registered_before_feature_arithmetic_utc=datetime.now(timezone.utc).isoformat(),
        inputs=[dict(path=str(path), sha256=sha(path)) for path in bound],
        allowed_precision_fields=ALLOWED_FIELDS,
        truth_fields_accessed=[], new_physics_steps=0,
        feature_options=feature_options, rank_options=rank_options,
        measurement_boundary='Exact simulated q/qdot, known prior applied inputs, observer wrench; normal domain is observer-conditioned. No solved contact or true terrain input.',
        historical_truth_clock='Not read here. Evaluator must distinguish latest 1 ms physics truth from preceding 5 ms observer interval and filtering.')
    save(output/'registered_before_arithmetic.json', registration)
    for path, name in ((Path(__file__), 'extract_features.py'),
                       (candidate_source, 'contact_hypotheses.py'),
                       (intrinsic_source, 'contact_sensing.py')):
        shutil.copyfile(path, source/name)
    shutil.copyfile(config, source/'model.yaml')
    shutil.copyfile(protocol_path, source/'protocol.json')
    with np.load(precision, allow_pickle=False) as data:
        raw = {name: data[name].copy() for name in ALLOWED_FIELDS}
    t = raw['time_s']
    if (len(t) != 8801 or abs(t[0]) > 1e-12 or abs(t[-1]-44) > 1e-8
            or not np.all(abs(np.diff(t)-.005) < 1e-9)):
        raise ValueError('Expected complete 44 s, 5 ms native history')
    expected_shapes = {'exact_qpos': (len(t), 10), 'exact_qvel': (len(t), 9),
        'exact_previous_applied': (len(t), 5), 'exact_observer_wrench': (len(t), 6),
        'exact_estimated_support_loads_N': (len(t), 4)}
    if any(raw[key].shape != shape for key, shape in expected_shapes.items()):
        raise ValueError('Unexpected recorded measurement shape')
    import mujoco
    from spherical_wheel.contact_sensing import intrinsic_normal
    if Path(inspect.getsourcefile(intrinsic_normal)).resolve() != intrinsic_source.resolve():
        raise RuntimeError('Imported intrinsic_normal is not the bound repository source')
    candidate = module(source/'contact_hypotheses.py')
    model = mujoco.MjModel.from_xml_path(str(xml))
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    feature_model = candidate.ContactHypothesisFeatures(model, **feature_options)
    radius = float(feature_model.radius)
    all_rows, normal_hints = [], []
    started = time.monotonic()
    def forbidden(*args, **kwargs):
        raise RuntimeError('I-60 feature extraction must not step physics or query solved contacts')
    with ExitStack() as guards:
        for name in ('mj_step', 'mj_step1', 'mj_step2', 'mj_collision', 'mj_contactForce'):
            guards.enter_context(patch.object(mujoco, name, side_effect=forbidden))
        for i in range(len(t)):
            wrench = raw['exact_observer_wrench'][i]
            hint = intrinsic_normal(wrench[:3], wrench[3:], radius)
            if hint is None:
                hint = np.full(3, np.nan)
            normal_hints.append(np.asarray(hint, float).copy())
            arguments = (raw['exact_qpos'][i], raw['exact_qvel'][i],
                raw['exact_previous_applied'][i], hint, wrench)
            original_arguments = [value.copy() for value in arguments]
            value = feature_model.evaluate(*arguments)
            if not all(np.array_equal(a, b, equal_nan=True)
                       for a, b in zip(arguments, original_arguments)):
                raise RuntimeError('Candidate feature generation mutated a measured input')
            projected = candidate.project_features(value)
            expected_family = np.tile(np.array([0, 1, 2, 3, 0, 1]), 17)
            expected_kind = np.tile(np.array([0, 0, 0, 0, 1, 1]), 17)
            if (np.asarray(projected['valid']).shape != (102,)
                    or not np.array_equal(projected['family_index'], expected_family)
                    or not np.array_equal(feature_model.kind, expected_kind)
                    or np.asarray(projected['frame_valid']).shape != ()):
                raise RuntimeError('Fixed candidate frame/schema contract failed')
            valid = np.asarray(projected['valid'], bool)
            for key in ('normal_angle_rad', 'velocity_residual_norm_m_s',
                        'moment_residual_norm', 'velocity_residual_m_s'):
                if not np.isfinite(np.asarray(projected[key])[valid]).all():
                    raise RuntimeError('A valid candidate has nonfinite features: '+key)
            all_rows.append(projected)
            if (i+1) % 1000 == 0:
                print(json.dumps(dict(case=case, rows=i+1,
                    elapsed_s=round(time.monotonic()-started, 3))), flush=True)
    keys = set(all_rows[0])
    if any(set(row) != keys for row in all_rows):
        raise ValueError('Feature projection schema changed within trace')
    features = {}
    for key in sorted(keys):
        values = [np.asarray(row[key]) for row in all_rows]
        if key in ('family_index', 'kind'):
            if not all(np.array_equal(values[0], value) for value in values):
                raise ValueError('Candidate metadata changed: '+key)
            features[key] = values[0]
        else:
            features[key] = np.stack(values)
    features['kind'] = np.asarray(feature_model.kind).copy()
    feature_path = output/'projected_features.npz'
    np.savez_compressed(feature_path, **features)
    context_path = output/'measurement_context.npz'
    measured_excitation = (np.linalg.norm(raw['exact_qvel'][:, :3], axis=1)
        +radius*np.linalg.norm(raw['exact_qvel'][:, 3:6], axis=1)
        +radius*np.linalg.norm(raw['exact_qvel'][:, 6:9], axis=1)
        +np.linalg.norm(raw['exact_previous_applied'][:, 3:], axis=1))
    np.savez_compressed(context_path, **raw, intrinsic_normal_hint=np.stack(normal_hints),
        measured_excitation_m_s=measured_excitation)
    scores = candidate.rank_hypotheses(features, **rank_options)
    score_path = output/'scores.npz'
    np.savez_compressed(score_path, **flatten_scores(scores))
    save(output/'metadata.json', dict(iteration='I-60', case=case, rows=len(t),
        candidate_count=int(len(features['family_index'])), model_xml_sha256=sha(xml),
        model_dimensions=dict(nq=int(model.nq), nv=int(model.nv), ngeom=int(model.ngeom)),
        radius_m=radius, feature_shapes={key:list(value.shape) for key,value in features.items()},
        source_model_recipe='Load the archived I-58 mirror XML directly; disable contact; private kinematics/Jacobians only.',
        no_truth_fields_loaded=True, no_physics_steps=True, live_controller_unchanged=True,
        nonfinite_input_entries={key:int((~np.isfinite(value)).sum()) for key,value in raw.items()},
        elapsed_s=time.monotonic()-started))
    for entry in registration['inputs']:
        if sha(entry['path']) != entry['sha256']:
            raise ValueError('A bound source/input changed: '+entry['path'])
    files = [path for path in output.rglob('*') if path.is_file() and '__pycache__' not in path.parts]
    save(output/'artifact_manifest.json', dict(iteration='I-60',
        sealed_before_any_truth_read_utc=datetime.now(timezone.utc).isoformat(),
        files=[dict(path=str(path.relative_to(output)), sha256=sha(path), bytes=path.stat().st_size)
               for path in sorted(files)]))
    print(json.dumps(dict(case=case, rows=len(t), scores_sealed=True,
        no_truth_read=True, new_physics_steps=0)), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--case', choices=CASES, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.protocol.resolve(), args.case, args.output.resolve())
