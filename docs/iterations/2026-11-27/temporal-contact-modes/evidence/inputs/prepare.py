"""Prepare I-70 truth-free measured primitives and local reusable feature cache."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from input_features import frame_features, geometry

MEASUREMENT_KEYS = dict(time_s='time_s', qpos='exact_qpos', qvel='exact_qvel',
    previous_applied='exact_previous_applied', observer_interval_mean='exact_observer_interval_mean',
    observer_generalized='exact_observer_generalized', intrinsic_normal_hint='intrinsic_normal_hint')
POSE_KEYS = dict(centre_m='centre_m', patch_axis_world='patch_axis_world', pose_valid='valid',
                 radius_m='radius_m', patch_lower_m='patch_lower_m', patch_upper_m='patch_upper_m')
FEATURE_KEYS = dict(original_valid='valid', original_frame_valid='frame_valid',
                    original_excitation_m_s='excitation_m_s')
CASES = ('m100_B', 'p030_B', 'p090_B')


def primitives_from(measurement, pose, projected):
    """Explicit whitelists: no truth, terrain, desired trajectory or native family."""
    out = {dst: np.array(measurement[src], copy=True) for dst, src in MEASUREMENT_KEYS.items()}
    out.update({dst: np.array(pose[src], copy=True) for dst, src in POSE_KEYS.items()})
    out.update({dst: np.array(projected[src], copy=True) for dst, src in FEATURE_KEYS.items()})
    if not np.array_equal(out['time_s'], pose['time_s']):
        raise ValueError('measurement/pose clocks must be identical')
    n = len(out['time_s'])
    expected = dict(qpos=(n, 10), qvel=(n, 9), previous_applied=(n, 5),
        observer_interval_mean=(n, 9), observer_generalized=(n, 9), intrinsic_normal_hint=(n, 3),
        centre_m=(n, 3), patch_axis_world=(n, 4, 3), pose_valid=(n,),
        patch_lower_m=(4,), patch_upper_m=(4,), original_valid=(n, 102), original_frame_valid=(n,),
        original_excitation_m_s=(n,))
    for key, shape in expected.items():
        if out[key].shape != shape: raise ValueError(f'{key} shape mismatch')
    if out['radius_m'].shape != () or float(out['radius_m']) <= 0:
        raise ValueError('positive scalar radius required')
    if not np.array_equal(out['centre_m'], out['qpos'][:, :3]):
        raise ValueError('analytical coincident-origin map requires fork centre at free-joint origin')
    if not np.array_equal(projected['family_index'], np.tile([0, 1, 2, 3, 0, 1], 17)):
        raise ValueError('original bank family ordering changed')
    return out


def derive(primitives, projected, *, progress=None):
    n = len(primitives['time_s']); shape = (n, 102, 3)
    cache = {key: np.full(shape, np.nan) for key in
             ('points_m', 'force_normals', 'material_velocity_m_s', 'tangent_velocity_m_s')}
    cache.update(normal_angle_rad=np.full((n, 102), np.nan), valid=np.zeros((n, 102), bool))
    maxima = dict(velocity_residual_m_s=0., normal_angle_rad=0., moment_residual_norm=0.)
    for i in range(n):
        bank = geometry(primitives, i)
        if not np.array_equal(bank['original_geometry_valid'], primitives['original_valid'][i]):
            raise ValueError(f'original102 geometry validity differs at frame{i}')
        frame = frame_features(primitives, i, bank=bank)
        for key in cache: cache[key][i] = frame[key] if key in frame else frame['valid']
        active = primitives['original_valid'][i]
        if active.any():
            difference = float(np.max(abs(frame['tangent_velocity_m_s'][active]-projected['velocity_residual_m_s'][i, active])))
            maxima['velocity_residual_m_s'] = max(maxima['velocity_residual_m_s'], difference)
            difference = float(np.max(abs(frame['normal_angle_rad'][active]-projected['normal_angle_rad'][i, active])))
            maxima['normal_angle_rad'] = max(maxima['normal_angle_rad'], difference)
            force = primitives['observer_generalized'][i, :3]
            moment = primitives['observer_generalized'][i, 3:6]
            residual = moment-np.cross(frame['points_m'][active]-primitives['centre_m'][i], force)
            predicted = np.linalg.norm(residual, axis=1)/(float(primitives['radius_m'])*np.linalg.norm(force))
            maxima['moment_residual_norm'] = max(maxima['moment_residual_norm'],
                float(np.max(abs(predicted-projected['moment_residual_norm'][i, active]))))
        if progress and (i % 1000 == 0 or i == n-1): progress(i, n)
    if any(value > 1e-12 for value in maxima.values()):
        raise ValueError(f'original feature numerical parity exceeds1e-12: {maxima}')
    return cache, dict(frames=n, candidates_per_frame=102, original_valid_exact=True,
                       maximum_original_feature_difference=maxima,
                       original_feature_tolerance=1e-12, frame0_all_candidates_invalid=bool(not cache['valid'][0].any()))


def load_fields(path, keys):
    with np.load(path, allow_pickle=False) as packet:
        return {key: packet[key] for key in keys}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--measurement-root', type=Path, required=True)
    p.add_argument('--pose-root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--case', choices=CASES)
    args = p.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    for case in (args.case,) if args.case else CASES:
        measurement = args.measurement_root/case/'measurement_context.npz'
        projected_path = args.measurement_root/case/'projected_features.npz'
        pose_path = args.pose_root/case/'pose_maps.npz'
        sources = (measurement, projected_path, pose_path)
        projected_keys = set(FEATURE_KEYS.values()) | {'family_index', 'velocity_residual_m_s', 'normal_angle_rad', 'moment_residual_norm'}
        m = load_fields(measurement, MEASUREMENT_KEYS.values())
        pose = load_fields(pose_path, set(POSE_KEYS.values()) | {'time_s'})
        projected = load_fields(projected_path, projected_keys)
        primitives = primitives_from(m, pose, projected)
        folder = args.output/case; folder.mkdir()
        np.savez_compressed(folder/'primitives.npz', **primitives)
        cache, parity = derive(primitives, projected,
             progress=lambda i, n: print(f'{case}: {i+1}/{n}', flush=True))
        np.savez_compressed(folder/'features.npz', **cache)
        metadata = dict(case=case, status='truth-free endpoint feature generation; no physics',
            input_sha256={str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources},
            source_sha256={name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
                           for name in ('prepare.py', 'input_features.py', 'bank_points.py')},
            read_measurement_fields=list(MEASUREMENT_KEYS.values()), read_pose_fields=list(POSE_KEYS.values()),
            no_truth_fields_read=True, no_native_family_prior=True, parity=parity,
            force_map_convention='Endpoint native J(q_t,p)^T worldforce; rotateangularrows with observerRmid(q_previous,q_current). Approximation, not substep contact history.',
            belt_convention='Held previous applied speed is actual surface speed only under existing ideal prescribed-belt simulation model.',
            frame0='No preceding interval; candidates invalid and method must chooseunknown.')
        (folder/'metadata.json').write_text(json.dumps(metadata, indent=2, sort_keys=True)+'\n')
        manifest = {path.name: dict(bytes=path.stat().st_size, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                    for path in folder.iterdir() if path.is_file()}
        (folder/'artifact_manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True)+'\n')
        print(json.dumps(dict(case=case, **parity)), flush=True)


if __name__ == '__main__': main()
