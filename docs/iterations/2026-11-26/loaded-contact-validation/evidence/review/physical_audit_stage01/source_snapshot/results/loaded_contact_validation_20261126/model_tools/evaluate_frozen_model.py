"""Reconstruct the frozen I56 measured-contact G at registered saved states.

This evaluator runs no physics, observer update, controller, or contact solver.
Predictions are sealed separately from any later native validation endpoints.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
from types import SimpleNamespace
from unittest.mock import patch

import mujoco
import numpy as np
import yaml

from spherical_wheel.contact_sensing import SupportLocator, measure_records
from spherical_wheel.handoff_model import MeasuredContactVelocityModel

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
SOURCE_RUN = ROOT/'results/constrained_handoff_20261126/campaign/runs/m100_H'
FROZEN_MODEL_SHA = '663a13b553c402adf3e1d795c34bfba7c5bd886471ef9c3cc36135e9ac08a559'
FROZEN_ADAPTER_SHA = 'd6bdcf77062b5c6dd060bd4c00f2cea3d9aa2bc773dda0e326313f4524e9ab84'
FIELDS = ('time_s', 'exact_qpos', 'exact_qvel', 'exact_applied', 'exact_previous_applied',
          'exact_native_applied', 'exact_observer_wrench', 'exact_input_bounds',
          'exact_estimated_support_loads_N', 'exact_estimated_contact_point_m',
          'exact_estimated_normal')
SUPPORTS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
INPUTS = ('shaft_torque_Nm', 'shell_A_torque_Nm', 'shell_B_torque_Nm',
          'belt_A_speed_m_s', 'belt_B_speed_m_s')
GENERALIZED = ('world_vx_m_s', 'world_vy_m_s', 'world_vz_m_s',
               'fork_wx_rad_s', 'fork_wy_rad_s', 'fork_wz_rad_s',
               'shaft_rate_rad_s', 'shell_A_rate_rad_s', 'shell_B_rate_rad_s')
PHYSICAL = GENERALIZED[:3] + ('world_wx_rad_s', 'world_wy_rad_s', 'world_wz_rad_s') + GENERALIZED[3:]


def read(path):
    return json.loads(Path(path).read_text())


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if isinstance(value, dict):
        return {key: safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(item) for item in value]
    return value


def physical_map(rotation):
    transform = np.zeros((12, 9))
    transform[:3, :3] = np.eye(3)
    transform[3:6, 3:6] = rotation
    transform[6:9, 3:6] = np.eye(3)
    transform[9:, 6:] = np.eye(3)
    return transform


def forbidden(*unused, **ignored):
    raise AssertionError('Frozen model evaluation attempted dynamics/observer/physics')


def impulse_proxy(response, command):
    impulse = (response.contact_impulse_offset + response.contact_impulse_response@command).reshape(-1, 3)
    residual = (response.constraint_offset + response.constraint_response@command).reshape(-1, 3)
    rows = []
    for name, load, normal, mu, vector, velocity in zip(response.contact_names,
            response.contact_loads_N, response.contact_normals_world,
            response.contact_friction, impulse, residual):
        normal_impulse = float(normal@vector)
        tangent_impulse = float(np.linalg.norm(vector-normal*normal_impulse))
        mu_value = float(mu) if np.isfinite(mu) else None
        positive_limit = mu*normal_impulse if normal_impulse > 0 and np.isfinite(mu) else 0.
        observed_load_limit = mu*load*response.horizon_s if np.isfinite(mu) else 0.
        rows.append(dict(name=name, estimated_normal_load_N=float(load), nominal_friction=mu_value,
            impulse_world_Ns=vector, equivalent_force_world_N=vector/response.horizon_s,
            normal_impulse_Ns=normal_impulse, tangential_impulse_Ns=tangent_impulse,
            normal_impulse_nonnegative=bool(normal_impulse >= 0),
            tangential_over_mu_positive_modeled_normal_impulse=tangent_impulse/positive_limit if positive_limit > 1e-15 else None,
            tangential_over_mu_estimated_load_times_h=tangent_impulse/observed_load_limit if observed_load_limit > 1e-15 else None,
            predicted_material_velocity_residual_world_m_s=velocity))
    return rows


def reconstruct(model, data, fields, index, config, provenance):
    data.qpos[:] = fields['exact_qpos'][index]
    data.qvel[:] = fields['exact_qvel'][index]
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    fork = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'fork'))
    for side, speed in zip(('A', 'B'), fields['exact_previous_applied'][index, 3:]):
        for name in (f'belt_{side}', f'belt_{side}_smooth'):
            geom = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))
            if geom >= 0:
                model.geom_surfacevel[geom, :3] = [0., speed, 0.]
    radius = float(config['geometry']['sphere_radius_m'])
    weight = float(model.body_subtreemass[fork])*np.linalg.norm(model.opt.gravity)
    minimum = float(provenance['parameters']['support_min_load_fraction'])*weight
    locator = SupportLocator(model, fork, radius, config['contact']['friction'][0], minimum)
    wrench = fields['exact_observer_wrench'][index]
    records = locator.records(data, wrench[:3], wrench[3:])
    measured = measure_records(SimpleNamespace(model=model, data=data, fork_id=fork), records)
    scale = float(provenance['candidate_settings']['slip_confidence_scale_m_s'])
    mismatch = float(measured['slip_speed_m_s'])
    confidence = 1./(1.+min(mismatch/scale, 1e6)**2)
    return records, measured, confidence


def validate_reconstruction(response, measured, confidence, fields, index, diagnostic):
    assert list(response.contact_names) == diagnostic['contact_names']
    assert response.constraint_rank == diagnostic['contact_model_rank']
    np.testing.assert_allclose(response.contact_loads_N, diagnostic['modeled_contact_loads_N'], atol=1e-9, rtol=0)
    np.testing.assert_allclose(confidence, diagnostic['tangent_confidence_proxy'], atol=1e-12, rtol=0)
    np.testing.assert_allclose(measured['slip_speed_m_s'], diagnostic['estimated_material_velocity_mismatch_m_s'], atol=1e-11, rtol=0)
    np.testing.assert_allclose([measured['loads_N'][name] for name in SUPPORTS], fields['exact_estimated_support_loads_N'][index], atol=1e-9, rtol=0)
    np.testing.assert_allclose(measured['point_m'], fields['exact_estimated_contact_point_m'][index], atol=1e-11, rtol=0)
    for label, key in [('native', 'exact_native_applied'), ('governed', 'exact_applied')]:
        command = fields[key][index]
        impulse = response.contact_impulse_offset + response.contact_impulse_response@command
        residual = np.linalg.norm(response.constraint_offset + response.constraint_response@command)
        np.testing.assert_allclose(impulse, diagnostic[label+'_soft_contact_impulse_Ns'], atol=1e-10, rtol=0)
        np.testing.assert_allclose(residual, diagnostic[label+'_contact_velocity_residual_norm_m_s'], atol=1e-11, rtol=0)
    return dict(contact_names_loads_rank_match=True, estimated_point_loads_match=True,
        slip_confidence_match=True, native_governed_impulse_and_residual_match=True,
        frozen_model_response_reconstruction_passed=True)


def evaluate_state(spec, fields, diagnostics, model, data, config, provenance, folder):
    instant = float(spec['time_s'])
    index = int(np.argmin(abs(fields['time_s']-instant)))
    assert abs(fields['time_s'][index]-instant) < 1e-8
    assert abs(diagnostics[index]['time_s']-instant) < 1e-8
    records, measured, confidence = reconstruct(model, data, fields, index, config, provenance)
    helper = MeasuredContactVelocityModel(model, contact_strength=provenance['candidate_settings']['contact_strength'])
    baseline = fields['exact_applied'][index]
    bounds = fields['exact_input_bounds'][index]
    arrays = dict(qpos=fields['exact_qpos'][index], qvel=fields['exact_qvel'][index],
                  baseline_command=baseline, input_bounds=bounds, previous_command=fields['exact_previous_applied'][index],
                  observer_wrench=fields['exact_observer_wrench'][index])
    report = dict(id=spec['id'], saved_time_s=float(fields['time_s'][index]), saved_index=index,
        tangent_confidence=confidence, estimated_slip_m_s=float(measured['slip_speed_m_s']),
        baseline_command=baseline, bounds=bounds, observer_records=records,
        filtered_estimator_normal_not_used_for_model=fields['exact_estimated_normal'][index], horizons={})
    for horizon in (.005, .02):
        key = f'h{round(horizon*1000):03d}ms'
        response = helper.evaluate(fields['exact_qpos'][index], fields['exact_qvel'][index], records,
            horizon_s=horizon, tangent_confidence=confidence)
        transform = physical_map(response.fork_rotation_world)
        if horizon == .02:
            report['original_20ms_reconstruction'] = validate_reconstruction(response, measured, confidence,
                fields, index, diagnostics[index]['diagnostics'])
        horizon_report = dict(horizon_s=horizon, contact_names=response.contact_names,
            contact_loads_N=response.contact_loads_N, loaded_response=response.response,
            free_response=response.free_response, loaded_physical_response=transform@response.response,
            free_physical_response=transform@response.free_response,
            offset=response.offset, free_offset=response.free_offset,
            contact_rank=response.constraint_rank, contact_normals_world=response.contact_normals_world,
            fixed_fork_rotation_world=response.fork_rotation_world,
            baseline_impulse_feasibility_proxies=impulse_proxy(response, baseline), perturbations=[])
        native = fields['exact_native_applied'][index]
        native_to_governed = baseline-native
        horizon_report['native_to_governed'] = dict(native_command=native,
            governed_command=baseline, delta_input=native_to_governed,
            loaded_delta_generalized_velocity=response.response@native_to_governed,
            free_delta_generalized_velocity=response.free_response@native_to_governed,
            loaded_delta_physical_velocity=transform@response.response@native_to_governed,
            free_delta_physical_velocity=transform@response.free_response@native_to_governed)
        for perturbation in spec.get('perturbations', []):
            delta = np.asarray(perturbation['delta_input'], dtype=float)
            assert delta.shape == (5,) and np.isfinite(delta).all()
            command = baseline+delta
            if np.any(abs(command) > bounds+1e-12):
                raise ValueError(f'{spec["id"]}/{perturbation["id"]}: perturbed command exceeds registered native bounds; no silent clipping')
            change, free_change = response.response@delta, response.free_response@delta
            horizon_report['perturbations'].append(dict(id=perturbation['id'], delta_input=delta,
                perturbed_command=command, loaded_delta_generalized_velocity=change,
                free_delta_generalized_velocity=free_change,
                loaded_delta_physical_velocity=transform@change,
                free_delta_physical_velocity=transform@free_change,
                loaded_affine_baseline_endpoint_velocity=response.predict(baseline),
                loaded_affine_perturbed_endpoint_velocity=response.predict(command),
                perturbed_impulse_feasibility_proxies=impulse_proxy(response, command)))
        report['horizons'][key] = horizon_report
        for name in ('response', 'free_response', 'offset', 'free_offset', 'mass_matrix', 'drive_matrix',
            'bias_force', 'passive_force', 'contact_jacobian', 'belt_surface_matrix',
            'contact_penalty_matrix_kg', 'contact_impulse_offset', 'contact_impulse_response',
            'constraint_offset', 'constraint_response', 'body_angular_velocity_maps_world',
            'body_momentum_maps_world', 'total_momentum_map_world', 'shell_axis_world'):
            arrays[key+'_'+name] = np.asarray(getattr(response, name))
        arrays[key+'_physical_map'] = transform
    folder.mkdir()
    np.savez_compressed(folder/'matrices.npz', **arrays)
    (folder/'predictions.json').write_text(json.dumps(safe(report), indent=2, allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--registration', type=Path, required=True)
    parser.add_argument('--stage', type=Path, required=True)
    args = parser.parse_args()
    if args.stage.exists():
        raise FileExistsError('Preserve previous prediction attempt')
    args.stage.mkdir(parents=True)
    registration = read(args.registration)
    assert registration.get('horizons_s', [.005, .02]) == [.005, .02]
    assert sha(ROOT/'src/spherical_wheel/handoff_model.py') == FROZEN_MODEL_SHA
    assert sha(ROOT/'results/constrained_handoff_20261126/campaign_tools/adapter.py') == FROZEN_ADAPTER_SHA
    provenance = read(SOURCE_RUN/'provenance.json')
    model_path = Path(provenance['models']['mirror_xml'])
    config_path = SOURCE_RUN/'source_snapshot/config/model.yaml'
    inputs = [Path(__file__), args.registration, model_path, config_path, SOURCE_RUN/'precision.npz',
        SOURCE_RUN/'governor_diagnostics.json', SOURCE_RUN/'provenance.json',
        ROOT/'src/spherical_wheel/handoff_model.py', ROOT/'src/spherical_wheel/contact_sensing.py',
        ROOT/'results/constrained_handoff_20261126/campaign_tools/adapter.py']
    hashes = {str(p.resolve().relative_to(ROOT)): sha(p) for p in inputs}
    declaration = dict(registered_before_prediction_utc=datetime.now(timezone.utc).isoformat(),
        source_run=str(SOURCE_RUN.relative_to(ROOT)), input_sha256=hashes,
        measurement_whitelist=FIELDS, excluded_precision_columns='All non-whitelisted fields, including all truth_* evaluation fields',
        reconstruction='Pure SupportLocator records from measured q/v and exact saved observer wrench; same prior belt speeds and slip/confidence heuristic as frozen adapter',
        horizons_s=[.005,.02], input_columns=INPUTS, generalized_velocity_rows=GENERALIZED,
        physical_velocity_rows=PHYSICAL, model_source_frozen=True, contact_strength=10.,
        no_simulator_or_observer_update=True, endpoint_interpretation='Frozen-geometry affine velocity sensitivity, not an integrated trajectory forecast')
    (args.stage/'declared_before_prediction.json').write_text(json.dumps(safe(declaration), indent=2)+'\n')
    snap = args.stage/'source_snapshot'
    snap.mkdir()
    for path in inputs:
        if path.suffix in ('.py', '.yaml', '.xml') or path == args.registration:
            target = snap/path.resolve().relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
    with np.load(SOURCE_RUN/'precision.npz', allow_pickle=False) as saved:
        fields = {key: saved[key].copy() for key in FIELDS}
    diagnostics = read(SOURCE_RUN/'governor_diagnostics.json')
    config = yaml.safe_load(config_path.read_text())
    reports = []
    with ExitStack() as guards:
        for name in ('mj_step', 'mj_step1', 'mj_step2', 'mj_forward', 'mj_inverse', 'mj_collision', 'mj_integratePos'):
            guards.enter_context(patch.object(mujoco, name, forbidden))
        # Static model terms are the only MuJoCo evaluation calls allowed.
        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)
        for state in registration['states']:
            reports.append(evaluate_state(state, fields, diagnostics, model, data, config, provenance, args.stage/state['id']))
    assert hashes == {str(p.resolve().relative_to(ROOT)): sha(p) for p in inputs}
    report = dict(iteration='I-57', completed_utc=datetime.now(timezone.utc).isoformat(),
        states=[dict(id=r['id'], saved_time_s=r['saved_time_s'], contact_names=r['horizons']['h020ms']['contact_names'],
            contact_loads_N=r['horizons']['h020ms']['contact_loads_N'], tangent_confidence=r['tangent_confidence'],
            reconstruction=r['original_20ms_reconstruction']) for r in reports],
        interpretation_notes=[
            'Loaded and free input columns share exactly the same saved state and rigid-body terms; free columns remove only the supplied contact penalty.',
            'Fork transverse angular changes are generalized velocity rows 3 and 4; physical world-Z rate is projected by the fixed initial fork rotation.',
            'All three relative encoder-rate components are retained; shell angular velocity is not identified with its relative encoder rate.',
            'At 20 ms, coefficients remain held at the initial geometry. There is no contact transition prediction, motor dynamics, unilateral impulse condition or friction-cone enforcement.',
            'Belt speeds are ideal prescribed surface velocities; their response columns are instantaneous penalties and do not scale linearly with horizon like torque columns.',
            'Contact strength 10 and slip-confidence scale .02 m/s are the frozen I56 model assumptions, not fitted physical contact compliance or measured friction.',
            'Affine offsets may imply infeasible impulses. Reported friction/load ratios are model diagnostics, not physical friction certificates.',
            'No later native endpoint was read in prediction. Scientific agreement must be assessed separately against preregistered native paired perturbations.'
        ], input_hashes_unchanged=True, dynamics_guard_active=True)
    (args.stage/'report.json').write_text(json.dumps(safe(report), indent=2, allow_nan=False)+'\n')
    files = [p for p in args.stage.rglob('*') if p.is_file() and p.name != 'artifact_manifest.json']
    (args.stage/'artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=str(p.relative_to(args.stage)),sha256=sha(p),bytes=p.stat().st_size) for p in sorted(files)]),indent=2)+'\n')
    print(json.dumps(safe(report), indent=2))


if __name__ == '__main__':
    main()
