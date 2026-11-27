"""Post-primary I57 saved-array baseline audit. NumPy only; no new physics."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
STUDY = ROOT/'results/loaded_contact_validation_20261126'
GROUPS = {'planar': (0, 1), 'transverse': (3, 4), 'world_yaw': (9,),
          'shaft': (6,), 'shell_A': (7,), 'shell_B': (8,)}
ROWS = ('world_vx_m_s', 'world_vy_m_s', 'world_vz_m_s',
        'initial_fork_wx_rad_s', 'initial_fork_wy_rad_s', 'initial_fork_wz_rad_s',
        'shaft_rate_rad_s', 'shell_A_rate_rad_s', 'shell_B_rate_rad_s', 'world_wz_rad_s')


def read(path):
    return json.loads(path.read_text())


def load(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {key: safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def rotation(q):
    w, x, y, z = q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def physical(qpos, qvel, R0):
    world_omega = rotation(qpos[3:7])@qvel[3:6]
    return np.r_[qvel[:3], R0.T@world_omega, qvel[6:], world_omega[2]]


def frozen_physical(qvel, R0):
    return np.r_[qvel, (R0@qvel[3:6])[2]]


def main():
    destination = HERE/'report.json'
    if destination.exists() or (HERE/'registered_before_arithmetic.json').exists():
        raise FileExistsError('Preserve earlier exploratory arithmetic')
    plan = read(HERE/'plan.json')
    protocol = read(STUDY/'protocol.json')
    inputs = [Path(__file__), HERE/'plan.json', STUDY/'protocol.json']
    for event in plan['events']:
        for branch in plan['branches']:
            folder = STUDY/'campaign/runs'/(event+'_'+branch)
            inputs.extend(folder/name for name in ('endpoints.npz', 'provenance.json', 'artifact_manifest.json'))
        inputs.extend(STUDY/'model_tools/predictions_stage01'/event/name for name in ('matrices.npz', 'predictions.json'))
    hashes = {str(path.relative_to(ROOT)): sha(path) for path in inputs}
    registration = dict(registered_before_arithmetic_utc=datetime.now(timezone.utc).isoformat(),
        input_sha256=hashes, plan=plan, outcome_awareness='Primary differential-response agreement was already known. These absolute-baseline computations had not yet been performed.',
        physical_rows=ROWS, groups=GROUPS, no_new_physics=True)
    (HERE/'registered_before_arithmetic.json').write_text(json.dumps(registration, indent=2)+'\n')
    report = dict(iteration='I-57', scope=plan['scope'], physical_rows=ROWS, events={},
        no_new_physics=True, no_new_acceptance_gates=True, no_fitting=True)
    for event in plan['events']:
        native_folder = STUDY/'campaign/runs'/(event+'_native')
        governed_folder = STUDY/'campaign/runs'/(event+'_governed')
        for folder in (native_folder, governed_folder):
            assert read(folder/'provenance.json')['success']
            manifest = read(folder/'artifact_manifest.json')
            for name in ('endpoints.npz', 'provenance.json'):
                assert sha(folder/name) == next(item['sha256'] for item in manifest['files'] if item['path'] == name)
        native, governed = load(native_folder/'endpoints.npz'), load(governed_folder/'endpoints.npz')
        model = load(STUDY/'model_tools/predictions_stage01'/event/'matrices.npz')
        R0 = rotation(model['qpos'][3:7])
        current = frozen_physical(model['qvel'], R0)
        un, ug = native['command_sequence'][0], governed['command_sequence'][0]
        assert np.all(native['command_sequence'] == un) and np.all(governed['command_sequence'] == ug)
        np.testing.assert_array_equal(native['exact_qpos'][0], model['qpos'])
        np.testing.assert_array_equal(governed['exact_qpos'][0], model['qpos'])
        event_report = dict(current_velocity=current, held_native_command=un, held_governed_command=ug,
            delta_input=ug-un, horizons={})
        for index, key in ((1, 'h005ms'), (4, 'h020ms')):
            response, offset = model[key+'_response'], model[key+'_offset']
            actual_native = physical(native['exact_qpos'][index], native['exact_qvel'][index], R0)
            actual_governed = physical(governed['exact_qpos'][index], governed['exact_qvel'][index], R0)
            differential_prediction = frozen_physical(response@(ug-un), R0)
            proxy_native, proxy_governed = current, current+differential_prediction
            affine_native = frozen_physical(offset+response@un, R0)
            affine_governed = frozen_physical(offset+response@ug, R0)
            omitted_native_drift = actual_native-current
            observed_delta = actual_governed-actual_native
            differential_error = observed_delta-differential_prediction
            governed_proxy_error = actual_governed-proxy_governed
            native_affine_error = actual_native-affine_native
            governed_affine_error = actual_governed-affine_governed
            proxy_residual = governed_proxy_error-(omitted_native_drift+differential_error)
            affine_residual = governed_affine_error-(native_affine_error+differential_error)
            np.testing.assert_allclose(proxy_residual, 0, atol=1e-12, rtol=0)
            np.testing.assert_allclose(affine_residual, 0, atol=1e-12, rtol=0)
            vectors = dict(actual_native=actual_native, actual_governed=actual_governed,
                current_proxy_native=proxy_native, current_proxy_governed=proxy_governed,
                frozen_affine_native=affine_native, frozen_affine_governed=affine_governed,
                omitted_native_baseline_drift=omitted_native_drift,
                observed_governed_minus_native=observed_delta, predicted_G_delta=differential_prediction,
                paired_G_delta_error=differential_error, governed_current_proxy_error=governed_proxy_error,
                native_affine_error=native_affine_error, governed_affine_error=governed_affine_error,
                proxy_decomposition_residual=proxy_residual, affine_decomposition_residual=affine_residual)
            groups = {}
            for name, indices in GROUPS.items():
                groups[name] = dict(vectors={label: vector[list(indices)] for label, vector in vectors.items()},
                    norms={label: float(np.linalg.norm(vector[list(indices)])) for label, vector in vectors.items()})
            event_report['horizons'][key] = dict(horizon_s=index*.005, vectors=vectors, groups=groups)
        report['events'][event] = event_report
    assert hashes == {str(path.relative_to(ROOT)): sha(path) for path in inputs}
    report['input_hashes_unchanged'] = True
    report['finished_utc'] = datetime.now(timezone.utc).isoformat()
    report['interpretation_limits'] = [
        'This post-primary exploratory audit was selected after differential model agreement was known; it is not a prespecified confirmation.',
        'The current-plus-G-delta proxy omits the held-native future change. The exact decomposition is algebraic and establishes neither optimal weights nor a stable replacement controller.',
        'Frozen affine offsets are evaluated diagnostically. Soft contact strength/confidence and held geometry do not enforce unilateral/friction feasibility or establish predictive offset accuracy.',
        'Two selected open-loop states do not identify the cause of the full I56 trajectory deviation or justify model/controller adoption.'
    ]
    destination.write_text(json.dumps(safe(report), indent=2, allow_nan=False)+'\n')
    (HERE/'artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=path.name, sha256=sha(path), bytes=path.stat().st_size) for path in sorted(HERE.iterdir()) if path.is_file() and path.name != 'artifact_manifest.json']), indent=2)+'\n')
    print(json.dumps({event: {key: {name: values['norms'] for name, values in horizon['groups'].items() if name in ('planar', 'transverse', 'world_yaw')} for key, horizon in data['horizons'].items()} for event, data in safe(report)['events'].items()}, indent=2))


if __name__ == '__main__':
    main()
