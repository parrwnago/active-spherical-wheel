"""I77 native rolling-map reconstruction from recorded measurements only.

No controller command, physics integration, contact solver or terrain query is
used. A contact-disabled StateMirror evaluates known frames/Jacobians. Frozen
observer wrenches reproduce stateless SupportLocator records. The sequential
stationary/allocation/dropout-held policy reproduces the raw rolling references.
Positive-pressure solver aggregates are instantaneous last-solve diagnostics,
not complete5ms wrench truth or physically executed causal interventions.
"""
from contextlib import ExitStack
from datetime import datetime, timezone
from importlib.metadata import version
from pathlib import Path
from unittest.mock import patch
import argparse, hashlib, json, shutil, sys, traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]
OLD = ROOT/'results/tangent_heading_routes_20261127'
CASES = ('straight_m100_N', 'straight_p090_N')
FAMILIES = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
SCALE = np.array([5., 5., 5., .3, .3])
PROTOCOL_SHA = '53106cdb758d76436e04b747c9483c4081abcc7ec1f2a163ace90b58b82085c2'
SEAL_SHA = 'e5e65236947d7b1945c90b3a74a133f055495851d1938b42dacd0ac26602fc2a'
TIMES = {'straight_m100_N': [18., 19., 20., 20.585, 21.040, 21.250, 22., 22.635],
         'straight_p090_N': [5.8, 18., 19., 20., 20.585, 20.775, 21.040, 21.250, 21.830, 22., 22.635]}
FIELDS = ('time_s', 'exact_qpos', 'exact_qvel', 'exact_previous_applied',
          'exact_reference_velocity_m_s', 'exact_reference_qvel', 'exact_rolling_feedforward',
          'exact_allocation_drive_reference', 'exact_observer_wrench', 'exact_estimated_normal',
          'exact_estimated_contact_point_m', 'exact_estimated_support_loads_N',
          'truth_patch_loads_N', 'truth_patch_points_m', 'truth_equivalent_point_m')


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def safe(v):
    if isinstance(v, dict): return {str(k): safe(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)): return [safe(x) for x in v]
    if isinstance(v, np.ndarray): return safe(v.tolist())
    if isinstance(v, np.generic): return safe(v.item())
    if isinstance(v, float) and not np.isfinite(v): return None
    return v

def write(path, value):
    with path.open('x') as f:
        json.dump(safe(value), f, indent=2, sort_keys=True, allow_nan=False); f.write('\n')


def register(stage):
    stage.mkdir(parents=True, exist_ok=False)
    protocol_path, seal_path = OLD/'protocol/protocol.json', OLD/'campaign/outputs_sealed.json'
    if sha(protocol_path) != PROTOCOL_SHA or sha(seal_path) != SEAL_SHA:
        raise ValueError('Frozen I76 baseline release differs')
    protocol = json.loads(protocol_path.read_text())
    for group in ('source_sha256', 'input_sha256'):
        for relative, digest in protocol[group].items():
            if sha(ROOT/relative) != digest: raise ValueError('Original source/input changed: '+relative)
    seal = json.loads(seal_path.read_text())
    if not seal['all_processes_successful'] or any(x['returncode'] != 0 for x in seal['processes']):
        raise ValueError('All18 recorded successful exits required')
    for relative, item in seal['files'].items():
        p = OLD/relative
        if p.stat().st_size != item['bytes'] or sha(p) != item['sha256']:
            raise ValueError('Original sealed output changed: '+relative)
    sources = sorted((ROOT/'src/spherical_wheel').rglob('*.py'))+[ROOT/'config/model.yaml', Path(__file__).resolve()]
    inputs = [protocol_path, seal_path, BASE/'timeline/events_stage01/report.json', BASE/'timeline/events_stage01/artifact_manifest.json']
    for case in CASES:
        inputs += [OLD/'campaign/runs'/case/name for name in ('precision.npz', 'provenance.json', 'summary.json')]
    for p in sources:
        q = stage/'source'/p.relative_to(ROOT); q.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(p, q)
    files = sorted(set(sources+inputs))
    write(stage/'registered_before_calculation.json', dict(
        iteration='I-77', utc=datetime.now(timezone.utc).isoformat(),
        files={str(p.relative_to(ROOT)): dict(bytes=p.stat().st_size, sha256=sha(p)) for p in files},
        cases=CASES, fixed_summary_times_s=TIMES, full_record_reconstruction=True,
        native_map='WorldXY contact Jacobian rows, unweighted perrecord; scaled columns[5,5,5,.3,.3], np.linalg.lstsq rcond1e-7. These scales are preferences, not motor limits.',
        no_physics_integration=True, no_controller_commands=True, no_terrain_queries=True,
        state_boundary='Recorded simulated q/qvel/previousapplied plus frozen observer wrench, known model geometry. No observer rerun or simulator contact query.',
        history='Reproduce stationaryzero, movingallocation and no-touching dropout-held raw vref/feedforward; update held references only when currenttouching.',
        algebraic_oracles=['Instantaneous positive-pressure family-aggregate points with known family bodies/geoms.', 'Same inferred family records with family-aggregate pressure points, available only if each inferred family has a positive-pressure point.'],
        oracle_limitation='Not actual commanded interventions or complete contact manifolds. Last1ms positivepressure labels are not full5ms wrench truth; zero-normal tangential forces and intra-interval transfers remain I68/I69 caveats.',
        interpretation='Matrix conditioning/target projection can identify an algebraic source of large references. It does not by itself establish the complete physical cause of path deviation.'))


def verify(stage):
    reg = json.loads((stage/'registered_before_calculation.json').read_text())
    for rel, b in reg['files'].items():
        p = ROOT/rel
        if p.stat().st_size != b['bytes'] or sha(p) != b['sha256']: raise ValueError('Registered bytes changed: '+rel)
    return reg


def family(name): return next((i for i, v in enumerate(FAMILIES) if name.startswith(v)), None)

def build_map(mirror, records, velocity, mujoco):
    rows, rhs, details, jacobians = [], [], [], []
    data, model = mirror.data, mirror.model
    joint_ids = [int(np.flatnonzero(model.jnt_dofadr == i)[0]) for i in (6, 7, 8)]
    for record in records:
        if family(record['name']) is None: continue
        jac = np.zeros((3, model.nv)); mujoco.mj_jac(model, data, jac, None, record['pos'], int(record['body']))
        row = np.zeros((2, 5)); row[:, :3] = jac[:2, 6:9]
        if record['name'].startswith('belt_'):
            side = 0 if record['name'].startswith('belt_A') else 1
            row[:, 3+side] = data.geom_xmat[int(record['geom'])].reshape(3, 3)[:2, 1]
        rows.extend(row); rhs.extend(-velocity); jacobians.append(jac)
        ancestry = set(); body = int(record['body'])
        while body:
            ancestry.add(body); body = int(model.body_parentid[body])
        analytic = np.column_stack([np.cross(data.xaxis[j], np.asarray(record['pos'])-data.xanchor[j])
                                   if int(model.jnt_bodyid[j]) in ancestry else np.zeros(3) for j in joint_ids])
        active = np.flatnonzero(np.linalg.norm(row, axis=0) > 1e-14)
        angles = []
        for a in range(len(active)):
            for b in range(a+1, len(active)):
                x, y = row[:, active[a]], row[:, active[b]]
                cosine = float(np.clip(x@y/(np.linalg.norm(x)*np.linalg.norm(y)), -1., 1.))
                angles.append(dict(columns=[int(active[a]), int(active[b])], directed_angle_deg=float(np.degrees(np.arccos(cosine))),
                                   acute_collinearity_angle_deg=float(np.degrees(np.arccos(abs(cosine))))))
        details.append(dict(family=FAMILIES[family(record['name'])], body=int(record['body']), geom=int(record['geom']),
                            point_m=np.asarray(record['pos']), load_N=float(record.get('normal_force', 0.)),
                            normal=np.asarray(record.get('normal', np.full(3, np.nan))), row=row,
                            joint_horizontal_lever_m_per_rad=np.linalg.norm(row[:, :3], axis=0),
                            belt_direction_horizontal_norm_per_linear_speed=np.linalg.norm(row[:, 3:], axis=0),
                            active_columns=active, active_direction_angles=angles,
                            independent_axis_cross_lever_jacobian_max_error=float(np.max(abs(analytic-jac[:, 6:9])))))
    return np.asarray(rows, float).reshape(-1, 5), np.asarray(rhs, float), details, jacobians


def solve_map(matrix, target):
    if not len(matrix):
        return dict(allocated=np.zeros(5), rank=0, singular_values=np.empty(0), matrix=matrix, scaled_matrix=matrix.copy(), target=target,
                    condition_resolved=np.nan, cutoff=np.nan, residual_norm=np.nan, target_modal_projection=np.empty(0),
                    scaled_reference_modal_coefficients=np.empty(0), normalized_reference_norm=0., left=np.empty((0, 0)), right=np.empty((0, 5)))
    scaled = matrix*SCALE
    z, _, rank, values = np.linalg.lstsq(scaled, target, rcond=1e-7)
    left, singular, right = np.linalg.svd(scaled, full_matrices=False)
    projection = left.T@target
    cutoff = 1e-7*values[0]
    coefficients = np.divide(projection, singular, out=np.zeros_like(projection), where=singular>cutoff)
    return dict(allocated=SCALE*z, rank=int(rank), singular_values=values, matrix=matrix, scaled_matrix=scaled, target=target,
                condition_resolved=float(values[0]/values[rank-1]) if rank else np.nan, cutoff=float(cutoff),
                residual_norm=float(np.linalg.norm(matrix@(SCALE*z)-target)), target_modal_projection=projection,
                scaled_reference_modal_coefficients=coefficients, normalized_reference_norm=float(np.linalg.norm(z)), left=left, right=right)


def fixed_arrays(n):
    return dict(time_s=np.zeros(n), record_count=np.zeros(n, int), branch_code=np.zeros(n, int),
        native_rank=np.zeros(n, int), native_singular_values=np.full((n, 5), np.nan),
        native_condition_resolved=np.full(n, np.nan), native_cutoff=np.full(n, np.nan), native_residual_norm=np.full(n, np.nan),
        native_target_modal_projection=np.full((n, 5), np.nan), native_scaled_reference_modal_coefficients=np.full((n, 5), np.nan),
        native_scaled_reference_norm=np.zeros(n), native_matrix=np.full((n, 8, 5), np.nan), native_target=np.full((n, 8), np.nan),
        native_reference=np.zeros((n, 5)), logged_reference=np.zeros((n, 5)), reconstruction_max_error=np.zeros(n),
        native_family_loads_N=np.zeros((n, 4)), native_family_points_m=np.full((n, 4, 3), np.nan),
        native_minimum_acute_direction_angle_deg=np.full(n, np.nan), native_axis_cross_jacobian_error=np.zeros(n),
        intrinsic_normal=np.full((n, 3), np.nan), filtered_normal=np.zeros((n, 3)), raw_filtered_normal_angle_deg=np.full(n, np.nan),
        current_no_slip_xy_rms_m_s=np.full(n, np.nan), current_no_slip_tangent_rms_m_s=np.full(n, np.nan),
        measured_base_rotation_contact_xy_rms_m_s=np.full(n, np.nan),
        truth_positive_family_mask=np.zeros((n, 4), bool), truth_five_percent_family_mask=np.zeros((n, 4), bool),
        inferred_family_mask=np.zeros((n, 4), bool), dominant_positive_pressure_family=np.full(n, -1, int),
        dominant_inferred_family=np.full(n, -1, int), instantaneous_positive_pressure_family_set_agreement=np.zeros(n, bool),
        estimated_to_pressure_equivalent_point_error_m=np.full(n, np.nan),
        truth_aggregate_reference=np.zeros((n, 5)), truth_aggregate_rank=np.zeros(n, int),
        truth_aggregate_singular_values=np.full((n, 5), np.nan), truth_aggregate_condition_resolved=np.full(n, np.nan),
        truth_aggregate_residual_norm=np.full(n, np.nan), truth_aggregate_record_count=np.zeros(n, int),
        same_family_point_oracle_available=np.zeros(n, bool), same_family_point_reference=np.full((n, 5), np.nan),
        same_family_point_condition_resolved=np.full(n, np.nan))


def analyze_case(case, stage, mirror, locator, params, mujoco, rolling_feedforward_records, measure_records):
    with np.load(OLD/'campaign/runs'/case/'precision.npz', allow_pickle=False) as archive:
        data = {key: archive[key].copy() for key in FIELDS}
    t = data['time_s']; n = len(t); history = fixed_arrays(n); history['time_s'][:] = t
    held = None; snapshots = {}; max_load_error = max_point_error = max_ref_error = 0.; exact_rows = 0
    targets = {int(np.argmin(abs(t-time))): time for time in TIMES[case]}
    for row in targets:
        if abs(t[row]-targets[row]) > 1e-9: raise ValueError('Frozen event time not on saved clock')
    for k, time in enumerate(t):
        mirror.set_state(data['exact_qpos'][k], data['exact_qvel'][k], time, data['exact_previous_applied'][k])
        if mirror.terrain is not None or mirror.ground is not None or mirror.data.ncon != 0:
            raise AssertionError('Mirror acquired contacts/ground')
        wrench = data['exact_observer_wrench'][k]; records = locator.records(mirror.data, wrench[:3], wrench[3:])
        measurement = measure_records(mirror, records); loads = np.array([measurement['loads_N'][f] for f in FAMILIES])
        load_error = float(np.max(abs(loads-data['exact_estimated_support_loads_N'][k])))
        point_error = float(np.max(abs(measurement['point_m']-data['exact_estimated_contact_point_m'][k])))
        max_load_error = max(max_load_error, load_error); max_point_error = max(max_point_error, point_error)
        velocity = data['exact_reference_velocity_m_s'][k]; moving = bool(np.linalg.norm(velocity)>0.); touching = bool(loads.sum()>.001)
        matrix, target, details, jacobians = build_map(mirror, records, velocity, mujoco); native = solve_map(matrix, target)
        if not moving: vref, feed, branch = np.zeros(9), np.zeros(5), 0
        elif not touching and held is not None: vref, feed, branch = held[0].copy(), held[1].copy(), 2
        else:
            vref, feed = rolling_feedforward_records(mirror, records, velocity, 0.); branch = 1
            if not np.array_equal(np.r_[vref[6:9], feed[3:]], native['allocated']):
                raise AssertionError('Independently built native matrix does not reproduce native helper exactly')
        if touching: held = (vref.copy(), feed.copy())
        rebuilt = np.r_[vref[6:9], feed[3:]]; logged = data['exact_allocation_drive_reference'][k]
        error = max(float(np.max(abs(rebuilt-logged))), float(np.max(abs(vref-data['exact_reference_qvel'][k]))), float(np.max(abs(feed-data['exact_rolling_feedforward'][k]))))
        exact = np.array_equal(rebuilt, logged) and np.array_equal(vref, data['exact_reference_qvel'][k]) and np.array_equal(feed, data['exact_rolling_feedforward'][k])
        exact_rows += int(exact); max_ref_error = max(max_ref_error, error)
        history['record_count'][k] = len(records); history['branch_code'][k] = branch
        history['native_rank'][k] = native['rank']; history['native_singular_values'][k, :len(native['singular_values'])] = native['singular_values']
        history['native_condition_resolved'][k] = native['condition_resolved']; history['native_cutoff'][k] = native['cutoff']
        history['native_residual_norm'][k] = native['residual_norm']; history['native_scaled_reference_norm'][k] = native['normalized_reference_norm']
        history['native_target_modal_projection'][k, :len(native['target_modal_projection'])] = native['target_modal_projection']
        history['native_scaled_reference_modal_coefficients'][k, :len(native['scaled_reference_modal_coefficients'])] = native['scaled_reference_modal_coefficients']
        history['native_matrix'][k, :len(matrix)] = matrix; history['native_target'][k, :len(target)] = target
        history['native_reference'][k] = rebuilt; history['logged_reference'][k] = logged; history['reconstruction_max_error'][k] = error
        history['native_family_loads_N'][k] = loads; history['inferred_family_mask'][k] = loads>0.
        if loads.sum()>0: history['dominant_inferred_family'][k] = int(np.argmax(loads))
        if records:
            normal = records[0]['normal']; history['intrinsic_normal'][k] = normal
            history['raw_filtered_normal_angle_deg'][k] = np.degrees(np.arccos(np.clip(normal@data['exact_estimated_normal'][k], -1., 1.)))
            xy, tangent, rotation = [], [], []
            for record, detail, jac in zip(records, details, jacobians):
                fi = family(record['name']); history['native_family_points_m'][k, fi] = record['pos']
                history['native_axis_cross_jacobian_error'][k] = max(history['native_axis_cross_jacobian_error'][k], detail['independent_axis_cross_lever_jacobian_max_error'])
                material = jac@mirror.data.qvel
                if record['name'].startswith('belt_'):
                    geom = int(record['geom']); material += mirror.data.geom_xmat[geom].reshape(3, 3)@mirror.model.geom_surfacevel[geom, :3]
                xy.append(np.dot(material[:2], material[:2])); projection = material-record['normal']*(record['normal']@material)
                tangent.append(np.dot(projection, projection)); rot = jac[:2, 3:6]@mirror.data.qvel[3:6]; rotation.append(rot@rot)
            history['current_no_slip_xy_rms_m_s'][k] = np.sqrt(np.mean(xy)); history['current_no_slip_tangent_rms_m_s'][k] = np.sqrt(np.mean(tangent))
            history['measured_base_rotation_contact_xy_rms_m_s'][k] = np.sqrt(np.mean(rotation))
            angles = [a['acute_collinearity_angle_deg'] for d in details for a in d['active_direction_angles']]
            if angles: history['native_minimum_acute_direction_angle_deg'][k] = min(angles)
        history['filtered_normal'][k] = data['exact_estimated_normal'][k]
        pressures, points = data['truth_patch_loads_N'][k], data['truth_patch_points_m'][k]
        mask = pressures>0.; history['truth_positive_family_mask'][k] = mask
        if pressures.sum()>0:
            history['truth_five_percent_family_mask'][k] = pressures>.05*pressures.sum(); history['dominant_positive_pressure_family'][k] = int(np.argmax(pressures))
            history['estimated_to_pressure_equivalent_point_error_m'][k] = np.linalg.norm(measurement['point_m']-data['truth_equivalent_point_m'][k])
        history['instantaneous_positive_pressure_family_set_agreement'][k] = np.array_equal(loads>0, mask)
        oracle_records = []
        patch_by_family = {p['support']: p for p in locator.patches}
        for i, f in enumerate(FAMILIES):
            if not mask[i] or not np.isfinite(points[i]).all(): continue
            patch0 = patch_by_family[f]
            oracle_records.append(dict(name=patch0['name'], body=patch0['body'], geom=patch0['geom'], pos=points[i], normal=data['exact_estimated_normal'][k], normal_force=pressures[i]))
        oracle_matrix, oracle_target, _, _ = build_map(mirror, oracle_records, velocity, mujoco); oracle = solve_map(oracle_matrix, oracle_target)
        history['truth_aggregate_reference'][k] = oracle['allocated']; history['truth_aggregate_rank'][k] = oracle['rank']
        history['truth_aggregate_singular_values'][k, :len(oracle['singular_values'])] = oracle['singular_values']
        history['truth_aggregate_condition_resolved'][k] = oracle['condition_resolved']; history['truth_aggregate_residual_norm'][k] = oracle['residual_norm']
        history['truth_aggregate_record_count'][k] = len(oracle_records)
        point_only = None
        if records and all(mask[family(r['name'])] and np.isfinite(points[family(r['name'])]).all() for r in records):
            copied = [{**r, 'pos': points[family(r['name'])].copy()} for r in records]
            a, b, _, _ = build_map(mirror, copied, velocity, mujoco); point_only = solve_map(a, b)
            history['same_family_point_oracle_available'][k] = True; history['same_family_point_reference'][k] = point_only['allocated']
            history['same_family_point_condition_resolved'][k] = point_only['condition_resolved']
        if k in targets:
            snapshots[f'{time:.12f}'] = dict(time_s=float(time), source_frame=k, branch=['stationary_zero', 'current_record_allocation', 'dropout_held'][branch],
                reference_reconstruction_exact=exact, reference_reconstruction_max_error=error, desired_velocity_xy_m_s=velocity,
                actual_joint_velocity_rad_s=mirror.data.qvel[6:9], previous_applied_input=data['exact_previous_applied'][k],
                inferred_records=details, native=native, logged_reference=logged,
                native_intrinsic_normal=history['intrinsic_normal'][k], filtered_normal=data['exact_estimated_normal'][k],
                last_solve_positive_pressure_loads_N=pressures, last_solve_positive_pressure_points_m=points,
                inferred_pressure_family_set_agreement=history['instantaneous_positive_pressure_family_set_agreement'][k],
                pressure_equivalent_point_error_m=history['estimated_to_pressure_equivalent_point_error_m'][k],
                truth_family_aggregate_algebraic_map=oracle, same_family_point_algebraic_map=point_only,
                measured_no_slip_tangent_rms_m_s=history['current_no_slip_tangent_rms_m_s'][k])
        if k%2000 == 0: print(json.dumps(dict(case=case, frame=k, rows=n)), flush=True)
    np.savez_compressed(stage/(case+'_history.npz'), **history)
    result = dict(case_id=case, rows=n, reference_exact_rows=exact_rows, all_reference_rows_exact=exact_rows==n,
        max_reference_absolute_error=max_ref_error, max_load_absolute_error_N=max_load_error, max_point_component_error_m=max_point_error,
        max_independent_axis_cross_jacobian_error=float(history['native_axis_cross_jacobian_error'].max()),
        branch_counts={name: int(np.sum(history['branch_code']==i)) for i, name in enumerate(('stationary_zero', 'current_record_allocation', 'dropout_held'))},
        history=case+'_history.npz', snapshots=snapshots)
    write(stage/(case+'_report.json'), result)
    if max_ref_error>1e-10 or max_load_error>1e-10 or max_point_error>1e-10:
        raise AssertionError('Native reconstruction does not match saved raw references/loads/point')
    return result


def run(stage):
    reg = verify(stage)
    if (stage/'report.json').exists(): raise FileExistsError('Preserve previous calculations')
    sys.path.insert(0, str(ROOT/'src'))
    import mujoco
    from spherical_wheel.mirror import StateMirror
    from spherical_wheel.model import load_config
    from spherical_wheel.offroad_control import OffroadParams
    from spherical_wheel.contact_sensing import SupportLocator, measure_records
    from spherical_wheel.experiments import rolling_feedforward_records
    versions = {x: version(x) for x in ('mujoco', 'numpy', 'scipy')}
    for case in CASES:
        old = json.loads((OLD/'campaign/runs'/case/'provenance.json').read_text())
        if versions != old['versions']: raise ValueError('Use original mirror-analysis library versions')
    calls = []
    def forbidden(*args, **kwargs):
        calls.append(True); raise AssertionError('Physics/contact-query API is prohibited')
    with ExitStack() as guards:
        for name in ('mj_step', 'mj_step1', 'mj_step2', 'mj_collision', 'mj_contactForce'):
            guards.enter_context(patch.object(mujoco, name, forbidden))
        mirror = StateMirror(load_config(), stage/'known_model')
        params = OffroadParams(mode='hardware', rolling_allocation='least_squares', track_contact_point=False)
        radius = float(mirror.config['geometry']['sphere_radius_m']); weight = float(mirror.model.body_subtreemass[mirror.fork_id])*abs(float(mirror.config['simulation']['gravity_m_s2'][2]))
        locator = SupportLocator(mirror.model, mirror.fork_id, radius, float(mirror.config['contact']['friction'][0]), params.support_min_load_fraction*weight)
        results = {case: analyze_case(case, stage, mirror, locator, params, mujoco, rolling_feedforward_records, measure_records) for case in CASES}
    verify(stage)
    write(stage/'report.json', dict(iteration='I-77', native_reconstruction_verified=all(r['max_reference_absolute_error']<=1e-10 for r in results.values()),
        all_reference_rows_bit_exact=all(r['all_reference_rows_exact'] for r in results.values()), cases=results, versions=versions,
        source_inputs_unchanged=True, no_physics_integration=True, forbidden_api_calls=len(calls), mirror_contact_disabled=True,
        native_matrix_uses='WorldXY Jacobian, unweighted percontact. Observer intrinsic point/family enters; filtered normal and load-share weighting do not enter the least-squares matrix.',
        scale=SCALE, rcond=1e-7, scale_interpretation='Existing normalized-reference preferences, not hard speed or actuator limits.',
        branch_code={'0': 'stationary_zero', '1': 'current_record_allocation', '2': 'dropout_held'},
        oracle_interpretation='Instantaneous positive-pressure family-aggregate algebraic comparisons only. Not fullmanifold, full5msforce truth, physical interventions, or proof of contact timing/physical causation.',
        material_velocity_interpretation='Residual at inferred points can indicate slip/contact/model/motion disagreement; it cannot uniquely identify contact error.',
        causality_limit='Exact native algebra and geometry can explain a large request. A changed algebraic oracle is not an executed closed-loop counterfactual.'))
    files = [p for p in stage.rglob('*') if p.is_file() and p.name != 'artifact_manifest.json']
    write(stage/'artifact_manifest.json', dict(files=[dict(path=str(p.relative_to(stage)), bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(files)]))
    print(json.dumps({case: dict(exact_rows=r['reference_exact_rows'], rows=r['rows'], max_error=r['max_reference_absolute_error']) for case, r in results.items()}), flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(); p.add_argument('--stage', type=Path, required=True); p.add_argument('--register', action='store_true'); p.add_argument('--run', action='store_true'); a = p.parse_args()
    if a.register: register(a.stage.resolve())
    if a.run:
        try: run(a.stage.resolve())
        except Exception:
            write(a.stage.resolve()/'setup_failure.json', dict(traceback=traceback.format_exc(), no_physics_integration=True)); raise
