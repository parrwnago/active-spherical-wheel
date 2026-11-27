"""I-68 frozen offline analysis of cached substeps and exact native probes."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from interval_core import (angle_deg, contact_frame, decomposition, intrinsic_normal,
                           middle_rotation, point_force_generalized, reduce_interval,
                           rotation, vector_stats, world_rows)

EVENTS = (7.875, 32.110, 35.055, 35.205)
TIME_TOL = 1e-8


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, np.ndarray):
        return clean(value.tolist())
    if isinstance(value, np.generic):
        return clean(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def scalar_stats(x, mask):
    a = np.asarray(x, float); y = a[np.asarray(mask, bool) & np.isfinite(a)]
    return dict(count=len(y), mean=float(np.mean(y)) if len(y) else None,
                p95=float(np.percentile(y, 95)) if len(y) else None,
                maximum=float(np.max(y)) if len(y) else None)


def split_stats(x, mask):
    return {label: vector_stats(x[:, sl], mask)
            for label, sl in [('force_N', slice(0, 3)), ('base_moment_Nm', slice(3, 6)),
                              ('joint_Nm', slice(6, 9))]}


def report_group(a, mask):
    m = np.asarray(mask, bool)
    common = m & a['valid']
    comparisons = {}
    for pose in ('before', 'after'):
        mu, interval, last = a['observer_interval_mean'], a[f'interval_{pose}'], a[f'last_{pose}']
        first = split_stats(mu-interval, common); second = split_stats(mu-last, common)
        r_i, r_l = first['joint_Nm']['vector_rms'], second['joint_Nm']['vector_rms']
        comparisons[pose] = dict(observer_minus_interval=first, observer_minus_last=second,
                                interval_minus_last=split_stats(interval-last, common),
                                joint_rms_interval_below_last=(bool(r_i < r_l-1e-12)
                                                               if r_i is not None else None))
    endpoint_valid = common & a['estimated_point_model_valid']
    decomp = decomposition(a['observer_interval_mean'], a['interval_before'], a['estimated_point_model'])
    report = dict(total_rows=int(m.sum()), valid_rows=int(common.sum()),
                  endpoint_truth_eligible_rows=int((m & a['endpoint_truth_eligible']).sum()),
                  comparisons=comparisons,
                  interval_routing_comparison_support=(all(comparisons[p]['joint_rms_interval_below_last'] is True
                                                           for p in ('before', 'after'))
                                                       if common.any() else None),
                  geometry_timing_sensitivity=split_stats(a['interval_before']-a['interval_after'], common),
                  endpoint_native_equivalent_dominant_family_proxy=dict(
                      valid_rows=int(endpoint_valid.sum()),
                      observer_minus_endpoint=split_stats(decomp['observer_minus_endpoint'], endpoint_valid),
                      interval_minus_endpoint=split_stats(decomp['interval_minus_endpoint'], endpoint_valid),
                      joint_squared_norm_cross_term=scalar_stats(
                          2*np.sum(decomp['observer_minus_interval'][:, 6:]*decomp['interval_minus_endpoint'][:, 6:], axis=1), endpoint_valid)),
                  slip={p: {k: scalar_stats(a[f'{p}_{k}'], m)
                            for k in ('slip_mean_norm_m_s', 'slip_rms_m_s', 'slip_max_m_s')}
                        for p in ('before', 'after')},
                  normal={k: scalar_stats(a[k], m) for k in
                          ('intrinsic_to_interval_normal_deg', 'live_to_interval_normal_deg',
                           'intrinsic_to_endpoint_normal_deg', 'live_to_endpoint_normal_deg',
                           'interval_normal_resultant_length')},
                  rejected_force_map=split_stats(a['rejected_interval_before'], common),
                  external_wrench=split_stats(a['external_interval_before'], common))
    return report


def analyse(trace, precision, *, radius_m=.12, old_precision=None, fork_com_local_m=None):
    """Only cached arrays are accepted. No simulator, controller or fit is called."""
    t = np.asarray(trace['time_s'], float); n = len(t)
    for key, shape in [('qpos_before', (n, 10)), ('qpos_after', (n, 10)),
                       ('qvel_before', (n, 9)), ('qvel_after', (n, 9)),
                       ('held_input', (n, 5)), ('disturbance_world', (n, 6))]:
        if np.shape(trace[key]) != shape:
            raise ValueError(f'{key}: expected {shape}')
    offsets = np.asarray(trace['contact_offsets'], np.int64)
    rows = np.asarray(trace['contact_rows'], float)
    if len(offsets) != n+1 or offsets[0] != 0 or offsets[-1] != len(rows) or np.any(np.diff(offsets) < 0):
        raise ValueError('invalid ragged contact offsets')
    expected_bodies = ['shell_A', 'shell_B', 'carrier', 'carrier', 'shell_A', 'shell_B', 'fork']
    if list(trace['tag_mirror_body_names'].astype(str)) != expected_bodies:
        raise ValueError('analytical map topology/tag body mapping mismatch')
    dt = np.asarray(trace['step_dt_s'], float)
    if not np.isfinite(dt).all() or np.max(np.abs(dt-.001), initial=0) > 1e-12:
        raise ValueError('registered 1 ms step sizes required; reset microsteps excluded')
    if np.max(np.abs(trace['bridge_time_after_s']-trace['bridge_time_before_s']-dt), initial=0) > TIME_TOL:
        raise ValueError('reported step duration/bridge clock mismatch')
    step = {f'{p}_generalized': np.full((n, 9), np.nan) for p in ('before', 'after')}
    for p in ('before', 'after'):
        step[f'{p}_family_generalized'] = np.full((n, 5, 9), np.nan)
        step[f'{p}_rejected_generalized'] = np.full((n, 9), np.nan)
        for k in ('slip_mean_norm_m_s', 'slip_rms_m_s', 'slip_max_m_s', 'pressure_N',
                  'normal_resultant_length'):
            step[f'{p}_{k}'] = np.full(n, np.nan)
        step[f'{p}_mean_normal'] = np.full((n, 3), np.nan)
        step[f'{p}_slip_mean_vector_m_s'] = np.full((n, 3), np.nan)
        step[f'{p}_pressure_by_family_N'] = np.full((n, 5), np.nan)
    step['valid'] = np.zeros(n, bool)
    row_slip_before, row_slip_after = np.full(len(rows), np.nan), np.full(len(rows), np.nan)
    invalid_rows = rejected_rows = 0
    external = np.zeros((n, 9))
    for i in range(n):
        r = rows[offsets[i]:offsets[i+1]]
        maps = {}
        for p in ('before', 'after'):
            z = contact_frame(trace[f'qpos_{p}'][i], trace[f'qvel_{p}'][i], r, trace['held_input'][i])
            maps[p] = z
            for k in ('generalized', 'generalized_by_family', 'rejected_generalized', 'pressure_N',
                      'slip_mean_norm_m_s', 'slip_rms_m_s', 'slip_max_m_s', 'normal_resultant_length',
                      'mean_normal', 'pressure_by_family_N', 'slip_mean_vector_m_s'):
                target = 'family_generalized' if k == 'generalized_by_family' else k
                step[f'{p}_{target}'][i] = z[k]
            if not z['slip_valid']:
                for k in ('slip_mean_norm_m_s', 'slip_rms_m_s', 'slip_max_m_s'):
                    step[f'{p}_{k}'][i] = np.nan
                step[f'{p}_slip_mean_vector_m_s'][i] = np.nan
            (row_slip_before if p == 'before' else row_slip_after)[offsets[i]:offsets[i+1]] = z['row_slip_speed_m_s']
        step['valid'][i] = maps['before']['force_map_valid'] and maps['after']['force_map_valid']
        invalid_rows += maps['before']['invalid_force_row_count']; rejected_rows += maps['before']['rejected_row_count']
        wrench = trace['disturbance_world'][i]
        if np.any(wrench != 0):
            if fork_com_local_m is None:
                raise ValueError('nonzero external wrench requires configured fork COM')
            q = trace['qpos_before'][i]; r = rotation(q[3:7])
            point = q[:3] + r @ np.asarray(fork_com_local_m)
            load, _ = point_force_generalized(q, point[None], wrench[None, :3], [6])
            external[i] = load[0]; external[i, 3:6] += r.T @ wrench[3:]
    ids = np.unique(trace['control_end_index']).astype(int); k = len(ids)
    a = {name: np.full((k, 9), np.nan) for name in
         ('observer_interval_mean', 'observer_generalized', 'interval_before', 'interval_after',
          'last_before', 'last_after', 'estimated_point_model', 'rejected_interval_before', 'external_interval_before')}
    a.update(time_s=np.full(k, np.nan), control_end_index=ids, valid=np.zeros(k, bool),
             endpoint_truth_eligible=np.zeros(k, bool), estimated_point_model_valid=np.zeros(k, bool),
             interval_family_generalized=np.full((k, 5, 9), np.nan),
             endpoint_family_pressure_N=np.full((k, 5), np.nan),
             interval_mean_normal=np.full((k, 3), np.nan), endpoint_mean_normal=np.full((k, 3), np.nan),
             intrinsic_normal=np.full((k, 3), np.nan), live_estimated_normal=np.full((k, 3), np.nan))
    for key in ('intrinsic_to_interval_normal_deg', 'live_to_interval_normal_deg',
                'intrinsic_to_endpoint_normal_deg', 'live_to_endpoint_normal_deg',
                'interval_normal_resultant_length'):
        a[key] = np.full(k, np.nan)
    for p in ('before', 'after'):
        for key in ('slip_mean_norm_m_s', 'slip_rms_m_s', 'slip_max_m_s'):
            a[f'{p}_{key}'] = np.full(k, np.nan)
    checks = dict(complete_holds=True, held_input_matches_previous_applied=True,
                  qpos_endpoint_match=True, qvel_endpoint_match=True,
                  maximum_input_difference=0., maximum_qpos_difference=0., maximum_qvel_difference=0.)
    for z, end in enumerate(ids):
        ix = np.flatnonzero(trace['control_end_index'] == end)
        if len(ix) != 5 or not np.array_equal(trace['substep_index'][ix], np.arange(1, 6)) or not trace['hold_completed'][ix].all():
            raise ValueError('each completed controller interval must contain exactly five ordered steps')
        if end <= 0 or end >= len(precision['time_s']):
            raise ValueError('precision index out of range')
        starts, ends = trace['control_start_time_s'][ix], trace['control_end_time_s'][ix]
        if (np.ptp(starts) > TIME_TOL or np.ptp(ends) > TIME_TOL
                or abs(np.sum(dt[ix])-(ends[0]-starts[0])) > TIME_TOL
                or abs(float(precision['time_s'][end])-ends[0]) > TIME_TOL
                or np.max(np.abs(t[ix]-(starts[0]+np.arange(1, 6)*.001))) > TIME_TOL):
            raise ValueError('control/observer interval timestamp alignment failed')
        if not np.all(trace['held_input'][ix] == trace['held_input'][ix[0]]):
            raise ValueError('command changed within held interval')
        cmd_diff = float(np.max(np.abs(trace['held_input'][ix]-precision['exact_previous_applied'][end])))
        q_diff = float(np.max(np.abs(trace['qpos_after'][ix[-1]]-precision['exact_qpos'][end])))
        v_diff = float(np.max(np.abs(trace['qvel_after'][ix[-1]]-precision['exact_qvel'][end])))
        checks['maximum_input_difference'] = max(checks['maximum_input_difference'], cmd_diff)
        checks['maximum_qpos_difference'] = max(checks['maximum_qpos_difference'], q_diff)
        checks['maximum_qvel_difference'] = max(checks['maximum_qvel_difference'], v_diff)
        if max(cmd_diff, q_diff, v_diff) > 1e-10:
            raise ValueError('held command or endpoint state does not match aligned exact probe')
        q0, q1 = trace['qpos_before'][ix[0]], trace['qpos_after'][ix[-1]]
        a['time_s'][z] = ends[0]; a['valid'][z] = bool(step['valid'][ix].all())
        a['observer_interval_mean'][z] = precision['exact_observer_interval_mean'][end]
        a['observer_generalized'][z] = precision['exact_observer_generalized'][end]
        ext = reduce_interval(external[ix], dt[ix], q0, q1)
        a['external_interval_before'][z] = ext
        for p in ('before', 'after'):
            if a['valid'][z]:
                a[f'interval_{p}'][z] = reduce_interval(step[f'{p}_generalized'][ix], dt[ix], q0, q1) + ext
                a[f'last_{p}'][z] = world_rows(step[f'{p}_generalized'][ix[-1]]+external[ix[-1]], middle_rotation(q0, q1))
            for key in ('slip_rms_m_s',):
                speed = step[f'{p}_{key}'][ix]; weight = step[f'{p}_pressure_N'][ix]*dt[ix]
                good = np.isfinite(speed) & np.isfinite(weight) & (weight > 0)
                if good.any():
                    a[f'{p}_{key}'][z] = float(np.sqrt(np.sum(weight[good]*speed[good]**2)/np.sum(weight[good])))
            velocity = step[f'{p}_slip_mean_vector_m_s'][ix]
            weight = step[f'{p}_pressure_N'][ix]*dt[ix]
            good = np.isfinite(velocity).all(axis=1) & (weight > 0)
            if good.any():
                mean = np.sum(weight[good, None]*velocity[good], axis=0)/np.sum(weight[good])
                a[f'{p}_slip_mean_norm_m_s'][z] = np.linalg.norm(mean)
            speed = step[f'{p}_slip_max_m_s'][ix]
            if np.isfinite(speed).any():
                a[f'{p}_slip_max_m_s'][z] = np.nanmax(speed)
        if a['valid'][z]:
            a['interval_family_generalized'][z] = reduce_interval(step['before_family_generalized'][ix], dt[ix], q0, q1)
            a['rejected_interval_before'][z] = reduce_interval(step['before_rejected_generalized'][ix], dt[ix], q0, q1)
        pressure = step['before_pressure_by_family_N'][ix[-1]]
        a['endpoint_family_pressure_N'][z] = pressure
        wheel_pressure = pressure[:4].sum()
        a['endpoint_truth_eligible'][z] = wheel_pressure >= 1 and np.max(pressure[:4]) >= .9*wheel_pressure and pressure[4] <= 1e-12
        normal = step['before_mean_normal'][ix]; weight = step['before_pressure_N'][ix]*dt[ix]
        good = np.isfinite(normal).all(axis=1) & (weight > 0)
        if good.any():
            # Recover each step's unnormalized pressure-mean normal, so dispersion is retained.
            resultant = step['before_normal_resultant_length'][ix]
            mean = np.sum(normal[good]*resultant[good, None]*weight[good, None], axis=0)/np.sum(weight[good])
            length = np.linalg.norm(mean); a['interval_normal_resultant_length'][z] = length
            if length > 0:
                a['interval_mean_normal'][z] = mean/length
        a['endpoint_mean_normal'][z] = normal[-1]
        gamma = a['observer_generalized'][z]
        hint = intrinsic_normal(gamma[:3], gamma[3:6], radius_m)
        live = precision['exact_estimated_normal'][end]
        a['intrinsic_normal'][z], a['live_estimated_normal'][z] = hint, live
        for label, value in (('intrinsic', hint), ('live', live)):
            for where in ('interval', 'endpoint'):
                a[f'{label}_to_{where}_normal_deg'][z] = angle_deg(value, a[f'{where}_mean_normal'][z])
        estimated_load = precision['exact_estimated_support_loads_N'][end]
        point = precision['exact_estimated_contact_point_m'][end]
        if np.isfinite(estimated_load).all() and estimated_load.sum() > 0 and np.isfinite(point).all():
            family = int(np.argmax(estimated_load))
            load, valid = point_force_generalized(q1, point[None], a['observer_interval_mean'][z, None, :3], [family])
            if valid[0]:
                a['estimated_point_model'][z] = world_rows(load[0], middle_rotation(q0, q1))
                a['estimated_point_model_valid'][z] = True
    masks = {'full_primary_time_1_to_41': (a['time_s'] >= 1-TIME_TOL) & (a['time_s'] < 41-TIME_TOL),
             'broad_32_to_36': (a['time_s'] >= 32-TIME_TOL) & (a['time_s'] < 36-TIME_TOL)}
    for event in EVENTS:
        masks[f'event_{event:.3f}_plus_minus_100ms'] = np.abs(a['time_s']-event) <= .100+TIME_TOL
    groups = {name: report_group(a, mask) for name, mask in masks.items()}
    truth_groups = {name: report_group(a, mask & a['endpoint_truth_eligible']) for name, mask in masks.items()}
    event_rows = {}
    for event in EVENTS:
        pick = np.flatnonzero(np.abs(a['time_s']-event) <= TIME_TOL)
        event_rows[f'{event:.3f}'] = {name: value[pick[0]] for name, value in a.items()} if len(pick) == 1 else dict(present=False)
    replication = None
    if old_precision is not None:
        if np.shape(old_precision['time_s']) != np.shape(precision['time_s']) or not np.allclose(old_precision['time_s'], precision['time_s'], atol=TIME_TOL, rtol=0):
            raise ValueError('old/new baseline time grids differ')
        replication = {}
        for field in ('exact_qpos', 'exact_qvel', 'exact_applied', 'exact_observer_interval_mean', 'exact_estimated_contact_point_m'):
            old, new = np.asarray(old_precision[field]), np.asarray(precision[field])
            diff = np.max(np.abs(new-old), axis=1)
            nz = np.flatnonzero(diff != 0)
            replication[field] = dict(array_exact=bool(np.array_equal(old, new)), max_absolute_difference=float(diff.max()),
                                      first_nonexact_time_s=float(precision['time_s'][nz[0]]) if len(nz) else None)
    report = dict(schema_version=1, steps=n, intervals=k, raw_contact_rows=len(rows),
                  invalid_force_rows=invalid_rows, rejected_rows=rejected_rows, checks=checks,
                  groups=groups, endpoint_truth_eligible_groups=truth_groups, events=event_rows,
                  old_baseline_replication=replication,
                  caveats=['Force-derived mapped reference; no exported contact couples or total constraint wrench.',
                           'Endpoint native equivalent point/dominant-family model differs from I66 sampled family mean.',
                           'Interval-vs-last strict comparison is descriptive and not calibrated observer agreement.',
                           'World normal and slip are solver-geometry/kinematic diagnostics, not independent sensors.'])
    step['time_s'] = t; step['control_end_index'] = trace['control_end_index']
    step['row_slip_before_m_s'], step['row_slip_after_m_s'] = row_slip_before, row_slip_after
    return a, step, clean(report)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--trace', type=Path, required=True)
    parser.add_argument('--precision', type=Path, required=True)
    parser.add_argument('--old-precision', type=Path)
    parser.add_argument('--radius-m', type=float, default=.12)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    trace, precision = np.load(args.trace, allow_pickle=False), np.load(args.precision, allow_pickle=False)
    old = np.load(args.old_precision, allow_pickle=False) if args.old_precision else None
    a, step, report = analyse(trace, precision, radius_m=args.radius_m, old_precision=old)
    args.output.mkdir(parents=True)
    np.savez_compressed(args.output/'interval_frames.npz', **a)
    np.savez_compressed(args.output/'substep_frames.npz', **step)
    report['inputs'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in (args.trace, args.precision, args.old_precision) if p}
    (args.output/'report.json').write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+'\n')
    manifest = {p.name: dict(bytes=p.stat().st_size, sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                for p in args.output.iterdir() if p.is_file()}
    (args.output/'artifact_manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True)+'\n')
    print(json.dumps(dict(intervals=report['intervals'], valid_primary=report['groups']['full_primary_time_1_to_41']['valid_rows'],
                          primary_comparison=report['groups']['full_primary_time_1_to_41']['comparisons'],
                          events=report['events']), allow_nan=False))


if __name__ == '__main__':
    main()
