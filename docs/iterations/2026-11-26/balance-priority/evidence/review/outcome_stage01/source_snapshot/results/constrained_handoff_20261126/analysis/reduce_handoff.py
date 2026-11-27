"""I56 registered saved-array comparison; no controller/model/physics calls.

Reuses only the immutable I49 metric functions and corrected I41 reference
EVENT DEFINITIONS, not their old outcomes, retries or per-policy histories.
New command, yaw and optimizer diagnostics remain separate from performance.
"""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import csv
import hashlib
import gzip
import json
import shutil
import types

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]
LEGACY = ROOT/'results/belt_capacity_sweep_20261126/analyze_capacity.py'
LEGACY_SHA256 = 'cbaf7ae18989e1a07e9dfaf4167dd2eaa75fd59d064c4259a7b75f79d05c3a6d'
OLD_PROTOCOL = ROOT/'results/contact_scheduling_joint_tracking_20261126/protocol.json'
PHASES = ('m100', 'p030', 'p090')
DT = .005
BOUNDS = np.array([1.5, .6, .6, .3, .3])
REFERENCE_FIELDS = ('time_s', 'exact_reference_xy', 'exact_reference_velocity_m_s', 'exact_reference_acceleration_m_s2')
EVENT_FIELDS = ('event_id', 'time_s', 'axis', 'axis_index', 'incoming_sign', 'outgoing_sign', 'window_s', 'reference_component_extremum_m')
PROJECTION_FIELDS = ('time_s', 'exact_qpos', 'exact_qvel', 'exact_reference_qpos',
    'exact_reference_xy', 'exact_reference_velocity_m_s', 'exact_reference_acceleration_m_s2',
    'exact_applied', 'exact_previous_applied', 'exact_input_bounds', 'exact_requested',
    'exact_native_requested')
DIAGNOSTIC_FIELDS = ('equality_residual_max', 'secondary_balance_delta', 'first_stage_balance_delta',
    'tangent_confidence_proxy', 'estimated_material_velocity_mismatch_m_s',
    'native_contact_velocity_residual_norm_m_s', 'governed_contact_velocity_residual_norm_m_s',
    'model_seconds', 'allocation_seconds', 'total_seconds', 'active_amplitude', 'active_slew',
    'secondary_recovery_used', 'secondary_primary_restore_used', 'solver_warnings',
    'relative_rotor_vertical_momentum_Nms', 'proxy_relative_rotor_vertical_momentum_Nms')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def safe(value):
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return safe(value.tolist())
    if isinstance(value, np.generic):
        return safe(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def save(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {k: z[k].copy() for k in z.files}


def pure_module(path, name):
    module = types.ModuleType(name)
    module.__file__ = str(path)
    exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
    return module


def dependencies():
    old = json.loads(OLD_PROTOCOL.read_text())
    helper = ROOT/old['inputs']['event_helper']
    turns = ROOT/old['inputs']['event_turns']
    if sha(LEGACY) != LEGACY_SHA256:
        raise ValueError('Original pure I49 metrics changed')
    for key, path in (('event_helper', helper), ('event_turns', turns)):
        if sha(path) != old['input_sha256'][key]:
            raise ValueError(f'Original corrected event definition changed: {key}')
    return helper, turns


def rotation(quat):
    q = np.asarray(quat, float)
    q = q/np.linalg.norm(q, axis=1)[:, None]
    w, x, y, z = q.T
    r = np.empty((len(q), 3, 3))
    r[:, 0, 0] = 1-2*(y*y+z*z)
    r[:, 0, 1] = 2*(x*y-z*w)
    r[:, 0, 2] = 2*(x*z+y*w)
    r[:, 1, 0] = 2*(x*y+z*w)
    r[:, 1, 1] = 1-2*(x*x+z*z)
    r[:, 1, 2] = 2*(y*z-x*w)
    r[:, 2, 0] = 2*(x*z-y*w)
    r[:, 2, 1] = 2*(y*z+x*w)
    r[:, 2, 2] = 1-2*(x*x+y*y)
    return r


def derivative(times, values, lag=1):
    out = np.full_like(values, np.nan, dtype=float)
    if len(times) > lag:
        out[lag:] = (values[lag:]-values[:-lag])/(times[lag:]-times[:-lag])[:, None]
    return out


def signals(raw):
    t, q, v = raw['time_s'], raw['exact_qpos'], raw['exact_qvel']
    r = rotation(q[:, 3:7])
    omega = np.einsum('nij,nj->ni', r, v[:, 3:6])
    x = r[:, :, 0]
    h2 = np.sum(x[:, :2]**2, axis=1)
    valid = h2 > .04
    heading = np.full(len(t), np.nan)
    indices = np.flatnonzero(valid)
    for run in np.split(indices, np.flatnonzero(np.diff(indices) > 1)+1):
        if len(run):
            heading[run] = np.unwrap(np.arctan2(x[run, 1], x[run, 0]))
    xdot = np.cross(omega, x)
    rate = (x[:, 0]*xdot[:, 1]-x[:, 1]*xdot[:, 0])/np.maximum(h2, 1e-30)
    rate[~valid] = np.nan
    return dict(time_s=t, world_omega_rad_s=omega, heading_rad=heading, heading_valid=valid,
        heading_rate_rad_s=rate, heading_projection_norm=np.sqrt(h2),
        planar_speed_m_s=np.linalg.norm(v[:, :2], axis=1),
        linear_acceleration5ms_m_s2=derivative(t, v[:, :3]),
        linear_acceleration50ms_m_s2=derivative(t, v[:, :3], 10),
        angular_acceleration_world5ms_rad_s2=derivative(t, omega),
        encoder_acceleration5ms_rad_s2=derivative(t, v[:, 6:9]))


def mask_window(t, window):
    return (t >= window[0]-1e-9) & (t <= window[1]+1e-9)


def covered(t, window):
    return bool(len(t) and t[0] <= window[0]+1e-9 and t[-1] >= window[1]-1e-9)


def vector_summary(values):
    a = np.asarray(values, float)
    if a.ndim == 1:
        a = a[:, None]
    finite = np.isfinite(a).all(axis=1)
    a = a[finite]
    if not len(a):
        return dict(available=False, samples=0)
    norm = np.linalg.norm(a, axis=1)
    return dict(available=True, samples=len(a), max_abs_by_component=np.max(abs(a), axis=0),
        rms_by_component=np.sqrt(np.mean(a*a, axis=0)), peak_norm=float(norm.max()),
        rms_norm=float(np.sqrt(np.mean(norm*norm))), p95_norm=float(np.quantile(norm, .95)))


def motion_window(raw, derived, window):
    t = raw['time_s']
    selected = mask_window(t, window)
    if not selected.any():
        return dict(available=False, complete=False, window_s=window, samples=0)
    valid = derived['heading_valid']
    pair = selected[1:] & selected[:-1] & valid[1:] & valid[:-1]
    increments = np.diff(derived['heading_rad'])[pair]
    omega = derived['world_omega_rad_s'][selected]
    return dict(available=True, complete=covered(t, window), window_s=window,
        samples=int(selected.sum()), all_heading_samples_valid=bool(valid[selected].all()),
        minimum_heading_projection_norm=float(derived['heading_projection_norm'][selected].min()),
        heading_absolute_travel_rad=float(abs(increments).sum()),
        heading_absolute_travel_turns=float(abs(increments).sum()/(2*np.pi)),
        heading_net_travel_rad=float(increments.sum()),
        world_z_peak_abs_rad_s=float(abs(omega[:, 2]).max()),
        world_z_rms_rad_s=float(np.sqrt(np.mean(omega[:, 2]**2))),
        world_z_end_rad_s=float(omega[-1, 2]),
        planar_speed_peak_m_s=float(derived['planar_speed_m_s'][selected].max()),
        world_angular_rate=vector_summary(omega),
        actual_joint_rate=vector_summary(raw['exact_qvel'][selected, 6:9]),
        linear_acceleration5ms=vector_summary(derived['linear_acceleration5ms_m_s2'][selected]),
        linear_acceleration50ms=vector_summary(derived['linear_acceleration50ms_m_s2'][selected]),
        angular_acceleration_world5ms=vector_summary(derived['angular_acceleration_world5ms_rad_s2'][selected]),
        encoder_acceleration5ms=vector_summary(derived['encoder_acceleration5ms_rad_s2'][selected]),
        interpretation='Measured5ms endpoint velocities and interval-average backward differences. Heading increments never bridge an invalid projection. Command bounds do not bound actual acceleration.')


def command_constraints(raw, slew_limits, tolerance=1e-10):
    t = raw['time_s']
    u = raw['exact_applied']
    previous = raw['exact_previous_applied']
    caps = raw['exact_input_bounds']
    requested = raw.get('exact_requested', raw['exact_native_requested'])
    if caps.shape != u.shape or previous.shape != u.shape:
        raise ValueError('Command/bound/history shapes disagree')
    if not np.array_equal(caps, np.broadcast_to(BOUNDS, caps.shape)):
        raise ValueError('Registered amplitude bounds changed')
    interval = np.r_[DT, np.diff(t)]
    delta = u-previous
    limit = np.asarray(slew_limits)[None, :]*interval[:, None]
    amplitude_violation = np.maximum(abs(u)-caps, 0.)
    request_violation = np.maximum(abs(requested)-caps, 0.)
    slew_violation = np.maximum(abs(delta)-limit, 0.)
    held = np.arange(len(t)) < len(t)-1
    native = raw['exact_native_requested']
    native_saturation = abs(native) > caps+1e-12
    if len(t) > 1:
        previous_matches = bool(np.array_equal(previous[1:], u[:-1]))
    else:
        previous_matches = True
    return dict(emitted_samples=len(t), held_samples=int(held.sum()),
        all_finite=bool(np.isfinite(np.r_[u.ravel(), previous.ravel(), requested.ravel()]).all()),
        actual_previous_input_matches=previous_matches,
        applied_box_max_violation=float(amplitude_violation.max(initial=0.)),
        requested_box_max_violation=float(request_violation.max(initial=0.)),
        slew_max_violation=float(slew_violation.max(initial=0.)),
        applied_box_pass=bool(np.max(amplitude_violation, initial=0.) <= tolerance),
        requested_box_pass=bool(np.max(request_violation, initial=0.) <= tolerance),
        actual_slew_pass=bool(np.max(slew_violation, initial=0.) <= tolerance),
        applied_max_abs_by_input=np.max(abs(u[held]), axis=0) if held.any() else np.zeros(5),
        maximum_emitted_step_by_input=np.max(abs(delta), axis=0),
        maximum_emitted_rate_by_input=np.max(abs(delta)/interval[:, None], axis=0),
        maximum_held_rate_by_input=np.max(abs(delta[held])/interval[held, None], axis=0) if held.any() else np.zeros(5),
        maximum_held_step_normalized=float(np.max(abs(delta[held])/caps[held])) if held.any() else None,
        native_infeasible_held_fraction=float(np.mean(native_saturation[held].any(axis=1))) if held.any() else None,
        native_infeasible_held_duration_by_input_s=native_saturation[held].sum(axis=0)*DT,
        final_requested_outside_held_fraction=float(np.mean((abs(requested[held]) > caps[held]+1e-12).any(axis=1))) if held.any() else None,
        scope='Hardbox/slew checks include all emitted commands, including first emission against actual priorinput and final unheld emission. Exposure excludes final unheld command. Units first3Nmand last2m/s; slewunitsNm/s andm/s².')


def scalar_stats(values):
    x = np.asarray(values, float)
    if not len(x):
        return dict(available=False, samples=0)
    finite = np.isfinite(x)
    y = x[finite]
    return dict(available=bool(len(y)), samples=len(x), nonfinite_samples=int((~finite).sum()),
        minimum=float(y.min()) if len(y) else None, median=float(np.median(y)) if len(y) else None,
        p95=float(np.quantile(y, .95)) if len(y) else None, maximum=float(y.max()) if len(y) else None,
        rms=float(np.sqrt(np.mean(y*y))) if len(y) else None)


def optimizer_metrics(rows, raw, candidate):
    if len(rows) != len(raw['time_s']):
        raise ValueError('Governor diagnostic row count differs from precision')
    for row, time in zip(rows, raw['time_s']):
        if row['time_s'] != time:
            raise ValueError('Governor/precision clocks disagree')
    diagnostics = [row['diagnostics'] for row in rows]
    if not candidate:
        if any(diagnostics):
            raise ValueError('Native case unexpectedly has optimizer diagnostics')
        return dict(available=False, reason='No optional governor in the native arm')
    if not all(isinstance(row, dict) and row for row in diagnostics):
        raise ValueError('Candidate optimizer diagnostics are missing')
    def values(name):
        return [d[name] for d in diagnostics]
    eq = np.asarray(values('equality_residual_max'), float)
    extra = np.asarray(values('secondary_balance_delta'), float)
    primary = np.asarray(values('first_stage_balance_delta'), float)
    names = ('tangent_confidence_proxy', 'estimated_material_velocity_mismatch_m_s',
        'native_contact_velocity_residual_norm_m_s', 'governed_contact_velocity_residual_norm_m_s',
        'model_seconds', 'allocation_seconds', 'total_seconds')
    stats = {name: scalar_stats(values(name)) for name in names}
    active_amp = np.asarray(values('active_amplitude'), bool)
    active_slew = np.asarray(values('active_slew'), bool)
    return dict(available=True, emitted_samples=len(rows),
        equality_residual_max=float(np.max(eq, initial=0.)), equality_all_finite=bool(np.isfinite(eq).all()),
        secondary_balance_extra_max=float(np.max(abs(extra), initial=0.)),
        secondary_balance_extra_all_finite=bool(np.isfinite(extra).all()),
        primary_balance_change=vector_summary(primary),
        secondary_recovery_samples=int(np.sum(values('secondary_recovery_used'))),
        primary_restore_samples=int(np.sum(values('secondary_primary_restore_used'))),
        solver_warning_samples=int(sum(bool(d['solver_warnings']) for d in diagnostics)),
        active_amplitude_held_fraction_by_input=active_amp[:-1].mean(axis=0) if len(rows)>1 else np.zeros(5),
        active_slew_held_fraction_by_input=active_slew[:-1].mean(axis=0) if len(rows)>1 else np.zeros(5),
        model_confidence_and_cost=stats,
        optimizer_wall_total_above5ms_samples=int(np.sum(np.asarray(values('total_seconds')) > .005)),
        relative_hinge_momentum_error_proxy=vector_summary(np.asarray(values('relative_rotor_vertical_momentum_Nms'))),
        projected_relative_hinge_momentum_error_proxy=vector_summary(np.asarray(values('proxy_relative_rotor_vertical_momentum_Nms'))),
        definitions='Momentum fields are relative-hinge vertical error proxies, not complete rotor/body angular momentum. Confidence is a declared slip-mismatch proxy, not a calibrated probability. Walltimes are instrumented implementation costs in lockstep, not demonstrated real-time hardware performance.')


def case_metrics(raw, rollout, summary, provenance, governor, protocol, program, events, metrics, helper):
    t = raw['time_s']
    if not len(t) or not np.isfinite(t).all() or np.any(abs(np.diff(t)-DT) > 1e-9):
        raise ValueError('Expected a nonempty registered5ms measured-state log')
    for name in REFERENCE_FIELDS:
        expected = program[name][:len(t)]
        if not np.array_equal(raw[name], expected):
            raise ValueError(f'Known timed reference program differs: {name}')
    candidate = provenance['case']['variant'] == 'handoff'
    if not provenance['source_hashes_unchanged_after'] or not provenance['frozen_bank_loaded']:
        raise ValueError('Execution source or frozen-bank condition failed')
    if provenance['fixed_bank_bytes_sha256'] != protocol['gain_cache']['sha256']:
        raise ValueError('Wrong fixed bank in case provenance')
    acceptance = protocol['acceptance']
    moving_window = acceptance['moving_window_s']
    absolute, relative = metrics.attitude(raw)
    track = metrics.tracking_metrics(raw, moving_window)
    whole_complete = covered(t, [0., 44.])
    moving_complete = covered(t, moving_window)
    track.update(full_44_coverage=whole_complete, moving_coverage=moving_complete,
        final_at44_m=track['final_m'] if whole_complete else None,
        full_peak_at44_m=track['full_peak_m'] if whole_complete else None,
        semantics='Observed-prefix metrics retained, but full44s/final gate values are unavailable if the trace terminates early.')
    geo = metrics.geometry(raw, program, helper, moving_window)
    turns = metrics.turns(raw, geo, events, helper)
    derived = signals(raw)
    constraints = command_constraints(raw, protocol['candidate_settings']['allocator']['slew_limits'])
    opt = optimizer_metrics(governor, raw, candidate)
    windows = {'full':[0.,44.], 'moving':moving_window, 'late':acceptance['yaw_window_s']}
    answer = dict(available=True, completed=bool(summary['completed']), failure=summary['failure'],
        observed_duration_s=float(t[-1]), samples=len(t), full44_coverage=whole_complete,
        moving_coverage=moving_complete, timed_tracking=track,
        attitude=dict(peak_absolute_deg=float(np.degrees(absolute).max()),
            peak_raw_reference_error_deg=float(np.degrees(relative).max()),
            rms_raw_reference_error_deg=float(np.degrees(np.sqrt(np.mean(relative*relative)))),
            rms_scope='All measured states0–observedend, including terminal state, matching legacyI52 attitude convention.',
            final_raw_reference_error_deg=float(np.degrees(relative[-1]))),
        original_settling=metrics.settling(t, raw['exact_qvel'], relative, summary['completed']),
        original_route_gates=summary.get('mars', {}).get('gates'),
        motion={name: motion_window(raw, derived, window) for name, window in windows.items()},
        commands=constraints, optimizer=opt, turns=turns,
        local_path=dict(rms_m=float(np.sqrt(np.mean(geo['local_path_distance_m']**2))),
            peak_m=float(np.max(geo['local_path_distance_m'])),
            artificial_projection_boundary_samples=int(geo['artificial_projection_window_boundary'].sum()),
            scope='Exact polyline projection restricted to±2s nominal-time branch; not globally unique cross-track distance.'),
        fixed_bank_numeric_sha256=provenance['fixed_bank_numeric_sha256'],
        raw_target_scope='Original current-state observer-derived formula unchanged. Target histories can differ naturally between policies after states diverge; no easier shaped target is substituted.')
    return answer, derived, geo


def gate_comparison(candidate, baseline, limits):
    if not candidate.get('available') or not baseline.get('available'):
        return dict(available=False, passes_all_conditions=False, reason='A required sealed run is unavailable')
    c, b = candidate, baseline
    ct, bt = c['timed_tracking'], b['timed_tracking']
    cy, by = c['motion']['late'], b['motion']['late']
    ca, ba = c['attitude'], b['attitude']
    cmd, qp = c['commands'], c['optimizer']
    checks = dict(
        full44_no_failure=bool(c['completed'] and c['failure'] is None and c['full44_coverage']),
        baseline_full44_no_failure=bool(b['completed'] and b['failure'] is None and b['full44_coverage']),
        baseline_historical_exact=bool((b.get('historical_parity') or {}).get('all_shared_fields_exact')),
        moving_window_complete=bool(c['moving_coverage'] and b['moving_coverage']),
        late_window_complete=bool(cy.get('complete') and by.get('complete')),
        late_heading_valid=bool(cy.get('all_heading_samples_valid') and by.get('all_heading_samples_valid')),
        same_frozen_numeric_bank=c['fixed_bank_numeric_sha256'] == b['fixed_bank_numeric_sha256'],
        commands_finite_and_previous_actual=bool(cmd['all_finite'] and cmd['actual_previous_input_matches']),
        final_command_box=bool(cmd['applied_box_pass'] and cmd['requested_box_pass']),
        final_command_slew=bool(cmd['actual_slew_pass']),
        secondary_balance_equality=bool(qp.get('available') and qp.get('equality_all_finite') and
            qp.get('secondary_balance_extra_all_finite') and qp['equality_residual_max'] <= 1e-8 and
            qp['secondary_balance_extra_max'] <= 1e-8))
    def compare(name, a, bound):
        checks[name] = bool(a is not None and bound is not None and np.isfinite(a) and np.isfinite(bound) and a <= bound+1e-12)
    compare('late_yaw_peak', cy.get('world_z_peak_abs_rad_s'),
        None if not by.get('available') else limits['yaw_peak_abs_ratio_max']*by['world_z_peak_abs_rad_s'])
    compare('late_heading_absolute_travel', cy.get('heading_absolute_travel_rad'),
        None if not by.get('available') else limits['heading_absolute_travel_ratio_max']*by['heading_absolute_travel_rad'])
    compare('moving_timed_xy_RMS', ct['moving_rms_m'],
        None if bt['moving_rms_m'] is None else limits['timed_xy_rms_ratio_max']*bt['moving_rms_m']+limits['timed_xy_rms_add_m'])
    compare('full_timed_xy_peak', ct['full_peak_at44_m'],
        None if bt['full_peak_at44_m'] is None else bt['full_peak_at44_m']+limits['timed_xy_peak_add_m'])
    compare('final_timed_xy', ct['final_at44_m'],
        None if bt['final_at44_m'] is None else bt['final_at44_m']+limits['final_xy_add_m'])
    compare('raw_reference_lean_peak', ca['peak_raw_reference_error_deg'],
        ba['peak_raw_reference_error_deg']+limits['relative_lean_peak_add_deg'])
    compare('raw_reference_lean_RMS', ca['rms_raw_reference_error_deg'],
        limits['relative_lean_rms_ratio_max']*ba['rms_raw_reference_error_deg']+limits['relative_lean_rms_add_deg'])
    compare('absolute_tilt_peak', ca['peak_absolute_deg'], ba['peak_absolute_deg']+limits['absolute_tilt_peak_add_deg'])
    changes = []
    for x, y in zip(c['turns'], b['turns']):
        assert x['event_id'] == y['event_id']
        a, z = x['geometric_component_overshoot_m'], y['geometric_component_overshoot_m']
        pass_turn = bool(x['window_complete'] and y['window_complete'] and a is not None and z is not None and
                         a <= z+limits['each_turn_overshoot_add_m']+1e-12)
        changes.append(dict(event_id=x['event_id'], passed=pass_turn,
            geometric_overshoot_change_m=None if a is None or z is None else a-z,
            candidate_lag_s=x['actual_reversal_lag_s'], baseline_lag_s=y['actual_reversal_lag_s'],
            candidate_reversal_missing=x['actual_reversal_missing_in_window']))
    checks['all_six_turn_overshoots'] = len(changes) == 6 and all(x['passed'] for x in changes)
    checks['conditional_original_settling'] = bool(not b['original_settling']['passed'] or c['original_settling']['passed'])
    return dict(available=True, passes_all_conditions=all(checks.values()), checks=checks, turn_changes=changes,
        failed_checks=[k for k, v in checks.items() if not v],
        scope='Predeclared one-route comparison; equalities are local model guarantees, not proof of balance. No default adoption or significance claim.')


def stage_inputs(output, sources, inputs, declaration):
    output.mkdir(parents=True, exist_ok=False)
    records = []
    for source in sources:
        target = output/'source_snapshot'/source.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        records.append(dict(origin=str(source.relative_to(ROOT)), snapshot=str(target.relative_to(output)), sha256=sha(source)))
    declaration.update(registered_utc=datetime.now(timezone.utc).isoformat(), sources=records,
        input_sha256={str(p.relative_to(ROOT)): sha(p) for p in inputs})
    save(output/'registered_before_arithmetic.json', declaration)
    return declaration


def write_tables(output, report):
    fields = ['case_id','available','completed','duration_s','moving_XY_RMS_mm','peak_XY_mm','final_XY_mm',
              'peak_raw_lean_deg','peak_absolute_tilt_deg','late_world_yaw_peak_rad_s','late_absolute_heading_turns',
              'slew_violation','secondary_equality','model_seconds_p95','allocation_seconds_p95']
    with (output/'cases.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields);writer.writeheader()
        for case, row in report['cases'].items():
            value = dict(case_id=case, available=row.get('available', False))
            if row.get('available'):
                qp=row['optimizer'];cost=qp.get('model_confidence_and_cost', {})
                value.update(completed=row['completed'],duration_s=row['observed_duration_s'],
                    moving_XY_RMS_mm=None if row['timed_tracking']['moving_rms_m'] is None else 1000*row['timed_tracking']['moving_rms_m'],
                    peak_XY_mm=1000*row['timed_tracking']['full_peak_m'],final_XY_mm=1000*row['timed_tracking']['final_m'],
                    peak_raw_lean_deg=row['attitude']['peak_raw_reference_error_deg'],peak_absolute_tilt_deg=row['attitude']['peak_absolute_deg'],
                    late_world_yaw_peak_rad_s=row['motion']['late'].get('world_z_peak_abs_rad_s'),
                    late_absolute_heading_turns=row['motion']['late'].get('heading_absolute_travel_turns'),
                    slew_violation=row['commands']['slew_max_violation'],secondary_equality=qp.get('equality_residual_max'),
                    model_seconds_p95=cost.get('model_seconds',{}).get('p95'),allocation_seconds_p95=cost.get('allocation_seconds',{}).get('p95'))
            writer.writerow(value)
    with (output/'turns.csv').open('w', newline='') as stream:
        fields=['case_id','event_id','window_complete','geometric_component_overshoot_m','actual_reversal_lag_s','actual_reversal_missing_in_window']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for case,row in report['cases'].items():
            for turn in row.get('turns',[]):writer.writerow(dict(case_id=case,**{k:turn[k] for k in fields if k!='case_id'}))


def reduce(output):
    helper_path, turns_path = dependencies()
    main_protocol = json.loads((BASE/'protocol.json').read_text())
    ablation_path = BASE/'ablation/protocol.json'
    ablation_protocol = json.loads(ablation_path.read_text()) if ablation_path.is_file() else None
    jobs = [(BASE, main_protocol, job) for job in main_protocol['cases']]
    if ablation_protocol is not None:
        jobs += [(BASE/'ablation', ablation_protocol, job) for job in ablation_protocol['cases']]
    replay_path=Path(__file__).with_name('replay_projection.py')
    sources=[Path(__file__),replay_path,LEGACY,helper_path,turns_path,OLD_PROTOCOL,BASE/'protocol.json']
    if ablation_path.is_file():sources.append(ablation_path)
    program_path=ROOT/main_protocol['historical_baseline_paths']['m100_B']
    inputs=[program_path]
    availability={};folders={}
    for root, protocol, job in jobs:
        case=job['case_id'];folder=root/'campaign/runs'/case;folders[case]=folder
        required=[folder/name for name in ('precision.npz','rollout.npz','summary.json','provenance.json','governor_diagnostics.json','artifact_manifest.json')]
        present=all(p.is_file() for p in required)
        availability[case]=dict(available=present, reason=None if present else ('execution_failure' if (folder/'failure.json').exists() else 'not_run_or_not_sealed'),
            case_source=str(root.relative_to(ROOT)), planned_phase_rad=job['phase_rad'], variant=job['variant'])
        if present:
            manifest=json.loads((folder/'artifact_manifest.json').read_text())
            for row in manifest['files']:
                if sha(folder/row['path'])!=row['sha256']:raise ValueError(f'Case manifest changed: {case}/{row["path"]}')
            inputs+=required
            historical=folder/'historical_precision_comparison.json'
            if historical.is_file():inputs.append(historical)
        elif (folder/'failure.json').is_file():inputs.append(folder/'failure.json')
    declared=stage_inputs(output,sources,inputs,dict(iteration='I-56',scope=__doc__,
        interpretation='Deterministic policy comparisons, one trajectory per registered cell; no statistical significance. Ablation cannot be promoted after outcomes.',
        main_acceptance=main_protocol['acceptance'],availability_before_reading=availability,
        metric_conventions=dict(attitude_RMS_window_s=[0.,44.],yaw_window_s=main_protocol['acceptance']['yaw_window_s'],
            command_constraints='All emitted including first/terminal; held exposure excludes terminal.',
            acceleration='Backward differences of world velocities at5ms/50ms; not instant/contactsolver acceleration.',
            reference_events='Only corrected six event definitions, not their stored old actual outcomes/retry metadata.')))
    metrics=pure_module(LEGACY,'_i56_fixed_i49_metrics');helper=pure_module(helper_path,'_i56_fixed_i41_events')
    events=[{k:r[k] for k in EVENT_FIELDS} for r in json.loads(turns_path.read_text())['rows']]
    if len(events)!=6:raise ValueError('Expected all six corrected reference events')
    with np.load(program_path,allow_pickle=False)as z:program={k:z[k].copy()for k in REFERENCE_FIELDS}
    report=dict(iteration='I-56',cases={},comparisons={},raw_target_unmodified=True,
        definitions=declared['metric_conventions'],no_new_physics_controller_or_observer_calls=True)
    data={};derived={}
    portable=output/'portable_inputs';portable.mkdir()
    np.savez_compressed(portable/'reference_program.npz',**program)
    save(portable/'events.json',events)
    portable_index=dict(iteration='I-56',timestamps='All measured5ms states, without downsampling.',
        sources=dict(reducer=str((output/'source_snapshot'/Path(__file__).relative_to(ROOT)).relative_to(output)),
            legacy_metrics=str((output/'source_snapshot'/LEGACY.relative_to(ROOT)).relative_to(output)),
            event_helper=str((output/'source_snapshot'/helper_path.relative_to(ROOT)).relative_to(output))),
        reference_program='portable_inputs/reference_program.npz',events='portable_inputs/events.json',
        reference_program_original_precision_sha256=sha(program_path),cases={},unavailable_cases={})
    for root,protocol,job in jobs:
        case=job['case_id']
        if not availability[case]['available']:
            report['cases'][case]=availability[case];portable_index['unavailable_cases'][case]=availability[case];continue
        folder=folders[case];raw=load(folder/'precision.npz');rollout=load(folder/'rollout.npz')
        summary=json.loads((folder/'summary.json').read_text());prov=json.loads((folder/'provenance.json').read_text());gov=json.loads((folder/'governor_diagnostics.json').read_text())
        row,sig,geo=case_metrics(raw,rollout,summary,prov,gov,protocol,program,events,metrics,helper)
        row.update(case_id=case,phase_rad=job['phase_rad'],variant=job['variant'],case_source=str(root.relative_to(ROOT)))
        historical=folder/'historical_precision_comparison.json'
        row['historical_parity']=json.loads(historical.read_text()) if historical.is_file() else None
        report['cases'][case]=row;data[case]=raw;derived[case]=sig
        np.savez_compressed(output/(case+'_signals.npz'),**sig)
        np.savez_compressed(output/(case+'_geometry.npz'),**geo)
        projection=portable/(case+'_precision_projection.npz')
        np.savez_compressed(projection,**{k:raw[k] for k in PROJECTION_FIELDS})
        compressed_diagnostics=portable/(case+'_diagnostics.json.gz')
        selected_rows=[dict(time_s=r['time_s'],diagnostics={k:r['diagnostics'][k] for k in DIAGNOSTIC_FIELDS}
            if r['diagnostics'] else {}) for r in gov]
        with gzip.open(compressed_diagnostics,'wt',encoding='utf8') as stream:
            json.dump(safe(selected_rows),stream,allow_nan=False,separators=(',',':'))
        metadata=portable/(case+'_metadata.json')
        save(metadata,dict(summary=summary,provenance=prov,protocol=protocol,historical_parity=row['historical_parity'],
            original_precision_path=str((folder/'precision.npz').relative_to(ROOT)),
            original_precision_sha256=sha(folder/'precision.npz'),projection_fields=PROJECTION_FIELDS,
            projection_exact=all(np.array_equal(raw[k],load(projection)[k],equal_nan=True) for k in PROJECTION_FIELDS)))
        portable_index['cases'][case]=dict(projection=str(projection.relative_to(output)),
            projection_sha256=sha(projection),diagnostics=str(compressed_diagnostics.relative_to(output)),
            diagnostics_sha256=sha(compressed_diagnostics),metadata=str(metadata.relative_to(output)),metadata_sha256=sha(metadata),
            original_precision_sha256=sha(folder/'precision.npz'))
    for phase in PHASES:
        b,h=phase+'_B',phase+'_H'
        if b in report['cases'] and h in report['cases']:
            comparison=gate_comparison(report['cases'][h],report['cases'][b],main_protocol['acceptance'])
            comparison.update(baseline=b,candidate=h,registered_extension_gate=True)
            report['comparisons'][h+'_vs_'+b]=comparison
    if 'm100_A' in report['cases']:
        comparison=gate_comparison(report['cases']['m100_A'],report['cases']['m100_B'],main_protocol['acceptance'])
        comparison.update(baseline='m100_B',candidate='m100_A',registered_extension_gate=False,
            interpretation='Predeclared no-yaw/no-momentum ablation; not a fallback candidate selected after seeing results.')
        report['comparisons']['m100_A_vs_m100_B']=comparison
    initial=report['comparisons'].get('m100_H_vs_m100_B',{})
    report['broader_campaign_eligible']=bool(initial.get('available') and initial.get('passes_all_conditions'))
    report['all_three_phase_screens_pass']=all(report['comparisons'].get(phase+'_H_vs_'+phase+'_B',{}).get('passes_all_conditions',False)for phase in PHASES)
    report['default_adoption']=False
    save(output/'portable_inputs.json',portable_index)
    save(output/'report.json',report);write_tables(output,report)
    for path,expected in declared['input_sha256'].items():
        if sha(ROOT/path)!=expected:raise ValueError(f'Registered input changed during reduction: {path}')
    for source in declared['sources']:
        if sha(ROOT/source['origin'])!=source['sha256']:raise ValueError(f'Registered source changed: {source["origin"]}')
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),bytes=p.stat().st_size,sha256=sha(p))
        for p in sorted(output.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))
    print(json.dumps(safe(dict(available_cases=[k for k,v in report['cases'].items()if v.get('available')],
        broader_campaign_eligible=report['broader_campaign_eligible'],comparisons={k:dict(available=v.get('available'),passed=v.get('passes_all_conditions'),failed=v.get('failed_checks'))for k,v in report['comparisons'].items()})),indent=2))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    reduce(args.output.resolve())


if __name__=='__main__':
    main()
