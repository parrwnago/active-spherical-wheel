"""Analyze contact estimates and the commands around trajectory corrections.

These calculations run after a trial. Simulator contact data are used only
for evaluation. A rolling-velocity mismatch is evidence that the current
rolling explanation is incomplete; it does not distinguish slip, contact
geometry error, actuator response, or measurement error by itself.

Use ``python -m spherical_wheel.contact_diagnostics rollout.npz --output DIR``
on a run recorded with ``record_diagnostics=True``.
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


SUPPORTS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
_COLORS = ('#2878b5', '#df7e31', '#269c83', '#a06ac2')
_COMPONENTS = (
    ('lqr_command', 'LQR', '#2878b5'),
    ('rolling_feedforward', 'Rolling feedforward', '#df7e31'),
    ('support_command', 'Holding torque', '#269c83'),
    ('transition_command_delta', 'Transition correction', '#a06ac2'),
    ('heading_command', 'Heading correction', '#ad7d22'),
    ('fade_command_delta', 'Unloaded correction', '#7d7d7d'),
)


def _array(log, key, count, shape=(), fill=np.nan):
    if key not in log:
        return np.full((count,)+shape, fill)
    value = np.asarray(log[key])
    expected = (count,)+shape
    if value.shape != expected:
        raise ValueError(f'{key} has shape {value.shape}; expected {expected}')
    return value


def _bounds(log, count, bounds=None):
    value = np.asarray(log.get('input_bounds') if bounds is None else bounds, float)
    if value.shape == (5,):
        value = np.broadcast_to(value, (count, 5))
    if value.shape != (count, 5) or not np.isfinite(value).all() or np.any(value <= 0):
        raise ValueError('Positive input_bounds must have shape (5,) or (samples, 5)')
    return value


def _angles(first, second):
    denominator = np.linalg.norm(first, axis=1)*np.linalg.norm(second, axis=1)
    cosine = np.full(len(first), np.nan)
    valid = denominator > 1e-12
    cosine[valid] = np.sum(first[valid]*second[valid], axis=1)/denominator[valid]
    return np.degrees(np.arccos(np.clip(cosine, -1., 1.)))


def _shares(loads):
    total = np.sum(loads, axis=1)
    result = np.zeros_like(loads, dtype=float)
    np.divide(loads, total[:, None], out=result, where=total[:, None] > 1e-9)
    return result


def diagnostic_signals(log, bounds=None):
    """Derive aligned diagnostic traces without changing any controller input.

    The primary mismatch is the load-weighted RMS of per-contact tangential
    velocity residuals. The norm of their weighted vector mean could hide two
    opposite residuals. Current saturation describes the command for the next
    interval; previous saturation contextualizes the measured motion at t.
    """
    time = np.asarray(log['time_s'], float)
    if time.ndim != 1 or len(time) < 2 or not np.isfinite(time).all() or np.any(np.diff(time) <= 0):
        raise ValueError('time_s must contain at least two finite, strictly increasing times')
    count = len(time)
    required = ('estimated_contact_point_m', 'estimated_support_loads_N',
                'contact_velocity_residual_rms_m_s', 'velocity_consistency_valid')
    missing = [key for key in required if key not in log]
    if missing:
        raise ValueError(f'Missing diagnostic fields {missing}; record the run with record_diagnostics=True')
    limit = _bounds(log, count, bounds)
    qpos = np.asarray(log['qpos'], float)
    if qpos.ndim != 2 or qpos.shape[0] != count or qpos.shape[1] < 3:
        raise ValueError('qpos must contain one body position per sample')
    reference = _array(log, 'reference_xy', count, (2,))
    requested = _array(log, 'command', count, (5,))
    applied = _array(log, 'input', count, (5,))
    estimated_point = _array(log, 'estimated_contact_point_m', count, (3,))
    estimated_loads = _array(log, 'estimated_support_loads_N', count, (4,))
    estimated_shares = _shares(estimated_loads)
    measured = _array(log, 'measured_body_velocity_m_s', count, (3,))
    predicted = _array(log, 'predicted_body_velocity_m_s', count, (3,))
    valid = _array(log, 'velocity_consistency_valid', count).astype(bool)
    residual = _array(log, 'contact_velocity_residual_rms_m_s', count).astype(float)
    residual[~valid] = np.nan
    estimated_offset = estimated_point-qpos[:, :3]
    contact_step = np.r_[0., np.linalg.norm(np.diff(estimated_offset, axis=0), axis=1)]
    contact_step[~valid] = np.nan
    contact_step[1:][~valid[:-1]] = np.nan
    requested_fraction = np.abs(requested)/limit
    saturated = np.asarray(log.get('actuator_saturated', requested_fraction > 1.+1e-8), bool)
    if saturated.shape != (count, 5):
        raise ValueError('actuator_saturated must have shape (samples, 5)')
    if 'previous_actuator_saturated' in log:
        previous_saturated = _array(log, 'previous_actuator_saturated', count, (5,)).astype(bool)
    else:
        previous_saturated = np.vstack((np.zeros((1, 5), bool), saturated[:-1]))
    command_step = np.r_[0., np.max(np.abs(np.diff(requested, axis=0))/limit[1:], axis=1)]
    applied_step = np.r_[0., np.max(np.abs(np.diff(applied, axis=0))/limit[1:], axis=1)]
    normal = _array(log, 'estimated_normal', count, (3,))
    true_normal = _array(log, 'ground_normal', count, (3,))
    true_point = _array(log, 'contact_point_m', count, (3,))
    true_loads = _array(log, 'support_loads_N', count, (4,))
    truth_contact_valid = np.isfinite(true_loads).all(axis=1) & (np.sum(true_loads, axis=1) > 1e-9)
    point_error = np.linalg.norm(estimated_point-true_point, axis=1)
    # measure_contact uses the wheel centre as a placeholder when unloaded;
    # comparing that placeholder to a spherical support invents a radius-size
    # point error. Presence disagreement is reported separately below.
    point_error[~(valid & truth_contact_valid)] = np.nan
    tracking_velocity = _array(log, 'tracking_velocity_error_m_s', count, (2,))
    reference_qvel = _array(log, 'reference_qvel', count, (9,))
    measured_qvel = np.asarray(log.get('qvel', np.full((count, 9), np.nan)))
    if measured_qvel.shape != (count, 9):
        raise ValueError('qvel must have shape (samples, 9)')
    result = {
        'time_s': time, 'input_bounds': limit,
        'tracking_error_m': np.linalg.norm(qpos[:, :2]-reference, axis=1),
        'tracking_velocity_error_m_s': np.linalg.norm(tracking_velocity, axis=1),
        'reference_joint_speed_max_rad_s': np.max(np.abs(reference_qvel[:, 6:9]), axis=1),
        'measured_joint_speed_max_rad_s': np.max(np.abs(measured_qvel[:, 6:9]), axis=1),
        'rolling_mismatch_rms_m_s': residual,
        'rolling_mismatch_max_m_s': _array(log, 'contact_velocity_residual_max_m_s', count),
        'velocity_consistency_valid': valid,
        'truth_contact_valid': truth_contact_valid,
        'kinematic_inconsistency': _array(log, 'kinematic_inconsistency', count, fill=False).astype(bool),
        'measured_body_speed_m_s': np.linalg.norm(measured, axis=1),
        'predicted_body_speed_m_s': np.linalg.norm(predicted, axis=1),
        'estimated_contact_offset_m': estimated_offset,
        'estimated_contact_step_m': contact_step,
        'contact_point_error_m': point_error,
        'estimated_support_shares': estimated_shares,
        'true_support_shares': _shares(true_loads),
        'estimated_share_step': np.r_[0., np.sum(np.abs(np.diff(estimated_shares, axis=0)), axis=1)],
        'normal_error_deg': _angles(normal, true_normal),
        'estimated_normal_step_deg': np.r_[0., _angles(normal[1:], normal[:-1])],
        'requested_fraction': requested_fraction,
        'applied_fraction': np.abs(applied)/limit,
        'command_step_fraction': command_step,
        'applied_command_step_fraction': applied_step,
        'torque_saturated': np.any(saturated[:, :3], axis=1),
        'belt_saturated': np.any(saturated[:, 3:], axis=1),
        'previous_torque_saturated': np.any(previous_saturated[:, :3], axis=1),
        'previous_belt_saturated': np.any(previous_saturated[:, 3:], axis=1),
        'truth_slip_speed_m_s': _array(log, 'slip_speed_m_s', count),
    }
    for key, _, _ in _COMPONENTS:
        result[key+'_fraction'] = np.max(np.abs(_array(log, key, count, (5,)))/limit, axis=1)
    component_keys = ('equilibrium_command',)+tuple(item[0] for item in _COMPONENTS)
    if all(key in log for key in component_keys):
        reconstructed = sum((_array(log, key, count, (5,)) for key in component_keys),
                            start=np.zeros((count, 5)))
        result['command_reconstruction_error_fraction'] = np.max(np.abs(requested-reconstructed)/limit, axis=1)
    else:
        result['command_reconstruction_error_fraction'] = np.full(count, np.nan)
    return result


def _finite_max(value):
    finite = np.asarray(value)[np.isfinite(value)]
    return float(np.max(finite)) if len(finite) else None


def _finite_percentile(value, percentile):
    finite = np.asarray(value)[np.isfinite(value)]
    return float(np.percentile(finite, percentile)) if len(finite) else None


def correction_events(log, bounds=None, count=3, separation_s=1.5, ignore_initial_s=1., window_s=1.):
    """Select spaced diagnostic events and report co-occurring signals.

    Tracking deviation, requested-command change, and rolling mismatch each
    nominate their strongest local peaks. Nearby nominations merge. These
    event labels identify observations, not inferred causes.
    """
    if count < 1 or separation_s < 0 or window_s <= 0:
        raise ValueError('count and window_s must be positive; separation_s must be nonnegative')
    signals = diagnostic_signals(log, bounds)
    time = signals['time_s']
    sources = (
        ('tracking_deviation', signals['tracking_error_m']),
        ('command_change', signals['command_step_fraction']),
        ('rolling_mismatch', signals['rolling_mismatch_rms_m_s']),
    )
    candidates = []
    for order, (name, raw) in enumerate(sources):
        values = np.asarray(raw).copy()
        values[~np.isfinite(values)] = -np.inf
        values[time < time[0]+ignore_initial_s] = -np.inf
        local = np.flatnonzero((values >= np.r_[-np.inf, values[:-1]]) &
                               (values > np.r_[values[1:], -np.inf]) & (values > 0))
        ranked, selected = local[np.argsort(values[local])[::-1]], []
        for index in ranked:
            if all(abs(time[index]-time[other]) >= separation_s for other in selected):
                candidates.append((len(selected), order, int(index), name))
                selected.append(int(index))
                if len(selected) >= count:
                    break
    chosen = []
    for _, _, index, reason in sorted(candidates):
        close = [event for event in chosen if abs(time[index]-time[event['index']]) < separation_s]
        if close:
            if reason not in close[0]['reasons']:
                close[0]['reasons'].append(reason)
        elif len(chosen) < count:
            chosen.append({'index': index, 'reasons': [reason]})
    events = []
    for number, item in enumerate(sorted(chosen, key=lambda event: event['index']), 1):
        index = item['index']
        selected = abs(time-time[index]) <= window_s
        valid = selected & signals['velocity_consistency_valid']
        event = {
            'event': number, 'index': index, 'time_s': float(time[index]),
            'reasons': item['reasons'], 'window_s': [float(time[selected][0]), float(time[selected][-1])],
            'tracking_error_mm': float(signals['tracking_error_m'][index]*1000),
            'rolling_mismatch_rms_m_s': (float(signals['rolling_mismatch_rms_m_s'][index]) if
                                       np.isfinite(signals['rolling_mismatch_rms_m_s'][index]) else None),
            'contact_point_error_mm': (float(signals['contact_point_error_m'][index]*1000) if
                                       np.isfinite(signals['contact_point_error_m'][index]) else None),
            'normal_error_deg': (float(signals['normal_error_deg'][index]) if
                                 np.isfinite(signals['normal_error_deg'][index]) else None),
            'window': {
                'max_tracking_error_mm': _finite_max(signals['tracking_error_m'][selected]*1000),
                'max_rolling_mismatch_rms_m_s': _finite_max(signals['rolling_mismatch_rms_m_s'][valid]),
                'max_contact_point_error_mm': _finite_max(signals['contact_point_error_m'][valid]*1000),
                'max_estimated_contact_step_mm': _finite_max(signals['estimated_contact_step_m'][valid]*1000),
                'max_estimated_load_share_step': _finite_max(signals['estimated_share_step'][selected]),
                'max_normal_error_deg': _finite_max(signals['normal_error_deg'][selected]),
                'max_command_step_fraction': _finite_max(signals['command_step_fraction'][selected]),
                'max_applied_command_step_fraction': _finite_max(signals['applied_command_step_fraction'][selected]),
                'max_reference_joint_speed_rad_s': _finite_max(signals['reference_joint_speed_max_rad_s'][selected]),
                'max_measured_joint_speed_rad_s': _finite_max(signals['measured_joint_speed_max_rad_s'][selected]),
                'requested_torque_saturation_fraction': float(np.mean(signals['torque_saturated'][selected])),
                'requested_belt_saturation_fraction': float(np.mean(signals['belt_saturated'][selected])),
                'previous_torque_saturation_fraction': float(np.mean(signals['previous_torque_saturated'][selected])),
                'previous_belt_saturation_fraction': float(np.mean(signals['previous_belt_saturated'][selected])),
                'kinematic_inconsistency_fraction': float(np.mean(signals['kinematic_inconsistency'][valid])) if valid.any() else None,
                'support_presence_disagreement_fraction': float(np.mean(
                    signals['velocity_consistency_valid'][selected] != signals['truth_contact_valid'][selected])),
            },
        }
        events.append(event)
    return events


def _plot(log, signals, events, output, title, interval=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    time = signals['time_s']
    selected = np.ones(len(time), bool) if interval is None else ((time >= interval[0]) & (time <= interval[1]))
    shown = time[selected]
    figure, axes = plt.subplots(4, 2, figsize=(15, 12), constrained_layout=True)
    axes = axes.ravel()
    reference, position = np.asarray(log['reference_xy']), np.asarray(log['qpos'])[:, :2]
    axes[0].plot(reference[:, 0], reference[:, 1], '--', color='#df7e31', label='Planned')
    axes[0].plot(position[selected, 0], position[selected, 1], color='#1496a6', label='Measured body path')
    for event in events:
        if selected[event['index']]:
            point = position[event['index']]
            axes[0].scatter(*point, s=30, color='#c73e3e')
            axes[0].annotate(str(event['event']), point, xytext=(4, 4), textcoords='offset points')
    axes[0].set(title='Path and selected correction events', xlabel='World x (m)', ylabel='World y (m)', aspect='equal')
    axes[1].plot(shown, signals['tracking_error_m'][selected]*1000, color='#2878b5', label='Position error')
    axes[1].set(title='Tracking deviation', ylabel='mm')
    axes[2].plot(shown, signals['measured_body_speed_m_s'][selected], color='#2878b5', label='Measured tangent speed')
    axes[2].plot(shown, signals['predicted_body_speed_m_s'][selected], color='#df7e31', label='Predicted from drives/contact')
    axes[2].plot(shown, signals['rolling_mismatch_rms_m_s'][selected], color='#c73e3e', label='Per-contact residual RMS')
    axes[2].axhline(.02, color='#888888', linestyle=':', linewidth=1, label='Mismatch flag threshold')
    axes[2].set(title='Velocity consistency (mismatch is not proven slip)', ylabel='m/s')
    axes[3].plot(shown, signals['contact_point_error_m'][selected]*1000, color='#2878b5', label='Estimated vs truth point')
    axes[3].plot(shown, signals['estimated_contact_step_m'][selected]*1000, color='#df7e31', label='Estimated offset change / sample')
    axes[3].set(title='Contact-point evaluation (truth excluded from control)', ylabel='mm')
    for support, color, column in zip(SUPPORTS, _COLORS, range(4)):
        axes[4].plot(shown, signals['estimated_support_shares'][selected, column], color=color, label=support)
        axes[4].plot(shown, signals['true_support_shares'][selected, column], '--', color=color, alpha=.5, linewidth=1)
    axes[4].set(title='Load share: estimated solid, truth dashed', ylabel='Share', ylim=(-.03, 1.03))
    fraction = signals['requested_fraction'][selected]
    axes[5].plot(shown, np.max(fraction[:, :3], axis=1), color='#2878b5', label='Largest torque request / limit')
    axes[5].plot(shown, np.max(fraction[:, 3:], axis=1), color='#df7e31', label='Largest belt request / limit')
    axes[5].axhline(1., color='#888888', linestyle=':', linewidth=1)
    axes[5].set(title='New command saturation (motion reflects previous interval)', ylabel='Request / limit')
    axes[6].plot(shown, signals['normal_error_deg'][selected], color='#2878b5', label='Estimated vs truth normal')
    axes[6].plot(shown, signals['estimated_normal_step_deg'][selected], color='#df7e31', label='Estimated change / sample')
    axes[6].set(title='Normal estimate evaluation', ylabel='Degrees')
    for key, label, color in _COMPONENTS:
        values = signals[key+'_fraction'][selected]
        if np.isfinite(values).any() and np.nanmax(values) > 1e-10:
            axes[7].plot(shown, values, label=label, color=color, linewidth=1.2)
    axes[7].set(title='Command components: largest absolute normalized channel', ylabel='Component / limit')
    for number, axis in enumerate(axes):
        axis.grid(True, alpha=.2)
        handles, _ = axis.get_legend_handles_labels()
        if handles:
            axis.legend(fontsize=7, loc='best', ncols=2 if number in (4, 7) else 1)
        if number:
            axis.set(xlabel='Time (s)', xlim=(shown[0], shown[-1]))
            for event in events:
                if shown[0] <= event['time_s'] <= shown[-1]:
                    axis.axvline(event['time_s'], color='#c73e3e', alpha=.5, linewidth=.8)
    figure.suptitle(title, fontsize=13)
    figure.text(.01, -.015, 'Diagnostic co-occurrences do not establish cause. Contact truth is evaluation-only. '
                'Residuals use measured motion and previous applied drive inputs, not desired velocity.', fontsize=8)
    figure.savefig(output, dpi=140, bbox_inches='tight')
    plt.close(figure)


def _plot_commands(log, signals, event, output, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    time = signals['time_s']
    selected = (time >= event['window_s'][0]) & (time <= event['window_s'][1])
    shown, bounds = time[selected], signals['input_bounds'][selected]
    figure, axes = plt.subplots(5, 1, figsize=(13, 11), sharex=True, constrained_layout=True)
    components = (('equilibrium_command', 'Equilibrium', '#8f9c9c'),)+_COMPONENTS
    for column, (axis, name) in enumerate(zip(axes, ('Shaft torque', 'Shell A torque', 'Shell B torque',
                                                   'Belt A speed', 'Belt B speed'))):
        for key, label, color in components:
            if key in log:
                value = np.asarray(log[key])[selected, column]/bounds[:, column]
                if np.isfinite(value).any() and np.nanmax(np.abs(value)) > 1e-10:
                    axis.plot(shown, value, label=label, color=color, linewidth=1.2)
        axis.plot(shown, np.asarray(log['command'])[selected, column]/bounds[:, column],
                  color='#181818', linewidth=2., label='Requested total')
        axis.plot(shown, np.asarray(log['input'])[selected, column]/bounds[:, column],
                  '--', color='#c73e3e', linewidth=1.4, label='Applied total')
        axis.axvline(event['time_s'], color='#c73e3e', alpha=.5, linewidth=.8)
        axis.axhline(1., color='#888888', linestyle=':', linewidth=1)
        axis.axhline(-1., color='#888888', linestyle=':', linewidth=1)
        axis.set(title=name, ylabel='Command / limit', xlim=(shown[0], shown[-1]))
        axis.grid(True, alpha=.2)
        axis.legend(fontsize=7, ncols=4, loc='best')
    axes[-1].set_xlabel('Time (s)')
    figure.suptitle(f"{title}: signed command components around event {event['event']} "
                   f"at {event['time_s']:.3f} s", fontsize=13)
    figure.text(.01, -.015, 'Commands at t act during the next interval. Estimated rolling residuals at t '
                'use measured motion and the previous applied inputs. Dotted lines are actuator limits.', fontsize=8)
    figure.savefig(output, dpi=140, bbox_inches='tight')
    plt.close(figure)


def _plot_allocation(log, signals, event, output, title):
    """Expose extreme drive-rate references that can disappear after clipping."""
    if 'reference_qvel' not in log or 'qvel' not in log:
        return
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    time = signals['time_s']
    selected = (time >= event['window_s'][0]) & (time <= event['window_s'][1])
    shown = time[selected]
    figure, axes = plt.subplots(3, 1, figsize=(13, 8), sharex=True, constrained_layout=True)
    axes[0].plot(shown, signals['measured_body_speed_m_s'][selected], label='Measured tangent speed', color='#2878b5')
    axes[0].plot(shown, signals['predicted_body_speed_m_s'][selected], label='Predicted from measured drives', color='#df7e31')
    axes[0].plot(shown, signals['rolling_mismatch_rms_m_s'][selected], label='Rolling mismatch RMS', color='#c73e3e')
    axes[0].set(title='Measured-motion consistency', ylabel='m/s')
    for column, name, color in zip(range(6, 9), ('Shaft', 'Shell A', 'Shell B'), _COLORS):
        axes[1].plot(shown, np.asarray(log['reference_qvel'])[selected, column], color=color, label=f'{name} reference')
        axes[1].plot(shown, np.asarray(log['qvel'])[selected, column], '--', color=color, alpha=.7, label=f'{name} measured')
    axes[1].set(title='Rolling allocation references and measured joint speeds', ylabel='rad/s')
    for support, color, column in zip(SUPPORTS, _COLORS, range(4)):
        axes[2].plot(shown, signals['estimated_support_shares'][selected, column], color=color, label=support)
    axes[2].set(title='Estimated support load shares', ylabel='Share', xlabel='Time (s)', ylim=(-.03, 1.03))
    for axis in axes:
        axis.axvline(event['time_s'], color='#c73e3e', alpha=.5, linewidth=.8)
        axis.set_xlim(shown[0], shown[-1])
        axis.grid(True, alpha=.2)
        axis.legend(fontsize=8, ncols=3, loc='best')
    figure.suptitle(f"{title}: allocation around event {event['event']} at {event['time_s']:.3f} s", fontsize=13)
    figure.savefig(output, dpi=140, bbox_inches='tight')
    plt.close(figure)


def analyze_run(log, output, title='Contact and command diagnostics', bounds=None, event_count=3):
    """Write a diagnostic overview, annotated event plots, JSON, and event CSV."""
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    signals = diagnostic_signals(log, bounds)
    events = correction_events(log, bounds, count=event_count)
    valid = signals['velocity_consistency_valid']
    report = {
        'samples': len(signals['time_s']), 'duration_s': float(signals['time_s'][-1]-signals['time_s'][0]),
        'velocity_consistency_valid_fraction': float(np.mean(valid)),
        'rolling_mismatch_rms_p95_m_s': _finite_percentile(signals['rolling_mismatch_rms_m_s'][valid], 95),
        'contact_point_error_p95_mm': _finite_percentile(signals['contact_point_error_m'][valid]*1000, 95),
        'estimated_contact_step_p95_mm': _finite_percentile(signals['estimated_contact_step_m'][valid]*1000, 95),
        'requested_torque_saturation_fraction': float(np.mean(signals['torque_saturated'])),
        'requested_belt_saturation_fraction': float(np.mean(signals['belt_saturated'])),
        'max_requested_command_step_fraction': _finite_max(signals['command_step_fraction']),
        'max_applied_command_step_fraction': _finite_max(signals['applied_command_step_fraction']),
        'max_command_reconstruction_error_fraction': _finite_max(signals['command_reconstruction_error_fraction']),
        'max_reference_joint_speed_rad_s': _finite_max(signals['reference_joint_speed_max_rad_s']),
        'max_measured_joint_speed_rad_s': _finite_max(signals['measured_joint_speed_max_rad_s']),
        'kinematic_inconsistency_fraction': float(np.mean(signals['kinematic_inconsistency'][valid])) if valid.any() else None,
        'support_presence_disagreement_fraction': float(np.mean(valid != signals['truth_contact_valid'])),
        'interpretation': 'Rolling mismatch can reflect slip, geometry/model error, actuator response, or measurement error; '
                          'event labels describe co-occurrences and do not infer cause.',
        'events': events,
    }
    _plot(log, signals, events, output/'overview.png', title)
    for event in events:
        _plot(log, signals, events, output/f"event_{event['event']:02d}.png",
              f"{title}: event {event['event']} at {event['time_s']:.3f} s "
              f"({', '.join(event['reasons'])})", event['window_s'])
        _plot_commands(log, signals, event, output/f"event_{event['event']:02d}_commands.png", title)
        _plot_allocation(log, signals, event, output/f"event_{event['event']:02d}_allocation.png", title)
    with (output/'events.csv').open('w', newline='') as stream:
        fields = ['event', 'time_s', 'reasons', 'tracking_error_mm', 'rolling_mismatch_rms_m_s',
                  'contact_point_error_mm', 'normal_error_deg']+list(events[0]['window'] if events else [])
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for event in events:
            writer.writerow({key: value for key, value in {
                **event, **event['window'], 'reasons': ', '.join(event['reasons'])}.items() if key in fields})
    (output/'diagnostics.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('rollout', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--title', default='Contact and command diagnostics')
    parser.add_argument('--events', type=int, default=3)
    args = parser.parse_args(argv)
    with np.load(args.rollout) as saved:
        log = {key: np.asarray(value) for key, value in saved.items()}
    report = analyze_run(log, args.output, args.title, event_count=args.events)
    print(json.dumps({'output': str(args.output), 'events': len(report['events'])}))


if __name__ == '__main__':
    main()
