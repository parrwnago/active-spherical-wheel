"""Compare recorded rolling-allocation cases on the original timed paths.

This evaluates saved runs only. Reference constraints and measured joint
speeds are reported separately: a bound on feedforward references does not
bound the simulated robot's actual joint speed.
"""
from __future__ import annotations

import argparse
import csv
from functools import lru_cache
import hashlib
import json
from pathlib import Path

import numpy as np

from spherical_wheel.contact_diagnostics import diagnostic_signals

CASES = [('plain_figure8', phase) for phase in (.3, .9, -1.)]+[
    ('shape_triangle', .3), ('shape_line_and_back', .3)]
LABELS = ['Figure-eight\n+0.30 rad', 'Figure-eight\n+0.90 rad', 'Figure-eight\n−1.00 rad',
          'Triangle\n+0.30 rad', 'Out-and-back\n+0.30 rad']
DRIVES = ('shaft', 'shell_A', 'shell_B', 'belt_A', 'belt_B')


@lru_cache(maxsize=None)
def file_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def configuration_hashes(root):
    return sorted({hashlib.sha256(json.dumps(json.loads(path.read_text()), sort_keys=True).encode()).hexdigest()
                   for path in Path(root).glob('_workspaces/*/flat/model_config.json')})


def load_case(root, name, phase):
    directory = Path(root)/name/f'phase_{phase:+.2f}'
    summary = json.loads((directory/'summary.json').read_text())
    with np.load(directory/'rollout.npz') as saved:
        log = {key: np.asarray(value) for key, value in saved.items()}
    return directory, log, summary


def interval_s(root, log, override=None):
    if override is not None:
        value, source = float(override), 'command-line override'
    else:
        values = {json.loads(path.read_text())['simulation']['control_interval_s']
                  for path in Path(root).glob('_workspaces/*/flat/model_config.json')}
        if len(values) == 1:
            value, source = float(values.pop()), 'saved workspace model configuration'
        elif len(values) > 1:
            raise ValueError('Workspace control intervals differ; provide --control-interval-s')
        else:
            # Single-precision time storage introduces small errors in adjacent
            # intervals. This fallback is explicit; prefer saved configuration.
            value, source = round(float(np.median(np.diff(log['time_s']))), 6), 'rounded median saved sample interval'
    if not np.isfinite(value) or value <= 0:
        raise ValueError('Control interval must be finite and positive')
    return value, source


def drive_reference(log):
    if 'allocation_drive_reference' in log:
        return np.asarray(log['allocation_drive_reference'], float)
    return np.column_stack((log['reference_qvel'][:, 6:9], log['rolling_feedforward'][:, 3:5])).astype(float)


def case_metrics(log, summary, variant, speed_limits, acceleration_limits, dt):
    signals = diagnostic_signals(log)
    drive = drive_reference(log)
    speed_ratio = np.abs(drive)/speed_limits
    acceleration_ratio = np.abs(np.diff(drive, axis=0))/(dt*acceleration_limits)
    speed_violations = np.any(speed_ratio > 1.+1e-5, axis=1)
    acceleration_violations = np.any(acceleration_ratio > 1.+1e-5, axis=1)
    valid = signals['velocity_consistency_valid'] & signals['truth_contact_valid']
    mismatch = signals['rolling_mismatch_rms_m_s'][valid]
    actual = np.abs(np.asarray(log['qvel'])[:, 6:9])
    root = {
        'route': summary['route']['name'], 'phase_rad': summary['route']['start_phase_rad'], 'variant': variant,
        'completed': summary['completed'], 'failure': summary['failure'],
        'duration_s': summary['completed_duration_s'], 'requested_duration_s': summary['requested_duration_s'],
        'rms_moving_mm': 1000*summary['tracking']['rms_error_while_moving_m'],
        'max_error_mm': 1000*summary['tracking']['max_error_m'],
        'final_error_mm': 1000*summary['tracking']['final_error_m'],
        'max_attitude_error_deg': summary['attitude']['max_error_from_reference_deg'],
        'max_reference_joint_speed_rad_s': float(np.max(np.abs(drive[:, :3]))),
        'max_measured_joint_speed_rad_s': float(np.max(actual)),
        'max_reference_speed_ratio': float(np.max(speed_ratio)),
        'max_reference_acceleration_ratio': float(np.max(acceleration_ratio)),
        'reference_speed_violation_samples': int(speed_violations.sum()),
        'reference_acceleration_violation_steps': int(acceleration_violations.sum()),
        'reference_speed_violation_fraction': float(np.mean(speed_violations)),
        'reference_acceleration_violation_fraction': float(np.mean(acceleration_violations)),
        'max_requested_command_step_fraction': float(np.max(signals['command_step_fraction'])),
        'max_requested_command_fraction': float(np.max(signals['requested_fraction'])),
        'max_requested_torque_fraction': float(np.max(signals['requested_fraction'][:, :3])),
        'max_requested_belt_fraction': float(np.max(signals['requested_fraction'][:, 3:])),
        'p95_applied_command_step_fraction': float(np.percentile(signals['applied_command_step_fraction'], 95)),
        'max_applied_command_step_fraction': float(np.max(signals['applied_command_step_fraction'])),
        'requested_torque_saturation_fraction': float(np.mean(signals['torque_saturated'])),
        'requested_belt_saturation_fraction': float(np.mean(signals['belt_saturated'])),
        'rolling_consistency_mismatch_p95_m_s': float(np.nanpercentile(mismatch, 95)) if len(mismatch) else None,
        'passed_all_gates': summary['mars']['passed'],
        'failed_gates': [key for key, passed in summary['mars']['gates'].items() if not passed],
        'contact_guarded_calls': summary['contact_input_audit']['checked_controller_calls'],
    }
    if 'allocation_active_bounds' in log:
        root['allocation_active_bounds_fraction'] = float(np.mean(np.any(log['allocation_active_bounds'], axis=1)))
    else:
        root['allocation_active_bounds_fraction'] = None
    for key in ('allocation_residual_rms_m_s', 'allocation_velocity_error_m_s'):
        if key in log:
            values = np.asarray(log[key])
            if values.ndim == 2:
                values = np.linalg.norm(values, axis=1)
            finite = values[np.isfinite(values)]
            root[key+'_p95'] = float(np.percentile(finite, 95)) if len(finite) else None
        else:
            root[key+'_p95'] = None
    for key in ('allocation_effort_weight_m_s', 'allocation_change_weight_m_s',
                'allocation_authority_m_s', 'allocation_effort_blend'):
        values = np.asarray(log.get(key, []), float)
        finite = values[np.isfinite(values)]
        for label, percentile in (('min', 0), ('p50', 50), ('p95', 95), ('max', 100)):
            root[key+'_'+label] = float(np.percentile(finite, percentile)) if len(finite) else None
    root['per_drive'] = {name: {
        'reference_abs_max': float(np.max(np.abs(drive[:, column]))),
        'reference_abs_p95': float(np.percentile(np.abs(drive[:, column]), 95)),
        'reference_abs_p99': float(np.percentile(np.abs(drive[:, column]), 99)),
        'reference_step_abs_max': float(np.max(np.abs(np.diff(drive[:, column])))),
        'speed_limit': float(speed_limits[column]),
        'acceleration_limit': float(acceleration_limits[column]),
        'reference_speed_max_ratio': float(np.max(speed_ratio[:, column])),
        'reference_acceleration_max_ratio': float(np.max(acceleration_ratio[:, column])),
        **({'measured_abs_max_rad_s': float(np.max(actual[:, column])),
            'measured_abs_p99_rad_s': float(np.percentile(actual[:, column], 99))} if column < 3 else {}),
    } for column, name in enumerate(DRIVES)}
    return root


def matched_path(first_log, first_summary, second_log, second_summary):
    first_time, second_time = first_log['time_s'], second_log['time_s']
    overlap = (second_time >= first_time[0]) & (second_time <= first_time[-1])
    interpolated = np.column_stack([np.interp(second_time[overlap], first_time, first_log['reference_xy'][:, column])
                                    for column in range(2)])
    reference_difference = float(np.max(np.abs(interpolated-second_log['reference_xy'][overlap])))
    first_params, second_params = first_summary['parameters'], second_summary['parameters']
    changed_parameters = {key: {'baseline': first_params.get(key), 'candidate': second_params.get(key)}
                          for key in sorted(set(first_params)|set(second_params))
                          if first_params.get(key) != second_params.get(key)}
    first_engine, second_engine = dict(first_summary['engine']), dict(second_summary['engine'])
    first_world, second_world = dict(first_engine.pop('world_info')), dict(second_engine.pop('world_info'))
    first_mesh, second_mesh = first_world.pop('dem_mesh', None), second_world.pop('dem_mesh', None)
    return {
        'route_path_metadata_equal': first_summary['route']['path'] == second_summary['route']['path'],
        'requested_duration_equal': first_summary['requested_duration_s'] == second_summary['requested_duration_s'],
        'saved_start_qpos_max_difference': float(np.max(np.abs(first_log['qpos'][0]-second_log['qpos'][0]))),
        'saved_start_qvel_max_difference': float(np.max(np.abs(first_log['qvel'][0]-second_log['qvel'][0]))),
        'overlap_reference_xy_max_difference_m': reference_difference,
        'original_timed_path_preserved': bool(first_summary['route']['path'] == second_summary['route']['path']
                                              and reference_difference < 1e-5),
        'changed_controller_parameters': changed_parameters,
        'engine_settings_equal': first_engine == second_engine and first_world == second_world,
        'dem_mesh_sha256_equal': (file_sha256(first_mesh) == file_sha256(second_mesh))
                                  if first_mesh and second_mesh else None,
    }


def plot_comparison(rows, output, candidate_label):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    metrics = [('rms_moving_mm', 'Moving RMS tracking error (mm)', False),
               ('max_error_mm', 'Largest tracking error (mm)', False),
               ('final_error_mm', 'Final tracking error (mm)', False),
               ('max_measured_joint_speed_rad_s', 'Largest measured joint speed (rad/s)', False),
               ('max_reference_speed_ratio', 'Largest drive reference / declared reference limit', True),
               ('max_reference_acceleration_ratio', 'Largest reference acceleration / declared limit', True),
               ('p95_applied_command_step_fraction', '95th percentile applied-command change / limit', False),
               ('max_attitude_error_deg', 'Largest attitude error (degrees)', False)]
    figure, axes = plt.subplots(4, 2, figsize=(14, 15), constrained_layout=True)
    x = np.arange(len(CASES))
    for axis, (metric, title, logarithmic) in zip(axes.ravel(), metrics):
        for variant, shift, color in (('baseline', -.2, '#2878b5'), (candidate_label, .2, '#269c83')):
            values = [next(row[metric] for row in rows if (row['route'], row['phase_rad'], row['variant']) ==
                           (name, phase, variant)) for name, phase in CASES]
            bars = axis.bar(x+shift, values, width=.38, color=color, label=variant.capitalize())
            axis.bar_label(bars, labels=[f'{value:.2g}' for value in values], fontsize=7, padding=3)
        axis.set(title=title, xticks=x, xticklabels=LABELS)
        axis.tick_params(axis='x', labelsize=8)
        axis.grid(axis='y', alpha=.2)
        axis.legend(fontsize=8)
        if logarithmic:
            axis.set_yscale('log')
            axis.axhline(1., color='#888888', linestyle=':', linewidth=1)
            lower, upper = axis.get_ylim()
            axis.set_ylim(lower, upper*2)
    figure.suptitle(f'Original timed trajectories: baseline versus {candidate_label} allocation', fontsize=14)
    figure.text(.01, -.012, 'Reference limits are declared simulation assumptions, not verified motor specifications. '
                'They constrain allocator references, not actual joint speed. Exact simulated state; estimated contacts.', fontsize=8)
    figure.savefig(output/'comparison.png', dpi=150, bbox_inches='tight')
    plt.close(figure)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--candidate-label', default='bounded')
    parser.add_argument('--control-interval-s', type=float)
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)
    rows, pairs = [], []
    for name, phase in CASES:
        _, original, original_summary = load_case(args.baseline, name, phase)
        _, candidate, candidate_summary = load_case(args.candidate, name, phase)
        speed_limits = np.asarray(candidate['allocation_speed_limits'][0], float)
        acceleration_limits = np.asarray(candidate['allocation_acceleration_limits'][0], float)
        if speed_limits.shape != (5,) or not np.isfinite(speed_limits).all() or np.any(speed_limits <= 0):
            raise ValueError('Candidate must record five positive finite reference speed limits')
        if acceleration_limits.shape != (5,) or not np.isfinite(acceleration_limits).all() or np.any(acceleration_limits <= 0):
            raise ValueError('Candidate must record five positive finite reference acceleration limits')
        if not np.array_equal(candidate['allocation_speed_limits'],
                              np.broadcast_to(speed_limits, candidate['allocation_speed_limits'].shape)):
            raise ValueError('Candidate reference speed limits vary within the run')
        if not np.array_equal(candidate['allocation_acceleration_limits'],
                              np.broadcast_to(acceleration_limits, candidate['allocation_acceleration_limits'].shape)):
            raise ValueError('Candidate reference acceleration limits vary within the run')
        pair = {'route': name, 'phase_rad': phase, **matched_path(original, original_summary, candidate, candidate_summary)}
        pairs.append(pair)
        if not pair['original_timed_path_preserved']:
            raise ValueError(f'{name}, phase {phase}: original timed path differs')
        for variant, root, log, summary in (('baseline', args.baseline, original, original_summary),
                                            (args.candidate_label, args.candidate, candidate, candidate_summary)):
            dt, source = interval_s(root, log, args.control_interval_s)
            row = case_metrics(log, summary, variant, speed_limits, acceleration_limits, dt)
            row.update(control_interval_s=dt, control_interval_source=source)
            rows.append(row)
    provenance = {}
    for name, root in (('baseline', args.baseline), ('candidate', args.candidate)):
        path = root/'provenance.json'
        provenance[name] = json.loads(path.read_text()) if path.is_file() else None
    source_changes = []
    if all(provenance.values()):
        hashes_a = provenance['baseline'].get('source_sha256', {})
        hashes_b = provenance['candidate'].get('source_sha256', {})
        source_changes = [name for name in sorted(set(hashes_a)|set(hashes_b))
                          if hashes_a.get(name) != hashes_b.get(name)]
    aggregate = {}
    for variant in ('baseline', args.candidate_label):
        group = [row for row in rows if row['variant'] == variant]
        aggregate[variant] = {
            'cases': len(group), 'completed': sum(row['completed'] for row in group),
            'passed_all_gates': sum(row['passed_all_gates'] for row in group),
            'reference_speed_violation_samples': sum(row['reference_speed_violation_samples'] for row in group),
            'reference_acceleration_violation_steps': sum(row['reference_acceleration_violation_steps'] for row in group),
            'max_reference_joint_speed_rad_s': max(row['max_reference_joint_speed_rad_s'] for row in group),
            'max_measured_joint_speed_rad_s': max(row['max_measured_joint_speed_rad_s'] for row in group),
            'max_requested_command_fraction': max(row['max_requested_command_fraction'] for row in group),
            'max_requested_command_step_fraction': max(row['max_requested_command_step_fraction'] for row in group),
            'contact_guarded_calls': sum(row['contact_guarded_calls'] for row in group),
        }
    report = {'rows': rows, 'aggregate': aggregate, 'path_and_start_audit': pairs,
              'provenance': provenance, 'source_files_with_different_recorded_hashes': source_changes,
              'baseline_directory': str(args.baseline.resolve()), 'candidate_directory': str(args.candidate.resolve()),
              'model_configurations_equal': configuration_hashes(args.baseline) == configuration_hashes(args.candidate),
              'interpretation': 'One recorded run per setting; numerical Gazebo reruns can diverge. '
                  'Reference speed/acceleration constraints do not guarantee actual motor speed/acceleration. '
                  'Baseline violations are retrospective comparisons against the candidate reference assumptions. '
                  'Requested motor-command saturation is distinct from reference-bound activation.'}
    (args.output/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    with (args.output/'comparison.csv').open('w', newline='') as stream:
        flat = [{key: value for key, value in row.items() if key != 'per_drive'} for row in rows]
        writer = csv.DictWriter(stream, fieldnames=list(flat[0]))
        writer.writeheader()
        writer.writerows(flat)
    plot_comparison(rows, args.output, args.candidate_label)
    for row in rows:
        print(f"{row['route']} {row['phase_rad']:+.2f} {row['variant']}: completed={row['completed']} "
              f"RMS={row['rms_moving_mm']:.1f}mm peak={row['max_error_mm']:.1f}mm "
              f"reference violations speed={row['reference_speed_violation_samples']} "
              f"acceleration={row['reference_acceleration_violation_steps']}", flush=True)


if __name__ == '__main__':
    main()
