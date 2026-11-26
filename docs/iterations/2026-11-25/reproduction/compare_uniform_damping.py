"""Post-run comparison of the diagnostic baseline and damped allocator trial."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from spherical_wheel.contact_diagnostics import diagnostic_signals

ROOT = Path(__file__).resolve().parent
CASES = [('plain_figure8', phase) for phase in (.3, .9, -1.)]+[
    ('shape_triangle', .3), ('shape_line_and_back', .3)]
LABELS = ['Figure-eight\n+0.30 rad', 'Figure-eight\n+0.90 rad', 'Figure-eight\n−1.00 rad',
          'Triangle\n+0.30 rad', 'Out-and-back\n+0.30 rad']


def main():
    rows, loaded = [], {}
    for name, phase in CASES:
        for variant in ('baseline', 'damped'):
            directory = ROOT/variant/name/f'phase_{phase:+.2f}'
            summary = json.loads((directory/'summary.json').read_text())
            with np.load(directory/'rollout.npz') as saved:
                log = {key: np.asarray(value) for key, value in saved.items()}
            signals = diagnostic_signals(log)
            actual_loaded = signals['truth_contact_valid']
            valid = signals['velocity_consistency_valid'] & actual_loaded
            mismatch = signals['rolling_mismatch_rms_m_s']
            truth_slip = signals['truth_slip_speed_m_s']
            valid &= np.isfinite(mismatch) & np.isfinite(truth_slip)
            flagged = valid & (mismatch > .02)
            correlation = None
            if valid.sum() > 2 and np.std(mismatch[valid]) > 1e-12 and np.std(truth_slip[valid]) > 1e-12:
                correlation = float(np.corrcoef(mismatch[valid], truth_slip[valid])[0, 1])
            point_error = signals['contact_point_error_m']
            row = {
                'route': name, 'phase_rad': phase, 'variant': variant,
                'completed': summary['completed'], 'failure': summary['failure'],
                'rms_moving_mm': 1000*summary['tracking']['rms_error_while_moving_m'],
                'max_error_mm': 1000*summary['tracking']['max_error_m'],
                'final_error_mm': 1000*summary['tracking']['final_error_m'],
                'max_reference_joint_speed_rad_s': float(np.max(signals['reference_joint_speed_max_rad_s'])),
                'max_measured_joint_speed_rad_s': float(np.max(signals['measured_joint_speed_max_rad_s'])),
                'max_command_step_fraction': float(np.max(signals['command_step_fraction'])),
                'p95_command_step_fraction': float(np.percentile(signals['command_step_fraction'], 95)),
                'max_applied_command_step_fraction': float(np.max(signals['applied_command_step_fraction'])),
                'p95_applied_command_step_fraction': float(np.percentile(signals['applied_command_step_fraction'], 95)),
                'mismatch_rms_p95_m_s': float(np.percentile(mismatch[valid], 95)) if valid.any() else None,
                'point_error_p95_mm': float(np.percentile(point_error[valid], 95)*1000) if valid.any() else None,
                'rolling_mismatch_truth_slip_pearson_loaded': correlation,
                'mismatch_flagged_samples': int(flagged.sum()),
                'flagged_fraction_with_truth_slip_over_0_02_m_s': float(np.mean(truth_slip[flagged] > .02)) if flagged.any() else None,
                'flagged_fraction_with_point_error_over_5_mm': float(np.mean(point_error[flagged] > .005)) if flagged.any() else None,
                'requested_torque_saturation_fraction': float(np.mean(signals['torque_saturated'])),
                'requested_belt_saturation_fraction': float(np.mean(signals['belt_saturated'])),
                'passed_all_gates': summary['mars']['passed'],
                'failed_gates': [key for key, passed in summary['mars']['gates'].items() if not passed],
            }
            rows.append(row)
            loaded[(name, phase, variant)] = (signals, valid)
    report = {'rows': rows,
              'interpretation': 'Single paired run per case. Completion is separate from all-gate acceptance. '
                'Mismatch and truth-slip statistics are co-occurrences, not causal attribution or a slip classifier. '
                'The diagnostic residual is per-contact RMS; logged truth slip is the norm of a weighted mean, '
                'so they can differ for multiple contacts even with accurate estimates.'}
    (ROOT/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    with (ROOT/'comparison.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    metrics = [('rms_moving_mm', 'Moving RMS tracking error (mm)', False),
               ('max_error_mm', 'Largest tracking error (mm)', False),
               ('final_error_mm', 'Final tracking error (mm)', False),
               ('max_reference_joint_speed_rad_s', 'Largest joint-speed reference (rad/s)', True),
               ('max_command_step_fraction', 'Largest command change / actuator limit', True),
               ('mismatch_rms_p95_m_s', '95th percentile rolling mismatch (m/s)', False)]
    figure, axes = plt.subplots(3, 2, figsize=(14, 12), constrained_layout=True)
    x = np.arange(len(CASES))
    for axis, (metric, title, logarithmic) in zip(axes.ravel(), metrics):
        for variant, shift, color in (('baseline', -.2, '#2878b5'), ('damped', .2, '#df7e31')):
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
            lower, upper = axis.get_ylim()
            axis.set_ylim(lower, upper*2)
    figure.suptitle('Diagnostic experiment: least-squares baseline versus damped rolling allocation', fontsize=14)
    figure.text(.01, -.012, 'All contacts estimated; exact simulated state and prepared balanced starts. '
                'One run per setting. A reduction in extreme references does not establish better tracking across all cases.', fontsize=8)
    figure.savefig(ROOT/'comparison.png', dpi=150, bbox_inches='tight')
    plt.close(figure)
    figure, axes = plt.subplots(2, 5, figsize=(19, 8), constrained_layout=True)
    for column, (name, phase) in enumerate(CASES):
        for row, variant in enumerate(('baseline', 'damped')):
            axis = axes[row, column]
            signals, valid = loaded[(name, phase, variant)]
            indices = np.flatnonzero(valid)
            indices = indices[::max(1, len(indices)//1500)]
            xx = signals['truth_slip_speed_m_s'][indices]
            yy = signals['rolling_mismatch_rms_m_s'][indices]
            color = np.minimum(signals['contact_point_error_m'][indices]*1000, 20.)
            points = axis.scatter(xx, yy, c=color, s=4, alpha=.35, cmap='viridis', vmin=0, vmax=20)
            extent = max(float(xx.max()), float(yy.max()), .02)
            axis.plot([0, extent], [0, extent], color='#999999', linestyle=':', linewidth=1)
            axis.set(title=f'{LABELS[column]}: {variant}', xlabel='Truth weighted-mean slip (m/s)',
                     ylabel='Estimated-contact RMS mismatch (m/s)', xlim=(0, extent), ylim=(0, extent))
            axis.grid(alpha=.2)
    figure.colorbar(points, ax=axes.ravel().tolist(), label='Equivalent contact-point error (mm, clipped at 20)')
    figure.suptitle('Velocity consistency versus evaluation-only slip and contact-point error', fontsize=14)
    figure.text(.01, -.012, 'These are different aggregation measures for multiple contacts. '
                'Scatter shows co-occurrence; it does not identify why a rolling mismatch occurred.', fontsize=8)
    figure.savefig(ROOT/'velocity_consistency_evaluation.png', dpi=150, bbox_inches='tight')
    plt.close(figure)
    for row in rows:
        print(row['route'],row['phase_rad'],row['variant'],
              'rms/peak/final',*[round(row[key],2) for key in ('rms_moving_mm','max_error_mm','final_error_mm')],
              'maxref',round(row['max_reference_joint_speed_rad_s'],3),flush=True)


if __name__ == '__main__':
    main()
