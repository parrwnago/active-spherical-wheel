"""Inspect the recorded adaptive allocator's negative-phase correction."""
from pathlib import Path
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from spherical_wheel.contact_diagnostics import diagnostic_signals

ROOT = Path(__file__).resolve().parent
DIRECTORY = ROOT/'bounded_adaptive/plain_figure8/phase_-1.00'
GROUPS = {'position': slice(0, 3), 'attitude': slice(3, 6), 'joint_position': slice(6, 9),
          'body_linear_velocity': slice(9, 12), 'body_angular_velocity': slice(12, 15),
          'joint_velocity': slice(15, 18)}


def main():
    with np.load(DIRECTORY/'rollout.npz') as saved:
        log = {key: np.asarray(value) for key, value in saved.items()}
    model = mujoco.MjModel.from_xml_path(str(next((ROOT/'bounded_adaptive').glob('_workspaces/*/flat/model.xml')).resolve()))
    signals = diagnostic_signals(log)
    time = log['time_s']
    selection = (time >= 28.) & (time <= 30.5)
    indices = np.flatnonzero(selection)
    contributions = {key: np.zeros((len(indices), 5)) for key in GROUPS}
    reconstruction_error = []
    for output_index, index in enumerate(indices):
        qpos = log['qpos'][index].astype(float)
        quat = qpos[3:7]
        yaw = float(np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]), 1-2*(quat[2]**2+quat[3]**2)))
        c, s = np.cos(yaw), np.sin(yaw)
        world_to_heading = np.array([[c, s], [-s, c]])
        target = qpos.copy()
        target[:2] = log['reference_xy'][index]
        mujoco.mju_mulQuat(target[3:7], log['reference_quat'][index].astype(float),
                          np.array([np.cos(yaw/2), 0., 0., np.sin(yaw/2)]))
        difference = np.zeros(9)
        mujoco.mj_differentiatePos(model, difference, 1., target, qpos)
        error = np.r_[difference, log['qvel'][index]-log['feedback_reference_qvel'][index]]
        error[:2] = world_to_heading@error[:2]
        error[9:11] = world_to_heading@error[9:11]
        error[6:9] = (error[6:9]+np.pi) % (2*np.pi)-np.pi
        error[7:9] = 0.
        error[6] = error[0]/.12-error[4]
        gain = log['feedback_gain'][index].astype(float)
        for key, part in GROUPS.items():
            contributions[key][output_index] = -gain[:, part]@error[part]
        reconstruction_error.append(float(np.max(np.abs(-gain@error-log['lqr_command'][index]))))
    snapshots = []
    for moment in (27.975, 28.5, 28.75, 28.975, 29.25, 29.975):
        index = int(np.argmin(abs(time-moment)))
        item = {
            'time_s': float(time[index]), 'tracking_error_mm': float(signals['tracking_error_m'][index]*1000),
            'attitude_error_deg': float(np.degrees(log['attitude_error_rad'][index])),
            'normal_error_deg': float(signals['normal_error_deg'][index]),
            'equivalent_contact_point_error_mm': float(signals['contact_point_error_m'][index]*1000),
            'truth_loads_N': log['support_loads_N'][index].tolist(),
            'estimated_loads_N': log['estimated_support_loads_N'][index].tolist(),
        }
        for key in ('command', 'input', 'lqr_command', 'rolling_feedforward', 'support_command',
                    'transition_command_delta', 'allocation_drive_reference', 'allocation_velocity_error_m_s',
                    'allocation_residual_rms_m_s', 'allocation_authority_m_s', 'allocation_effort_weight_m_s',
                    'allocation_effort_blend', 'allocation_active_bounds', 'contact_velocity_residual_rms_m_s'):
            item[key] = log[key][index].tolist()
        if index in indices:
            location = int(np.flatnonzero(indices == index)[0])
            item['reconstructed_lqr_contributions'] = {key: value[location].tolist() for key, value in contributions.items()}
        snapshots.append(item)
    report = {
        'case': 'plain_figure8/phase_-1.00', 'window_s': [28., 30.5],
        'max_lqr_reconstruction_absolute_error': max(reconstruction_error),
        'interpretation': 'The LQR decomposition is reconstructed from saved single-precision states/gains and checked '
            'against logged LQR commands. Co-occurrence does not establish causation. At28.975s the allocator explains '
            'the desired velocity with a negligible residual under its estimated geometry, but estimated support is '
            'beltA while instantaneous solver support is shellA. Feedback belt demand is dominated by attitude and '
            'velocity terms, not position alone, and exceeds the belt command limit. Small equivalent-point and '
            'normal errors do not guarantee correct support identity near a patch boundary.',
        'snapshots': snapshots,
    }
    (ROOT/'adaptive_negative_phase_event.json').write_text(json.dumps(report, indent=2)+'\n')
    figure, axes = plt.subplots(5, 1, figsize=(14, 13), sharex=True, constrained_layout=True)
    shown = time[selection]
    axes[0].plot(shown, signals['tracking_error_m'][selection]*1000, color='#2878b5', label='Position error')
    axes[0].set(title='Tracking deviation', ylabel='mm')
    for column, label, color in ((0, 'Shell A', '#2878b5'), (2, 'Belt A', '#df7e31')):
        axes[1].plot(shown, signals['estimated_support_shares'][selection, column], color=color, label=label+' estimated')
        axes[1].plot(shown, signals['true_support_shares'][selection, column], '--', color=color, alpha=.6, label=label+' truth')
    axes[1].set(title='Support identity: estimated versus evaluation-only truth', ylabel='Load share', ylim=(-.03, 1.03))
    axes[2].plot(shown, log['allocation_residual_rms_m_s'][selection], label='Allocation feasibility residual', color='#269c83')
    axes[2].plot(shown, log['contact_velocity_residual_rms_m_s'][selection], label='Measured rolling mismatch', color='#c73e3e')
    axes[2].plot(shown, log['slip_speed_m_s'][selection], label='Truth weighted-mean slip', color='#777777', alpha=.8)
    axes[2].set(title='Allocation feasibility differs from measured-motion consistency', ylabel='m/s')
    for key, label, color in (('command', 'Requested total', '#181818'), ('input', 'Applied total', '#c73e3e'),
                              ('lqr_command', 'LQR', '#2878b5'), ('rolling_feedforward', 'Rolling feedforward', '#269c83')):
        axes[3].plot(shown, log[key][selection, 3], label=label, color=color)
    axes[3].axhline(.3, color='#888888', linestyle=':', linewidth=1)
    axes[3].axhline(-.3, color='#888888', linestyle=':', linewidth=1)
    axes[3].set(title='Belt A command: LQR correction dominates the modest feedforward demand', ylabel='m/s')
    for key, color in (('attitude', '#df7e31'), ('body_linear_velocity', '#2878b5'),
                       ('body_angular_velocity', '#a06ac2'), ('position', '#269c83')):
        axes[4].plot(shown, contributions[key][:, 3], label=key.replace('_', ' '), color=color)
    axes[4].set(title='Reconstructed Belt A LQR contributions', ylabel='m/s', xlabel='Time (s)')
    for axis in axes:
        axis.axvline(28.975, color='#c73e3e', alpha=.5, linewidth=1)
        axis.axvline(29.975, color='#c73e3e', alpha=.35, linewidth=1)
        axis.legend(fontsize=8, loc='best', ncols=3)
        axis.grid(alpha=.2)
        axis.set_xlim(shown[0], shown[-1])
    figure.suptitle('Adaptive bounded allocation, figure-eight phase−1.00: remaining mid-course correction', fontsize=14)
    figure.text(.01, -.012, 'All contacts estimated in control; state measurements exact. '
                'Vertical markers: onset sample28.975s and maximum tracking deviation29.975s. These observations do not prove one cause.', fontsize=8)
    figure.savefig(ROOT/'adaptive_negative_phase_event.png', dpi=150, bbox_inches='tight')
    plt.close(figure)
    print('Saved adaptive_negative_phase_event.json/png; max reconstruction error', max(reconstruction_error))


if __name__ == '__main__':
    main()
