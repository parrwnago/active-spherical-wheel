"""Offline allocator regression: a smooth phase sweep with one fixed support.

Synthetic flat-ground contacts are prescribed solely for kinematics. No physics,
observer, LQR, or trajectory controller runs. Fixed platform attitude and valid
interior shell-B support isolate allocation from support-transition behavior.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from spherical_wheel.contact_sensing import SupportLocator
from spherical_wheel.model import REPO_ROOT, generate_model, load_config
from spherical_wheel import rolling_allocation as allocation


SCALE = np.array([5., 5., 5., .3, .3])
SPEED_LIMITS = np.array([20., 20., 20., .3, .3])
ACCELERATION_LIMITS = np.array([100., 100., 100., 3., 3.])
TARGETS = {'x': np.array([1., 0.]), 'y': np.array([0., 1.]),
           'diagonal': np.array([1., 1.])/np.sqrt(2.)}
COLORS = {'least_squares': '#2878b5', 'bounded_adaptive': '#df7e31', 'feasible_directional': '#269c83'}
DISCLAIMER = ('Offline synthetic kinematic fixture only. No contact transition, '
              'physics, balance controller, measured motion, or hardware capability is tested.')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def snapshot(output, settings):
    sources = [Path(__file__).resolve(), *sorted((REPO_ROOT/'src/spherical_wheel').rglob('*.py'))]
    sources += sorted((REPO_ROOT/'config').glob('*.yaml'))
    hashes = {}
    for source in sources:
        relative = source.relative_to(REPO_ROOT)
        data = source.read_bytes()
        target = output/'source_snapshot'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        hashes[str(relative)] = digest(data)
    provenance = {'captured_before_computation_utc': datetime.now(timezone.utc).isoformat(),
                  'source_sha256': hashes, 'settings': settings,
                  'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=REPO_ROOT, text=True).strip(),
                  'mujoco_version': mujoco.__version__, 'numpy_version': np.__version__,
                  'scope': DISCLAIMER}
    (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    return provenance


def quintic(value):
    value = np.clip(value, 0., 1.)
    return value**3*(10.+value*(-15.+6.*value))


class FixedShellGeometry:
    def __init__(self, output):
        config = load_config()
        self.config = config
        self.radius = float(config['geometry']['sphere_radius_m'])
        self.model = mujoco.MjModel.from_xml_path(str(generate_model(config, output/'model')))
        self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.data = mujoco.MjData(self.model)
        self.fork = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'fork')
        self.weight = float(self.model.body_subtreemass[self.fork])*abs(config['simulation']['gravity_m_s2'][2])
        self.locator = SupportLocator(self.model, self.fork, self.radius, config['contact']['friction'][0], .05*self.weight)
        self.patch = next(item for item in self.locator.patches if item['name'] == 'shell_B_smooth')
        self.fork_boxes = [index for index in range(self.model.ngeom)
                           if self.model.geom_bodyid[index] == self.fork
                           and self.model.geom_type[index] == mujoco.mjtGeom.mjGEOM_BOX]

    def matrix(self, phase):
        roll = np.deg2rad(-20.)
        self.data.qpos[:] = [0., 0., self.radius, np.cos(roll/2), np.sin(roll/2), 0., 0., phase, 0., 0.]
        self.data.qvel[:] = 0.
        mujoco.mj_kinematics(self.model, self.data)
        mujoco.mj_comPos(self.model, self.data)
        records = self.locator.records(self.data, np.array([0., 0., self.weight]), np.zeros(3))
        assert self.data.ncon == 0
        assert len(records) == 1 and records[0]['name'] == 'shell_B_smooth', records
        record = records[0]
        point = np.asarray(record['pos'])
        offset = point-self.data.xpos[self.fork]
        axis = self.data.xmat[record['body']].reshape(3, 3)[:, 0]
        signed = self.patch['sign']*float(axis@offset)
        margin = min(signed-self.patch['lower'], self.patch['upper']-signed)
        assert margin > 0.
        assert abs(np.linalg.norm(offset)-self.radius) < 1e-10
        assert np.linalg.norm(point) < 1e-10
        jacobian = np.zeros((3, self.model.nv))
        mujoco.mj_jac(self.model, self.data, jacobian, None, point, record['body'])
        matrix = np.zeros((2, 5))
        matrix[:, :3] = jacobian[:2, 6:9]
        assert np.all(matrix[:, [1, 3, 4]] == 0.)
        clearance = min(float(self.data.geom_xpos[index, 2]
                        -np.abs(self.data.geom_xmat[index].reshape(3, 3)[2])@self.model.geom_size[index])
                        for index in self.fork_boxes)
        assert clearance > 0.
        tilt = float(np.degrees(np.arccos(np.clip(self.data.xmat[self.fork].reshape(3, 3)[2, 2], -1., 1.))))
        assert abs(tilt-20.) < 1e-10
        return matrix, margin, clearance


def make_allocator(variant):
    if variant == 'least_squares':
        return None
    if variant == 'bounded_adaptive':
        return allocation.BoundedRollingAllocator(allocation.AllocationSettings(weak_effort_m_s=.005))
    # This adapter is the sole coupling to the new production API.
    if variant == 'feasible_directional':
        return allocation.FeasibleRollingAllocator()
    raise ValueError(variant)


def solve(variant, allocator, matrix, demand, dt):
    if variant == 'least_squares':
        drive = SCALE*np.linalg.lstsq(matrix*SCALE, -demand, rcond=1e-7)[0]
        return drive, demand.copy(), {}
    drive = allocator.solve(matrix, -demand, dt, authority_matrix=matrix)
    diagnostics = allocator.diagnostics
    accepted = (np.asarray(diagnostics['allocation_accepted_velocity_m_s'], float)
                if variant == 'feasible_directional' else demand.copy())
    return drive, accepted, diagnostics


def run(variant, target, matrices, demand, time, dt):
    allocator = make_allocator(variant)
    drives, accepted, achieved, effort, bounds, responses, modal_efforts, valid = [], [], [], [], [], [], [], []
    for matrix, velocity in zip(matrices, demand):
        drive, chosen, info = solve(variant, allocator, matrix, velocity, dt)
        assert drive.shape == (5,) and chosen.shape == (2,)
        assert np.isfinite(drive).all() and np.isfinite(chosen).all()
        drives.append(drive); accepted.append(chosen); achieved.append(-matrix@drive)
        effort.append(info.get('allocation_effort_weight_m_s', np.nan))
        bounds.append(info.get('allocation_active_bounds', np.zeros(5, dtype=bool)))
        responses.append(info.get('allocation_velocity_response', np.full((2, 2), np.nan)))
        modal_efforts.append(info.get('allocation_modal_effort_m_s', np.full(2, np.nan)))
        valid.append(info.get('allocation_accepted_velocity_valid', False))
    drive = np.asarray(drives)
    accepted = np.asarray(accepted)
    achieved = np.asarray(achieved)
    steps = np.diff(np.vstack((np.zeros(5), drive)), axis=0)
    speed_ratio = np.abs(drive)/SPEED_LIMITS
    acceleration_ratio = np.abs(steps)/(ACCELERATION_LIMITS*dt)
    speed_violations = np.any(np.abs(drive) > SPEED_LIMITS+1e-8, axis=1)
    acceleration_violations = np.any(np.abs(steps) > ACCELERATION_LIMITS*dt+1e-8, axis=1)
    if variant != 'least_squares':
        assert not speed_violations.any(), (variant, target, speed_ratio.max())
        assert not acceleration_violations.any(), (variant, target, acceleration_ratio.max())
    pole = len(time)//2
    summary = {
        'variant': variant, 'target': target, 'samples': len(time),
        'speed_violation_samples': int(speed_violations.sum()),
        'acceleration_violation_steps': int(acceleration_violations.sum()),
        'max_speed_ratio': float(speed_ratio.max()), 'max_acceleration_ratio': float(acceleration_ratio.max()),
        'max_joint_reference_rad_s': float(np.abs(drive[:, :3]).max()),
        'max_normalized_drive_step': float(np.max(np.abs(steps/SCALE))),
        'max_accepted_velocity_step_m_s': float(np.max(np.linalg.norm(np.diff(accepted, axis=0), axis=1))),
        'max_demanded_velocity_step_m_s': float(np.max(np.linalg.norm(np.diff(demand, axis=0), axis=1))),
        'max_achievable_velocity_step_m_s': float(np.max(np.linalg.norm(np.diff(achieved, axis=0), axis=1))),
        'rms_nominal_velocity_residual_m_s': float(np.sqrt(np.mean(np.sum((achieved-demand)**2, axis=1)))),
        'max_nominal_velocity_residual_m_s': float(np.max(np.linalg.norm(achieved-demand, axis=1))),
        'max_accepted_velocity_residual_m_s': float(np.max(np.linalg.norm(achieved-accepted, axis=1))),
        'demand_velocity_time_integral_m': (dt*np.sum((demand[1:]+demand[:-1])/2, axis=0)).tolist(),
        'accepted_velocity_time_integral_m': (dt*np.sum((accepted[1:]+accepted[:-1])/2, axis=0)).tolist(),
        'achievable_velocity_time_integral_m': (dt*np.sum((achieved[1:]+achieved[:-1])/2, axis=0)).tolist(),
        'rejected_reference_velocity_time_integral_m': (dt*np.sum(((demand-accepted)[1:]+(demand-accepted)[:-1])/2, axis=0)).tolist(),
        'at_pole': {'time_s': float(time[pole]), 'demanded_velocity_xy_m_s': demand[pole].tolist(),
                    'accepted_velocity_xy_m_s': accepted[pole].tolist(),
                    'achievable_velocity_xy_m_s': achieved[pole].tolist(), 'drive_reference': drive[pole].tolist()},
        'accepted_reference_semantics': ('Allocator feasible body reference' if variant == 'feasible_directional' else
                                         'Demand unchanged: this old allocator does not supply a feasible body-reference acceptance step'),
    }
    history = {'drive_reference': drive, 'accepted_velocity_m_s': accepted, 'achievable_velocity_m_s': achieved,
               'effort_weight_m_s': np.asarray(effort), 'active_bounds': np.asarray(bounds),
               'velocity_response': np.asarray(responses), 'modal_effort_m_s': np.asarray(modal_efforts),
               'accepted_velocity_valid': np.asarray(valid)}
    return summary, history


def plot(output, time, phase, sigma, demand, results):
    fig, axes = plt.subplots(5, 3, figsize=(17, 17), sharex=True, constrained_layout=True)
    for column, target in enumerate(TARGETS):
        axes[0, column].plot(time, np.degrees(phase), color='#333333', label='Shaft phase')
        twin = axes[0, column].twinx()
        twin.semilogy(time, np.maximum(sigma, 1e-9), color='#777777', alpha=.6)
        twin.set_ylabel('Weak authority (m/s)', fontsize=8)
        axes[0, column].set(title=f'{target}: nominal target 0.06 m/s', ylabel='Phase (°)')
        for component, style in enumerate(('-', '--')):
            axes[1, column].plot(time, demand[target][:, component], color='#111111', linestyle=style,
                                 linewidth=1.6, label=f'Demand {"xy"[component]}')
        for (name, case), (_, history) in results.items():
            if case != target:
                continue
            color = COLORS[name]
            for component, style in enumerate(('-', '--')):
                axes[1, column].plot(time, history['accepted_velocity_m_s'][:, component], color=color,
                                     linestyle=style, label=f'{name} accepted {"xy"[component]}')
            residual = np.linalg.norm(history['achievable_velocity_m_s']-demand[target], axis=1)
            axes[2, column].plot(time, residual, color=color, label=name)
            axes[3, column].plot(time, np.max(np.abs(history['drive_reference'][:, :3]), axis=1), color=color)
            changes = np.max(np.abs(np.diff(history['drive_reference']/SCALE, axis=0)), axis=1)
            axes[4, column].plot(time[1:], changes, color=color)
        axes[1, column].set_ylabel('Body reference components (m/s)')
        axes[2, column].set_ylabel('Kinematic error from demand (m/s)')
        axes[3, column].set_yscale('symlog', linthresh=1.)
        axes[3, column].axhline(20., color='#888888', linestyle=':', label='Joint reference cap')
        axes[3, column].set_ylabel('Largest joint reference (rad/s)')
        axes[4, column].set_yscale('symlog', linthresh=.1)
        axes[4, column].set(ylabel='Per-step normalized drive change', xlabel='Time (s)')
        for row in range(5):
            axes[row, column].axvline(time[len(time)//2], color='#b44848', linewidth=1, linestyle=':')
            axes[row, column].grid(alpha=.2)
            axes[row, column].set_xlim(time[0], time[-1])
        axes[1, column].legend(fontsize=6, ncols=2)
        axes[2, column].legend(fontsize=7)
    fig.suptitle('Fixed shell-B allocation regression: constant platform roll −20°, no contact transitions', fontsize=14)
    fig.text(.01, -.012, DISCLAIMER+' Old modes pass nominal demand unchanged to balance feedback; '
             'accepted does not mean achieved for those modes.', fontsize=8)
    for suffix in ('png', 'pdf'):
        fig.savefig(output/f'comparison.{suffix}', dpi=150, bbox_inches='tight')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=REPO_ROOT/'results/fixed_contact_allocation_20261125/fixed_support')
    parser.add_argument('--old-baselines-only', action='store_true')
    parser.add_argument('--duration-s', type=float, default=8.)
    parser.add_argument('--dt', type=float, default=.005)
    args = parser.parse_args()
    if args.duration_s <= 1. or args.dt <= 0. or not np.isfinite([args.duration_s, args.dt]).all():
        parser.error('Need finite duration >1 s and positive dt')
    intervals = int(round(args.duration_s/args.dt))
    if intervals % 2 or abs(intervals*args.dt-args.duration_s) > 1e-10:
        parser.error('Duration must contain an even, integer number of intervals')
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=True)
    variants = ['least_squares', 'bounded_adaptive']+([] if args.old_baselines_only else ['feasible_directional'])
    settings = {'duration_s': args.duration_s, 'dt': args.dt, 'variants': variants, 'speed_m_s': .06,
                'roll_deg': -20., 'pitch_deg': 0., 'phase_start_rad': -.6, 'phase_turn_rad': -np.pi/2,
                'phase_profile': 'C2 quintic out and back; no crossing beyond the singular phase',
                'demand_profile': 'C2 half-second startup ramp, then constant nominal direction/speed',
                'speed_limits': SPEED_LIMITS.tolist(), 'acceleration_limits': ACCELERATION_LIMITS.tolist(),
                'scope': DISCLAIMER}
    provenance = snapshot(output, settings)
    time = np.arange(intervals+1)*args.dt
    progress = np.where(time <= args.duration_s/2, time/(args.duration_s/2),
                        (args.duration_s-time)/(args.duration_s/2))
    phase = -.6+(-np.pi/2+.6)*quintic(progress)
    fixture = FixedShellGeometry(output)
    matrices, margins, clearance = [], [], []
    for value in phase:
        matrix, margin, clear = fixture.matrix(value)
        matrices.append(matrix); margins.append(margin); clearance.append(clear)
    matrices = np.asarray(matrices)
    singular = np.linalg.svd(matrices*SCALE, compute_uv=False)
    assert singular[len(time)//2, 1] < 1e-12
    assert singular[0, 1] > .4
    assert np.all(matrices[:, :, [1, 3, 4]] == 0.)
    demand = {target: .06*quintic(time/.5)[:, None]*direction for target, direction in TARGETS.items()}
    results = {(variant, target): run(variant, target, matrices, demand[target], time, args.dt)
               for variant in variants for target in TARGETS}
    assertions = {
        'same_single_shell_B_every_sample': True, 'shell_patch_interior_every_sample': True,
        'fixed_platform_tilt_20deg_every_sample': True, 'fork_clear_of_plane_every_sample': True,
        'no_physics_or_controller_execution': True,
        'pole_weak_authority_below_1e_12_m_s': bool(singular[len(time)//2, 1] < 1e-12),
        'old_LS_Y_has_speed_limit_violations': results[('least_squares', 'y')][0]['speed_violation_samples'] > 0,
        'bounded_modes_obey_speed_and_acceleration_limits': all(summary['speed_violation_samples'] == summary['acceleration_violation_steps'] == 0
            for (variant, _), (summary, _) in results.items() if variant != 'least_squares'),
    }
    if 'feasible_directional' in variants:
        candidate = {target: results[('feasible_directional', target)] for target in TARGETS}
        assertions.update(
            feasible_reference_valid_every_sample=all(
                np.all(history['accepted_velocity_valid']) for _, history in candidate.values()),
            feasible_reference_matches_allocated_kinematics_every_sample=all(
                metrics['max_accepted_velocity_residual_m_s'] < 1e-10 for metrics, _ in candidate.values()),
            feasible_Y_rejects_unavailable_pole_component=abs(
                candidate['y'][0]['at_pole']['accepted_velocity_xy_m_s'][1]) < 1e-10,
            feasible_X_preserved_at_pole=abs(
                candidate['x'][0]['at_pole']['accepted_velocity_xy_m_s'][0]-.06) < 1e-5,
            feasible_diagonal_preserves_available_X_component_at_pole=abs(
                candidate['diagonal'][0]['at_pole']['accepted_velocity_xy_m_s'][0]-.06/np.sqrt(2.)) < 1e-5,
            feasible_diagonal_rejects_unavailable_Y_component_at_pole=abs(
                candidate['diagonal'][0]['at_pole']['accepted_velocity_xy_m_s'][1]) < 1e-10,
        )
    assertions = {name: bool(value) for name, value in assertions.items()}
    assert all(assertions.values()), assertions
    summary = {'scope': DISCLAIMER, 'settings': settings,
               'geometry': {'samples': len(time), 'support': 'shell_B_smooth', 'support_transitions': 0,
                            'contact_point_world_m': [0., 0., 0.],
                            'max_normalized_J_step_frobenius_m_s': float(np.max(np.linalg.norm(np.diff(matrices*SCALE, axis=0), axis=(1, 2)))),
                            'min_patch_margin_m': float(np.min(margins)), 'min_fork_clearance_m': float(np.min(clearance)),
                            'min_authority_m_s': float(singular[:, 1].min()), 'initial_authority_m_s': float(singular[0, 1])},
               'assertions': assertions, 'cases': [item[0] for item in results.values()],
               'source_sha256': provenance['source_sha256'],
               'limitations': ['Phase is prescribed as a kinematic parameter, not integrated from allocated motor rates.',
                               'Returning phase restores authority by construction; this does not show autonomous reconfiguration or escape from a zero-drive singular configuration.',
                               'Velocity time integrals are kinematic diagnostics, not measured travel or destination completion.',
                               'No friction, torque/current, dynamic balance, or closed-loop path claim.',
                               'The old allocator accepted field denotes unchanged demand, not achievable motion.']}
    (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    rows = []
    for (variant, target), (metrics, history) in results.items():
        directory = output/target/variant; directory.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(directory/'history.npz', time_s=time, phase_rad=phase,
                            contact_matrix=matrices, weak_authority_m_s=singular[:, 1],
                            patch_margin_m=margins, nominal_velocity_m_s=np.tile(.06*TARGETS[target], (len(time), 1)),
                            demanded_velocity_m_s=demand[target], **history)
        (directory/'summary.json').write_text(json.dumps(metrics, indent=2, allow_nan=False)+'\n')
        for i, now in enumerate(time):
            rows.append({'variant': variant, 'target': target, 'time_s': float(now), 'phase_rad': float(phase[i]),
                         'authority_m_s': float(singular[i, 1]), 'demand_x_m_s': float(demand[target][i, 0]),
                         'demand_y_m_s': float(demand[target][i, 1]),
                         **{f'accepted_{"xy"[j]}_m_s': float(history['accepted_velocity_m_s'][i, j]) for j in range(2)},
                         **{f'achievable_{"xy"[j]}_m_s': float(history['achievable_velocity_m_s'][i, j]) for j in range(2)},
                         **{f'drive_{j}': float(history['drive_reference'][i, j]) for j in range(5)}})
    with (output/'histories.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    plot(output, time, phase, singular[:, 1], demand, results)
    print(json.dumps({'geometry': summary['geometry'], 'assertions': assertions,
                      'cases': [{key: value for key, value in item.items() if key not in ('at_pole',)} for item in summary['cases']]}, indent=2))


if __name__ == '__main__':
    main()
