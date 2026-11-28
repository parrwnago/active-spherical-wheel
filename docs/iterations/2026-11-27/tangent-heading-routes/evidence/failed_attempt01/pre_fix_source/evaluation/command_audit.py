"""I76 bounded descriptive command audit, defined during the 18-case campaign.

This plan is fixed before reading fresh outcomes, but is not a prephysics
control gate. Execute only after root releases all eighteen sealed outputs and
the canonical evaluation. For EVERY case select its earliest maximum held
normalized clipping excess and retain +/-25 ms context. A zero maximum is an
explicit no-clipping tie, not a spike. No physics, new inference, Jacobian,
interaction-vector analysis, parameter tuning or acceptance change is made.

The ledger/error-block method follows the preserved I75 audit; I75 is unchanged.
"""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import shutil

import numpy as np
import yaml


BASE = Path(__file__).resolve().parents[1]
ROOT = BASE.parents[1]
CONTEXT_HALF_WIDTH_S = .025
BLOCKS = (('XY_position', 0, 2), ('height', 2, 3),
          ('roll_pitch_attitude', 3, 5), ('heading_position', 5, 6),
          ('derived_rolling_shaft_error', 6, 7), ('cyclic_shell_position', 7, 9),
          ('body_linear_velocity', 9, 12), ('body_angular_velocity', 12, 15),
          ('shaft_velocity', 15, 16), ('shell_velocities', 16, 18))
FIELDS = ('time_s', 'command_held_after_row', 'exact_requested', 'exact_applied',
          'exact_input_bounds', 'exact_equilibrium_command', 'exact_rolling_feedforward',
          'exact_lqr_command', 'exact_support_command', 'exact_transition_command_delta',
          'exact_heading_command', 'exact_fade_command_delta', 'exact_clipping_command_delta',
          'exact_feedback_gain', 'exact_error', 'exact_qpos', 'exact_qvel',
          'exact_reference_qpos', 'exact_reference_qvel', 'exact_feedback_reference_qvel',
          'exact_allocation_drive_reference', 'exact_estimated_support_loads_N',
          'exact_estimated_normal', 'exact_effective_phase_rad', 'exact_drive_fade',
          'heading__raw_heading_rad', 'heading__raw_heading_rate_rad_s',
          'heading__command_heading_rad', 'heading__raw_error_rad',
          'heading__command_error_rad', 'heading__error_used_rad', 'heading__eligible',
          'heading__vertical_lever', 'heading__horizontal_lever', 'heading__free_shell_mask',
          'heading__effective_motor_limit_Nm', 'heading__wanted_world_yaw_torque_Nm',
          'heading__euler_heading_rate_rad_s', 'heading__world_z_rate_rad_s')
LEDGER_FIELDS = ('exact_requested', 'exact_applied', 'exact_input_bounds',
                 'exact_equilibrium_command', 'exact_rolling_feedforward', 'exact_lqr_command',
                 'exact_support_command', 'exact_transition_command_delta', 'exact_heading_command',
                 'exact_fade_command_delta', 'exact_clipping_command_delta')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, dict):
        return {str(key): safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return safe(value.tolist())
    if isinstance(value, np.generic):
        return safe(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write(path, value):
    path.write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def peak_components(t, values, mask):
    """Earliest maximum absolute component on the explicitly supplied scope."""
    values = np.asarray(values)
    if values.ndim == 1:
        values = values[:, None]
    indices = np.flatnonzero(mask)
    if not len(indices):
        return []
    if not np.isfinite(values[indices]).all():
        raise ValueError('Cannot report a finite component peak from nonfinite selected data')
    peaks = []
    for channel in range(values.shape[1]):
        index = int(indices[np.argmax(abs(values[indices, channel]))])
        peaks.append(dict(channel=channel, source_frame=index, time_s=float(t[index]),
                          signed_value=float(values[index, channel]),
                          absolute_value=float(abs(values[index, channel]))))
    return peaks


def verify_output_release(protocol, protocol_path, seal_path, runs_root, evaluation):
    """Verify all 18 sealed case files before any fresh NumPy-array load."""
    jobs = protocol['cases']
    expected = {(route, policy, phase) for route in ('straight', 'arc', 'eight')
                for policy in ('N', 'T') for phase in (-1., .3, .9)}
    if (protocol.get('iteration') != 'I-76' or len(jobs) != 18 or
        len({job['case_id'] for job in jobs}) != 18 or
        {(job['route_id'], job['policy'], float(job['phase_rad'])) for job in jobs} != expected):
        raise ValueError('Exactly eighteen I76 route/phase/policy cases required')
    seal = json.loads(seal_path.read_text())
    processes = seal.get('processes', [])
    if (seal.get('protocol_sha256') != sha(protocol_path) or
        not seal.get('all_processes_successful') or len(processes) != 18 or
        {item['case_id'] for item in processes} != {job['case_id'] for job in jobs} or
        any(item.get('returncode') != 0 for item in processes)):
        raise ValueError('Successful all-eighteen output seal required')
    resolved = {}
    for relative, binding in seal.get('files', {}).items():
        path = (BASE/relative).resolve()
        if sha(path) != binding['sha256'] or path.stat().st_size != binding['bytes']:
            raise ValueError('Sealed case output changed: '+relative)
        resolved[path] = binding
    for job in jobs:
        for name in ('precision.npz', 'summary.json', 'provenance.json'):
            if (runs_root/job['case_id']/name).resolve() not in resolved:
                raise ValueError('Required case file is absent from the all-eighteen output seal')
    manifest = json.loads((evaluation/'artifact_manifest.json').read_text())
    canonical_bindings = {item['path']: item for item in manifest['files']}
    for name in ['science.json']+[job['case_id']+'_signals.npz' for job in jobs]:
        binding = canonical_bindings.get(name)
        path = evaluation/name
        if binding is None or sha(path) != binding['sha256'] or path.stat().st_size != binding['bytes']:
            raise ValueError('Canonical evaluation file is missing or changed: '+name)


def case_audit(data, signals, job, radius, canonical_peak, output):
    t = data['time_s']
    held = data['command_held_after_row'].astype(bool)
    if not np.array_equal(held, np.arange(len(t)) < len(t)-1):
        raise ValueError('Terminal emitted command must remain explicitly unheld')
    if not np.array_equal(t, signals['time_s']):
        raise ValueError('Canonical signal and command clocks differ')
    if (not len(t) or abs(t[0]) > 1e-9 or
        float(t[-1]) > float(job['duration_s'])+1e-9 or
        not np.all(abs(np.diff(t)-.005) <= 1e-9)):
        raise ValueError('Actual case clock disagrees with its registered duration')
    gain, error = data['exact_feedback_gain'], data['exact_error']
    blocks = {name: -np.einsum('nij,nj->ni', gain[:, :, lo:hi], error[:, lo:hi])
              for name, lo, hi in BLOCKS}
    caps = data['exact_input_bounds']
    clipped = np.clip(data['exact_requested'], -caps, caps)
    excess = np.linalg.norm((data['exact_requested']-clipped)/caps, axis=1)
    before_optional = (data['exact_equilibrium_command']+data['exact_rolling_feedforward']+
                       data['exact_support_command']+data['exact_lqr_command'])
    requested = (before_optional+data['exact_transition_command_delta']+
                 data['exact_heading_command']+data['exact_fade_command_delta'])
    applied = requested+data['exact_clipping_command_delta']
    pairs = dict(lqr_full=(-np.einsum('nij,nj->ni', gain, error), data['exact_lqr_command']),
                 lqr_blocks=(sum(blocks.values()), data['exact_lqr_command']),
                 requested_ledger=(requested, data['exact_requested']),
                 applied_ledger=(applied, data['exact_applied']),
                 applied_is_clipped=(clipped, data['exact_applied']),
                 derived_rolling_shaft_error=(error[:, 0]/radius-error[:, 4], error[:, 6]),
                 body_angular_velocity_error=(data['exact_qvel'][:, 3:6]-data['exact_feedback_reference_qvel'][:, 3:6], error[:, 12:15]),
                 joint_velocity_error=(data['exact_qvel'][:, 6:9]-data['exact_feedback_reference_qvel'][:, 6:9], error[:, 15:18]))
    verification = {}
    for name, (left, right) in pairs.items():
        passed = bool(np.allclose(left, right, atol=1e-9, rtol=1e-12))
        verification[name] = dict(passed=passed, max_abs_difference=float(np.max(abs(left-right))))
        if not passed:
            raise AssertionError(job['case_id']+' ledger '+name)
    zero_refs = {name: bool(np.array_equal(data[name][:, 3:6], np.zeros_like(data[name][:, 3:6])))
                 for name in ('exact_reference_qvel', 'exact_feedback_reference_qvel')}
    if not all(zero_refs.values()):
        raise AssertionError('Declared zero body-angular-reference policy changed')
    extrema = dict(actual_joint_velocity_rad_s=dict(scope='All actual physical-state rows, including terminal state.',
                   channel_names=['shaft', 'shell_A', 'shell_B'], channel_units=['rad/s']*3,
                   components=peak_components(t, data['exact_qvel'][:, 6:9], np.ones(len(t), bool))))
    for name, key in (('raw_joint_reference_rad_s', 'exact_reference_qvel'),
                      ('feedback_joint_reference_rad_s', 'exact_feedback_reference_qvel')):
        extrema[name] = dict(scope='Held command intervals only; terminal emitted reference excluded.',
                             channel_names=['shaft', 'shell_A', 'shell_B'], channel_units=['rad/s']*3,
                             components=peak_components(t, data[key][:, 6:9], held))
    for name, key in (('allocation_reference_mixed_units', 'exact_allocation_drive_reference'),
                      ('rolling_feedforward_mixed_units', 'exact_rolling_feedforward'),
                      ('lqr_command_mixed_units', 'exact_lqr_command'),
                      ('requested_mixed_units', 'exact_requested'),
                      ('applied_mixed_units', 'exact_applied')):
        extrema[name] = dict(scope='Held command intervals only; terminal emitted command excluded.',
                             channel_names=['shaft', 'shell_A', 'shell_B', 'belt_A', 'belt_B'],
                             channel_units=(['rad/s']*3+['m/s']*2 if key == 'exact_allocation_drive_reference'
                                            else ['Nm']*3+['m/s']*2),
                             components=peak_components(t, data[key], held))
    event = None
    indices = np.flatnonzero(held)
    if len(indices):
        index = int(indices[np.argmax(excess[indices])])
        center = float(t[index])
        context = np.flatnonzero(abs(t-center) <= CONTEXT_HALF_WIDTH_S+1e-9)
        context_data = {key: value[context].copy() for key, value in data.items()}
        context_data.update(source_frame_index=context, normalized_clipping_excess=excess[context])
        context_data.update({'lqr_block__'+name: value[context].copy() for name, value in blocks.items()})
        np.savez_compressed(output/(job['case_id']+'_context.npz'), **context_data)
        zero_tie = bool(excess[index] == 0.)
        event = dict(source_frame=index, time_s=center, command_is_held=True,
                     normalized_clipping_excess=float(excess[index]), earliest_zero_excess_tie=zero_tie,
                     event_kind='no_held_clipping_earliest_zero_tie' if zero_tie else 'maximum_held_clipping_excess',
                     no_clipping_not_a_spike=zero_tie,
                     ledger={key: data[key][index] for key in LEDGER_FIELDS},
                     lqr_error_blocks={name: value[index] for name, value in blocks.items()},
                     error18=error[index], actual_joint_velocity_rad_s=data['exact_qvel'][index, 6:9],
                     reference_joint_velocity_rad_s=data['exact_reference_qvel'][index, 6:9],
                     feedback_reference_joint_velocity_rad_s=data['exact_feedback_reference_qvel'][index, 6:9],
                     heading={key: data[key][index] for key in FIELDS if key.startswith('heading__')},
                     estimated_support_loads_N=data['exact_estimated_support_loads_N'][index],
                     context=dict(first_s=float(t[context[0]]), last_s=float(t[context[-1]]), samples=len(context),
                                  path=job['case_id']+'_context.npz',
                                  terminal_policy='Context preserves held flag; any terminal emitted row is unheld.'))
        if canonical_peak is None or abs(float(excess[index])-canonical_peak) > 1e-9:
            raise AssertionError('Canonical clipping-excess peak mismatch')
    elif canonical_peak is not None:
        raise AssertionError('Canonical peak exists despite no held intervals')
    return dict(case=job, actual_clock=dict(registered_duration_s=job['duration_s'],
                actual_terminal_s=float(t[-1]), physical_state_rows=len(t), held_intervals=int(held.sum())),
                zero_body_angular_references_verified=zero_refs, full_record_extrema=extrema,
                maximum_held_clipping_event=event, verification=verification,
                verification_scope='Eight full-record algebraic identities; these are not eight independent software tests.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('protocol', 'runs-root', 'all-runs-seal', 'evaluation', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    protocol_path, seal_path = args.protocol.resolve(), args.all_runs_seal.resolve()
    runs_root, evaluation, output = args.runs_root.resolve(), args.evaluation.resolve(), args.output.resolve()
    if output.exists():
        raise FileExistsError('Preserve previous command audit stages')
    protocol = json.loads(protocol_path.read_text())
    verify_output_release(protocol, protocol_path, seal_path, runs_root, evaluation)
    native_sources = [ROOT/'config/model.yaml', ROOT/'src/spherical_wheel/offroad_control.py', ROOT/'src/spherical_wheel/sim.py']
    for path in native_sources:
        if protocol['source_sha256'].get(str(path.relative_to(ROOT))) != sha(path):
            raise ValueError('Frozen native source/config changed')
    paths = [Path(__file__).resolve(), protocol_path, seal_path, evaluation/'science.json',
             evaluation/'artifact_manifest.json', *native_sources]
    for job in protocol['cases']:
        paths += [runs_root/job['case_id']/'precision.npz', evaluation/(job['case_id']+'_signals.npz')]
    bindings = [dict(path=str(path), sha256=sha(path), bytes=path.stat().st_size) for path in paths]
    output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(Path(__file__), output/'audit.py')
    write(output/'registered_before_arithmetic.json', dict(
        iteration='I-76', utc=datetime.now(timezone.utc).isoformat(), inputs=bindings,
        plan_timing='Diagnostic defined during the running campaign, before fresh outcome reads; not a prephysics control gate.',
        execution_release='All eighteen outputs sealed and canonical evaluation completed; root release required.',
        no_new_physics=True, no_new_contact_or_terrain_inference=True, no_Jacobian_or_new_model=True,
        no_parameter_or_gate_changes=True, no_interaction_vector_analysis=True,
        selection='Every one of eighteen cases; earliest maximum held normalized clipping excess; zero tie explicitly no clipping.',
        context_half_width_s=CONTEXT_HALF_WIDTH_S, source_error_blocks=BLOCKS,
        derived_shaft_note='Native error6 = heading-relative XY error0 / radius minus pitch error4; distinct from measured shaft angle.',
        command_scope='Exclude final emitted but unheld command from selection and command/reference extrema; actual state peaks include terminal state.'))
    radius = float(yaml.safe_load((ROOT/'config/model.yaml').read_text())['geometry']['sphere_radius_m'])
    science = json.loads((evaluation/'science.json').read_text())
    if science.get('iteration') != 'I-76' or set(science['cases']) != {job['case_id'] for job in protocol['cases']}:
        raise ValueError('Canonical science report does not cover all eighteen I76 cases')
    reports = {}
    for job in protocol['cases']:
        case = job['case_id']
        if science['cases'][case]['case'] != job:
            raise ValueError('Canonical case differs from registered job')
        with np.load(runs_root/case/'precision.npz', allow_pickle=False) as archive:
            missing = set(FIELDS)-set(archive.files)
            if missing:
                raise ValueError(case+' missing '+str(sorted(missing)))
            data = {key: archive[key].copy() for key in FIELDS}
        with np.load(evaluation/(case+'_signals.npz'), allow_pickle=False) as archive:
            signals = dict(time_s=archive['time_s'].copy())
        peak = science['cases'][case]['commands']['normalized_clipping_excess']['maximum']
        reports[case] = case_audit(data, signals, job, radius, peak, output)
    for item in bindings:
        path = Path(item['path'])
        if sha(path) != item['sha256'] or path.stat().st_size != item['bytes']:
            raise ValueError('Audit source/input changed during arithmetic')
    write(output/'report.json', dict(
        iteration='I-76', plan_timing='Defined during campaign before outcome reads; diagnostic only, not a prephysics control gate.',
        cases=reports, all_eighteen_cases_included=True, all_ledgers_verified=True,
        canonical_status_unchanged=science['decision']['status'], no_new_physics=True,
        no_new_inference=True, no_gate_changes=True,
        mixed_unit_note='Five-command first3 are motor torques Nm; last2 prescribed belt speeds m/s. Joint qvel6:9 are shaft,shellA,shellB rad/s. Allocation reference first3 are joint rad/s and last2 belt m/s; all extrema have explicit per-channel units.',
        interpretation_limit='Algebraic decomposition localizes command demand; it does not establish unique physical cause, contact-map conditioning, contact timing, net yaw authority or whole-path causality.'))
    write(output/'artifact_manifest.json', dict(files=[dict(path=path.name, sha256=sha(path), bytes=path.stat().st_size)
          for path in sorted(output.iterdir()) if path.is_file() and path.name != 'artifact_manifest.json']))
    print(json.dumps({case: dict(time_s=None if row['maximum_held_clipping_event'] is None else row['maximum_held_clipping_event']['time_s'],
                                clipping_excess=None if row['maximum_held_clipping_event'] is None else row['maximum_held_clipping_event']['normalized_clipping_excess'])
                      for case, row in reports.items()}, allow_nan=False))


if __name__ == '__main__':
    main()
