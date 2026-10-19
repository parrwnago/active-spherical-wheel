"""Live-lab semantics: forces are physical, finite, logged, and independent of UI polling."""
import csv
import io

import mujoco
import numpy as np
import pytest

from spherical_wheel.lab_runtime import SimulationLab
from spherical_wheel.model import generate_model


@pytest.fixture(scope='module')
def session(tmp_path_factory):
    model_path = generate_model(output_dir=tmp_path_factory.mktemp('lab_model'))
    return SimulationLab(model_path=model_path)


@pytest.fixture
def lab(session):
    session.command({'action': 'controller', 'enabled': True})
    session.command({'action': 'belt', 'enabled': True})
    session.command({'action': 'reset', 'scenario': 'shell'})
    return session


def test_push_lifetime_uses_simulation_time_and_has_no_residual_force(lab):
    lab.command({'action': 'reset', 'scenario': 'polar'})
    lab.command({'action': 'push', 'force_N': [0., .5, 0.], 'duration_s': .02})
    for _ in range(5):
        lab.tick()
    assert lab.plant.data.time == 0.
    assert len(lab.pulses) == 1
    assert len(lab.samples) == 1
    lab.command({'action': 'resume'})
    lab.tick()
    lab.tick()
    lab.command({'action': 'pause'})
    paused_time = lab.plant.data.time
    for _ in range(10):
        lab.tick()
    assert lab.plant.data.time == paused_time
    lab.command({'action': 'resume'})
    for _ in range(3):
        lab.tick()
    assert lab.plant.data.time == pytest.approx(.025)
    forces = np.asarray([s['force_N'] for s in lab.samples])
    np.testing.assert_allclose(forces[:, [0, 2]], 0., atol=0.)
    assert np.count_nonzero(forces[:, 1]) == 4
    assert forces[:, 1].sum() * lab.dt == pytest.approx(.5*.02)
    np.testing.assert_allclose(lab.plant.data.xfrc_applied, 0., atol=0.)
    np.testing.assert_allclose(lab.plant.data.qfrc_applied, 0., atol=0.)
    assert not lab.pulses


def test_belt_ablation_updates_mesh_and_smooth_contact_geoms(lab):
    lab.command({'action': 'reset', 'scenario': 'polar', 'initial_tilt_deg': [2., 0.]})
    lab.advance()
    assert len(lab.plant.all_belt_ids) == 4
    assert abs(lab.samples[-1]['input'][3]) > .1
    surface = lab.plant.model.geom_surfacevel[lab.plant.all_belt_ids]
    assert np.max(abs(surface)) > .1
    lab.command({'action': 'belt', 'enabled': False})
    lab.advance()
    assert abs(lab.samples[-1]['command'][3]) > .001
    np.testing.assert_array_equal(lab.samples[-1]['input'][3:], 0.)
    np.testing.assert_array_equal(lab.plant.model.geom_surfacevel[lab.plant.all_belt_ids], 0.)
    lab.command({'action': 'controller', 'enabled': False})
    lab.advance()
    np.testing.assert_array_equal(lab.samples[-1]['input'], 0.)
    np.testing.assert_array_equal(lab.plant.data.ctrl, 0.)
    np.testing.assert_array_equal(lab.plant.data.qfrc_applied, 0.)
    np.testing.assert_array_equal(lab.plant.data.xfrc_applied, 0.)


def test_fallen_session_stops_until_reset_and_reset_clears_pushes(lab):
    lab.command({'action': 'reset', 'scenario': 'polar', 'initial_tilt_deg': [2., 0.]})
    lab.command({'action': 'controller', 'enabled': False})
    lab.command({'action': 'resume'})
    lab.advance(steps=250)
    assert lab.failed
    assert not lab.running
    failure_time = lab.plant.data.time
    lab.tick()
    lab.advance()
    assert lab.plant.data.time == failure_time
    with pytest.raises(ValueError, match='Reset'):
        lab.command({'action': 'resume'})
    lab.command({'action': 'reset', 'scenario': 'polar'})
    lab.command({'action': 'push', 'force_N': [0., .5, 0.], 'duration_s': .1})
    lab.advance()
    assert lab.pulses
    assert np.linalg.norm(lab.plant.data.xfrc_applied) > 0.
    previous_run = lab.run_id
    lab.command({'action': 'reset', 'scenario': 'shell'})
    state = lab.state()
    assert lab.run_id != previous_run
    assert not state['failed'] and not state['running']
    assert state['time_s'] == 0.
    assert state['sample_count'] == 1
    assert not lab.pulses and lab.last_push is None
    np.testing.assert_array_equal(lab.plant.data.xfrc_applied, 0.)
    np.testing.assert_array_equal(lab.plant.last_input, 0.)
    np.testing.assert_array_equal(lab.plant.model.geom_surfacevel[lab.plant.all_belt_ids], 0.)


def test_reference_is_fixed_to_planned_origin_not_current_position(lab):
    lab.command({'action': 'trajectory', 'kind': 'curve', 'duration_s': 5., 'distance_m': .04, 'lateral_m': .02})
    lab.advance(steps=40)
    reference_before = tuple(v.copy() for v in lab._reference())
    # Move only the simulated actual state. A drifting target would hide error.
    lab.plant.data.qpos[:2] += np.array([.20, -.15])
    mujoco.mj_forward(lab.plant.model, lab.plant.data)
    for actual, expected in zip(lab._reference(), reference_before):
        np.testing.assert_array_equal(actual, expected)
    assert np.linalg.norm(lab.plant.data.qpos[:2]-lab._reference()[0]) > .2


def test_csv_contains_actual_force_input_and_interval_alignment(lab):
    lab.command({'action': 'reset', 'scenario': 'polar', 'initial_tilt_deg': [2., 0.]})
    lab.command({'action': 'push', 'force_N': [0., .5, 0.], 'duration_s': .02})
    lab.advance(steps=5)
    rows = list(csv.DictReader(io.StringIO(lab.export_csv())))
    assert len(rows) == len(lab.samples) == 6
    names = ['shaft_torque_Nm', 'shell_A_torque_Nm', 'shell_B_torque_Nm', 'belt_A_speed_m_s', 'belt_B_speed_m_s']
    for row, sample in zip(rows, lab.samples):
        assert float(row['time_s']) == sample['time_s']
        assert float(row['interval_start_s']) == sample['interval_start_s']
        np.testing.assert_array_equal([float(row[n]) for n in ['force_x_N', 'force_y_N', 'force_z_N']], sample['force_N'])
        np.testing.assert_array_equal([float(row[n]) for n in names], sample['input'])
    assert float(rows[-1]['time_s'])-float(rows[-1]['interval_start_s']) == pytest.approx(lab.dt)
    assert sum(float(row['force_y_N'])*lab.dt for row in rows) == pytest.approx(.01)
    assert abs(float(rows[1]['belt_A_speed_m_s'])) > .1


def test_recovery_time_waits_for_all_overlapping_pushes_to_end(lab):
    lab.command({'action': 'push', 'force_N': [0., .001, 0.], 'duration_s': 1.})
    lab.advance(steps=20)
    lab.command({'action': 'push', 'force_N': [.001, 0., 0.], 'duration_s': .02})
    lab.advance(steps=100)
    assert lab.plant.data.time == pytest.approx(.6)
    assert lab.last_push['time_s']+lab.last_push['duration_s'] < lab.plant.data.time
    assert lab.pulses  # The earlier, longer push is still physically active.
    assert lab.recovery_time_s is None
    lab.advance(steps=180)
    assert not lab.pulses
    assert lab.recovery_time_s is not None


def test_disabled_incoming_belt_is_neither_commanded_nor_marked_as_preparing(lab):
    lab.command({'action': 'reset', 'scenario': 'transition'})
    point = lab._design(1.2)
    velocity = np.zeros(lab.plant.model.nv)
    velocity[:2] = [.12, .09]
    velocity[6] = 1.
    lab.plant.reset(qpos=point['equilibrium']['qpos'], qvel=velocity)
    proposed = lab._synchronizer.preview(np.zeros(5))
    assert proposed['blend_weights'][3] > .1
    lab.command({'action': 'belt', 'enabled': False})
    lab.advance()
    state = lab.state()
    np.testing.assert_array_equal(state['inputs'][3:], 0.)
    np.testing.assert_array_equal(state['synchronization']['blend_weights'][3:], 0.)
    assert all(t['blend_weight'] == 0. and t['disabled']
               for t in state['synchronization']['targets'] if t['command_index'] >= 3)
    np.testing.assert_array_equal(lab.plant.model.geom_surfacevel[lab.plant.all_belt_ids], 0.)


def test_live_tracking_can_recover_from_a_finite_push(lab):
    lab.command({'action': 'trajectory', 'kind': 'curve', 'duration_s': 5., 'distance_m': .04, 'lateral_m': .02})
    lab.advance(steps=200)
    lab.command({'action': 'push', 'force_N': [.5, 0., 0.], 'duration_s': .1})
    lab.advance(steps=800)
    state = lab.state()
    assert state['time_s'] == pytest.approx(5.)
    assert not state['failed']
    assert state['tilt_deg'] < 1.
    assert state['tracking_error_m'] < .01
    np.testing.assert_allclose(lab.plant.data.qfrc_applied, 0., atol=0.)
    np.testing.assert_allclose(lab.plant.data.xfrc_applied, 0., atol=0.)
    assert all(np.all(abs(sample['input']) <= lab.plant.bounds) for sample in lab.samples)


def test_polar_tracking_crosses_from_belt_to_shell_and_recovers_push(lab, tmp_path, monkeypatch):
    """A fixed polar gain is insufficient after this path exits the polar cap."""
    import json
    from spherical_wheel import lab_runtime

    lab.command({'action': 'reset', 'scenario': 'polar'})
    lab.command({'action': 'trajectory', 'kind': 'curve', 'duration_s': 10., 'distance_m': .04, 'lateral_m': .02})
    assert lab.plant.model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert lab.plant.model.neq == 0
    contact_checks = []
    seen_contacts = set()
    for index in range(101):
        names = sorted({name for contact in lab.plant.contact_summary()
                        if 'floor' in contact['geoms'] and contact['normal_force_N'] > .001
                        for name in contact['geoms'] if name != 'floor'})
        seen_contacts.update(names)
        contact_checks.append({'time_s': float(lab.plant.data.time), 'floor_contacts': names,
                               'selected_gain_phase_rad': lab.selected['phase'],
                               'shaft_phase_rad': float(lab.plant.data.qpos[7])})
        np.testing.assert_array_equal(lab.plant.data.qfrc_applied, 0.)
        nonfork_force = lab.plant.data.xfrc_applied.copy()
        nonfork_force[lab.plant.fork_id] = 0.
        np.testing.assert_array_equal(nonfork_force, 0.)
        # The sole commanded world force is the finite lateral pulse on the fork.
        actual = lab.plant.data.xfrc_applied[lab.plant.fork_id]
        assert actual[1] in (0., .5)
        np.testing.assert_array_equal(actual[[0, 2, 3, 4, 5]], 0.)
        assert not lab.failed
        if index == 100:
            break
        if index == 20:
            assert lab.plant.data.time == pytest.approx(2.)
            lab.command({'action': 'push', 'force_N': [0., .5, 0.], 'duration_s': .1})
        lab.advance(steps=20)

    state = lab.state()
    assert state['time_s'] == pytest.approx(10.)
    assert state['tilt_deg'] < 1.
    assert state['tracking_error_m'] < .01
    assert any(name.startswith('belt_A') for name in seen_contacts)
    assert any(name.startswith('shell_A') for name in seen_contacts), seen_contacts
    assert not any(name.startswith(('fork', 'platform')) for name in seen_contacts)
    forces = np.asarray([sample['force_N'] for sample in lab.samples])
    np.testing.assert_array_equal(forces[:, [0, 2]], 0.)
    assert np.count_nonzero(forces[:, 1]) == 20
    assert forces[:, 1].sum() * lab.dt == pytest.approx(.05)
    np.testing.assert_array_equal(lab.plant.data.xfrc_applied, 0.)

    # Exercise the portable saved model/log rather than persisting a temp model path.
    monkeypatch.setattr(lab_runtime, 'REPO_ROOT', tmp_path)
    saved = lab.save_run()
    with np.load(saved / 'rollout.npz') as rollout:
        assert str(rollout['model_path']) == 'model/model.xml'
        assert (saved / str(rollout['model_path'])).is_file()
        np.testing.assert_array_equal(rollout['force_N'], forces)
    check = {'result': 'passed', 'duration_s': state['time_s'], 'final_tilt_deg': state['tilt_deg'],
             'final_tracking_error_m': state['tracking_error_m'], 'final_shaft_phase_rad': float(lab.plant.data.qpos[7]),
             'push_impulse_Ns': (forces.sum(axis=0)*lab.dt).tolist(),
             'push_time_s': 2., 'push_duration_s': .1, 'observed_floor_contacts': sorted(seen_contacts),
             'free_root_joint': True, 'equality_constraints': 0,
             'no_generalized_external_forces': True, 'no_external_forces_except_finite_fork_Y_push': True,
             'contact_checks': contact_checks,
             'limitations': 'Ideal belt speed servos and provisional physical parameters; this is one demonstrated local path.'}
    (saved / 'polar_tracking_check.json').write_text(json.dumps(check, indent=2))
    print(f'POLAR_TRACKING_ARTIFACTS={saved}')
