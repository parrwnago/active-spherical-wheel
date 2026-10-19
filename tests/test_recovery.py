"""Matched recovery and first-load preparation under the declared physical plant."""
from pathlib import Path
import types

import mujoco
import numpy as np
import pytest

from spherical_wheel.control import find_equilibrium
from spherical_wheel.lab_runtime import SimulationLab
from spherical_wheel.model import generate_model
from spherical_wheel.recovery_control import _fingerprint, _read_cache


@pytest.fixture(scope='module')
def controllers(tmp_path_factory):
    model = generate_model(output_dir=tmp_path_factory.mktemp('recovery_model'))
    advanced = SimulationLab(model_path=model, controller_mode='transition_aware')
    baseline = SimulationLab(model_path=model, controller_mode='local_lqr')
    assert advanced.config == baseline.config
    np.testing.assert_array_equal(advanced.plant.bounds, baseline.plant.bounds)
    return advanced, baseline


def _physical_guard(lab, mass, inertia):
    model, data = lab.plant.model, lab.plant.data
    assert model.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE
    assert model.neq == 0
    np.testing.assert_array_equal(model.body_mass, mass)
    np.testing.assert_array_equal(model.body_inertia, inertia)
    np.testing.assert_array_equal(data.qfrc_applied, 0.)
    actual = data.xfrc_applied.copy()
    actual[lab.plant.fork_id, :3] -= lab.samples[-1]['force_N']
    np.testing.assert_array_equal(actual, 0.)
    assert np.all(abs(lab.samples[-1]['input']) <= lab.plant.bounds)


def _repeated_pushes(lab, scenario, strength, duration, spacing, directions, synchronize):
    lab.command({'action': 'synchronization', 'enabled': synchronize})
    lab.command({'action': 'controller', 'enabled': True})
    lab.command({'action': 'belt', 'enabled': True})
    lab.command({'action': 'reset', 'scenario': scenario})
    mass, inertia = lab.plant.model.body_mass.copy(), lab.plant.model.body_inertia.copy()
    events = {round((1.+j*spacing)/lab.dt): strength*np.asarray(directions[j % len(directions)]) for j in range(8)}
    for index in range(round(12./lab.dt)):
        if index in events:
            lab.command({'action': 'push', 'force_N': events[index].tolist(), 'duration_s': duration})
        lab.advance()
        _physical_guard(lab, mass, inertia)
        if lab.failed:
            break
    samples = list(lab.samples)
    force = np.asarray([sample['force_N'] for sample in samples])
    reference = np.asarray([sample['reference_xy'] for sample in samples])
    np.testing.assert_array_equal(reference, np.zeros_like(reference))
    assert np.max(np.linalg.norm(force, axis=1)) <= max(1., 2*strength)+1e-12
    return {'failed': lab.failed, 'time_s': lab.plant.data.time,
            'max_tilt_deg': max(sample['tilt_deg'] for sample in samples),
            'final_tilt_deg': samples[-1]['tilt_deg'],
            'final_speed_m_s': np.linalg.norm(samples[-1]['qvel'][:2]),
            'final_angular_speed_rad_s': np.linalg.norm(samples[-1]['qvel'][3:6]),
            'final_position_error_m': np.linalg.norm(samples[-1]['qpos'][:2]-reference[-1]),
            'impulse_Ns': force.sum(axis=0)*lab.dt}


@pytest.mark.parametrize('scenario,strength,duration,spacing,directions,old_fall_s', [
    ('shell', .5, .3, .15, [(0., 1., 0.)], 5.89),
    ('polar', 1., .2, .25, [(1., 0., 0.), (0., 1., 0.), (-1., 0., 0.), (0., -1., 0.)], 2.15),
])
def test_repeated_push_recovery_improves_without_changing_plant_or_target(
        controllers, scenario, strength, duration, spacing, directions, old_fall_s):
    advanced, baseline = controllers
    old = _repeated_pushes(baseline, scenario, strength, duration, spacing, directions, True)
    assert old['failed']
    assert old['time_s'] == pytest.approx(old_fall_s, abs=.01)
    recovered = []
    for sync in (False, True):
        result = _repeated_pushes(advanced, scenario, strength, duration, spacing, directions, sync)
        assert not result['failed']
        assert result['time_s'] == pytest.approx(12.)
        assert result['max_tilt_deg'] < 15.
        assert result['final_tilt_deg'] < 1.
        assert result['final_speed_m_s'] < .03
        assert result['final_angular_speed_rad_s'] < .08
        # The shell case balances with a measured position bias; its target was
        # never moved to the actual state. This test does not assert zero bias.
        assert result['final_position_error_m'] < .10
        expected_impulse = [0., 1.2, 0.] if scenario == 'shell' else [0., 0., 0.]
        np.testing.assert_allclose(result['impulse_Ns'], expected_impulse, atol=1e-12)
        recovered.append(result)
    assert recovered[1]['final_position_error_m'] < recovered[0]['final_position_error_m']


def test_gain_cache_fingerprint_rejects_changed_physical_configuration_or_bounds(controllers):
    advanced, _ = controllers
    designer = advanced.designer
    cache = advanced.model_path.parent/'control_schedule.npz'
    original, _ = _fingerprint(designer)
    points, metadata = _read_cache(cache, original, designer)
    assert points and metadata['cache_loaded']
    old_mass = designer.config['geometry']['platform']['mass_kg']
    try:
        designer.config['geometry']['platform']['mass_kg'] = old_mass*1.01
        changed, _ = _fingerprint(designer)
        assert changed != original
        with pytest.raises(ValueError, match='fingerprint'):
            _read_cache(cache, changed, designer)
    finally:
        designer.config['geometry']['platform']['mass_kg'] = old_mass
    old_bound = designer.bounds[0]
    try:
        designer.bounds[0] *= 1.01
        changed, _ = _fingerprint(designer)
        assert changed != original
        with pytest.raises(ValueError, match='fingerprint'):
            _read_cache(cache, changed, designer)
    finally:
        designer.bounds[0] = old_bound
    restored, _ = _fingerprint(designer)
    assert restored == original


def test_feedback_follows_heading_and_wraps_cyclic_shell_phase_errors(controllers):
    advanced, _ = controllers
    advanced.command({'action': 'reset', 'scenario': 'shell'})
    plant = advanced.plant
    eq, design = advanced._ordinary['equilibrium'], advanced._ordinary['design']
    plant.reset(qpos=eq['qpos'], qvel=eq['qvel'])
    perturbation = np.zeros(18)
    perturbation[:2] = [.012, -.009]
    perturbation[3:5] = [.018, -.012]
    perturbation[9:11] = [.023, -.017]
    perturbation[12:15] = [.04, -.03, .01]
    plant.perturb(perturbation)
    qpos, qvel = plant.data.qpos.copy(), plant.data.qvel.copy()
    qref = eq['qpos'].copy()
    lean = np.zeros(9); lean[3:5] = [.003, -.002]
    mujoco.mj_integratePos(plant.model, qref, lean, 1.)
    vref = np.zeros(9); vref[:2] = [.005, .001]
    first, _ = advanced._feedback.compute(qref, vref, eq['u'], design['K'])
    yaw = .73
    rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    yaw_quat = np.array([np.cos(yaw/2), 0., 0., np.sin(yaw/2)])
    inverse_yaw = yaw_quat.copy(); inverse_yaw[1:] *= -1
    qpos[:2], qvel[:2] = rotation@qpos[:2], rotation@qvel[:2]
    mujoco.mju_mulQuat(qpos[3:7], yaw_quat, qpos[3:7].copy())
    qref[:2], vref[:2] = rotation@qref[:2], rotation@vref[:2]
    rotated_lean = np.empty(4)
    mujoco.mju_mulQuat(rotated_lean, yaw_quat, qref[3:7])
    mujoco.mju_mulQuat(qref[3:7], rotated_lean, inverse_yaw)
    plant.reset(qpos=qpos, qvel=qvel)
    rotated, _ = advanced._feedback.compute(qref, vref, eq['u'], design['K'])
    np.testing.assert_allclose(rotated, first, atol=1e-8, rtol=1e-8)
    qref[8] += 2*np.pi
    qref[9] -= 4*np.pi
    wrapped, _ = advanced._feedback.compute(qref, vref, eq['u'], design['K'])
    np.testing.assert_allclose(wrapped, rotated, atol=1e-8, rtol=1e-8)


def _incoming_shell_observation(lab):
    model, data = lab.plant.model, lab.plant.data
    geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'shell_A_smooth')
    body = int(model.geom_bodyid[geom])
    point = data.qpos[:3]+[0., 0., -lab.config['geometry']['sphere_radius_m']]
    jacobian = np.zeros((3, model.nv))
    mujoco.mj_jac(model, data, jacobian, None, point, body)
    candidate = np.linalg.norm((jacobian@data.qvel)[:2])
    normal, slip = 0., 0.
    for index, contact in enumerate(data.contact):
        if contact.efc_address < 0:
            continue
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or '' for g in contact.geom]
        if 'floor' not in names or not any(name.startswith('shell_A') for name in names):
            continue
        force = np.zeros(6); mujoco.mj_contactForce(model, data, index, force)
        normal += max(0., force[0])
        mujoco.mj_jac(model, data, jacobian, None, contact.pos, body)
        slip = max(slip, np.linalg.norm(contact.frame.reshape(3, 3)[1:]@(jacobian@data.qvel)))
    return candidate, normal, slip


def _controlled_cap_exit(lab, sync):
    lab.command({'action': 'synchronization', 'enabled': sync})
    lab.command({'action': 'reset', 'scenario': 'polar'})
    eq = find_equilibrium(lab.designer, 1.74)
    velocity = np.zeros(9); velocity[:2] = [.096, .1]; velocity[6] = .8
    lab.plant.reset(qpos=eq['qpos'], qvel=velocity)
    lab.nominal_joints = eq['qpos'][7:10].copy()
    lab.anchor = eq['qpos'][:2].copy()
    lab.plant.apply_input([0., 0., 0., -.1, 0.])
    start = (lab.plant.data.qpos.copy(), lab.plant.data.qvel.copy())
    anchor, desired_velocity = lab.anchor.copy(), velocity[:2].copy()
    original_reference = lab._reference
    lab._reference = types.MethodType(lambda self:(anchor+self.plant.data.time*desired_velocity,
                                                   desired_velocity, np.zeros(2)), lab)
    lab.trajectory['kind'] = 'line'
    mass, inertia = lab.plant.model.body_mass.copy(), lab.plant.model.body_inertia.copy()
    observations = []
    try:
        for _ in range(100):
            candidate, normal, slip = _incoming_shell_observation(lab)
            observations.append([lab.plant.data.time, candidate, normal, slip,
                                 lab.samples[-1]['sync_weights'][1]])
            lab.advance()
            _physical_guard(lab, mass, inertia)
            assert not lab.failed
            np.testing.assert_array_equal(lab.samples[-1]['force_N'], 0.)
    finally:
        lab._reference = original_reference
    observed = np.asarray(observations)
    loaded = np.flatnonzero(observed[:, 2] > 1.)
    assert loaded.size
    return start, observed, int(loaded[0])


def test_incoming_shell_is_prepared_before_first_load_not_only_after_contact(controllers):
    advanced, _ = controllers
    start_off, off, first_off = _controlled_cap_exit(advanced, False)
    start_on, on, first_on = _controlled_cap_exit(advanced, True)
    for actual, expected in zip(start_on, start_off):
        np.testing.assert_allclose(actual, expected, atol=1e-12)
    assert np.max(on[:first_on, 4]) > .1
    np.testing.assert_array_equal(off[:, 4], 0.)
    # These observations precede the first 1 N incoming normal load. The
    # measured point velocity includes root/carrier motion and shell rotation.
    assert on[first_on-1, 1] < .7*off[first_off-1, 1]
    assert on[first_on-4, 1] < .7*off[first_off-4, 1]
    assert on[first_on, 3] < .4*off[first_off, 3]
