"""The momentum observer: the ground wrench from motion, drive torques and the model alone.

The observer must reproduce the contact force and moment the simulator
applied, without reading any contact data, and the contact-aware controller
must run on it without the solved force and moment.
"""
import math

import mujoco
import numpy as np
import pytest

import spherical_wheel.offroad_control as offroad_control
from spherical_wheel.contact_sensing import MomentumObserver, intrinsic_normal
from spherical_wheel.model import REPO_ROOT
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import ReferencePath, Workspace, place_on_ground, run_offroad
from spherical_wheel.offroad_suite import scenario_job
from spherical_wheel.sim import Plant
from spherical_wheel.terrain import Terrain, make_terrain

SEED_CACHE = REPO_ROOT/'generated'/'offroad'/'flat'/'control_schedule.npz'


@pytest.fixture(scope='module')
def workspace(tmp_path_factory):
    return Workspace(directory=tmp_path_factory.mktemp('momentum_observer'),
                     seed_cache=SEED_CACHE if SEED_CACHE.is_file() else None)


def _airborne_plant(qvel_scale=0.):
    plant = Plant()
    qpos = plant.data.qpos.copy()
    qpos[2] = 3.
    rng = np.random.default_rng(3)
    plant.reset(qpos=qpos, qvel=qvel_scale*rng.normal(size=plant.model.nv))
    return plant


def test_model_terms_are_the_ones_mujoco_integrates():
    """Mass matrix, bias and passive forces, and the drive map, exactly as MuJoCo uses them.

    With MuJoCo's own stepping rule (forces at the start of the physics step,
    the change of mass matrix times the new velocity) the free-joint rows then
    balance to round-off over violent airborne motion with random torques.
    """
    plant = _airborne_plant(.5)
    model, data = plant.model, plant.data
    observer = MomentumObserver(model, plant.fork_id)
    rng = np.random.default_rng(4)
    mass = np.empty((model.nv, model.nv))
    h = model.opt.timestep
    for _ in range(60):
        plant.apply_input(rng.uniform(-1, 1, 5)*plant.bounds)
        for _ in range(plant.substeps):
            mujoco.mj_forward(model, data)
            mass0, bias0, passive0, _, _ = observer._terms(data.qpos, data.qvel)
            mujoco.mj_fullM(model, data, mass)
            np.testing.assert_array_equal(mass0, mass)
            np.testing.assert_allclose(bias0, data.qfrc_bias, rtol=0, atol=1e-12)
            np.testing.assert_allclose(passive0, data.qfrc_passive, rtol=0, atol=1e-15)
            np.testing.assert_allclose(observer._torques(plant.last_input), data.qfrc_actuator, rtol=0, atol=1e-15)
            qvel0 = data.qvel.copy()
            mujoco.mj_step(model, data)
            mass1, _, _, _, _ = observer._terms(data.qpos, data.qvel)
            residual = (mass1@data.qvel-mass0@qvel0-h*(observer._torques(plant.last_input)+passive0-bias0)
                        - (mass1-mass0)@data.qvel)/h
            assert np.abs(residual[:6]).max() < 1e-9


def test_free_joint_rows_are_the_force_and_its_moment_about_the_wheel_centre():
    plant = _airborne_plant(.3)
    model, data = plant.model, plant.data
    rng = np.random.default_rng(6)
    rotation = data.xmat[plant.fork_id].reshape(3, 3)
    for body in ('fork', 'carrier', 'shell_A', 'shell_B'):
        index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
        force, point = rng.normal(size=3), data.xpos[index]+.1*rng.normal(size=3)
        generalized = np.zeros(model.nv)
        mujoco.mj_applyFT(model, data, force, np.zeros(3), point, index, generalized)
        world = MomentumObserver._to_world(generalized, rotation)
        np.testing.assert_allclose(world[:3], force, atol=1e-14)
        np.testing.assert_allclose(world[3:6], np.cross(point-data.xpos[plant.fork_id], force), atol=1e-14)


def test_a_push_in_the_air_is_recovered_and_drive_torques_cancel():
    """No ground: the only external load is a known push, under random drive torques."""
    plant = _airborne_plant()
    model, data = plant.model, plant.data
    observer = MomentumObserver(model, plant.fork_id, cutoff_rad_s=1e9)
    observer.reset(data.qpos, data.qvel, data.time)
    rng = np.random.default_rng(8)
    for _ in range(80):
        push = np.r_[3.*rng.normal(size=3), .2*rng.normal(size=3)]
        plant.apply_input(.3*rng.uniform(-1, 1, 5)*plant.bounds)
        data.xfrc_applied[:] = 0
        data.xfrc_applied[plant.fork_id] = push
        applied = np.zeros(6)
        for _ in range(plant.substeps):
            mujoco.mj_step(model, data)
            arm = data.xipos[plant.fork_id]-data.xpos[plant.fork_id]
            applied += np.r_[push[:3], push[3:]+np.cross(arm, push[:3])]/plant.substeps
        mujoco.mj_forward(model, data)
        assert data.ncon == 0
        force, moment = observer.update(data.qpos, data.qvel, plant.last_input, data.time)
        assert np.abs(force-applied[:3]).max() < 2e-2
        assert np.abs(moment-applied[3:]).max() < 5e-3


@pytest.mark.parametrize('name', ['hills', 'bumps', 'washboard'])
def test_observer_reproduces_the_contact_impulse_the_simulator_applied(workspace, name):
    """On terrain, under the contact-aware controller, against MuJoCo's applied contact impulse."""
    job = scenario_job(name)
    terrain, path = Terrain.from_spec(job['terrain']), ReferencePath.from_spec(job['path'])
    plant = workspace.plant(terrain)
    model, data = plant.model, plant.data
    controller = OffroadController(plant, workspace.design, OffroadParams(), .30)
    place_on_ground(plant, controller, path.origin, .30, True, 0.)
    observer = MomentumObserver(model, plant.fork_id, cutoff_rad_s=1e9)
    observer.reset(data.qpos, data.qvel, data.time)
    h = model.opt.timestep
    force_error, moment_error, normal_error = [], [], []
    for _ in range(int(6./plant.dt)):
        applied, _ = controller.command(*path(data.time))
        plant.apply_input(applied)
        data.xfrc_applied[:] = 0
        impulse = np.zeros(6)
        for _ in range(plant.substeps):
            plant.sync_ground()
            mujoco.mj_step(model, data)
            impulse += MomentumObserver._to_world(data.qfrc_constraint, data.xmat[plant.fork_id].reshape(3, 3))[:6]*h
        plant._forward()
        controller.after_step()
        observer.update(data.qpos, data.qvel, plant.last_input, data.time)
        mean = impulse/plant.dt
        force_error.append(np.linalg.norm(observer.interval_mean[:3]-mean[:3]))
        moment_error.append(np.linalg.norm(observer.interval_mean[3:6]-mean[3:]))
        if mean[2] > .3*plant.total_mass*9.81:
            # Against the normal of the applied impulse, not the terrain normal:
            # on the shell rims the force-and-moment normal is not the ground's.
            estimate = intrinsic_normal(*observer.wrench(), plant.radius)
            reference = intrinsic_normal(mean[:3], mean[3:], plant.radius)
            normal_error.append(math.degrees(math.acos(min(1., float(estimate@reference)))))
    assert max(force_error) < 5e-3 and np.median(force_error) < 1e-4
    assert max(moment_error) < 3e-3 and np.median(moment_error) < 3e-4
    assert max(normal_error) < .05


@pytest.mark.parametrize('grade, cross', [(8., 0.), (0., 6.), (10., 5.)])
def test_static_start_finds_the_slope_without_ground_truth(workspace, grade, cross):
    terrain = make_terrain('slope', grade_deg=grade, cross_deg=cross)
    plant = workspace.plant(terrain)
    controller = OffroadController(plant, workspace.design, OffroadParams(mode='momentum_observer'), .30)
    place_on_ground(plant, controller, (.5, .2), .30, True, 0.)
    truth = plant.terrain.normal_at(.5, .2)
    assert math.degrees(math.acos(min(1., float(controller.estimator.normal@truth)))) < .05


def test_observer_mode_never_uses_the_solved_force_or_moment(workspace, monkeypatch):
    """Corrupting every solved-force field the estimator could read leaves the commands unchanged."""
    terrain = make_terrain('slope', grade_deg=8.)
    path = ReferencePath(.3, 0., 4., start_s=.5)

    def commands():
        plant = workspace.plant(terrain)
        controller = OffroadController(plant, workspace.design, OffroadParams(mode='momentum_observer'), .30)
        place_on_ground(plant, controller, path.origin, .30, True, 0.)
        out = []
        for _ in range(300):
            applied, _ = controller.command(*path(plant.data.time))
            out.append(applied)
            plant.step(applied)
            controller.after_step()
        return np.array(out)

    clean = commands()
    real = offroad_control.measure_contact

    def corrupted(plant):
        measurement = real(plant)
        return {**measurement, 'force_N': -3*measurement['force_N'], 'moment_Nm': 5*measurement['moment_Nm']+1.,
                'normal': np.array([1., 0., 0.]), 'normal_force_N': 0., 'tangential_force_norm_N': 99.,
                'slip_speed_m_s': 9.}

    monkeypatch.setattr(offroad_control, 'measure_contact', corrupted)
    np.testing.assert_array_equal(commands(), clean)


def test_observer_mode_climbs_a_ramp_like_force_sensing(workspace):
    terrain = make_terrain('ramp', grade_deg=8.)
    path = ReferencePath(1., 0., 12.)
    results = {}
    for mode in ('terrain_aware', 'momentum_observer'):
        log, summary = run_offroad(workspace, terrain, path, OffroadParams(mode=mode))
        results[mode] = summary
    observed = results['momentum_observer']
    assert observed['controller'] == 'momentum_observer' and observed['completed']
    assert observed['contact']['normal_estimate_error_rms_deg'] < .3
    assert observed['tracking']['final_error_m'] < results['terrain_aware']['tracking']['final_error_m']+.01
    assert observed['sensing']['observer_force_error_median_N'] < .05


def test_momentum_observer_is_its_own_controller_mode():
    """Selected by mode alone; it cannot be combined with the solver's normal."""
    assert OffroadParams(mode='momentum_observer').ground_aware
    assert not OffroadParams(mode='flat_baseline').ground_aware
    with pytest.raises(ValueError):
        OffroadParams(mode='momentum_observer', normal_source='solver')
    with pytest.raises(ValueError):
        OffroadParams(normal_source='observer')
