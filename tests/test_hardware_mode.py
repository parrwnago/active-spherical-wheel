"""The hardware controller mode: no contact, ground or terrain data, only what a real robot measures.

The controller sees the robot through a ``StateMirror`` (measured state and
applied inputs, collisions off, no ground). The momentum observer's force and
moment locate the support (``SupportLocator``), and those estimated contacts
replace the solver's everywhere the other modes read them.
"""
import mujoco
import numpy as np
import pytest

import spherical_wheel.contact_sensing as contact_sensing
import spherical_wheel.offroad_control as offroad_control
import spherical_wheel.synchronization as synchronization
from spherical_wheel.contact_sensing import SUPPORTS, SupportLocator, measure_contact, measure_records
from spherical_wheel.model import REPO_ROOT
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import (ReferencePath, Workspace, measure_for, place_on_ground,
                                                 run_offroad)
from spherical_wheel.offroad_suite import scenario_job
from spherical_wheel.terrain import Terrain, make_terrain

SEED_CACHE = REPO_ROOT/'generated'/'offroad'/'flat'/'control_schedule.npz'


@pytest.fixture(scope='module')
def workspace(tmp_path_factory):
    return Workspace(directory=tmp_path_factory.mktemp('hardware_mode'),
                     seed_cache=SEED_CACHE if SEED_CACHE.is_file() else None)


def _locator(controller):
    return SupportLocator(controller.plant.model, controller.plant.fork_id, controller.radius,
                          controller.nominal_friction, .05*controller.weight)


def _shares(loads):
    total = np.array([loads[name] for name in SUPPORTS])
    return total/max(total.sum(), 1e-12)


@pytest.mark.parametrize('name', ['hills', 'washboard'])
def test_locator_finds_the_solved_surface_and_point_from_the_solved_force_and_moment(workspace, name):
    """Patch geometry alone: given the solver's force and moment, the solver's surfaces and contact point.

    Washboard runs crosswise, so the support crosses the equatorial gap and the
    rims share the load. Disagreements are confined to handovers, where two
    surfaces carry load at once and the locator gives it to one.
    """
    job = scenario_job(name)
    terrain, path = Terrain.from_spec(job['terrain']), ReferencePath.from_spec(job['path'])
    plant = workspace.plant(terrain)
    controller = OffroadController(plant, workspace.design, OffroadParams(), .30)
    place_on_ground(plant, controller, path.origin, .30, True, 0.)
    locator = _locator(controller)
    same, share_error, point_error = [], [], []
    for _ in range(int(8./plant.dt)):
        applied, diagnostics = controller.command(*path(plant.data.time))
        measurement = diagnostics['measurement']
        if measurement['normal_force_N'] > .3*controller.weight:
            located = measure_records(plant, locator.records(plant.data, measurement['force_N'],
                                                             measurement['moment_Nm']))
            true, estimated = _shares(measurement['loads_N']), _shares(located['loads_N'])
            same.append(true.argmax() == estimated.argmax())
            share_error.append(np.abs(true-estimated).max())
            if true.max() > .999:
                point_error.append(np.linalg.norm(located['point_m']-measurement['point_m']))
        plant.step(applied)
        controller.after_step()
    assert np.mean(same) > .98
    assert np.median(share_error) < 1e-3
    assert np.median(point_error) < 2e-4 and np.percentile(point_error, 95) < 2e-3


def test_rims_share_the_load_by_the_lever_rule(workspace):
    """Two rim contacts carrying given shares are recovered from their net force and moment alone."""
    mirror = workspace.mirror()
    model, data = mirror.model, mirror.data
    params = OffroadParams(mode='hardware')
    controller = OffroadController(mirror, workspace.design, params, .30)
    locator = controller.locator
    rng = np.random.default_rng(2)
    for _ in range(50):
        qpos = data.qpos.copy()
        qpos[3:7] = rng.normal(size=4)
        qpos[3:7] /= np.linalg.norm(qpos[3:7])
        qpos[7:10] = rng.uniform(-np.pi, np.pi, 3)
        mirror.set_state(qpos, np.zeros(model.nv), 0., np.zeros(5))
        centre = data.xpos[mirror.fork_id]
        shells = locator.shells
        axis = data.xmat[shells[0]['body']].reshape(3, 3)[:, 0]
        # A ground normal whose sphere support lies in the gap.
        down = np.cross(axis, rng.normal(size=3))
        down /= np.linalg.norm(down)
        normal = -down
        rims = [patch['sign']*patch['lower']*float(data.xmat[patch['body']].reshape(3, 3)[:, 0]@axis)
                for patch in shells]
        points = [centre+x*axis+np.sqrt(locator.radius**2-x*x)*down for x in rims]
        share = rng.uniform(.05, .95)
        weight = controller.weight*rng.uniform(.5, 1.5)
        tangent = .2*np.cross(normal, axis)
        forces = [s*weight*(normal+tangent) for s in (share, 1.-share)]
        force = sum(forces)
        moment = sum(np.cross(p-centre, f) for p, f in zip(points, forces))
        records = locator.records(data, force, moment)
        loads = {record['name']: record['normal_force'] for record in records}
        located = loads.get(shells[0]['name'], 0.)/sum(loads.values())
        assert abs(located-share) < 2e-3
        # On the rims the sensed normal points from the equivalent support to
        # the centre, up to asin(gap/R) = 4.8 deg off the ground normal along
        # the axis, so the load along it is up to 1-cos(4.8 deg) = 0.35% low.
        assert abs(sum(loads.values())-weight*float(normal@(normal+tangent))) < 4e-3*weight


def test_mirror_holds_only_measurements(workspace):
    mirror = workspace.mirror()
    assert mirror.terrain is None and mirror.ground is None
    assert mirror.model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    plant = workspace.plant(make_terrain('slope', grade_deg=8.))
    plant.reset()
    for _ in range(40):  # falling and landing: the robot has contacts, the mirror none
        plant.step(np.zeros(5))
    assert plant.data.ncon > 0
    mirror.follow(plant)
    assert mirror.data.ncon == 0
    np.testing.assert_array_equal(mirror.data.qpos, plant.data.qpos)
    np.testing.assert_array_equal(mirror.data.qvel, plant.data.qvel)
    np.testing.assert_array_equal(mirror.last_input, plant.last_input)
    np.testing.assert_allclose(mirror.data.xpos[mirror.fork_id], plant.data.xpos[plant.fork_id], atol=1e-12)
    with pytest.raises(RuntimeError):
        mirror.step(np.zeros(5))
    with pytest.raises(RuntimeError):
        mirror.reset()


def test_hardware_mode_never_reads_a_contact(workspace, monkeypatch):
    """Every path to solved contacts raises; the hardware controller still balances and drives up a slope."""
    terrain = make_terrain('slope', grade_deg=8.)
    plant = workspace.plant(terrain)
    mirror = workspace.mirror()
    controller = OffroadController(mirror, workspace.design, OffroadParams(mode='hardware'), .30)
    path = ReferencePath(.3, 0., 4., start_s=.5)
    place_on_ground(plant, controller, path.origin, .30, True, 0.)

    def forbidden(*args, **kwargs):
        raise AssertionError('the hardware controller read a contact')

    for module, name in ((offroad_control, 'measure_contact'), (offroad_control, 'rolling_feedforward'),
                         (contact_sensing, 'records_from_mujoco'), (mujoco, 'mj_contactForce')):
        monkeypatch.setattr(module, name, forbidden)
    monkeypatch.setattr(synchronization.ContactSynchronizer, '_normal_loads', forbidden)
    for _ in range(int(5./plant.dt)):
        measure_for(controller, plant)
        applied, diagnostics = controller.command(*path(plant.data.time))
        plant.step(applied)
        controller.after_step()
    monkeypatch.undo()
    truth = measure_contact(plant)
    assert truth['in_contact'] and not plant.balance_failed
    assert np.linalg.norm(plant.data.qpos[:2]-path(plant.data.time)[0]) < .05
    assert diagnostics['measurement']['in_contact']


def test_hardware_mode_climbs_a_ramp_like_force_sensing(workspace):
    terrain = make_terrain('ramp', grade_deg=8.)
    path = ReferencePath(1., 0., 12.)
    results = {mode: run_offroad(workspace, terrain, path, OffroadParams(mode=mode))[1]
               for mode in ('terrain_aware', 'hardware')}
    hardware = results['hardware']
    assert hardware['controller'] == 'hardware' and hardware['completed']
    assert hardware['contact']['normal_estimate_error_rms_deg'] < .3
    assert hardware['tracking']['final_error_m'] < results['terrain_aware']['tracking']['final_error_m']+.01
    # The log's contact columns are the solver's, so the observer is checked against the truth.
    assert hardware['sensing']['observer_force_error_median_N'] < .05


def test_hardware_is_its_own_controller_mode():
    assert OffroadParams(mode='hardware').ground_aware
    with pytest.raises(ValueError):
        OffroadParams(mode='hardware', normal_source='solver')
    with pytest.raises(ValueError):
        OffroadParams(mode='hardware', support_min_load_fraction=0.)
