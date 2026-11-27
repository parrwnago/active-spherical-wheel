"""Bounded reference allocation: feasibility, optimality, and state transitions."""
from dataclasses import replace

import numpy as np
import pytest

from spherical_wheel.rolling_allocation import AllocationSettings, BoundedRollingAllocator


def _cost(drive, matrix, target, previous, settings):
    scale = BoundedRollingAllocator.scale
    return (np.linalg.norm(matrix@drive-target)**2
            +settings.effort_m_s**2*np.linalg.norm(drive/scale)**2
            +settings.change_m_s**2*np.linalg.norm((drive-previous)/scale)**2)


def test_nearly_parallel_directions_accept_tracking_error_within_hard_limits():
    matrix = np.array([[.1, .03, 0., 0., 0.], [.03, .009001, 0., 0., 0.]])
    target = np.array([.06, .03])
    # Make the soft penalties weak so this specifically exercises hard bounds.
    settings = AllocationSettings(speed_limits=(.8, .8, .8, .05, .05),
                                  effort_m_s=1e-6, change_m_s=1e-6)
    allocator = BoundedRollingAllocator(settings)
    unbounded = np.linalg.lstsq(matrix, target, rcond=None)[0]
    assert np.max(abs(unbounded)) > 1000.

    drive = allocator.solve(matrix, target, 1.)
    assert np.all(abs(drive) <= np.array(settings.speed_limits)+1e-12)
    assert np.linalg.norm(matrix@drive-target) > .001
    assert allocator.diagnostics['allocation_active_bounds'].any()
    # Clipping the algebraic inverse does not solve the bounded tracking task:
    # the remaining available direction must be reallocated within the box.
    clipped = np.clip(unbounded, -np.array(settings.speed_limits), settings.speed_limits)
    assert _cost(drive, matrix, target, np.zeros(5), settings) < _cost(
        clipped, matrix, target, np.zeros(5), settings)-1e-6


def test_solution_matches_independent_unconstrained_quadratic_optimum():
    matrix = np.array([[.12, .025, 0., 1., 0.], [0., -.03, .07, 0., 1.]])
    settings = AllocationSettings(effort_m_s=.023, change_m_s=.017)
    allocator = BoundedRollingAllocator(settings)
    scale = allocator.scale
    for target in (np.array([.04, -.03]), np.array([-.02, .06])):
        previous = allocator.previous.copy()
        inverse_scale_squared = np.diag(1./scale**2)
        hessian = matrix.T@matrix+(settings.effort_m_s**2+settings.change_m_s**2)*inverse_scale_squared
        linear = matrix.T@target+settings.change_m_s**2*inverse_scale_squared@previous
        expected = np.linalg.solve(hessian, linear)
        drive = allocator.solve(matrix, target, 1.)
        assert not allocator.diagnostics['allocation_active_bounds'].any()
        np.testing.assert_allclose(drive, expected, rtol=1e-8, atol=1e-10)


def test_box_constrained_solution_satisfies_optimality_conditions():
    matrix = np.array([[.12, .03, 0., .4, 0.], [0., -.02, .04, 0., 1.]])
    target = np.array([.8, -.5])
    settings = AllocationSettings(speed_limits=(1., 1., 1., .1, .1),
                                  acceleration_limits=(5., 7., 9., .4, .6))
    allocator = BoundedRollingAllocator(settings)
    dt = .01
    previous = allocator.previous.copy()
    drive = allocator.solve(matrix, target, dt)
    lower = np.maximum(-np.array(settings.speed_limits), previous-dt*np.array(settings.acceleration_limits))
    upper = np.minimum(settings.speed_limits, previous+dt*np.array(settings.acceleration_limits))
    gradient = (matrix.T@(matrix@drive-target)
                +settings.effort_m_s**2*drive/allocator.scale**2
                +settings.change_m_s**2*(drive-previous)/allocator.scale**2)
    at_lower = np.isclose(drive, lower, atol=1e-10, rtol=0.)
    at_upper = np.isclose(drive, upper, atol=1e-10, rtol=0.)
    assert (at_lower | at_upper).any()
    assert np.all(gradient[at_lower] >= -1e-9)
    assert np.all(gradient[at_upper] <= 1e-9)
    np.testing.assert_allclose(gradient[~(at_lower | at_upper)], 0., atol=1e-9)


def test_reference_bounds_hold_through_start_handover_reversal_and_stop():
    settings = AllocationSettings(speed_limits=(2., 2., 2., .2, .2),
                                  acceleration_limits=(5., 7., 9., .4, .6))
    allocator = BoundedRollingAllocator(settings)
    first = np.array([[.12, .03, 0., 1., 0.], [0., .02, .1, 0., 1.]])
    second = np.array([[.12, 0., .03, 0., 1.], [0., .1, .02, 1., 0.]])
    limits, acceleration, dt = np.array(settings.speed_limits), np.array(settings.acceleration_limits), .01
    phases = ((first, np.array([.6, -.4]), 40),
              (second, np.array([-.5, .5]), 40),
              (second, np.zeros(2), 160))
    for matrix, target, count in phases:
        for _ in range(count):
            previous = allocator.previous.copy()
            drive = allocator.solve(matrix, target, dt)
            assert np.all(abs(drive) <= limits+1e-12)
            assert np.all(abs(drive-previous) <= dt*acceleration+1e-12)
    np.testing.assert_allclose(drive, 0., atol=1e-8)


def test_effort_and_change_weights_have_distinct_expected_effects():
    matrix = np.array([[.03, 0., 0., 0., 0.]])
    target = np.array([.06])
    base = AllocationSettings(effort_m_s=.001, change_m_s=.000001)
    gentle = BoundedRollingAllocator(base).solve(matrix, target, 1.)
    costly = BoundedRollingAllocator(replace(base, effort_m_s=.1)).solve(matrix, target, 1.)
    assert 0 < costly[0] < gentle[0]
    assert np.linalg.norm(matrix@costly-target) > np.linalg.norm(matrix@gentle-target)

    # With no contact objective, equal effort/change penalties have the known
    # optimum previous/2; increasing change cost favors retaining that reference.
    initial = np.array([1., -.5, .2, .03, -.02])
    ordinary = BoundedRollingAllocator(AllocationSettings(effort_m_s=.01, change_m_s=.01))
    reluctant = BoundedRollingAllocator(AllocationSettings(effort_m_s=.01, change_m_s=.02))
    ordinary.previous = initial.copy()
    reluctant.previous = initial.copy()
    np.testing.assert_allclose(ordinary.solve([], [], 1.), initial*.5, atol=1e-12)
    np.testing.assert_allclose(reluctant.solve([], [], 1.), initial*.8, atol=1e-12)


def test_normalized_contact_splitting_does_not_change_the_solution():
    matrix = np.array([[.12, .02, 0., 1., 0.], [0., .04, .07, 0., 1.]])
    target = np.array([.03, -.02])
    original = BoundedRollingAllocator().solve(matrix, target, 1.)
    # Two identical contact records sharing the same total load must not
    # increase tracking weight relative to the effort/change penalties.
    split = BoundedRollingAllocator().solve(np.vstack((matrix, matrix))/np.sqrt(2.),
                                            np.tile(target, 2)/np.sqrt(2.), 1.)
    np.testing.assert_allclose(split, original, rtol=1e-8, atol=1e-10)


def test_reset_clears_reference_history_and_returned_arrays_do_not_alias_state():
    allocator = BoundedRollingAllocator()
    matrix, target = np.eye(5), np.ones(5)
    first = allocator.solve(matrix, target, .005)
    expected = first.copy()
    first[:] = 100.
    np.testing.assert_array_equal(allocator.previous, expected)
    np.testing.assert_array_equal(allocator.diagnostics['allocation_drive_reference'], expected)
    allocator.reset()
    np.testing.assert_array_equal(allocator.previous, 0.)
    assert allocator.diagnostics == {}
    np.testing.assert_array_equal(allocator.solve(matrix, target, .005), expected)


@pytest.mark.parametrize('options', [
    {'speed_limits': (1.,)*4}, {'speed_limits': (1., 1., 1., 0., .3)},
    {'speed_limits': (1., 1., 1., np.inf, .3)},
    {'acceleration_limits': (1., 1., -1., 1., 1.)},
    {'acceleration_limits': (1., 1., np.nan, 1., 1.)},
    {'effort_m_s': 0.}, {'effort_m_s': np.nan}, {'change_m_s': -.1}, {'change_m_s': np.inf},
    {'weak_effort_m_s': .0001}, {'weak_effort_m_s': np.nan},
    {'authority_m_s': 0.}, {'authority_m_s': np.inf},
])
def test_invalid_settings_are_rejected(options):
    with pytest.raises(ValueError):
        AllocationSettings(**options)


@pytest.mark.parametrize('matrix,target,dt', [
    (np.zeros((5, 2)), np.zeros(2), .005),
    (np.zeros(5), np.zeros(1), .005),
    (np.eye(5), np.zeros(4), .005),
    (np.full((1, 5), np.nan), np.zeros(1), .005),
    (np.zeros((1, 5)), np.array([np.inf]), .005),
    (np.eye(5), np.zeros(5), 0.),
    (np.eye(5), np.zeros(5), -.01),
    (np.eye(5), np.zeros(5), np.nan),
    (np.eye(5), np.zeros(5), np.inf),
])
def test_invalid_solve_inputs_do_not_advance_reference_state(matrix, target, dt):
    allocator = BoundedRollingAllocator()
    previous = allocator.solve(np.eye(5), np.ones(5), .005)
    with pytest.raises(ValueError):
        allocator.solve(matrix, target, dt)
    np.testing.assert_array_equal(allocator.previous, previous)


@pytest.fixture(scope='module')
def allocation_workspace(tmp_path_factory):
    from spherical_wheel.model import REPO_ROOT
    from spherical_wheel.offroad_experiments import Workspace

    seed = REPO_ROOT/'generated'/'offroad'/'flat'/'control_schedule.npz'
    return Workspace(directory=tmp_path_factory.mktemp('bounded_allocation'),
                     seed_cache=seed if seed.is_file() else None)


def test_controller_unloaded_stop_restart_preserves_latest_bounded_references(
        allocation_workspace, monkeypatch):
    import mujoco
    import spherical_wheel.offroad_control as control

    mirror = allocation_workspace.mirror()
    controller = control.OffroadController(
        mirror, allocation_workspace.design,
        control.OffroadParams(mode='hardware', rolling_allocation='bounded'), .30,
        record_diagnostics=True)
    # Supply an estimator fixture directly. No solved contact is used anywhere
    # in these command calls; static measured state isolates reference history.
    records = controller.locator.records(mirror.data, np.array([0., 0., controller.weight]), np.zeros(3))
    assert records
    loaded = True
    monkeypatch.setattr(controller.locator, 'records', lambda *args: records if loaded else [])

    def forbidden(*args, **kwargs):
        raise AssertionError('Bounded hardware control attempted to read simulator contacts')

    monkeypatch.setattr(control, 'measure_contact', forbidden)
    monkeypatch.setattr(control, 'records_from_mujoco', forbidden)
    monkeypatch.setattr(mujoco, 'mj_contactForce', forbidden)
    previous = np.zeros(5)
    acceleration = np.asarray(controller.params.rolling_acceleration_limits)

    def command(velocity):
        nonlocal previous
        mirror.data.time += mirror.dt
        _, details = controller.command(mirror.data.qpos[:2], np.asarray(velocity), np.zeros(2))
        actual_reference = np.r_[details['reference_qvel'][6:9], details['rolling_feedforward'][3:]]
        assert np.all(abs(actual_reference-previous) <= mirror.dt*acceleration+1e-12)
        np.testing.assert_array_equal(actual_reference, controller.rolling_allocator.previous)
        np.testing.assert_array_equal(actual_reference, details['allocation_drive_reference'])
        previous = actual_reference.copy()
        return details

    for _ in range(20):
        command([.5, 0.])
    last_loaded = previous.copy()
    assert np.linalg.norm(last_loaded) > 1.
    loaded = False
    for _ in range(30):
        command([0., 0.])
    stopped = previous.copy()
    assert np.linalg.norm(stopped) < 1e-6
    # Resuming without renewed support must not restore the older, much faster
    # last-loaded reference or present its stale contact residual as current.
    details = command([.5, 0.])
    np.testing.assert_array_equal(previous, stopped)
    assert np.isnan(details['allocation_residual_rms_m_s'])
    assert np.isnan(details['allocation_velocity_error_m_s']).all()


def test_nonhardware_bounded_controller_uses_external_contact_provider(
        allocation_workspace, monkeypatch):
    import spherical_wheel.offroad_control as control
    from spherical_wheel.contact_sensing import SupportLocator
    from spherical_wheel.terrain import Terrain

    plant = allocation_workspace.plant(Terrain(name='flat'))
    controller = control.OffroadController(
        plant, allocation_workspace.design,
        control.OffroadParams(mode='terrain_aware', rolling_allocation='bounded', synchronize=False), .30)
    locator = SupportLocator(plant.model, plant.fork_id, controller.radius,
                             controller.nominal_friction, .05*controller.weight)
    records = locator.records(plant.data, np.array([0., 0., controller.weight]), np.zeros(3))
    assert records
    reads = 0

    def provider():
        nonlocal reads
        reads += 1
        return records

    def forbidden(*args, **kwargs):
        raise AssertionError('External contacts were bypassed in favor of the MuJoCo mirror')

    # GazeboPlant exposes this same protocol; a synthetic provider makes the
    # dispatch test independent of installing or running a second physics engine.
    monkeypatch.setattr(plant, 'contact_records', provider, raising=False)
    monkeypatch.setattr(control, 'records_from_mujoco', forbidden)
    controller.command(plant.data.qpos[:2], np.array([.1, 0.]), np.zeros(2))
    assert reads >= 2  # measurement and bounded reference construction
    assert np.linalg.norm(controller.rolling_allocator.previous) > .01


def test_adaptive_effort_changes_continuously_as_rolling_directions_separate():
    settings = AllocationSettings(effort_m_s=.001, weak_effort_m_s=.005, authority_m_s=.02)
    effort, authority = [], []
    # Two equally strong drives rotate from parallel to perpendicular. There
    # is no binary geometry mode switch or threshold at which the cost jumps.
    for angle in np.linspace(0., np.pi/2, 101):
        matrix = np.array([[.1, .1*np.cos(angle), 0., 0., 0.],
                           [0., .1*np.sin(angle), 0., 0., 0.]])
        allocator = BoundedRollingAllocator(settings)
        allocator.solve(matrix, np.array([0., .05]), 1.)
        effort.append(allocator.diagnostics['allocation_effort_weight_m_s'])
        authority.append(allocator.diagnostics['allocation_authority_m_s'])
    assert authority[0] == pytest.approx(0.)
    assert authority[-1] == pytest.approx(.5)
    assert np.all(np.diff(authority) > 0)
    assert np.all(np.diff(effort) < 0)
    assert effort[0] == pytest.approx(settings.weak_effort_m_s)
    assert settings.effort_m_s < effort[-1] < settings.effort_m_s+1e-5

    # At the declared authority scale, exactly half of the cost increase is
    # active. Arbitrarily close geometries on either side remain close in cost.
    near = []
    for weak_direction in settings.authority_m_s*np.array([1.-1e-6, 1., 1.+1e-6]):
        authority_matrix = np.zeros((2, 5))
        authority_matrix[0, 0] = .5/allocator.scale[0]
        authority_matrix[1, 1] = weak_direction/allocator.scale[1]
        allocator = BoundedRollingAllocator(settings)
        allocator.solve(matrix, np.array([0., .05]), 1., authority_matrix=authority_matrix)
        near.append(allocator.diagnostics['allocation_effort_weight_m_s'])
    assert near[1] == pytest.approx(.003)
    assert near[0] > near[1] > near[2]
    assert near[0]-near[2] < 1e-8


def test_adaptive_effort_and_solution_are_invariant_to_horizontal_frame_rotation():
    settings = AllocationSettings(weak_effort_m_s=.005)
    contacts = np.array([[[.12, .02, 0., 1., 0.], [0., .03, .07, 0., 1.]],
                         [[.10, 0., .04, 0., 1.], [0., .08, .01, 1., 0.]]])
    shares = np.array([.65, .35])
    velocity = np.array([.08, -.03])
    angle = 1.234
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    previous = np.array([.2, -.1, .3, .01, -.02])
    results = []
    for transform in (np.eye(2), rotation):
        rows = np.einsum('ij,njk->nik', transform, contacts)
        weighted = (np.sqrt(shares)[:, None, None]*rows).reshape(-1, 5)
        target = (np.sqrt(shares)[:, None]*(transform@velocity)[None, :]).ravel()
        mean = np.sum(shares[:, None, None]*rows, axis=0)
        allocator = BoundedRollingAllocator(settings)
        allocator.previous = previous.copy()
        drive = allocator.solve(weighted, target, .005, authority_matrix=mean)
        results.append((drive, allocator.diagnostics))
    np.testing.assert_allclose(results[0][0], results[1][0], atol=1e-10, rtol=1e-8)
    for key in ('allocation_authority_m_s', 'allocation_effort_blend',
                'allocation_effort_weight_m_s', 'allocation_residual_rms_m_s'):
        assert results[0][1][key] == pytest.approx(results[1][1][key], abs=1e-12)


def test_disabling_adaptive_effort_preserves_fixed_cost_for_any_authority():
    matrix = np.array([[.12, .03, 0., 0., 0.], [0., .02, .04, 0., 0.]])
    target = np.array([.06, -.02])
    settings = AllocationSettings(effort_m_s=.003)
    drives = []
    for authority in (np.zeros((2, 5)), matrix):
        allocator = BoundedRollingAllocator(settings)
        drives.append(allocator.solve(matrix, target, 1., authority_matrix=authority))
        assert allocator.diagnostics['allocation_effort_weight_m_s'] == .003
        assert allocator.diagnostics['allocation_effort_blend'] == 0.
    np.testing.assert_array_equal(drives[0], drives[1])


def test_record_helper_preserves_adaptive_solution_when_a_contact_is_split(allocation_workspace):
    import mujoco
    from spherical_wheel.contact_sensing import SupportLocator
    from spherical_wheel.experiments import rolling_feedforward_records

    mirror = allocation_workspace.mirror()
    qpos = mirror.data.qpos.copy()
    qpos[3:7] = [1., 0., 0., 0.]
    qpos[7:] = 0.  # flat support on both equatorial rims
    mirror.set_state(qpos, np.zeros(9), 0., np.zeros(5))
    radius = mirror.config['geometry']['sphere_radius_m']
    weight = float(mirror.model.body_subtreemass[mirror.fork_id])*9.81
    locator = SupportLocator(mirror.model, mirror.fork_id, radius, .7, .05*weight)
    records = locator.records(mirror.data, np.array([0., 0., weight]), np.zeros(3))
    assert len(records) == 2
    split = [dict(records[0], normal_force=.2*records[0]['normal_force']),
             dict(records[0], normal_force=.8*records[0]['normal_force']), records[1]]
    velocity = np.array([.04, -.02])
    settings = AllocationSettings(weak_effort_m_s=.005)
    results = []
    for contacts in (records, split):
        allocator = BoundedRollingAllocator(settings)
        references, feedforward = rolling_feedforward_records(mirror, contacts, velocity, allocator=allocator)
        results.append((references, feedforward, allocator.diagnostics))
    np.testing.assert_allclose(results[0][0], results[1][0], atol=1e-10)
    np.testing.assert_allclose(results[0][1], results[1][1], atol=1e-10)
    for key in ('allocation_authority_m_s', 'allocation_effort_blend',
                'allocation_effort_weight_m_s', 'allocation_residual_rms_m_s'):
        assert results[0][2][key] == pytest.approx(results[1][2][key], abs=1e-12)

    # Independently reconstruct the load-weighted mean Jacobian: the helper
    # must use the mean for authority, not an unnormalized stack of contacts.
    mean = np.zeros((2, 5))
    total = sum(record['normal_force'] for record in records)
    for record in records:
        jacobian = np.zeros((3, mirror.model.nv))
        mujoco.mj_jac(mirror.model, mirror.data, jacobian, None, record['pos'], record['body'])
        mean[:, :3] += record['normal_force']/total*jacobian[:2, 6:9]
    expected_authority = np.linalg.svd(mean*BoundedRollingAllocator.scale, compute_uv=False)[1]
    assert results[0][2]['allocation_authority_m_s'] == pytest.approx(expected_authority)


@pytest.mark.parametrize('authority', [np.zeros(5), np.zeros((5, 2)), np.full((2, 5), np.nan)])
def test_invalid_authority_does_not_advance_the_reference(authority):
    allocator = BoundedRollingAllocator(AllocationSettings(weak_effort_m_s=.005))
    previous = allocator.solve(np.eye(5), np.ones(5), .005)
    with pytest.raises(ValueError, match='Authority matrix'):
        allocator.solve(np.eye(5), np.ones(5), .005, authority_matrix=authority)
    np.testing.assert_array_equal(allocator.previous, previous)


@pytest.mark.parametrize('adaptive', [False, True])
def test_bounded_rollout_logs_geometry_cost_and_preserves_reference_bounds(allocation_workspace, adaptive):
    from spherical_wheel.offroad_control import OffroadParams
    from spherical_wheel.offroad_experiments import ReferencePath, run_offroad
    from spherical_wheel.terrain import Terrain

    params = OffroadParams(mode='hardware', rolling_allocation='bounded', rolling_adaptive_effort=adaptive)
    path = ReferencePath(.1, 0., 2., start_s=.01, ramp_s=.3)
    log, _ = run_offroad(allocation_workspace, Terrain(name='flat'), path, params,
                         duration_s=.1, record_diagnostics=True)
    drive = np.column_stack((log['reference_qvel'][:, 6:9], log['rolling_feedforward'][:, 3:]))
    np.testing.assert_array_equal(log['allocation_drive_reference'], drive)
    assert np.all(abs(drive) <= np.asarray(params.rolling_speed_limits)+1e-12)
    assert np.all(abs(np.diff(drive, axis=0)) <=
                  np.diff(log['time_s'])[:, None]*np.asarray(params.rolling_acceleration_limits)+1e-12)
    assert np.isfinite(log['allocation_authority_m_s']).all()
    assert np.isfinite(log['allocation_effort_weight_m_s']).all()
    if adaptive:
        blend = log['allocation_effort_blend']
        assert np.all((blend > 0.) & (blend <= 1.))
        np.testing.assert_allclose(log['allocation_effort_weight_m_s'],
                                   params.rolling_effort_m_s+blend*(params.rolling_weak_effort_m_s
                                                                 -params.rolling_effort_m_s))
    else:
        np.testing.assert_array_equal(log['allocation_effort_blend'], 0.)
        np.testing.assert_array_equal(log['allocation_effort_weight_m_s'], params.rolling_effort_m_s)
    components = sum(log[key] for key in (
        'equilibrium_command', 'rolling_feedforward', 'support_command', 'lqr_command',
        'transition_command_delta', 'heading_command', 'fade_command_delta'))
    np.testing.assert_allclose(components, log['command'], atol=1e-12)
