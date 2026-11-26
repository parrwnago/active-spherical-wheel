"""Same-contact rolling feasibility and the feedback references it governs.

Synthetic shell-B contacts isolate the original shaft/hemisphere geometry.
These tests do not establish dynamic stability or eventual path completion.
"""
import mujoco
import numpy as np
import pytest

from spherical_wheel.contact_sensing import SupportLocator, measure_records
from spherical_wheel.experiments import rolling_feedforward_records
from spherical_wheel.feasible_reference import (
    FeasibleMotionReference, FeasibleReferenceSettings, balance_travel_priority,
)
from spherical_wheel.mirror import StateMirror
from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.rolling_allocation import (
    AllocationSettings, BoundedRollingAllocator, FeasibleRollingAllocator,
)


@pytest.fixture(scope='module')
def mirror(tmp_path_factory):
    return StateMirror(load_config(), tmp_path_factory.mktemp('feasible_geometry'))


def _shell_b(plant, phase):
    radius = plant.config['geometry']['sphere_radius_m']
    roll = np.radians(-20.)
    qpos = np.r_[0., 0., radius, np.cos(roll/2), np.sin(roll/2), 0., 0., phase, 0., 0.]
    plant.set_state(qpos, np.zeros(9), 0., np.zeros(5))
    weight = float(plant.model.body_subtreemass[plant.fork_id])*9.81
    locator = SupportLocator(plant.model, plant.fork_id, radius, .7, .05*weight)
    records = locator.records(plant.data, np.array([0., 0., weight]), np.zeros(3))
    assert len(records) == 1 and records[0]['name'] == 'shell_B_smooth'
    patch = next(item for item in locator.patches if item['name'] == records[0]['name'])
    axis = plant.data.xmat[records[0]['body']].reshape(3, 3)[:, 0]
    axial = patch['sign']*float(axis@(records[0]['pos']-plant.data.xpos[plant.fork_id]))
    assert min(axial-patch['lower'], patch['upper']-axial) > .0028
    return records


def _matrix(plant, record):
    jacobian = np.zeros((3, plant.model.nv))
    mujoco.mj_jac(plant.model, plant.data, jacobian, None, record['pos'], record['body'])
    result = np.zeros((2, 5))
    result[:, :3] = jacobian[:2, 6:9]
    return result


@pytest.mark.parametrize('phase', [-.6, -np.pi/2+.001, -np.pi/2])
def test_single_contact_body_and_drive_references_satisfy_same_rolling_constraint(mirror, phase):
    records = _shell_b(mirror, phase)
    allocator = FeasibleRollingAllocator()
    qvel, command = rolling_feedforward_records(mirror, records, np.array([.06, .04]), allocator=allocator)
    drive = np.r_[qvel[6:9], command[3:]]
    np.testing.assert_array_equal(drive, allocator.diagnostics['allocation_drive_reference'])
    np.testing.assert_array_equal(qvel[:2], allocator.diagnostics['allocation_accepted_velocity_m_s'])
    np.testing.assert_allclose(_matrix(mirror, records[0])@drive+qvel[:2], 0., atol=1e-12)
    assert allocator.diagnostics['allocation_accepted_velocity_valid']
    assert allocator.diagnostics['allocation_reference_residual_rms_m_s'] < 1e-12


def test_exact_rank_loss_retains_useful_direction_and_reports_rejected_motion(mirror):
    records = _shell_b(mirror, -np.pi/2)
    allocator = FeasibleRollingAllocator()
    requested = np.array([.06, .04])
    qvel, command = rolling_feedforward_records(mirror, records, requested, allocator=allocator)
    np.testing.assert_allclose(qvel[:2], [.06, 0.], atol=1e-6)
    assert np.max(abs(np.r_[qvel[6:9], command[3:]])) < 1.
    assert allocator.diagnostics['allocation_residual_rms_m_s'] == pytest.approx(.04, abs=1e-6)
    response = allocator.diagnostics['allocation_velocity_response']
    assert response[0, 0] > .999 and abs(response[1, 1]) < 1e-12
    np.testing.assert_allclose(response, response.T, atol=1e-12)
    assert np.linalg.eigvalsh(response).min() >= -1e-12
    assert np.linalg.eigvalsh(response).max() <= 1.+1e-12
    modal = allocator.diagnostics['allocation_modal_effort_m_s']
    assert modal[0] < .0011 and modal[1] == pytest.approx(.005)


def test_unavailable_direction_is_not_misreported_as_destination_progress(mirror):
    records = _shell_b(mirror, -np.pi/2)
    allocator = FeasibleRollingAllocator()
    requested = np.array([0., .06])
    for _ in range(20):
        qvel, _ = rolling_feedforward_records(mirror, records, requested, allocator=allocator)
        np.testing.assert_allclose(qvel[:2], 0., atol=1e-12)
        np.testing.assert_array_equal(allocator.diagnostics['allocation_requested_velocity_m_s'], requested)
        assert allocator.diagnostics['allocation_residual_rms_m_s'] == pytest.approx(.06)
    # Zero accepted velocity is consistent and finite, but cannot complete a
    # destination in this direction without a subsequent configuration change.
    assert allocator.diagnostics['allocation_reference_residual_rms_m_s'] < 1e-12


def test_speed_and_slew_bounds_survive_rank_loss_recovery_and_stopping(mirror):
    settings = AllocationSettings(speed_limits=(1., 1., 1., .05, .05),
                                  acceleration_limits=(5., 5., 5., .3, .3), weak_effort_m_s=.005)
    allocator = FeasibleRollingAllocator(settings)
    phases = np.r_[np.full(20, -.6), np.linspace(-.6, -np.pi/2, 30),
                   np.full(20, -np.pi/2), np.linspace(-np.pi/2, -.6, 30), np.full(100, -.6)]
    previous = np.zeros(5)
    for index, phase in enumerate(phases):
        records = _shell_b(mirror, phase)
        requested = np.array([.15, .12]) if index < 100 else np.zeros(2)
        qvel, command = rolling_feedforward_records(mirror, records, requested, allocator=allocator)
        drive = np.r_[qvel[6:9], command[3:]]
        assert np.all(abs(drive) <= np.asarray(settings.speed_limits)+1e-12)
        assert np.all(abs(drive-previous) <= mirror.dt*np.asarray(settings.acceleration_limits)+1e-12)
        np.testing.assert_allclose(_matrix(mirror, records[0])@drive+qvel[:2], 0., atol=1e-12)
        previous = drive.copy()
    np.testing.assert_allclose(previous, 0., atol=1e-8)


def test_contact_dropout_holds_drives_without_claiming_a_body_motion_constraint(mirror):
    records = _shell_b(mirror, -.6)
    allocator = FeasibleRollingAllocator()
    rolling_feedforward_records(mirror, records, np.array([.05, .03]), allocator=allocator)
    previous = allocator.previous.copy()
    held = allocator.hold()
    np.testing.assert_array_equal(held, previous)
    np.testing.assert_array_equal(allocator.previous, previous)
    np.testing.assert_array_equal(allocator.diagnostics['allocation_accepted_velocity_m_s'], 0.)
    np.testing.assert_array_equal(allocator.diagnostics['allocation_velocity_response'], 0.)
    assert not allocator.diagnostics['allocation_accepted_velocity_valid']
    assert np.isnan(allocator.diagnostics['allocation_reference_residual_rms_m_s'])


def test_directional_solution_is_invariant_to_horizontal_frame_rotation():
    matrix = np.array([[.112, 0., .041, 0., 0.], [0., 0., .0001, 0., 0.]])
    velocity = np.array([.04, .05])
    angle = .731
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    first, rotated = FeasibleRollingAllocator(), FeasibleRollingAllocator()
    drive = first.solve(matrix, -velocity, .005)
    other = rotated.solve(rotation@matrix, -rotation@velocity, .005)
    np.testing.assert_allclose(other, drive, atol=1e-10, rtol=1e-8)
    np.testing.assert_allclose(rotated.diagnostics['allocation_accepted_velocity_m_s'],
                               rotation@first.diagnostics['allocation_accepted_velocity_m_s'], atol=1e-12)
    np.testing.assert_allclose(rotated.diagnostics['allocation_velocity_response'],
                               rotation@first.diagnostics['allocation_velocity_response']@rotation.T, atol=1e-12)


def test_position_and_catchup_project_before_capping_the_useful_direction():
    reference = FeasibleMotionReference()
    nominal, measured = np.array([.02, 5.]), np.zeros(2)
    response = np.diag([1., 0.])
    requested = reference.request(nominal, [.01, .03], measured, response=response)
    np.testing.assert_allclose(requested, [.03, .03], atol=1e-12)
    result = reference.feedback(nominal, measured, [.3, .6], [.04, 0.], response)
    np.testing.assert_allclose(result['xy'], [.02, 0.], atol=1e-12)
    np.testing.assert_array_equal(result['velocity'], [.04, 0.])
    np.testing.assert_array_equal(result['acceleration'], [.3, 0.])
    np.testing.assert_allclose(result['relaxed_position_error_m'], [0., 5.], atol=1e-12)
    np.testing.assert_array_equal(nominal, [.02, 5.])


def test_original_destination_remains_available_for_catchup_when_authority_returns():
    reference = FeasibleMotionReference()
    goal, measured = np.array([0., .5]), np.zeros(2)
    np.testing.assert_array_equal(reference.request(goal, [0., 0.], measured, response=np.zeros((2, 2))), 0.)
    np.testing.assert_allclose(reference.request(goal, [0., 0.], measured, response=np.eye(2)), [0., .06])
    result = reference.feedback(goal, measured, [0., 0.], [0., .06], np.eye(2))
    np.testing.assert_allclose(result['xy'], [0., .05])
    np.testing.assert_allclose(result['relaxed_position_error_m'], [0., .45])
    np.testing.assert_array_equal(reference.request(goal, [0., 0.], goal, response=np.eye(2)), 0.)


def test_zero_catchup_gain_preserves_nominal_velocity_and_restores_position_target():
    reference = FeasibleMotionReference(FeasibleReferenceSettings(catchup_gain_s_inv=0.))
    goal, measured = np.array([0., .5]), np.zeros(2)
    nominal_velocity = np.array([.01, .02])
    unavailable, available = np.diag([1., 0.]), np.eye(2)
    for response in (unavailable, available):
        np.testing.assert_array_equal(
            reference.request(goal, nominal_velocity, measured, response=response), nominal_velocity)
        np.testing.assert_array_equal(reference.request(goal, [0., 0.], measured, response=response), 0.)
    blocked = reference.feedback(goal, measured, [0., 0.], [0., 0.], unavailable)
    restored = reference.feedback(goal, measured, [0., 0.], [0., 0.], available)
    np.testing.assert_array_equal(blocked['xy'], measured)
    np.testing.assert_allclose(blocked['relaxed_position_error_m'], goal-measured)
    np.testing.assert_allclose(restored['xy'], [0., .05])
    np.testing.assert_allclose(restored['relaxed_position_error_m'], [0., .45])
    np.testing.assert_array_equal(goal, [0., .5])


def test_invalid_support_freezes_feedback_position_velocity_and_travel_lean():
    reference = FeasibleMotionReference()
    result = reference.feedback([1., 2.], [.3, .4], [.2, .5], [.1, .2], np.eye(2), valid=False)
    np.testing.assert_array_equal(result['xy'], [.3, .4])
    np.testing.assert_array_equal(result['velocity'], 0.)
    np.testing.assert_array_equal(result['acceleration'], 0.)
    np.testing.assert_allclose(result['relaxed_position_error_m'], [.7, 1.6])


def test_balance_priority_is_bounded_monotone_and_smooth_across_its_scale():
    scale = np.radians(3.)
    errors = np.r_[np.linspace(0., 4.*scale, 101), 100.*scale, 1e8*scale]
    values = np.array([balance_travel_priority(error, 0., scale)[0] for error in errors])
    assert values[0] == 1.
    assert np.isfinite(values).all() and np.all((0. <= values) & (values <= 1.))
    assert np.all(np.diff(values) < 0.)
    assert balance_travel_priority(scale, 0., scale)[0] == pytest.approx(.5)
    # The scheduling scale is not a discontinuous threshold. Its one-sided
    # slopes converge, while the slope is flat at zero error.
    delta = scale*1e-5
    below, at, above = [balance_travel_priority(error, 0., scale)[0]
                        for error in (scale-delta, scale, scale+delta)]
    assert (at-below)/delta == pytest.approx((above-at)/delta, rel=5e-5)
    assert abs(balance_travel_priority(delta, 0., scale)[0]-1.) < 1e-12


def test_balance_priority_accounts_for_lean_rate_without_requiring_lean_error():
    scale, horizon, error = .05, .1, .02
    position = balance_travel_priority(error, 0., scale, horizon)
    rate = balance_travel_priority(0., error/horizon, scale, horizon)
    np.testing.assert_allclose(position, rate)
    combined = balance_travel_priority(error, error/horizon, scale, horizon)
    assert combined[0] < position[0]
    assert combined[1] == pytest.approx(2.*error)
    assert balance_travel_priority(0., 100., scale, horizon_s=0.) == (1., 0.)
    assert balance_travel_priority(error, 100., scale, horizon_s=0.) == position


@pytest.mark.parametrize('arguments', [
    (-.1, 0., .05, .1), (0., -.1, .05, .1),
    (0., 0., 0., .1), (0., 0., -.05, .1), (0., 0., .05, -.1),
    (np.nan, 0., .05, .1), (0., np.inf, .05, .1),
    (0., 0., np.nan, .1), (0., 0., .05, np.inf),
])
def test_balance_priority_rejects_invalid_inputs(arguments):
    with pytest.raises(ValueError):
        balance_travel_priority(*arguments)


@pytest.mark.parametrize('priority', [0., .25, 1.])
def test_travel_priority_scales_demand_position_and_lean_but_not_accepted_velocity(priority):
    reference = FeasibleMotionReference()
    nominal, measured = np.array([.1, 2.]), np.array([.01, .2])
    response, accepted = np.diag([.8, 0.]), np.array([.03, 0.])
    nominal_velocity, acceleration = np.array([.04, .02]), np.array([.2, .3])
    unscaled_request = reference.request(nominal, nominal_velocity, measured, response=response)
    request = reference.request(nominal, nominal_velocity, measured, response=response, priority=priority)
    np.testing.assert_allclose(request, priority*unscaled_request)
    unscaled = reference.feedback(nominal, measured, acceleration, accepted, response)
    result = reference.feedback(nominal, measured, acceleration, accepted, response, priority=priority)
    np.testing.assert_allclose(result['xy']-measured, priority*(unscaled['xy']-measured), atol=1e-15)
    np.testing.assert_allclose(result['acceleration'], priority*unscaled['acceleration'])
    np.testing.assert_array_equal(result['velocity'], accepted)
    np.testing.assert_array_equal(result['projection'], unscaled['projection'])
    np.testing.assert_allclose(result['relaxed_position_error_m'], nominal-result['xy'])
    assert np.linalg.norm(result['xy']-measured) <= priority*reference.settings.position_error_limit_m+1e-15
    # A lower priority does not mutate the nominal goal or consume its backlog.
    restored = reference.feedback(nominal, measured, acceleration, accepted, response, priority=1.)
    np.testing.assert_array_equal(restored['xy'], unscaled['xy'])


@pytest.mark.parametrize('priority', [-.1, 1.1, np.nan, np.inf])
def test_reference_rejects_invalid_travel_priority(priority):
    reference = FeasibleMotionReference()
    with pytest.raises(ValueError):
        reference.request([0., 0.], [0., 0.], [0., 0.], priority=priority)
    with pytest.raises(ValueError):
        reference.feedback([0., 0.], [0., 0.], [0., 0.], [0., 0.], np.eye(2), priority=priority)


@pytest.mark.parametrize('phase', [-.6, -np.pi/2])
def test_original_least_squares_reference_path_is_bit_identical(mirror, phase):
    records = _shell_b(mirror, phase)
    velocity = np.array([.06, .04])
    matrix = _matrix(mirror, records[0])
    # Exact pre-existing allocation expression, including normalization and
    # rank tolerance. The opt-in governor must not enter this code path.
    scale = np.array([5., 5., 5., .3, .3])
    expected = scale*np.linalg.lstsq(matrix*scale, -velocity, rcond=1e-7)[0]
    qvel, command = rolling_feedforward_records(mirror, records, velocity)
    np.testing.assert_array_equal(qvel, np.r_[velocity, 0., 0., 0., 0., expected[:3]])
    np.testing.assert_array_equal(command, np.r_[np.zeros(3), expected[3:]])


def test_existing_bounded_mode_retains_its_nominal_body_reference(mirror):
    records = _shell_b(mirror, -np.pi/2)
    velocity = np.array([0., .06])
    qvel, _ = rolling_feedforward_records(mirror, records, velocity, allocator=BoundedRollingAllocator())
    np.testing.assert_array_equal(qvel[:2], velocity)


@pytest.mark.parametrize('settings', [
    {'catchup_gain_s_inv': -.1}, {'catchup_gain_s_inv': np.inf},
    {'catchup_speed_m_s': -.1}, {'position_error_limit_m': np.nan},
])
def test_invalid_reference_settings_are_rejected(settings):
    with pytest.raises(ValueError):
        FeasibleReferenceSettings(**settings)


@pytest.fixture(scope='module')
def workspace(tmp_path_factory):
    from spherical_wheel.offroad_experiments import Workspace
    seed = REPO_ROOT/'generated'/'offroad'/'flat'/'control_schedule.npz'
    return Workspace(directory=tmp_path_factory.mktemp('feasible_controller'),
                     seed_cache=seed if seed.is_file() else None)


def test_controller_relaxes_missing_direction_without_changing_balance_gains(workspace, monkeypatch):
    import spherical_wheel.offroad_control as control
    plant = workspace.mirror()
    records = _shell_b(plant, -np.pi/2)
    candidate = control.OffroadController(plant, workspace.design,
        control.OffroadParams(mode='hardware', rolling_allocation='feasible'), .3, record_diagnostics=True)
    baseline = control.OffroadController(plant, workspace.design,
        control.OffroadParams(mode='hardware', rolling_allocation='bounded'), .3, record_diagnostics=True)
    monkeypatch.setattr(candidate.locator, 'records', lambda *args: records)
    monkeypatch.setattr(baseline.locator, 'records', lambda *args: records)

    def forbidden(*args, **kwargs):
        raise AssertionError('Feasible hardware mode attempted to read simulator contacts')

    monkeypatch.setattr(control, 'measure_contact', forbidden)
    monkeypatch.setattr(control, 'records_from_mujoco', forbidden)
    monkeypatch.setattr(mujoco, 'mj_contactForce', forbidden)
    plant.data.time += plant.dt
    goal, velocity = np.array([.02, .5]), np.array([.03, .05])
    _, actual = candidate.command(goal, velocity, np.zeros(2))
    _, old = baseline.command(goal, velocity, np.zeros(2))
    assert abs(actual['feedback_reference_xy'][1]-plant.data.qpos[1]) < 1e-12
    assert actual['feedback_reference_xy'][0] > .019
    assert abs(actual['feedback_reference_qvel'][1]) < 1e-12
    assert actual['reference_position_error_relaxed_m'][1] == pytest.approx(.5)
    assert abs(actual['error'][1]) < 1e-12
    assert abs(old['error'][1]) > .4
    np.testing.assert_array_equal(actual['feedback_gain'], old['feedback_gain'])
    np.testing.assert_array_equal(actual['error'][3:6], old['error'][3:6])
    np.testing.assert_array_equal(actual['error'][12:15], old['error'][12:15])
    np.testing.assert_array_equal(actual['reference_velocity_m_s'], velocity)


def test_controller_dropout_does_not_reuse_a_stale_nominal_or_catchup_request(workspace, monkeypatch):
    import spherical_wheel.offroad_control as control
    plant = workspace.mirror()
    records = _shell_b(plant, -.6)
    controller = control.OffroadController(plant, workspace.design,
        control.OffroadParams(mode='hardware', rolling_allocation='feasible'), .3, record_diagnostics=True)
    loaded = True
    monkeypatch.setattr(controller.locator, 'records', lambda *args: records if loaded else [])
    plant.data.time += plant.dt
    _, first = controller.command(np.array([.02, .03]), np.array([.04, .02]), np.zeros(2))
    loaded = False
    plant.data.time += plant.dt
    _, missing = controller.command(np.array([.5, .5]), np.zeros(2), np.zeros(2))
    np.testing.assert_array_equal(missing['allocation_drive_reference'], first['allocation_drive_reference'])
    np.testing.assert_array_equal(missing['feedback_reference_xy'], plant.data.qpos[:2])
    np.testing.assert_array_equal(missing['feedback_reference_qvel'][:2], 0.)
    np.testing.assert_array_equal(missing['allocation_requested_velocity_m_s'], 0.)
    np.testing.assert_array_equal(missing['reference_catchup_velocity_m_s'], 0.)
    assert not missing['allocation_accepted_velocity_valid']
    assert np.isnan(missing['allocation_reference_residual_rms_m_s'])


@pytest.mark.parametrize('angular_velocity', [(0., 0., 0.), (.2, -.1, 0.), (0., 0., .5)])
def test_balance_guard_preserves_balance_gains_and_uses_only_transverse_lean_rate(
        workspace, monkeypatch, angular_velocity):
    import spherical_wheel.offroad_control as control
    plant = workspace.mirror()
    records = _shell_b(plant, -.6)
    qvel = np.zeros(9)
    qvel[3:6] = angular_velocity
    plant.set_state(plant.data.qpos.copy(), qvel, plant.dt, np.zeros(5))
    guarded = control.OffroadController(plant, workspace.design,
        control.OffroadParams(mode='hardware', rolling_allocation='feasible', synchronize=False,
                              rolling_reference_balance_guard=True), .3, record_diagnostics=True)
    baseline = control.OffroadController(plant, workspace.design,
        control.OffroadParams(mode='hardware', rolling_allocation='feasible', synchronize=False),
        .3, record_diagnostics=True)
    for controller in (guarded, baseline):
        monkeypatch.setattr(controller.locator, 'records', lambda *args: records)
        # Isolate the guard with an explicit synthetic flat-support estimate.
        # The observer's static startup wrench can otherwise imply a normal
        # whose intended lean already matches this deliberately tilted pose.
        controller.estimator.reset([0., 0., 1.])
        monkeypatch.setattr(controller.observer, 'measurement', lambda *args: measure_records(plant, records))

    def forbidden(*args, **kwargs):
        raise AssertionError('Balance guard attempted to read simulator contacts')

    monkeypatch.setattr(control, 'measure_contact', forbidden)
    monkeypatch.setattr(control, 'records_from_mujoco', forbidden)
    monkeypatch.setattr(mujoco, 'mj_contactForce', forbidden)
    goal, velocity = np.array([.02, .03]), np.array([.04, .02])
    _, actual = guarded.command(goal, velocity, np.zeros(2))
    _, old = baseline.command(goal, velocity, np.zeros(2))
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, actual['reference_quat'])
    up = plant.data.xmat[plant.fork_id].reshape(3, 3)[:, 2]
    lean_error = np.arccos(np.clip(up@rotation.reshape(3, 3)[:, 2], -1., 1.))
    priority, metric = balance_travel_priority(lean_error, np.linalg.norm(qvel[3:5]), np.radians(3.))
    assert 0. < priority < 1.
    assert actual['reference_travel_priority'] == pytest.approx(priority)
    assert actual['reference_balance_metric_rad'] == pytest.approx(metric)
    assert old['reference_travel_priority'] == 1.
    np.testing.assert_allclose(actual['allocation_requested_velocity_m_s'],
                               priority*old['allocation_requested_velocity_m_s'])
    np.testing.assert_allclose(actual['feedback_reference_xy']-plant.data.qpos[:2],
                               priority*(old['feedback_reference_xy']-plant.data.qpos[:2]), atol=1e-15)
    np.testing.assert_array_equal(actual['feedback_reference_qvel'][:2],
                                  actual['allocation_accepted_velocity_m_s'])
    np.testing.assert_allclose(_matrix(plant, records[0])@actual['allocation_drive_reference']
                               +actual['feedback_reference_qvel'][:2], 0., atol=1e-12)
    for key in ('feedback_gain', 'reference_quat', 'support_command', 'reference_acceleration_m_s2'):
        np.testing.assert_array_equal(actual[key], old[key])
    np.testing.assert_array_equal(actual['error'][3:6], old['error'][3:6])
    np.testing.assert_array_equal(actual['error'][12:15], old['error'][12:15])


@pytest.mark.parametrize('mode', ['flat_baseline', 'terrain_aware', 'momentum_observer', 'hardware'])
def test_disabled_governor_settings_do_not_change_existing_controller_modes(workspace, monkeypatch, mode):
    import spherical_wheel.offroad_control as control
    plant = workspace.mirror()
    records = _shell_b(plant, -.6)
    monkeypatch.setattr(plant, 'contact_records', lambda: records, raising=False)
    monkeypatch.setattr(control, 'measure_contact', lambda current: measure_records(current, records))
    first = control.OffroadController(plant, workspace.design,
        control.OffroadParams(mode=mode, synchronize=False), .3)
    other = control.OffroadController(plant, workspace.design,
        control.OffroadParams(mode=mode, synchronize=False,
            rolling_reference_catchup_gain_s_inv=7., rolling_reference_catchup_speed_m_s=.3,
            rolling_reference_position_limit_m=.4, rolling_reference_balance_guard=True,
            rolling_reference_lean_scale_deg=1., rolling_reference_rate_horizon_s=.3), .3)
    assert first.motion_reference is None and other.motion_reference is None
    if first.locator is not None:
        monkeypatch.setattr(first.locator, 'records', lambda *args: records)
        monkeypatch.setattr(other.locator, 'records', lambda *args: records)
    for _ in range(5):
        plant.data.time += plant.dt
        command, details = first.command([.02, .03], [.04, .02], [0., 0.])
        unchanged, other_details = other.command([.02, .03], [.04, .02], [0., 0.])
        np.testing.assert_array_equal(command, unchanged)
        for key in ('requested', 'reference_qpos', 'reference_qvel', 'error'):
            np.testing.assert_array_equal(details[key], other_details[key])
        first.after_step()
        other.after_step()
