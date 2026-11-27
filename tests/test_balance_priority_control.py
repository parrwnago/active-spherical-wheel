"""Native balance preservation with amplitude-only feasibility and soft change."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

import spherical_wheel.balance_priority_control as module
from spherical_wheel.balance_priority_control import BalancePriorityAllocator
from spherical_wheel.handoff_control import BoundedHandoffAllocator, HandoffSettings

CAPS = np.array([1.5, .6, .6, .3, .3])
SETTINGS = HandoffSettings(yaw_weight=0., momentum_weight=0.)


def arguments(G=None, **overrides):
    result = dict(native_requested=np.zeros(5), input_bounds=CAPS,
        previous_applied=np.zeros(5), dt_s=.005, qvel=np.zeros(9),
        response=np.zeros((9, 5)) if G is None else G,
        fork_rotation_world=np.eye(3), shell_axis_world=np.array([0., 0., 1.]),
        shell_axial_inertia=np.array([.0013, .0013]),
        support_loads_N=np.array([0., 18., 0., 0.]), reference_shell_rates=np.zeros(2))
    result.update(overrides)
    return result


def solve(G=None, *, settings=SETTINGS, **overrides):
    return BalancePriorityAllocator(settings).solve(**arguments(G, **overrides))


def test_late_reversal_preserves_native_balance_despite_previous_positive_shaft():
    G = np.zeros((9, 5)); G[3, 0] = 1.
    native = np.array([-1.5, 0., 0., 0., 0.])
    previous = np.array([1.1975, 0., 0., 0., 0.])
    args = arguments(G, native_requested=native, previous_applied=previous)
    old = BoundedHandoffAllocator(SETTINGS).solve(**args)
    new = BalancePriorityAllocator(SETTINGS).solve(**args)
    assert old.command[0] >= .8975-1e-12
    assert new.command[0] == pytest.approx(-1.5, abs=1e-12)
    np.testing.assert_allclose(G[3:5]@(new.command-native), 0., atol=1e-10)
    assert new.diagnostics['diagnostic_rate_threshold_exceeded'][0]
    assert not new.diagnostics['hard_slew_enabled']
    np.testing.assert_array_equal(new.diagnostics['lower_command'], -CAPS)


def test_rank_zero_matches_closed_form_of_same_secondary_cost():
    native = CAPS*np.array([.8, -.3, .6, .9, -.5])
    previous = CAPS*np.array([-.4, .8, -.6, -.7, .9])
    ready = np.array([0., .2, .8, 1., .4])
    result = solve(native_requested=native, previous_applied=previous, readiness=ready)
    w = SETTINGS.native_weight*(1+SETTINGS.readiness_multiplier*ready)
    expected = CAPS*(w*(native/CAPS)+SETTINGS.change_weight*(previous/CAPS))/(w+SETTINGS.change_weight+SETTINGS.effort_weight)
    np.testing.assert_allclose(result.command, expected, atol=1e-12)
    assert result.diagnostics['balance_rank'] == 0


@pytest.mark.parametrize('rank', [1, 2])
def test_rank_one_and_two_preserve_the_original_physical_response(rank):
    G = np.zeros((9, 5)); G[3, :3] = [1., 2., 3.]
    G[4] = -4.*G[3] if rank == 1 else [2., -1., .3, .2, -.1]
    native = CAPS*np.array([.5, -.3, .1, .2, -.4])
    result = solve(G, native_requested=native, previous_applied=-native)
    assert result.diagnostics['balance_rank'] == rank
    np.testing.assert_allclose(G[3:5]@(result.command-native), 0., atol=1e-9)
    assert result.diagnostics['secondary_cost'] <= result.diagnostics['primary_point_secondary_cost']+1e-8


def test_secondary_interior_optimum_matches_independent_equality_kkt():
    G = np.zeros((9, 5)); G[:2] = [[.1, .2, 0., .3, -.1], [.2, 0., -.1, .1, .2]]
    G[3] = [.4, -.2, .3, .1, -.2]
    native = CAPS*np.array([.2, -.1, .3, .1, -.2])
    previous = CAPS*np.array([-.1, .2, -.2, .2, .1])
    ready = np.array([0., .3, .8, .5, .2])
    travel = G[:2]*CAPS/SETTINGS.travel_scale_m_s
    weights = SETTINGS.native_weight*(1+SETTINGS.readiness_multiplier*ready)
    H = SETTINGS.travel_weight*travel.T@travel+np.diag(weights+SETTINGS.change_weight+SETTINGS.effort_weight)
    rhs = SETTINGS.travel_weight*travel.T@travel@(native/CAPS)+weights*(native/CAPS)+SETTINGS.change_weight*(previous/CAPS)
    E = (G[3]*CAPS)[None, :]
    kkt = np.block([[H, E.T], [E, np.zeros((1, 1))]])
    expected = np.linalg.solve(kkt, np.r_[rhs, E@(native/CAPS)])[:5]*CAPS
    result = solve(G, native_requested=native, previous_applied=previous, readiness=ready)
    np.testing.assert_allclose(result.command, expected, atol=2e-7)


def test_smoothing_remains_active_only_in_available_balance_nullspace():
    G = np.zeros((9, 5)); G[3, 0] = 1.
    native = np.array([.9, .4, -.3, .2, -.1])
    previous = -native
    smooth = solve(G, native_requested=native, previous_applied=previous)
    no_change = solve(G, settings=replace(SETTINGS, change_weight=0.), native_requested=native, previous_applied=previous)
    assert smooth.command[0] == pytest.approx(native[0], abs=1e-10)
    assert abs(smooth.command[1]-previous[1]) < abs(no_change.command[1]-previous[1])


def test_actual_dt_changes_diagnostics_not_the_cap_normalized_soft_objective():
    G = np.zeros((9, 5)); G[3, 0] = 1.
    native = np.array([.5, .2, 0., 0., 0.])
    a = solve(G, native_requested=native, dt_s=.005)
    b = solve(G, native_requested=native, dt_s=.02)
    np.testing.assert_allclose(a.command, b.command, atol=1e-12)
    assert a.diagnostics['diagnostic_rate_threshold_exceeded'][0]
    assert not b.diagnostics['diagnostic_rate_threshold_exceeded'][0]
    np.testing.assert_allclose(a.diagnostics['command_change_per_s'], 4*b.diagnostics['command_change_per_s'])


def test_input_unit_changes_preserve_the_physical_solution():
    rng = np.random.default_rng(5801)
    G = rng.normal(size=(9, 5))
    native, previous = CAPS*rng.uniform(-.5, .5, (2, 5))
    scale = np.array([1000., .02, 10., .001, 2.])
    a = solve(G, native_requested=native, previous_applied=previous)
    b = solve(G/scale, settings=replace(SETTINGS, slew_limits=np.asarray(SETTINGS.slew_limits)*scale),
              native_requested=native*scale, previous_applied=previous*scale, input_bounds=CAPS*scale)
    np.testing.assert_allclose(a.command, b.command/scale, atol=1e-8)
    assert a.diagnostics['balance_rank'] == b.diagnostics['balance_rank'] == 2


def test_balance_rank_is_not_lost_when_a_constraint_row_has_small_units():
    G = np.zeros((9, 5)); G[3, 0] = 1e-18; G[4, 1] = 2.
    native = np.array([.7, -.3, .1, -.2, .2])
    result = solve(G, native_requested=native, previous_applied=-native)
    assert result.diagnostics['balance_rank'] == 2
    np.testing.assert_allclose(result.command[:2], native[:2], atol=1e-9)


def test_previous_input_is_supplied_and_no_allocator_history_is_owned():
    allocator = BalancePriorityAllocator()
    base = arguments(native_requested=CAPS*.8)
    a = allocator.solve(**base)
    base['previous_applied'] = -CAPS
    b = allocator.solve(**base)
    assert np.all(b.command < a.command)
    repeated = allocator.solve(**arguments(native_requested=CAPS*.8))
    np.testing.assert_array_equal(a.command, repeated.command)


def test_yaw_momentum_targets_and_qvel_cannot_add_an_auxiliary_cost():
    G = np.zeros((9, 5)); G[5, 1] = 10.; G[7, 1] = 20.
    a = solve(G, native_requested=CAPS*.3)
    b = solve(G, native_requested=CAPS*.3, qvel=np.arange(9)*10.,
        preparation_shell_rates=np.array([-20., 20.]), reference_shell_rates=np.array([5., -5.]))
    np.testing.assert_array_equal(a.command, b.command)
    assert not b.diagnostics['auxiliary_damping_enabled']


@pytest.mark.parametrize('rank', [0, 1])
@pytest.mark.parametrize('failure', ['exception', 'unsuccessful', 'false_success'])
def test_solver_failure_or_unverified_output_falls_back_to_exact_native(monkeypatch, rank, failure):
    G = np.zeros((9, 5))
    if rank:
        G[3, 0] = 1.
    native = np.array([1.7, -.2, .1, -.1, .2])
    def broken(*args, **kwargs):
        if failure == 'exception':
            raise RuntimeError('injected numerical solver failure')
        return SimpleNamespace(success=failure == 'false_success', x=np.full(5, 10.), message=failure)
    monkeypatch.setattr(module, 'minimize' if rank else 'lsq_linear', broken)
    result = solve(G, native_requested=native)
    np.testing.assert_array_equal(result.command, np.clip(native, -CAPS, CAPS))
    assert result.diagnostics['native_fallback_used']
    assert result.diagnostics['physical_balance_residual_max'] == 0.


def test_random_maps_keep_total_caps_and_verified_native_response():
    rng = np.random.default_rng(5802)
    for _ in range(20):
        G = rng.normal(size=(9, 5))
        native = rng.normal(size=5)*10.
        result = solve(G, native_requested=native, previous_applied=CAPS*rng.uniform(-1., 1., 5))
        assert np.all(abs(result.command) <= CAPS)
        np.testing.assert_allclose(G[3:5]@(result.command-np.clip(native, -CAPS, CAPS)), 0., atol=1e-8)
        assert result.diagnostics['secondary_cost'] <= result.diagnostics['primary_point_secondary_cost']+1e-7


def test_inputs_and_output_do_not_alias():
    G = np.eye(9, 5); native = CAPS*.3; previous = -CAPS*.2
    originals = [value.copy() for value in (G, native, previous, CAPS)]
    result = solve(G, native_requested=native, previous_applied=previous)
    for value, original in zip((G, native, previous, CAPS), originals):
        np.testing.assert_array_equal(value, original)
    with pytest.raises(ValueError):
        result.command[0] = 0.


@pytest.mark.parametrize('settings', [HandoffSettings(), replace(SETTINGS, yaw_weight=1e-30), replace(SETTINGS, momentum_weight=.1)])
def test_nonzero_yaw_or_momentum_policy_is_rejected(settings):
    with pytest.raises(ValueError, match='zero yaw and momentum'):
        BalancePriorityAllocator(settings)


@pytest.mark.parametrize('overrides', [
    {'dt_s': 0.}, {'dt_s': np.nan}, {'response': np.full((9, 5), np.nan)},
    {'previous_applied': np.ones(5)*100.}, {'support_loads_N': -np.ones(4)},
    {'shell_axis_world': np.zeros(3)}, {'fork_rotation_world': np.ones((3, 3))},
    {'readiness': np.ones(5)*1.1}, {'shell_axial_inertia': np.zeros(2)},
])
def test_invalid_problem_is_rejected(overrides):
    with pytest.raises(ValueError):
        solve(**overrides)
