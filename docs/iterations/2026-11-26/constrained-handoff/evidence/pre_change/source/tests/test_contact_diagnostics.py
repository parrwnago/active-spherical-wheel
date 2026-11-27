"""Offline diagnostic semantics: measured motion, timing, and ambiguous mismatch."""
import numpy as np
import pytest

from spherical_wheel.contact_diagnostics import correction_events, diagnostic_signals


def _log(count=101, dt=.01):
    qpos = np.zeros((count, 10))
    qpos[:, 3] = 1.
    point = qpos[:, :3]+np.array([0., 0., -.12])
    normal = np.tile([0., 0., 1.], (count, 1))
    loads = np.tile([18., 0., 0., 0.], (count, 1))
    return {
        'time_s': np.arange(count)*dt, 'qpos': qpos,
        'reference_xy': np.zeros((count, 2)),
        'command': np.zeros((count, 5)), 'input': np.zeros((count, 5)),
        'input_bounds': np.tile([1.5, .6, .6, .3, .3], (count, 1)),
        'estimated_contact_point_m': point.copy(), 'contact_point_m': point.copy(),
        'estimated_support_loads_N': loads.copy(), 'support_loads_N': loads.copy(),
        'estimated_normal': normal.copy(), 'ground_normal': normal.copy(),
        'contact_velocity_residual_rms_m_s': np.zeros(count),
        'contact_velocity_residual_max_m_s': np.zeros(count),
        'contact_velocity_residual_m_s': np.zeros((count, 3)),
        'velocity_consistency_valid': np.ones(count, bool),
        'kinematic_inconsistency': np.zeros(count, bool),
        'measured_body_velocity_m_s': np.zeros((count, 3)),
        'predicted_body_velocity_m_s': np.zeros((count, 3)),
    }


def test_per_contact_rms_preserves_opposing_velocity_residuals():
    log = _log()
    # Two estimated contacts can have opposite tangential residual vectors:
    # their weighted vector mean is zero while neither contact is rolling.
    log['estimated_support_loads_N'][:] = [9., 9., 0., 0.]
    log['contact_velocity_residual_rms_m_s'][:] = .2
    log['contact_velocity_residual_max_m_s'][:] = .2
    log['kinematic_inconsistency'][:] = True
    signals = diagnostic_signals(log)
    np.testing.assert_array_equal(signals['rolling_mismatch_rms_m_s'], .2)
    assert signals['kinematic_inconsistency'].all()


def test_body_translation_is_not_reported_as_a_contact_geometry_jump():
    log = _log()
    shift = np.linspace(0., 1., len(log['time_s']))
    log['qpos'][:, 0] = shift
    log['estimated_contact_point_m'][:, 0] = shift
    signals = diagnostic_signals(log)
    np.testing.assert_allclose(signals['estimated_contact_step_m'], 0., atol=1e-15)
    # A change relative to the wheel centre must remain visible.
    log['estimated_contact_point_m'][50:, 1] += .01
    signals = diagnostic_signals(log)
    assert signals['estimated_contact_step_m'][50] == pytest.approx(.01)
    assert np.count_nonzero(signals['estimated_contact_step_m'] > 1e-12) == 1


def test_saturation_of_new_commands_is_separate_from_measured_interval():
    log = _log()
    log['command'][40, 3] = .4
    log['previous_actuator_saturated'] = np.zeros((101, 5), bool)
    log['previous_actuator_saturated'][41, 3] = True
    signals = diagnostic_signals(log)
    assert signals['belt_saturated'][40] and not signals['previous_belt_saturated'][40]
    assert signals['previous_belt_saturated'][41] and not signals['belt_saturated'][41]
    assert not signals['torque_saturated'].any()


def test_invalid_contact_estimates_do_not_become_rolling_mismatch_events():
    log = _log()
    log['contact_velocity_residual_rms_m_s'][50] = 100.
    log['velocity_consistency_valid'][50] = False
    signals = diagnostic_signals(log)
    assert np.isnan(signals['rolling_mismatch_rms_m_s'][50])
    assert np.isnan(signals['estimated_contact_step_m'][50:52]).all()
    assert correction_events(log, ignore_initial_s=0.) == []


def test_spaced_events_keep_observations_from_different_sources():
    log = _log(1001)
    log['qpos'][200, 0] = .2
    log['command'][500:510, 0] = .3
    log['contact_velocity_residual_rms_m_s'][800] = .1
    log['kinematic_inconsistency'][800] = True
    events = correction_events(log, count=3, separation_s=1., ignore_initial_s=0.)
    assert len(events) == 3
    assert [event['time_s'] for event in events] == pytest.approx([2., 5.1, 8.])
    assert events[0]['reasons'] == ['tracking_deviation']
    assert events[1]['reasons'] == ['command_change']
    assert events[2]['reasons'] == ['rolling_mismatch']
    # Reporting explicitly distinguishes measured-interval saturation from
    # the new request; a mismatch alone makes no confirmed-slip assertion.
    assert 'previous_belt_saturation_fraction' in events[2]['window']
    assert 'slip' not in events[2]


def test_missing_sensor_diagnostics_fail_with_a_recording_instruction():
    log = _log()
    del log['estimated_support_loads_N']
    with pytest.raises(ValueError, match='record_diagnostics=True'):
        diagnostic_signals(log)


def test_signed_command_components_reconstruct_the_requested_command():
    log = _log()
    component_keys = ('equilibrium_command', 'lqr_command', 'rolling_feedforward', 'support_command',
                      'transition_command_delta', 'heading_command', 'fade_command_delta')
    rng = np.random.default_rng(4)
    for key in component_keys:
        log[key] = rng.uniform(-.1, .1, (101, 5))
    log['command'] = sum(log[key] for key in component_keys)
    signals = diagnostic_signals(log)
    np.testing.assert_array_equal(signals['command_reconstruction_error_fraction'], 0.)
    # Opposing components should remain visible even when their total cancels.
    log['lqr_command'][50, 3] += .3
    assert diagnostic_signals(log)['command_reconstruction_error_fraction'][50] == pytest.approx(1.)


def test_unloaded_truth_placeholder_is_not_a_contact_point_error():
    log = _log()
    log['support_loads_N'][50] = 0.
    log['contact_point_m'][50] = log['qpos'][50, :3]
    signals = diagnostic_signals(log)
    assert signals['velocity_consistency_valid'][50]
    assert not signals['truth_contact_valid'][50]
    assert np.isnan(signals['contact_point_error_m'][50])
    assert np.nanmax(signals['contact_point_error_m']) == 0.
