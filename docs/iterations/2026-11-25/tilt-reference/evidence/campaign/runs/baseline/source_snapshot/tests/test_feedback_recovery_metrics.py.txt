"""Recovery classification distinguishes a brief return from sustained settling."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest


SPEC = importlib.util.spec_from_file_location('feedback_recovery_runner',
    Path(__file__).parents[1]/'scripts/run-feedback-recovery.py')
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


def trace():
    time = np.arange(0., 3.005, .005)
    qpos = np.zeros((len(time), 10))
    qpos[:, 0] = .01
    qpos[:, 3] = 1.
    return {'time_s': time, 'qpos': qpos, 'qvel': np.zeros((len(time), 9)),
        'reference_xy': np.zeros((len(time), 2)), 'attitude_error_rad': np.zeros(len(time)),
        'tilt_rad': np.zeros(len(time)), 'reference_quat': qpos[:, 3:7].copy(),
        'mpc_fallback': np.zeros(len(time), bool),
        'mpc_total_wall_s': np.zeros(len(time)), 'mpc_updated': np.zeros(len(time), bool),
        'recovery_actual_input_at_limit': np.zeros((len(time), 5), bool),
        'normal_force_N': np.full(len(time), 18.)}


def test_return_requires_a_sustained_window_and_low_velocity():
    log = trace()
    log['qpos'][log['time_s'] >= 1., 0] = 0.
    result = RUNNER.recovery_metrics(log, .4)
    assert result['goal_error_at_feedback_start_mm'] == pytest.approx(10.)
    assert result['first_entry_within_5mm_time_s'] == pytest.approx(1.)
    assert result['first_1s_settled_window_start_s'] == pytest.approx(1.)
    assert result['final_1s_within_goal_and_speed_and_reference_lean']
    log['qvel'][:, 0] = .006
    moving = RUNNER.recovery_metrics(log, .4)
    assert moving['first_entry_within_5mm_time_s'] == pytest.approx(1.)
    assert moving['first_1s_settled_window_start_s'] is None
    assert not moving['final_1s_within_goal_and_speed_and_reference_lean']


def test_brief_goal_crossing_does_not_count_as_recovery():
    log = trace()
    crossing = (log['time_s'] >= 1.) & (log['time_s'] <= 1.05)
    log['qpos'][crossing, 0] = 0.
    result = RUNNER.recovery_metrics(log, .4)
    assert result['first_entry_within_5mm_time_s'] == pytest.approx(1.)
    assert result['first_1s_settled_window_start_s'] is None
    assert not result['final_1s_within_goal_and_speed_and_reference_lean']


def test_absolute_tilt_is_reported_separately_from_reference_error():
    log = trace()
    log['qpos'][:, 0] = 0.
    lean = np.deg2rad(10.)
    log['tilt_rad'][:] = lean
    log['reference_quat'][:] = [np.cos(lean/2), np.sin(lean/2), 0., 0.]
    result = RUNNER.recovery_metrics(log, .4)
    assert result['final_1s_within_goal_and_speed_and_reference_lean']
    assert result['absolute_platform_tilt_final_deg'] == pytest.approx(10.)
    assert result['reference_platform_tilt_final_deg'] == pytest.approx(10.)
    assert result['reference_lean_error_final_deg'] == 0.
