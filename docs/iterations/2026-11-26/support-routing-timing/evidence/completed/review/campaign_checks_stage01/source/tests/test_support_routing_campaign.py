"""Independent integration contracts for I46's non-actuated routing audit.

Synthetic bridge/feedback fixtures test clocks and information flow without
running Gazebo, changing old sources, reading terrain or replaying a campaign.
"""
from copy import deepcopy
from pathlib import Path
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from spherical_wheel import support_routing_audit as audit
from spherical_wheel.gazebo.bridge import STATE_FIELDS


SCRIPT = Path(__file__).resolve().parents[1]/'scripts/audit-support-routing.py'
driver = ModuleType('_independent_support_routing_campaign')
driver.__file__ = str(SCRIPT)
exec(compile(SCRIPT.read_text(), str(SCRIPT), 'exec'), driver.__dict__)


def contact_row(tag, force_N, position=(0., 0., 0.), admitted=True):
    row = np.zeros(16)
    row[0], row[1:4], row[4:7], row[7:10] = tag, position, [0., 0., 1.], [0., 0., force_N]
    row[10:13] = [.001, .7, float(admitted)]
    return row


class MovingBridge:
    def __init__(self, rows=None):
        self.steps, self.run_calls, self.state_reads, self.contact_reads = 0, [], 0, 0
        self.make_rows = rows

    def run(self, count):
        self.run_calls.append(count)
        self.steps += count

    def state(self):
        self.state_reads += 1
        value = np.zeros(len(STATE_FIELDS))
        value[:5] = [self.steps*.001, self.steps, self.steps*.01, 0., .12]
        return value

    def contacts(self):
        self.contact_reads += 1
        if self.make_rows is not None:
            return self.make_rows(self.steps)
        return np.array([contact_row(0, self.steps, [self.steps*.02, 0., 0.])])

    def command(self, *args):
        raise AssertionError('Sampler cannot issue a command')

    def _pull(self):
        raise AssertionError('Sampler cannot update the control mirror')


def fake_controller():
    calls = []
    data = SimpleNamespace(qpos=np.r_[0., 0., .12, 1., 0., 0., 0., 1.1, 0., 0.],
                           qvel=np.zeros(9), time=1., ctrl=np.zeros(5), xmat=np.eye(3).reshape(1, 9))
    observer_data = SimpleNamespace(qpos=data.qpos.copy(), qvel=data.qvel.copy())
    observer = SimpleNamespace(generalized=np.r_[0., 0., 10., 0., 0., 0., 0., 0., 0.],
        interval_mean=np.r_[2., 0., 10., 0., 0., 0., 0., 0., 0.], _previous=(.995, np.eye(9)),
        _mass=np.eye(9), _data=observer_data)

    def forbidden(*args, **kwargs):
        raise AssertionError('A diagnostic must not update observer or call native feedback again')

    observer.update = forbidden
    observer.wrench = lambda: (observer.generalized[:3].copy(), observer.generalized[3:6].copy())

    def records(current, force, moment):
        assert current is data
        calls.append(('locator', force.copy(), moment.copy(), current.qpos.copy()))
        if force[2] <= 0:
            return []
        patch = 'belt_A' if force[0] > 1. else 'shell_A'
        return [dict(name=patch+'_smooth', normal_force=float(force[2]),
                     pos=np.array([0., 0., 0.]))]

    locator = SimpleNamespace(radius=.12, records=records,
        patches=[dict(body=0, sign=1., lower=0., upper=.12) for _ in range(4)])

    def gain(phase, loads):
        calls.append(('gain', phase, dict(loads)))
        value = sum((index+1)*loads[name] for index, name in enumerate(audit.LABELS))/sum(loads.values())
        return np.full((5, 18), value)

    def support(normal, loads):
        calls.append(('holding', normal.copy(), dict(loads)))
        return np.array([0., .1*loads['shell_A'], .1*loads['shell_B'],
                         .2*loads['belt_A'], .2*loads['belt_B']])

    controller = SimpleNamespace(plant=SimpleNamespace(data=data, last_input=np.zeros(5),
        model=SimpleNamespace(geom_surfacevel=np.zeros((4, 3)))), observer=observer,
        estimator=SimpleNamespace(normal=np.array([0., 0., 1.]), friction=.7), locator=locator,
        synchronizer=SimpleNamespace(_plane=None, _data=SimpleNamespace(qpos=data.qpos.copy(), qvel=np.zeros(9), time=1.)),
        feedback=SimpleNamespace(_gain=gain), params=SimpleNamespace(support_feedforward=1.3), support_torque=support,
        ordinary={'equilibrium':{'u':np.arange(5)*.01}}, _terrain_feedback=forbidden,
        _held={'loads':dict(zip(audit.LABELS, [10., 0., 0., 0.]))}, nominal_joints=np.array([1.1, 0., 0.]),
        _vref=np.zeros(9), _sync={'targets':[]}, _plane=None, _slow_normal=np.array([0., 0., 1.]),
        _fall_end=None, _yaw_error=.02, heading_reference=.4, _tilt_reference=None)
    normal = np.array([0., .1, np.sqrt(.99)])
    measurement = dict(loads_N=dict(zip(audit.LABELS, [10., 0., 0., 0.])), point_m=np.zeros(3))
    native_gain = np.ones((5, 18)); native_gain[0] = np.arange(18)*.01
    error = np.linspace(-.01, .02, 18)
    ff = np.linspace(-.03, .04, 5)
    native_support = 1.3*support(normal, measurement['loads_N'])
    native_feedback = dict(K=native_gain, error=error, support_torque=native_support, effective_phase_rad=.8)
    command = controller.ordinary['equilibrium']['u']+ff+native_support-native_gain@error
    calls.clear()
    return controller, normal, measurement, native_feedback, command, ff, calls


def test_probe_returns_exact_original_command_and_invokes_native_feedback_once(monkeypatch):
    probe, calls = driver.NativeFeedbackProbe(), []
    command, feedback = np.arange(5, dtype=float), {'K':'native-result'}
    qref, vref, ff, normal = np.arange(10.), np.arange(9.), np.arange(5.), np.array([0., 0., 1.])

    def original(*args):
        calls.append(('native', args))
        return command, feedback

    def shadow(*args):
        calls.append(('shadow', args))
        assert len(calls) == 2 and calls[0][0] == 'native'
        assert args[4] is feedback and args[5] is command
        return {'scope':'unexecuted'}

    monkeypatch.setattr(driver, 'feedback_routing_shadow', shadow)
    probe.begin()
    result = probe.feedback(original, object(), qref, vref, ff, normal, {'loads_N':{}}, True)
    assert result[0] is command and result[1] is feedback
    assert probe.finish() == dict(feedback_calls=1, shadow={'scope':'unexecuted'})
    assert [name for name, _ in calls] == ['native', 'shadow']


def test_probe_rejects_clock_scope_overlap_and_multiple_native_calls(monkeypatch):
    monkeypatch.setattr(driver, 'feedback_routing_shadow', lambda *args: {})
    probe = driver.NativeFeedbackProbe()
    args = (lambda *a:(np.zeros(5), {}), object(), np.zeros(10), np.zeros(9), np.zeros(5), np.ones(3), {}, True)
    with pytest.raises(AssertionError):
        probe.feedback(*args)
    probe.begin()
    with pytest.raises(AssertionError):
        probe.begin()
    with pytest.raises(AssertionError):
        probe.finish()
    probe.feedback(*args)
    probe.feedback(*args)
    with pytest.raises(AssertionError):
        probe.finish()


def test_probe_catches_native_reference_mutation_before_returning_a_diagnostic(monkeypatch):
    monkeypatch.setattr(driver, 'feedback_routing_shadow', lambda *args: {})
    probe, normal = driver.NativeFeedbackProbe(), np.array([0., 0., 1.])

    def mutating_native(controller, qref, vref, ff, current_normal, measurement, touching):
        current_normal[0] = .5
        return np.zeros(5), {}

    probe.begin()
    with pytest.raises(AssertionError):
        probe.feedback(mutating_native, object(), np.zeros(10), np.zeros(9), np.zeros(5), normal, {}, True)


def test_shadow_routes_only_gain_and_holding_with_same_error_normal_and_feedforward():
    controller, normal, measurement, feedback, command, ff, calls = fake_controller()
    before = audit.history_fingerprint(controller)
    inputs = deepcopy((normal, measurement, feedback, command, ff))
    output = audit.feedback_routing_shadow(controller, normal, measurement, True, feedback, command, ff)
    assert output['fallback_code'] == 0 and all(output['invariants'].values())
    assert audit.history_fingerprint(controller) == before
    assert [row[0] for row in calls] == ['locator', 'locator', 'gain', 'holding']
    np.testing.assert_array_equal(calls[0][1], controller.observer.generalized[:3])
    np.testing.assert_array_equal(calls[1][1], controller.observer.interval_mean[:3])
    np.testing.assert_array_equal(calls[0][3], calls[1][3])
    np.testing.assert_array_equal(output['used_loads_N'], [0., 0., 10., 0.])
    np.testing.assert_array_equal(output['interval_share_feedback_gain'][0], feedback['K'][0])
    np.testing.assert_array_equal(output['native_error'], feedback['error'])
    np.testing.assert_array_equal(output['native_normal'], normal)
    np.testing.assert_array_equal(output['native_feedforward'], ff)
    expected = output['delta_support_command']-(output['interval_share_feedback_gain']-feedback['K'])@feedback['error']
    np.testing.assert_allclose(output['delta_pre_sync_command'], expected, rtol=0., atol=1e-15)
    np.testing.assert_allclose(output['interval_share_pre_sync_command']-command, expected, rtol=0., atol=1e-15)
    assert output['native_reconstruction_max_abs_gap'] == 0.
    assert audit.digest((normal, measurement, feedback, command, ff)) == audit.digest(inputs)


def test_native_dropout_preserves_held_feedback_loads_even_if_fast_support_is_loaded():
    controller, normal, measurement, feedback, command, ff, calls = fake_controller()
    controller.observer.generalized[:] = 0.
    measurement['loads_N'] = dict.fromkeys(audit.LABELS, 0.)
    controller._held['loads'] = dict(zip(audit.LABELS, [0., 8., 0., 0.]))
    before = audit.history_fingerprint(controller)
    result = audit.feedback_routing_shadow(controller, normal, measurement, False, feedback, command, ff)
    assert result['fallback_code'] == 1 and result['interval']['total_load_N'] > 0
    np.testing.assert_array_equal(result['used_loads_N'], np.zeros(4))
    np.testing.assert_array_equal(result['native_feedback_effective_loads_N'], [0., 8., 0., 0.])
    np.testing.assert_array_equal(result['used_feedback_effective_loads_N'], [0., 8., 0., 0.])
    np.testing.assert_array_equal(result['delta_pre_sync_command'], np.zeros(5))
    assert [row[0] for row in calls] == ['locator', 'locator']
    assert audit.history_fingerprint(controller) == before


def test_native_filtered_locator_mismatch_is_a_hard_integrity_failure():
    controller, normal, measurement, feedback, command, ff, _ = fake_controller()
    measurement['loads_N']['shell_A'] = 9.999
    with pytest.raises(AssertionError, match='reproduce native'):
        audit.feedback_routing_shadow(controller, normal, measurement, True, feedback, command, ff)


def test_five_postsolve_samples_use_moving_centres_and_force_weighted_points():
    sim, sampler = MovingBridge(), None
    sampler = audit.IntervalTruthSampler(sim)
    assert sampler.completed_interval(0.) is None  # t0 has no preceding interval.
    sampler.begin(0.)
    sampler.run(5)
    result = sampler.finish()
    assert sim.run_calls == [1]*5 and sim.state_reads == sim.contact_reads == 5
    np.testing.assert_allclose(result['substep_time_s'], np.arange(1, 6)*.001, rtol=0., atol=0.)
    np.testing.assert_array_equal(result['substep_time_s'], result['substep_bridge_time_s'])
    np.testing.assert_array_equal(result['substep_loads_N'][:, 0], np.arange(1, 6))
    np.testing.assert_array_equal(result['mean_loads_N'], [3., 0., 0., 0.])
    # Contact x=.02*k, centre x=.01*k and Fz=k => My=-.01*k**2.
    np.testing.assert_allclose(result['mean_wrench_world'], [0., 0., 3., 0., -.11, 0.], rtol=0., atol=2e-16)
    assert result['mean_wrench_world'][4] != pytest.approx(-.07)  # Fixed final-centre convention.
    assert result['mean_patch_points_m'][0, 0] == pytest.approx(.02*55/15)
    assert result['mean_patch_points_m'][0, 0] != pytest.approx(.06)  # Mean of unweighted points.
    assert result['substep_wrench_world'][-1, 2] == 5. != result['mean_wrench_world'][2]
    assert sampler.completed_interval(.005) is result


def test_interval_truth_averages_loads_before_normalizing_not_per_tick_shares():
    def rows(step):
        return np.array([contact_row(0 if step < 5 else 2, 1. if step < 5 else 20.)])
    sampler = audit.IntervalTruthSampler(MovingBridge(rows))
    sampler.begin(1.)
    sampler.run(5)
    result = sampler.finish()
    np.testing.assert_array_equal(result['mean_loads_N'], [.8, 0., 4., 0.])
    shares = result['mean_loads_N']/result['mean_loads_N'].sum()
    np.testing.assert_allclose(shares, [1/6, 0., 5/6, 0.], rtol=0., atol=1e-15)
    assert shares[0] != pytest.approx(.8)


def test_sampler_delegates_setup_without_truth_reads_and_rejects_misaligned_or_partial_intervals():
    sim = MovingBridge()
    sampler = audit.IntervalTruthSampler(sim)
    sampler.run(3)
    assert sim.run_calls == [3] and sim.state_reads == sim.contact_reads == 0
    assert sampler.latest is None
    sampler.begin(.003)
    with pytest.raises(AssertionError):
        sampler.finish()
    with pytest.raises(AssertionError):
        sampler.run(4)
    sampler.run(5)
    sampler.finish()
    with pytest.raises(ValueError, match='align'):
        sampler.completed_interval(.009)
    assert sampler.completed_interval(.008)['start_time_s'] == .003


def test_weak_resolved_count_caveat_and_body_contacts_stay_separate_in_truth():
    rows = np.array([contact_row(4, 1., [0., 0., 0.]),
                     contact_row(0, .04, [.1, 0., 0.]),
                     contact_row(6, 50.), contact_row(2, 100., admitted=False)])
    truth = audit.bridge_contact_truth(rows, [0., 0., .12])
    np.testing.assert_array_equal(truth['resolved_contact_counts'], [1, 0, 0, 0])
    np.testing.assert_array_equal(truth['loads_N'], [1.04, 0., 0., 0.])
    assert truth['patch_points_m'][0, 0] == pytest.approx(.004/1.04)
    assert truth['body_load_N'] == 50. and truth['wrench_world'][2] == 1.04
    np.testing.assert_array_equal(truth['body_wrench_world'], [0., 0., 50., 0., 0., 0.])
    np.testing.assert_array_equal(truth['all_external_wrench_world'], truth['wrench_world']+truth['body_wrench_world'])
    empty = audit.bridge_contact_truth(np.empty((0, 16)), [0., 0., .12])
    assert np.isnan(empty['equivalent_point_m']).all()
    np.testing.assert_array_equal(empty['loads_N'], np.zeros(4))
