"""I-46 kinematic/native-feedback fixtures; no Gazebo or dynamics stepping."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import sys
import types

import numpy as np
import pytest

from spherical_wheel.contact_sensing import SupportLocator
from spherical_wheel.gazebo.bridge import STATE_FIELDS
from spherical_wheel.mirror import StateMirror
from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.support_routing_audit import (
    IntervalTruthSampler, bridge_contact_truth, feedback_routing_shadow,
    history_fingerprint, scaled_interval_loads, support_from_wrench,
)


def test_same_world_wrench_classification_changes_only_with_current_geometry(tmp_path):
    mirror = StateMirror(load_config(), tmp_path/'mirror')
    q = np.array([0., 0., .12, 1., 0., 0., 0., 0., 0., 0.])
    locator = SupportLocator(mirror.model, mirror.fork_id, .12, .7, .01)
    mirror.set_state(q, np.zeros(9), 1., np.zeros(5))
    rim = support_from_wrench(locator, mirror.data, [0., 0., 20., 0., 0., 0.])
    np.testing.assert_allclose(rim['shares'], [.5, .5, 0., 0.], atol=1e-14)
    assert rim['record_count'] == 2 and rim['point_valid']
    assert rim['point_m'][2] > 0.  # Shared-rim mean differs from ideal spherical extremum.
    q[7] = np.pi/2
    mirror.set_state(q, np.zeros(9), 1., np.zeros(5))
    pole = support_from_wrench(locator, mirror.data, [0., 0., 20., 0., 0., 0.])
    np.testing.assert_array_equal(pole['shares'], [0., 0., 1., 0.])
    assert pole['dominant_patch_index'] == 2
    for k, patch in enumerate(locator.patches):
        axis = mirror.data.xmat[patch['body']].reshape(3, 3)[:, 0]
        coordinate = patch['sign']*axis@np.array([0., 0., -.12])
        assert pole['lower_band_margin_m'][k] == pytest.approx(coordinate-patch['lower'])
        assert pole['upper_band_margin_m'][k] == pytest.approx(patch['upper']-coordinate)


def test_world_moment_is_not_rotated_again_and_weak_support_is_explicit(tmp_path):
    mirror = StateMirror(load_config(), tmp_path/'mirror')
    q = mirror.data.qpos.copy()
    q[:3] = [2., -3., .12]
    q[3:7] = [np.cos(.3), 0., 0., np.sin(.3)]
    mirror.set_state(q, np.zeros(9), 1., np.zeros(5))
    locator = SupportLocator(mirror.model, mirror.fork_id, .12, .7, 1.)
    normal = np.array([.2, -.1, np.sqrt(.95)])
    force = np.array([0., 0., 20.])
    moment = np.cross(-.12*normal, force)
    support = support_from_wrench(locator, mirror.data, np.r_[force, moment])
    np.testing.assert_allclose(support['normal'], normal, atol=1e-14)
    weak = support_from_wrench(locator, mirror.data, np.r_[force*.001, moment*.001])
    assert weak['intrinsic_normal_valid'] and not weak['records_available'] and not weak['point_valid']
    assert np.isnan(weak['point_m']).all()
    invalid = support_from_wrench(locator, mirror.data, np.full(6, np.nan))
    assert not invalid['valid_wrench'] and not invalid['records_available']


def test_native_controller_shadow_is_pure_and_decomposes_gain_and_holding(tmp_path):
    from spherical_wheel.offroad_experiments import Workspace
    from spherical_wheel.offroad_control import OffroadController, OffroadParams
    workspace = Workspace(load_config(), tmp_path/'workspace',
                          seed_cache=REPO_ROOT/'generated/offroad/flat/control_schedule.npz')
    mirror = workspace.mirror()
    controller = OffroadController(mirror, workspace.design,
        OffroadParams(mode='hardware', rolling_allocation='least_squares', track_contact_point=False),
        phase=1.1, record_diagnostics=True)
    q = controller.ordinary['equilibrium']['qpos'].copy()
    q[0] += .01
    mirror.set_state(q, np.zeros(9), 0., np.zeros(5))
    controller.reset(1.1)
    _, info = controller.command(np.zeros(2), np.zeros(2), np.zeros(2))
    # A prescribed world wrench fixture, not an estimator update or force read.
    axis = mirror.data.xmat[controller.carrier_id].reshape(3, 3)[:, 0]
    target_normal = -axis if axis[2] < 0 else axis
    if target_normal[2] < 0:
        target_normal = -target_normal
    force = controller.weight*target_normal
    controller.observer.interval_mean[:6] = np.r_[force, np.zeros(3)]
    native_feedback = dict(K=info['feedback_gain'], error=info['error'],
        support_torque=info['support_torque'], effective_phase_rad=info['effective_phase_rad'])
    native = (np.asarray(controller.ordinary['equilibrium']['u'])+info['rolling_feedforward']
              +info['support_torque']-info['feedback_gain']@info['error'])
    before = history_fingerprint(controller)
    input_copy = copy.deepcopy((info, native_feedback))
    shadow = feedback_routing_shadow(controller, info['estimate']['normal'], info['measurement'],
        sum(info['measurement']['loads_N'].values()) > .001,
        native_feedback, native, info['rolling_feedforward'])
    assert shadow['fallback_code'] == 0
    assert history_fingerprint(controller) == before and all(shadow['invariants'].values())
    np.testing.assert_array_equal(shadow['interval_share_feedback_gain'][0], info['feedback_gain'][0])
    expected = shadow['delta_support_command']-(shadow['interval_share_feedback_gain']-info['feedback_gain'])@info['error']
    np.testing.assert_allclose(shadow['delta_pre_sync_command'], expected, atol=1e-15)
    np.testing.assert_allclose(shadow['interval_share_pre_sync_command']-native, expected, atol=1e-14)
    np.testing.assert_array_equal(native_feedback['K'], input_copy[1]['K'])
    np.testing.assert_array_equal(info['rolling_feedforward'], input_copy[0]['rolling_feedforward'])


@pytest.mark.parametrize('touching,native,fast,valid,code', [
    (False, [0., 0., 0., 0.], [1., 0., 0., 0.], True, 1),
    (True, [.001, 0., 0., 0.], [1., 0., 0., 0.], True, 2),
    (True, [5., 0., 0., 0.], [.001, 0., 0., 0.], True, 3),
    (True, [5., 0., 0., 0.], [np.nan, 0., 0., 0.], True, 4),
    (True, [5., 0., 0., 0.], [1., 0., 0., 0.], False, 4),
])
def test_share_routing_fallback_preserves_native_loads(touching, native, fast, valid, code):
    used, actual = scaled_interval_loads(native, fast, touching, valid)
    assert actual == code
    np.testing.assert_array_equal(used, native)


def contact_row(position, force, tag=0, admitted=1.):
    row = np.zeros(16)
    row[0], row[1:4], row[4:7], row[7:10] = tag, position, [0., 0., 1.], force
    row[10], row[11], row[12] = .001, .7, admitted
    return row


def test_bridge_truth_uses_admission_rim_mapping_and_matching_centre():
    centre = np.array([1., 2., .12])
    rows = np.array([contact_row([1.01, 2., 0.], [0., 0., 10.], tag=4),
                     contact_row([1., 2.02, 0.], [0., 0., 4.], tag=3),
                     contact_row([1.02, 2.03, 0.], [0., 0., 90.], tag=6),
                     contact_row([1., 2., 0.], [0., 0., 999.], admitted=0.)])
    truth = bridge_contact_truth(rows, centre)
    np.testing.assert_array_equal(truth['loads_N'], [10., 0., 0., 4.])
    np.testing.assert_allclose(truth['wrench_world'], [0., 0., 14., .08, -.1, 0.], atol=1e-14)
    assert truth['body_load_N'] == 90.
    np.testing.assert_allclose(truth['body_wrench_world'], [0., 0., 90., 2.7, -1.8, 0.], atol=2e-14)
    np.testing.assert_allclose(truth['all_external_wrench_world'], [0., 0., 104., 2.78, -1.9, 0.], atol=2e-14)


class FakeSim:
    def __init__(self):
        self.steps = 0
        self.calls = []

    def run(self, iterations):
        self.calls.append(iterations)
        self.steps += iterations

    def state(self):
        value = np.zeros(len(STATE_FIELDS))
        value[:5] = [self.steps*.001, self.steps, self.steps*.01, 0., .12]
        return value

    def contacts(self):
        return np.array([contact_row([0., 0., 0.], [0., 0., 10.])])


def test_sampler_keeps_control_clock_and_means_about_each_moving_centre():
    sim = FakeSim()
    sampler = IntervalTruthSampler(sim)
    sampler.run(2)  # Setup/reset is delegated, not collected.
    assert sampler.latest is None and sim.calls == [2]
    sampler.begin(1.)
    sampler.run(5)
    result = sampler.finish()
    assert sim.calls == [2, 1, 1, 1, 1, 1]
    np.testing.assert_allclose(result['substep_time_s'], 1.+np.arange(1, 6)*.001, atol=0.)
    np.testing.assert_allclose(result['substep_bridge_time_s'], np.arange(3, 8)*.001, atol=0.)
    np.testing.assert_allclose(result['mean_wrench_world'], [0., 0., 10., 0., .5, 0.], atol=1e-14)
    # Fixed endpoint-centre moments would instead yield .7Nm: averaging is explicit.
    assert result['mean_wrench_world'][4] != pytest.approx(.7)
    assert len(result['substep_contact_rows']) == 5 and result['substep_bridge_state'].shape == (5, len(STATE_FIELDS))
    assert sampler.completed_interval(1.005) is result
    with pytest.raises(ValueError, match='align'):
        sampler.completed_interval(1.01)


def test_new_driver_refuses_an_existing_history_stage(tmp_path, monkeypatch):
    path = REPO_ROOT/'scripts/audit-support-routing.py'
    module = types.ModuleType('_I46_fixture_driver')
    module.__file__ = str(path)
    exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
    stage = tmp_path/'campaign/historical_fixture'
    stage.mkdir(parents=True)
    monkeypatch.setattr(sys, 'argv', ['audit', '--output', str(tmp_path), '--historical', '--tag', 'fixture'])
    (tmp_path/'protocol.json').write_text('{"iteration":"I-46"}')
    with pytest.raises(FileExistsError):
        module.main()
    assert not list(stage.iterdir())


def fixture_driver():
    path = REPO_ROOT/'scripts/audit-support-routing.py'
    module = types.ModuleType('_I46_gate_fixture_driver')
    module.__file__ = str(path)
    exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
    return module


def make_bridge_gate(tmp_path):
    module = fixture_driver()
    module.REPO_ROOT = tmp_path
    family = tmp_path/'family'
    source = tmp_path/'source.py'
    source.write_text('# fixed fixture source\n')
    binary = tmp_path/'bridge.so'
    binary.write_bytes(b'fixed binary')
    inputs = {}
    for branch in ('batch', 'single'):
        directory = family/'review'/f'bridge_fixture_stage01_{branch}'
        directory.mkdir(parents=True)
        (directory/'source.py').write_bytes(source.read_bytes())
        (directory/'bridge.so').write_bytes(binary.read_bytes())
        (directory/'bridge_fixture.npz').write_bytes(b'fixture array hash binding')
        provenance = dict(sources=[dict(origin='source.py', snapshot='source.py', sha256=module.sha(source))],
            bridge_binary=dict(path='bridge.so', snapshot='bridge.so', sha256=module.sha(binary)))
        (directory/'provenance.json').write_text(json.dumps(provenance))
        for filename in ('bridge_fixture.npz', 'provenance.json'):
            path = directory/filename
            inputs[str(path.relative_to(tmp_path))] = module.sha(path)
    qa = family/'review/bridge_equivalence_stage01/report.json'
    qa.parent.mkdir()
    qa.write_text(json.dumps(dict(passed=True, inputs_sha256=inputs)))
    return module, family, qa


def test_collector_gate_binds_both_fixtures_and_current_source_binary(tmp_path):
    module, family, _ = make_bridge_gate(tmp_path)
    assert module.verify_bridge_equivalence(family)['required_fixture_inputs'] == 4
    (tmp_path/'source.py').write_text('# different source\n')
    with pytest.raises(RuntimeError, match='source no longer matches'):
        module.verify_bridge_equivalence(family)


@pytest.mark.parametrize('failed_report', [dict(passed=False), dict(passed=True, inputs_sha256={})])
def test_collector_gate_rejects_failed_or_unbound_pass_marker(tmp_path, failed_report):
    module, family, qa = make_bridge_gate(tmp_path)
    qa.write_text(json.dumps(failed_report))
    with pytest.raises(RuntimeError, match='must pass|bind both'):
        module.verify_bridge_equivalence(family)
