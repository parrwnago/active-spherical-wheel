"""Integration fixtures for I-45; no dynamics, controller or scene execution."""
from __future__ import annotations

import copy
import hashlib
from pathlib import Path
import sys
import types

import numpy as np
import pytest

from spherical_wheel.contact_pose_diagnostic import (
    ARMS, AnchorProvider, classify_saved_contact_truth, compare_values,
    generate_pose_conditions, score_contact_preview,
)
from spherical_wheel.model import REPO_ROOT
from spherical_wheel.terrain_depth import DepthFrame
from spherical_wheel.terrain_memory import (
    MemorySettings, TerrainMemory, WheelGeometry, preview_contacts,
)


class Plane:
    def __init__(self, anchor=1.):
        self.anchor = anchor
        self.calls = []

    def query(self, xy, at_time_s):
        assert at_time_s == self.anchor
        self.calls.append((np.asarray(xy).copy(), at_time_s))
        return dict(supported=True, reason='fixture_plane', point_m=np.r_[xy, 0.],
                    normal=np.array([0., 0., 1.]), height_m=0.)


def state():
    return np.array([.005, .005, .12, 1., 0., 0., 0., .2, -.3, .4])


def module_from_runner():
    path = REPO_ROOT/'scripts/diagnose-contact-preview-pose.py'
    module = types.ModuleType('_I45_fixture_runner')
    module.__file__ = str(path)
    exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
    return module


def test_cv_keeps_full_unchanged_I44_preview_and_oracle_does_not_reintegrate():
    geometry = WheelGeometry(.12, 15.46600995, .01)
    q = state()
    v = np.array([.1, -.05, .01, .1, -.2, .3, .4, -.5, .6])
    future = {h: q+np.r_[h*np.array([.2, -.3, .4]), np.zeros(4), h*np.array([4., 5., 6.])]
              for h in (.05, .1, .2)}
    future[.1][3:7] = np.array([.8, .1, .3, -.2])
    q_before, v_before, future_before = q.copy(), v.copy(), copy.deepcopy(future)
    providers = {'observer_plane': Plane(), 'memory_four': Plane()}
    result = generate_pose_conditions(providers, geometry, q, v, 1., future)
    assert set(result) == set(ARMS)
    metadata = {'pose_source', 'uses_future_configuration', 'deployable_input_condition'}
    old = preview_contacts(Plane(), geometry, q, v, 1.)['rows']
    for arm in ('observer_cv', 'memory_cv'):
        for row, original in zip(result[arm], old):
            assert not compare_values({k: x for k, x in row.items() if k not in metadata}, original, atol=0.)
    for arm in ('observer_actual_pose', 'memory_actual_pose'):
        for horizon, row in zip((.05, .1, .2), result[arm]):
            np.testing.assert_array_equal(row['predicted_qpos'][:3], future[horizon][:3])
            np.testing.assert_array_equal(row['predicted_qpos'][7:], future[horizon][7:])
            np.testing.assert_allclose(row['predicted_qpos'][3:7],
                future[horizon][3:7]/np.linalg.norm(future[horizon][3:7]), atol=4*np.finfo(float).eps, rtol=0.)
            legacy_endpoint = preview_contacts(Plane(), geometry, future[horizon], np.zeros(9), 1., horizons_s=(0.,))['rows'][0]
            np.testing.assert_array_equal(row['predicted_qpos'], legacy_endpoint['predicted_qpos'])
            assert row['uses_future_configuration'] and not row['deployable_input_condition']
            assert row['endpoint_evaluation_horizon_s'] == 0.
            assert row['horizon_s'] == horizon and row['time_s'] == 1.+horizon
    np.testing.assert_array_equal(q, q_before)
    np.testing.assert_array_equal(v, v_before)
    for h in future:
        np.testing.assert_array_equal(future[h], future_before[h])


def flat_frame():
    transform = np.eye(4)
    transform[:3, :3] = np.diag([1., -1., -1.])
    transform[2, 3] = 1.
    return DepthFrame(camera_name='fixture', depth_m=np.ones((21, 21)),
        valid_mask=np.ones((21, 21), bool),
        intrinsics=np.array([[100., 0., 10.], [0., 100., 10.], [0., 0., 1.]]),
        body_from_camera=transform, world_from_camera=transform,
        capture_time_s=.5, delivered_at_s=.55)


def test_locked_live_memory_and_snapshot_have_identical_preview_without_mutation():
    memory = TerrainMemory(MemorySettings(normal_radius_m=.06))
    memory.ingest([flat_frame()], available_at_s=1.)
    snapshot = memory.snapshot(at_time_s=1.)
    cells_before = memory.observed_cells(at_time_s=1.)
    geometry = WheelGeometry(.12, 15.46600995, .01)
    q, v = state(), np.zeros(9)
    future = {h: q.copy() for h in (.05, .1, .2)}
    live = generate_pose_conditions(dict(observer_plane=Plane(), memory_four=memory),
                                    geometry, q, v, 1., future)
    frozen = generate_pose_conditions(dict(observer_plane=Plane(), memory_four=snapshot),
                                      geometry, q, v, 1., future)
    assert not compare_values(live, frozen, atol=0.)
    assert not compare_values(memory.observed_cells(at_time_s=1.), cells_before, atol=0.)
    with pytest.raises(ValueError, match='past anchor'):
        memory.query(q[:2], at_time_s=.9)
    with pytest.raises(ValueError, match='frozen anchor'):
        AnchorProvider(memory, 1.).query(q[:2], at_time_s=1.1)


def test_truth_mean_qualification_and_wrong_patch_distance_survive_identity_failure():
    points = np.full((4, 3), np.nan)
    points[0] = [1., 2., 3.]
    truth = classify_saved_contact_truth([5., 0., 0., 0.], [1, 0, 0, 0], points,
                                       [1., 2., 3.], 20.)
    assert truth['category'] == 'single_resolved_patch_mean'
    assert truth['primary_eligible'] and 'Weak contacts on the same patch can remain' in truth['qualification']
    preview = dict(candidates=[dict(patch='belt_B', point_m=np.array([1.001, 2., 3.]))],
                   contact_prediction_supported=True, ambiguous=False)
    score = score_contact_preview(preview, truth)
    assert not score['primary_patch_in_candidates']
    assert score['primary_nearest_candidate_error_m'] == pytest.approx(.001)
    preview['candidates'].append(dict(patch='belt_B', point_m=np.array([1.002, 2., 3.])))
    preview['contact_prediction_supported'] = False
    score = score_contact_preview(preview, truth)
    assert score['unique_patch_count'] == 1 and score['geometric_candidate_count'] == 2
    assert score['primary_nearest_candidate_error_m'] == pytest.approx(.001)
    assert not score['primary_patch_in_candidates']


@pytest.mark.parametrize('loads,counts,category', [
    ([0., 0., 0., 0.], [0, 0, 0, 0], 'unloaded'),
    ([.01, 0., 0., 0.], [0, 0, 0, 0], 'weak_or_unresolved'),
    ([5., 0., 0., 0.], [2, 0, 0, 0], 'single_patch_multiple_resolved'),
    ([5., 5., 0., 0.], [1, 1, 0, 0], 'multiple_loaded_patches'),
    ([5., .051, 0., 0.], [1, 0, 0, 0], 'weak_or_unresolved'),
])
def test_truth_strata_preserve_legacy_mean_without_relabeling_as_single_contact(loads, counts, category):
    truth = classify_saved_contact_truth(loads, counts, np.zeros((4, 3)), [0., 0., 0.], 20.)
    assert truth['category'] == category and not truth['primary_eligible']
    assert truth['legacy_equivalent_eligible'] == (max(loads) >= 1.)


def test_streamed_map_prefix_matches_first_rows_without_loading_tail(tmp_path):
    runner = module_from_runner()
    arrays = dict(centers_m=np.arange(9000, dtype=float).reshape(3000, 3),
                  count=np.arange(3000, dtype=np.int64))
    path = tmp_path/'map.npz'
    np.savez_compressed(path, **arrays)
    for key, values in arrays.items():
        np.testing.assert_array_equal(runner.npz_array_prefix(path, key, 7), values[:7])
    with pytest.raises(ValueError, match='prefix'):
        runner.npz_array_prefix(path, 'count', 3001)


def test_runner_refuses_existing_stage_before_source_or_campaign_execution(tmp_path, monkeypatch):
    runner = module_from_runner()
    (tmp_path/'protocol.json').write_text('{}')
    stage = tmp_path/'campaign'/'fixture_stage'
    stage.mkdir(parents=True)
    sentinel = stage/'preserved.txt'
    sentinel.write_text('earlier failure')
    monkeypatch.setattr(sys, 'argv', ['diagnose', '--output', str(tmp_path), '--tag', 'fixture_stage', '--run'])
    with pytest.raises(FileExistsError):
        runner.main()
    assert sentinel.read_text() == 'earlier failure'
    assert sorted(p.name for p in stage.iterdir()) == ['preserved.txt']


def test_runner_preflight_verifies_registered_hash_without_generating_predictions(tmp_path):
    runner = module_from_runner()
    payload = tmp_path/'payload'
    payload.write_bytes(b'fixture')
    protocol = dict(inputs={'fixture': str(payload)},
                    input_sha256={'fixture': hashlib.sha256(b'fixture').hexdigest()})
    assert runner.verify_inputs(protocol) == protocol['input_sha256']
    payload.write_bytes(b'changed')
    with pytest.raises(RuntimeError, match='inputs changed'):
        runner.verify_inputs(protocol)
