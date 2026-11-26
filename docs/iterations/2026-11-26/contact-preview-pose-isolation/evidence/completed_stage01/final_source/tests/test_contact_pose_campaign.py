"""Independent contracts for the saved-pose contact-preview ablation.

Synthetic planes/states exercise information boundaries and denominators.  No
controller, scene truth, depth campaign, Gazebo step or old artifact is used.
"""
from copy import deepcopy
from types import SimpleNamespace

import numpy as np
import pytest

from spherical_wheel import contact_pose_diagnostic as diagnostic
from spherical_wheel.terrain_memory import WheelGeometry


HORIZONS = (.05, .1, .2)
THRESHOLDS = dict(
    primary_common_min_n=30, primary_common_fraction_min=.5,
    RMS_relative_improvement_min=.1, RMS_absolute_improvement_m_min=.0005,
    p95_error_increase_m_max=.0005, matched_identity_recall_loss_max=.05,
    matched_ambiguity_fraction_increase_max=.05,
    uniform_memory_complete_nonempty_coverage_min=.7,
    oracle_memory_coverage_loss_vs_CV_memory_max=.05)


class QueryOnlyPlane:
    """A provider that fails if generation asks for observations or future time."""
    settings = SimpleNamespace(max_plane_rms_m=.003)

    def __init__(self, anchor=2., height=0.):
        self.anchor, self.height, self.queries = anchor, height, []

    def query(self, xy, *, at_time_s):
        assert at_time_s == self.anchor
        self.queries.append((np.asarray(xy).copy(), at_time_s))
        return dict(supported=True, reason='synthetic_observed_plane',
                    height_m=self.height, point_m=np.r_[xy, self.height],
                    normal=np.array([0., 0., 1.]))

    def ingest(self, *args, **kwargs):
        raise AssertionError('Generation must not acquire or ingest observations')

    @property
    def future_depth(self):
        raise AssertionError('Future depth is unavailable')


def configurations():
    q = np.array([0., 0., .12, np.cos(.3), 0., 0., np.sin(.3), .4, .2, -.3])
    v = np.array([.2, -.1, 0., .1, .2, -.3, .6, -1.2, 1.5])
    future = {}
    for index, h in enumerate(HORIZONS):
        # Deliberately different from constant velocity, including every joint.
        future[h] = np.array([.04+index*.03, -.02, .12,
                              1.001*np.cos(.12), 1.001*np.sin(.12), 0., 0.,
                              1.1+index*.13, -.7-index*.2, .8+index*.4])
    return q, v, future


def primary_truth(**changes):
    values = dict(loads_N=np.array([10., .02, 0., 0.]),
                  contact_counts=np.array([1, 0, 0, 0]),
                  patch_points_m=np.array([[1., 0., 0.], [np.nan]*3,
                                           [np.nan]*3, [np.nan]*3]),
                  equivalent_point_m=np.array([.8, .1, 0.]), weight_N=20.)
    values.update(changes)
    return diagnostic.classify_saved_contact_truth(**values)


def metric(rmse=.005, p95=.01, identity=.8, ambiguity=.1):
    return dict(rmse_m=rmse, p95_m=p95, identity_recall=identity,
                ambiguity_fraction=ambiguity)


def pair(memory=None):
    return dict(observer=metric(), memory=metric(.004, .009) if memory is None else memory)


def fixed_slot(index=80, *, primary=True, legacy=True):
    return [dict(arm=arm, anchor_index=index, horizon_s=.1,
                 contact_score=dict(primary_eligible=primary,
                                    legacy_equivalent_eligible=legacy,
                                    contact_preview_complete=True,
                                    nonempty_candidates=True,
                                    primary_patch_in_candidates=False,
                                    candidate_ambiguous=True,
                                    primary_nearest_candidate_error_m=.5))
            for arm in diagnostic.ARMS]


def test_endpoint_selection_is_exact_saved_time_and_never_interpolation():
    times = np.arange(201, dtype=float)*(.005+1e-15)
    original = times.copy()
    assert diagnostic.endpoint_indices(times, .4) == {.05:90, .1:100, .2:120}
    np.testing.assert_array_equal(times, original)
    for kwargs in (dict(anchor_time_s=.401), dict(anchor_time_s=.9),
                   dict(anchor_time_s=.4, horizons_s=(.1, .1))):
        with pytest.raises(ValueError):
            diagnostic.endpoint_indices(times, **kwargs)


def test_invalid_saved_clocks_and_nonfinite_tolerance_cannot_bypass_alignment():
    for times in ([], [0., 0.], [.1, 0.], [0., np.nan], [[0., .1]]):
        with pytest.raises(ValueError):
            diagnostic.endpoint_indices(times, 0.)
    for tolerance in (np.nan, np.inf, -.1):
        with pytest.raises(ValueError):
            diagnostic.endpoint_indices([0., .3], .1, tolerance_s=tolerance)


def test_all_generation_queries_use_anchor_clock_without_scoring_or_ingestion(monkeypatch):
    q, v, future = configurations()
    before_q, before_v, before_future = q.copy(), v.copy(), deepcopy(future)
    providers = {name:QueryOnlyPlane() for name in diagnostic.PROVIDERS}
    real_preview = diagnostic.preview_contacts
    calls = []

    def preview_spy(provider, geometry, qpos, qvel, anchor_time_s, **kwargs):
        calls.append((np.asarray(qpos).copy(), np.asarray(qvel).copy(),
                      anchor_time_s, tuple(kwargs['horizons_s'])))
        return real_preview(provider, geometry, qpos, qvel, anchor_time_s, **kwargs)

    def forbidden_score(*args, **kwargs):
        raise AssertionError('Evaluator truth must not enter condition generation')

    monkeypatch.setattr(diagnostic, 'preview_contacts', preview_spy)
    monkeypatch.setattr(diagnostic, 'score_contact_preview', forbidden_score)
    monkeypatch.setattr(diagnostic, 'classify_saved_contact_truth', forbidden_score)
    result = diagnostic.generate_pose_conditions(providers, WheelGeometry(.12, 15.466, .01),
                                                 q, v, 2., future)
    assert set(result) == set(diagnostic.ARMS)
    assert len(calls) == 8  # Two CV calls plus six zero-horizon endpoint calls.
    for block in (calls[:4], calls[4:]):
        np.testing.assert_array_equal(block[0][0], q)
        np.testing.assert_array_equal(block[0][1], v)
        assert block[0][3] == HORIZONS
        for h, call in zip(HORIZONS, block[1:]):
            np.testing.assert_array_equal(call[0], future[h])
            np.testing.assert_array_equal(call[1], np.zeros(9))
            assert call[2:] == (2., (0.,))
    for arm, rows in result.items():
        assert len(rows) == 3
        for h, row in zip(HORIZONS, rows):
            assert row['horizon_s'] == h and row['time_s'] == 2.+h
            if arm.endswith('actual_pose'):
                expected = future[h].copy()
                expected[3:7] /= np.linalg.norm(expected[3:7])
                np.testing.assert_allclose(row['predicted_qpos'], expected, rtol=0., atol=2e-15)
                assert row['uses_future_configuration'] and not row['deployable_input_condition']
                assert row['endpoint_evaluation_horizon_s'] == 0.
                assert row['endpoint_quaternion_normalized_by_legacy_preview']
    assert all(len(provider.queries) >= 3 for provider in providers.values())
    assert all(clock == 2. for provider in providers.values() for _, clock in provider.queries)
    np.testing.assert_array_equal(q, before_q)
    np.testing.assert_array_equal(v, before_v)
    for h in HORIZONS:
        np.testing.assert_array_equal(future[h], before_future[h])


def test_actual_pose_inputs_cannot_change_cv_rows_and_do_not_get_extra_joint_motion():
    q, v, future = configurations()
    geometry = WheelGeometry(.12, 15.466, .01)

    def generate(endpoints):
        return diagnostic.generate_pose_conditions({name:QueryOnlyPlane() for name in diagnostic.PROVIDERS},
                                                    geometry, q, v, 2., endpoints)

    first = generate(future)
    changed = deepcopy(future)
    for h in HORIZONS:
        changed[h][:3] += [.3, .1, .02]
        changed[h][7:] += [2., -3., 4.]
    second = generate(changed)
    for arm in ('observer_cv', 'memory_cv'):
        assert diagnostic.compare_values(first[arm], second[arm]) == []
    for arm in ('observer_actual_pose', 'memory_actual_pose'):
        for h, row in zip(HORIZONS, second[arm]):
            np.testing.assert_array_equal(row['predicted_qpos'][7:], changed[h][7:])
            np.testing.assert_array_equal(row['predicted_qpos'][:3], changed[h][:3])
        assert diagnostic.compare_values(first[arm], second[arm])


def test_anchor_facade_rejects_future_queries_and_protects_query_coordinates():
    provider = QueryOnlyPlane()
    facade = diagnostic.AnchorProvider(provider, 2.)
    xy = np.array([.1, .2])
    facade.query(xy, at_time_s=2.)
    provider.queries[0][0][:] = 99.
    np.testing.assert_array_equal(xy, [.1, .2])
    for time in (2.1, np.nan, np.inf):
        with pytest.raises(ValueError):
            facade.query(xy, at_time_s=time)


def test_primary_truth_selects_resolved_patch_mean_and_preserves_count_qualification():
    point = np.array([[1., 0., 0.], [np.nan]*3, [np.nan]*3, [np.nan]*3])
    truth = primary_truth(patch_points_m=point)
    assert truth['primary_eligible'] and truth['primary_patch'] == 'shell_A'
    assert truth['load_threshold_N'] == 1.
    np.testing.assert_array_equal(truth['primary_target_m'], [1., 0., 0.])
    np.testing.assert_array_equal(truth['legacy_target_m'], [.8, .1, 0.])
    assert 'Weak contacts on the same patch can remain' in truth['qualification']
    truth['primary_target_m'][0] = 50.
    assert point[0, 0] == 1.


def test_primary_eligibility_enforces_load_count_and_other_patch_boundaries():
    assert primary_truth(loads_N=[1., .05, 0., 0.])['primary_eligible']
    assert not primary_truth(loads_N=[1., .050001, 0., 0.])['primary_eligible']
    assert not primary_truth(loads_N=[.999999, 0., 0., 0.])['primary_eligible']
    assert not primary_truth(loads_N=[10., 1., 0., 0.], contact_counts=[1, 1, 0, 0])['primary_eligible']
    assert not primary_truth(contact_counts=[2, 0, 0, 0])['primary_eligible']
    assert not primary_truth(contact_counts=[0, 1, 0, 0])['primary_eligible']
    for invalid in ([.5, 0., 0., 0.], [-1, 0, 0, 0], [np.nan, 0, 0, 0]):
        with pytest.raises(ValueError):
            primary_truth(contact_counts=invalid)


def test_invalid_patch_and_unloaded_fallback_do_not_become_valid_primary_targets():
    invalid = primary_truth(patch_points_m=np.full((4, 3), np.nan))
    assert not invalid['primary_eligible'] and invalid['primary_target_m'] is None
    assert invalid['legacy_equivalent_eligible']
    unloaded = primary_truth(loads_N=[0., 0., 0., 0.], contact_counts=[0, 0, 0, 0])
    assert not unloaded['primary_eligible'] and not unloaded['legacy_equivalent_eligible']
    assert unloaded['legacy_target_m'] is None and unloaded['category'] == 'unloaded'


def test_nearest_any_candidate_retains_wrong_identity_and_separate_candidate_counts():
    truth = primary_truth()
    preview = dict(contact_prediction_supported=True, ambiguous=True, candidates=[
        dict(patch='belt_B', point_m=[1., 0., 0.]),
        dict(patch='belt_B', point_m=[1.01, 0., 0.])])
    score = diagnostic.score_contact_preview(preview, truth)
    assert score['primary_nearest_candidate_error_m'] == 0.
    assert not score['primary_patch_in_candidates']
    assert score['geometric_candidate_count'] == 2 and score['unique_patch_count'] == 1
    assert score['candidate_ambiguous']
    preview['candidates'].append(dict(patch='shell_A', point_m=[1.2, 0., 0.]))
    again = diagnostic.score_contact_preview(preview, truth)
    assert again['primary_nearest_candidate_error_m'] == 0. and again['primary_patch_in_candidates']


def test_partial_empty_and_unloaded_previews_never_earn_primary_identity_success():
    candidate = dict(patch='shell_A', point_m=[1., 0., 0.])
    partial = diagnostic.score_contact_preview(dict(contact_prediction_supported=False, candidates=[candidate]), primary_truth())
    assert partial['primary_nearest_candidate_error_m'] == 0.  # Retained only for separate partial diagnostics.
    assert not partial['contact_preview_complete'] and not partial['primary_patch_in_candidates']
    empty = diagnostic.score_contact_preview(dict(contact_prediction_supported=True, candidates=[]), primary_truth())
    assert empty['primary_nearest_candidate_error_m'] is None and not empty['primary_patch_in_candidates']
    unloaded = primary_truth(loads_N=[0., 0., 0., 0.], contact_counts=[0, 0, 0, 0])
    score = diagnostic.score_contact_preview(dict(contact_prediction_supported=True, candidates=[candidate]), unloaded)
    assert score['primary_nearest_candidate_error_m'] is None and score['legacy_nearest_candidate_error_m'] is None


def test_common_fourway_matching_does_not_select_on_identity_error_or_ambiguity():
    rows = fixed_slot()
    rows[0]['contact_score']['primary_nearest_candidate_error_m'] = 4.
    rows[1]['contact_score']['primary_patch_in_candidates'] = True
    rows[2]['contact_score']['candidate_ambiguous'] = False
    matched = diagnostic.common_fourway_rows(rows)
    assert matched['query_slots'] == matched['matched_slots'] == 1
    assert matched['groups'][0][0] == (80, .1) and matched['exclusions'] == {}


def test_common_fourway_exclusions_keep_fixed_denominators_and_legacy_separate():
    rows = fixed_slot(1)+fixed_slot(2, primary=False)+fixed_slot(3)+fixed_slot(4)
    rows[8]['contact_score']['contact_preview_complete'] = False
    rows[12]['contact_score']['nonempty_candidates'] = False
    primary = diagnostic.common_fourway_rows(rows)
    assert primary['query_slots'] == 4 and primary['matched_slots'] == 1
    assert primary['exclusions'] == dict(truth_ineligible=1, incomplete_preview=1, empty_candidates=1)
    legacy = diagnostic.common_fourway_rows(rows, target='legacy')
    assert legacy['query_slots'] == 4 and legacy['matched_slots'] == 2


def test_common_fourway_rejects_missing_duplicate_and_arm_dependent_truth():
    for rows in (fixed_slot()[:-1], fixed_slot()+[deepcopy(fixed_slot()[0])]):
        with pytest.raises(ValueError):
            diagnostic.common_fourway_rows(rows)
    inconsistent = fixed_slot()
    inconsistent[-1]['contact_score']['primary_eligible'] = False
    with pytest.raises(ValueError):
        diagnostic.common_fourway_rows(inconsistent)


def test_quality_constraints_pass_exact_registered_boundaries_and_fail_missing_values():
    observer = metric()
    memory = metric(.0045, .0105, .75, .15)
    assert diagnostic.quality_gates(observer, memory, THRESHOLDS)['passed']
    for field, value in dict(rmse_m=.0045001, p95_m=.0105001,
                             identity_recall=.7499999, ambiguity_fraction=.1500001).items():
        changed = {**memory, field:value}
        assert not diagnostic.quality_gates(observer, changed, THRESHOLDS)['passed']
    for value in (None, np.nan):
        assert not diagnostic.quality_gates(observer, {**memory, 'rmse_m':value}, THRESHOLDS)['passed']


def test_insufficient_common_support_is_inconclusive_including_empty_denominator():
    for common, eligible in ((0, 0), (29, 40), (30, 61)):
        verdict = diagnostic.diagnostic_decision(common, eligible, pair(), pair(), .8, .8, THRESHOLDS)
        assert verdict['decision_branch'] == 'inconclusive_insufficient_common_primary'
        if eligible == 0:
            assert verdict['common_primary_fraction'] is None


def test_support_gate_compares_oracle_memory_coverage_to_cv_memory_not_observer():
    boundary = diagnostic.diagnostic_decision(30, 60, pair(), pair(), .75, .70, THRESHOLDS)
    assert all(boundary['support_checks'].values())
    insufficient = diagnostic.diagnostic_decision(30, 60, pair(), pair(), .76, .70, THRESHOLDS)
    assert not insufficient['support_checks']['memory_coverage_loss']
    assert insufficient['decision_branch'] == 'insufficient_useful_advantage_no_integration'


def test_decision_prioritizes_oracle_failure_and_distinguishes_motion_from_aggregation():
    def decide(cv, oracle):
        return diagnostic.diagnostic_decision(40, 60, cv, oracle, .8, .8, THRESHOLDS)['decision_branch']
    assert decide(pair(), pair(metric(.006))) == 'park_current_map_to_contact_integration_oracle_memory_worse'
    assert decide(pair(metric(.005)), pair()) == 'investigate_motion_prediction_keep_LQR_unchanged'
    assert decide(pair(), pair()) == 'audit_metric_aggregation_and_stratification'
    assert decide(pair(), pair(metric(.004, .02))) == 'insufficient_useful_advantage_no_integration'
