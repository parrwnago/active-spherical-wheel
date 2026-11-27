"""Synthetic scientific contracts; no historical arrays, labels or physics."""
from pathlib import Path
import importlib.util
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location('evaluate_temporal', Path(__file__).with_name('evaluate_temporal.py'))
ev = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ev)


def fixture(n=81):
    time = 1. + np.arange(n)*.005
    loads = np.tile([10., 0., 0., 0.], (n, 1))
    points = np.zeros((n, 4, 3))
    packet = dict(time_s=time, truth_patch_loads_N=loads, truth_patch_points_m=points)
    for name in ev.METHODS:
        packet[name+'_points_m'] = points.copy()
    for name in ('N', 'F', 'A'):
        packet[name+'_family'] = np.zeros(n, np.int8)
    probability = np.zeros((n, 21)); probability[:, 0] = 1.
    return packet, dict(time_s=time.copy(), emission_probability21=probability.copy(), posterior21=probability.copy())


class TemporalEvaluationContracts(unittest.TestCase):
    def test_collapse_before_map_not_map_before_collapse(self):
        p = np.zeros((1, 21)); p[0, 0:2] = .26; p[0, 2] = .45; p[0, 20] = .03
        bits, confidence = ev.decode(ev.collapse(p))
        self.assertEqual(int(bits[0]), 1)
        self.assertEqual(int(np.argmax(p)), 2)
        self.assertAlmostEqual(float(confidence[0]), .52)

    def test_pressure_dominance_and_multiset_differ_at_ten_percent(self):
        truth = ev.truth_labels([[9., 1., 0., 0.], [.5, .5, 0., 0.], [.01, 0., 0., 0.]])
        np.testing.assert_array_equal(truth['truth_family'], [0, -1, -1])
        np.testing.assert_array_equal(truth['truth_support_bits'], [3, 3, 0])
        np.testing.assert_array_equal(truth['truth_loaded'], [True, True, False])

    def test_negative_or_nonfinite_pressure_never_labels(self):
        truth = ev.truth_labels([[10., -1., 0., 0.], [10., np.nan, 0., 0.]])
        self.assertFalse(truth['truth_loaded'].any())

    def test_three_family_truth_stays_in_loaded_denominator(self):
        truth = ev.truth_labels([[8., 1., 1., 0.]])
        metrics, _ = ev.method_metrics(np.array([1]), ev.onehot(np.array([1])), truth, np.array([True]))
        self.assertEqual(metrics['multiset']['denominator'], 1)
        self.assertEqual(metrics['multiset']['errors'], 1)
        self.assertEqual(metrics['multiset']['unsupported_truth_count'], 1)
        self.assertEqual(metrics['calibration']['representable_count'], 0)

    def test_pairs_unknowns_are_singleton_errors_and_unknown_not_coverage(self):
        truth = ev.truth_labels([[10., 0., 0., 0.]]*3)
        bits = np.array([0, 3, 1])
        metrics, _ = ev.method_metrics(bits, ev.onehot(bits), truth, np.ones(3, bool))
        self.assertEqual(metrics['singleton']['errors'], 2)
        self.assertEqual(metrics['singleton']['pair_output'], 1)
        self.assertEqual(metrics['singleton']['unknown'], 1)
        self.assertEqual(metrics['confidence']['covered'], 2)
        self.assertEqual(metrics['confidence']['wrong'], 1)
        self.assertAlmostEqual(metrics['calibration']['set_brier_mean'], 4./3.)

    def test_unknown_posterior_cannot_obtain_perfect_selective_accuracy(self):
        packet, pred = fixture()
        pred['posterior21'][:] = 0.; pred['posterior21'][:, 20] = 1.
        frames, report = ev.evaluate_case(packet, pred)
        self.assertEqual(report['methods']['hmm']['singleton']['errors'], len(packet['time_s']))
        self.assertEqual(report['methods']['hmm']['confidence']['covered'], 0)
        self.assertEqual(report['methods']['hmm']['calibration']['set_brier_mean'], 2.)
        self.assertTrue(np.isnan(frames['hmm__point_distance_mm']).all())

    def test_brier_and_reliability_do_not_claim_calibration(self):
        truth = ev.truth_labels([[10., 0., 0., 0.]]*2)
        metrics, arrays = ev.method_metrics(np.array([1, 2]), ev.onehot(np.array([1, 2])), truth, np.ones(2, bool))
        np.testing.assert_array_equal(arrays['set_brier'], [0., 2.])
        self.assertEqual(metrics['calibration']['set_brier_mean'], 1.)
        self.assertEqual(metrics['calibration']['support_reliability']['ece'], .5)
        self.assertEqual(metrics['calibration']['family_inclusion_brier_mean'], .25)

    def test_invalid_probability_is_rejected_not_normalized(self):
        for bad in (np.full((1, 21), .1), np.full((1, 21), np.nan)):
            with self.assertRaises(ValueError):
                ev.collapse(bad)

    def test_half_open_primary_window(self):
        np.testing.assert_array_equal(ev.primary_window(np.array([.995, 1., 40.995, 41., 44.])),
                                      [False, True, True, False, False])

    def test_signed_latency_early_and_late(self):
        time = 1.+np.arange(81)*.005
        truth = np.r_[np.zeros(40, int), np.ones(41, int)]
        early = ev.family_bits(np.r_[np.zeros(37, int), np.ones(44, int)])
        late = ev.family_bits(np.r_[np.zeros(44, int), np.ones(37, int)])
        result = ev.timing(time, truth, np.ones(81, bool), dict(early=early, late=late))
        self.assertEqual(len(result['events']), 1)
        self.assertAlmostEqual(result['events'][0]['methods']['early']['signed_latency_s'], -.015)
        self.assertAlmostEqual(result['events'][0]['methods']['late']['signed_latency_s'], .020)

    def test_short_new_family_attempt_is_reverted_not_success(self):
        time = 1.+np.arange(81)*.005
        truth = np.r_[np.zeros(40, int), np.ones(41, int)]
        prediction = np.zeros(81, int); prediction[39] = 1
        result = ev.timing(time, truth, np.ones(81, bool), dict(hmm=ev.family_bits(prediction)))
        self.assertEqual(result['methods']['hmm']['missed'], 1)
        self.assertEqual(result['methods']['hmm']['reverted_attempts'], 1)
        self.assertIsNone(result['methods']['hmm']['absolute_latency_s']['median'])

    def test_false_switches_report_unknown_and_pair_separately(self):
        time = 1.+np.arange(81)*.005
        bits = np.ones(81, int); bits[10] = 0; bits[20] = 3; bits[30] = 2
        result = ev.timing(time, np.zeros(81, int), np.ones(81, bool), dict(hmm=bits))
        report = result['methods']['hmm']
        self.assertEqual(report['false_switches'], 6)
        self.assertEqual(report['false_unknown_transitions'], 2)
        self.assertEqual(report['false_pair_transitions'], 2)
        self.assertEqual(report['false_singleton_toggles'], 2)
        self.assertEqual(report['events'], 0)

    def test_short_truth_run_does_not_create_timing_event(self):
        time = 1.+np.arange(81)*.005
        truth = np.zeros(81, int); truth[40:45] = 1
        result = ev.timing(time, truth, np.ones(81, bool), dict(hmm=ev.family_bits(truth)))
        self.assertEqual(len(result['events']), 0)

    def test_long_unobserved_gap_does_not_create_transition(self):
        time = 1.+np.arange(100)*.005
        truth = np.r_[np.zeros(30, int), np.full(40, -1), np.ones(30, int)]
        result = ev.timing(time, truth, truth >= 0, dict(hmm=ev.family_bits(truth)))
        self.assertEqual(len(result['events']), 0)

    def test_wrong_family_point_is_not_excluded_from_all_singletons(self):
        packet, pred = fixture()
        packet['F_family'][:] = 1
        packet['F_points_m'][:, 1, 0] = .1
        frames, report = ev.evaluate_case(packet, pred)
        f = report['points']['methods']['F']
        self.assertEqual(f['all_scorable_singletons_mm']['maximum'], 100.)
        self.assertEqual(f['correct_family_only_mm']['count'], 0)
        self.assertEqual(report['points']['all_methods_correct_family_matched_count'], 0)
        self.assertEqual(report['methods']['F']['singleton']['errors'], len(packet['time_s']))

    def test_missing_prediction_never_shrinks_primary_denominator(self):
        packet, pred = fixture()
        packet['N_family'][:] = -1
        packet['N_points_m'][:] = np.nan
        frames, report = ev.evaluate_case(packet, pred)
        self.assertEqual(report['methods']['N']['singleton']['denominator'], 81)
        self.assertEqual(report['methods']['N']['singleton']['errors'], 81)
        self.assertEqual(report['points']['methods']['N']['missing'], 81)

    def test_no_timing_events_is_unscorable_not_a_latency_pass(self):
        packet, pred = fixture()
        frames, report = ev.evaluate_case(packet, pred)
        cases = {c: report for c in ev.CASES}
        pooled = ev.aggregate(cases, {c: frames for c in ev.CASES})
        gates = {row['name']: row for row in ev.gates(pooled, cases)}
        self.assertFalse(gates['transition_absolute_latency_median']['passed'])
        self.assertFalse(gates['transition_absolute_latency_p95']['passed'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
