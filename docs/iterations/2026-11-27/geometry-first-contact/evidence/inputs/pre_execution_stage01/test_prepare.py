"""Synthetic byte-copy, sparse-point and fallback-boundary contracts."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from prepare import project, native_family
from packet_io import PROJECTED_KEYS, decode_rank


def fixture():
    n, c = 2, 4
    features = dict(excitation_m_s=np.array([.1, .2]), family_index=np.array([2, 0, 3, 1], np.int8),
        frame_valid=np.ones(n, bool), moment_residual_norm=np.zeros((n, c)),
        normal_angle_rad=np.zeros((n, c)), valid=np.array([[True, False, True, True], [False, True, True, False]]),
        velocity_residual_m_s=np.zeros((n, c, 3)), velocity_residual_norm_m_s=np.zeros((n, c)), kind=np.zeros(c, np.int8))
    points = np.arange(n*c*3, dtype=float).reshape(n, c, 3)/100
    context = dict(time_s=np.array([0., .005]), exact_estimated_contact_point_m=np.array([[0., 0., .05], [.01, .01, .04]]),
        exact_estimated_support_loads_N=np.array([[0., 2., 0., 0.], [0., 0., 3., 0.]]))
    layout = dict(candidate_index=np.array([42, 9, 77, 11]), family_index=features['family_index'], kind=features['kind'])
    return features, points, context, layout


class ProjectionContracts(unittest.TestCase):
    def test_exact_projected_anchor_and_explicit_id_order(self):
        f, points, context, layout = fixture(); rank, fallback, _ = project(f, points, context, layout)
        decoded = decode_rank(rank)
        for key in PROJECTED_KEYS: self.assertEqual(decoded['projected_features'][key].tobytes(), f[key].tobytes())
        np.testing.assert_equal(decoded['candidate_ids'], [42, 9, 77, 11])
        np.testing.assert_equal(decoded['native_anchor_point_m'], context['exact_estimated_contact_point_m'])
        self.assertEqual(decoded['native_anchor_point_m'][0, 2], .05)  # interior N is not normalized.
        self.assertNotIn('native_family', rank)
        self.assertNotIn('native_support_loads_N', rank)

    def test_sparse_original_valid_points_exact_invalid_nan(self):
        f, points, context, layout = fixture(); rank, _, _ = project(f, points, context, layout)
        decoded = decode_rank(rank); mask = f['valid']
        self.assertEqual(decoded['world_points'][mask].tobytes(), points[mask].tobytes())
        self.assertTrue(np.isnan(decoded['world_points'][~mask]).all())
        rank['point_flat_index'] = rank['point_flat_index'][:-1]
        with self.assertRaisesRegex(ValueError, 'exactly the original valid'): decode_rank(rank)

    def test_missing_valid_point_errors_without_dropping_candidate(self):
        f, points, context, layout = fixture(); points[0, 0] = np.nan
        with self.assertRaisesRegex(ValueError, 'never drop'): project(f, points, context, layout)
        f, points, context, layout = fixture(); points[0, 1] = np.nan  # already invalid
        project(f, points, context, layout)

    def test_fallback_load_changes_never_change_rank_packet(self):
        f, points, context, layout = fixture(); original, _, _ = project(f, points, context, layout)
        context['exact_estimated_support_loads_N'] = np.array([[7., 0, 0, 0], [0, 0, 0, 9.]])
        changed, fallback, _ = project(f, points, context, layout)
        for key in original: self.assertEqual(original[key].tobytes(), changed[key].tobytes())
        np.testing.assert_equal(fallback['native_family'], [0, 3])
        loads = np.array([[1., 1, 0, 0], [0, 0, 0, 0], [np.nan, 2, 0, 0], [-1, 2, 0, 0], [0, 3, 0, 0]])
        np.testing.assert_equal(native_family(loads), [-1, -1, -1, -1, 1])


if __name__ == '__main__': unittest.main()
