"""Closed-form independent verifier contracts; no saved outputs."""
from pathlib import Path
import sys
import math
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from verify import frame, fallback_family, NORMAL_SCALE


def fixture():
    f = dict(family_index=np.array([0, 0, 1, 1]), kind=np.zeros(4, int), valid=np.ones(4, bool),
        frame_valid=True, excitation_m_s=.1, normal_angle_rad=np.array([0., 0., .1, 0.]),
        moment_residual_norm=np.zeros(4), velocity_residual_m_s=np.array([[0.,0,0],[.03,0,0],[.001,0,0],[0.,0,0]]),
        velocity_residual_norm_m_s=np.array([0., .03, .001, 0.]))
    return f, np.array([[.01,0,0],[-.01,0,0],[.02,0,0],[.04,0,0]]), np.zeros(3), np.array([9,3,6,2])


class IndependentContracts(unittest.TestCase):
    def test_exact_geometry_id_tie_and_complete_candidate_score(self):
        f, p, a, ids = fixture(); out, _ = frame(f, p, a, ids)
        np.testing.assert_equal(out['selected_candidate_id'], [3, 6, -1, -1])
        self.assertEqual(out['fused__chosen_family'], 1)
        self.assertAlmostEqual(out['fused__family_scores'][1], (.1/NORMAL_SCALE)**2+(.001/.02)**2, places=14)
        self.assertTrue(out['fused__accepted'])
        np.testing.assert_equal(out['fused__point_m'], [.02,0,0])

    def test_low_excitation_does_not_change_raw_active_selection(self):
        f, p, a, ids = fixture(); f['excitation_m_s'] = .001
        out, _ = frame(f, p, a, ids)
        self.assertTrue(out['legacy_active_mask'].all())
        self.assertEqual(out['fused__chosen_family'], 1)
        self.assertFalse(out['fused__accepted'])
        out, _ = frame(f, p, np.full(3, np.nan), ids)
        self.assertFalse(out['selected_mask'].any()); self.assertEqual(out['fused__chosen_family'], -1)
        self.assertTrue(np.isnan(out['family_distance_squared_m2']).all())
        p[0] = np.nan
        with self.assertRaisesRegex(ValueError, 'missing active'): frame(f, p, a, ids)

    def test_exact_margin_and_velocity_vector_separation_gates(self):
        f = dict(family_index=np.array([0,1]),kind=np.zeros(2,int),valid=np.ones(2,bool),frame_valid=True,
            excitation_m_s=.02,normal_angle_rad=np.array([0., NORMAL_SCALE*.5]),moment_residual_norm=np.zeros(2),
            velocity_residual_m_s=np.array([[0.,0,0],[.01,0,0]]),velocity_residual_norm_m_s=np.array([0.,.01]))
        out, _ = frame(f, np.array([[0.,0,0],[.01,0,0]]), np.zeros(3), np.array([11,4]))
        self.assertEqual(out['geometry__score_margin'], .25); self.assertTrue(out['geometry__accepted'])
        f['velocity_residual_m_s'][1,0] = .009; f['velocity_residual_norm_m_s'][1] = .009
        out, _ = frame(f, np.array([[0.,0,0],[.01,0,0]]), np.zeros(3), np.array([11,4]))
        self.assertFalse(out['geometry__accepted'])

    def test_native_fallback_ties_invalid_and_unique_positive_rule(self):
        values = np.array([[0.,2,0,0],[1.,1,0,0],[0,0,0,0],[np.nan,2,0,0],[-1.,2,0,0]])
        np.testing.assert_equal(fallback_family(values), [1,-1,-1,-1,-1])


if __name__ == '__main__': unittest.main()
