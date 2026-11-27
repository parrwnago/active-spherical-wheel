"""Synthetic contracts for the captured original-order checks."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from captured_lcp_checks import check_original_problem


class CapturedContracts(unittest.TestCase):
    def call(self, x, **changes):
        a = np.zeros((3, 4)); a[:, :3] = [[1, 1.5, 0], [1.5, 4, 0], [0, 0, 2]]
        # Padding is deliberately nonfinite: it is not part of the matrix.
        a[:, 3] = np.nan
        args = dict(n=3, stride=4, A_before=a, b_before=[1, 4, 0],
                    lo_before=[0, -.7, -.7], hi_before=[np.inf, .7, .7],
                    findex_before=[-1, 0, 0], x_after=x)
        args.update(changes)
        return check_original_problem(**args)

    def test_final_normal_not_initial_or_postmutated_box(self):
        r = self.call([0, .7, 0])
        self.assertEqual(r['friction_max_bound_violation'], .7)
        np.testing.assert_equal(r['friction_bound_violating_indices'], [1])
        np.testing.assert_equal(r['original_constant_box_violation'], 0)
        np.testing.assert_equal(r['final_dependent_hi'][1:], 0)

    def test_coupled_reference_and_zero_width_friction(self):
        r = self.call([1/2.05, .7/2.05, 0])
        self.assertLess(r['max_abs_unit_step_projected_kkt_residual'], 1e-14)
        self.assertEqual(r['friction_max_bound_violation'], 0)
        r = self.call([0, 0, 0])
        self.assertEqual(r['unit_step_projected_kkt_residual'][1], 0)
        self.assertEqual(r['unit_step_projected_kkt_residual'][0], -1)

    def test_nonfinite_solution_is_unknown_not_success(self):
        r = self.call([np.nan, 0, 0])
        self.assertFalse(r['valid_final_vector'])

    def test_original_indices_and_layout_are_required(self):
        with self.assertRaises(ValueError):
            self.call([0, 0, 0], findex_before=[-1, 5, 0])
        with self.assertRaises(ValueError):
            self.call([0, 0, 0], stride=3)


if __name__ == '__main__':
    unittest.main()
