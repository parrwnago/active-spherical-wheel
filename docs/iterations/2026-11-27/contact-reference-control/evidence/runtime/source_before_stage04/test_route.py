import unittest
import numpy as np
from route import SmoothSPath


class RouteContracts(unittest.TestCase):
    def test_exact_stationary_holds_and_common_origin(self):
        path = SmoothSPath()
        for t in (-5., 0., 3.):
            p, v, a = path(t)
            np.testing.assert_array_equal(p, [-1.79, 7.22])
            np.testing.assert_array_equal(v, [0., 0.])
            np.testing.assert_array_equal(a, [0., 0.])
        endpoint = path.origin + path.rotation @ [1.2, 0.]
        for t in (35., 45., 80.):
            p, v, a = path(t)
            np.testing.assert_array_equal(p, endpoint)
            np.testing.assert_array_equal(v, [0., 0.])
            np.testing.assert_array_equal(a, [0., 0.])

    def test_analytic_derivatives(self):
        path, dt = SmoothSPath(), 1e-3
        for t in (5., 11., 19., 29., 33.):
            p, v, a = path(t)
            before, after = path(t-dt), path(t+dt)
            np.testing.assert_allclose((after[0]-before[0])/(2*dt), v, atol=3e-9, rtol=3e-6)
            np.testing.assert_allclose((after[1]-before[1])/(2*dt), a, atol=3e-9, rtol=3e-6)

    def test_c2_boundary_limits(self):
        path = SmoothSPath()
        for t in (3.+1e-5, 35.-1e-5):
            _, v, a = path(t)
            self.assertLess(np.linalg.norm(v), 1e-10)
            self.assertLess(np.linalg.norm(a), 1e-7)

    def test_no_history_or_mutation(self):
        path = SmoothSPath()
        first = tuple(x.copy() for x in path(11.))
        path(29.)
        for a, b in zip(first, path(11.)):
            np.testing.assert_array_equal(a, b)
        self.assertEqual(path.total_duration_s, 45.)


if __name__ == "__main__":
    unittest.main()
