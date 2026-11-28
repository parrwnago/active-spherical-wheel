"""I86 contracts for physical-interval alignment and force bookkeeping."""
import numpy as np
from core import force_sets, fixed_basis, window_indices, sampled_balance


def test_signed_force_keeps_rejected_and_body_rows():
    c = np.zeros((4, 16)); c[:, 0] = [0, 2, 4, 6]
    c[:, 6] = 1.; c[:, 7:10] = [[1,0,3], [2,0,-2], [4,0,5], [8,0,9]]
    c[:, 12] = [1,1,0,1]
    got = force_sets(c)
    np.testing.assert_array_equal(got['all'], [15,0,15])
    np.testing.assert_array_equal(got['admitted'], [11,0,10])
    np.testing.assert_array_equal(got['native'], [9,0,12])
    np.testing.assert_array_equal(got['family'][0], [5,0,8])
    np.testing.assert_array_equal(got['body'], [8,0,9])


def test_fixed_basis_is_orthonormal_and_downhill():
    basis = fixed_basis([.6,0,.8], [0,0,-10])
    np.testing.assert_allclose(basis[0], [.8,0,-.6])
    np.testing.assert_allclose(basis@basis.T, np.eye(3), atol=1e-15)


def test_completed_interval_window_excludes_start_and_includes_end():
    t = np.array([1., 1.001, 1.002, 1.003])
    np.testing.assert_array_equal(window_indices(t, 1., 1.002), [1,2])


def test_external_force_closes_whole_COM_momentum_not_root_acceleration():
    force = np.array([[1,2,20], [3,4,20.]])
    external = np.array([[1,0,0,0,0,0], [0,0,0,0,0,0.]])
    before = np.zeros((2,3)); after = np.array([[.001,.001,0], [.0015,.002,0]])
    net, required, residual = sampled_balance(force, external, [0,0,-10], 2., before, after, .001)
    np.testing.assert_allclose(net, [[2,2,0], [3,4,0]])
    np.testing.assert_allclose(required, net)
    np.testing.assert_allclose(residual, 0, atol=1e-12)
