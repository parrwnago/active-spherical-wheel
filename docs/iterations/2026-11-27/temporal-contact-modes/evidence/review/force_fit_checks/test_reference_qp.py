"""Synthetic physical-constraint/reference-optimizer contracts only."""
import unittest
import numpy as np
from verify_force_fits import physical_constraints,reference_solve


class PhysicalForceReference(unittest.TestCase):
    def test_inscribed_square_is_distinct_from_a_circular_cone(self):
        c=physical_constraints([[0.,0.,1.]])
        corner=np.array([.7/np.sqrt(2)*10,.7/np.sqrt(2)*10,10.])
        self.assertGreaterEqual(float((c@corner).min()),-1e-12)
        self.assertLess(float((c@np.array([7.,0.,10.])).min()),0.)

    def test_share_inequalities_exclude_ghost_pair(self):
        c=physical_constraints([[0.,0.,1.],[0.,0.,1.]])
        self.assertGreaterEqual(float((c@np.array([0,0,2,0,0,18])).min()),-1e-12)
        self.assertLess(float((c@np.array([0,0,1,0,0,19])).min()),0.)

    def test_independent_cartesian_qp_recovers_a_feasible_known_force(self):
        maps=np.zeros((1,9,3));maps[0,:3]=np.eye(3)
        force=np.array([1.,-.5,10.]);y=maps[0]@force
        solved,objective,c=reference_solve(y,maps,[[0.,0.,1.]],np.ones(9))
        self.assertTrue(solved.success)
        np.testing.assert_allclose(solved.x,force,atol=1e-8)
        self.assertLess(objective(solved.x),1e-15)
        self.assertGreaterEqual(float((c@solved.x).min()),-1e-10)


if __name__=='__main__':unittest.main(verbosity=2)
