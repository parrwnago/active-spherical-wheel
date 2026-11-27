"""Independent graph/evidence closed-form contracts; no saved outputs."""
from pathlib import Path
import sys
import math
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from verify import prior, transition, first_step, history_step, observation_matrix


class IndependentContracts(unittest.TestCase):
    def test_closed_form_retention_and_jump_odds(self):
        p = prior(); t = transition(.005)
        np.testing.assert_allclose(t.sum(axis=1), 1., atol=2e-16, rtol=0)
        self.assertEqual(t[0, 0], math.exp(-.005/.05))
        self.assertEqual(t[8, 8], math.exp(-.005/.02))
        self.assertAlmostEqual(t[0, 1]/t[0, 2], 4*p[1]/p[2], places=14)
        self.assertAlmostEqual(t[0, 8]/t[0, 2], 2*p[8]/p[2], places=14)
        self.assertAlmostEqual(t[8, 0]/t[8, 1], p[0]/p[1], places=14)

    def test_equal_evidence_prior_then_markov_prediction_and_unknown(self):
        cost = np.zeros(21)
        first = first_step(cost)['probability'][[0,1,2,3,4,5,6,7,20]]
        np.testing.assert_equal(first, prior())
        p, _, _ = history_step(first, cost, None, .005, initial=False)
        expected = prior() @ transition(.005); expected /= expected.sum()
        np.testing.assert_allclose(p, expected, atol=1e-16, rtol=0)
        only_unknown = np.full(21, np.inf); only_unknown[20] = 0.
        selected = first_step(only_unknown)
        self.assertEqual(selected['selected_current_mode'], 20)
        self.assertEqual(selected['support_index'], 10)

    def test_directed_edge_likelihood_ratio_and_unavailable_fallback(self):
        node = np.zeros(21); node[2] = 8.
        edges = np.full((6,2,2), np.inf); edges[0,1,0] = 0.
        costs = observation_matrix(node, edges)
        self.assertEqual(costs[0,2], 0.)
        self.assertEqual(costs[1,2], 0.)
        self.assertEqual(costs[2,0], node[0])
        self.assertEqual(observation_matrix(node, None)[0,2], 8.)
        old = np.zeros(9); old[0] = 1.
        actual, _, _ = history_step(old, node, edges, .005, initial=False)
        noedge, _, _ = history_step(old, node, None, .005, initial=False)
        self.assertAlmostEqual((actual[2]/actual[0])/(noedge[2]/noedge[0]), math.exp(2.), places=13)


if __name__ == '__main__': unittest.main()
