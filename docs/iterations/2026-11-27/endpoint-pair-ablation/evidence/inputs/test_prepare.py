"""Synthetic complete-fit point lookup tests; no saved inference data."""
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from prepare import project, PAIR_SETS, EVIDENCE_KEYS


def fixture():
    # IDs deliberately shuffled and unrelated to storage offsets.
    ids = np.array([42, 9, 77, 11], np.int64); families = np.array([2, 0, 3, 1])
    layout = dict(candidate_index=ids, family_index=families, kind=np.zeros(4, np.int8))
    by_family = {int(f): int(ids[i]) for i, f in enumerate(families)}
    points = np.arange(24, dtype=float).reshape(2, 4, 3)/10
    ev = dict(node_cost=np.arange(42, dtype=float).reshape(2, 21),
        edge_cost=np.arange(48, dtype=float).reshape(2, 6, 2, 2),
        node_valid=np.zeros((2, 21), bool), edge_valid=np.ones((2, 6, 2, 2), bool),
        node_candidate_index=np.full((2, 21, 2), -1, np.int64),
        edge_candidate_index=np.full((2, 6, 2, 2, 2), -1, np.int64))
    for mode in range(8):
        ev['node_valid'][:, mode] = True
        ev['node_candidate_index'][:, mode, 0] = by_family[mode//2]
    ev['node_valid'][:, 20] = True
    for pair, fs in enumerate(PAIR_SETS):
        ev['edge_candidate_index'][:, pair, :, :, :] = [by_family[int(f)] for f in fs]
    return np.array([0., .005]), ev, points, np.ones((2, 4), bool), layout


class ProjectionContracts(unittest.TestCase):
    def test_explicit_id_mapping_and_current_role_from_same_fit(self):
        args = fixture(); out, _ = project(*args)
        np.testing.assert_equal(out['node_endpoint_points_m'][:, 0], args[2][:, 1])
        # Pair(0,1), endpoint role1 is family1: storage slot3, not role0'sslot1.
        np.testing.assert_equal(out['edge_endpoint_points_m'][:, 0, 1, 0], args[2][:, 3])
        self.assertEqual(out['edge_endpoint_candidate_index'][0, 0, 1, 0], 11)

    def test_cost_ids_and_validity_bytes_unchanged(self):
        args = fixture(); out, parity = project(*args)
        for key in EVIDENCE_KEYS:
            self.assertEqual(out[key].tobytes(), args[1][key].tobytes())
            self.assertEqual(out[key].dtype, args[1][key].dtype)
        self.assertTrue(parity['physical_point_selection_exact'])

    def test_pair_unknown_and_invalid_fit_points_remain_nan(self):
        args = fixture(); args[1]['node_valid'][:, 8] = True
        args[1]['node_candidate_index'][:, 8] = [9, 11]
        args[1]['edge_valid'][0, 0, 1, 0] = False
        out, _ = project(*args)
        self.assertTrue(np.isnan(out['node_endpoint_points_m'][:, 8:]).all())
        self.assertTrue(np.isnan(out['edge_endpoint_points_m'][0, 0, 1, 0]).all())
        self.assertFalse(out['edge_endpoint_valid'][0, 0, 1, 0])

    def test_bad_family_missing_id_or_invalid_original_candidate_rejected(self):
        args = fixture(); args[1]['node_candidate_index'][0, 0, 0] = 42
        with self.assertRaisesRegex(ValueError, 'wrong endpoint family'): project(*args)
        args = fixture(); args[1]['node_candidate_index'][0, 0, 0] = 999
        with self.assertRaisesRegex(ValueError, 'absent'): project(*args)
        args = fixture(); args[3][0, 1] = False
        with self.assertRaisesRegex(ValueError, 'original valid'): project(*args)


if __name__ == '__main__': unittest.main()
