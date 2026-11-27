"""Synthetic contracts for post-result explanatory calculations only."""
from pathlib import Path
import importlib.util
import unittest
import numpy as np

s=importlib.util.spec_from_file_location('posthoc_diagnose',Path(__file__).with_name('diagnose.py'))
d=importlib.util.module_from_spec(s);s.loader.exec_module(d)


class DiagnosticContracts(unittest.TestCase):
    def test_truth_cardinality_does_not_assume_pairs(self):
        np.testing.assert_array_equal(d.cardinality(np.array([0,1,3,7,15])),[0,1,2,3,4])

    def test_paired_gain_loss_uses_identical_denominator(self):
        got=d.paired(np.array([1,1,0,0],bool),np.array([1,0,1,0],bool),np.ones(4,bool))
        self.assertEqual(got,dict(denominator=4,both_correct=1,both_wrong=1,gained=1,lost=1,old_errors=2,new_errors=2))

    def test_error_spells_split_truth_changes_and_time_gaps(self):
        rows,ids=d.spells('synthetic','hmm',np.array([1.,1.005,1.01,1.02]),np.full(4,2),np.full(4,3),
                          np.array([0,0,1,1]),np.array([1,1,2,2]),np.full(4,.9))
        self.assertEqual([r['rows'] for r in rows],[2,1,1])
        self.assertEqual([r['sampled_duration_s'] for r in rows],[.01,.005,.005])
        np.testing.assert_array_equal(ids,[0,0,1,2])

    def test_unknown_pair_and_wrong_singleton_remain_distinct(self):
        got=d.error_type(np.array([0,3,2,1,1]),np.array([0,0,0,1,0],bool),np.array([1,1,1,1,0],bool))
        np.testing.assert_array_equal(got,[1,2,3,0,-1])


if __name__=='__main__':unittest.main(verbosity=2)
