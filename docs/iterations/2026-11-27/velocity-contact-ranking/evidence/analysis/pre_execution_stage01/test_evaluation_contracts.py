"""Independent small denominator/window contracts; no historical data."""
import unittest
import numpy as np

import evaluate_scores as audit


def frames_fixture():
    truth=np.array([0,0,1,1]);native=np.array([0,0,1,1])
    valid=np.array([[False]*4,[True,False,False,False],
                    [True,True,False,False],[True,True,False,False]])
    result=dict(time_s=1.+.005*np.arange(4),truth_family=truth,native_family=native,
        truth_eligible=np.ones(4,bool),truth_loaded=np.ones(4,bool),
        candidate_family_valid=valid,valid_family_count=valid.sum(axis=1),
        measured_excitation_m_s=np.full(4,.1),truth_patch_contact_counts=np.ones((4,4),int))
    for mode in audit.MODES:
        result[mode+'__chosen_family']=np.array([-1,0,0,1])
        result[mode+'__accepted_family']=np.array([-1,-1,0,1])
        result[mode+'__family_scores']=np.where(valid,np.array([0.,1.,2.,3.]),np.inf)
    return result


class EvaluationContracts(unittest.TestCase):
    def test_truth_dominance_ties_and_load_floor(self):
        labels,eligible,loaded,share,unique,total=audit.dominant_labels(
            np.array([[9.,1.,0.,0.],[1.,1.,0.,0.],[.1,0.,0.,0.],
                      [np.nan,1.,0.,0.],[0.,0.,0.,0.]]),1.,.9)
        np.testing.assert_array_equal(labels,[0,-1,-1,-1,-1])
        np.testing.assert_array_equal(eligible,[True,False,False,False,False])
        self.assertTrue(loaded[1]);self.assertFalse(unique[1])

    def test_half_open_primary_and_terminal(self):
        time=.005*np.arange(8801)
        self.assertEqual(int(audit.mask_window(time,[1.,41.]).sum()),8000)
        self.assertEqual(int(audit.mask_window(time,[0.,44.]).sum()),8800)
        self.assertFalse(audit.mask_window(time,[0.,44.])[-1])

    def test_missing_candidate_is_error_and_native_subset_is_matched(self):
        f=frames_fixture();r=audit.metrics_for_window(f,np.ones(4,bool))
        self.assertEqual(r['counts']['dominant_eligible'],4)
        self.assertEqual(r['counts']['candidate_no_family'],1)
        self.assertEqual(r['modes']['fused']['raw']['error_count'],2)
        self.assertEqual(r['modes']['fused']['accepted']['error_fraction'],.5)
        self.assertEqual(r['modes']['fused']['native_on_same_accepted_subset']['count'],2)
        self.assertEqual(r['modes']['fused']['native_on_same_accepted_subset']['error_count'],0)
        self.assertEqual(r['modes']['fused']['accepted_coverage_ambiguous_motion'],1.)
        self.assertEqual(r['modes']['fused']['geometry_forced']['count'],1)

    def test_false_switch_ignores_true_family_change_but_counts_unknown(self):
        f=frames_fixture();mask=np.ones(4,bool)
        self.assertEqual(audit.count_switches(f['fused__chosen_family'],f['truth_family'],
                                             f['truth_eligible'],mask),2)
        self.assertEqual(audit.count_switches(f['native_family'],f['truth_family'],
                                             f['truth_eligible'],mask),0)

    def test_stable_truth_uses_both_sides(self):
        truth=np.r_[np.zeros(6,int),np.ones(6,int)]
        eligible=np.ones(12,bool)
        stable=audit.stable_mask(truth,eligible,steps=2)
        np.testing.assert_array_equal(np.flatnonzero(stable),[2,3,8,9])

    def test_timing_retains_ambiguous_gap_and_missing_onset(self):
        time=.005*np.arange(200,240)
        truth=np.r_[np.zeros(10,int),np.full(3,-1),np.ones(27,int)]
        eligible=truth>=0
        predicted={'native':truth.copy(),'missing':np.zeros(len(time),int)}
        events=audit.timing_events(time,truth,eligible,predicted)
        self.assertEqual(len(events),1)
        self.assertAlmostEqual(events[0]['unknown_or_ambiguous_gap_s'],.015)
        self.assertEqual(events[0]['methods']['native']['status'],'same_sample')
        self.assertEqual(events[0]['methods']['missing']['status'],'missing')


if __name__=='__main__':
    unittest.main()
