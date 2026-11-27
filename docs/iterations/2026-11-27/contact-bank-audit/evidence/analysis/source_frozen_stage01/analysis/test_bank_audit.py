"""Focused synthetic I67 contracts; no historical bank scoring or data reads."""
from pathlib import Path
import copy,importlib.util,unittest
import numpy as np

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_i67_bank_audit',HERE/'bank_audit.py')
B=importlib.util.module_from_spec(spec);spec.loader.exec_module(B)


def active_packet(n=2,c=6):
    return dict(old__normal_angle_rad=np.zeros((n,c)),old__velocity_residual_norm_m_s=np.zeros((n,c)),
        old__moment_residual_norm=np.zeros((n,c)),old__velocity_residual_m_s=np.zeros((n,c,3)),
        old__valid=np.ones((n,c),bool),old__frame_valid=np.ones(n,bool),old__excitation_m_s=np.zeros(n))


def toy_analysis():
    """One frame/five slots: selected nearest point differs from lowest score."""
    p=active_packet(1,5);family=np.array([0,0,1,2,3]);points=np.array([[[.6,0.,.8],[0.,0.,1.],[1.,0.,0.],[0.,1.,0.],[1.,1.,0.]]])
    p['old__normal_angle_rad'][0,1]=.05
    components,cost=B.original_costs(p)
    p.update(time_s=np.array([1.]),source_index=np.array([200]),old__family_index=family,
        old_fused__candidate_index=np.array([[0,2,3,4]]),old_fused__family_scores=cost[:,[0,2,3,4]],
        sampled_reference_point_m=points[:,0].copy(),truth_family=np.array([0]),correct_branch=np.array([0]),
        saved_F_index=np.array([0]),saved_Nalt_index=np.array([2]),saved_point_F_m=points[:,0].copy(),saved_point_Nalt_m=points[:,2].copy(),
        cohort_useful=np.array([True]),cohort_harmful=np.array([False]),cohort_focus=np.array([False]),cohort_retained_useful=np.array([True]),
        observer_interval_mean=np.array([[0.,0.,1.,0.,0.,0.,0.,0.,0.]]),
        linear_at_centre=np.zeros((1,4,3,3)),angular_world=np.tile(np.eye(3),(1,4,1,1)),centre_m=np.zeros((1,3)),
        fixed_scale_Nm=np.ones(1),frozen_mean=np.zeros(3),frozen_cholesky=np.eye(3),
        reference_residual_Nm=np.array([[0.,.6,0.]]),old_I65_e_F=np.array([[0.,.6,0.]]),old_I65_e_Nalt=np.array([[0.,1.,0.]]),
        old_I65_code=np.array([-1],np.int8),reference_cf_code=np.array([-1],np.int8),
        reference_cf_F_score=np.array([.36]),reference_cf_Nalt_score=np.ones(1),radius_m=np.array(1.),event_anchor=np.array([True]))
    bank=dict(time_s=p['time_s'].copy(),source_index=p['source_index'].copy(),family_index=family.copy(),points_m=points,reconstructed=np.ones((1,5),bool))
    return p,bank


class BankAuditContracts(unittest.TestCase):
    def test_original_active_uses_finite_excitation_not_confidence_threshold(self):
        p=active_packet();p['old__excitation_m_s'][:]=[0.,np.nan]
        a=B.original_active(p);self.assertTrue(a[0].all());self.assertFalse(a[1].any())
        p['old__normal_angle_rad'][0,0]=np.nan;p['old__velocity_residual_norm_m_s'][0,1]=-1.
        p['old__moment_residual_norm'][0,2]=-1.;p['old__velocity_residual_m_s'][0,3,0]=np.nan
        p['old__valid'][0,4]=False
        np.testing.assert_array_equal(B.original_active(p)[0],[False,False,False,False,False,True])

    def test_original_active_validates_vector_speed_and_frame_flag(self):
        p=active_packet(1,1);p['old__velocity_residual_norm_m_s'][0,0]=1.
        with self.assertRaises(ValueError):B.original_active(p)
        p['old__frame_valid'][0]=False;self.assertFalse(B.original_active(p).any())

    def test_original_fused_cost_is_sum_of_fixed_components(self):
        p=active_packet(1,1);p['old__normal_angle_rad'][:]=np.deg2rad(10.);p['old__moment_residual_norm'][:]=2.
        p['old__velocity_residual_norm_m_s'][:]=.02
        components,cost=B.original_costs(p)
        np.testing.assert_allclose(components,[[[1.,4.,1.]]]);np.testing.assert_allclose(cost,[[6.]])

    def test_minimum_keeps_exact_stable_ties_and_missing_mask_unknown(self):
        values=np.array([[2.,1.,1.,np.nan],[np.nan,0.,5.,-np.inf],[1.,1.-1e-13,2.,3.]])
        mask=np.array([[1,1,1,1],[1,0,0,1],[1,1,1,1]],bool)
        index,value=B.minimum(values,mask)
        np.testing.assert_array_equal(index,[1,-1,1]);self.assertTrue(np.isnan(value[1]));self.assertLess(value[2],1.)

    def test_gather_unknown_never_reads_last_slot_and_preserves_shapes(self):
        vector=np.arange(18,dtype=float).reshape(3,3,2);index=np.array([1,-1,0])
        result=B.gather(vector,index);self.assertEqual(result.shape,(3,2));self.assertTrue(np.isnan(result[1]).all())
        np.testing.assert_array_equal(result[0],vector[0,1]);scalar=B.gather(vector[:,:,0],index)
        self.assertEqual(scalar.shape,(3,));self.assertTrue(np.isnan(scalar[1]))

    def test_scores_use_one_frozen_shared_bias_and_covariance_without_mutation(self):
        packet=dict(frozen_mean=np.array([1.,1.,1.]),frozen_cholesky=np.diag([2.,1.,.5]))
        original=copy.deepcopy(packet);values=np.array([[[3.,1.,2.],[1.,1.,1.]]])
        np.testing.assert_array_equal(B.scores(values,packet),[[5.,0.]])
        shift=np.array([4.,-2.,3.]);translated=dict(frozen_mean=packet['frozen_mean']+shift,frozen_cholesky=packet['frozen_cholesky'])
        np.testing.assert_array_equal(B.scores(values+shift,translated),[[5.,0.]])
        for k in packet:np.testing.assert_array_equal(packet[k],original[k])

    def test_minimum_score_can_be_ambiguous_while_another_existing_point_wins(self):
        e=np.array([[[.995,0.,0.],[.995,.02,0.]]]);wrong=np.array([[1.,0.,0.]])
        score=np.sum(e*e,axis=2);out=B.pair_capacity(e,score,wrong,np.ones(1),np.ones((1,2),bool))
        index,_=B.minimum(score,np.ones((1,2),bool))
        self.assertEqual(index[0],0);self.assertTrue(out['ambiguous'][0,0]);self.assertFalse(out['win'][0,0])
        self.assertTrue(out['win'][0,1]);self.assertTrue(np.any(out['win'],axis=1)[0])

    def test_wrong_branch_is_fixed_for_every_point_and_unknown_is_not_a_win(self):
        e=np.array([[[.2,0.,0.],[.5,0.,0.]],[[.1,0.,0.],[.2,0.,0.]]]);wrong=np.array([[1.,0.,0.],[np.nan,0.,0.]])
        wrong_score=np.array([1.,np.nan]);saved=wrong.copy();saved_score=wrong_score.copy()
        out=B.pair_capacity(e,np.sum(e*e,axis=2),wrong,wrong_score,np.ones((2,2),bool))
        self.assertTrue(out['win'][0].all());self.assertFalse(out['known'][1].any());self.assertFalse(out['win'][1].any())
        np.testing.assert_array_equal(wrong,saved);np.testing.assert_array_equal(wrong_score,saved_score)

    def test_selected_membership_and_existing_baseline_success_are_inclusion_invariants(self):
        p,b=toy_analysis();a,v=B.analyse(p,b)
        self.assertTrue(a['selected_member'][0]);self.assertTrue(a['baseline_correct'][0]);self.assertTrue(a['any_bank_correct'][0])
        self.assertTrue(v['existence_nonregression_is_inclusion_QA'])
        b['reconstructed'][0,0]=False
        with self.assertRaisesRegex(ValueError,'selected point not in usable bank'):B.analyse(p,b)

    def test_nearest_geometry_and_best_load_are_distinct_and_reference_is_not_score_floor(self):
        p,b=toy_analysis();a,_=B.analyse(p,b)
        self.assertEqual(a['nearest_id'][0],0);self.assertEqual(a['minscore_id'][0],1)
        self.assertEqual(a['nearest_distance_m'][0],0.);self.assertGreater(a['minimum_score_point_distance_m'][0],0.)
        self.assertLess(a['min_minus_reference_score'][0],0.)
        self.assertGreater(a['load_selection_gap'][0],0.)

    def test_nonfinite_saved_selected_reference_is_rejected(self):
        p,b=toy_analysis();p['old_I65_e_F'][0,0]=np.nan
        with self.assertRaisesRegex(ValueError,'selected load residual changed'):B.analyse(p,b)

    def test_group_report_preserves_full_denominator_and_29_reference_repairs(self):
        n=30;zero=np.zeros(n);a=dict(reference_correct=np.r_[np.ones(29,bool),False],baseline_correct=np.r_[np.zeros(29,bool),True],
            any_bank_correct=np.r_[np.ones(28,bool),False,True],nearest_geometry_correct=np.zeros(n,bool),minscore_correct=np.zeros(n,bool),
            usable_correct_count=np.r_[np.ones(28,int),0,1])
        for k in ('selected_distance_m','nearest_distance_m','pre_admission_distance_m','geometric_selection_gap_m','geometric_admission_gap_m','reference_radial_error_m',
                  'selected_load_score','min_load_score','reference_load_score','load_selection_gap','min_minus_reference_score','minimum_reference_torque_distance_Nm',
                  'nearest_geometry_load_score','minimum_score_point_distance_m'):a[k]=zero.copy()
        a['min_load_score'][28]=np.nan
        r=B.report_group(a,np.ones(n,bool))
        self.assertEqual((r['denominator'],r['known_bank']),(30,29));self.assertEqual(r['newly_correct_reference_denominator'],29)
        self.assertEqual((r['reference_repairs_supported'],r['reference_repairs_unsupported']),(28,1))
        self.assertEqual(sum(r['capacity_matrix'].values()),30);self.assertEqual(r['load']['min_load_score']['known'],29)


if __name__=='__main__':unittest.main()
