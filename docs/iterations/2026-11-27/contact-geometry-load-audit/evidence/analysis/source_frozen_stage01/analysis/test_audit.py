"""Synthetic I-66 contracts; no saved robot residuals or pose maps loaded."""
from pathlib import Path
import importlib.util
import unittest
import numpy as np

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_i66_audit_under_test',HERE/'audit.py')
A=importlib.util.module_from_spec(spec);spec.loader.exec_module(A)


def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def fixture(n=1):
    t=np.arange(n)*.005+1.
    centre=np.zeros((n,3));L=np.zeros((n,4,3,3));angular=np.tile(np.eye(3),(n,4,1,1))
    Fpoint=np.tile([1.,0.,0.],(n,1));Npoint=np.tile([0.,1.,0.],(n,1))
    pose=dict(time_s=t.copy(),centre_m=centre,linear_at_centre=L,angular_world=angular,
        valid=np.ones(n,bool),radius_m=np.array(1.),patch_axis_world=np.tile([0.,0.,1.],(n,4,1)),
        patch_lower_m=np.zeros(4),patch_upper_m=np.ones(4))
    raw=np.zeros((n,9));raw[:,:3]=[0.,1.,0.];raw[:,3:6]=[0.,0.,2.];raw[:,6:9]=[0.,0.,3.]
    primitive=dict(time_s=t.copy(),observer_interval_mean=raw,observer_generalized=raw.copy(),
        H_F=np.tile(skew([1.,0.,0.]),(n,1,1)),H_Nalt=np.tile(skew([0.,1.,0.]),(n,1,1)),
        H_F_valid=np.ones(n,bool),H_Nalt_valid=np.ones(n,bool),proposed_family=np.zeros(n,int),native_family=np.zeros(n,int),
        point_F_m=Fpoint,point_Nalt_m=Npoint)
    points=np.full((n,4,3),np.nan);points[:,0]=[.3,.4,0.]
    loads=np.zeros((n,4));loads[:,0]=1.
    truth=dict(truth_time_s=t.copy(),truth_patch_points_m=points,truth_patch_loads_N=loads)
    base=dict(time_s=t.copy(),truth_family=np.zeros(n,int),radius_m=np.ones(n),force_norm_N=np.ones(n),
        primary_eligible=np.ones(n,bool),i60_accepted=np.ones(n,bool),N_family=np.zeros(n,int),A_family=np.zeros(n,int))
    return pose,primitive,truth,base


class ScoreSpy:
    """No fitting API; every call must reuse the exact frozen model object."""
    def __init__(self):self.calls=[]
    def score_pair(self,F,N,model,pattern_epsilon,valid_F,valid_N):
        self.calls.append((F.copy(),N.copy(),model))
        sf=np.sum(F*F,axis=1);sn=np.sum(N*N,axis=1);valid=valid_F&valid_N&np.isfinite(sf)&np.isfinite(sn)
        ambiguous=valid&(np.linalg.norm(F-N,axis=1)<=pattern_epsilon)
        tolerance=1e-9*np.maximum(1.,np.maximum(sf,sn));tied=valid&~ambiguous&(abs(sf-sn)<=tolerance)
        known=valid&~ambiguous&~tied
        return dict(score_F=sf,score_N=sn,valid_pair=valid,ambiguous=ambiguous,tie=tied,
            F_lower=known&(sf<sn),N_lower=known&(sn<sf))


class AuditContracts(unittest.TestCase):
    def test_moment_arm_cross_product_sign_and_joint_projection(self):
        L=np.zeros((1,3,3));angular=np.array([np.diag([2.,3.,4.])])
        pred=A.point_prediction(L,angular,np.zeros((1,3)),np.array([[1.,0.,0.]]),np.array([[0.,1.,0.]]))
        np.testing.assert_array_equal(pred,[[0.,0.,4.]])

    def test_world_translation_of_point_and_centre_leaves_prediction_unchanged(self):
        L=np.array([np.eye(3)]);angular=np.array([[[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]]])
        c=np.array([[2.,3.,4.]]);p=np.array([[3.,5.,7.]]);force=np.array([[2.,-1.,4.]])
        a=A.point_prediction(L,angular,c,p,force);b=A.point_prediction(L,angular,c+[10.,20.,30.],p+[10.,20.,30.],force)
        np.testing.assert_array_equal(a,b)

    def test_point_shift_parallel_to_force_has_zero_load_effect(self):
        pose,p,t,b=fixture();t['truth_patch_points_m'][:,0]=p['point_F_m']+[0.,2.,0.]
        out=A.routes(pose,p,t,b)
        np.testing.assert_array_equal(out['geometry_load_delta_Nm'],np.zeros((1,3)))
        np.testing.assert_allclose(out['point_parallel_m'],[2.]);np.testing.assert_allclose(out['point_perpendicular_m'],[0.])

    def test_pure_couple_is_not_represented_by_zero_point_force(self):
        pose,p,t,b=fixture();p['observer_interval_mean'][:,:3]=0.
        out=A.routes(pose,p,t,b)
        np.testing.assert_array_equal(out['T_sampled_point_prediction_Nm'],[[0.,0.,0.]])
        np.testing.assert_array_equal(out['W_single_family_wrench_prediction_Nm'],[[0.,0.,2.]])
        self.assertTrue(out['T_sampled_point_valid'][0] and out['W_single_family_wrench_valid'][0])
        self.assertTrue(np.isnan(out['point_parallel_m'][0]) and np.isnan(out['point_perpendicular_m'][0]))

    def test_geometry_SSE_and_free_couple_decompositions(self):
        out=A.routes(*fixture())
        for key in ('geometry_identity_error_Nm','SSE_gain_identity_error_Nm2','free_couple_identity_error_Nm'):
            np.testing.assert_allclose(out[key],0.,atol=1e-14)
        np.testing.assert_allclose(out['geometry_load_delta_Nm'],[[0.,0.,-.7]])
        np.testing.assert_allclose(out['free_couple_joint_Nm'],[[0.,0.,1.7]])

    def test_raw_interior_mean_point_is_retained_not_projected(self):
        out=A.routes(*fixture())
        self.assertTrue(out['T_sampled_point_valid'][0]);self.assertFalse(out['reference_material_band_valid'][0])
        np.testing.assert_array_equal(out['reference_point_m'],[[.3,.4,0.]])
        np.testing.assert_allclose(out['reference_radial_error_m'],[-.5])

    def test_zero_share_nan_points_are_harmless(self):
        pose,p,t,b=fixture()
        pred,valid,share=A.split_prediction(pose['linear_at_centre'],pose['angular_world'],pose['centre_m'],
            t['truth_patch_points_m'],t['truth_patch_loads_N'],p['observer_interval_mean'][:,:3],np.ones(1,bool))
        self.assertTrue(valid[0]);np.testing.assert_array_equal(share,[[1.,0.,0.,0.]])
        np.testing.assert_allclose(pred,[[0.,0.,.3]])

    def test_positive_missing_point_is_unknown_without_reweighting(self):
        pose,p,t,b=fixture();t['truth_patch_loads_N'][0]=[1.,3.,0.,0.]
        pred,valid,share=A.split_prediction(pose['linear_at_centre'],pose['angular_world'],pose['centre_m'],
            t['truth_patch_points_m'],t['truth_patch_loads_N'],p['observer_interval_mean'][:,:3],np.ones(1,bool))
        self.assertFalse(valid[0]);self.assertTrue(np.isnan(pred).all());np.testing.assert_array_equal(share,[[.25,.75,0.,0.]])

    def test_split_preserves_loads_and_uses_each_family_map(self):
        pose,p,t,b=fixture();t['truth_patch_loads_N'][0]=[1.,3.,0.,0.];t['truth_patch_points_m'][0,1]=[0.,0.,1.]
        pred,valid,share=A.split_prediction(pose['linear_at_centre'],pose['angular_world'],pose['centre_m'],
            t['truth_patch_points_m'],t['truth_patch_loads_N'],p['observer_interval_mean'][:,:3],np.ones(1,bool))
        self.assertTrue(valid[0]);np.testing.assert_allclose(pred,[[-.75,0.,.075]])
        np.testing.assert_array_equal(share,[[.25,.75,0.,0.]])

    def test_split_nonfinite_or_negative_load_and_zero_total_are_unknown(self):
        for loads in ([np.nan,1.,0.,0.],[-1.,2.,0.,0.],[0.,0.,0.,0.]):
            pose,p,t,b=fixture();t['truth_patch_loads_N'][0]=loads
            pred,valid,_=A.split_prediction(pose['linear_at_centre'],pose['angular_world'],pose['centre_m'],
                t['truth_patch_points_m'],t['truth_patch_loads_N'],p['observer_interval_mean'][:,:3],np.ones(1,bool))
            self.assertFalse(valid[0]);self.assertTrue(np.isnan(pred).all())

    def test_F_first_choice_does_not_minimize_reference_distance_or_residual(self):
        pose,p,t,b=fixture();p['observer_interval_mean'][:,6:9]=0.;t['truth_patch_points_m'][:,0]=p['point_Nalt_m']
        out=A.routes(pose,p,t,b)
        self.assertEqual(out['correct_branch'][0],0)
        np.testing.assert_array_equal(out['candidate_point_m'],p['point_F_m'])
        self.assertGreater(np.linalg.norm(out['F_original_residual_Nm'][0]),np.linalg.norm(out['Nalt_original_residual_Nm'][0]))

    def test_wrong_F_family_uses_correct_Nalt_or_unknown(self):
        np.testing.assert_array_equal(A.choose_correct_branch(np.array([1,0,0,0]),np.array([0,0,1,0]),
            np.array([0,0,2,-1]),np.array([1,0,1,1],bool),np.array([1,1,1,1],bool)),[1,1,-1,-1])

    def test_full_denominator_keeps_unknown_paired_rows(self):
        pose,p,t,b=fixture(3);p['H_F_valid'][1]=False;p['H_Nalt_valid'][1]=False;t['truth_patch_points_m'][2,0]=np.nan
        out=A.routes(pose,p,t,b);summary=A.describe(out,np.ones(3,bool))
        self.assertEqual(summary['denominator'],3);self.assertEqual(summary['paired_count'],1);self.assertEqual(summary['unknown_pair'],2)
        self.assertEqual(summary['routes']['C_candidate']['count'],2);self.assertEqual(summary['routes']['T_sampled_point']['count'],2)

    def test_counterfactual_only_scores_distinct_family_opportunities_without_refit(self):
        model=object();spy=ScoreSpy()
        arrays=dict(fixed_scale_Nm=np.ones(2),F_original_residual_Nm=np.array([[2.,0.,0.],[3.,0.,0.]]),
            Nalt_original_residual_Nm=np.array([[2.,0.,0.],[2.,0.,0.]]),F_original_valid=np.ones(2,bool),
            Nalt_original_valid=np.ones(2,bool),correct_branch=np.zeros(2,int),
            T_sampled_point_residual_Nm=np.array([[0.,0.,0.],[1.,0.,0.]]),T_sampled_point_valid=np.ones(2,bool))
        base=dict(primary_eligible=np.ones(2,bool),N_family=np.array([0,1]),A_family=np.array([0,0]),truth_family=np.array([0,0]))
        result=A.counterfactual(arrays,base,model,spy,dict(comparison=dict(pattern_epsilon_dimensionless=.01)))
        self.assertEqual([len(call[0]) for call in spy.calls],[1,1]);self.assertTrue(all(call[2] is model for call in spy.calls))
        np.testing.assert_array_equal(result['applicable'],[False,True]);self.assertEqual(result['original_code'][0],3)
        self.assertTrue(np.isnan(result['sampled_correct_point_F_score'][0]))
        self.assertEqual(result['original_code'][1],1);self.assertEqual(result['sampled_correct_point_code'][1],-1)
        np.testing.assert_array_equal(spy.calls[0][1],spy.calls[1][1])

    def test_counterfactual_report_retains_ties_unknowns_and_ambiguity(self):
        arrays=dict(cf_original_code=np.array([1,-1,-1]),cf_sampled_correct_point_code=np.array([-1,3,2]))
        masks=dict(useful=np.ones(3,bool),harmful=np.zeros(3,bool),focus=np.ones(3,bool))
        row=A.counterfactual_report(arrays,masks)['useful']
        self.assertEqual((row['denominator'],row['replacement_correct'],row['unknown'],row['ambiguous']),(3,1,1,1))

    def test_vector_force_statistics_have_force_units(self):
        row=A.vector_stats(np.array([[3.,4.,0.]]),np.ones(1,bool),unit='N')
        self.assertEqual(row['vector_RMS_N'],5.);self.assertNotIn('vector_RMS_Nm',row)

    def test_empty_phase_focus_is_explicitly_unscorable(self):
        stats=dict(count=2,vector_RMS_Nm=1.,vector_p95_Nm=1.)
        improved=dict(count=2,vector_RMS_Nm=.5,vector_p95_Nm=.5)
        focus=dict(denominator=2,paired_count=2,paired_C=stats,paired_T=improved)
        summary=dict(focus=focus,harmful=dict(denominator=1,paired_count=1))
        empty=dict(denominator=0,paired_count=0,paired_C=dict(count=0),paired_T=dict(count=0))
        result=A.qualify(summary,dict(empty=dict(cohorts=dict(focus=empty))),None,None,
            dict(primary_comparison=dict(roundoff_Nm=1e-10,coverage_min=.99,minimum_vector_RMS_reduction_fraction=.1)))
        self.assertIsNone(result['gates']['empty_RMS_nonincrease']);self.assertTrue(result['qualified_geometric_improvement'])


if __name__=='__main__':unittest.main()
