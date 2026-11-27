"""Synthetic unittest contracts for shared calibration; no historical data."""
from pathlib import Path
import sys,unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parent))
import covariance_score as C


def identity_model():
    return dict(mean=np.zeros(3),cholesky=np.eye(3),covariance=np.eye(3))


class CovarianceContracts(unittest.TestCase):
    def test_correlated_error_model_reverses_inappropriate_maxabs_ranking(self):
        common=np.sqrt(3.*1.9)*np.array([1.,1.,0.])/np.sqrt(2.)
        opposite=np.sqrt(3.*.1)*np.array([1.,-1.,0.])/np.sqrt(2.)
        vertical=np.array([0.,0.,np.sqrt(3.)])
        training=np.array([common,-common,opposite,-opposite,vertical,-vertical])
        model=C.fit_shared_covariance(training,np.ones(6),rho=0.,floor_std=1e-6)
        expected=np.array([[1.,.9,0.],[.9,1.,0.],[0.,0.,1.]])
        np.testing.assert_allclose(model['population_covariance'],expected,atol=1e-15)
        first=np.array([1.,1.,0.]);second=np.array([.6,-.6,0.])
        self.assertGreater(np.max(abs(first)),np.max(abs(second)))
        pair=C.score_pair(first,second,model)
        self.assertTrue(pair['F_lower']);self.assertFalse(pair['unknown'])
        self.assertAlmostEqual(float(pair['score_F']),2./1.9,places=10)
        self.assertAlmostEqual(float(pair['score_N']),.72/.1,places=8)

    def test_equal_episode_weighting_survives_duplicate_rows(self):
        collapsed=np.array([[2.,0.,0.],[0.,2.,0.]])
        duplicated=np.array([[2.,0.,0.],[2.,0.,0.],[2.,0.,0.],[0.,2.,0.]])
        a=C.fit_shared_covariance(collapsed,[.5,.5],rho=.25,floor_std=.1)
        b=C.fit_shared_covariance(duplicated,[1./6.,1./6.,1./6.,.5],rho=.25,floor_std=.1)
        uniform=C.fit_shared_covariance(duplicated,np.ones(4),rho=.25,floor_std=.1)
        for field in ['mean','population_covariance','covariance']:np.testing.assert_allclose(a[field],b[field],atol=1e-15)
        np.testing.assert_allclose(a['mean'],[1.,1.,0.]);np.testing.assert_allclose(uniform['mean'],[1.5,.5,0.])

    def test_population_covariance_and_fixed_shrinkage_formula(self):
        model=C.fit_shared_covariance([[2.,0.,0.],[-2.,0.,0.]],[.5,.5],rho=.5,floor_std=.2)
        np.testing.assert_array_equal(model['mean'],np.zeros(3))
        np.testing.assert_array_equal(model['population_covariance'],np.diag([4.,0.,0.]))
        expected=.5*np.diag([4.,0.,0.])+.5*(4./3.)*np.eye(3)+.04*np.eye(3)
        np.testing.assert_allclose(model['covariance'],expected,atol=1e-15)
        np.testing.assert_allclose(model['cholesky']@model['cholesky'].T,expected,atol=1e-15)

    def test_positive_floor_handles_zero_covariance_without_hidden_retuning(self):
        rows=np.tile([2.,-1.,.5],(4,1));model=C.fit_shared_covariance(rows,np.ones(4),rho=1.,floor_std=.2)
        np.testing.assert_array_equal(model['population_covariance'],np.zeros((3,3)))
        np.testing.assert_allclose(model['covariance'],.04*np.eye(3))
        self.assertAlmostEqual(float(C.score_residuals([2.2,-1.,.5],model)),1.)

    def test_training_rows_must_be_finite_nonempty_and_three_dimensional(self):
        for rows,weight in [([],[]),([[1.,2.]],[1.]),([[np.nan,0.,0.]],[1.]),([[np.inf,0.,0.]],[1.])]:
            with self.subTest(rows=rows),self.assertRaises(ValueError):C.fit_shared_covariance(rows,weight,rho=.5,floor_std=.1)

    def test_weight_checks_and_normalization(self):
        rows=np.zeros((2,3))
        for weight in [[1.],[-1.,2.],[0.,0.],[np.nan,1.],[np.inf,1.]]:
            with self.subTest(weight=weight),self.assertRaises(ValueError):C.fit_shared_covariance(rows,weight,rho=.5,floor_std=.1)
        a=C.fit_shared_covariance([[0.,0.,0.],[2.,0.,0.]],[1.,3.],rho=.5,floor_std=.1)
        np.testing.assert_array_equal(a['normalized_weights'],[.25,.75]);np.testing.assert_array_equal(a['mean'],[1.5,0.,0.])

    def test_regularization_parameters_are_validated(self):
        for rho,floor in [(-.1,.1),(1.1,.1),(np.nan,.1),(.5,0.),(.5,-1.),(.5,np.inf)]:
            with self.subTest(rho=rho,floor=floor),self.assertRaises(ValueError):C.fit_shared_covariance([[0.,0.,0.]],[1.],rho=rho,floor_std=floor)

    def test_pair_swap_preserves_shared_scores_and_reverses_preference(self):
        first=np.array([[1.,0.,0.],[0.,3.,0.]]);second=np.array([[2.,0.,0.],[0.,2.,0.]])
        a=C.score_pair(first,second,identity_model());b=C.score_pair(second,first,identity_model())
        np.testing.assert_array_equal(a['score_F'],b['score_N']);np.testing.assert_array_equal(a['score_N'],b['score_F'])
        np.testing.assert_array_equal(a['F_lower'],b['N_lower']);np.testing.assert_array_equal(a['N_lower'],b['F_lower'])
        np.testing.assert_array_equal(a['unknown'],b['unknown'])

    def test_common_bias_translation_preserves_covariance_and_scores(self):
        rows=np.array([[0.,0.,0.],[1.,2.,0.],[-1.,0.,1.],[2.,-1.,0.]])
        shift=np.array([3.,-4.,2.]);weights=np.array([.1,.2,.3,.4])
        a=C.fit_shared_covariance(rows,weights,rho=.25,floor_std=.1)
        b=C.fit_shared_covariance(rows+shift,weights,rho=.25,floor_std=.1)
        np.testing.assert_allclose(b['mean'],a['mean']+shift,atol=1e-15)
        np.testing.assert_allclose(b['covariance'],a['covariance'],atol=2e-15)
        first=np.array([.2,-.3,.4]);second=np.array([1.,.5,-.2])
        x=C.score_pair(first,second,a);y=C.score_pair(first+shift,second+shift,b)
        np.testing.assert_allclose(x['score_F'],y['score_F'],atol=2e-14);np.testing.assert_allclose(x['score_N'],y['score_N'],atol=2e-14)
        np.testing.assert_array_equal(x['decision'],y['decision'])

    def test_ties_identical_patterns_and_invalid_rows_are_unknown(self):
        first=np.array([[1.,0.,0.],[1.,2.,3.],[np.nan,0.,0.],[0.,0.,0.]])
        second=np.array([[0.,1.,0.],[1.,2.,3.],[1.,0.,0.],[1.,0.,0.]])
        out=C.score_pair(first,second,identity_model(),valid_F=[True,True,True,False])
        np.testing.assert_array_equal(out['unknown'],np.ones(4,bool))
        self.assertTrue(out['tie'][0]);self.assertTrue(out['ambiguous'][1]);self.assertTrue(out['invalid'][2]);self.assertTrue(out['invalid'][3])
        self.assertFalse(out['F_lower'].any());self.assertFalse(out['N_lower'].any())

    def test_pattern_epsilon_is_on_shared_normalized_residual_difference(self):
        out=C.score_pair([0.,0.,0.],[1e-4,0.,0.],identity_model(),pattern_epsilon=1e-3)
        self.assertTrue(out['ambiguous']);self.assertTrue(out['unknown'])
        for epsilon in [-1.,np.nan,np.inf]:
            with self.subTest(epsilon=epsilon),self.assertRaises(ValueError):C.score_pair([0.,0.,0.],[1.,0.,0.],identity_model(),pattern_epsilon=epsilon)

    def test_relative_score_tie_scales_at_large_scores(self):
        tied=C.score_pair([1000.,0.,0.],[1000.+2e-7,0.,0.],identity_model())
        separate=C.score_pair([1000.,0.,0.],[1000.+2e-6,0.,0.],identity_model())
        self.assertTrue(tied['tie']);self.assertTrue(tied['unknown']);self.assertTrue(separate['F_lower'])

    def test_common_engineering_normalization_preserves_unknowns(self):
        raw=np.array([[.15,.30,0.],[1.,2.,3.],[np.nan,0.,0.]])
        normalized,valid=C.normalize_residuals(raw,[.12,0.,.12],[10.,10.,10.])
        np.testing.assert_allclose(normalized[0],[1.,2.,0.]);np.testing.assert_array_equal(valid,[True,False,False])
        self.assertTrue(np.isnan(normalized[1:]).all())

    def test_train_inference_and_model_inputs_are_not_mutated(self):
        training=np.array([[1.,2.,3.],[-1.,0.,2.],[0.,1.,-1.]])
        weights=np.array([1.,2.,1.]);training_copy=training.copy();weights_copy=weights.copy()
        model=C.fit_shared_covariance(training,weights,rho=.25,floor_std=.1)
        frozen={key:value.copy() for key,value in model.items() if isinstance(value,np.ndarray)}
        first=np.array([[1.,0.,0.],[np.nan,0.,0.]]);second=np.array([[0.,1.,0.],[1.,2.,3.]])
        first_copy=first.copy();second_copy=second.copy();C.score_pair(first,second,model)
        C.normalize_residuals(first,.12,10.)
        np.testing.assert_array_equal(training,training_copy);np.testing.assert_array_equal(weights,weights_copy)
        np.testing.assert_array_equal(first,first_copy);np.testing.assert_array_equal(second,second_copy)
        for key,value in frozen.items():np.testing.assert_array_equal(model[key],value)


if __name__=='__main__':unittest.main(verbosity=2)
