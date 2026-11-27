"""Synthetic I-65 calibration boundary and fixed shadow-policy contracts."""
from pathlib import Path
import unittest
import numpy as np
import evaluate as E

ROOT=Path(__file__).resolve().parents[1]
METHOD=E.module(ROOT/'method/covariance_score.py','_i65_method_contracts')
PROTOCOL=E.read(ROOT/'protocol/protocol.json')


def fixture(n=220,offset=0.):
    x=np.arange(n,dtype=float)
    e=np.column_stack((np.sin(x)*.1+offset,np.cos(x)*.05,np.sin(x*.3)*.02))
    b=dict(primary_eligible=np.ones(n,bool),i60_accepted=np.ones(n,bool),
           radius_m=np.ones(n),force_norm_N=np.ones(n),truth_family=np.zeros(n,np.int64))
    p=dict(proposed_family=np.zeros(n,np.int64),native_family=np.ones(n,np.int64),
           material_F_valid=np.ones(n,bool),material_Nalt_valid=np.ones(n,bool),H_F_valid=np.ones(n,bool))
    f={}
    for model in E.MODELS:
        f.update({model+'_F_residual_Nm':e*.13,model+'_Nalt_residual_Nm':(e+1)*.13,
                  model+'_F_valid':np.ones(n,bool),model+'_Nalt_valid':np.ones(n,bool)})
    return b,p,f


class EvaluationContracts(unittest.TestCase):
    def test_one_teacher_per_timestamp_F_first(self):
        b,p,f=fixture()
        p['native_family'][:]=0
        x,chosen=E.calibrated_rows(b,p,f,'M1',METHOD)
        self.assertEqual(len(x),len(b['truth_family']))
        self.assertTrue(np.all(chosen==0))

    def test_teacher_fallback_and_no_correct_branch(self):
        b,p,f=fixture()
        p['proposed_family'][:3]=2
        p['native_family'][:2]=0
        _,chosen=E.calibrated_rows(b,p,f,'M1',METHOD)
        np.testing.assert_array_equal(chosen[:4],[1,1,-1,0])

    def test_material_validity_and_eligibility(self):
        b,p,f=fixture()
        p['material_F_valid'][0]=False
        b['primary_eligible'][1]=False
        b['i60_accepted'][2]=False
        _,chosen=E.calibrated_rows(b,p,f,'M1',METHOD)
        np.testing.assert_array_equal(chosen[:4],[-1,-1,-1,0])

    def test_equal_episode_weights_include_between_episode_bias(self):
        a=fixture(n=220,offset=0.);b=fixture(n=440,offset=2.)
        fit,_,_=E.fit_fold(['a','b'],{'a':a,'b':b},'M1',METHOD,PROTOCOL)
        xa,_=E.calibrated_rows(*a,'M1',METHOD)
        xb,_=E.calibrated_rows(*b,'M1',METHOD)
        mu=(xa.mean(axis=0)+xb.mean(axis=0))/2
        np.testing.assert_allclose(fit['mean'],mu,atol=1e-14)
        self.assertGreater(fit['population_covariance'][0,0],.9)

    def test_heldout_cannot_be_passed_to_fit(self):
        with self.assertRaises(ValueError):
            E.fit_fold(['a','b'],{'a':fixture(),'b':fixture(),'heldout':fixture()},'M1',METHOD,PROTOCOL)

    def test_scorer_needs_no_truth_or_point_errors(self):
        a=fixture();fit,_,_=E.fit_fold(['a','b'],{'a':a,'b':fixture(offset=.1)},'M1',METHOD,PROTOCOL)
        b,p,f=fixture()
        del b['truth_family'];del b['primary_eligible']
        del p['proposed_family'];del p['native_family']
        out=E.score_heldout(b,p,f,'M1',fit,METHOD,PROTOCOL)
        self.assertTrue(np.all(out['shared_covariance_known']))

    def test_failed_fit_is_unknown_and_retains_A(self):
        a=fixture(n=10)
        fit,_,_=E.fit_fold(['a','b'],{'a':a,'b':fixture()},'M1',METHOD,PROTOCOL)
        self.assertIsNone(fit)
        out=E.score_heldout(*a,'M1',fit,METHOD,PROTOCOL)
        self.assertTrue(np.all(out['shared_covariance_code']==3))
        self.assertTrue(np.all(out['shared_covariance_keep']))

    def test_policy_unknown_ambiguous_tie_and_invalid_map(self):
        accepted=np.array([1,1,1,1,1,1,0],bool)
        valid=np.array([1,1,1,1,1,0,1],bool)
        code=np.array([-1,1,0,2,3,-1,-1],np.int8)
        np.testing.assert_array_equal(E.shadow_keep(accepted,valid,code),[1,0,1,1,1,0,0])

    def test_ambiguity_precedes_score_tie(self):
        eF=np.array([[0,0,0],[1,0,0],[0,0,0]],float)
        eN=np.array([[.001,0,0],[-1,0,0],[1,0,0]],float)
        code=E.compare_scores(np.array([0.,1.,0.]),np.array([.1,1.,1.]),eF,eN,np.ones(3,bool),PROTOCOL['comparison'])
        np.testing.assert_array_equal(code,[2,0,-1])

    def test_full_denominators_count_ambiguity_unknown_as_not_correct(self):
        b=dict(primary_eligible=np.ones(4,bool),i60_accepted=np.ones(4,bool),
               N_family=np.array([1,1,0,0]),A_family=np.array([0,0,1,1]),truth_family=np.zeros(4,int))
        out=dict(shared_covariance_code=np.array([-1,2,1,3]),shared_covariance_known=np.array([1,1,1,0],bool))
        r=E.discrimination(b,out,'shared_covariance')
        self.assertEqual(r['useful']['denominator'],2);self.assertEqual(r['useful']['correct'],1)
        self.assertEqual(r['harmful']['denominator'],2);self.assertEqual(r['harmful']['correct'],1)


if __name__=='__main__':unittest.main()
