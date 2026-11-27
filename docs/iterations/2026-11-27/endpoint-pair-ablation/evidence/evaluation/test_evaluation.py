"""New I71 projection/continuation contracts; frozen I70 tests are reused."""
from pathlib import Path
import importlib.util
import unittest
import numpy as np


def imported(name,file):
    spec=importlib.util.spec_from_file_location(name,Path(__file__).with_name(file))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


prepare=imported('i71_prepare','prepare_evaluation.py')
evaluation=imported('i71_evaluate','evaluate_ablation.py')


def report(h=2,e=3):
    methods={m:dict(singleton=dict(denominator=100,errors=n,unknown=10,pair_output=0),
                    confidence=dict(coverage=.9,covered=90,wrong=n))
             for m,n in [('hmm',h),('emission',e),('state_hmm',h)]}
    timing={m:dict(missed=0,reverted_attempts=1,false_switches=2,
                  absolute_latency_s=dict(p95=.02)) for m in ('hmm','emission')}
    return dict(methods=methods,timing=timing)


def fixtures():
    baseline=dict(time_s=np.array([1.,1.005]),truth_patch_loads_N=np.ones((2,4)),
        truth_patch_points_m=np.full((2,4,3),9.),N_family=np.array([0,1]),F_family=np.array([1,2]),
        A_family=np.array([1,1]),i60_accepted=np.array([1,0],bool))
    baseline.update({m+'_points_m':np.full((2,4,3),float(i)) for i,m in enumerate(('N','F','A'))})
    # Deliberate stale fields must never be copied to the new evaluation packet.
    baseline.update({m+'_points_m':np.full((2,4,3),-123.) for m in ('emission','state_hmm','hmm')})
    pred=dict(time_s=baseline['time_s'].copy())
    pred.update({m+'_points_m':np.full((2,4,3),float(i+10)) for i,m in enumerate(('emission','state_hmm','hmm'))})
    return baseline,pred


class NewI71Contracts(unittest.TestCase):
    def test_new_points_replace_every_stale_I70_method(self):
        baseline,pred=fixtures();out=prepare.project(baseline,pred)
        for m in ('emission','state_hmm','hmm'):
            np.testing.assert_array_equal(out[m+'_points_m'],pred[m+'_points_m'])
            self.assertFalse(np.any(out[m+'_points_m']==-123.))
        for key in prepare.BASELINE_FIELDS:
            np.testing.assert_array_equal(out[key],baseline[key])

    def test_projection_rejects_clock_change(self):
        baseline,pred=fixtures();pred['time_s'][1]+=.001
        with self.assertRaises(ValueError):prepare.project(baseline,pred)

    def test_strict_pooled_improvement_cannot_pass_with_equal_errors(self):
        pooled=report(3,3);cases={c:report(1,1) for c in evaluation.CASES}
        gates={r['name']:r for r in evaluation.continuation_gates(pooled,cases)}
        self.assertFalse(gates['pooled_errors_strictly_less_than_matching_frame']['passed'])

    def test_one_phase_regression_fails_despite_pooled_gain(self):
        pooled=report(5,8);cases={c:report(1,3) for c in evaluation.CASES}
        cases['p030_B']=report(3,2)
        gates={r['name']:r for r in evaluation.continuation_gates(pooled,cases)}
        self.assertTrue(gates['pooled_errors_strictly_less_than_matching_frame']['passed'])
        self.assertFalse(gates['p030_B_errors_no_more_than_matching_frame']['passed'])

    def test_latency_and_missed_events_cannot_be_hidden(self):
        pooled=report();cases={c:report() for c in evaluation.CASES}
        pooled['timing']['hmm']['missed']=1
        pooled['timing']['hmm']['absolute_latency_s']['p95']=.055
        gates={r['name']:r for r in evaluation.continuation_gates(pooled,cases)}
        self.assertFalse(gates['timing_missed_no_more_than_matching_frame']['passed'])
        self.assertFalse(gates['timing_absolute_p95_at_most50ms']['passed'])

    def test_unscorable_latency_is_not_zero(self):
        pooled=report();cases={c:report() for c in evaluation.CASES}
        pooled['timing']['hmm']['absolute_latency_s']['p95']=None
        gates={r['name']:r for r in evaluation.continuation_gates(pooled,cases)}
        self.assertFalse(gates['timing_absolute_p95_at_most50ms']['passed'])

    def test_unknown_counts_remain_in_continuation_context(self):
        pooled=report();cases={c:report() for c in evaluation.CASES}
        got=evaluation.continuation_context(pooled,cases)
        self.assertEqual(got['pooled']['hmm']['primary']['unknown'],10)
        self.assertEqual(got['pooled']['hmm']['primary']['denominator'],100)
        self.assertTrue(got['no_adoption_authority'])

    def test_restricted_api_rejects_pair_mass(self):
        p=np.zeros((2,21));p[:,0]=1.
        prediction=dict(time_s=np.array([1.,1.005]),emission_probability21=p.copy(),
                        state_hmm_probability21=p.copy(),posterior21=p.copy())
        for m in ('emission','state_hmm','hmm'):
            points=np.full((2,4,3),np.nan);points[:,0]=0.;prediction[m+'_points_m']=points
        evaluation.check_restricted_api(prediction)
        prediction['posterior21'][0,8]=.01
        with self.assertRaises(ValueError):evaluation.check_restricted_api(prediction)

    def test_frozen_metric_source_is_byte_exact(self):
        self.assertEqual(evaluation.metrics.sha(evaluation.REFERENCE),evaluation.EXPECTED_REFERENCE_SHA)


if __name__=='__main__':unittest.main(verbosity=2)
