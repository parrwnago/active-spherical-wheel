"""New I72 projection/policy/gate contracts; no saved arrays or physics."""
from pathlib import Path
import importlib.util
import unittest
import numpy as np


def imported(name,file):
    s=importlib.util.spec_from_file_location(name,Path(__file__).with_name(file));m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m


p=imported('i72_projection','prepare_evaluation.py')
e=imported('i72_evaluation','evaluate_selector.py')


def fixture(n=20):
    time=1.+np.arange(n)*.005;family=np.zeros(n,np.int8);point=np.zeros((n,3))
    loads=np.tile([10.,0.,0.,0.],(n,1));truth=e.ref.truth_labels(loads)
    packet=dict(time_s=time,truth_patch_loads_N=loads,truth_patch_points_m=np.zeros((n,4,3)),
        old_primary_dominant=truth['truth_dominant'],old_truth_family=truth['truth_family'],
        old_ambiguous_motion=np.ones(n,bool),old_accepted=np.ones(n,bool),
        newF_accepted=np.ones(n,bool),newVG_accepted=np.ones(n,bool))
    for m in ('N','oldF','A','newF','newVG'):packet[m+'_family']=family.copy();packet[m+'_point_m']=point.copy()
    return packet


def projection_fixture(n=4):
    x=fixture(n);oldaccepted=np.arange(n)%2==0
    points=np.full((n,4,3),np.nan);points[:,0]=0.
    fpoints=points.copy();fpoints[:,0,0]=.002
    apoints=np.where(oldaccepted[:,None,None],fpoints,points)
    base=dict(time_s=x['time_s'],truth_patch_loads_N=x['truth_patch_loads_N'],truth_patch_points_m=x['truth_patch_points_m'],
        N_family=x['N_family'],F_family=x['oldF_family'],A_family=x['A_family'],i60_accepted=oldaccepted,
        N_points_m=points,F_points_m=fpoints,A_points_m=apoints)
    old=dict(time_s=x['time_s'].copy(),primary_mask=np.ones(n,bool),truth_eligible=np.ones(n,bool),
        truth_family=np.zeros(n,np.int8),native_family=x['N_family'].copy(),
        fused__chosen_family=x['oldF_family'].copy(),fused__accepted_family=np.where(oldaccepted,0,-1),
        valid_family_count=np.full(n,2),measured_excitation_m_s=np.full(n,.1))
    pred=dict(time_s=x['time_s'].copy())
    for prefix in ('fused','velocity'):
        pred[prefix+'__chosen_family']=np.zeros(n,np.int8)
        pred[prefix+'__accepted_family']=np.zeros(n,np.int8)
        pred[prefix+'__point_m']=np.tile([.003,0.,0.],(n,1))
    return base,old,pred


def gates_for(packet):
    frame,case=e.evaluate_case(packet)
    cases={c:case for c in e.CASES};frames={c:frame for c in e.CASES}
    pooled=e.aggregate(frames,cases)
    return {x['name']:x for x in e.gates(pooled,cases)},pooled


class SelectorEvaluationContracts(unittest.TestCase):
    def test_nativefallback_preserves_equivalentpoint_and_family(self):
        x=fixture();x['newF_accepted'][:]=False;x['newF_family'][:]=1;x['newF_point_m'][:]=.1
        policy=e.policies(x)['C']
        np.testing.assert_array_equal(policy['family'],x['N_family'])
        np.testing.assert_array_equal(policy['point_m'],x['N_point_m'])

    def test_accepted_samefamily_physicalpoint_update_is_not_lost(self):
        x=fixture();x['newF_point_m'][:,0]=.001
        policy=e.policies(x)['C']
        np.testing.assert_array_equal(policy['family'],x['N_family'])
        np.testing.assert_array_equal(policy['point_m'],x['newF_point_m'])
        self.assertFalse(np.array_equal(policy['point_m'],x['N_point_m']))

    def test_secondary_VG_does_not_replace_primary_fused(self):
        x=fixture();x['newF_family'][:]=1
        policy=e.policies(x)
        self.assertTrue(np.all(policy['C']['family']==1))
        self.assertTrue(np.all(policy['VG_C']['family']==0))

    def test_acceptnothing_cannot_pass_coverage_by_nativefallback(self):
        x=fixture();x['newF_accepted'][:]=False
        gates,pooled=gates_for(x)
        self.assertEqual(pooled['methods']['C']['singleton']['errors'],0)
        self.assertFalse(gates['old_ambiguous_motion_acceptance_coverage_no_less_than_old_A']['passed'])
        self.assertFalse(gates['all_primary_acceptance_coverage_no_less_than_old_A']['passed'])

    def test_point_missing_does_not_shrink_fixed_primary_comparison(self):
        x=fixture();x['newF_point_m'][0]=np.nan
        gates,pooled=gates_for(x)
        fixed=pooled['points']['fixed_primary_C_vs_N']
        self.assertEqual(fixed['C_missing_on_native_reference'],3)
        self.assertEqual(fixed['reference_count'],60)
        self.assertIsNone(fixed['C_mm']['rms'])
        self.assertFalse(gates['fullprimary_C_point_coverage_no_less_than_N']['passed'])
        self.assertFalse(gates['fixedprimary_point_maximum_no_worse_than_N_plus1um']['passed'])

    def test_point_gate_units_one_micrometre_not_one_millimetre(self):
        x=fixture();x['newF_point_m'][:,0]=2e-6
        gates,pooled=gates_for(x)
        self.assertAlmostEqual(pooled['points']['fixed_primary_C_vs_N']['C_mm']['rms'],.002)
        self.assertEqual(gates['fixedprimary_point_rms_no_worse_than_N_plus1um']['limit'],.001)
        self.assertFalse(gates['fixedprimary_point_rms_no_worse_than_N_plus1um']['passed'])

    def test_full_loaded_pair_rows_remain_errors_without_primary_label(self):
        x=fixture();x['truth_patch_loads_N'][:]=[6.,4.,0.,0.]
        t=e.ref.truth_labels(x['truth_patch_loads_N']);x['old_primary_dominant']=t['truth_dominant'];x['old_truth_family']=t['truth_family']
        x['old_ambiguous_motion'][:]=False
        frame,report=e.evaluate_case(x)
        self.assertEqual(report['methods']['C']['singleton']['denominator'],0)
        self.assertEqual(report['methods']['C']['loaded_support']['denominator'],20)
        self.assertEqual(report['methods']['C']['loaded_support']['errors'],20)

    def test_projection_keeps_old_ambiguity_domain_and_newphysicalpoints(self):
        base,old,pred=projection_fixture();old['valid_family_count'][0]=1;old['measured_excitation_m_s'][1]=.001
        result=p.project(base,old,pred)
        np.testing.assert_array_equal(result['old_ambiguous_motion'],[False,False,True,True])
        np.testing.assert_array_equal(result['newF_point_m'],pred['fused__point_m'])
        np.testing.assert_array_equal(result['A_point_m'][0],[.002,0.,0.])
        np.testing.assert_array_equal(result['N_point_m'][0],[0.,0.,0.])

    def test_projection_rejects_stale_oldA_samefamily_point(self):
        base,old,pred=projection_fixture();base['A_points_m'][0,0]=0.
        with self.assertRaises(ValueError):p.project(base,old,pred)

    def test_projection_rejects_mismatched_native_consumer_fallback(self):
        base,old,pred=projection_fixture();pred['N_family']=np.ones(4,np.int8)
        with self.assertRaises(ValueError):p.project(base,old,pred)

    def test_accepted_candidate_requires_its_rawfamily_and_finitepoint(self):
        base,old,pred=projection_fixture();pred['fused__accepted_family'][0]=1
        with self.assertRaises(ValueError):p.project(base,old,pred)

    def test_wrongfamily_points_are_not_removed_from_primary_stats(self):
        x=fixture();x['newF_family'][:]=1;x['newF_point_m'][:,0]=.01
        gates,pooled=gates_for(x)
        self.assertEqual(pooled['methods']['C']['singleton']['errors'],60)
        self.assertAlmostEqual(pooled['points']['fixed_primary_C_vs_N']['C_mm']['maximum'],10.)


if __name__=='__main__':unittest.main(verbosity=2)
