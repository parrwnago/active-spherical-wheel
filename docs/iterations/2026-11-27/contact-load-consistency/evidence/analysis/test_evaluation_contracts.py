"""Synthetic I-61 policy/evaluator contracts; no saved robot data are opened."""
import copy
import importlib.util
from pathlib import Path
import json

import numpy as np

HERE=Path(__file__).resolve().parent


def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value);return value


E=module(HERE/'evaluate_load_gate.py','_i61_eval_contract')
X=module(HERE.parent/'campaign_tools/extract_features.py','_i61_extract_contract')
PROTOCOL=json.loads((HERE.parent/'protocol/protocol.json').read_text())


def fake_frames():
    truth=np.array([0,1,1,0]);native=np.array([0,0,1,1]);A=np.array([1,1,1,0]);C=np.array([0,1,1,0])
    n=len(truth);eligible=np.ones(n,bool);target=np.zeros((n,3))
    frames=dict(time_s=np.arange(n)*.005,truth_family=truth,truth_eligible=eligible.copy(),truth_loaded=eligible.copy(),
        N_family=native,A_family=A,C_family=C,A_uses_proposal=eligible.copy(),C_uses_proposal=np.array([False,True,True,True]),
        N_point_m=np.ones((n,3))*.001,A_point_m=np.ones((n,3))*.0009,C_point_m=np.ones((n,3))*.0008,
        load_unknown=np.zeros(n,bool),stable_truth_mask=eligible.copy(),oracle_geometry_valid=eligible.copy(),
        oracle_geometry_pass=eligible.copy(),truth_resolved_contact_counts_gt005N=np.ones((n,4),int),
        truth_equivalent_point_m=target,truth_dominant_patch_point_m=target,
        equivalent_radial_R_proxy=target,dominant_patch_radial_R_proxy=target)
    frames['harmful_A']=eligible&(native==truth)&(A!=truth)
    frames['harmful_C']=eligible&(native==truth)&(C!=truth)
    frames['useful_A']=eligible&(native!=truth)&(A==truth)
    frames['retained_useful']=frames['useful_A']&(C==truth)
    return frames


def test_same_family_accepted_proposal_changes_point_and_veto_restores_native():
    native=np.array([0,0]);point=np.array([[0.,0.,0.],[0.,0.,0.]])
    proposed=np.array([0,1]);proposal=np.array([[.01,0.,0.],[.02,0.,0.]])
    result=X.policies(native,point,proposed,proposal,np.array([True,False]),np.array([False,True]))
    np.testing.assert_array_equal(result['A_family'],native)
    np.testing.assert_array_equal(result['A_point_m'][0],proposal[0])
    np.testing.assert_array_equal(result['C_point_m'],point)
    assert not result['C_uses_proposal'].any()


def test_harmful_and_useful_metrics_use_all_rows_and_same_corrections():
    result=E.metrics(fake_frames(),np.ones(4,bool))
    assert result['harmful_overrides']==dict(A=1,C=0,relative_reduction=1.)
    assert result['useful_corrections']==dict(A=2,C_retained=2,retention=1.)
    assert [result['policies'][p]['error_count'] for p in E.POLICIES]==[2,1,0]


def test_abstain_all_cannot_pass_retention_or_native_benefit():
    frames=fake_frames();frames['C_family']=frames['N_family'].copy()
    frames['C_point_m']=frames['N_point_m'].copy();frames['C_uses_proposal'][:]=False
    frames['harmful_C'][:]=False;frames['retained_useful'][:]=False
    result=E.metrics(frames,np.ones(4,bool));cases={case:dict(windows={'primary':result}) for case in E.CASES}
    gates={x['name']:x for x in E.gate_report(result,cases,PROTOCOL)}
    assert not gates['useful_correction_retention']['passed']
    assert not gates['C_improves_native']['passed']


def test_half_open_window_excludes_stop_and_terminal():
    time=np.array([0.,.995,1.,40.995,41.,44.])
    np.testing.assert_array_equal(E.mask_window(time,[1.,41.]),[False,False,True,True,False,False])


def test_load_ties_missing_and_dominance_are_explicit():
    loads=np.array([[1.,1.,0.,0.],[0.,0.,0.,0.],[9.,1.,0.,0.],[.9,.2,0.,0.]])
    family,eligible,loaded,share=E.dominant(loads,1.,.9)
    np.testing.assert_array_equal(family,[-1,-1,0,-1])
    np.testing.assert_array_equal(eligible,[False,False,True,False])


def test_point_statistics_use_common_finite_policy_mask():
    frames=fake_frames();frames['A_point_m'][0]=np.nan
    result=E.metrics(frames,np.ones(4,bool))['points']['equivalent_raw']['all']
    assert result['eligible_count']==4 and result['matched_count']==3
    assert result['missing_policy']['A']==1
    assert all(result['policies'][p]['count']==3 for p in E.POLICIES)


def test_invalid_oracle_rows_stay_in_denominator_as_failures():
    frames=fake_frames();frames['oracle_geometry_valid'][0]=False;frames['oracle_geometry_pass'][0]=False
    result=E.metrics(frames,np.ones(4,bool))['oracle_geometry']
    assert result['denominator']==4 and result['valid_count']==3 and result['pass_fraction']==.75


def test_point_gate_allowance_is_in_metres_not_millimetres():
    result=E.metrics(fake_frames(),np.ones(4,bool));base=result['points']['equivalent_raw']['all']['policies']['N']['RMS_mm']
    for change,expected in ((.0000005,True),(.000002,False)):
        row=copy.deepcopy(result);row['points']['equivalent_raw']['all']['policies']['C']['RMS_mm']=base+change
        cases={case:dict(windows={'primary':row}) for case in E.CASES}
        gate=next(x for x in E.gate_report(row,cases,PROTOCOL) if x['name']=='point_RMS_mm_C_vs_native_m')
        assert gate['passed'] is expected


def test_empty_risk_denominator_is_not_a_passing_improvement():
    frames=fake_frames();frames['harmful_A'][:]=False;frames['harmful_C'][:]=False
    row=E.metrics(frames,np.ones(4,bool));cases={case:dict(windows={'primary':row}) for case in E.CASES}
    gate=next(x for x in E.gate_report(row,cases,PROTOCOL) if x['name']=='harmful_override_reduction')
    assert not gate['scorable'] and not gate['passed']
