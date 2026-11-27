"""Synthetic contracts for the fixed I-62 tolerance/selection mechanism."""
import copy,importlib.util,json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_i62_sweep',HERE/'tolerance_sweep.py');S=importlib.util.module_from_spec(spec);spec.loader.exec_module(S)
P=json.loads((HERE.parent/'protocol/protocol.json').read_text())


def example():
    return dict(time_s=np.arange(4)*.005+1.,gate_valid=np.array([True,True,True,False]),
        joint_max_abs_Nm=np.array([0.,.02,.08,0.]),radius_m=np.full(4,.12),force_norm_N=np.full(4,10.),
        i60_accepted=np.ones(4,bool),N_family=np.array([0,0,1,1]),A_family=np.array([1,1,1,0]),
        truth_family=np.array([0,1,1,0]),primary_eligible=np.ones(4,bool),point_mask=np.ones(4,bool),
        native_distance_m=np.array([.01,.01,.01,.01]),proposed_distance_m=np.array([.02,.001,.001,.001]),
        oracle_valid=np.ones(4,bool),oracle_joint_max_abs_Nm=np.array([0.,.02,.08,0.]),oracle_mask=np.ones(4,bool),
        i61_keep_proposal=np.array([True,True,True,False]),i61_C_family=np.array([1,1,1,1]),
        i61_distance_m=np.array([.02,.001,.001,.01]))


def tiny_grid():
    return dict(a_index=np.array([0,6,20]),b_index=np.array([0,20,0]),
        a_Nm=np.array([0.,.03,.10]),b_dimensionless=np.array([0.,.10,0.]))


def test_zero_threshold_accepts_only_exact_zero_valid_residual():
    np.testing.assert_array_equal(S.keep_proposal(example(),0.,0.),[True,False,False,False])


def test_off_is_A_even_when_load_feature_is_invalid():
    data=example();np.testing.assert_array_equal(S.keep_proposal(data,None,None,off=True),data['i60_accepted'])


def test_grid_includes_original_I61_pair_with_registered_integer_construction():
    grid=S.parameter_grid(P);assert len(grid['a_index'])==3721
    selected=(grid['a_index']==6)&(grid['b_index']==20)
    assert selected.sum()==1 and grid['a_Nm'][selected][0]==.03 and grid['b_dimensionless'][selected][0]==.10


def test_chunked_scan_matches_scalar_policy_metrics_including_percentile():
    data=example();grid=tiny_grid();many=S.scan_group(data,grid,chunk=2)
    for i in range(3):
        keep=S.keep_proposal(data,grid['a_Nm'][i],grid['b_dimensionless'][i]);one=S.score_keeps(data,keep)
        for key in one:np.testing.assert_array_equal(many[key][i:i+1],one[key])


def test_same_family_accepted_geometry_change_is_scored():
    data=example();data['A_family']=data['N_family'].copy()
    keep=np.array([False,True,True,False]);score=S.score_keeps(data,keep)
    assert score['point_SSE_mm2'][0]==202.


def test_nonaccepted_invalid_proposal_does_not_exclude_native_fallback_point():
    data=example();data['i60_accepted'][0]=False;data['proposed_distance_m'][0]=np.nan
    keep=S.keep_proposal(data,0.,0.,off=True);score=S.score_keeps(data,keep)
    assert np.isfinite(score['point_RMS_mm'][0]) and data['point_mask'].sum()==4


def test_lexicographic_selection_uses_errors_sse_veto_then_indices():
    grid=dict(a_index=np.array([1,0,0]),b_index=np.array([0,2,1]),a_Nm=np.array([.005,0.,0.]),b_dimensionless=np.array([0.,.01,.005]))
    metrics=dict(error_count=np.array([1,1,1]),point_SSE_mm2=np.array([2.,2.,2.]),veto_count=np.array([3,3,3]))
    chosen=S.select_candidate(metrics,np.ones(3,bool),grid);assert chosen['grid_index']==2
    metrics['point_SSE_mm2'][0]=1.;assert S.select_candidate(metrics,np.ones(3,bool),grid)['grid_index']==0


def test_no_feasible_selection_is_explicit_off_and_failed():
    result=S.select_candidate(dict(error_count=np.zeros(3,int)),np.zeros(3,bool),tiny_grid())
    assert result['selection_failed'] and result['policy']=='off' and result['grid_index'] is None


def test_zero_harm_phase_is_flagged_but_pooled_training_benefit_is_not_invented():
    data=example();data['A_family']=data['truth_family'].copy()
    data['N_family']=data['truth_family'].copy();base=S.constants(data)
    metrics=S.scan_group(data,tiny_grid());feasible,gates=S.training_feasibility(metrics,base,{'phase':metrics},{'phase':base},P)
    assert not gates['harmful_removal'].any() and not feasible.any()
    assert gates['phase_harmful_nonincrease'].all()


def test_original_policy_benchmark_compares_against_stored_I61_labels():
    data=example();grid=S.parameter_grid(P);metrics=S.scan_group(data,grid)
    result=S.verify_benchmarks({'synthetic':data},grid,{'synthetic':metrics})
    assert result['synthetic']['I61_families_exact']
    assert result['synthetic']['original_I61_C_primary_error_count']==2


def test_benchmark_roundoff_allowance_is_unit_specific_and_does_not_replace_values():
    checked=S.benchmark_float_check('point_SSE_mm2',50000.+2e-10,50000.)
    assert checked['passes'] and checked['batched']!=checked['direct']
    assert not S.benchmark_float_check('point_RMS_mm',3.+2e-10,3.)['passes']
    assert S.benchmark_float_check('point_RMS_mm',3.+5e-15,3.)['passes']
    assert not S.benchmark_float_check('point_SSE_mm2',50000.+1e-5,50000.)['passes']
