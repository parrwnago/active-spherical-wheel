"""Contracts for interval-to-endpoint output and causal consumer integration."""
from pathlib import Path
import importlib.util
import numpy as np

SPEC=importlib.util.spec_from_file_location('i70_runner',Path(__file__).with_name('run_shadow_case.py'))
M=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(M)


def model_fixture():
    def storage(shape):
        return dict(valid=np.zeros(shape,bool),selected_point_m=np.full(shape+(2,3),np.nan),
            estimated_force_N=np.full(shape+(2,3),np.nan),selected_candidate_index=np.full(shape+(2,),-1,int),
            conditional_point_covariance_m2=np.zeros(shape+(2,3,3)))
    return dict(node_cost=np.zeros(21),edge_cost=np.full((6,2,2),np.inf),node=storage((21,)),edge=storage((6,2,2)),
        force_fit_count=1,failed_force_fit_count=0,shortlist_count=np.ones(4,int),observation_valid=True,
        shortlist_candidate_index=np.zeros((4,2),int))


def test_directed_pair_key_is_endpoint_specific_and_unknown_is_not_a_contact():
    assert M.transition_key(0,2)==(0,1,0)
    assert M.transition_key(2,1)==(0,0,1)
    assert M.transition_key(0,1) is None
    assert M.transition_key(20,2) is None
    assert M.transition_key(0,20) is None


def test_unavailable_transition_uses_current_evidence_and_unknown_remains_finite():
    node=np.arange(21,dtype=float);edge=np.full((6,2,2),np.inf)
    result=M.edge_evidence(node,edge)
    np.testing.assert_array_equal(result,np.broadcast_to(-node,(21,21)))
    edge[0,1,0]=.5;result=M.edge_evidence(node,edge)
    assert result[0,2]==-.5
    assert np.isfinite(result[:,20]).all()


def test_exiting_contact_force_is_retained_without_exporting_its_endpoint_point():
    model=model_fixture();key=(0,1,0);model['edge_cost'][key]=0;model['edge']['valid'][key]=True
    model['edge']['selected_point_m'][key]=[[1,2,3],[4,5,6]]
    model['edge']['estimated_force_N'][key]=[[0,0,10],[0,0,20]]
    model['edge']['selected_candidate_index'][key]=[7,8]
    decision=dict(selected_previous_mode=0,selected_current_mode=2)
    out=M.selected_geometry(model,decision,use_edges=True)
    assert np.isnan(out['points_m'][0]).all()
    np.testing.assert_array_equal(out['points_m'][1],[4,5,6])
    np.testing.assert_array_equal(out['interval_force_by_family_N'][:2],[[0,0,10],[0,0,20]])
    assert out['candidate_index'][1]==8 and out['candidate_index'][0]==-1


def test_node_pair_exports_each_point_on_its_own_family():
    model=model_fixture();mode=8;model['node']['valid'][mode]=True
    model['node']['selected_point_m'][mode]=[[1,0,0],[0,2,0]]
    model['node']['estimated_force_N'][mode]=[[0,0,2],[0,0,3]]
    model['node']['selected_candidate_index'][mode]=[3,4]
    out=M.selected_geometry(model,dict(selected_previous_mode=20,selected_current_mode=mode),use_edges=True)
    np.testing.assert_array_equal(out['points_m'][:2],[[1,0,0],[0,2,0]])
    assert np.isnan(out['points_m'][2:]).all()


def test_consumer_unknown_has_no_fabricated_point_or_force():
    model=model_fixture();model['node_cost'][:20]=np.inf;model['node_cost'][20]=9.
    row,ev=M.ShadowInference().update(0.,model)
    assert row['hmm_support_index']==10
    assert np.isnan(row['hmm_points_m']).all()
    assert np.isnan(row['hmm_interval_force_by_family_N']).all()
    assert ev['node_cost'][20]==9.
