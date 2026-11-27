"""Synthetic contracts only; no historical feature arithmetic."""
import numpy as np
import condition_diagnostic as D


def example():
    return dict(time_s=np.arange(4)*.005+1.,gate_valid=np.array([True,True,True,False]),
        joint_max_abs_Nm=np.array([0.,.02,.08,0.]),radius_m=np.full(4,.12),force_norm_N=np.full(4,10.),
        i60_accepted=np.ones(4,bool),N_family=np.array([0,0,1,1]),A_family=np.array([1,1,1,0]),
        truth_family=np.array([0,1,1,0]),primary_eligible=np.ones(4,bool),point_mask=np.ones(4,bool),
        native_distance_m=np.full(4,.01),proposed_distance_m=np.array([.02,.001,.001,.001]),
        oracle_valid=np.ones(4,bool),oracle_mask=np.ones(4,bool),
        combined_risk_r=np.array([0.,.3,.7,np.nan]),risk_valid=np.array([True,True,True,False]),
        phase_index=np.zeros(4,dtype=np.int64))


def pair():
    d=example();d={k:v[:2].copy() for k,v in d.items()}
    d.update(joint_max_abs_Nm=np.array([.17,.30]),force_norm_N=np.array([2.2/.12,2.4/.12]),
        N_family=np.array([0,1]),A_family=np.array([1,0]),truth_family=np.array([0,0]),
        combined_risk_r=np.array([0.,1.]),risk_valid=np.ones(2,bool))
    return d


def test_c_zero_exactly_old_gate_even_for_unknown_risk():
    d=example()
    np.testing.assert_array_equal(D.keep(d,0.,0.,0.),D.metrics.keep_proposal(d,0.,0.))
    d['combined_risk_r'][:]=np.nan;d['risk_valid'][:]=False
    np.testing.assert_array_equal(D.keep(d,.03,.10,0.),D.metrics.keep_proposal(d,.03,.10))


def test_positive_c_unknown_risk_preserves_A_even_with_invalid_load():
    np.testing.assert_array_equal(D.keep(example(),0.,0.,.1),[True,True,False,True])


def test_known_risk_invalid_load_reverts_native():
    d=example();d['gate_valid'][1]=False
    assert not D.keep(d,1.,1.,1.)[1]
    d['risk_valid'][1]=False;d['combined_risk_r'][1]=np.nan
    assert D.keep(d,1.,1.,1.)[1]


def test_off_preserves_A_and_never_adds_nonaccepted_proposals():
    d=example();d['i60_accepted'][0]=False
    np.testing.assert_array_equal(D.keep(d,None,None,None,off=True),d['i60_accepted'])
    assert not D.keep(d,100.,100.,100.)[0]


def test_dynamic_risk_can_break_old_dominance_without_proving_acceptance():
    d=pair();old=D.exact_dominance(d,False);new=D.exact_dominance(d,True)
    assert old['report']['forced_useful_loss_lower_bound']==1
    assert new['report']['forced_useful_loss_lower_bound']==0
    assert old['forced_mask'].dtype==bool and new['forced_mask'].dtype==bool


def test_equal_risk_does_not_break_structural_dominance():
    d=pair();d['combined_risk_r'][:]=1.
    r=D.exact_dominance(d,True)
    assert r['report']['forced_useful_loss_lower_bound']==1
    assert all(v>=0 for v in r['report']['witnesses'][0]['margins_as_float'])


def test_unknown_useful_not_forced_unknown_unsafe_is_unrejectable():
    d=pair();d['risk_valid'][1]=False;d['combined_risk_r'][1]=np.nan
    assert D.exact_dominance(d,True)['report']['forced_useful_loss_lower_bound']==0
    d=pair();d['risk_valid'][0]=False;d['combined_risk_r'][0]=np.nan
    assert D.exact_dominance(d,True)['report']['unsafe_unknown_risk_count']==1


def test_fixed_schedule_size_and_controls_excluded_from_selection():
    options=D.schedules();assert len(options)==20 and sum(o['eligible_for_fit'] for o in options)==18
    scores=dict(error_count=np.ones(20,int),point_SSE_mm2=np.ones(20),veto_count=np.ones(20,int))
    scores['error_count'][18:]=0
    assert D.select(scores,np.ones(20,bool),options)['schedule_index']==0


def test_selection_order_and_failed_fit_are_explicit():
    options=D.schedules();scores=dict(error_count=np.ones(20,int),point_SSE_mm2=np.ones(20),veto_count=np.ones(20,int))
    assert D.select(scores,np.zeros(20,bool),options)==dict(selection_failed=True,policy='off',schedule_index=None)
    scores['point_SSE_mm2'][7]=0
    assert D.select(scores,np.ones(20,bool),options)['schedule_index']==7


def test_event_clusters_use_full_rate_adjacency_and_phase():
    d=example();d['time_s']=np.array([1.,1.005,1.015,1.02]);d['phase_index'][-1]=1
    c=D.clusters(d,np.ones(4,bool))
    assert [row['frame_count'] for row in c]==[2,1,1]


def test_unknown_rows_are_scored_as_policy_A_not_removed():
    d=example();kept=D.keep(d,0.,0.,.1);scores=D.metrics.score_keeps(d,kept)
    assert len(kept)==4 and scores['retained_proposal_count'][0]==3
    assert np.isfinite(scores['point_RMS_mm'][0])


def test_zero_risk_equality_is_a_valid_nonnegative_sign_certificate():
    d=pair();d['combined_risk_r'][:]=0.
    r=D.exact_dominance(d,True)
    assert r['report']['forced_useful_loss_lower_bound']==1
    assert r['report']['witnesses'][0]['margins_as_float'][2]==0.


def test_feature_force_never_overwrites_inherited_policy_force():
    d=example();old=d['force_norm_N'].copy()
    feature=dict(time_s=d['time_s'].copy(),force_norm_N=np.nextafter(old,np.inf),new_signal=np.ones(4))
    out=D.merge_features(d,feature)
    np.testing.assert_array_equal(out['force_norm_N'],old)
    np.testing.assert_array_equal(out['feature_force_norm_N'],feature['force_norm_N'])


def test_old_control_parity_checks_frozen_flags_and_points():
    d=example();old=D.keep(d,.03,.10,0.)
    d['i61_keep_proposal']=old.copy();d['i61_C_family']=np.where(old,d['A_family'],d['N_family'])
    d['i61_distance_m']=np.where(old,d['proposed_distance_m'],d['native_distance_m'])
    assert D.verify_controls({'m100_B':d})['m100_B']['I61_flags_exact']
    d['i61_keep_proposal'][0]=~d['i61_keep_proposal'][0]
    try:D.verify_controls({'m100_B':d})
    except ValueError:pass
    else:raise AssertionError('Corrupt inherited control was accepted')


def test_prior_witness_set_checks_identities_not_just_count():
    old=[dict(case='m100_B',time_s=1.005)]
    assert D.match_old_witnesses(old,[dict(case='m100_B',time_s=1.0050000000001)])['count']==1
    try:D.match_old_witnesses(old,[dict(case='p030_B',time_s=1.005)])
    except ValueError:pass
    else:raise AssertionError('Different prior witness identity was accepted')
