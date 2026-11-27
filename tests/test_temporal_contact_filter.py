"""Causal probability/timing contracts; independent of any saved robot labels."""
import numpy as np
import pytest
from spherical_wheel.temporal_contact_filter import (FilterSettings,TemporalContactFilter,
    base_prior,filter_sequence,frame_sequence,summarize,transition_matrix)


def test_prior_and_transition_are_normalized_for_irregular_times():
    p=base_prior();assert p.shape==(21,);np.testing.assert_allclose(p.sum(),1.)
    for dt in [.001,.005,.017,.1]:
        m=transition_matrix(dt);np.testing.assert_allclose(m.sum(axis=1),1.,atol=1e-14)
        assert np.all(m>=0)
    assert transition_matrix(.005)[0,0]>transition_matrix(.017)[0,0]


def test_support_probability_adds_slip_and_rolling_without_averaging_bodies():
    p=np.zeros(21);p[0]=.3;p[1]=.25;p[2]=.45
    out=summarize(p);assert out['support_index']==0
    assert out['confidence']==.55
    np.testing.assert_allclose(out['family_inclusion_probability'],[.55,.45,0,0])


def test_missing_all_known_evidence_returns_unknown_without_nan():
    e=np.full(21,-np.inf);e[20]=0
    out=TemporalContactFilter().update(0.,e)
    assert out['support_index']==10 and out['unknown_probability']==1.


def test_first_update_matches_independent_bayes_calculation():
    e=-np.arange(21)/3.;p=base_prior()*np.exp(.25*e);p/=p.sum()
    out=TemporalContactFilter().update(0.,e)
    np.testing.assert_allclose(out['probability'],p,rtol=1e-14,atol=1e-16)


def test_transition_observation_can_distinguish_same_current_evidence():
    estimator=TemporalContactFilter();e=np.full(21,-100.);e[0]=0.;e[20]=-20.
    estimator.update(0.,e)
    current=np.full(21,-100.);current[0]=current[2]=0.;current[20]=-20.
    edge=np.broadcast_to(current,(21,21)).copy();edge[:,0]-=100.;edge[0,2]=10.
    out=estimator.update(.005,current,edge)
    assert out['support_index']==1


def test_constant_evidence_retains_a_contact_but_overwhelming_new_evidence_switches():
    est=TemporalContactFilter();e=np.full(21,-50.);e[0]=0.;e[20]=-20.
    for t in np.arange(10)*.005:assert est.update(t,e)['support_index']==0
    e[0]=-100.;e[2]=0.
    assert est.update(.05,e)['support_index']==1


def test_prefix_is_invariant_to_arbitrary_future_measurements():
    rng=np.random.default_rng(70);e=rng.normal(size=(25,21));times=np.arange(25)*.005
    a=filter_sequence(times,e);e[10:]=rng.normal(0,100,size=(15,21));b=filter_sequence(times,e)
    np.testing.assert_array_equal(a['probability'][:10],b['probability'][:10])
    np.testing.assert_array_equal(a['probability'][:10],filter_sequence(times[:10],e[:10])['probability'])


def test_nonmonotonic_or_duplicate_time_is_not_a_second_measurement():
    est=TemporalContactFilter();est.update(.01,np.zeros(21))
    with pytest.raises(ValueError):est.update(.01,np.zeros(21))
    with pytest.raises(ValueError):est.update(.005,np.zeros(21))


def test_large_gap_resets_history_to_same_first_update():
    e=-np.arange(21,dtype=float);est=TemporalContactFilter();est.update(0.,e)
    out=est.update(.1,-e)
    reference=TemporalContactFilter().update(.1,-e)
    assert out['history_reset'];np.testing.assert_array_equal(out['probability'],reference['probability'])


def test_invalid_evidence_does_not_silently_create_confidence():
    for value in [np.nan,np.inf]:
        e=np.zeros(21);e[3]=value
        with pytest.raises(ValueError):TemporalContactFilter().update(0.,e)
    edge=np.zeros((21,21));edge[0,20]=-np.inf
    with pytest.raises(ValueError):TemporalContactFilter().update(0.,np.zeros(21),edge)


def test_frame_control_matches_repeated_fresh_filters():
    e=np.arange(63,dtype=float).reshape(3,21)*-.1
    actual=frame_sequence(e)
    for i in range(3):np.testing.assert_array_equal(actual['probability'][i],TemporalContactFilter().update(0.,e[i])['probability'])


def test_settings_reject_invalid_priors():
    with pytest.raises(ValueError):FilterSettings(unknown_prior_mass=.2)
    with pytest.raises(ValueError):FilterSettings(known_dwell_s=0)
