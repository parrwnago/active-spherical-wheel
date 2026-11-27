"""Synthetic I-64 analysis contracts; never reads original numeric inputs."""
import json
from pathlib import Path
import numpy as np
import history_diagnostic as D

P=json.loads((Path(__file__).resolve().parent.parent/'protocol/protocol.json').read_text())


def basic(n=4):
    d=dict(time_s=np.arange(n)*.005,force_norm_N=np.full(n,10.),radius_m=np.full(n,.12),
        i60_accepted=np.ones(n,bool),N_family=np.zeros(n,int),A_family=np.ones(n,int),truth_family=np.ones(n,int),
        primary_eligible=np.ones(n,bool),point_mask=np.ones(n,bool),native_distance_m=np.full(n,.01),
        proposed_distance_m=np.full(n,.001),H_F_valid=np.ones(n,bool),H_Nalt_valid=np.ones(n,bool),H_Neq_valid=np.ones(n,bool))
    for model in D.MODELS:
        for branch in ('F','Nalt'):
            d[model+'_'+branch+'_score']=np.full(n,.5 if branch=='F' else 1.)
            d[model+'_'+branch+'_score_valid']=np.ones(n,bool)
    return d


def test_unknown_history_retains_A_but_invalid_current_map_never_does():
    d=basic();d['M2_F_score_valid'][:]=False;d['M2_F_score'][:]=np.nan
    np.testing.assert_array_equal(D.shadow_keep(d,'M2'),np.ones(4,bool))
    d['H_F_valid'][1]=False;assert not D.shadow_keep(d,'M2')[1]
    d['i60_accepted'][2]=False;assert not D.shadow_keep(d,'M2')[2]


def test_unfiltered_models_have_no_history_fallback():
    d=basic();d['M0_F_score_valid'][0]=False;d['M1_F_score_valid'][0]=False
    assert not D.shadow_keep(d,'M0')[0] and not D.shadow_keep(d,'M1')[0]


def test_inclusive_component_bound_is_preserved():
    d=basic();d['M2_F_score'][:]=[1.,np.nextafter(1.,2.),.5,.9]
    np.testing.assert_array_equal(D.shadow_keep(d,'M2'),[True,False,True,True])


def test_ties_and_unknowns_are_distinct():
    eps=1e-9;F=np.array([0.,0.,0.,np.nan]);N=np.array([2*eps,-2*eps,eps,0.])
    np.testing.assert_array_equal(D.classify_pair(F,N,np.array([1,1,1,0],bool),eps),[-1,1,0,-2])


def test_vector_RMS_is_not_component_RMS():
    stats=D.statistics(np.array([[3.,4.,0.],[0.,0.,0.]]),np.ones(2,bool))
    assert stats['vector_RMS_Nm']==np.sqrt(12.5)
    np.testing.assert_allclose(stats['per_joint_RMS_Nm'],[np.sqrt(4.5),np.sqrt(8.),0.])


def ranking_data(old_h,new_h,old_u,new_u):
    d=basic(300);d['truth_family'][:]=0;d['A_family'][:]=0;d['N_family'][:]=1
    d['N_family'][:6]=0;d['A_family'][:6]=1
    for model,h,u in [('M0',old_h,old_u),('M2',new_h,new_u)]:
        F=d[model+'_F_score'];N=d[model+'_Nalt_score'];F[:]=1.;N[:]=.5
        F[:6]=.5;N[:6]=1.;F[:h]=1.;N[:h]=.5
        F[6:6+u]=.5;N[6:6+u]=1.
    return d,dict(harmful=np.arange(300)<6,useful=np.arange(300)>=6)


def test_Q2_full_denominators_require_265_useful_not_264():
    d,g=ranking_data(2,3,263,264);r=D.qualify_Q2(d,g,P)
    assert not r['gates']['useful_90pct_correct'] and not r['promising']
    d,g=ranking_data(2,3,264,265);assert D.qualify_Q2(d,g,P)['promising']


def test_Q2_improvement_cannot_hide_loss_of_correct_harmful_decisions():
    d,g=ranking_data(6,3,200,294);r=D.qualify_Q2(d,g,P)
    assert not r['gates']['harmful_nonregression'] and not r['promising']


def test_Q2_if_all_harmful_already_correct_requires_useful_improvement():
    d,g=ranking_data(6,6,265,265);assert not D.qualify_Q2(d,g,P)['promising']
    d,g=ranking_data(6,6,265,266);assert D.qualify_Q2(d,g,P)['promising']


def test_unknown_pair_is_not_dropped_from_group_denominator():
    d=basic();d['M2_Nalt_score_valid'][0]=False
    report=D.pair_report(d,np.ones(4,bool),'M2',1e-9)
    assert report['denominator']==4 and report['known']==3 and report['F_preferred']==3 and report['unknown']==1


def test_history_family_truth_subset_uses_previous50ms_and_no_window_leak():
    d=basic(12);d['truth_family'][:]=d['A_family'];d['primary_eligible'][:10]=False
    assert D.history_family_correct(d)[10]
    d['truth_family'][0]=-1;assert not D.history_family_correct(d)[10]


def test_event_clusters_distinguish_gaps_from_adjacent_frames():
    t=np.array([1.,1.005,1.015]);c=D.clusters(t,np.ones(3,bool))
    assert [x['frame_count'] for x in c]==[2,1]


def test_features_cannot_overwrite_frozen_policy_force_radius():
    d=basic();f=dict(time_s=d['time_s'].copy(),force_norm_N=np.full(4,20.),radius_m=np.full(4,.24))
    p=dict(time_s=d['time_s'].copy(),observer_generalized=np.zeros((4,9)),observer_interval_mean=np.zeros((4,9)))
    for key in ('H_F_valid','H_Nalt_valid','H_Neq_valid','I61_F_valid','I61_Nalt_valid','I61_C_uses_proposal',
        'material_F_valid','material_Nalt_valid','material_Neq_valid'):p[key]=np.ones(4,bool)
    for key in ('I61_F_residual_Nm','I61_Nalt_residual_Nm'):p[key]=np.zeros((4,3))
    for model in D.MODELS:
        for branch in ('F','Nalt'):
            f[model+'_'+branch+'_residual_Nm']=np.zeros((4,3));f[model+'_'+branch+'_valid']=np.ones(4,bool)
    out=D.join(d,f,p)
    np.testing.assert_array_equal(out['force_norm_N'],d['force_norm_N'])
    np.testing.assert_array_equal(out['radius_m'],d['radius_m'])
    np.testing.assert_array_equal(out['feature_radius_m'],f['radius_m'])
