"""I71 synthetic conditioning, causal recurrence and point-consumer contracts."""
import unittest
import numpy as np
import frozen_temporal_contact_filter as frozen
import restricted_filter as restricted
from run_case import infer_packet,selected_geometry,edge_evidence,transition_key


def packet(n=12):
    out=dict(time_s=np.arange(n)*.005,node_cost=np.tile(np.r_[np.arange(8)*.1,np.full(12,-1000.),9.],(n,1)),
        edge_cost=np.full((n,6,2,2),.3),node_valid=np.tile(np.r_[np.ones(8,bool),np.zeros(13,bool)],(n,1)),
        edge_valid=np.ones((n,6,2,2),bool),node_candidate_index=np.full((n,21,2),-1,np.int64),
        edge_candidate_index=np.full((n,6,2,2,2),-1,np.int64),
        node_endpoint_points_m=np.full((n,21,3),np.nan),edge_endpoint_points_m=np.full((n,6,2,2,3),np.nan))
    for mode in range(8):
        family=mode//2;out['node_candidate_index'][:,mode,0]=family
        out['node_endpoint_points_m'][:,mode]=[family,0.,1.]
    for pair,families in enumerate(frozen.SUPPORT_SETS[4:10]):
        for role in range(2):
            for motion in range(2):
                out['edge_candidate_index'][:,pair,role,motion]=families
                out['edge_endpoint_points_m'][:,pair,role,motion]=[families[role],.1,1.]
    return out


def manual_transition(dt):
    """Probability-domain transform, separate from restricted implementation."""
    full=frozen.transition_matrix(dt);out=np.zeros((21,21));allowed=list(range(8))+[20]
    for i in allowed:
        total=sum(full[i,j] for j in allowed if i!=j)
        out[i,i]=full[i,i]
        for j in allowed:
            if i!=j:out[i,j]=(1-full[i,i])*full[i,j]/total
    return out


class RestrictedInferenceContracts(unittest.TestCase):
    def test_initial_prior_is_conditioned_with_relative_preferences_unchanged(self):
        original=frozen.base_prior();actual=restricted.initial_prior()
        expected=original.copy();expected[8:20]=0.;expected/=expected.sum()
        np.testing.assert_array_equal(actual,expected)
        self.assertAlmostEqual(actual[0]/actual[1],9.)
        self.assertAlmostEqual(actual[20],.1/.9)

    def test_original_diagonal_holding_probability_is_exact(self):
        for dt in (.001,.005,.02,.049):
            original=frozen.transition_matrix(dt);actual=restricted.transition_matrix(dt)
            np.testing.assert_array_equal(actual.diagonal()[restricted.ALLOWED_MODES],original.diagonal()[restricted.ALLOWED_MODES])
            np.testing.assert_allclose(actual[restricted.ALLOWED_MODES].sum(axis=1),1.,atol=1e-14,rtol=0)
            np.testing.assert_array_equal(actual[8:20],0.)

    def test_allowed_offdiagonal_relative_odds_are_preserved(self):
        original=frozen.transition_matrix(.005);actual=restricted.transition_matrix(.005)
        for i in restricted.ALLOWED_MODES:
            destinations=[j for j in restricted.ALLOWED_MODES if j!=i]
            ratios=actual[i,destinations]/original[i,destinations]
            np.testing.assert_allclose(ratios,np.full(len(ratios),ratios[0]),atol=1e-14,rtol=0.)
        self.assertGreater(original[0,8:20].sum(),0.)
        self.assertGreater(actual[0,2],original[0,2])

    def test_log_filter_matches_independent_probability_recurrence(self):
        estimator=restricted.RestrictedTemporalFilter();p=np.r_[np.tile([.18,.02],4),np.zeros(12),.1]/.9
        for k in range(15):
            evidence=-.1*np.arange(21)+np.sin(np.arange(21)+k)*.3;evidence[8:20]=20.;evidence[20]=-3.
            edge=np.broadcast_to(evidence,(21,21)).copy();edge[0,2]+=1.5
            e=edge.copy();e[:,8:20]=-np.inf;e[8:20,:]=-np.inf
            masked=evidence.copy();masked[8:20]=-np.inf
            if k==0:p*=np.exp(.25*masked)
            else:p=np.sum(p[:,None]*manual_transition(.005)*np.exp(.25*e),axis=0)
            p/=p.sum()
            actual=estimator.update(k*.005,evidence,edge)
            np.testing.assert_allclose(actual['probability'],p,atol=1e-14,rtol=0.)
            self.assertIn(actual['selected_current_mode'],list(restricted.ALLOWED_MODES))

    def test_disabled_endpoints_have_exact_zero_probability_in_every_control(self):
        out=infer_packet(packet())
        for key in ('emission_probability21','state_hmm_probability21','posterior21'):
            np.testing.assert_array_equal(out[key][:,8:20],0.)
            np.testing.assert_allclose(out[key].sum(axis=1),1.,atol=1e-14,rtol=0.)
        for prefix in ('emission','state_hmm','hmm'):
            self.assertFalse(np.isin(out[prefix+'_selected_current_mode'],np.arange(8,20)).any())

    def test_frame_result_matches_original_model_with_pair_potentials_masked(self):
        evidence=-packet(1)['node_cost'][0];masked=evidence.copy();masked[8:20]=-np.inf
        original=frozen.frame_sequence(masked[None])
        actual=restricted.frame_update(evidence)
        np.testing.assert_allclose(actual['probability'],original['probability'][0],atol=1e-15,rtol=0.)

    def test_direct_singleton_edge_retains_frozen_interval_potential(self):
        data=packet(1);data['edge_cost'][0,0,1,0]=.0123
        edge=edge_evidence(data['node_cost'][0],data['edge_cost'][0])
        self.assertEqual(edge[0,2],-.0123)
        self.assertEqual(edge[0,1],-data['node_cost'][0,1])
        self.assertEqual(edge[20,2],-data['node_cost'][0,2])
        self.assertTrue(np.isneginf(edge[:,8:20]).all())

    def test_edge_point_is_same_complete_fit_endpoint_role(self):
        data=packet(1);data['edge_candidate_index'][0,0,1,0]=[6,7]
        data['edge_endpoint_points_m'][0,0,1,0]=[.25,.125,-.12]
        decision=dict(selected_previous_mode=0,selected_current_mode=2)
        actual=selected_geometry(data,0,decision,use_edges=True)
        np.testing.assert_array_equal(actual['points_m'][1],[.25,.125,-.12])
        self.assertEqual(actual['candidate_index'][1],7)
        self.assertEqual(actual['selected_edge_index'],2)
        ordinary=selected_geometry(data,0,decision,use_edges=False)
        np.testing.assert_array_equal(ordinary['points_m'][1],data['node_endpoint_points_m'][0,2])

    def test_same_family_motion_switch_uses_node_not_union_edge(self):
        self.assertIsNone(transition_key(0,1));self.assertIsNone(transition_key(20,0))
        data=packet(1);decision=dict(selected_previous_mode=0,selected_current_mode=1)
        actual=selected_geometry(data,0,decision,use_edges=True)
        self.assertEqual(actual['selected_edge_index'],-1)
        self.assertEqual(actual['candidate_index'][0],0)

    def test_prefix_unchanged_by_future_costs_or_points(self):
        data=packet(15);prefix={key:value[:6].copy() for key,value in data.items()}
        a=infer_packet(prefix);data['node_cost'][6:,:8]+=np.arange(8)[None,:]*30.
        data['edge_endpoint_points_m'][6:]+=10.
        b=infer_packet(data)
        for key in a:np.testing.assert_array_equal(a[key],b[key][:6])

    def test_gap_reset_preserves_conditioned_prior_and_rejects_duplicate_time(self):
        a=restricted.RestrictedTemporalFilter();e=-np.arange(21,dtype=float);e[20]=-2.
        a.update(0.,e);a.update(.005,e);after=a.update(.1,e)
        b=restricted.RestrictedTemporalFilter();fresh=b.update(.1,e)
        np.testing.assert_array_equal(after['probability'],fresh['probability'])
        self.assertTrue(after['history_reset']);self.assertEqual(after['selected_previous_mode'],-1)
        with self.assertRaises(ValueError):a.update(.1,e)

    def test_unknown_remains_unknown_without_fabricated_point(self):
        data=packet(1);data['node_cost'][0,:8]=1000.;data['node_cost'][0,20]=0.
        out=infer_packet(data)
        for prefix in ('emission','state_hmm','hmm'):
            self.assertEqual(out[prefix+'_selected_current_mode'][0],20)
            self.assertTrue(np.isnan(out[prefix+'_points_m'][0]).all())
            np.testing.assert_array_equal(out[prefix+'_candidate_index'][0],-1)

    def test_invalid_family_or_fit_metadata_is_rejected(self):
        data=packet(2);data['node_candidate_index'][0,0,0]=1
        with self.assertRaises(ValueError):infer_packet(data)
        data=packet(2);data['edge_valid'][0,0,0,0]=False
        with self.assertRaises(ValueError):infer_packet(data)

    def test_no_input_mutation(self):
        data=packet(3);before={k:v.tobytes() for k,v in data.items()};infer_packet(data)
        self.assertEqual(before,{k:v.tobytes() for k,v in data.items()})


if __name__=='__main__':unittest.main(verbosity=2)
