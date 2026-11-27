"""Synthetic geometry-only reduction and frozen-I60 delegation contracts."""
import hashlib
import importlib.util
from pathlib import Path
import unittest

import numpy as np
from spherical_wheel.geometry_first_contact import (geometry_first_rank,legacy_active_mask,select_geometry_points)


FROZEN_PATH=Path(__file__).resolve().parents[1]/'docs/iterations/2026-11-27/velocity-contact-ranking/evidence/portable_tools/rank_stage01/source/contact_hypotheses.py'
assert hashlib.sha256(FROZEN_PATH.read_bytes()).hexdigest()=='c0995d4acb268c73e834c488514aa9f34cd2b3d57aca516c63a394429b93f532'
spec=importlib.util.spec_from_file_location('_i60_frozen_rank_contract',FROZEN_PATH)
frozen=importlib.util.module_from_spec(spec);spec.loader.exec_module(frozen)


def features():
    vector=np.array([[0.,0.,0.],[.01,0.,0.],[.04,0.,0.],[.05,0.,0.]])
    return dict(family_index=np.array([0,0,1,1]),valid=np.ones(4,bool),frame_valid=np.asarray(True),
        excitation_m_s=np.asarray(.2),normal_angle_rad=np.zeros(4),velocity_residual_m_s=vector,
        velocity_residual_norm_m_s=np.linalg.norm(vector,axis=-1),moment_residual_norm=np.zeros(4))


def setup():
    return features(),np.array([[.01,0.,0.],[.04,0.,0.],[.02,0.,0.],[.05,0.,0.]]),np.zeros(3),np.array([20,7,60,9])


class GeometryFirstContracts(unittest.TestCase):
    def test_geometry_representative_ignores_all_finite_cost_values(self):
        f,p,a,ids=setup();expected=select_geometry_points(f,p,a,ids)
        f['normal_angle_rad'][:]=[1.4,0.,.7,0.]
        f['moment_residual_norm'][:]=[100.,0.,50.,0.]
        f['velocity_residual_m_s'][:]=[[9.,0.,0.],[0.,0.,0.],[7.,0.,0.],[0.,0.,0.]]
        f['velocity_residual_norm_m_s']=np.linalg.norm(f['velocity_residual_m_s'],axis=-1)
        actual=select_geometry_points(f,p,a,ids)
        np.testing.assert_array_equal(actual['selected_candidate_id'],expected['selected_candidate_id'])
        np.testing.assert_array_equal(actual['family_distance_squared_m2'],expected['family_distance_squared_m2'])
        np.testing.assert_array_equal(actual['selected_candidate_id'],[20,60,-1,-1])

    def test_distance_is_squared_three_dimensional_world_distance(self):
        f,p,a,ids=setup();p[:2]=[[0.,0.,2.],[0.,1.,0.]]
        out=select_geometry_points(f,p,a,ids)
        self.assertEqual(out['selected_candidate_id'][0],7)
        self.assertEqual(out['family_distance_squared_m2'][0],1.)

    def test_anchor_is_used_directly_without_projection_or_physical_replacement(self):
        f,p,a,ids=setup();anchor=np.array([.03,0.,0.])
        out=select_geometry_points(f,p,anchor,ids)
        self.assertEqual(out['selected_candidate_id'][0],7)
        np.testing.assert_allclose(out['family_distance_squared_m2'][0],.01**2,atol=1e-18)

    def test_exact_distance_tie_uses_original_id_not_input_order(self):
        f,p,a,ids=setup();p[:2]=[[1.,0.,0.],[-1.,0.,0.]]
        out=select_geometry_points(f,p,a,ids)
        self.assertEqual(out['selected_candidate_id'][0],7)
        self.assertEqual(out['selected_row_index'][0],1)

    def test_original_physical_family_is_not_relabelled_to_native_anchor(self):
        f,p,a,ids=setup();a=p[2].copy()
        out=geometry_first_rank(f,p,a,ids,rank_hypotheses=frozen.rank_hypotheses)
        for family,row in enumerate(out['selected_row_index']):
            if row>=0:self.assertEqual(f['family_index'][row],family)
        self.assertEqual(out['ranking']['fused']['chosen_family'],0)
        self.assertEqual(out['selected_candidate_id'][0],20)

    def test_one_candidate_per_family_preserves_every_frozen_rank_field(self):
        f,p,a,ids=setup();keep=np.array([0,2])
        f={key:(value[keep] if key not in ('frame_valid','excitation_m_s') else value) for key,value in f.items()}
        original=frozen.rank_hypotheses(f)
        out=geometry_first_rank(f,p[keep],a,ids[keep],rank_hypotheses=frozen.rank_hypotheses)
        for mode in original:
            for key in original[mode]:np.testing.assert_array_equal(out['ranking'][mode][key],original[mode][key])

    def test_delegate_receives_only_mask_change_and_identical_options(self):
        f,p,a,ids=setup();options=dict(minimum_margin=.25,minimum_excitation_m_s=.02)
        def delegate(reduced,**given):
            self.assertEqual(given,options)
            for key in f:
                if key!='valid':np.testing.assert_array_equal(reduced[key],f[key])
            np.testing.assert_array_equal(reduced['valid'],[True,False,True,False])
            return frozen.rank_hypotheses(reduced,**given)
        geometry_first_rank(f,p,a,ids,rank_hypotheses=delegate,rank_options=options)

    def test_missing_and_nonfinite_anchor_yield_raw_unknown_and_no_acceptance(self):
        for anchor in (None,np.array([np.nan,0.,0.]),np.array([0.,np.inf,0.])):
            f,p,a,ids=setup();out=geometry_first_rank(f,p,anchor,ids,rank_hypotheses=frozen.rank_hypotheses)
            self.assertFalse(out['anchor_available']);self.assertFalse(out['selected_mask'].any())
            np.testing.assert_array_equal(out['selected_candidate_id'],-1)
            for result in out['ranking'].values():
                self.assertEqual(result['chosen_family'],-1);self.assertEqual(result['accepted_family'],-1)

    def test_inactive_nan_points_are_ignored_but_missing_active_point_fails(self):
        f,p,a,ids=setup();f['valid'][0]=False;p[0]=np.nan
        out=select_geometry_points(f,p,a,ids)
        self.assertEqual(out['selected_candidate_id'][0],7)
        f['valid'][0]=True
        with self.assertRaises(ValueError):select_geometry_points(f,p,a,ids)

    def test_low_excitation_preserves_raw_choice_but_not_frozen_acceptance(self):
        f,p,a,ids=setup();f['excitation_m_s']=np.asarray(.001)
        out=geometry_first_rank(f,p,a,ids,rank_hypotheses=frozen.rank_hypotheses)
        self.assertEqual(out['selected_mask'].sum(),2)
        self.assertEqual(out['ranking']['fused']['chosen_family'],0)
        self.assertEqual(out['ranking']['fused']['accepted_family'],-1)

    def test_original_frame_and_finite_feature_eligibility_are_preserved(self):
        f,p,a,ids=setup();f['normal_angle_rad'][0]=np.nan
        out=select_geometry_points(f,p,a,ids)
        self.assertFalse(out['legacy_active_mask'][0]);self.assertEqual(out['selected_candidate_id'][0],7)
        f['frame_valid']=np.asarray(False);p[:]=np.nan
        out=select_geometry_points(f,p,a,ids);self.assertFalse(out['selected_mask'].any())

    def test_norm_validation_cannot_be_hidden_by_unselected_candidate(self):
        f,p,a,ids=setup();f['velocity_residual_norm_m_s'][1]=9.
        with self.assertRaises(ValueError):select_geometry_points(f,p,a,ids)

    def test_single_physical_family_never_becomes_confident_by_duplicates(self):
        f,p,a,ids=setup();f['family_index'][:]=0
        out=geometry_first_rank(f,p,a,ids,rank_hypotheses=frozen.rank_hypotheses)
        self.assertEqual(out['selected_mask'].sum(),1)
        self.assertTrue(out['ranking']['fused']['geometry_forced'])
        self.assertEqual(out['ranking']['fused']['accepted_family'],-1)

    def test_batch_prefix_and_individual_frames_agree_without_future_information(self):
        f,p,a,ids=setup();batch={key:(value if key=='family_index' else np.stack([value]*4)) for key,value in f.items()}
        points=np.stack([p]*4);anchors=np.stack([a]*4)
        full=geometry_first_rank(batch,points,anchors,ids,rank_hypotheses=frozen.rank_hypotheses)
        anchors[2:]=[.2,0.,0.];batch['moment_residual_norm'][2:]+=100.
        changed=geometry_first_rank(batch,points,anchors,ids,rank_hypotheses=frozen.rank_hypotheses)
        single=geometry_first_rank(f,p,a,ids,rank_hypotheses=frozen.rank_hypotheses)
        for key in full:
            if key=='ranking':
                for mode in full[key]:
                    for field in full[key][mode]:
                        np.testing.assert_array_equal(full[key][mode][field][:2],changed[key][mode][field][:2])
                        np.testing.assert_array_equal(full[key][mode][field][0],single[key][mode][field])
            else:
                np.testing.assert_array_equal(full[key][:2],changed[key][:2])
                np.testing.assert_array_equal(full[key][0],single[key])

    def test_no_input_mutation_and_ranker_row_indices_are_not_bank_ids(self):
        f,p,a,ids=setup();before={key:value.tobytes() for key,value in f.items()}
        for value in f.values():value.setflags(write=False)
        p.setflags(write=False);a.setflags(write=False);ids.setflags(write=False)
        out=geometry_first_rank(f,p,a,ids,rank_hypotheses=frozen.rank_hypotheses)
        self.assertEqual(out['ranking']['fused']['candidate_index'][0],0)
        self.assertEqual(out['selected_candidate_id'][0],20)
        self.assertEqual(before,{key:value.tobytes() for key,value in f.items()})

    def test_duplicate_ids_and_incorrect_shapes_are_rejected(self):
        f,p,a,ids=setup();ids[1]=ids[0]
        with self.assertRaises(ValueError):select_geometry_points(f,p,a,ids)
        with self.assertRaises(ValueError):legacy_active_mask(dict(valid=np.zeros(4,bool)))
        with self.assertRaises(ValueError):select_geometry_points(f,p,np.zeros((1,3)),np.arange(4))


if __name__=='__main__':unittest.main(verbosity=2)
