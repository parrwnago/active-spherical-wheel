"""Synthetic parity with the frozen I60 functions; no historical generation."""
from pathlib import Path
import importlib.util,unittest
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value
B=module(HERE/'bank_points.py','_i67_test_bank')
OLD=module(ROOT/'results/velocity_contact_rank_20261127/campaign/features_stage01/m100_B/source/contact_hypotheses.py','_i67_original_geometry')


def pose():
    theta=.37;axis=np.array([np.cos(theta),np.sin(theta),0.])
    seam=.11565465836
    return dict(centre_m=np.array([.2,-.1,.3]),patch_axis_world=np.array([axis,-axis,axis,-axis]),
        patch_lower_m=np.array([.01,.01,seam,seam]),patch_upper_m=np.array([seam,seam,.12,.12]),radius_m=np.asarray(.12),valid=True)


def original_points(p,hint):
    normal=OLD._unit(hint);seeds=OLD.normal_seeds(normal);points=[]
    for seed in seeds:
        for slot,family in enumerate([0,1,2,3,0,1]):
            lower=p['patch_lower_m'][family];upper=lower if slot>=4 else p['patch_upper_m'][family]
            offset=OLD._band_projection(-float(p['radius_m'])*seed,p['patch_axis_world'][family],float(p['radius_m']),lower,upper)
            points.append(p['centre_m']+offset)
    return np.asarray(points)


class BankContracts(unittest.TestCase):
    def test_all102_synthetic_coordinates_match_original_scalar_operations_exactly(self):
        p=pose()
        for hint in [[0.,0.,1.],[.3,-.5,2.],[-1.,.01,.1],[1.,1.,1.]]:
            actual=B.reconstruct_bank(p,np.asarray(hint))['points_m'];expected=original_points(p,np.asarray(hint))
            np.testing.assert_array_equal(actual,expected)
            self.assertEqual(actual.tobytes(),expected.tobytes())

    def test_seed_vectors_match_frozen_source_bit_for_bit(self):
        for hint in [[0.,0.,1.],[.3,.7,2.],[1.,1.,1.]]:
            normal=OLD._unit(hint)
            np.testing.assert_array_equal(B.normal_seeds(normal),OLD.normal_seeds(normal))

    def test_original_id_family_and_rim_kind_layout(self):
        layout=B.candidate_layout();np.testing.assert_array_equal(layout['candidate_index'],np.arange(102))
        np.testing.assert_array_equal(layout['family_index'],np.tile([0,1,2,3,0,1],17))
        np.testing.assert_array_equal(layout['kind'],np.tile([0,0,0,0,1,1],17))
        self.assertEqual(layout['seed_index'][101],16);self.assertEqual(layout['slot'][101],5)

    def test_selected_ids_equal_same_bank_slots_and_unknown_is_not_guessed(self):
        p=pose();hint=np.array([.3,.1,1.]);bank=B.reconstruct_bank(p,hint)
        ids=np.array([0,4,11,67,101,-1]);selected=B.reconstruct_selected(p,hint,ids)
        np.testing.assert_array_equal(selected[:-1],bank['points_m'][ids[:-1]])
        self.assertTrue(np.isnan(selected[-1]).all())

    def test_rims_remain_original_shell_families_at_the_lower_band(self):
        p=pose();bank=B.reconstruct_bank(p,[.1,.2,1.]);rim=bank['kind']==1
        for index in np.flatnonzero(rim):
            family=bank['family_index'][index]
            self.assertIn(family,[0,1]);self.assertAlmostEqual(bank['axial_coordinate_m'][index],p['patch_lower_m'][family],places=14)

    def test_projection_keeps_sphere_radius_and_frozen_patch_bands(self):
        p=pose();bank=B.reconstruct_bank(p,[.1,.2,1.])
        np.testing.assert_allclose(np.linalg.norm(bank['points_m']-p['centre_m'],axis=1),.12,atol=1e-15)
        self.assertTrue(bank['within_band'].all())

    def test_pre_admission_geometry_is_not_silently_called_usable(self):
        p=pose();bank=B.reconstruct_bank(p,[0.,0.,1.],observer_force_N=[0.,0.,10.])
        self.assertTrue(bank['reconstructed'].all());self.assertFalse(bank['original_geometry_valid'].all())
        self.assertTrue((bank['normal_angle_rad']>B.DEFAULT_CONE_RAD).any())

    def test_original_frame_and_compression_failures_remain_invalid(self):
        p=pose();hint=np.array([0.,0.,1.])
        for force,frame in [([0.,0.,.5],True),([0.,0.,-10.],True),([0.,0.,10.],False)]:
            bank=B.reconstruct_bank(p,hint,observer_force_N=force,original_frame_valid=frame)
            self.assertFalse(bank['original_geometry_valid'].any())

    def test_mask_only_skips_slots_and_does_not_change_requested_coordinates(self):
        p=pose();hint=np.array([.1,.2,1.]);mask=np.arange(102)%7==0
        all_points=B.reconstruct_bank(p,hint);some=B.reconstruct_bank(p,hint,candidate_mask=mask)
        np.testing.assert_array_equal(some['points_m'][mask],all_points['points_m'][mask])
        self.assertTrue(np.isnan(some['points_m'][~mask]).all());np.testing.assert_array_equal(some['reconstructed'],mask)

    def test_pole_projection_degeneracy_matches_original_fallback(self):
        p=pose();hint=-p['patch_axis_world'][0]
        actual=B.reconstruct_bank(p,hint)['points_m'];expected=original_points(p,hint)
        np.testing.assert_array_equal(actual,expected);self.assertTrue(np.isfinite(actual).all())

    def test_invalid_hint_or_pose_cannot_reuse_previous_points(self):
        p=pose()
        for hint in [[0.,0.,0.],[np.nan,0.,1.]]:self.assertFalse(B.reconstruct_bank(p,hint)['reconstructed'].any())
        p['valid']=False;self.assertFalse(B.reconstruct_bank(p,[0.,0.,1.])['reconstructed'].any())

    def test_invalid_ids_and_shapes_raise(self):
        p=pose()
        for ids in [[102],[1.5]]:
            with self.subTest(ids=ids),self.assertRaises(ValueError):B.reconstruct_selected(p,[0.,0.,1.],ids)
        with self.assertRaises(ValueError):B.reconstruct_bank(p,[0.,0.,1.],candidate_mask=np.ones(101,bool))

    def test_no_inputs_or_frozen_masks_are_mutated(self):
        p=pose();before={key:value.copy() if isinstance(value,np.ndarray) else value for key,value in p.items()}
        normal=np.array([.2,.3,1.]);copy_normal=normal.copy();mask=np.arange(102)%2==0;copy_mask=mask.copy()
        B.reconstruct_bank(p,normal,observer_force_N=np.array([0.,0.,10.]),candidate_mask=mask)
        np.testing.assert_array_equal(normal,copy_normal);np.testing.assert_array_equal(mask,copy_mask)
        for key in p:np.testing.assert_array_equal(p[key],before[key])


if __name__=='__main__':unittest.main(verbosity=2)
