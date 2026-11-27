"""Synthetic numerical contracts for the opt-in shadow contact profiles."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from spherical_wheel import shadow_contact_models as models


def skew(point):
    x, y, z = point
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def inputs():
    points = np.array([[-.06, 0., -.1], [.06, 0., -.1], [0., -.06, -.1], [0., .06, -.1]])
    maps = np.zeros((4, 9, 3))
    for i, point in enumerate(points):
        maps[i, :3] = np.eye(3)
        maps[i, 3:6] = skew(point)
        maps[i, 6] = skew(point)[1]
    return dict(observation9=maps[0] @ [0., 0., 10.], points_m=points,
        force_normals=np.tile([0., 0., 1.], (4, 1)),
        material_velocity_m_s=np.zeros((4, 3)), force_maps=maps,
        family_index=np.arange(4), valid=np.ones(4, bool), normal_hint=np.array([0., 0., 1.]),
        candidate_index=np.arange(4))


class ContactProfileContracts(unittest.TestCase):
    def test_exact_mode_order(self):
        self.assertEqual(models.SUPPORT_SETS[:4], ((0,), (1,), (2,), (3,)))
        self.assertEqual(models.PAIR_SETS, ((0,1),(0,2),(0,3),(1,2),(1,3),(2,3)))
        np.testing.assert_array_equal(models.MODE_FAMILIES[8], [0,1])
        np.testing.assert_array_equal(models.MODE_FAMILIES[20], [-1,-1])
        self.assertFalse(models.MODE_FAMILIES.flags.writeable)

    def test_friction_rays_have_unit_normal_and_inscribed_tangent_limit(self):
        n=np.array([.2,-.3,.9]);n/=np.linalg.norm(n)
        rays=models.friction_rays(n,.7)
        np.testing.assert_allclose(n @ rays,np.ones(4),atol=1e-14)
        tangent=rays-n[:,None]*(n @ rays)
        np.testing.assert_allclose(np.linalg.norm(tangent,axis=0),.7,atol=1e-14)
        np.testing.assert_allclose(rays.mean(axis=1),n,atol=1e-14)

    def test_single_force_reconstructs_a_feasible_observation(self):
        data=inputs();force=np.array([1.,-.5,12.])
        y=data['force_maps'][0] @ force
        fit=models.fit_contact_forces(y,data['force_maps'][:1],data['force_normals'][:1])
        self.assertTrue(fit['valid'])
        np.testing.assert_allclose(fit['forces_N'][0],force,atol=1e-9)
        np.testing.assert_allclose(fit['predicted9'],y,atol=1e-9)
        self.assertLess(fit['cost'],1e-14)

    def test_pair_fits_two_independent_load_shares(self):
        data=inputs();maps=data['force_maps'][:2]
        y=maps[0] @ [0.,0.,5.]+maps[1] @ [0.,0.,15.]
        fit=models.fit_contact_forces(y,maps,data['force_normals'][:2])
        self.assertTrue(fit['valid'])
        np.testing.assert_allclose(fit['predicted9'],y,atol=1e-8)
        np.testing.assert_allclose(fit['forces_N'][:,2],[5.,15.],atol=1e-8)
        np.testing.assert_allclose(fit['normal_share'],[.25,.75],atol=1e-9)
        for f,n in zip(fit['forces_N'],data['force_normals'][:2]):
            self.assertLessEqual(np.linalg.norm(f-n*(n@f)),.7*(n@f)+1e-9)

    def test_pair_cannot_hide_a_zero_force_ghost_contact(self):
        maps=np.zeros((2,9,3));maps[:,:3]=np.eye(3)
        maps[0,7,2]=.1;maps[1,8,2]=.1
        y=maps[1] @ [0.,0.,20.]
        fit=models.fit_contact_forces(y,maps,np.tile([0.,0.,1.],(2,1)))
        self.assertTrue(fit['valid'])
        self.assertGreaterEqual(fit['normal_share'].min(),.1-1e-10)
        self.assertGreater(fit['forces_N'][0,2],0.)
        self.assertGreater(fit['cost'],0.)

    def test_zero_observation_does_not_create_support_force(self):
        data=inputs()
        fit=models.fit_contact_forces(np.zeros(9),data['force_maps'][:2],data['force_normals'][:2])
        self.assertFalse(fit['valid'])
        self.assertFalse(fit['solve_failed'])

    def test_missing_observation_low_force_and_invalid_hint_are_unknown_only(self):
        for variant in ('missing','low','hint'):
            data=inputs()
            if variant=='missing':data['observation_valid']=False
            if variant=='low':data['observation9']=np.r_[0.,0.,.9,np.zeros(6)]
            if variant=='hint':data['normal_hint']=np.zeros(3)
            out=models.evaluate_frame(**data)
            self.assertTrue(np.isinf(out['node_cost'][:20]).all())
            self.assertEqual(out['node_cost'][20],9.)
            self.assertEqual(out['force_fit_count'],0)
            self.assertTrue(np.isnan(out['node']['selected_point_m'][20]).all())

    def test_normal_velocity_is_removed_from_rolling_cue(self):
        data=inputs();plain=models.evaluate_frame(**data)
        data['material_velocity_m_s'][:,2]=3.
        changed=models.evaluate_frame(**data)
        np.testing.assert_allclose(plain['node_cost'],changed['node_cost'],atol=1e-12)

    def test_slip_has_relative_gaussian_normalizer(self):
        out=models.evaluate_frame(**inputs())
        self.assertAlmostEqual(out['node_cost'][1]-out['node_cost'][0],2*np.log(.15/.02),places=10)
        data=inputs();data['material_velocity_m_s'][:]=[.2,0.,0.]
        out=models.evaluate_frame(**data)
        self.assertLess(out['node_cost'][1],out['node_cost'][0])

    def test_endpoint_edge_relaxes_only_exiting_contact_velocity(self):
        data=inputs();data['valid'][2:]=False
        data['observation9']=data['force_maps'][0] @ [0.,0.,5.]+data['force_maps'][1] @ [0.,0.,15.]
        data['material_velocity_m_s'][0]=[.12,0.,0.]
        out=models.evaluate_frame(**data)
        self.assertAlmostEqual(out['node_cost'][8]-out['edge_cost'][0,1,0],18.,places=8)
        self.assertAlmostEqual(out['node_cost'][8],out['edge_cost'][0,0,0],places=8)
        np.testing.assert_array_equal(out['edge']['selected_candidate_index'][0,1,0],[0,1])
        np.testing.assert_allclose(out['node']['estimated_force_N'][8],out['edge']['estimated_force_N'][0,1,0])

    def test_force_fits_are_reused_for_motion_and_directed_edges(self):
        out=models.evaluate_frame(**inputs())
        self.assertEqual(out['force_fit_count'],10)  #four single+six pair fits
        self.assertEqual(out['node_cost'].shape,(21,))
        self.assertEqual(out['edge_cost'].shape,(6,2,2))

    def test_two_nominees_are_deduplicated_and_ids_break_ties(self):
        data=inputs()
        for key in ('points_m','force_normals','material_velocity_m_s','force_maps','family_index','valid'):
            data[key]=np.concatenate((data[key],data[key][:1]),axis=0)
        data['candidate_index']=np.array([10,11,12,13,2])
        out=models.evaluate_frame(**data)
        self.assertEqual(out['shortlist_count'][0],1)
        self.assertEqual(out['shortlist_candidate_index'][0,0],2)

    def test_input_row_permutation_preserves_identity_and_potentials(self):
        data=inputs();expected=models.evaluate_frame(**data)
        order=np.array([3,1,0,2])
        for key in ('points_m','force_normals','material_velocity_m_s','force_maps','family_index','valid','candidate_index'):
            data[key]=data[key][order]
        actual=models.evaluate_frame(**data)
        np.testing.assert_allclose(actual['node_cost'],expected['node_cost'],atol=1e-12)
        np.testing.assert_array_equal(actual['node']['selected_candidate_index'],expected['node']['selected_candidate_index'])

    def test_mean_exponential_cost_and_conditional_covariance(self):
        points=np.array([[0.,0.,0.],[.002,0.,0.]])
        fit=dict(forces_N=np.array([[0.,0.,1.]]),predicted9=np.zeros(9),residual9=np.zeros(9),
            normal_share=np.ones(1),cost=0.,rank=3,active_rays=1)
        storage=models._storage((1,))
        cost=models._aggregate([((0,),fit),((1,),fit)],[2.,6.],[2.,6.],[0.,0.],points,np.arange(2),storage,0)
        self.assertAlmostEqual(cost,2.-np.log((1.+np.exp(-4.))/2.),places=12)
        weight=1/(1+np.exp(-4.))
        self.assertAlmostEqual(storage['conditional_point_covariance_m2'][0,0,0,0],weight*(1-weight)*.002**2,places=15)
        np.testing.assert_array_equal(storage['selected_point_m'][0,0],points[0])

    def test_coupled_map_point_output_never_combines_alternatives(self):
        points=np.arange(12,dtype=float).reshape(4,3)
        fit=dict(forces_N=np.ones((2,3)),predicted9=np.zeros(9),residual9=np.zeros(9),
            normal_share=np.array([.4,.6]),cost=0.,rank=6,active_rays=2)
        storage=models._storage((1,))
        models._aggregate([((0,1),fit),((2,3),fit)],[1.,0.],[1.,0.],[0.,0.],points,np.arange(4),storage,0)
        np.testing.assert_array_equal(storage['selected_candidate_index'][0],[2,3])
        np.testing.assert_array_equal(storage['selected_point_m'][0],points[[2,3]])

    def test_solver_failure_is_explicit_not_an_unlogged_fallback(self):
        data=inputs()
        with patch.object(models,'nnls',side_effect=RuntimeError('fixed limit')):
            out=models.evaluate_frame(**data)
        self.assertEqual(out['failed_force_fit_count'],10)
        self.assertTrue(np.isinf(out['node_cost'][:20]).all())

    def test_no_input_mutation_and_no_live_imports(self):
        data=inputs();copies={k:v.copy() for k,v in data.items()}
        models.evaluate_frame(**data)
        for key in copies:np.testing.assert_array_equal(data[key],copies[key])
        source=Path(models.__file__).read_text()
        self.assertNotIn('from .control',source)
        self.assertNotIn('import mujoco',source)

    def test_invalid_dimensions_ids_and_configuration_are_rejected(self):
        data=inputs();data['candidate_index']=np.array([0,0,2,3])
        with self.assertRaises(ValueError):models.evaluate_frame(**data)
        with self.assertRaises(ValueError):models.ModelConfig(minimum_pair_normal_share=.5)
        with self.assertRaises(ValueError):models.friction_rays(np.zeros(3))
        with self.assertRaises(ValueError):models.evaluate_frame(**inputs(),dt_s=0.)


if __name__=='__main__':unittest.main(verbosity=2)
