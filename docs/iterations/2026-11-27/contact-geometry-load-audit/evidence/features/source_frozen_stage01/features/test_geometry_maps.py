"""Focused synthetic load-free geometry contracts, using unittest only."""
from pathlib import Path
from unittest.mock import patch
import importlib.util,unittest
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value
G=module(HERE/'geometry_maps.py','_i66_test_geometry_maps')


def fixture():
    import mujoco
    xml='''<mujoco><size nuser_geom="5"/><worldbody><body name="fork"><freejoint/><geom size=".01"/>
    <body name="shaft" pos=".01 .02 0"><joint axis="0 0 1"/><geom size=".01"/>
    <geom name="belt_A_smooth" size=".12" user="1 .11 .12 4 .12"/>
    <geom name="belt_B_smooth" size=".12" user="-1 .11 .12 4 .12"/>
    <body name="a" pos="0 0 .01"><joint axis="1 0 0"/><geom name="shell_A_smooth" size=".12" user="1 .01 .11 2 .12"/></body>
    <body name="b" pos="0 0 -.01"><joint axis="1 0 0"/><geom name="shell_B_smooth" size=".12" user="-1 .01 .11 2 .12"/></body>
    </body></body></worldbody></mujoco>'''
    core=module(ROOT/'src/spherical_wheel/contact_load_consistency.py','_i66_test_load_core')
    geometry=module(ROOT/'src/spherical_wheel/contact_hypotheses.py','_i66_test_patches')
    model=mujoco.MjModel.from_xml_string(xml);return G.EndpointPoseMaps(model,core,geometry),model.qpos0.copy()


def stack(poses):
    dynamic=('centre_m','linear_at_centre','angular_world','valid','patch_axis_world','hinge_axis_world','hinge_anchor_m')
    return {**{key:np.asarray([pose[key] for pose in poses]) for key in dynamic},
        **{key:poses[0][key].copy() for key in ['radius_m','patch_lower_m','patch_upper_m','ancestor_active']}}


class GeometryContracts(unittest.TestCase):
    def test_skew_is_world_cross_product(self):
        vectors=np.array([[1.,2.,3.],[-1.,0.,2.]])
        force=np.array([2.,-1.,.5]);np.testing.assert_array_equal(G.skew(vectors)@force,np.cross(vectors,force))

    def test_affine_point_map_matches_direct_mujoco_jacobian(self):
        import mujoco
        generator,q=fixture();q[7:]=[.3,.4,-.2];pose=generator.evaluate(q)
        points=pose['centre_m']+np.array([[.1,.05,.02],[-.08,.04,-.01],[.12,0.,0.],[-.12,0.,0.]])
        result=G.family_point_maps(pose,points)
        for k,patch_info in enumerate(generator.patches):
            jac=np.zeros((3,generator.model.nv));mujoco.mj_jac(generator.model,generator.data,jac,None,points[k],patch_info['body'])
            np.testing.assert_allclose(result['H'][k],jac[:,6:9].T,atol=1e-15)

    def test_independent_hinge_formula_matches_affine_map(self):
        generator,q=fixture();q[7:]=[.3,.4,-.2];pose=generator.evaluate(q)
        points=pose['centre_m']+np.array([[.1,.05,.02],[-.08,.04,-.01],[.12,0.,0.],[-.12,0.,0.]])
        np.testing.assert_allclose(G.analytical_family_point_maps(pose,points)['H'],G.family_point_maps(pose,points)['H'],atol=1e-15)

    def test_centre_linear_map_can_be_nonzero_and_is_not_assumed_away(self):
        generator,q=fixture();pose=generator.evaluate(q);centres=np.tile(pose['centre_m'],(4,1))
        self.assertGreater(np.max(abs(pose['linear_at_centre'])),.001)
        np.testing.assert_allclose(G.analytical_family_point_maps(pose,centres)['H'],pose['linear_at_centre'],atol=1e-15)

    def test_raw_inside_sphere_mean_remains_valid_without_projection(self):
        generator,q=fixture();pose=generator.evaluate(q)
        point=.12*np.array([np.cos(.5),np.sin(.5),0.]);physical=G.assigned_point_maps(pose,point,np.asarray(0))
        virtual=G.assigned_point_maps(pose,.5*point,np.asarray(0))
        self.assertTrue(physical['material_valid']);self.assertTrue(virtual['valid']);self.assertFalse(virtual['material_valid'])
        self.assertLess(float(virtual['radius_error_m']),0.)
        self.assertGreater(np.max(abs(physical['H']-virtual['H'])),.01)

    def test_nonloaded_family_point_is_unknown_individually(self):
        generator,q=fixture();pose=generator.evaluate(q);points=np.tile([.12,0.,0.],(4,1));points[1]=np.nan
        result=G.family_point_maps(pose,points)
        np.testing.assert_array_equal(result['valid'],[True,False,True,True]);self.assertTrue(np.isnan(result['H'][1]).all())

    def test_unknown_assignment_does_not_borrow_another_family(self):
        generator,q=fixture();pose=generator.evaluate(q);value=G.assigned_point_maps(pose,np.array([.12,0.,0.]),np.asarray(-1))
        self.assertFalse(value['valid']);self.assertFalse(value['material_valid']);self.assertTrue(np.isnan(value['H']).all())

    def test_assigned_family_changes_joint_route_without_relabeling_point(self):
        generator,q=fixture();pose=generator.evaluate(q);point=np.array([.1,.05,.02])
        shell=G.assigned_point_maps(pose,point,np.asarray(0));belt=G.assigned_point_maps(pose,point,np.asarray(2))
        self.assertGreater(np.max(abs(shell['H'][1:])),0.);np.testing.assert_array_equal(belt['H'][1:],0.)

    def test_vectorized_cache_matches_single_pose_results(self):
        generator,q=fixture();first=generator.evaluate(q);q[7]=.4;second=generator.evaluate(q);pose=stack([first,second])
        points=np.array([[[.1,.04,.02]]*4,[[-.05,.03,.01]]*4])
        all_maps=G.family_point_maps(pose,points)
        for i,item in enumerate([first,second]):np.testing.assert_allclose(all_maps['H'][i],G.family_point_maps(item,points[i])['H'],atol=1e-15)

    def test_world_rotation_preserves_scalar_joint_torque(self):
        generator,q=fixture();pose=generator.evaluate(q);point=np.array([.1,.05,.02]);force=np.array([2.,3.,5.])
        first=G.assigned_point_maps(pose,point,np.asarray(0))['H']@force
        rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
        q[3:7]=[np.sqrt(.5),0.,0.,np.sqrt(.5)];rotated=generator.evaluate(q)
        second=G.assigned_point_maps(rotated,rotation@point,np.asarray(0))['H']@(rotation@force)
        np.testing.assert_allclose(first,second,atol=1e-15)

    def test_point_force_and_equivalent_centre_wrench_agree(self):
        generator,q=fixture();pose=generator.evaluate(q);point=np.array([.1,.05,.02]);force=np.array([2.,3.,5.])
        mapped=G.assigned_point_maps(pose,point,np.asarray(0))['H']@force
        direct=pose['linear_at_centre'][0]@force+pose['angular_world'][0]@np.cross(point-pose['centre_m'],force)
        np.testing.assert_allclose(mapped,direct,atol=1e-15)

    def test_point_shift_along_force_has_no_joint_load_effect(self):
        generator,q=fixture();pose=generator.evaluate(q);point=np.array([.1,.05,.02]);force=np.array([2.,3.,5.])
        first=G.assigned_point_maps(pose,point,np.asarray(0))['H']@force
        second=G.assigned_point_maps(pose,point+.01*force,np.asarray(0))['H']@force
        np.testing.assert_allclose(first,second,atol=1e-15)

    def test_bad_pose_is_explicit_and_cannot_reuse_previous_map(self):
        generator,q=fixture();self.assertTrue(generator.evaluate(q)['valid']);q[3:7]=0.
        invalid=generator.evaluate(q);self.assertFalse(invalid['valid']);self.assertTrue(np.isnan(invalid['linear_at_centre']).all())

    def test_private_kinematics_never_steps_or_queries_contact(self):
        import mujoco
        generator,q=fixture();before=q.copy()
        with patch.object(mujoco,'mj_step',side_effect=AssertionError('physics')),patch.object(mujoco,'mj_forward',side_effect=AssertionError('forward')),patch.object(mujoco,'mj_collision',side_effect=AssertionError('collision')),patch.object(mujoco,'mj_contactForce',side_effect=AssertionError('contact')):
            value=generator.evaluate(q)
        self.assertTrue(value['valid']);np.testing.assert_array_equal(q,before)

    def test_numpy_point_mapping_does_not_mutate_cache_or_points(self):
        generator,q=fixture();pose=generator.evaluate(q);frozen={key:value.copy() if isinstance(value,np.ndarray) else value for key,value in pose.items()}
        points=np.tile([.1,.05,.02],(4,1));before=points.copy();G.family_point_maps(pose,points)
        np.testing.assert_array_equal(points,before)
        for key in pose:np.testing.assert_array_equal(pose[key],frozen[key])


if __name__=='__main__':unittest.main(verbosity=2)
