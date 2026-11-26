"""Ideal range adapter contracts, including opaque first-hit robot masking."""
from dataclasses import FrozenInstanceError,fields
import json

import mujoco
import numpy as np
import pytest

from spherical_wheel.terrain_depth import (CameraSpec,DepthFrame,FourCameraDepthAdapter,
    TerrainRayScene,_crop_mesh,four_camera_rig)


def downward_camera(width=3,height=3,max_range=4.,origin=(0.,0.,1.)):
    transform=np.diag([1.,-1.,-1.,1.]);transform[:3,3]=origin
    k=np.array([[1.,0.,(width-1)/2],[0.,1.,(height-1)/2],[0.,0.,1.]])
    return CameraSpec('test',width,height,k,transform,.08,max_range)


def plane_scene(extra='',robot=()):
    xml=f'<mujoco><worldbody><geom name="ground" type="plane" size="10 10 .1"/>{extra}</worldbody></mujoco>'
    return TerrainRayScene.from_xml_string(xml,['ground'],robot)


def test_axial_depth_differs_from_ray_range_and_is_finite_field_of_view():
    scene=plane_scene();camera=downward_camera();frame=FourCameraDepthAdapter(scene,[camera]).render_batch([],1.,1.05)[0]
    np.testing.assert_allclose(frame.depth_m,np.ones((3,3)),atol=1e-14)
    assert frame.valid_mask.all()
    # The corner ray travels sqrt(3) m, while axial Z remains 1 m.
    assert frame.depth_m[0,0]==pytest.approx(1.)
    assert frame.depth_m.shape==(camera.height,camera.width)


def test_range_cutoff_uses_euclidean_first_hit_not_axial_depth():
    frame=FourCameraDepthAdapter(plane_scene(),[downward_camera(max_range=1.1)]).render_batch([],0.,.05)[0]
    assert frame.valid_mask.sum()==1 and frame.valid_mask[1,1]
    assert np.isnan(frame.depth_m[0,0])


def test_first_robot_hit_is_masked_and_cannot_see_through_to_ground():
    scene=plane_scene('<geom name="robot" type="box" size=".2 .2 .1" pos="0 0 .5"/>',['robot'])
    frame=FourCameraDepthAdapter(scene,[downward_camera(width=1,height=1)]).render_batch([],0.,.05)[0]
    assert not frame.valid_mask[0,0] and np.isnan(frame.depth_m[0,0])
    # Evaluator-only environment query ignores robot occluders and has no 2 m camera cap.
    hit=scene.environment_hit([[0.,0.,10.]],[[0.,0.,-2.]])
    assert hit['valid'][0] and hit['range_m'][0]==pytest.approx(10.)
    np.testing.assert_allclose(hit['normal_world'][0],[0.,0.,1.])


def test_near_first_hit_is_invalid_without_searching_farther():
    xml='<mujoco><worldbody><geom name="near" type="box" size=".2 .2 .01" pos="0 0 .96"/><geom name="ground" type="plane" size="10 10 .1"/></worldbody></mujoco>'
    scene=TerrainRayScene.from_xml_string(xml,['near','ground'])
    frame=FourCameraDepthAdapter(scene,[downward_camera(width=1,height=1)]).render_batch([],0.,.05)[0]
    assert not frame.valid_mask[0,0] and np.isnan(frame.depth_m[0,0])


def test_environment_occlusion_returns_nearest_surface():
    xml='<mujoco><worldbody><geom name="step" type="box" size=".2 .2 .1" pos="0 0 .5"/><geom name="ground" type="plane" size="10 10 .1"/></worldbody></mujoco>'
    scene=TerrainRayScene.from_xml_string(xml,['step','ground'])
    frame=FourCameraDepthAdapter(scene,[downward_camera(width=1,height=1)]).render_batch([],0.,.05)[0]
    assert frame.depth_m[0,0]==pytest.approx(.4)


def test_capture_transform_follows_measured_body_pose_without_dynamics():
    xml='<mujoco><worldbody><geom name="ground" type="plane" size="10 10 .1"/><body name="fork"><freejoint/><inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/></body></worldbody></mujoco>'
    scene=TerrainRayScene.from_xml_string(xml,['ground'])
    camera=downward_camera(width=1,height=1,origin=(.1,0.,1.))
    qpos=np.array([2.,3.,0.,np.sqrt(.5),0.,0.,np.sqrt(.5)])
    frame=FourCameraDepthAdapter(scene,[camera]).render_batch(qpos,2.,2.05)[0]
    np.testing.assert_allclose(frame.world_from_camera[:3,3],[2.,3.1,1.],atol=1e-14)
    assert frame.depth_m[0,0]==pytest.approx(1.)
    assert frame.capture_time_s==2. and frame.delivered_at_s==2.05


def test_robot_joint_motion_changes_self_occlusion():
    xml='<mujoco><worldbody><geom name="ground" type="plane" size="10 10 .1"/><body name="arm" pos="0 0 .5"><joint type="slide" axis="1 0 0"/><geom name="robot" type="box" size=".2 .2 .1" mass="1"/></body></worldbody></mujoco>'
    scene=TerrainRayScene.from_xml_string(xml,['ground'],['robot']);adapter=FourCameraDepthAdapter(scene,[downward_camera(width=1,height=1)])
    assert not adapter.render_batch([0.],0.,.05)[0].valid_mask[0,0]
    frame=adapter.render_batch([1.],.1,.15)[0]
    assert frame.valid_mask[0,0] and frame.depth_m[0,0]==pytest.approx(1.)


def test_concave_mesh_rays_preserve_valley_not_convex_hull():
    vertices='-1 -1 1 1 -1 1 1 1 1 -1 1 1 0 0 0';faces='0 1 4 1 2 4 2 3 4 3 0 4'
    xml=f'<mujoco><asset><mesh name="valley" vertex="{vertices}" face="{faces}" inertia="shell"/></asset><worldbody><geom name="surface" type="mesh" mesh="valley"/></worldbody></mujoco>'
    scene=TerrainRayScene.from_xml_string(xml,['surface'])
    hit=scene.environment_hit([[.1,.2,2.]],[[0.,0.,-1.]])
    assert hit['range_m'][0]==pytest.approx(1.8,abs=5e-8)
    np.testing.assert_allclose(hit['point_world'][0],[.1,.2,.2],atol=5e-8)
    np.testing.assert_allclose(hit['normal_world'][0],[0.,-np.sqrt(.5),np.sqrt(.5)],atol=5e-8)


def test_crop_retains_crossing_triangle_even_when_no_vertex_inside():
    vertices=np.array([[-2.,-.2,0.],[2.,-.2,0.],[0.,2.,0.]])
    v,f=_crop_mesh(vertices,np.array([[0,1,2]]),[-.1,.1,-.1,.1])
    assert len(f)==1
    np.testing.assert_array_equal(v[f],vertices[[[0,1,2]]])


def test_four_fixed_cameras_have_declared_mounts_and_orthonormal_optical_axes():
    rig=four_camera_rig();assert len(rig)==4
    np.testing.assert_array_equal([c.body_from_camera[:3,3]for c in rig],[[.105,0.,.235],[0.,.135,.235],[-.105,0.,.235],[0.,-.135,.235]])
    for camera in rig:
        assert camera.width==64 and camera.height==48
        assert camera.intrinsics[0,0]==camera.intrinsics[1,1]==pytest.approx(32/np.tan(np.radians(50)))
        assert camera.intrinsics[0,2]==31.5 and camera.intrinsics[1,2]==23.5
        assert camera.min_range_m==.08 and camera.max_range_m==2.
        np.testing.assert_allclose(camera.body_from_camera[:3,:3].T@camera.body_from_camera[:3,:3],np.eye(3),atol=1e-14)
        assert camera.body_from_camera[2,2]==pytest.approx(-np.sqrt(.5))


def test_frames_leak_no_geometry_and_defensively_copy_arrays():
    adapter=FourCameraDepthAdapter(plane_scene(),[downward_camera()]);frame=adapter.render_batch([],1.,1.05)[0]
    assert {f.name for f in fields(frame)}=={'camera_name','depth_m','valid_mask','intrinsics','body_from_camera','world_from_camera','capture_time_s','delivered_at_s'}
    with pytest.raises(ValueError):frame.depth_m[0,0]=2.
    with pytest.raises(ValueError):frame.world_from_camera[0,3]=9.
    with pytest.raises(FrozenInstanceError):frame.capture_time_s=3.
    later=adapter.render_batch([],2.,2.05)[0]
    assert not np.shares_memory(frame.depth_m,later.depth_m)
    assert not np.shares_memory(frame.world_from_camera,adapter.cameras[0].body_from_camera)


def test_adapter_never_calls_dynamics_or_contact_force(monkeypatch):
    scene=plane_scene();adapter=FourCameraDepthAdapter(scene,[downward_camera()])
    def forbidden(*args,**kwargs):raise AssertionError('Dynamics or solved force read')
    for name in ('mj_step','mj_forward','mj_contactForce'):monkeypatch.setattr(mujoco,name,forbidden)
    assert adapter.render_batch([],0.,.05)[0].valid_mask.all()


@pytest.mark.parametrize('times',[(1.,.9),(-1.,0.),(np.nan,1.),(0.,np.inf)])
def test_invalid_delivery_or_capture_time_rejected(times):
    adapter=FourCameraDepthAdapter(plane_scene(),[downward_camera()])
    with pytest.raises(ValueError):adapter.render_batch([],*times)


def test_missing_hits_are_nan_and_evaluator_normals_oppose_ray():
    scene=TerrainRayScene.from_xml_string('<mujoco><worldbody><geom name="box" type="box" size=".1 .1 .1"/></worldbody></mujoco>',['box'])
    hit=scene.environment_hit([[0.,0.,2.],[1.,1.,2.]],[[0.,0.,-1.],[0.,0.,-1.]])
    assert hit['valid'].tolist()==[True,False]
    assert np.isnan(hit['range_m'][1])and np.isnan(hit['normal_world'][1]).all()
    assert hit['normal_world'][0]@np.array([0.,0.,-1.])<0
    assert set(hit)=={'range_m','point_world','normal_world','valid'}


def test_invalid_calibration_or_rays_rejected():
    with pytest.raises(ValueError):CameraSpec('x',0,1,np.eye(3),np.eye(4))
    with pytest.raises(ValueError):CameraSpec('x',1,1,np.diag([-1.,1.,1.]),np.eye(4))
    scene=plane_scene()
    with pytest.raises(ValueError):scene.environment_hit([[0.,0.,1.]],[[0.,0.,0.]])
    with pytest.raises(ValueError):scene.set_qpos([np.nan])
