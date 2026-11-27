"""Independent pure contracts for the offline terrain-preview campaign.

These tests exercise saved-clock selection, body-frame kinematics and evaluator
semantics. They do not run Gazebo, acquire scene truth, or modify old rollouts.
"""
from pathlib import Path
import importlib.util

import mujoco
import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1]/'scripts/run-terrain-memory.py'
SPEC = importlib.util.spec_from_file_location('terrain_preview_campaign_contracts',SCRIPT)
campaign = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(campaign)


@pytest.fixture(scope='module')
def state_model():
    """Free body plus three hinges: same nq/nv ordering, no terrain/dynamics."""
    return mujoco.MjModel.from_xml_string('''
      <mujoco><worldbody><body name="fork"><freejoint/>
        <geom type="sphere" size=".05" mass="1"/>
        <body><joint type="hinge" axis="1 0 0"/><geom type="sphere" size=".02" mass=".1"/>
          <body><joint type="hinge" axis="0 1 0"/><geom type="sphere" size=".02" mass=".1"/>
            <body><joint type="hinge" axis="0 0 1"/><geom type="sphere" size=".02" mass=".1"/></body>
          </body>
        </body>
      </body></worldbody></mujoco>''')


def test_anchor_memberships_are_119_queries_with_one_shared_stratum():
    times = np.arange(8801,dtype=float)*.005
    anchors = campaign.make_anchors(times)
    assert len(anchors) == 119
    assert sum('uniform' in row['strata'] for row in anchors) == 109
    assert sum('dense_bump' in row['strata'] for row in anchors) == 11
    assert len({row['anchor_index'] for row in anchors}) == 119
    assert [row['anchor_index'] for row in anchors] == sorted(row['anchor_index'] for row in anchors)
    overlap = [row for row in anchors if len(row['strata']) == 2]
    assert len(overlap) == 1
    assert overlap[0]['anchor_index'] == 1360
    assert set(overlap[0]['strata']) == {'uniform','dense_bump'}
    assert anchors[0]['anchor_time_s'] == pytest.approx(.4)
    assert anchors[-1]['anchor_time_s'] == pytest.approx(43.6)


def test_anchors_use_saved_clock_and_leave_the_full_horizon_available():
    times = np.arange(8801,dtype=float)*(.005+1e-15)
    before = times.copy()
    anchors = campaign.make_anchors(times)
    for row in anchors:
        assert row['anchor_time_s'] == times[row['anchor_index']]
    np.testing.assert_array_equal(times,before)
    short = campaign.make_anchors(np.arange(121,dtype=float)*.005)
    assert len(short) == 1 and short[0]['anchor_time_s'] == pytest.approx(.4)
    assert short[0]['anchor_time_s']+.2 <= .6+1e-12


def test_anchor_clock_rejects_empty_nonfinite_repeated_and_backwards_times():
    for invalid in ([],[0.,0.],[.1,0.],[0.,np.nan],[[0.,.005]]):
        with pytest.raises(ValueError):
            campaign.make_anchors(invalid)


def test_cv_advances_world_translation_body_rotation_and_all_three_joints(state_model):
    assert (state_model.nq,state_model.nv) == (10,9)
    yaw_half = np.pi/4
    qpos = np.array([1.,2.,3.,np.cos(yaw_half),0.,0.,np.sin(yaw_half),.4,-.2,.8])
    qvel = np.array([.3,-.2,.1,1.,0.,0.,1.5,-2.,.5])
    old_q,old_v = qpos.copy(),qvel.copy()
    predicted = campaign.cv_state(qpos,qvel,.2,state_model)
    # Analytic q_old * Exp(body-x*.2), rather than a world-x left product.
    half = .1
    expected_quat = np.array([np.cos(yaw_half)*np.cos(half),np.cos(yaw_half)*np.sin(half),
                              np.sin(yaw_half)*np.sin(half),np.sin(yaw_half)*np.cos(half)])
    np.testing.assert_allclose(predicted[:3],[1.06,1.96,3.02],atol=1e-14,rtol=0.)
    np.testing.assert_allclose(predicted[3:7],expected_quat,atol=1e-14,rtol=0.)
    np.testing.assert_allclose(predicted[7:],[.7,-.6,.9],atol=1e-14,rtol=0.)
    np.testing.assert_array_equal(qpos,old_q)
    np.testing.assert_array_equal(qvel,old_v)
    assert not np.shares_memory(predicted,qpos)


def test_cv_zero_horizon_is_an_independent_complete_state_copy(state_model):
    qpos = state_model.qpos0.copy()
    qvel = np.arange(state_model.nv,dtype=float)
    zero = campaign.cv_state(qpos,qvel,0.,state_model)
    np.testing.assert_array_equal(zero,qpos)
    zero[0] = 999.
    assert qpos[0] != 999.
    first = campaign.cv_state(qpos,qvel,.1,state_model)
    second = campaign.cv_state(qpos,qvel,.1,state_model)
    np.testing.assert_array_equal(first,second)
    np.testing.assert_array_equal(state_model.qpos0,qpos)


def test_cv_rejects_invalid_horizons_and_incomplete_or_nonfinite_states(state_model):
    qpos,qvel = state_model.qpos0.copy(),np.zeros(state_model.nv)
    with pytest.raises(ValueError):campaign.cv_state(qpos,qvel,-.1,state_model)
    with pytest.raises(ValueError):campaign.cv_state(qpos,qvel,np.inf,state_model)
    with pytest.raises(ValueError):campaign.cv_state(qpos[:-1],qvel,.1,state_model)
    qvel[3] = np.nan
    with pytest.raises(ValueError):campaign.cv_state(qpos,qvel,.1,state_model)


def test_availability_uses_50ms_delivery_including_the_arrival_boundary():
    captures = np.array([0.,.05,.1,.15])
    delivered = captures+.05
    np.testing.assert_array_equal(campaign.available_batch_indices(delivered,.1),[0,1])
    np.testing.assert_array_equal(campaign.available_batch_indices(delivered,.1-1e-6),[0])
    np.testing.assert_array_equal(campaign.available_batch_indices(delivered,.049),[])
    assert len(campaign.available_batch_indices(captures,.1)) == 3  # An incorrect capture-time policy leaks a batch.


def test_appending_future_deliveries_does_not_change_available_history():
    past = np.array([.05,.1,.15])
    combined = np.r_[past,.2,.25,20.]
    np.testing.assert_array_equal(campaign.available_batch_indices(past,.1),
                                  campaign.available_batch_indices(combined,.1))
    with pytest.raises(ValueError):campaign.available_batch_indices([.1,.05],.2)
    with pytest.raises(ValueError):campaign.available_batch_indices([.1,np.nan],.2)


def test_observer_plane_is_current_center_minus_radius_along_unit_normal():
    center,normal = np.array([1.,2.,.4]),np.array([0.,3.,4.])
    plane = campaign.ObserverPlane(center,normal,.12,6.8)
    query = plane.query(np.array([1.2,2.2]),at_time_s=6.8)
    assert query['supported']
    np.testing.assert_allclose(plane.point,[1.,1.928,.304],atol=1e-14)
    np.testing.assert_allclose(query['normal'],[0.,.6,.8],atol=1e-14)
    np.testing.assert_allclose(query['point_m'],[1.2,2.2,.1],atol=1e-14)
    assert query['height_m'] == pytest.approx(.1)
    assert query['age_s'] is None and query['support_cells'] is None
    center[:],normal[:] = 99.,99.
    np.testing.assert_allclose(plane.point,[1.,1.928,.304],atol=1e-14)


def test_observer_plane_keeps_anchor_knowledge_frozen_and_outputs_isolated():
    plane = campaign.ObserverPlane([0.,0.,.12],[0.,0.,1.],.12,2.)
    result = plane.query([.1,.2],at_time_s=2.)
    result['normal'][:] = 7.
    result['point_m'][:] = 8.
    again = plane.query([.1,.2],at_time_s=2.)
    np.testing.assert_array_equal(again['normal'],[0.,0.,1.])
    np.testing.assert_array_equal(again['point_m'],[.1,.2,0.])
    with pytest.raises(ValueError):plane.query([.1,.2],at_time_s=2.2)
    vertical = campaign.ObserverPlane([0.,0.,.12],[1.,0.,0.],.12,2.)
    assert not vertical.query([0.,0.],at_time_s=2.)['supported']


def test_surface_score_has_signed_height_and_unit_independent_normal_angle():
    truth = {'valid':True,'point_m':np.array([.1,.2,1.]),
             'normal':3*np.array([0.,np.sin(np.pi/6),np.cos(np.pi/6)])}
    query = {'supported':True,'height_m':1.012,'normal':np.array([0.,0.,2.])}
    score = campaign.score_surface(query,truth)
    assert score['supported'] and score['truth_valid']
    assert score['height_error_m'] == pytest.approx(.012)
    assert score['normal_error_deg'] == pytest.approx(30.)
    opposing = campaign.score_surface({'supported':True,'height_m':1.,'normal':[0.,0.,-1.]},
                                      {'valid':True,'point_m':[0.,0.,1.],'normal':[0.,0.,1.]})
    assert opposing['normal_error_deg'] == pytest.approx(180.)


def test_surface_unknown_or_invalid_truth_never_becomes_zero_error_success():
    unknown = campaign.score_surface({'supported':False,'reason':'unobserved_cell'},
                                     {'valid':True,'normal_ambiguous':False})
    assert not unknown['supported'] and unknown['truth_valid']
    assert unknown['height_error_m'] is None and unknown['normal_error_deg'] is None
    invalid_truth = campaign.score_surface({'supported':True,'reason':'observed'}, {'valid':False})
    assert invalid_truth['supported'] and not invalid_truth['truth_valid']
    assert invalid_truth['height_error_m'] is None and invalid_truth['normal_error_deg'] is None


def test_normal_ambiguity_preserves_height_coverage_but_abstains_from_angle():
    # At edges/multiple facets there need not be one defensible normal label.
    score = campaign.score_surface({'supported':True,'height_m':.99},
        {'valid':True,'point_m':[0.,0.,1.],'normal_ambiguous':True})
    assert score['supported'] and score['truth_valid'] and score['normal_ambiguous']
    assert score['height_error_m'] == pytest.approx(-.01)
    assert score['normal_error_deg'] is None


def _occluding_plane_adapter():
    from spherical_wheel.terrain_depth import FourCameraDepthAdapter,TerrainRayScene
    # A declared synthetic robot box, not an approximation asserted to be the
    # native robot. Cameras, range limits and map settings stay at their defaults.
    xml = '''<mujoco><worldbody><geom name="ground" type="plane" size="4 4 .1"/>
      <body name="fork"><freejoint/>
        <geom name="opaque_body" type="box" size=".16 .16 .12" mass="1"/>
      </body></worldbody></mujoco>'''
    scene = TerrainRayScene.from_xml_string(xml,['ground'],['opaque_body'])
    return scene,FourCameraDepthAdapter(scene)


def test_actual_rays_to_frames_to_memory_retain_prior_visible_ground_under_current_robot():
    from spherical_wheel.terrain_memory import TerrainMemory
    scene,adapter = _occluding_plane_adapter()
    query = np.array([.005,.005])
    camera = adapter.cameras[0]
    # Declare the earlier pose analytically from a fixed front pixel (u31,v26)
    # so its ground intersection lies in the query's1cm cell near x0. The pose
    # is about−.4m, with no search over rendered/map outcomes or setting changes.
    optical = np.linalg.solve(camera.intrinsics,[31.,26.,1.])
    direction = camera.body_from_camera[:3,:3]@optical
    camera_height = .12+camera.body_from_camera[2,3]
    previous_x = query[0]-camera.body_from_camera[0,3]+camera_height*direction[0]/direction[2]
    assert -.45 < previous_x < -.35
    earlier_pose = np.array([previous_x,0.,.12,1.,0.,0.,0.])
    current_pose = np.array([0.,0.,.12,1.,0.,0.,0.])
    earlier = adapter.render_batch(earlier_pose,0.,.05)
    current = adapter.render_batch(current_pose,1.,1.05)
    assert len(earlier) == len(current) == 4
    # This ray has valid terrain within range but an opaque robot first hit.
    front = current[0]
    ray = np.linalg.solve(front.intrinsics,[31.,47.,1.])
    hit = scene.environment_hit(front.world_from_camera[None,:3,3],
                                (front.world_from_camera[:3,:3]@ray)[None,:])
    assert hit['valid'][0] and .08 < hit['range_m'][0] < 2.
    assert not front.valid_mask[47,31] and np.isnan(front.depth_m[47,31])
    remembered,latest = TerrainMemory(),TerrainMemory()
    remembered.ingest(earlier,available_at_s=.05)
    remembered.ingest(current,available_at_s=1.05)
    latest.ingest(current,available_at_s=1.05)
    old_ground = remembered.query(query,at_time_s=1.05)
    current_only = latest.query(query,at_time_s=1.05)
    assert old_ground['supported']
    assert old_ground['height_m'] == pytest.approx(0.,abs=1e-12)
    np.testing.assert_allclose(old_ground['normal'],[0.,0.,1.],atol=1e-12)
    assert old_ground['first_capture_s'] == old_ground['last_capture_s'] == 0.
    assert not current_only['supported'] and current_only['reason'] == 'unobserved_cell'


def test_actual_four_view_world_coverage_rotates_between_camera_indices_at_yaw90():
    from spherical_wheel.terrain_memory import TerrainMemory
    _,adapter = _occluding_plane_adapter()
    poses = [np.array([0.,0.,.12,1.,0.,0.,0.]),
             np.array([0.,0.,.12,np.sqrt(.5),0.,0.,np.sqrt(.5)])]
    counts = []
    for number,pose in enumerate(poses):
        batch = adapter.render_batch(pose,float(number),float(number)+.05)
        per_camera = []
        for frame in batch:
            memory = TerrainMemory()
            memory.ingest([frame],available_at_s=float(number)+.05)
            points = memory.observed_cells(at_time_s=float(number)+.05)['centers_m']
            # Fixed world+x patch, well outside the body; no truth goes into memory.
            per_camera.append(int(np.count_nonzero((points[:,0]>.4)&(points[:,0]<.6)&(abs(points[:,1])<.05))))
        counts.append(per_camera)
    assert counts[0][0] > 0 and counts[1][3] > 0
    assert counts[0][1:] == [0,0,0]
    assert counts[1][:3] == [0,0,0]
    # Mount radii differ across axes, so pixel-identical images are not expected;
    # the physical world direction must nevertheless move from camera000 to270.


def test_unloaded_finite_fallback_point_is_not_scored_as_a_contact():
    candidates = [{'point_m':np.array([0.,0.,0.])},{'point_m':np.array([0.,0.,.01])}]
    finite_fallback_center = np.array([0.,0.,.12])
    assert campaign.candidate_point_error(candidates,finite_fallback_center,[]) is None
    assert campaign.candidate_point_error(candidates,[np.nan,0.,0.],['shell_A']) is None
    assert campaign.candidate_point_error([],finite_fallback_center,['shell_A']) is None
    assert campaign.candidate_point_error(candidates,[0.,0.,.012],['shell_A']) == pytest.approx(.002)
