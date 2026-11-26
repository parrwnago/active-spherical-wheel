"""Observed-depth causality and geometric preview tests; no scene/physics use."""
from types import SimpleNamespace
import copy

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from spherical_wheel.terrain_memory import (MemorySettings,TerrainMemory,WheelGeometry,
    carrier_axis,contact_candidates,preview_contacts,propagate_cv)


def frame(*,capture=0.,delivery=0.,a=0.,b=0.,height=0.,yaw=0.,origin=(.005,.005,1.),valid=None,name='test'):
    width=height_pixels=11;intrinsic=np.array([[100.,0.,5.],[0.,100.,5.],[0.,0.,1.]])
    rotation=Rotation.from_rotvec([0.,0.,yaw]).as_matrix()@np.diag([1.,-1.,-1.])
    transform=np.eye(4);transform[:3,:3]=rotation;transform[:3,3]=origin
    row,column=np.indices((height_pixels,width))
    optical=np.stack(((column-5.)/100.,(row-5.)/100.,np.ones_like(row)),axis=-1)
    directions=optical@rotation.T;origin=np.asarray(origin)
    depth=(a*origin[0]+b*origin[1]+height-origin[2])/(directions[:,:,2]-a*directions[:,:,0]-b*directions[:,:,1])
    mask=np.ones(depth.shape,bool) if valid is None else np.asarray(valid,bool)
    depth=np.where(mask,depth,np.nan)
    return SimpleNamespace(camera_name=name,depth_m=depth,valid_mask=mask,intrinsics=intrinsic,
        world_from_camera=transform,capture_time_s=capture,delivered_at_s=delivery)


def state(phase=.3,yaw=0.,center=(0.,0.,.12)):
    return np.r_[center,np.cos(yaw/2),0.,0.,np.sin(yaw/2),phase,0.,0.]


GEOMETRY=WheelGeometry(.12,15.46600995,.010)


class Plane:
    settings=MemorySettings()
    def __init__(self,normal=(0.,0.,1.)):
        self.normal=np.asarray(normal,float);self.normal/=np.linalg.norm(self.normal);self.calls=[]
    def query(self,xy,at_time_s):
        self.calls.append((np.asarray(xy).copy(),at_time_s))
        z=-self.normal[:2]@xy/self.normal[2]
        return {'supported':True,'reason':'declared_test_plane','normal':self.normal.copy(),
            'point_m':np.r_[xy,z],'height_m':float(z)}


def test_axial_depth_capture_registration_and_slope_fit():
    memory=TerrainMemory();memory.ingest([frame(a=.12,b=-.07,height=.03)],0.)
    result=memory.query([.005,.005],0.)
    assert result['supported']
    assert result['height_m']==pytest.approx(.03+.12*.005-.07*.005,abs=1e-12)
    normal=np.array([-.12,.07,1.]);normal/=np.linalg.norm(normal)
    np.testing.assert_allclose(result['normal'],normal,atol=1e-12,rtol=0.)


def test_yawed_capture_transform_places_observations_in_world_frame():
    memory=TerrainMemory();memory.ingest([frame(a=.1,b=.2,yaw=np.pi/2,origin=(.105,.205,1.))],0.)
    result=memory.query([.105,.205],0.)
    assert result['supported']
    assert result['height_m']==pytest.approx(.1*.105+.2*.205,abs=1e-12)


def test_latency_boundary_and_batch_validation_are_causal_and_atomic():
    memory=TerrainMemory();delayed=frame(capture=.1,delivery=.15)
    with pytest.raises(ValueError):memory.ingest([delayed],.149)
    assert len(memory.observed_cells(.149)['centers_m'])==0
    valid=frame(capture=0.,delivery=0.,name='early')
    with pytest.raises(ValueError):memory.ingest([valid,delayed],.1)
    assert len(memory.observed_cells(.1)['centers_m'])==0
    memory.ingest([delayed],.15)
    assert memory.query([.005,.005],.15)['supported']


def test_memory_survives_new_frame_occlusion_while_latest_batch_is_unknown():
    memory=TerrainMemory();memory.ingest([frame()],0.)
    empty=frame(capture=1.,delivery=1.05,valid=np.zeros((11,11),bool))
    memory.ingest([empty],1.05)
    current=TerrainMemory();current.ingest([empty],1.05)
    result=memory.query([.005,.005],1.05)
    assert result['supported'] and result['first_capture_s']==0. and result['age_s']==1.05
    assert not current.query([.005,.005],1.05)['supported']


def test_unseen_cell_is_unknown_even_inside_surrounding_flat_support():
    valid=np.ones((11,11),bool);valid[5,5]=False
    memory=TerrainMemory();memory.ingest([frame(valid=valid)],0.)
    result=memory.query([.005,.005],0.)
    assert not result['supported'] and result['reason']=='unobserved_cell'


def test_observed_boundary_cell_does_not_authorize_extrapolation():
    memory=TerrainMemory();memory.ingest([frame()],0.)
    result=memory.query([.059,.059],0.)
    assert not result['supported'] and result['reason']=='outside_observed_support_hull'


def test_two_heights_in_one_cell_abstain_instead_of_averaging_surface():
    memory=TerrainMemory();memory.ingest([frame()],0.)
    memory.ingest([frame(capture=.1,delivery=.1,height=.04)],.1)
    result=memory.query([.005,.005],.1)
    assert not result['supported'] and result['reason']=='multiple_heights_in_query_cell'


def test_collinear_support_and_explicitly_stale_support_abstain():
    valid=np.zeros((11,11),bool);valid[5,:]=True
    memory=TerrainMemory();memory.ingest([frame(valid=valid)],0.)
    assert memory.query([.005,.005],0.)['reason']=='degenerate_xy_support'
    stale=TerrainMemory(MemorySettings(max_age_s=.1));stale.ingest([frame()],0.)
    assert stale.query([.005,.005],.2)['reason']=='stale_query_cell'


def test_duplicate_frames_do_not_inflate_memory_and_conflicts_reject():
    memory=TerrainMemory();observation=frame();first=memory.ingest([observation],0.)
    duplicate=memory.ingest([observation],.1)
    assert duplicate['frames_added']==0 and duplicate['points_added']==0
    assert first['points_added']==int(memory.observed_cells(.1)['count'].sum())
    conflict=copy.deepcopy(observation);conflict.depth_m*=.95
    with pytest.raises(ValueError):memory.ingest([conflict],.1)


def test_snapshot_and_export_isolation_prevent_future_history_leak():
    memory=TerrainMemory();memory.ingest([frame()],0.);snapshot=memory.snapshot(.05)
    memory.ingest([frame(capture=.1,delivery=.1,height=.04)],.1)
    with pytest.raises(ValueError):memory.query([.005,.005],.05)
    assert snapshot.query([.005,.005],.05)['height_m']==pytest.approx(0.,abs=1e-12)
    exported=snapshot.observed_cells(.05);exported['centers_m'][:]=999.
    assert snapshot.query([.005,.005],.05)['height_m']==pytest.approx(0.,abs=1e-12)


def test_cv_uses_world_translation_and_right_body_angular_integration():
    q=state(yaw=.7);velocity=np.array([.1,-.03,.02,.2,-.15,.3,.7,.4,-.2]);horizon=.2
    predicted=propagate_cv(q,velocity,horizon)
    actual=Rotation.from_quat(q[[4,5,6,3]])*Rotation.from_rotvec(velocity[3:6]*horizon)
    np.testing.assert_allclose(predicted[:3],q[:3]+velocity[:3]*horizon,atol=1e-15)
    np.testing.assert_allclose(Rotation.from_quat(predicted[[4,5,6,3]]).as_matrix(),actual.as_matrix(),atol=1e-14)
    np.testing.assert_allclose(predicted[7:],q[7:]+velocity[6:]*horizon,atol=1e-15)
    np.testing.assert_array_equal(q,state(yaw=.7))


def test_body_yaw_and_shaft_rotation_determine_support_identity():
    plane=Plane(normal=(-.15,0.,1.));normal=plane.normal;center=GEOMETRY.radius_m*normal
    first=state(phase=0.,center=center);second=state(phase=0.,yaw=np.pi,center=center)
    surface=plane.query(center[:2],0.)
    # This plane passes through world origin, so its point atcenterXY is valid.
    assert {r['patch'] for r in contact_candidates(surface,first,GEOMETRY)['candidates']}=={'shell_A'}
    assert {r['patch'] for r in contact_candidates(surface,second,GEOMETRY)['candidates']}=={'shell_B'}
    first[8:]=[2.,-3.]
    np.testing.assert_allclose(carrier_axis(first),[1.,0.,0.],atol=1e-14)


def test_pole_is_belt_and_polar_boundary_retains_both_components():
    surface=Plane().query([0.,0.],0.)
    pole=contact_candidates(surface,state(phase=np.pi/2),GEOMETRY)['candidates']
    assert {r['patch'] for r in pole}=={'belt_A'}
    boundary=np.pi/2-np.radians(GEOMETRY.polar_cap_angle_deg)
    both=contact_candidates(surface,state(phase=boundary),GEOMETRY)['candidates']
    assert {r['patch'] for r in both}=={'shell_A','belt_A'}
    assert all(r['feature']=='polar_edge' for r in both)


def test_sloped_pole_roundoff_retains_belt_contact():
    # Axial coordinate rounds one ulp above radius on this valid upward plane.
    # A zero-radius polar ring has a unique pole, requiring no radial direction.
    q=np.array([.018911952494233754,.07627199300593433,.09069135094240548,
        .5654683610430311,-.17047282124699234,.7458851709347084,-.3079608118821292,
        -.14961868735132722,0.,0.])
    normal=np.array([.15759960411861462,.6355999417161194,.755761257853379])
    surface={'supported':True,'point_m':np.zeros(3),'normal':normal,'height_m':0.}
    candidates=contact_candidates(surface,q,GEOMETRY)['candidates']
    assert {row['patch'] for row in candidates}=={'belt_A'}
    np.testing.assert_allclose(candidates[0]['point_m'],np.zeros(3),atol=1e-15,rtol=0.)


def test_equatorial_gap_returns_ideal_rims_without_load_shares():
    height=np.sqrt(GEOMETRY.radius_m**2-GEOMETRY.equatorial_gap_half_m**2)
    surface=Plane().query([0.,0.],0.)
    rims=contact_candidates(surface,state(phase=0.,center=(0.,0.,height)),GEOMETRY)['candidates']
    assert {r['patch'] for r in rims}=={'shell_A','shell_B'}
    assert all(r['feature']=='rim' and r['rim_idealized'] for r in rims)
    assert all(abs(r['clearance_m'])<1e-14 and 'load' not in r for r in rims)


def test_preview_freezes_knowledge_time_and_distinguishes_known_clearance():
    provider=Plane();velocity=np.zeros(9);velocity[0]=.1
    result=preview_contacts(provider,GEOMETRY,state(),velocity,1.)
    assert len(result['rows'])==3 and all(t==1. for _,t in provider.calls)
    assert all(row['supported_surface'] and row['contact_prediction_supported'] for row in result['rows'])
    np.testing.assert_allclose(result['rows'][-1]['predicted_qpos'][0],.02)
    airborne=preview_contacts(Plane(),GEOMETRY,state(center=(0.,0.,1.)),np.zeros(9),1.)['rows'][0]
    assert airborne['supported_surface'] and airborne['supported'] and airborne['known_no_near_contact'] and not airborne['candidates']
    unseen=preview_contacts(TerrainMemory(),GEOMETRY,state(),np.zeros(9),0.)['rows'][0]
    assert not unseen['supported_surface'] and not unseen['known_no_near_contact']


def test_unknown_contact_region_is_not_filled_from_known_center_plane():
    class CenterOnly(Plane):
        def query(self,xy,at_time_s):
            if np.linalg.norm(xy)>1e-6:return {'supported':False,'reason':'unobserved_cell'}
            return super().query(xy,at_time_s)
    row=preview_contacts(CenterOnly(normal=(-.2,0.,1.)),GEOMETRY,state(phase=0.),np.zeros(9),0.)['rows'][0]
    assert row['supported_surface'] and not row['contact_prediction_supported']
    assert row['reason']=='partial_contact_region_unknown' and not row['candidates']
