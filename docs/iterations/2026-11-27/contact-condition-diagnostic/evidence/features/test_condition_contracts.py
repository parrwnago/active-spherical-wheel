"""Synthetic causality, boundary and frame contracts; no historical samples."""
import importlib.util
from pathlib import Path
from unittest.mock import patch
import numpy as np
import pytest

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_i63_test_features',HERE/'condition_features.py');F=importlib.util.module_from_spec(spec);spec.loader.exec_module(F)


def example(n=5):
    radius=.12;seam=radius*np.cos(.3)
    axes=np.tile(np.array([[1.,0.,0.],[-1.,0.,0.],[1.,0.,0.],[-1.,0.,0.]]),(n,1,1))
    gamma=np.zeros((n,9));gamma[:,2]=10.
    normal=np.tile([-np.cos(.3),-np.sin(.3),0.],(n,1))
    return dict(time_s=np.arange(n)*.005,observer_generalized=gamma,observer_interval_mean=gamma.copy(),
        intrinsic_normal_hint=normal,patch_axis_world=axes,patch_lower_m=np.array([.01,.01,seam,seam]),
        patch_upper_m=np.array([seam,seam,radius,radius]),radius_m=np.asarray(radius))


def test_future_changes_do_not_change_any_prefix_feature():
    first=example(7);second={key:value.copy() for key,value in first.items()}
    second['observer_interval_mean'][4:,6:9]=100.;second['intrinsic_normal_hint'][4:]=[0.,0.,1.]
    second['patch_axis_world'][4:]=np.roll(second['patch_axis_world'][4:],1,axis=2)
    a,b=F.derive_features(first),F.derive_features(second)
    for key in a:np.testing.assert_array_equal(a[key][:4],b[key][:4])


def test_first_and_gapped_or_invalid_predecessors_have_unknown_risk():
    data=example();data['time_s'][2:]+=.005;data['intrinsic_normal_hint'][3]=np.nan
    out=F.derive_features(data)
    np.testing.assert_array_equal(out['risk_valid'],[False,True,False,False,False])
    assert np.isnan(out['combined_risk_r'][~out['risk_valid']]).all()


def test_joint_risk_uses_maxabs_and_old_load_scale_not_l2():
    data=example();data['observer_interval_mean'][:,6:9]=[.075,.075,0.]
    out=F.derive_features(data)
    np.testing.assert_allclose(out['joint_innovation_scale_Nm'],.15)
    np.testing.assert_allclose(out['joint_load_risk_u'],.5)
    np.testing.assert_allclose(out['joint_innovation_normalized'],np.sqrt(2.)*.075/1.2)


def test_force_floor_and_nonfinite_observer_remain_unknown():
    data=example();data['observer_generalized'][1,2]=.5;data['observer_interval_mean'][2,7]=np.nan
    out=F.derive_features(data)
    assert not out['risk_valid'][1] and not out['risk_valid'][2]
    assert np.isnan(out['combined_risk_r'][1:3]).all()


def test_shell_belt_boundary_is_zero_arclength_not_equatorial_rim():
    out=F.derive_features(example())
    np.testing.assert_allclose(out['seam_signed_offset_m'][:,[0,2]],0.,atol=1e-15)
    assert (out['lower_signed_margin_m'][:,0]>.1).all()
    np.testing.assert_allclose(out['seam_risk_s'][1:],1.,atol=1e-13)


def test_pole_has_surface_distance_outside_static_12mm_window():
    data=example();data['intrinsic_normal_hint'][:]=[-1.,0.,0.]
    out=F.derive_features(data)
    np.testing.assert_allclose(out['nearest_seam_distance_m'],.036)
    assert (np.abs(out['seam_axial_signed_offset_m'][:,0])<.012).all()
    np.testing.assert_array_equal(out['seam_risk_s'][1:],0.)


def test_signed_arclength_rate_is_per_seam_and_uses_both_motion_directions():
    data=example();angles=np.array([.3,.31,.32,.31,.30])
    data['intrinsic_normal_hint'][:,0]=-np.cos(angles);data['intrinsic_normal_hint'][:,1]=-np.sin(angles)
    out=F.derive_features(data)
    np.testing.assert_allclose(out['seam_signed_offset_m'][:,0],.12*(angles-.3),atol=1e-15)
    np.testing.assert_allclose(out['seam_signed_rate_m_s'][1:,0],[.24,.24,-.24,-.24],atol=1e-12)
    assert (out['per_seam_risk'][1:,0]>.9).all()


def test_corotating_normal_and_axes_preserves_all_geometry():
    first=example();second={key:value.copy() for key,value in first.items()}
    rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    second['intrinsic_normal_hint']=first['intrinsic_normal_hint']@rotation.T
    second['patch_axis_world']=first['patch_axis_world']@rotation.T
    a,b=F.derive_features(first),F.derive_features(second)
    for key in ['axial_coordinate_m','seam_signed_offset_m','seam_signed_rate_m_s','seam_risk_s']:
        np.testing.assert_allclose(a[key],b[key],atol=1e-14,equal_nan=True)


def test_axis_rotation_without_normal_rotation_changes_geometry():
    data=example();data['patch_axis_world']=np.roll(data['patch_axis_world'],1,axis=2)
    out=F.derive_features(data)
    assert np.all(np.abs(out['seam_signed_offset_m'][:,0])>.1)


def test_duplicate_or_backwards_times_are_rejected():
    data=example();data['time_s'][2]=data['time_s'][1]
    with pytest.raises(ValueError,match='Strictly increasing'):F.derive_features(data)


def test_normal_scale_does_not_change_geometry_and_zero_normal_is_unknown():
    first=example();second={key:value.copy() for key,value in first.items()};second['intrinsic_normal_hint']*=17.
    np.testing.assert_allclose(F.derive_features(first)['seam_signed_offset_m'],F.derive_features(second)['seam_signed_offset_m'],atol=1e-15)
    second['intrinsic_normal_hint'][2]=0.;out=F.derive_features(second)
    assert not out['risk_valid'][2] and not out['risk_valid'][3]


def test_private_kinematics_rotates_known_axes_without_physics_or_contact_query():
    import mujoco
    xml='''<mujoco><size nuser_geom="5"/><worldbody><body name="fork"><freejoint/>
    <geom name="belt_A_smooth" size=".12" user="1 .11 .12 4 .12"/>
    <geom name="belt_B_smooth" size=".12" user="-1 .11 .12 4 .12"/>
    <body name="shaft"><joint axis="1 0 0"/><geom size=".01"/>
    <body name="a"><joint axis="1 0 0"/><geom name="shell_A_smooth" size=".12" user="1 .01 .11 2 .12"/></body>
    <body name="b"><joint axis="1 0 0"/><geom name="shell_B_smooth" size=".12" user="-1 .01 .11 2 .12"/></body>
    </body></body></worldbody></mujoco>'''
    model=mujoco.MjModel.from_xml_string(xml);private=F.PatchAxisKinematics(model);q=model.qpos0.copy()
    q[3:7]=[np.sqrt(.5),0.,0.,np.sqrt(.5)]
    with patch.object(mujoco,'mj_forward',side_effect=AssertionError('physics')),patch.object(mujoco,'mj_contactForce',side_effect=AssertionError('contact')):
        axes=private.axes(q)
    np.testing.assert_allclose(axes,[[0.,1.,0.],[0.,-1.,0.],[0.,1.,0.],[0.,-1.,0.]],atol=1e-14)
    assert not np.shares_memory(q,private.data.qpos)
