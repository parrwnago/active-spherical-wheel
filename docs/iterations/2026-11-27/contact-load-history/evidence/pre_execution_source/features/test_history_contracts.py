"""Synthetic temporal and kinematic contracts; no original-data arithmetic."""
from pathlib import Path
from unittest.mock import patch
import importlib.util
import numpy as np
import pytest

HERE=Path(__file__).resolve().parent
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value
H=module(HERE/'history_features.py','_i64_test_history')


def example(n=110,varying=False):
    time=np.arange(n)*.005;raw=np.zeros((n,9));raw[:,0]=10.;raw[:,2]=5.
    maps=np.zeros((n,3,3));maps[:,0,0]=.03;maps[:,1,2]=.04
    if varying:maps[:,0,0]+=.02*np.sin(20.*time)
    raw[:,6:9]=np.einsum('nij,nj->ni',maps,raw[:,:3])
    gamma=raw.copy()
    for i in range(1,n):gamma[i]=H._step(gamma[i-1],raw[i],time[i]-time[i-1],100.)
    return dict(time_s=time,observer_generalized=gamma,observer_interval_mean=raw,radius_m=np.asarray(.12),
        H_F=maps,H_Nalt=.5*maps,H_Neq=maps.copy(),H_F_valid=np.ones(n,bool),
        H_Nalt_valid=np.ones(n,bool),H_Neq_valid=np.ones(n,bool),i60_accepted=np.ones(n,bool))


def test_observer_full9_recurrence_is_reconstructed_in_original_operation_order():
    p=example(varying=True);out=H.derive_history(p)
    np.testing.assert_array_equal(out['observer_reconstructed9'],p['observer_generalized'])
    np.testing.assert_array_equal(out['observer_full_recurrence_error9'],0.)


def test_constant_map_filter_commutes_and_raw_force_enters_only_once():
    p=example();p['observer_interval_mean'][55:,0]=20.
    p['observer_interval_mean'][:,6:9]=np.einsum('nij,nj->ni',p['H_F'],p['observer_interval_mean'][:,:3])
    for i in range(1,len(p['time_s'])):p['observer_generalized'][i]=H._step(p['observer_generalized'][i-1],p['observer_interval_mean'][i],p['time_s'][i]-p['time_s'][i-1],100.)
    out=H.derive_history(p);valid=out['M2_F_valid']
    np.testing.assert_allclose(out['M0_F_residual_Nm'],0.,atol=2e-16)
    np.testing.assert_allclose(out['M2_F_residual_Nm'][valid],0.,atol=2e-16)
    double=np.zeros_like(p['observer_generalized'][:,:3]);double[0]=p['observer_generalized'][0,:3]
    for i in range(1,len(double)):double[i]=H._step(double[i-1],p['observer_generalized'][i,:3],p['time_s'][i]-p['time_s'][i-1],100.)
    double_prediction=np.einsum('nij,nj->ni',p['H_F'],double)
    assert np.max(abs(double_prediction-p['observer_generalized'][:,6:9]))>.05


def test_varying_map_does_not_commute_but_correct_matched_history_does():
    out=H.derive_history(example(varying=True));valid=out['M2_F_valid']
    assert np.max(abs(out['M0_F_residual_Nm'][valid]))>.01
    np.testing.assert_allclose(out['M2_F_residual_Nm'][valid],0.,atol=2e-16)


def test_observed_prior_one_step_is_only_scaled_raw_residual():
    p=example(varying=True);p['H_F']*=.7;out=H.derive_history(p)
    np.testing.assert_allclose(out['one_step_F_identity_error_Nm'][1:],0.,atol=2e-16)
    assert np.max(abs(out['one_step_F_residual_Nm'][1:]))>0


def test_inherited_plus_current_decomposition_matches_common_history_residual():
    p=example(varying=True);p['H_Neq']*=.6;out=H.derive_history(p)
    for model in ['M2','M3']:
        for branch in ['F','Nalt']:
            valid=out[model+'_'+branch+'_valid']
            np.testing.assert_allclose(out[model+'_'+branch+'_decomposition_error_Nm'][valid],0.,atol=2e-16)
            share=out[model+'_'+branch+'_inherited_norm_share']+out[model+'_'+branch+'_current_norm_share']
            np.testing.assert_allclose(share[np.isfinite(share)],1.,atol=2e-16)


def test_counterfactual_candidate_cannot_rewrite_committed_history():
    first=example();first['i60_accepted'][:]=False
    second={key:value.copy() for key,value in first.items()};second['H_F'][60:]*=2.
    a,b=H.derive_history(first),H.derive_history(second)
    np.testing.assert_array_equal(a['M2_anchor_after_Nm'],b['M2_anchor_after_Nm'])
    assert np.max(abs(a['M2_F_residual_Nm'][60:]-b['M2_F_residual_Nm'][60:]))>.1


def test_missing_alternative_does_not_reset_valid_committed_path():
    p=example();p['H_Nalt_valid'][60]=False;out=H.derive_history(p)
    assert not out['M2_Nalt_valid'][60] and out['M2_F_valid'][60]
    assert not out['M2_reset'][60] and out['M2_F_valid'][61]


def test_invalid_committed_path_reseeds_then_requires_full200ms():
    p=example();p['H_F_valid'][50]=False;out=H.derive_history(p)
    assert out['M2_reset'][50] and out['M2_seeded'][51]
    assert not out['M2_F_valid'][90] and out['M2_F_valid'][91]
    assert np.isnan(out['M2_F_residual_Nm'][51]).all()
    assert out['M2_seed_weight'][51]==1.
    np.testing.assert_allclose(out['M2_seed_weight'][91],np.exp(-20.),rtol=1e-12)


def test_time_gap_row_is_invalid_and_following_adjacent_row_seeds():
    p=example();p['time_s'][55:]+=.005;out=H.derive_history(p)
    assert out['M2_reset'][55] and not out['M2_chain_active'][55]
    assert out['M2_seeded'][56] and not out['M2_F_valid'][56]


def test_full_observer_control_does_not_apply_last_interval_mean_over_a_time_gap():
    p=example();p['time_s'][55:]+=.005;out=H.derive_history(p)
    assert np.isnan(out['observer_reconstructed9'][55]).all()
    assert not out['observer_reconstruction_valid'][55]
    assert out['observer_reconstruction_seeded'][56] and not out['observer_reconstruction_valid'][56]
    np.testing.assert_array_equal(out['observer_reconstructed9'][56],p['observer_generalized'][56])
    assert out['observer_reconstruction_valid'][57] and not out['observer_reconstruction_seeded'][57]


def test_low_filtered_force_breaks_history_but_not_recurrence_verification():
    p=example();p['observer_generalized'][50,:3]=0.;out=H.derive_history(p)
    assert not out['observation_valid'][50] and out['M2_reset'][50]
    assert np.isfinite(out['observer_reconstructed9'][50]).all()


def test_future_data_changes_preserve_every_prefix_output():
    first=example(varying=True);second={key:value.copy() for key,value in first.items()}
    second['observer_interval_mean'][80:]+=3.;second['observer_generalized'][80:]+=7.;second['H_F'][80:]*=2.
    a,b=H.derive_history(first),H.derive_history(second)
    for key in a:np.testing.assert_array_equal(a[key][:80],b[key][:80])


def test_measured_initial_seed_does_not_count_as_model_consistency():
    p=example();p['H_F']*=4.;out=H.derive_history(p)
    np.testing.assert_array_equal(out['M2_committed_residual_Nm'][0],0.)
    assert not out['M2_F_valid'][:40].any() and out['M2_F_valid'][40]


def test_native_anchor_is_sensitivity_control_not_selected_from_alternatives():
    p=example(varying=True);p['H_Neq']*=.6;out=H.derive_history(p)
    valid=out['M2_F_valid']&out['M3_F_valid']
    np.testing.assert_allclose(out['M2_F_residual_Nm'][valid],0.,atol=2e-16)
    assert np.max(abs(out['M3_F_residual_Nm'][valid]))>.03


def test_duplicate_or_backwards_times_are_rejected():
    p=example();p['time_s'][20]=p['time_s'][19]
    with pytest.raises(ValueError,match='Strictly increasing'):H.derive_history(p)


def kinematic_fixture():
    import mujoco
    xml='''<mujoco><size nuser_geom="5"/><worldbody><body name="fork"><freejoint/>
    <geom name="belt_A_smooth" size=".12" user="1 .11 .12 4 .12"/>
    <geom name="belt_B_smooth" size=".12" user="-1 .11 .12 4 .12"/>
    <body name="shaft"><joint axis="0 0 1"/><geom size=".01"/>
    <body name="a"><joint axis="1 0 0"/><geom name="shell_A_smooth" size=".12" user="1 .01 .11 2 .12"/></body>
    <body name="b"><joint axis="1 0 0"/><geom name="shell_B_smooth" size=".12" user="-1 .01 .11 2 .12"/></body>
    </body></body></worldbody></mujoco>'''
    root=HERE.parents[2];core=module(root/'src/spherical_wheel/contact_load_consistency.py','_i64_test_core')
    geometry=module(root/'src/spherical_wheel/contact_hypotheses.py','_i64_test_geometry')
    model=mujoco.MjModel.from_xml_string(xml);return H.FrozenEndpointMaps(model,core,geometry),model.qpos0.copy()


def test_world_force_mapping_and_virtual_work_without_physics_or_contacts():
    import mujoco
    maps,q=kinematic_fixture();point=.12*np.array([np.cos(.5),np.sin(.5),0.]);force=np.array([2.,3.,5.])
    with patch.object(mujoco,'mj_forward',side_effect=AssertionError('physics')),patch.object(mujoco,'mj_contactForce',side_effect=AssertionError('contact')):
        value=maps.point_map(q,point,0)
    assert value['map_valid'] and value['material_point_valid']
    jac=np.zeros((3,maps.model.nv));mujoco.mj_jac(maps.model,maps.data,jac,None,point,maps.core.geometry.patches[0]['body'])
    speed=np.array([1.,2.,3.]);np.testing.assert_allclose((value['H']@force)@speed,force@(jac[:,6:9]@speed),atol=1e-15)
    np.testing.assert_allclose(value['H']@(force+force),value['H']@force+value['H']@force)


def test_virtual_native_point_is_not_silently_projected_to_sphere():
    maps,q=kinematic_fixture();point=.12*np.array([np.cos(.5),np.sin(.5),0.])
    physical=maps.point_map(q,point,0);virtual=maps.point_map(q,.5*point,0)
    assert virtual['map_valid'] and not virtual['material_point_valid']
    np.testing.assert_allclose(virtual['H'],.5*physical['H'],atol=1e-15)


def test_rotating_pose_point_and_force_preserves_scalar_joint_load():
    maps,q=kinematic_fixture();point=.12*np.array([np.cos(.5),np.sin(.5),0.]);force=np.array([2.,3.,5.])
    first=maps.point_map(q,point,0)['H']@force
    q[3:7]=[np.sqrt(.5),0.,0.,np.sqrt(.5)];rotation=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
    second=maps.point_map(q,rotation@point,0)['H']@(rotation@force)
    np.testing.assert_allclose(first,second,atol=1e-15)


def test_carrier_contact_has_no_external_shell_joint_load():
    maps,q=kinematic_fixture();value=maps.point_map(q,[.12,0.,0.],2)
    assert value['map_valid'] and value['material_point_valid']
    np.testing.assert_array_equal(value['H'],np.zeros((3,3)))
