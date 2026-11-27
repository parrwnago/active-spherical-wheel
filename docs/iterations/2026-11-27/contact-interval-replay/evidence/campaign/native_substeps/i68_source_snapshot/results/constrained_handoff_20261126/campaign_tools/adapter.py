"""Explicit observer-only I56 adapter; native code/targets are not modified."""
from dataclasses import asdict
from time import perf_counter
import numpy as np
from spherical_wheel.handoff_control import BoundedHandoffAllocator, HandoffSettings
from spherical_wheel.handoff_model import MeasuredContactVelocityModel


def create(controller, settings):
    if controller.params.mode != 'hardware' or controller.locator is None:
        raise ValueError('I56 adapter requires the observer-only contact boundary')
    allowed={'allocator','contact_strength','slip_confidence_scale_m_s'}
    if set(settings)-allowed:raise ValueError('Unknown candidate settings')
    allocator_settings=HandoffSettings(**settings['allocator'])
    scale=float(settings['slip_confidence_scale_m_s'])
    if not np.isfinite(scale) or scale<=0:raise ValueError('Positive mismatch confidence scale required')
    model=MeasuredContactVelocityModel(controller.plant.model,contact_strength=float(settings['contact_strength']))
    return dict(model=model,allocator=BoundedHandoffAllocator(allocator_settings),
                settings=asdict(allocator_settings),slip_scale=scale)


def apply(instance, controller, native_requested, native_applied, info):
    started=perf_counter();plant=controller.plant
    np.testing.assert_array_equal(native_applied,np.clip(native_requested,-plant.bounds,plant.bounds))
    # Pure locator geometry is reconstructed from the very same observer wrench;
    # there is no observer update, contact query, terrain access or live forward.
    records=controller.locator.records(plant.data,*controller.observer.wrench())
    mismatch=float(info['measurement']['slip_speed_m_s'])
    if not np.isfinite(mismatch) or mismatch<0:raise ValueError('Finite measured material-velocity mismatch required')
    ratio=min(mismatch/instance['slip_scale'],1e6)
    confidence=1./(1.+ratio*ratio)
    response=instance['model'].evaluate(plant.data.qpos,plant.data.qvel,records,
        horizon_s=instance['allocator'].settings.horizon_s,tangent_confidence=confidence)
    model_finished=perf_counter()
    weights=np.asarray(info['sync_weights'],float)
    preparation=np.asarray(info['reference_qvel'],float)[7:9].copy()
    readiness=weights.copy()
    limit=float(controller.synchronizer.shell_blend_limit)
    readiness[1:3]=np.clip(readiness[1:3]/max(limit,1e-12),0.,1.)
    for target in controller._sync['targets']:
        index=int(target['command_index'])
        if index in (1,2) and 'target_joint_speed_rad_s' in target:
            preparation[index-1]=float(target['target_joint_speed_rad_s'])
    result=instance['allocator'].solve(native_requested,plant.bounds,plant.last_input,plant.dt,
        plant.data.qvel,response.response,response.fork_rotation_world,response.shell_axis_world,
        response.shell_axial_inertias_kg_m2,np.asarray(info['estimated_support_loads_N'],float),
        np.asarray(info['reference_qvel'],float)[7:9],readiness=readiness,
        preparation_shell_rates=preparation)
    finished=perf_counter();command=result.command.copy();diagnostic=dict(result.diagnostics)
    nominal=np.asarray(native_applied)
    native_impulse=response.contact_impulse_offset+response.contact_impulse_response@nominal
    governed_impulse=response.contact_impulse_offset+response.contact_impulse_response@command
    diagnostic.update(contact_names=list(response.contact_names),contact_model_rank=response.constraint_rank,
        tangent_confidence_proxy=confidence,estimated_material_velocity_mismatch_m_s=mismatch,
        modeled_contact_loads_N=response.contact_loads_N.copy(),
        native_contact_velocity_residual_norm_m_s=float(np.linalg.norm(response.constraint_offset+response.constraint_response@nominal)),
        governed_contact_velocity_residual_norm_m_s=float(np.linalg.norm(response.constraint_offset+response.constraint_response@command)),
        native_soft_contact_impulse_Ns=native_impulse,governed_soft_contact_impulse_Ns=governed_impulse,
        model_seconds=model_finished-started,allocation_seconds=finished-model_finished,total_seconds=finished-started,
        model_warning='Soft held-geometry sensitivity; projected impulses not constrained unilateral/friction feasible. Command bounds do not bound actual acceleration.')
    return command.copy(),command.copy(),diagnostic
