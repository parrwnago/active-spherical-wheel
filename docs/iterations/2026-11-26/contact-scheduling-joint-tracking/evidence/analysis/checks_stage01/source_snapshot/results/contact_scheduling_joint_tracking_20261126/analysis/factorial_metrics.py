"""I52 pure saved-array joint-tracking, support/phase and factorial arithmetic.

No controller, observer, dynamics, kinematics or contact-prediction imports.
Differences are descriptive effects of the registered deterministic policies;
one run per cell does not support a sampling-uncertainty or significance claim.
"""
from __future__ import annotations

import numpy as np

PATCHES=('shell_A','shell_B','belt_A','belt_B')
JOINTS=('shaft','hemisphere_A','hemisphere_B')
VARIANTS=('B','R','N','NR')


def wrapped(angle):
    return np.arctan2(np.sin(angle),np.cos(angle))


def held_selection(times,window=None):
    """Commands at last recorded time have no subsequent held interval."""
    t=np.asarray(times,float);mask=np.zeros(len(t),bool)
    if len(t)>1:mask[:-1]=True
    if window is not None:mask&=(t>=window[0]-1e-9)&(t<window[1]-1e-9)
    return mask


def vector_stats(values):
    a=np.asarray(values,float)
    if a.ndim!=2 or a.shape[1]!=3:raise ValueError('Expected [sample,3] joint vectors')
    if not len(a):return dict(samples=0,available=False)
    norms=np.linalg.norm(a,axis=1)
    return dict(samples=len(a),available=True,
        rms_byjoint=np.sqrt(np.mean(a*a,axis=0)),mae_byjoint=np.mean(abs(a),axis=0),
        p95_abs_byjoint=np.quantile(abs(a),.95,axis=0),max_abs_byjoint=np.max(abs(a),axis=0),
        rms_norm=float(np.sqrt(np.mean(norms*norms))),p95_norm=float(np.quantile(norms,.95)),max_norm=float(norms.max()))


def joint_tracking(raw,guard_enabled,windows,dt=.005):
    t=raw['time_s'];actual=raw['exact_qvel'][:,6:9]
    native=raw['exact_native_joint_reference'];guarded=raw['exact_guarded_joint_reference']
    if actual.shape!=native.shape or actual.shape!=guarded.shape:raise ValueError('Joint vector shapes differ')
    modified=np.any(guarded!=native,axis=1);held=held_selection(t)
    steps=np.diff(guarded,axis=0)/np.diff(t)[:,None] if len(t)>1 else np.empty((0,3))
    output=dict(joint_order=JOINTS,units='rad/s for rates/errors; rad/s² for reference changes',
        interpretation='Current measured rates versus current requested references, before the next command is applied. This is not a motor step-response identification. Native and guarded targets are both retained; easier targets alone are not proof of improved motion.',
        reference_guard_enabled=bool(guard_enabled),modified_held_samples=int(modified[held].sum()),modified_held_duration_s=float(modified[held].sum()*dt),
        first_modified_time_s=float(t[np.flatnonzero(modified)[0]]) if modified.any() else None,
        max_abs_native_reference_rad_s=float(abs(native[held]).max()) if held.any() else None,
        max_abs_guarded_reference_rad_s=float(abs(guarded[held]).max()) if held.any() else None,
        max_abs_actual_joint_rate_rad_s=float(abs(actual[held]).max()) if held.any() else None,
        max_abs_guarded_reference_acceleration_rad_s2=float(abs(steps).max()) if len(steps) else None,
        reference_acceleration_scope='All emitted references, including terminal emission, for the mechanistic bound check. Error/rate/exposure summaries exclude the terminal unheld row.',
        speed_clip_held_samples_byjoint=np.sum(abs(native[held])>20.,axis=0),windows={})
    for name,window in windows.items():
        selected=held_selection(t,window)
        output['windows'][name]=dict(window_s=window,held_samples=int(selected.sum()),held_duration_s=float(selected.sum()*dt),
            complete=bool(window is None or t[0]<=window[0]+1e-9 and t[-1]>=window[1]-1e-9),
            actual=vector_stats(actual[selected]),native_reference=vector_stats(native[selected]),guarded_reference=vector_stats(guarded[selected]),
            actual_minus_native_ref=vector_stats(actual[selected]-native[selected]),
            actual_minus_guarded_ref=vector_stats(actual[selected]-guarded[selected]))
    return output


def phase_metrics(raw,normal_enabled,dt=.005):
    t=raw['time_s'];selected=held_selection(t)
    vertical=raw['exact_phase_world_vertical_rad'];normal=raw['exact_phase_estimated_normal_rad'];chosen=raw['exact_effective_phase_rad']
    expected=normal if normal_enabled else vertical
    flags=np.asarray(raw['exact_schedule_on_ground_normal'],bool)
    step=np.degrees(wrapped(np.diff(chosen[selected])))
    delta=np.degrees(wrapped(normal[selected]-vertical[selected]))
    carrier=raw['exact_estimated_normal_carrier'][selected]
    normal_xz=raw['exact_phase_normal_xz_norm'][selected]
    vertical_xz=raw['exact_phase_world_xz_norm'][selected]
    return dict(source='Current carrier rotation and estimated normal; no true terrain normal used by controller.',
        normal_phase_enabled=bool(normal_enabled),flag_matches_registered_policy=bool(np.all(flags==normal_enabled)),
        selected_phase_matches_registered_source=bool(np.max(abs(wrapped(chosen-expected)))<1e-10),
        selected_phase_max_abs_wrapped_step_deg=float(abs(step).max()) if len(step) else None,
        selected_phase_total_wrapped_variation_deg=float(abs(step).sum()),
        same_current_state_normal_minus_vertical_deg=dict(median_abs=float(np.median(abs(delta))) if len(delta) else None,
            p95_abs=float(np.quantile(abs(delta),.95)) if len(delta) else None,max_abs=float(abs(delta).max()) if len(delta) else None),
        carrier_normal_projection=dict(held_samples=int(selected.sum()),
            max_abs_carrier_Y=float(abs(carrier[:,1]).max()) if len(carrier) else None,
            p95_abs_carrier_Y=float(np.quantile(abs(carrier[:,1]),.95)) if len(carrier) else None,
            min_normal_XZ_norm=float(normal_xz.min()) if len(normal_xz) else None,
            p05_normal_XZ_norm=float(np.quantile(normal_xz,.05)) if len(normal_xz) else None,
            min_world_vertical_XZ_norm=float(vertical_xz.min()) if len(vertical_xz) else None,
            normal_XZ_undefined_held_samples=int(np.sum(normal_xz<=1e-12)),
            world_vertical_XZ_undefined_held_samples=int(np.sum(vertical_xz<=1e-12)),
            interpretation='The one-angle lookup omits carrier-Y normal information; no new control threshold is introduced.'),
        interpretation='Wrapped angular increments avoid artificial2π jumps. Phase variation is a descriptive scheduling signal, not a torque change or causal proof of path error.')


def stable_runs(times,labels,dwell_s=.015):
    t=np.asarray(times);labels=np.asarray(labels);runs=[]
    if not len(t):return runs
    edges=np.r_[0,np.flatnonzero(labels[1:]!=labels[:-1])+1,len(t)]
    for start,stop in zip(edges[:-1],edges[1:]):
        last=stop-1;span=float(t[last]-t[start])
        if span+1e-10>=dwell_s:runs.append(dict(label=str(labels[start]),first_index=int(start),last_index=int(last),first_s=float(t[start]),last_s=float(t[last]),dwell_span_s=span))
    return runs


def support_modes(raw,weight_N,dt=.005):
    t=raw['time_s'];selected=held_selection(t);minimum=max(.05,.05*float(weight_N));result={};masks={}
    for source,key in [('estimated','exact_estimated_support_loads_N'),('truth','truth_patch_loads_N')]:
        loads=np.asarray(raw[key],float)[selected];total=loads.sum(axis=1);active=total>=minimum
        shares=np.divide(loads,total[:,None],out=np.zeros_like(loads),where=total[:,None]>0)
        loaded=(shares>=.05)&active[:,None];code=np.sum(loaded*(1<<np.arange(4)),axis=1);masks[source]=(active,code)
        dominant=np.where(active,np.argmax(loads,axis=1),-1)
        family=np.full(len(total),'unloaded',dtype='<U10');family[active]='mixed'
        family[active&(shares[:,:2].sum(axis=1)>=.8)]='shell';family[active&(shares[:,2:].sum(axis=1)>=.8)]='belt'
        labels,count=np.unique(family,return_counts=True);codes,counts=np.unique(code,return_counts=True)
        result[source]=dict(held_samples=len(total),held_duration_s=float(len(total)*dt),
            family_duration_s={str(k):float(v*dt) for k,v in zip(labels,count)},
            patch_loaded_duration_s={k:float(v*dt)for k,v in zip(PATCHES,loaded.sum(axis=0))},
            loaded_set_code_duration_s={str(int(k)):float(v*dt)for k,v in zip(codes,counts)},
            multiple_loaded_patches_duration_s=float(np.sum(loaded.sum(axis=1)>1)*dt),
            instantaneous_dominant_patch_changes=int(np.sum(dominant[1:]!=dominant[:-1])),
            stable_family_runs=stable_runs(t[selected],family),
            dominant_unique80_duration_s=float(np.sum(active&(shares.max(axis=1)>=.8))*dt))
    valid=masks['estimated'][0]&masks['truth'][0];same=masks['estimated'][1]==masks['truth'][1]
    result['comparison']=dict(common_loaded_samples=int(valid.sum()),common_loaded_duration_s=float(valid.sum()*dt),
        loaded_set_disagreement_samples=int(np.sum(valid&~same)),loaded_set_disagreement_duration_s=float(np.sum(valid&~same)*dt),
        loaded_set_agreement_fraction=float(np.mean(same[valid])) if valid.any() else None,
        interpretation='Truth is evaluation-only. Loaded-set mismatch is conditional on both sources having enough total load; denominator is explicit.')
    if 'truth_patch_contact_counts'in raw:
        counts=np.asarray(raw['truth_patch_contact_counts'])[selected]
        result['truth']['more_than_one_resolved_point_duration_s']=float(np.sum(counts.sum(axis=1)>1)*dt)
        result['truth']['resolved_point_definition']='Saved count of individual simulated contact records with normal force>.05N; multiple points can belong to one patch.'
    result['definitions']=dict(minimum_total_load_N=minimum,loaded_patch_share=.05,unique_family_share=.8,stable_dwell_span_s=.015,
        dwell='Last minus first observation >=15ms, not samplecount×dt. Instantaneous changes can chatter and are not automatically physical handoff counts.',
        patch_bit_order=PATCHES,truth='Current simulated endpoint forces; not averaged over the preceding5ms and not hardware measurements.')
    return result


def factorial(values):
    """One metric/phase: yNR-yN-yR+yB is the unscaled interaction contrast."""
    if set(values)!=set(VARIANTS):raise ValueError('Exactly B,R,N,NR are required')
    if any(v is None or not np.isfinite(v)for v in values.values()):return dict(available=False,values=values)
    b,r,n,nr=(float(values[k])for k in VARIANTS)
    return dict(available=True,values=dict(zip(VARIANTS,[b,r,n,nr])),
        R_minus_B=r-b,N_minus_B=n-b,NR_minus_B=nr-b,NR_minus_N=nr-n,NR_minus_R=nr-r,
        reference_main_effect=.5*((r-b)+(nr-n)),normal_main_effect=.5*((n-b)+(nr-r)),
        interaction_difference_of_differences=nr-n-r+b,
        convention='Absolute outcome differences, candidate minus comparator. Negative is favorable only for metrics explicitly defined as lower-is-better. No statistical uncertainty is inferred.')


def arithmetic_checks():
    # New pure arithmetic fixtures; no saved campaign arrays are read.
    t=np.arange(5)*.005;mask=held_selection(t,[0,.02]);assert mask.tolist()==[True,True,True,True,False]
    fake=dict(time_s=t,exact_qvel=np.zeros((5,9)),exact_native_joint_reference=np.full((5,3),100.),exact_guarded_joint_reference=np.ones((5,3)))
    stats=joint_tracking(fake,True,{'full':None});w=stats['windows']['full']
    assert w['held_samples']==4 and abs(w['actual_minus_native_ref']['rms_norm']-100*np.sqrt(3))<1e-12
    assert abs(w['actual_minus_guarded_ref']['rms_norm']-np.sqrt(3))<1e-12
    fake['exact_qvel'][-1,6:9]=1e6;assert joint_tracking(fake,True,{'full':None})['max_abs_actual_joint_rate_rad_s']==0
    f=factorial({'B':10.,'R':8.,'N':7.,'NR':4.});assert f['interaction_difference_of_differences']==-1. and f['reference_main_effect']==-2.5
    assert not factorial({'B':None,'R':8.,'N':7.,'NR':4.})['available']
    q=np.radians(np.array([179.,-179.]));assert abs(np.degrees(wrapped(np.diff(q)))[0]-2.)<1e-10
    assert not stable_runs(np.arange(3)*.005,['shell']*3)
    assert len(stable_runs(np.arange(4)*.005,['shell']*4))==1
    return dict(passed=True,scope='Five grouped new arithmetic contracts: held-terminal exclusion, separate native/guarded joint-error targets, factorial signs/missing values, wrapped phase steps, and15ms dwell span. Not a full repository test suite.')
