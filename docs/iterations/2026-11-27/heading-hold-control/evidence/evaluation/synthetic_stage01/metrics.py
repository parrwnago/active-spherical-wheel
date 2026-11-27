"""I74 heading-hold physical metrics. Pure NumPy; no simulator imports.

Utility formulas for rotation, windows, path distance and scoped statistics are
retained from I73. New heading benefits use physical world-z angular velocity,
while Euler heading travel remains a separately named quantity.
"""
import numpy as np

DT=.005


def stats(values):
    a=np.asarray(values,float)
    finite=np.isfinite(a)
    if not len(a) or not finite.all():
        return dict(count=len(a),finite_count=int(finite.sum()),complete=False,rms=None,mean=None,p95=None,maximum=None)
    return dict(count=len(a),finite_count=len(a),complete=True,rms=float(np.sqrt(np.mean(a*a))),
                mean=float(np.mean(a)),p95=float(np.percentile(a,95)),maximum=float(np.max(a)))


def rotation(q):
    q=np.asarray(q,float);norm=np.linalg.norm(q,axis=1)
    q=np.divide(q,norm[:,None],out=np.full_like(q,np.nan),where=norm[:,None]>1e-12)
    w,x,y,z=q.T
    return np.stack([1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),
        1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)],axis=1).reshape(-1,3,3)


def window(t,limits,terminal=True):
    lo,hi=limits
    return (t>=lo-1e-9)&((t<=hi+1e-9) if terminal else (t<hi-1e-9))


def exact_equal(a,b):
    a,b=np.asarray(a),np.asarray(b)
    return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()


def polyline_distance(xy,path,chunk=128):
    """Euclidean distance to the shared raw moving-reference polyline."""
    xy,path=np.asarray(xy,float),np.asarray(path,float)
    if len(path)<2:return np.full(len(xy),np.nan)
    a=path[:-1];d=np.diff(path,axis=0);den=np.sum(d*d,axis=1);keep=den>0.
    if not keep.any():return np.linalg.norm(xy-path[0],axis=1)
    a,d,den=a[keep],d[keep],den[keep];result=np.full(len(xy),np.nan)
    for start in range(0,len(xy),chunk):
        p=xy[start:start+chunk];w=p[:,None,:]-a[None,:,:]
        u=np.clip(np.sum(w*d[None,:,:],axis=2)/den[None,:],0.,1.)
        delta=w-u[:,:,None]*d[None,:,:]
        result[start:start+len(p)]=np.sqrt(np.min(np.sum(delta*delta,axis=2),axis=1))
    return result


def settling(t,signals,config):
    hold=window(t,config['clock']['hold_s']);final=window(t,config['clock']['final_settling_s'])
    limits=config['settling'];keys=('goal_distance_m','body_xy_speed_m_s','world_z_angular_speed_abs_rad_s','reference_up_error_deg')
    bound=(limits['goal_radius_m'],limits['body_xy_speed_m_s'],limits['world_z_angular_speed_rad_s'],limits['raw_reference_up_error_deg'])
    good=np.ones(len(t),bool)
    for k,b in zip(keys,bound):good &= np.isfinite(signals[k])&(signals[k]<=b+1e-12)
    required=int(round((config['clock']['final_settling_s'][1]-config['clock']['final_settling_s'][0])/DT))+1
    complete=bool(final.sum()==required and len(t) and abs(t[-1]-config['clock']['total_s'])<=1e-9)
    first=None;start=None;first_end=None
    for i in np.flatnonzero(hold):
        if not good[i]:start=None;continue
        if start is None or (i>0 and abs(t[i]-t[i-1]-DT)>1e-9):start=i
        if t[i]-t[start]>=limits['sustained_s']-1e-9:
            first=float(t[start]);first_end=i;break
    exited=False;exit_time=None
    if first_end is not None:
        bad=np.flatnonzero(hold&~good&(np.arange(len(t))>first_end))
        if len(bad):exited=True;exit_time=float(t[bad[0]])
    maxima={k:stats(signals[k][final]) for k in keys}
    passed=bool(complete and good[final].all())
    heading_good=good & (signals['heading_error_abs_deg']<=limits['heading_error_deg']+1e-12)
    first_heading=None;heading_start=None
    for i in np.flatnonzero(hold):
        if not heading_good[i]:heading_start=None;continue
        if heading_start is None or (i>0 and abs(t[i]-t[i-1]-DT)>1e-9):heading_start=i
        if t[i]-t[heading_start]>=limits['sustained_s']-1e-9:
            first_heading=float(t[heading_start]);break
    return dict(passed=passed,final_window_complete=complete,final_samples=int(final.sum()),final2s=maxima,
        first_sustained_entry_s=first,first_entry_delay_from_hold_s=None if first is None else first-config['clock']['hold_s'][0],
        exited_after_first_entry=exited,first_later_exit_s=exit_time,
        with_heading_passed=bool(complete and heading_good[final].all()),
        first_sustained_entry_with_heading_s=first_heading,
        final2s_heading_error_deg=stats(signals['heading_error_abs_deg'][final]),
        descriptive_only=True,
        hold={k:stats(signals[k][hold]) for k in keys}),good


def case_metrics(packet,summary,config):
    t=packet['time_s'];n=len(t);clock=config['clock'];full=window(t,[0.,clock['total_s']]);moving=window(t,clock['moving_s'],False)
    late=window(t,clock['late_s']);heading_window=window(t,clock['heading_s']);held=packet['command_held_after_row'].astype(bool)
    expected=int(round(clock['total_s']/DT))+1
    clock_ok=bool(n==expected and n>=2 and abs(t[0])<=1e-9 and abs(t[-1]-clock['total_s'])<=1e-9
        and np.all(abs(np.diff(t)-DT)<=1e-9))
    held_ok=np.array_equal(held,np.arange(n)<n-1)
    R=rotation(packet['qpos'][:,3:7]);Rref=rotation(packet['reference_quat']);up=R[:,:,2];upref=Rref[:,:,2]
    omega=np.einsum('nij,nj->ni',R,packet['qvel'][:,3:6]);xy=packet['qpos'][:,:2]
    error=np.linalg.norm(xy-packet['reference_xy'],axis=1);speed=np.linalg.norm(packet['qvel'][:,:2],axis=1)
    lean=np.degrees(np.arccos(np.clip(np.sum(up*upref,axis=1),-1.,1.)))
    tilt=np.degrees(np.arccos(np.clip(up[:,2],-1.,1.)));tiltrate=np.linalg.norm(packet['qvel'][:,3:5],axis=1)
    yaw=np.arctan2(R[:,1,0],R[:,0,0]);yawvalid=np.isfinite(yaw)&(np.linalg.norm(R[:,:2,0],axis=1)>1e-10)
    yawerror=np.arctan2(np.sin(yaw-packet['heading_target']),np.cos(yaw-packet['heading_target']))
    yawdelta=np.arctan2(np.sin(np.diff(yaw)),np.cos(np.diff(yaw)))
    latepairs=late[:-1]&late[1:]&yawvalid[:-1]&yawvalid[1:]&(abs(np.diff(t)-DT)<=1e-9)
    goal=packet['goal_xy'];goalerror=np.linalg.norm(xy-goal,axis=1)
    path=packet['path_xy']
    geometric=np.full(n,np.nan);geometric[moving]=polyline_distance(xy[moving],path)
    signals=dict(time_s=t,timed_xy_error_m=error,geometric_cross_track_m=geometric,goal_distance_m=goalerror,
        body_xy_speed_m_s=speed,world_z_angular_speed_abs_rad_s=abs(omega[:,2]),world_z_angular_speed_rad_s=omega[:,2],
        reference_up_error_deg=lean,absolute_tilt_deg=tilt,body_up_rate_rad_s=tiltrate,
        heading_rad=yaw,unwrapped_heading_rad=np.unwrap(yaw),heading_error_rad=yawerror,
        heading_error_abs_deg=np.degrees(abs(yawerror)),heading_target_rad=packet['heading_target'])
    settle,good=settling(t,signals,config);signals['simultaneous_settling_good']=good
    request,applied,previous,caps=(packet[k] for k in ('requested','applied','previous_applied','caps'))
    finite_state=bool(np.isfinite(packet['qpos']).all() and np.isfinite(packet['qvel']).all())
    finite_commands=bool(np.isfinite(np.r_[request[held].ravel(),applied[held].ravel(),previous[held].ravel(),caps[held].ravel()]).all())
    box=bool(finite_commands and (caps[held]>0).all() and np.all(abs(applied[held])<=caps[held]+1e-12))
    clip_exact=np.array_equal(applied[held],np.clip(request[held],-caps[held],caps[held]))
    prior_exact=np.array_equal(previous[1:],applied[:-1])
    steps=np.linalg.norm((applied[held]-previous[held])/caps[held],axis=1)
    clipped=abs(request)>caps+1e-12
    helper_norm=np.linalg.norm(packet['heading_command'][:,:3],axis=1)
    exp=config['helper_activity'];exposed=held&(helper_norm>=exp['motor_torque_norm_threshold_Nm'])
    duration=np.r_[np.diff(t),0.];exposure_s=float(np.sum(duration[exposed]))
    excess=np.linalg.norm((request-np.clip(request,-caps,caps))/caps,axis=1)
    signals.update(heading_helper_motor_norm_Nm=helper_norm,heading_helper_active=exposed,
        heading_helper_eligible=packet['heading_eligible'],heading_vertical_lever=packet['vertical_lever'],
        held_command_interval=held,any_requested_clipping=clipped.any(axis=1))
    k=10
    acc=(packet['qvel'][k:,:3]-packet['qvel'][:-k,:3])/(k*DT) if n>k else np.empty((0,3))
    accnorm=np.linalg.norm(acc,axis=1);validacc=(abs(t[k:]-t[:-k]-.05)<=1e-9)&full[k:]&full[:-k]
    invariants=bool(np.all(packet['observer_update_count']==1) and packet['native_feedback_verified'].all()
        and packet['prior_matches_physical'].all() and packet['no_truth_read'].all()
        and packet['helper_matches'].all()
        and np.isfinite(np.r_[packet['reference_qvel'].ravel(),packet['rolling_feedforward'].ravel(),packet['heading_command'].ravel()]).all()
        and np.all(~(helper_norm>1e-12)|packet['heading_eligible']))
    completed=bool(summary.get('completed',False) and summary.get('failure') is None and clock_ok)
    row=dict(completed=completed,failure=summary.get('failure'),frames=n,clock_complete=clock_ok,
        invariant_ok=invariants,held_mask_ok=held_ok,finite_state=finite_state,finite_held_commands=finite_commands,
        actual_prior_history_exact=prior_exact,applied_caps_ok=box,applied_matches_clipped_request=clip_exact,
        tracking=dict(moving_timed_rms_m=stats(error[moving])['rms'],full_timed_peak_m=stats(error[full])['maximum'],
            final_xy_m=float(error[-1]),geometric_moving_m=stats(geometric[moving])),
        attitude=dict(raw_reference_peak_deg=stats(lean[full])['maximum'],raw_reference_rms_deg=stats(lean[full])['rms'],
            absolute_tilt_peak_deg=stats(tilt[full])['maximum'],body_up_rate_rad_s=stats(tiltrate[full])),
        motion=dict(full_planar_speed_peak_m_s=stats(speed[full])['maximum'],
            full_50ms_acceleration_peak_m_s2=stats(accnorm[validacc])['maximum'],
            full_world_z_peak_rad_s=stats(abs(omega[full,2]))['maximum'],
            late_world_z_peak_rad_s=stats(abs(omega[late,2]))['maximum'],
            late_heading_absolute_travel_rad=float(np.sum(abs(yawdelta[latepairs]))) if yawvalid[late].all() else None,
            late_heading_valid=bool(yawvalid[late].all())),settling=settle,
        heading=dict(error_rms_rad=stats(yawerror[heading_window])['rms'],
            error_peak_rad=stats(abs(yawerror[heading_window]))['maximum'],
            late_physical_absolute_rotation_rad=float(np.trapz(abs(omega[late,2]),t[late])) if late.sum()>1 else None,
            full_error_deg=stats(np.degrees(abs(yawerror[full]))),
            final_error_deg=float(np.degrees(abs(yawerror[-1]))),
            target_rad=float(packet['heading_target'][0]),
            target_constant=bool(np.array_equal(packet['heading_target'],np.full(n,packet['heading_target'][0]))),
            all_samples_valid=bool(yawvalid.all()),definition='Wrapped pointing error; physical rotation integral distinct from Euler-heading travel.'),
        activity=dict(seconds=exposure_s,passed=exposure_s+1e-9>=exp['minimum_held_seconds_per_H_phase'],
            threshold_scope='Actual helper torque before fade/clipping; not a full applied-command counterfactual.',
            active_held_intervals=int(exposed.sum()),eligible_held_seconds=float(np.sum(duration[held&packet['heading_eligible']])),
            eligible_held_intervals=int(np.sum(held&packet['heading_eligible'])),
            helper_motor_norm_Nm=stats(helper_norm[held]),
            vertical_lever=stats(abs(packet['vertical_lever'][held])),
            free_shell_held_counts=np.sum(packet['free_shell_mask'][held],axis=0)),
        commands=dict(held_intervals=int(held.sum()),normalized_step=stats(steps),
            clipped_held_intervals=int(np.sum(held&clipped.any(axis=1))),clipped_held_fraction=float(np.mean(clipped[held].any(axis=1))) if held.any() else None,
            clipped_per_channel=np.sum(clipped[held],axis=0),
            clipped_per_channel_fraction=np.mean(clipped[held],axis=0) if held.any() else np.full(5,np.nan),
            normalized_clipping_excess=stats(excess[held]),terminal_unheld_excluded=True,
            final_unheld_request_finite=bool(np.isfinite(request[-1]).all())),
        preparation=dict(weight=stats(np.linalg.norm(packet['sync_weights'][held],axis=1)),
            command_delta=stats(np.linalg.norm(packet['transition_delta'][held]/caps[held],axis=1))))
    return row,signals


NONREG=(('moving_timed_XY_RMS',('tracking','moving_timed_rms_m'),1.10,.001),
 ('full_timed_XY_peak',('tracking','full_timed_peak_m'),1.,.010),('final_XY',('tracking','final_xy_m'),1.,.010),
 ('raw_lean_peak',('attitude','raw_reference_peak_deg'),1.,1.),('raw_lean_RMS',('attitude','raw_reference_rms_deg'),1.10,.1),
 ('absolute_tilt_peak',('attitude','absolute_tilt_peak_deg'),1.,1.),
 ('late_world_yaw_peak',('motion','late_world_z_peak_rad_s'),1.05,.010),
 ('late_heading_absolute_travel',('motion','late_heading_absolute_travel_rad'),1.05,.010),
 ('full_planar_speed_peak',('motion','full_planar_speed_peak_m_s'),1.10,.010),
 ('full_50ms_3D_acceleration_peak',('motion','full_50ms_acceleration_peak_m_s2'),1.10,.100))


def compare(candidate,native):
    checks=[]
    for name,path,ratio,add in NONREG:
        c,n=candidate[path[0]][path[1]],native[path[0]][path[1]]
        bound=None if n is None else ratio*n+add
        passed=c is not None and bound is not None and np.isfinite(c) and np.isfinite(bound) and c<=bound+1e-12
        checks.append(dict(name=name,actual=c,limit=bound,passed=bool(passed)))
    extras=(('full_world_z_peak',('motion','full_world_z_peak_rad_s'),1.05,.010),
        ('held_any_clipping_fraction',('commands','clipped_held_fraction'),1.,.020),
        ('held_clipping_excess_RMS',('commands','normalized_clipping_excess','rms'),1.10,.010),
        ('held_command_step_peak',('commands','normalized_step','maximum'),1.10,.050))
    for name,path,ratio,add in extras:
        c,n=candidate,native
        for key in path:c,n=c[key],n[key]
        limit=None if n is None else ratio*n+add
        passed=c is not None and limit is not None and np.isfinite(c) and np.isfinite(limit) and c<=limit+1e-12
        checks.append(dict(name=name,actual=c,limit=limit,passed=bool(passed)))
    for channel,(c,n) in enumerate(zip(candidate['commands']['clipped_per_channel_fraction'],native['commands']['clipped_per_channel_fraction'])):
        checks.append(dict(name=f'held_clipping_fraction_channel{channel}',actual=float(c),limit=float(n+.020),
                           passed=bool(np.isfinite(c) and np.isfinite(n) and c<=n+.020+1e-12)))
    return checks


def decide(groups,setup_valid):
    checks=[]
    for phase,arms in groups.items():
        n,c=arms['N'],arms['H']
        hard=all(c[k] and n[k] for k in ('completed','finite_state','finite_held_commands','applied_caps_ok',
            'applied_matches_clipped_request','actual_prior_history_exact','held_mask_ok'))
        checks.append(dict(name=phase+'_physical_completion_bounds',passed=hard))
        for key,name in [('error_rms_rad','heading_error_RMS'),('late_physical_absolute_rotation_rad','late_physical_rotation_integral')]:
            actual,baseline=c['heading'][key],n['heading'][key]
            limit=None if baseline is None else .80*baseline
            passed=actual is not None and limit is not None and np.isfinite(actual) and np.isfinite(limit) and actual<=limit+1e-12
            checks.append(dict(name=phase+'_'+name,actual=actual,limit=limit,passed=bool(passed)))
        checks += [{**r,'name':phase+'_'+r['name']} for r in compare(c,n)]
    physical_pass=all(x['passed'] for x in checks)
    exposure=all(a['H']['activity']['passed'] for a in groups.values())
    status=('INVALID_SETUP' if not setup_valid else 'FAIL_HEADING_SCREEN' if not physical_pass else
            'INCONCLUSIVE' if not exposure else 'PASS_HEADING_SCREEN')
    return dict(status=status,physical_control_checks=checks,physical_control_passed=physical_pass,
        all_three_H_helper_activity=exposure,
        no_automatic_adoption=True,all_three_phases_required=True)
