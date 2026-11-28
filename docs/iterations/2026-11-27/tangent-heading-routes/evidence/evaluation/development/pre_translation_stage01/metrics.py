"""I76 diverse-route tangent physical metrics, pure NumPy.

Raw planned heading is the primary objective. Exact Euler heading-rate tracking
is distinct from world-z physical angular velocity and from command shaping lag.
No simulator, controller or outcome-dependent tuning is imported.
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


def euler_heading_rate(R, omega_world, denominator_min=1e-8):
    """Derivative of atan2(R[1,0],R[0,0]), not world omega_z."""
    h=R[:,:,0];hdot=np.cross(omega_world,h)
    denominator=h[:,0]**2+h[:,1]**2
    valid=np.isfinite(denominator)&(denominator>denominator_min)&np.isfinite(hdot).all(axis=1)
    rate=np.divide(h[:,0]*hdot[:,1]-h[:,1]*hdot[:,0],denominator,
                   out=np.full(len(R),np.nan),where=valid)
    return rate,valid


def case_config(config,route_id):
    return {**config,'clock':config['clock_by_route'][route_id]}


def shape_derivatives(route_id,u):
    """Independent frozen local geometry and first three u derivatives."""
    u=np.asarray(u,float);z=np.zeros_like(u)
    if route_id=='straight':
        return (np.column_stack((1.2*u,z)),np.column_stack((np.full_like(u,1.2),z)),
                np.column_stack((z,z)),np.column_stack((z,z)))
    if route_id=='arc':
        k=np.pi/2.;a=k*u
        return (np.column_stack((.6*np.sin(a),.6*(1-np.cos(a)))),
                np.column_stack((.6*k*np.cos(a),.6*k*np.sin(a))),
                np.column_stack((-.6*k*k*np.sin(a),.6*k*k*np.cos(a))),
                np.column_stack((-.6*k**3*np.cos(a),-.6*k**3*np.sin(a))))
    if route_id=='eight':
        k=2*np.pi;a=k*u;b=2*k*u
        return (np.column_stack((.5*np.sin(a),.25*np.sin(b))),
                np.column_stack((.5*k*np.cos(a),.5*k*np.cos(b))),
                np.column_stack((-.5*k*k*np.sin(a),-k*k*np.sin(b))),
                np.column_stack((-.5*k**3*np.cos(a),-2*k**3*np.cos(b))))
    raise ValueError('Unknown frozen route '+str(route_id))


def geometric_polyline(route_id,samples=4097):
    local=shape_derivatives(route_id,np.linspace(0.,1.,samples))[0]
    a=np.radians(113.);R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    return local@R.T+np.array([-2.4,10.])


def planned_xy_reference(route,times):
    """Independent frozen XY/V/A from declared geometry and quintic progress."""
    t=np.asarray(times,float);duration=float(route.duration_s)
    r=np.clip((t-float(route.start_s))/duration,0.,1.)
    u=10*r**3-15*r**4+6*r**5
    du=(30*r**2-60*r**3+30*r**4)/duration
    ddu=(60*r-180*r**2+120*r**3)/duration**2
    stationary=(t<=route.start_s)|(t>=route.start_s+duration)
    du[stationary]=0.;ddu[stationary]=0.
    c,d1,d2,_=shape_derivatives(route.route_id,u)
    a=np.radians(113.);R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    return dict(reference_xy=c@R.T+np.array([-2.4,10.]),
        reference_velocity=(d1*du[:,None])@R.T,
        reference_acceleration=(d2*du[:,None]**2+d1*ddu[:,None])@R.T)


def raw_reference(route,times):
    """Independent analytic route tangent/rate/acceleration, including stops."""
    t=np.asarray(times,float);duration=float(route.duration_s)
    r=np.clip((t-float(route.start_s))/duration,0.,1.)
    u=10*r**3-15*r**4+6*r**5
    du=(30*r**2-60*r**3+30*r**4)/duration
    ddu=(60*r-180*r**2+120*r**3)/duration**2
    stationary=(t<=route.start_s)|(t>=route.start_s+duration)
    du[stationary]=0.;ddu[stationary]=0.
    _,d1,d2,d3=shape_derivatives(route.route_id,u)
    denominator=np.sum(d1*d1,axis=1)
    numerator=d1[:,0]*d2[:,1]-d1[:,1]*d2[:,0]
    numerator_prime=d1[:,0]*d3[:,1]-d1[:,1]*d3[:,0]
    denominator_prime=2*np.sum(d1*d2,axis=1)
    psi_u=numerator/denominator
    psi_uu=(numerator_prime*denominator-numerator*denominator_prime)/denominator**2
    a=np.radians(113.);R=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    tangent=d1@R.T
    return dict(raw_heading=np.arctan2(tangent[:,1],tangent[:,0]),raw_rate=psi_u*du,
                raw_acceleration=psi_uu*du**2+psi_u*ddu)


def wrapped(value):return np.arctan2(np.sin(value),np.cos(value))


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
        descriptive_only=False,
        conditional_four_condition_nonregression='If matched N passes, T must also pass.',
        five_condition_descriptive_only=True,
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
    yawerror=wrapped(yaw-packet['raw_heading'])
    command_error=wrapped(yaw-packet['command_heading'])
    euler_rate,rate_valid=euler_heading_rate(R,omega,config['reference_guards']['forward_projection_denominator_min'])
    yawvalid &= rate_valid
    raw_rate_error=euler_rate-packet['raw_rate']
    command_rate_error=euler_rate-packet['command_rate']
    yawdelta=np.arctan2(np.sin(np.diff(yaw)),np.cos(np.diff(yaw)))
    latepairs=late[:-1]&late[1:]&yawvalid[:-1]&yawvalid[1:]&(abs(np.diff(t)-DT)<=1e-9)
    goal=packet['goal_xy'];goalerror=np.linalg.norm(xy-goal,axis=1)
    path=packet['path_xy']
    geometric=np.full(n,np.nan);geometric[moving]=polyline_distance(xy[moving],path)
    signals=dict(time_s=t,timed_xy_error_m=error,geometric_cross_track_m=geometric,goal_distance_m=goalerror,
        body_xy_speed_m_s=speed,world_z_angular_speed_abs_rad_s=abs(omega[:,2]),world_z_angular_speed_rad_s=omega[:,2],
        reference_up_error_deg=lean,absolute_tilt_deg=tilt,body_up_rate_rad_s=tiltrate,
        heading_rad=yaw,unwrapped_heading_rad=np.unwrap(yaw),heading_error_rad=yawerror,
        heading_error_abs_deg=np.degrees(abs(yawerror)),heading_target_rad=packet['raw_heading'],
        raw_heading_rad=packet['raw_heading'],raw_heading_rate_rad_s=packet['raw_rate'],
        raw_heading_acceleration_rad_s2=packet['raw_acceleration'],
        command_heading_rad=packet['command_heading'],command_heading_rate_rad_s=packet['command_rate'],
        command_heading_acceleration_rad_s2=packet['command_acceleration'],
        command_heading_error_rad=command_error,command_heading_error_abs_deg=np.degrees(abs(command_error)),
        euler_heading_rate_rad_s=euler_rate,euler_heading_rate_valid=rate_valid,
        raw_euler_heading_rate_error_rad_s=raw_rate_error,command_euler_heading_rate_error_rad_s=command_rate_error,
        raw_command_heading_lag_rad=wrapped(packet['command_heading']-packet['raw_heading']),
        raw_command_rate_lag_rad_s=packet['command_rate']-packet['raw_rate'])
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
            raw_initial_target_rad=float(packet['raw_heading'][0]),
            raw_initial_heading_error_deg=float(np.degrees(abs(yawerror[0]))),
            initial_alignment_ok=bool(abs(yawerror[0])<=np.radians(config['reference_guards']['initial_raw_heading_error_max_deg'])+1e-12),
            raw_euler_rate_error_rms_rad_s=stats(raw_rate_error[heading_window])['rms'],
            raw_euler_rate_error_peak_rad_s=stats(abs(raw_rate_error[heading_window]))['maximum'],
            command_error_rms_rad=stats(command_error[heading_window])['rms'],
            command_euler_rate_error_rms_rad_s=stats(command_rate_error[heading_window])['rms'],
            raw_command_heading_lag_rad=stats(abs(wrapped(packet['command_heading']-packet['raw_heading']))),
            raw_command_rate_lag_rad_s=stats(abs(packet['command_rate']-packet['raw_rate'])),
            raw_rate_rad_s=stats(abs(packet['raw_rate'])),
            raw_acceleration_rad_s2=stats(abs(packet['raw_acceleration'])),
            command_rate_rad_s=stats(abs(packet['command_rate'])),
            command_acceleration_rad_s2=stats(abs(packet['command_acceleration'])),
            reference_guards_ok=bool(np.isfinite(np.r_[packet['raw_heading'],packet['raw_rate'],packet['raw_acceleration'],packet['command_heading'],packet['command_rate'],packet['command_acceleration']]).all()
                and np.max(abs(packet['raw_rate']))<=config['reference_guards']['heading_rate_limit_rad_s']+1e-12
                and np.max(abs(packet['raw_acceleration']))<=config['reference_guards']['heading_acceleration_limit_rad_s2']+1e-12
                and np.max(abs(packet['command_rate']))<=config['reference_guards']['heading_rate_limit_rad_s']+1e-12
                and np.max(abs(packet['command_acceleration']))<=config['reference_guards']['heading_acceleration_limit_rad_s2']+1e-12
                and np.all(abs(wrapped(packet['command_heading']-packet['raw_heading']))<=1e-12)
                and np.all(abs(packet['command_rate']-packet['raw_rate'])<=1e-12)
                and np.all(abs(packet['command_acceleration']-packet['raw_acceleration'])<=1e-12)),
            all_samples_valid=bool(yawvalid.all()),definition='Raw geometric tangent pointing/Euler-rate error primary; shaped-command error and physical world-z rotation remain separate.'),
        activity=dict(seconds=exposure_s,passed=exposure_s+1e-9>=exp['minimum_held_seconds_per_T_phase'],
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
 ('full_planar_speed_peak',('motion','full_planar_speed_peak_m_s'),1.10,.010),
 ('full_50ms_3D_acceleration_peak',('motion','full_50ms_acceleration_peak_m_s2'),1.10,.100))


def compare(candidate,native):
    checks=[]
    for name,path,ratio,add in NONREG:
        c,n=candidate[path[0]][path[1]],native[path[0]][path[1]]
        bound=None if n is None else ratio*n+add
        passed=c is not None and bound is not None and np.isfinite(c) and np.isfinite(bound) and c<=bound+1e-12
        checks.append(dict(name=name,actual=c,limit=bound,passed=bool(passed)))
    extras=(('raw_euler_rate_error_peak',('heading','raw_euler_rate_error_peak_rad_s'),1.10,.050),
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
    for statistic,name,ratio,add in [('rms','moving_geometric_cross_track_RMS',1.10,.001),('maximum','moving_geometric_cross_track_peak',1.,.010)]:
        c,n=candidate['tracking']['geometric_moving_m'][statistic],native['tracking']['geometric_moving_m'][statistic]
        limit=None if n is None else ratio*n+add
        checks.append(dict(name=name,actual=c,limit=limit,passed=bool(c is not None and limit is not None and np.isfinite(c) and np.isfinite(limit) and c<=limit+1e-12)))
    checks.append(dict(name='preserve_native_four_condition_settling',native_passed=native['settling']['passed'],actual=candidate['settling']['passed'],
        passed=bool(not native['settling']['passed'] or candidate['settling']['passed'])))
    return checks


def decide(groups,setup_valid,config):
    setup_valid=bool(setup_valid and len(groups)==9)
    checks=[]
    for phase,arms in groups.items():
        n,c=arms['N'],arms['T']
        hard=all(c[k] and n[k] for k in ('completed','finite_state','finite_held_commands','applied_caps_ok',
            'applied_matches_clipped_request','actual_prior_history_exact','held_mask_ok')) and c['heading']['all_samples_valid'] and n['heading']['all_samples_valid']
        checks.append(dict(name=phase+'_physical_completion_bounds',passed=hard))
        benefit=config['heading_benefit']
        for key,name,ratio,floor in [
            ('error_rms_rad','heading_error_RMS',benefit['raw_heading_error_rms_ratio'],np.radians(benefit['raw_heading_absolute_floor_deg'])),
            ('raw_euler_rate_error_rms_rad_s','raw_euler_heading_rate_error_RMS',benefit['raw_euler_yaw_rate_error_rms_ratio'],benefit['raw_euler_rate_error_absolute_floor_rad_s'])]:
            actual,baseline=c['heading'][key],n['heading'][key]
            relative_limit=None if baseline is None else ratio*baseline
            limit=None if relative_limit is None else max(relative_limit,floor)
            passed=actual is not None and limit is not None and np.isfinite(actual) and np.isfinite(limit) and actual<=limit+1e-12
            improved=bool(passed and baseline>0. and actual<baseline and actual<=relative_limit+1e-12)
            checks.append(dict(name=phase+'_'+name,actual=actual,baseline=baseline,limit=limit,relative_limit=relative_limit,absolute_floor=floor,
                achieved_relative_benefit=improved,pass_basis='relative_benefit' if improved else 'acceptable_absolute_error' if passed else 'failed',passed=bool(passed)))
        checks += [{**r,'name':phase+'_'+r['name']} for r in compare(c,n)]
    physical_pass=all(x['passed'] for x in checks)
    exposure=all(a['T']['activity']['passed'] for a in groups.values())
    status=('INVALID_SETUP' if not setup_valid else 'FAIL_TANGENT_SCREEN' if not physical_pass else
            'INCONCLUSIVE' if not exposure else 'PASS_TANGENT_SCREEN')
    return dict(status=status,physical_control_checks=checks,physical_control_passed=physical_pass,
        all_nine_T_helper_activity=exposure,
        no_automatic_adoption=True,all_nine_route_phase_pairs_required=True)
