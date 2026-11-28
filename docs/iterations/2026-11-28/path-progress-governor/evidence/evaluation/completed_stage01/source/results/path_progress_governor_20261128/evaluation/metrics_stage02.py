"""I78 canonical stage02 metrics: original grading with arrival-roundoff bookkeeping fix.

No controller, simulator, contact oracle or outcome-dependent parameter search.
Input packets use neutral names; the release evaluator owns schema mapping.
"""
from __future__ import annotations

import numpy as np

DT = .005
ORIGIN = np.array([-2.4, 10.])
ANGLE = np.deg2rad(113.)
DIRECTION = np.array([np.cos(ANGLE), np.sin(ANGLE)])
LENGTH = 1.2
GOAL = ORIGIN + LENGTH*DIRECTION


def stats(values, weights=None):
    a = np.asarray(values, float)
    if not a.size or not np.isfinite(a).all():
        return dict(complete=False, count=int(a.size), finite_count=int(np.isfinite(a).sum()),
                    maximum=None, mean=None, rms=None, p95=None)
    if weights is None:
        mean, square = np.mean(a), np.mean(a*a)
    else:
        w = np.asarray(weights, float)
        if w.shape != a.shape or not np.isfinite(w).all() or np.any(w < 0) or w.sum() <= 0:
            raise ValueError('Finite nonnegative positive-sum metric weights required')
        mean, square = np.sum(w*a)/w.sum(), np.sum(w*a*a)/w.sum()
    return dict(complete=True, count=int(a.size), finite_count=int(a.size),
                maximum=float(a.max()), mean=float(mean), rms=float(np.sqrt(square)),
                p95=float(np.percentile(a, 95)))


def rotation(q):
    q = np.asarray(q, float)
    if q.ndim != 2 or q.shape[1] != 4:
        raise ValueError('Quaternion rows must have shape(n,4)')
    norm = np.linalg.norm(q, axis=1)
    q = np.divide(q, norm[:, None], out=np.full_like(q, np.nan), where=norm[:, None] > 1e-12)
    w, x, y, z = q.T
    return np.stack((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w),
                     2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w),
                     2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)), axis=1).reshape(-1, 3, 3)


def straight_distance(xy):
    """Exact finite-segment distance plus unbounded along-line coordinate."""
    xy = np.asarray(xy, float)
    along = (xy-ORIGIN)@DIRECTION
    closest = ORIGIN + np.clip(along, 0., LENGTH)[:, None]*DIRECTION
    error = np.linalg.norm(xy-closest, axis=1)
    normal = np.array([-DIRECTION[1], DIRECTION[0]])
    signed_cross = (xy-ORIGIN)@normal
    return error, along/LENGTH, signed_cross


def geometric_reference(u, udot, uddot):
    u, udot, uddot = (np.asarray(v, float) for v in (u, udot, uddot))
    return (ORIGIN+LENGTH*u[:, None]*DIRECTION,
            LENGTH*udot[:, None]*DIRECTION,
            LENGTH*uddot[:, None]*DIRECTION)


def native_reference(times):
    t = np.asarray(times, float)
    r = np.clip((t-3.)/32., 0., 1.)
    u = np.clip(10*r**3-15*r**4+6*r**5, 0., 1.)
    du = (30*r**2-60*r**3+30*r**4)/32.
    ddu = (60*r-180*r**2+120*r**3)/(32.**2)
    stopped = (t <= 3.) | (t >= 35.)
    du[stopped] = ddu[stopped] = 0.
    return (u, du, ddu, *geometric_reference(u, du, ddu))


def progress_rms(t, u, error, *, complete):
    """Trapezoidal integral in reference u, never grade an incomplete path."""
    t, u, error = (np.asarray(x, float) for x in (t, u, error))
    if not complete:
        return dict(complete=False, rms=None, covered_progress=float(u[-1]-u[0]) if len(u) else 0.,
                    reason='Full path not completed; no successful partial-path RMS.')
    if len(t) < 2 or not np.isfinite(np.r_[t, u, error]).all():
        return dict(complete=False, rms=None, covered_progress=None, reason='Missing/nonfinite samples')
    du = np.diff(u)
    if np.any(du < -1e-12) or abs((u[-1]-u[0])-1.) > 1e-8:
        raise ValueError('Complete monotonic unit progress required')
    du = np.maximum(du, 0.)
    value = np.sum(.5*(error[1:]**2+error[:-1]**2)*du)
    return dict(complete=True, rms=float(np.sqrt(value/du.sum())), covered_progress=float(du.sum()),
                definition='Trapezoidal error²du over complete unit reference progress; pauses get zero weight.')


def no_progress_intervals(t, coordinate, goal_error, *, arrival_s, window_s, min_delta, goal_radius_m,
                          coordinate_name='geometric_u'):
    """Fixed-length traversal windows, requiring goal-error condition throughout."""
    t, coordinate, goal_error = (np.asarray(x, float) for x in (t, coordinate, goal_error))
    steps = int(round(window_s/DT))
    if abs(steps*DT-window_s) > 1e-10 or steps < 1:
        raise ValueError('No-progress window must be a positive integer control interval')
    found = []
    end = float(t[-1]) if arrival_s is None else float(arrival_s)
    for i in range(len(t)-steps):
        j = i+steps
        after_traversal = (t[j] >= end-1e-9) if arrival_s is not None else (t[j] > end+1e-9)
        if t[i] < 3.-1e-9 or after_traversal:
            continue
        if (abs((t[j]-t[i])-window_s) <= 1e-9 and coordinate[j]-coordinate[i] < min_delta-1e-12
                and np.all(np.isfinite(goal_error[i:j+1]))
                and np.all(goal_error[i:j+1] > goal_radius_m)):
            found.append((i, j))
    # Collapse overlapping flagged windows into descriptive episodes, retaining
    # the first exact window and total extension. They are not independent trials.
    episodes = []
    for first, last in found:
        if episodes and first <= episodes[-1]['last_index']:
            episodes[-1]['last_index'] = last
            episodes[-1]['end_s'] = float(t[last])
        else:
            episodes.append(dict(first_index=first, last_index=last,
                first_window_end_s=float(t[last]), start_s=float(t[first]), end_s=float(t[last]),
                first_window_delta=float(coordinate[last]-coordinate[first])))
    return dict(passed=not episodes, flagged_windows=len(found), episodes=episodes,
                window_s=window_s, min_delta=min_delta, coordinate_name=coordinate_name,
                goal_radius_m=goal_radius_m)


def case_metrics(packet, summary, config, arrival_s):
    """Metric fields use the first arrival+10s episode, never a later easy tail."""
    t = np.asarray(packet['time_s'], float)
    n = len(t)
    grid_ok = bool(n >= 2 and abs(t[0]) <= 1e-9 and np.all(abs(np.diff(t)-DT) <= 1e-9))
    max_end = float(config['max_duration_s'])
    goal_hold_s = float(config['goal_hold_s'])
    declared_arrival = None if arrival_s is None else float(arrival_s)
    if declared_arrival is not None and (not np.isfinite(declared_arrival) or declared_arrival < 3.-1e-9):
        raise ValueError('Finite path arrival no earlier than motion start required')
    path_done = bool(declared_arrival is not None and declared_arrival <= max_end+1e-9
                     and declared_arrival <= float(t[-1])+1e-9)
    desired_end = max_end if declared_arrival is None else declared_arrival+goal_hold_s
    end = min(float(t[-1]), desired_end)
    episode = t <= end+1e-9
    traversal = (t >= 3.-1e-9) & ((t <= declared_arrival+1e-9) if declared_arrival is not None else episode)
    full_hold = bool(path_done and desired_end <= max_end+1e-9
                     and float(t[-1]) >= desired_end-1e-9)
    goal = np.linalg.norm(packet['qpos'][:, :2]-GOAL, axis=1)
    speed = np.linalg.norm(packet['qvel'][:, :2], axis=1)
    geometric, actual_along, signed_cross = straight_distance(packet['qpos'][:, :2])
    own_reference_error = np.linalg.norm(packet['qpos'][:, :2]-packet['reference_xy'], axis=1)
    original_u, _, _, original_xy, _, _ = native_reference(t)
    old_schedule_error = np.linalg.norm(packet['qpos'][:, :2]-original_xy, axis=1)
    R, Rref = rotation(packet['qpos'][:, 3:7]), rotation(packet['reference_quat'])
    up, ref_up = R[:, :, 2], Rref[:, :, 2]
    lean = np.degrees(np.arccos(np.clip(np.sum(up*ref_up, axis=1), -1., 1.)))
    tilt = np.degrees(np.arccos(np.clip(up[:, 2], -1., 1.)))
    omega_world = np.einsum('nij,nj->ni', R, packet['qvel'][:, 3:6])
    omega_z = omega_world[:, 2]
    held = np.asarray(packet['held'], bool)
    expected_held = np.arange(n) < n-1
    command_scope = held & (t < end-1e-9)
    request, applied, previous, caps = (np.asarray(packet[k]) for k in ('requested','applied','previous_applied','caps'))
    finite = bool(np.isfinite(np.r_[packet['qpos'].ravel(),packet['qvel'].ravel(),
                                  request[command_scope].ravel(),applied[command_scope].ravel(),
                                  previous[command_scope].ravel(),caps[command_scope].ravel()]).all())
    cap_ok = bool(finite and np.all(caps[command_scope] > 0)
                  and np.all(abs(applied[command_scope]) <= caps[command_scope]+1e-12))
    clip_identity = bool(np.array_equal(applied[held],np.clip(request[held],-caps[held],caps[held])))
    prior_identity = bool(np.array_equal(previous[1:],applied[:-1]))
    clip_mask = abs(request) > caps+1e-12
    excess = np.linalg.norm((request-np.clip(request,-caps,caps))/caps,axis=1)
    command_step = np.linalg.norm((applied-previous)/caps,axis=1)
    u = np.asarray(packet['u'],float)
    virtual_time = np.asarray(packet['virtual_time_s'],float)
    virtual_clock_ok = bool(virtual_time.shape==t.shape and np.isfinite(virtual_time).all()
        and abs(virtual_time[0]-t[0])<=1e-9 and np.all(np.diff(virtual_time)>=-1e-12)
        and np.all(np.diff(virtual_time)<=np.diff(t)+1e-9)
        and np.all(virtual_time<=t+1e-9))
    progress_ok = bool(np.isfinite(u).all() and np.all((u>=-1e-12)&(u<=1.+1e-12))
                       and np.all(np.diff(u)>=-1e-12))
    arrival_match = False
    if declared_arrival is not None:
        idx = np.flatnonzero(abs(t-declared_arrival)<=1e-9)
        near_terminal=np.flatnonzero(virtual_time>=35.-1e-9)
        arrival_match = bool(len(idx)==1 and len(near_terminal)>0
                             and virtual_time[idx[0]]>=35.-1e-9
                             and declared_arrival-t[near_terminal[0]]<=DT+1e-9
                             and np.all(abs(u[idx[0]:]-1.)<=1e-10)
                             and np.all(abs(packet['reference_xy'][idx[0]:]-GOAL)<=1e-10)
                             and np.all(abs(packet['reference_velocity'][idx[0]:])<=1e-10)
                             and np.all(abs(packet['reference_acceleration'][idx[0]:])<=1e-10))
    path_done &= arrival_match
    full_hold &= path_done
    stall_cfg=config['stall']
    stall = no_progress_intervals(t,packet['virtual_time_s'],goal,arrival_s=declared_arrival,
        window_s=stall_cfg['window_s'],min_delta=stall_cfg['min_delta_virtual_time_s'],
        goal_radius_m=stall_cfg['goal_radius_m'],coordinate_name='virtual_time_s')
    geometric_slow = no_progress_intervals(t,u,goal,arrival_s=declared_arrival,
        window_s=stall_cfg['window_s'],min_delta=stall_cfg['geometric_min_delta_u_diagnostic'],
        goal_radius_m=stall_cfg['goal_radius_m'],coordinate_name='geometric_u')
    geometric_slow['diagnostic_only']=True
    final = np.zeros(n,bool)
    if full_hold:
        final = (t >= desired_end-float(config['settling']['sustained_s'])-1e-9)&(t<=desired_end+1e-9)
    required = int(round(float(config['settling']['sustained_s'])/DT))+1
    final_complete = bool(final.sum()==required)
    settle=config['settling']
    position_good=(goal<=settle['goal_radius_m']+1e-12)&(speed<=settle['body_xy_speed_m_s']+1e-12)
    all_good=position_good&(abs(omega_z)<=settle['world_z_angular_speed_rad_s']+1e-12)&(lean<=settle['raw_reference_up_error_deg']+1e-12)
    final_position_pass=bool(final_complete and position_good[final].all())
    final_four_pass=bool(final_complete and all_good[final].all())
    k=10
    acceleration=(packet['qvel'][k:,:3]-packet['qvel'][:-k,:3])/(k*DT)
    acceleration_valid=episode[k:]&episode[:-k] if n>k else np.empty(0,bool)
    ref_expected=geometric_reference(u,packet['udot'],packet['uddot'])
    ref_residuals={key:float(np.max(abs(packet[key]-value))) for key,value in
        zip(('reference_xy','reference_velocity','reference_acceleration'),ref_expected)}
    ref_ok=bool(all(v<=config['reference_consistency_atol'] for v in ref_residuals.values()))
    invariant_ok=bool(np.all(packet['observer_update_count']==1) and np.all(packet['no_truth_read'])
                      and np.all(packet['prior_matches_physical']) and not np.any(packet['heading_command']))
    observed_completion=bool(summary.get('completed',False) and summary.get('failure') is None)
    final_index=int(np.flatnonzero(episode)[-1])
    finite_reference=bool(np.isfinite(np.r_[u,packet['udot'],packet['uddot'],packet['reference_quat'].ravel(),packet['reference_xy'].ravel(),
                                           packet['reference_velocity'].ravel(),packet['reference_acceleration'].ravel()]).all())
    row=dict(physical_run_completed=observed_completion,physical_failure=summary.get('failure'),
        frames=n,actual_last_time_s=float(t[-1]),primary_episode_end_s=end,declared_reference_arrival_s=declared_arrival,
        requested_horizon_is_path_completion=False,recorded_extra_tail_s=max(0.,float(t[-1])-desired_end),
        grid_ok=grid_ok,held_mask_ok=bool(np.array_equal(held,expected_held)),finite_state_commands=finite,
        cap_ok=cap_ok,clip_identity=clip_identity,prior_input_identity=prior_identity,
        reference_consistency_ok=ref_ok,reference_residuals=ref_residuals,finite_reference=finite_reference,
        progress_valid=progress_ok,virtual_clock_valid=virtual_clock_ok,
        reference_path_completed=path_done,full_ten_second_hold=full_hold,
        arrival_added_time_s=None if declared_arrival is None else declared_arrival-35.,
        traversal_time_ratio=None if declared_arrival is None else (declared_arrival-3.)/32.,
        maximum_virtual_clock_lag_s=float(np.max(t[traversal]-virtual_time[traversal])) if traversal.any() else None,
        native_measurement_boundary_ok=invariant_ok,no_progress=stall,geometric_progress_slow=geometric_slow,
        tracking=dict(geometric_traversal=stats(geometric[traversal]),
            geometric_progress_rms=progress_rms(t[traversal],u[traversal],geometric[traversal],complete=path_done),
            own_reference_xy=stats(own_reference_error[traversal]),
            original_clock_schedule_error=stats(old_schedule_error[episode]),
            original_clock_error_is_primary=False,
            goal_error_at_primary_end_m=float(goal[final_index]),
            physical_goal_position_converged=final_position_pass),
        attitude=dict(own_raw_target_peak_deg=stats(lean[episode])['maximum'],
                      own_raw_target_rms_deg=stats(lean[episode])['rms'],
                      absolute_tilt_peak_deg=stats(tilt[episode])['maximum']),
        motion=dict(planar_speed_peak_m_s=stats(speed[episode])['maximum'],
                    world_z_rate_peak_rad_s=stats(abs(omega_z[episode]))['maximum'],
                    acceleration_50ms_peak_m_s2=stats(np.linalg.norm(acceleration,axis=1)[acceleration_valid])['maximum']),
        settling=dict(final_window_complete=final_complete,final_samples=int(final.sum()),
                      physical_position_passed=final_position_pass,four_condition_passed=final_four_pass,
                      goal=stats(goal[final]),speed=stats(speed[final]),world_z_rate=stats(abs(omega_z[final])),
                      own_reference_up_error=stats(lean[final])),
        commands=dict(held_intervals=int(command_scope.sum()),
            clipped_any_seconds=float(np.sum(clip_mask[command_scope].any(axis=1))*DT),
            clipped_channel_seconds=(np.sum(clip_mask[command_scope],axis=0)*DT).tolist(),
            clipped_any_fraction=float(np.mean(clip_mask[command_scope].any(axis=1))) if command_scope.any() else None,
            clipped_channel_fraction=np.mean(clip_mask[command_scope],axis=0).tolist() if command_scope.any() else [None]*5,
            normalized_clipping_excess=stats(excess[command_scope]),normalized_applied_step=stats(command_step[command_scope]),
            requested_peak_abs=np.max(abs(request[command_scope]),axis=0).tolist() if command_scope.any() else [None]*5,
            final_unheld_request_finite=bool(np.isfinite(request[-1]).all()),
            terminal_unheld_command_excluded=True))
    signals=dict(time_s=t,progress=u,actual_along_coordinate=actual_along,signed_cross_track_m=signed_cross,
        geometric_error_m=geometric,own_reference_xy_error_m=own_reference_error,
        original_clock_schedule_error_m=old_schedule_error,goal_error_m=goal,body_xy_speed_m_s=speed,
        current_raw_target_up_error_deg=lean,absolute_tilt_deg=tilt,world_z_rate_rad_s=omega_z,
        normalized_clipping_excess=excess,normalized_applied_command_step=command_step,
        primary_episode_mask=episode,traversal_mask=traversal,final_settling_mask=final,
        original_progress=original_u)
    return row,signals


def _get(row,path):
    value=row
    for key in path:value=value[key]
    return value


def compare(candidate,native,config,*,lead_phase=False):
    """Root-approved fixed nonregression checks; no data-dependent tolerances."""
    checks=[]
    for name,entry in config['nonregression'].items():
        c,n=_get(candidate,entry['path']),_get(native,entry['path'])
        bound=None if n is None else entry['ratio']*n+entry['add']
        okay=bool(c is not None and bound is not None and np.isfinite(c) and np.isfinite(bound)
                  and c<=bound+1e-12)
        checks.append(dict(name=name,actual=c,native=n,limit=bound,passed=okay))
    for key,extra in [('clipped_channel_fraction',config['channel_clipping_fraction_add']),
                      ('clipped_channel_seconds',config['channel_clipping_seconds_add'])]:
        for i,(c,n) in enumerate(zip(candidate['commands'][key],native['commands'][key])):
            bound=None if n is None else n+extra
            checks.append(dict(name=f'{key}_{i}',actual=c,native=n,limit=bound,
                passed=bool(c is not None and bound is not None and np.isfinite(c)
                            and np.isfinite(bound) and c<=bound+1e-12)))
    checks.append(dict(name='preserve_native_four_condition_settling',
        native=native['settling']['four_condition_passed'],actual=candidate['settling']['four_condition_passed'],
        passed=bool(not native['settling']['four_condition_passed'] or candidate['settling']['four_condition_passed'])))
    if lead_phase:
        c=candidate['tracking']['geometric_traversal']['maximum']
        n=native['tracking']['geometric_traversal']['maximum']
        bound=None if n is None else config['lead_peak_ratio']*n
        checks.append(dict(name='minus_one_geometric_peak_benefit',actual=c,native=n,limit=bound,
            passed=bool(c is not None and bound is not None and np.isfinite(c)
                        and np.isfinite(bound) and c<=bound+1e-12)))
    return checks


def decide(cases,config,*,setup_valid):
    expected={(-1.,p)for p in ('N','G')}|{(.3,p)for p in ('N','G')}|{(.9,p)for p in ('N','G')}
    setup_valid=bool(setup_valid and set(cases)==expected)
    checks=[]
    if set(cases)==expected:
        for phase in (-1.,.3,.9):
            native,candidate=(cases[(phase,p)] for p in ('N','G'))
            physical=('physical_run_completed','grid_ok','held_mask_ok','finite_state_commands','cap_ok',
                'clip_identity','prior_input_identity','finite_reference','reference_path_completed',
                'full_ten_second_hold','progress_valid','virtual_clock_valid','native_measurement_boundary_ok')
            checks.append(dict(name=f'{phase}_physical_completion_and_bounds',
                passed=bool(all(native[k] and candidate[k] for k in physical))))
            checks.append(dict(name=f'{phase}_physical_goal_position_convergence',
                passed=bool(candidate['settling']['physical_position_passed'])))
            checks.append(dict(name=f'{phase}_no_prolonged_clock_stall',passed=bool(candidate['no_progress']['passed'])))
            checks.extend({**c,'name':f'{phase}_'+c['name']} for c in compare(candidate,native,config,lead_phase=phase==-1.))
    passed=bool(setup_valid and checks and all(c['passed'] for c in checks))
    status='INVALID_SETUP' if not setup_valid else 'PASS_PROGRESS_SCREEN' if passed else 'FAIL_PROGRESS_SCREEN'
    return dict(status=status,passed=passed,checks=checks,all_three_phases_required=True,
                automatic_adoption=False,old_clock_timing_is_secondary=True)
