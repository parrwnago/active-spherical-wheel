"""I49 predeclared saved-array capacity comparison; no physics/controller calls."""
from __future__ import annotations
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import types

import numpy as np

FAMILY = Path(__file__).resolve().parent
ROOT = FAMILY.parents[1]
DT = .005
OLD_BOUNDS = np.array([1.5, .6, .6, .3, .3])


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for b in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


def safe(value):
    if isinstance(value, dict):return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):return [safe(v) for v in value]
    if isinstance(value, np.ndarray):return safe(value.tolist())
    if isinstance(value, np.generic):return safe(value.item())
    if isinstance(value, Path):return str(value)
    if isinstance(value, float) and not np.isfinite(value):return None
    return value


def save(path, value):path.write_text(json.dumps(safe(value), indent=2, allow_nan=False) + '\n')
def npz(path):
    with np.load(path) as z:return {k: z[k].copy() for k in z.files}


def load_helper(path):
    result = types.ModuleType('_I49_fixed_I41_metrics');result.__file__ = str(path)
    exec(compile(path.read_text(), str(path), 'exec'), result.__dict__)
    return result


def freeze(output, protocol, mode):
    output.mkdir(parents=True, exist_ok=False)
    sources = output / 'source';sources.mkdir()
    records = []
    for path in [Path(__file__), FAMILY / 'protocol.json', ROOT / protocol['inputs']['event_helper'], ROOT/'src/spherical_wheel/mobility.py']:
        target = sources / path.name;shutil.copyfile(path, target)
        records.append(dict(origin=str(path.relative_to(ROOT)), snapshot=str(target.relative_to(output)), sha256=sha(path)))
    deps = FAMILY / 'analysis/dependencies'
    for key in ('event_helper', 'event_turns'):
        origin = ROOT / protocol['inputs'][key];target = deps / origin.name
        if target.exists():assert sha(target) == protocol['input_sha256'][key]
        else:shutil.copyfile(origin, target)
    save(output / 'declaration_before_execution.json', dict(captured_utc=datetime.now(timezone.utc).isoformat(), mode=mode, sources=records,
        scope='Saved-array analysis only; fixed known reference and immutable corrected I41 helper; no simulation/control.'))
    return load_helper(deps / Path(protocol['inputs']['event_helper']).name)


def body_up(quat):
    q = np.asarray(quat, float);q = q / np.linalg.norm(q, axis=1)[:, None]
    w,x,y,z = q.T
    return np.column_stack([2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)])


def attitude(data):
    up = body_up(data['exact_qpos'][:, 3:7])
    ref = body_up(data['exact_reference_qpos'][:, 3:7])
    absolute = np.arccos(np.clip(up[:, 2], -1, 1))
    relative = np.arccos(np.clip(np.einsum('ij,ij->i', up, ref), -1, 1))
    return absolute, relative


def tracking_metrics(data, moving_window):
    t=data['time_s'];error=np.linalg.norm(data['exact_qpos'][:,:2]-data['exact_reference_xy'],axis=1)
    moving=(t>=moving_window[0])&(t<=moving_window[1])
    return dict(moving_rms_m=float(np.sqrt(np.mean(error[moving]**2)))if moving.any()else None,
        moving_p95_m=float(np.quantile(error[moving],.95))if moving.any()else None,moving_samples=int(moving.sum()),
        full_peak_m=float(error.max()),final_m=float(error[-1]),full_rms_m=float(np.sqrt(np.mean(error*error))),
        moving_window_complete=bool(t[-1]>=moving_window[1]-1e-9))


def settling(times, qvel, relative, completed):
    tail = times >= times[-1]-1.
    angular = np.linalg.norm(qvel[tail, 3:6], axis=1)
    linear = np.linalg.norm(qvel[tail, :2], axis=1)
    lean = np.degrees(relative[tail])
    checks = dict(completed=bool(completed), relative_lean_below1deg=bool(np.all(lean < 1.)),
                  planar_speed_below030=bool(np.all(linear < .03)), angular_rate_below080=bool(np.all(angular < .08)))
    return dict(passed=all(checks.values()), checks=checks, tail_s=[float(times[tail][0]), float(times[-1])],
                max_relative_lean_deg=float(lean.max()), max_planar_speed_m_s=float(linear.max()), max_angular_rate_rad_s=float(angular.max()))


def held_threshold(values, threshold):
    values = np.abs(np.asarray(values)[:-1])
    return dict(threshold_m_s=threshold, held_samples=len(values),
        at_or_above_duration_s=float(np.sum(values >= threshold-1e-12)*DT),
        strictly_outside_duration_s=float(np.sum(values > threshold+1e-12)*DT),
        at_or_above_fraction=float(np.mean(values >= threshold-1e-12)) if len(values) else None,
        strictly_outside_fraction=float(np.mean(values > threshold+1e-12)) if len(values) else None,
        comparison_tolerance_m_s=1e-12)


def belt_metrics(data, cap):
    result = {}
    for k,name in ((3,'belt_A'), (4,'belt_B')):
        requested, applied = data['exact_native_requested'][:, k], data['exact_applied'][:, k]
        request_max=float(abs(requested[:-1]).max())if len(requested)>1 else None
        applied_max=float(abs(applied[:-1]).max())if len(applied)>1 else None
        result[name] = dict(max_requested_abs_m_s=request_max, max_applied_abs_m_s=applied_max,
            old030=dict(requested=held_threshold(requested,.3), applied=held_threshold(applied,.3)),
            own_cap=dict(requested=held_threshold(requested,cap), applied=held_threshold(applied,cap)),
            max_requested_over_old030=None if request_max is None else request_max/.3, max_requested_over_own_cap=None if request_max is None else request_max/cap)
    return result


def command_metrics(data, bounds):
    result={}
    for name,key in (('requested','exact_native_requested'),('applied','exact_applied')):
        held=data[key][:-1];steps=np.diff(held,axis=0)
        result[name]=dict(max_common030_normalized=float(np.max(abs(held)/OLD_BOUNDS))if len(held)else None,
            max_own_cap_normalized=float(np.max(abs(held)/bounds))if len(held)else None,
            max_step_common030_normalized=float(np.max(abs(steps)/OLD_BOUNDS)) if len(steps) else None,
            max_step_own_cap_normalized=float(np.max(abs(steps)/bounds)) if len(steps) else None)
    result['joint_torque_outside_held_duration_s']=(np.sum(abs(data['exact_native_requested'][:-1,:3])>bounds[:3]+1e-12,axis=0)*DT).tolist()
    # EXACT original summarize() convention, separately from tolerance-based diagnostics above.
    result['original_any_request_strict_saturated_fraction']=float(np.mean(np.any(abs(data['exact_native_requested'][:-1])>bounds,axis=1)))if len(data['time_s'])>1 else 0.
    return result


def geometry(data, program, helper, moving_window):
    active=(program['time_s']>=moving_window[0]-1e-9)&(program['time_s']<=moving_window[1]+1e-9)
    rt,xy=program['time_s'][active],program['exact_reference_xy'][active]
    rows=[helper.local_projection(rt,xy,p,float(t),2.) for t,p in zip(data['time_s'],data['exact_qpos'][:,:2])]
    return dict(time_s=data['time_s'],local_path_distance_m=np.array([r['distance_m']for r in rows]),
        closest_reference_time_s=np.array([r['closest_time_s']for r in rows]),
        artificial_projection_window_boundary=np.array([r['artificial_time_window_boundary']for r in rows]),
        closest_at_route_endpoint=np.array([r['at_route_endpoint']for r in rows]))


def departure(times, geo, helper, window):
    first=helper.first_sustained(times,geo['local_path_distance_m'],.02,.1,1.)
    if first is None:where='not_observed'
    elif first['start_s']<window[0]-1e-9:where='earlier_than_context_window'
    elif first['start_s']<=window[1]+1e-9:where='inside_context_window'
    else:where='later_than_context_window'
    i=int(np.argmin(abs(times-window[0])))
    available=bool(abs(times[i]-window[0])<1e-8)
    return dict(first_sustained_20mm=first,context_window_s=window,onset_location=where,
        context_left_censored=bool(available and geo['local_path_distance_m'][i]>.02),
        context_start_available=available,context_end_available=bool(times[-1]>=window[1]-1e-9),
        projection_boundary_samples=int(geo['artificial_projection_window_boundary'].sum()),
        interpretation='Global onset retained even outside6.4–7.4; local distance is time-branch-restricted, not a globally unique cross-track distance.')


def turns(data, geo, events, helper):
    t=data['time_s'];xy=data['exact_qpos'][:,:2];v=data['exact_qvel'][:,:2];rows=[]
    for event in events:
        at=event['time_s'];axis=event['axis_index'];lo,hi=event['window_s']
        selected=(t>=lo-1e-9)&(t<=hi+1e-9);after=selected&(t>=at)
        signed=event['incoming_sign']*(xy[:,axis]-event['reference_component_extremum_m'])
        candidates=helper.actual_reversal_candidates(t,v[:,axis],event['incoming_sign'],event['outgoing_sign'],lo,hi,.1)
        first=candidates[0]if candidates else None
        rows.append(dict(event_id=event['event_id'],reference_time_s=at,axis=event['axis'],window_s=[lo,hi],
            window_complete=bool(t[0]<=lo+1e-9 and t[-1]>=hi-1e-9),
            geometric_component_overshoot_m=max(0.,float(signed[selected].max()))if selected.any()else None,
            post_event_geometric_overshoot_m=max(0.,float(signed[after].max()))if after.any()else None,
            actual_reversal_lag_s=float(first['interpolated_zero_time_s']-at)if first else None,
            actual_reversal_missing_in_window=first is None,actual_reversal_multiple_in_window=len(candidates)>1,
            actual_reversal_candidates=candidates,
            projection_boundary_samples=int(geo['artificial_projection_window_boundary'][selected].sum())))
    return rows


def slip_contact(rollout, weight):
    slip=rollout['slip_speed_m_s'][:-1].astype(float);load=rollout['normal_force_N'][:-1].astype(float)/weight
    body=rollout['fork_load_N'][:-1].astype(float)
    return dict(source='Standard saved rollout simulator-contact diagnostics, encoded float32; no hardware measurement claim.',
        held_samples=len(slip),slip_max_m_s=float(slip.max())if len(slip)else None,slip_rms_m_s=float(np.sqrt(np.mean(slip*slip)))if len(slip)else None,
        slip_distance_m=float(slip.sum()*DT),sliding_held_duration_s=float(np.sum(slip>.02)*DT),
        wheel_load_fraction_min=float(load.min())if len(load)else None,wheel_load_fraction_max=float(load.max())if len(load)else None,wheel_load_fraction_std=float(load.std())if len(load)else None,
        unloaded_held_duration_s=float(np.sum(load<=.05)*DT),body_load_max_N=float(body.max())if len(body)else None,body_load_positive_held_duration_s=float(np.sum(body>0)*DT))


def compare(candidate, baseline, limits):
    ct,bt=candidate['timed_tracking'],baseline['timed_tracking'];checks={}
    checks.update(completed_no_fall=bool(candidate['completed']and candidate['failure']is None),
        moving_window_complete=bool(ct['moving_window_complete']),
        moving_RMS_10pct=ct['moving_rms_m']is not None and bt['moving_rms_m']is not None and ct['moving_rms_m']<=bt['moving_rms_m']*(1-limits['moving_RMS_relative_reduction_min'])+1e-12,
        peak_error_no_regression=ct['full_peak_m']<=bt['full_peak_m']+limits['peak_error_increase_max_m']+1e-12,
        final_error_no_regression=ct['final_m']<=bt['final_m']+limits['final_error_increase_max_m']+1e-12,
        peak_relative_lean_no_regression=candidate['attitude']['peak_reference_error_deg']<=baseline['attitude']['peak_reference_error_deg']+limits['peak_reference_lean_increase_max_deg']+1e-10,
        original_settled=candidate['original_settling']['passed'],original_route_gates=candidate['original_route_gates']['passed'])
    changes=[]
    for c,b in zip(candidate['turns'],baseline['turns']):
        valid=bool(c['window_complete']and b['window_complete']and c['actual_reversal_lag_s']is not None and b['actual_reversal_lag_s']is not None)
        overshoot=None if c['geometric_component_overshoot_m']is None or b['geometric_component_overshoot_m']is None else c['geometric_component_overshoot_m']-b['geometric_component_overshoot_m']
        lag=None if not valid else c['actual_reversal_lag_s']-b['actual_reversal_lag_s']
        changes.append(dict(event_id=c['event_id'],coverage=valid,overshoot_difference_m=overshoot,reversal_lag_difference_s=lag,
            overshoot_pass=overshoot is not None and overshoot<=limits['every_turn_overshoot_increase_max_m']+1e-12,
            lag_pass=lag is not None and lag<=limits['every_turn_reversal_lag_increase_max_s']+1e-10))
    checks.update(all_six_turn_coverage=len(changes)==6 and all(r['coverage']for r in changes),
        every_turn_overshoot=all(r['overshoot_pass']for r in changes),every_turn_lag=all(r['lag_pass']for r in changes))
    return dict(passes_all_conditions=all(checks.values()),checks=checks,turn_changes=changes,
        moving_RMS_relative_reduction=None if ct['moving_rms_m']is None or bt['moving_rms_m']is None or bt['moving_rms_m']==0 else(bt['moving_rms_m']-ct['moving_rms_m'])/bt['moving_rms_m'],
        interpretation='One-route descriptive screen only; no default adoption, general robustness or real motor-capacity recommendation.')


def checks(helper):
    helper.arithmetic_checks()
    x=np.array([.1,.3,.9]);m=held_threshold(x,.3)
    assert m['at_or_above_duration_s']==DT and m['strictly_outside_duration_s']==0
    times=np.arange(201)*DT;v=np.zeros((201,9));r=np.zeros(201)
    assert settling(times,v,r,True)['passed'];assert not settling(times,v,r,False)['passed']
    v[-1,3]=.08;assert not settling(times,v,r,True)['passed']
    t=np.arange(21)*.005;geo=dict(local_path_distance_m=np.ones(21)*.021,artificial_projection_window_boundary=np.zeros(21,bool))
    # Exact100ms span needs21 observations at5ms; a short trace must not be promoted.
    assert helper.first_sustained(t[:-1],geo['local_path_distance_m'][:-1],.02,.1,0)is None
    assert helper.first_sustained(t,geo['local_path_distance_m'],.02,.1,0)is not None
    early=dict(time_s=np.array([0.,.005]),exact_qpos=np.zeros((2,10)),exact_reference_xy=np.zeros((2,2)))
    unavailable=tracking_metrics(early,[1.,41.]);assert unavailable['moving_rms_m']is None and not unavailable['moving_window_complete']
    early.update(exact_native_requested=np.zeros((2,5)),exact_applied=np.zeros((2,5)))
    assert belt_metrics(early,.6)['belt_A']['max_requested_abs_m_s']==0
    terminal=dict(time_s=np.array([0.]),exact_native_requested=np.ones((1,5))*100,exact_applied=np.zeros((1,5)))
    assert belt_metrics(terminal,.9)['belt_A']['max_requested_abs_m_s']is None
    assert command_metrics(terminal,OLD_BOUNDS)['requested']['max_common030_normalized']is None
    return dict(passed=True,scope='Corrected I41 analytic checks plus terminal-held duration, strict threshold, original settling boundary, incomplete outcome and100ms span checks; no physics.')


def run(output,protocol,helper):
    for key,path in protocol['inputs'].items():assert sha(ROOT/path)==protocol['input_sha256'][key]
    for path,expected in protocol['source_sha256'].items():assert sha(ROOT/path)==expected
    folders={j['case_id']:FAMILY/'campaign/runs'/j['case_id']for j in protocol['cases']}
    inputs={}
    for folder in folders.values():
        if (folder/'execution_failure.json').exists():raise RuntimeError('Preserve setup/execution failure before scientific analysis: '+str(folder))
        manifest=json.loads((folder/'artifact_manifest.json').read_text())
        for row in manifest['files']:
            path=folder/row['path'];assert sha(path)==row['sha256']
        for name in ('precision.npz','rollout.npz','summary.json','provenance.json'):
            path=folder/name;inputs[str(path.relative_to(ROOT))]=sha(path)
    save(output/'inputs_before_reduction.json',inputs)
    data={case:npz(folder/'precision.npz')for case,folder in folders.items()}
    program=npz(ROOT/protocol['inputs']['historical_precision'])
    baseline_id=protocol['cases'][0]['case_id'];base=data[baseline_id]
    parity={k:base[k].dtype==program[k].dtype and base[k].shape==program[k].shape and base[k].tobytes()==program[k].tobytes()for k in program}
    save(output/'baseline_parity_before_metrics.json',parity)
    if not all(parity.values()):raise RuntimeError('Baseline failed historical17-field parity; no scientific interpretation')
    metadata={case:json.loads((folder/'summary.json').read_text())['belt_capacity_sweep']for case,folder in folders.items()}
    design0=metadata[baseline_id];fixed_design_checks={};initial_checks={}
    for case,meta in metadata.items():
        fixed_design_checks[case]={key:meta[key]==design0[key]for key in('fixed_design','ordinary_design_sha256','gain_cache_sha256','design_belt_cap_m_s')}
        fixed_design_checks[case]['design_bounds_remain030']=meta['fixed_design']['design_bounds']==OLD_BOUNDS.tolist()
        fixed_design_checks[case]['gain_cache_matches_registration']=meta['gain_cache_sha256']==protocol['execution']['gain_cache_sha256']
        fixed_design_checks[case]['configuration_unchanged']=meta['configuration_unchanged']is True
        initial_checks[case]={key:data[case][key][0].tobytes()==base[key][0].tobytes()for key in('exact_qpos','exact_qvel','exact_observer_wrench','exact_estimated_normal','exact_nominal_joints_before','exact_previous_applied','exact_feedback_gain')}
    assert all(all(v.values())for v in fixed_design_checks.values()),fixed_design_checks
    assert all(all(v.values())for v in initial_checks.values()),initial_checks
    events=json.loads((FAMILY/'analysis/dependencies/turn_events.json').read_text())['rows'];assert len(events)==6
    target_geometry=npz(ROOT/protocol['inputs']['event_geometry'])
    cases={};geometry_cache={}
    from spherical_wheel.mobility import TARGETS
    for job in protocol['cases']:
        case=job['case_id'];raw=data[case];folder=folders[case];t=raw['time_s'];cap=job['cap_m_s']
        assert len(t)>=1 and np.all(abs(np.diff(t)-DT)<1e-9)
        for k in ('time_s','exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2'):
            assert raw[k].tobytes()==program[k][:len(t)].tobytes(),(case,k)
        summary=json.loads((folder/'summary.json').read_text());provenance=json.loads((folder/'provenance.json').read_text())
        bounds=np.array([1.5,.6,.6,cap,cap]);observed=raw['exact_input_bounds']
        assert np.array_equal(observed,np.broadcast_to(bounds,observed.shape))
        assert np.all(abs(raw['exact_applied'])<=bounds+1e-12)
        rollout=npz(folder/'rollout.npz');assert len(rollout['time_s'])==len(t)
        moving=(t>=protocol['evaluation']['timed_moving_window_s'][0])&(t<=protocol['evaluation']['timed_moving_window_s'][1])
        completed=bool(summary['completed']);absolute,relative=attitude(raw)
        track=tracking_metrics(raw,protocol['evaluation']['timed_moving_window_s'])
        assert provenance['design']==metadata[case]['fixed_design']
        assert provenance['ordinary_design_sha256']==metadata[case]['ordinary_design_sha256']
        assert provenance['gain_cache_sha256']==metadata[case]['gain_cache_sha256']
        settle=settling(t,raw['exact_qvel'],relative,completed)
        commands=command_metrics(raw,bounds)
        closure=float(np.linalg.norm(raw['exact_qpos'][-1,:2]-raw['exact_qpos'][0,:2]))
        # Preserve summarize()'s original0 RMS default if no moving samples; the
        # scientific screen still marks missing/incomplete moving data unavailable.
        gates=dict(completed=completed,rms=(track['moving_rms_m']if moving.any()else 0.)<=TARGETS['rms_m'],
            p95=track['moving_p95_m']is not None and track['moving_p95_m']<=TARGETS['p95_m'],
            max=track['full_peak_m']<=TARGETS['max_m'],final=track['final_m']<=TARGETS['final_m'],
            attitude=float(np.degrees(relative).max())<=TARGETS['attitude_deg'],
            saturation=commands['original_any_request_strict_saturated_fraction']<=TARGETS['saturated_fraction'],
            settled=settle['passed'],closure=closure<=TARGETS['closure_m'])
        assert gates==summary['mars']['gates'],(case,gates,summary['mars']['gates'])
        geo=geometry(raw,program,helper,protocol['evaluation']['timed_moving_window_s']);geometry_cache[case]=geo
        np.savez_compressed(output/(case+'_geometry.npz'),**geo)
        cases[case]=dict(cap_m_s=cap,completed=completed,failure=summary['failure'],completed_duration_s=float(t[-1]),samples=len(t),
            timed_tracking=track,attitude=dict(peak_absolute_deg=float(np.degrees(absolute).max()),peak_reference_error_deg=float(np.degrees(relative).max()),
                rms_reference_error_deg=float(np.degrees(np.sqrt(np.mean(relative*relative)))),final_reference_error_deg=float(np.degrees(relative[-1]))),
            original_settling=settle,original_route_gates=dict(gates=gates,passed=all(gates.values()),targets=TARGETS,closure_m=closure),
            belts=belt_metrics(raw,cap),commands=commands,departure=departure(t,geo,helper,protocol['evaluation']['early_window_s']),
            turns=turns(raw,geo,events,helper),slip_contact=slip_contact(rollout,provenance['weight_N']),fixed_design=metadata[case])
    cache=geometry_cache[baseline_id]
    geometry_gap={k:float(np.max(abs(cache[k].astype(float)-target_geometry[k].astype(float))))for k in cache}
    assert all(x<1e-10 for x in geometry_gap.values()),geometry_gap
    for row,event in zip(cases[baseline_id]['turns'],events):
        assert abs(row['geometric_component_overshoot_m']*1000-event['geometric_component_overshoot_mm'])<1e-8
        assert abs(row['actual_reversal_lag_s']-event['actual_reversal_lag_s'])<1e-10
    comparisons={case:compare(c,cases[baseline_id],protocol['provisional_screen'])for case,c in cases.items()if case!=baseline_id}
    report=dict(iteration='I-49',cases=cases,baseline_id=baseline_id,comparisons=comparisons,
        integrity=dict(baseline17_fields_byte_exact=parity,baseline_geometry_max_gaps=geometry_gap,fixed_reference_program=True,
            fixed_design_checks=fixed_design_checks,initial_state_and_gain_checks=initial_checks),
        conventions=dict(held_dt_s=DT,terminal_excluded=True,moving_window_s=protocol['evaluation']['timed_moving_window_s'],
            old_bound_normalization=OLD_BOUNDS.tolist(),own_cap_normalization='Reported alongside commonold.30; not a substitute for absolute/common metrics.',
            turn_geometry='All6fixedI41 component extrema/windows;100ms new-sign dwell; missing coverage fails.',
            settle='Original completed+last1s referencelean<1deg,planarspeed<.03m/s,angularrate<.08rad/s.'),
        limits=protocol['limitations'])
    save(output/'report.json',report)
    with(output/'case_metrics.csv').open('w',newline='')as stream:
        fields=['case_id','cap_m_s','completed','moving_rms_m','full_peak_m','final_m','peak_absolute_deg','peak_reference_error_deg','settled','original_route_passed','first20mm_time_s']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for case,c in cases.items():
            onset=c['departure']['first_sustained_20mm'];writer.writerow(dict(case_id=case,cap_m_s=c['cap_m_s'],completed=c['completed'],
                **{k:c['timed_tracking'][k]for k in ('moving_rms_m','full_peak_m','final_m')},
                **{k:c['attitude'][k]for k in ('peak_absolute_deg','peak_reference_error_deg')},settled=c['original_settling']['passed'],
                original_route_passed=c['original_route_gates']['passed'],first20mm_time_s=None if onset is None else onset['start_s']))
    with(output/'turn_metrics.csv').open('w',newline='')as stream:
        fields=['case_id','event_id','reference_time_s','axis','window_complete','geometric_component_overshoot_m','actual_reversal_lag_s','actual_reversal_missing_in_window']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for case,c in cases.items():writer.writerows(dict(case_id=case,**{k:r[k]for k in fields if k!='case_id'})for r in c['turns'])
    for path,expected in inputs.items():assert sha(ROOT/path)==expected
    for path,expected in protocol['source_sha256'].items():assert sha(ROOT/path)==expected
    for row in json.loads((output/'declaration_before_execution.json').read_text())['sources']:
        assert sha(ROOT/row['origin'])==row['sha256']
    files=[p for p in output.rglob('*')if p.is_file()and p.name!='artifact_manifest.json']
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),bytes=p.stat().st_size,sha256=sha(p))for p in sorted(files)]))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--checks-only',action='store_true')
    args=parser.parse_args();protocol=json.loads((FAMILY/'protocol.json').read_text());output=args.output.resolve()
    helper=freeze(output,protocol,'pure_checks'if args.checks_only else'registered_saved_data_reduction')
    save(output/'pure_checks.json',checks(helper))
    if not args.checks_only:run(output,protocol,helper)
