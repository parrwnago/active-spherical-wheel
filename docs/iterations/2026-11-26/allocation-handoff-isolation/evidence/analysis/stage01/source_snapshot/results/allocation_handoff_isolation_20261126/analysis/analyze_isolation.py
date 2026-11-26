"""I51 registered paired native outcome reduction; saved data/current geometry.

No controller/observer/preview/dynamics calls. Exact frozen I49 metric functions
and corrected I41 event definitions are reused without editing their sources.
The fifth phase-schedule diagnostic is deliberately outside this four-arm file.
"""
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch
import argparse
import csv
import hashlib
import json
import shutil
import types
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
FAMILY = HERE.parent
ROOT = FAMILY.parents[1]
I49_METRICS = ROOT/'results/belt_capacity_sweep_20261126/analyze_capacity.py'
I49_SHA256 = 'cbaf7ae18989e1a07e9dfaf4167dd2eaa75fd59d064c4259a7b75f79d05c3a6d'
CASES = ('base030', 'reference030', 'base060', 'handoff060')
LABELS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
BASE_BOUNDS = np.array([1.5, .6, .6, .3, .3])
DT = .005


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def safe(value):
    if isinstance(value, dict): return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)): return [safe(v) for v in value]
    if isinstance(value, np.ndarray): return safe(value.tolist())
    if isinstance(value, np.generic): return safe(value.item())
    if isinstance(value, Path): return str(value)
    if isinstance(value, float) and not np.isfinite(value): return None
    return value


def save(path, value):
    path.write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def load(path):
    with np.load(path, allow_pickle=False) as arrays:
        return {k: arrays[k].copy() for k in arrays.files}


def same(a, b):
    return a.shape == b.shape and a.dtype == b.dtype and a.tobytes() == b.tobytes()


def module_from_source(path, name):
    module = types.ModuleType(name)
    module.__file__ = str(path)
    exec(compile(path.read_text(), str(path), 'exec'), module.__dict__)
    return module


def freeze(output, protocol, checks_only):
    output.mkdir(parents=True, exist_ok=False)
    assert sha(I49_METRICS) == I49_SHA256
    paths = [Path(__file__).resolve(), FAMILY/'protocol.json', I49_METRICS,
        ROOT/protocol['inputs']['event_helper'], ROOT/protocol['inputs']['event_turns'],
        ROOT/'src/spherical_wheel/mobility.py']
    records = []
    for source in paths:
        target = output/'source_snapshot'/source.relative_to(ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        records.append(dict(origin=str(source.relative_to(ROOT)), snapshot=str(target.relative_to(output)), sha256=sha(source)))
    save(output/'declaration_before_execution.json', dict(captured_utc=datetime.now(timezone.utc).isoformat(), sources=records,
        checks_only=checks_only, pairs=protocol['pairs'],
        scope='Four-arm saved-data reduction and optional private CURRENT-pose Jacobians only. No controller/observer/preview/integration/physics execution. No fifth-arm analysis or output tuning.'))
    metrics = module_from_source(I49_METRICS, '_i51_frozen_i49_metrics')
    helper = module_from_source(ROOT/protocol['inputs']['event_helper'], '_i51_corrected_i41_events')
    return metrics, helper


def pure_checks(metrics, helper):
    # These repeat upstream arithmetic contracts; do not add their counts to
    # historical or repository test totals.
    upstream = metrics.checks(helper)
    fake = dict(time_s=np.array([0., DT, 2*DT]), exact_native_requested=np.array([[0.,0.,0.,.4,0.], [0.,0.,0.,.6,0.], [0.,0.,0.,9.,0.]]),
        exact_applied=np.array([[0.,0.,0.,.4,0.], [0.,0.,0.,.6,0.], [0.,0.,0.,.6,0.]]))
    held = metrics.belt_metrics(fake, .6)
    assert held['belt_A']['max_requested_abs_m_s'] == .6
    assert held['belt_A']['old030']['applied']['strictly_outside_duration_s'] == .01
    assert held['belt_A']['own_cap']['applied']['at_or_above_duration_s'] == .005
    commands = metrics.command_metrics(fake, np.array([1.5,.6,.6,.6,.6]))
    assert commands['applied']['max_common030_normalized'] == 2.
    assert commands['applied']['max_own_cap_normalized'] == 1.
    # Total projection may be nonzero while native request is inside its box.
    native = np.array([0.,0.,0.,.4,0.]); final = native.copy(); final[3] = .015
    assert np.all(abs(native) <= np.array([1.5,.6,.6,.6,.6])) and np.any(final-native != 0)
    return dict(passed=True, upstream_contracts=upstream,
        new_scope='Held-terminal/own-vs-common bounds and total projection versus native-box clipping distinction. Upstream repeated checks are not extra repo tests.')


def input_declaration(output, protocol):
    inputs = {}
    for key, relative in protocol['inputs'].items():
        assert sha(ROOT/relative) == protocol['input_sha256'][key]
        inputs[relative] = protocol['input_sha256'][key]
    for relative, expected in protocol['source_sha256'].items():
        assert sha(ROOT/relative) == expected
    for case in CASES:
        folder = FAMILY/'campaign/runs'/case
        if (folder/'execution_failure.json').exists():
            raise RuntimeError('Preserve execution/setup failure before reduction: '+case)
        manifest = json.loads((folder/'artifact_manifest.json').read_text())
        for entry in manifest['files']:
            assert sha(folder/entry['path']) == entry['sha256']
        for name in ('precision.npz', 'rollout.npz', 'summary.json', 'provenance.json', 'candidate_diagnostics.json', 'artifact_manifest.json'):
            source = folder/name
            inputs[str(source.relative_to(ROOT))] = sha(source)
    matches = list((FAMILY/'campaign/_workspaces/reference030').glob('worker_*/mirror/model.xml'))
    assert len(matches) == 1
    model_path = matches[0]
    inputs[str(model_path.relative_to(ROOT))] = sha(model_path)
    for mesh in ET.parse(model_path).getroot().findall('asset/mesh'):
        source = model_path.parent/mesh.attrib['file']
        inputs[str(source.relative_to(ROOT))] = sha(source)
    save(output/'inputs_before_reduction.json', dict(input_sha256=inputs, current_kinematics_model=str(model_path.relative_to(ROOT)),
        current_kinematics_scope='Only mj_kinematics/mj_comPos/mj_jac. Current single-patch point is exact; plural saved points are float32 and explicitly labelled.'))
    return inputs, model_path


def trace_metrics(times, values, mask):
    selected = np.asarray(values)[mask]
    return dict(samples=int(np.sum(mask)), available=bool(len(selected)),
        rms_m=float(np.sqrt(np.mean(selected**2))) if len(selected) else None,
        peak_m=float(np.max(selected)) if len(selected) else None,
        window_complete=None)


def guard_contracts(case, raw, diagnostics):
    t = raw['time_s']; n = len(t); held = slice(0, -1)
    actual = raw['exact_applied']; native = raw['exact_native_applied']; request = raw['exact_native_requested']
    bounds = raw['exact_input_bounds']
    np.testing.assert_array_equal(native, np.clip(request, -bounds, bounds))
    np.testing.assert_array_equal(raw['exact_native_clipping_command_delta'], native-request)
    np.testing.assert_array_equal(raw['exact_handoff_command_delta'], actual-native)
    np.testing.assert_array_equal(raw['exact_clipping_command_delta'], actual-request)
    np.testing.assert_array_equal(raw['exact_previous_applied'][1:], actual[:-1])
    np.testing.assert_array_equal(raw['exact_reference_guard_delta'], raw['exact_guarded_joint_reference']-raw['exact_native_joint_reference'])
    np.testing.assert_array_equal(raw['exact_reference_qvel'][:,6:9], raw['exact_guarded_joint_reference'])
    np.testing.assert_array_equal(raw['exact_feedback_reference_qvel'][:,6:9], raw['exact_guarded_joint_reference'])
    np.testing.assert_array_equal(raw['exact_allocation_drive_reference'][:,:3], raw['exact_guarded_joint_reference'])
    changed = np.any(raw['exact_reference_guard_delta'] != 0, axis=1)
    projected = np.any(actual != native, axis=1)
    reference = dict(modified_held_samples=int(np.sum(changed[:-1])), modified_held_duration_s=float(np.sum(changed[:-1])*DT),
        first_modified_time_s=float(t[np.flatnonzero(changed)[0]]) if changed.any() else None,
        max_native_joint_reference_rad_s=float(np.max(abs(raw['exact_native_joint_reference']))),
        max_guarded_joint_reference_rad_s=float(np.max(abs(raw['exact_guarded_joint_reference']))),
        max_guarded_joint_reference_acceleration_rad_s2=float(np.max(abs(np.diff(raw['exact_guarded_joint_reference'],axis=0))/np.diff(t)[:,None])) if n>1 else None)
    if case == 'reference030':
        assert reference['max_guarded_joint_reference_rad_s'] <= 20.+1e-12
        assert reference['max_guarded_joint_reference_acceleration_rad_s2'] is None or reference['max_guarded_joint_reference_acceleration_rad_s2'] <= 100.+1e-8
        assert all(row['reference'] is not None and row['handoff'] is None for row in diagnostics)
    else:
        assert not changed.any()
        assert all(row['reference'] is None for row in diagnostics)
    handoff = dict(modified_held_samples=int(np.sum(projected[:-1])), modified_held_duration_s=float(np.sum(projected[:-1])*DT),
        first_modified_time_s=float(t[np.flatnonzero(projected)[0]]) if projected.any() else None,
        max_handoff_delta_bychannel=np.max(abs(actual-native),axis=0))
    if case == 'handoff060':
        np.testing.assert_array_equal(actual[:,:3], native[:,:3])
        episode = np.array([row['handoff']['episode_active'] for row in diagnostics], bool)
        triggered = np.array([row['handoff']['triggered'] for row in diagnostics], bool)
        assert not triggered[0].any()
        lower = np.maximum(-bounds[:,3:], raw['exact_previous_applied'][:,3:]-3.*DT)
        upper = np.minimum(bounds[:,3:], raw['exact_previous_applied'][:,3:]+3.*DT)
        assert np.all(actual[:,3:][episode] >= lower[episode]-1e-12)
        assert np.all(actual[:,3:][episode] <= upper[episode]+1e-12)
        np.testing.assert_array_equal(actual[:,3:][~episode], native[:,3:][~episode])
        handoff.update(active_episode_held_samples_bybelt=np.sum(episode[:-1],axis=0),
            triggered_held_samples_bybelt=np.sum(triggered[:-1],axis=0),
            max_abs_active_episode_slew_m_s2=float(np.max((abs(actual[:,3:]-raw['exact_previous_applied'][:,3:])/DT)[episode])) if episode.any() else None,
            active_episode_contract_passed=True)
    else:
        np.testing.assert_array_equal(actual, native)
        assert all(row['handoff'] is None for row in diagnostics)
    handoff['native_box_clipped_held_fraction'] = float(np.mean(np.any(native[:-1] != request[:-1],axis=1))) if n>1 else None
    handoff['total_projection_held_fraction'] = float(np.mean(np.any(actual[:-1] != request[:-1],axis=1))) if n>1 else None
    handoff['interpretation'] = 'Native box clipping and extra handoff projection are separate; common clipping delta is their total raw-to-actual difference.'
    return dict(passed=True, reference=reference, handoff=handoff)


def current_reference_residual(output, raw, log, model_path):
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(model_path)); data = mujoco.MjData(model)
    metadata = []
    for label in LABELS:
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, label+'_smooth')
        metadata.append((geom, int(model.geom_bodyid[geom])))
    native_norm = np.full(len(raw['time_s']), np.nan); guarded_norm = native_norm.copy()
    quality = np.zeros(len(native_norm), np.int8)
    def forbidden(*args, **kwargs): raise AssertionError('Dynamics/integration/contact query forbidden in saved residual reduction')
    with ExitStack() as stack:
        for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_integratePos','mj_contactForce'):
            stack.enter_context(patch.object(mujoco,name,forbidden))
        for i, qpos in enumerate(raw['exact_qpos']):
            loaded = np.flatnonzero(raw['exact_estimated_support_loads_N'][i] > 0)
            if not len(loaded): continue
            data.qpos[:] = qpos; mujoco.mj_kinematics(model,data); mujoco.mj_comPos(model,data)
            rows=[]
            for j in loaded:
                geom, body = metadata[j]
                point = (raw['exact_estimated_contact_point_m'][i] if len(loaded)==1 else log['estimated_contact_points_m'][i,j].astype(float))
                if not np.isfinite(point).all(): break
                jac = np.zeros((3,model.nv)); mujoco.mj_jac(model,data,jac,None,point,body)
                matrix = np.zeros((2,5)); matrix[:,:3] = jac[:2,6:9]
                if j >= 2: matrix[:,j+1] = data.geom_xmat[geom].reshape(3,3)[:2,1]
                rows.extend(matrix)
            else:
                matrix=np.asarray(rows); rhs=np.tile(-raw['exact_reference_velocity_m_s'][i],len(loaded))
                native=np.r_[raw['exact_native_joint_reference'][i],raw['exact_rolling_feedforward'][i,3:]]
                guarded=np.r_[raw['exact_guarded_joint_reference'][i],raw['exact_rolling_feedforward'][i,3:]]
                native_norm[i]=np.linalg.norm(matrix@native-rhs); guarded_norm[i]=np.linalg.norm(matrix@guarded-rhs)
                quality[i]=1 if len(loaded)==1 else 2
    changed=np.any(raw['exact_reference_guard_delta']!=0,axis=1); usable=changed&(quality>0)
    np.savez_compressed(output/'reference030_current_rolling_residual.npz',time_s=raw['time_s'],native_residual_norm_m_s=native_norm,
        guarded_residual_norm_m_s=guarded_norm,point_quality=quality,reference_modified=changed)
    return dict(scope='Current estimated contact kinematics only; not actual tracking/slip/force feasibility. Native LS rows unweighted; belt FF unchanged.',
        point_quality_codes={'0':'no usable current estimated records','1':'exact single-patch global point','2':'plural per-patch points encodedfloat32'},
        available_rows=int(np.sum(quality>0)), exact_single_patch_rows=int(np.sum(quality==1)), plural_float32_rows=int(np.sum(quality==2)),
        modified_rows_with_geometry=int(usable.sum()), modified_rows_without_geometry=int(np.sum(changed&(quality==0))),
        max_native_residual_on_modified_m_s=float(np.max(native_norm[usable])) if usable.any() else None,
        max_guarded_residual_on_modified_m_s=float(np.max(guarded_norm[usable])) if usable.any() else None,
        rms_guarded_residual_on_modified_m_s=float(np.sqrt(np.mean(guarded_norm[usable]**2))) if usable.any() else None)


def reduce(output, protocol, metrics, helper):
    inputs, model_path = input_declaration(output, protocol)
    data={case:load(FAMILY/'campaign/runs'/case/'precision.npz') for case in CASES}
    program=load(ROOT/protocol['inputs']['historical_precision'])
    parity={}
    for case,key in (('base030','i49_base030_precision'),('base060','i49_base060_precision')):
        prior=load(ROOT/protocol['inputs'][key]);parity[case]={k:k in data[case] and same(data[case][k],v) for k,v in prior.items()}
    save(output/'baseline_parity_before_metrics.json',parity)
    assert all(all(checks.values()) for checks in parity.values()), 'I49 baseline parity failed; no scientific interpretation'
    summaries={case:json.loads((FAMILY/'campaign/runs'/case/'summary.json').read_text()) for case in CASES}
    provenance={case:json.loads((FAMILY/'campaign/runs'/case/'provenance.json').read_text()) for case in CASES}
    metadata={case:summaries[case]['allocation_handoff_isolation'] for case in CASES}
    reference=metadata['base030'];fixed={};initial={}
    for case in CASES:
        fixed[case]={key:metadata[case][key]==reference[key] for key in ('fixed_design','ordinary_design_sha256','gain_cache_sha256','design_belt_cap_m_s')}
        fixed[case]['design_bounds_remain030']=metadata[case]['fixed_design']['design_bounds']==BASE_BOUNDS.tolist()
        initial[case]={key:same(data[case][key][0],data['base030'][key][0]) for key in ('exact_qpos','exact_qvel','exact_observer_wrench','exact_estimated_normal','exact_nominal_joints_before','exact_previous_applied','exact_feedback_gain')}
    assert all(all(row.values()) for row in fixed.values()) and all(all(row.values()) for row in initial.values())
    events=json.loads((ROOT/protocol['inputs']['event_turns']).read_text())['rows'];assert len(events)==6
    from spherical_wheel.mobility import TARGETS
    cases={};geometries={}
    for job in protocol['cases']:
        case=job['case_id'];raw=data[case];t=raw['time_s'];summary=summaries[case];cap=job['cap_m_s']
        assert len(t)>0 and np.all(abs(np.diff(t)-DT)<1e-9)
        for key in ('time_s','exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2'):
            assert same(raw[key],program[key][:len(t)]),(case,key)
        bounds=BASE_BOUNDS.copy();bounds[3:]=cap
        assert raw['exact_input_bounds'].shape==(len(t),5)
        np.testing.assert_array_equal(raw['exact_input_bounds'],np.broadcast_to(bounds,(len(t),5)))
        assert np.all(abs(raw['exact_applied'])<=bounds+1e-12)
        log=load(FAMILY/'campaign/runs'/case/'rollout.npz')
        diagnostics=json.loads((FAMILY/'campaign/runs'/case/'candidate_diagnostics.json').read_text());assert len(diagnostics)==len(t)
        contracts=guard_contracts(case,raw,diagnostics)
        track=metrics.tracking_metrics(raw,protocol['evaluation']['timed_moving_window_s'])
        absolute,relative=metrics.attitude(raw);settle=metrics.settling(t,raw['exact_qvel'],relative,summary['completed'])
        commands=metrics.command_metrics(raw,bounds)
        native_raw=dict(raw);native_raw['exact_applied']=raw['exact_native_applied'];commands['pre_handoff_native_box_applied']=metrics.command_metrics(native_raw,bounds)['applied']
        closure=float(np.linalg.norm(raw['exact_qpos'][-1,:2]-raw['exact_qpos'][0,:2]))
        gates=dict(completed=bool(summary['completed']),rms=(track['moving_rms_m'] or 0.)<=TARGETS['rms_m'],
            p95=track['moving_p95_m'] is not None and track['moving_p95_m']<=TARGETS['p95_m'],max=track['full_peak_m']<=TARGETS['max_m'],
            final=track['final_m']<=TARGETS['final_m'],attitude=float(np.degrees(relative).max())<=TARGETS['attitude_deg'],
            saturation=commands['original_any_request_strict_saturated_fraction']<=TARGETS['saturated_fraction'],settled=settle['passed'],closure=closure<=TARGETS['closure_m'])
        assert gates==summary['mars']['gates'],(case,gates,summary['mars']['gates'])
        geometry=metrics.geometry(raw,program,helper,protocol['evaluation']['timed_moving_window_s']);geometries[case]=geometry
        np.savez_compressed(output/(case+'_geometry.npz'),**geometry)
        contexts={}
        for name,window in (('early',protocol['evaluation']['early_window_s']),('late',protocol['evaluation']['late_audit_window_s']),('handoff_motivation',[31.5,32.5])):
            mask=(t>=window[0]-1e-9)&(t<=window[1]+1e-9);context=trace_metrics(t,geometry['local_path_distance_m'],mask)
            context.update(window_s=window,window_complete=bool(t[0]<=window[0]+1e-9 and t[-1]>=window[1]-1e-9));contexts[name]=context
        cases[case]=dict(cap_m_s=cap,intervention=job['intervention'],completed=bool(summary['completed']),failure=summary['failure'],samples=len(t),completed_duration_s=float(t[-1]),
            timed_tracking=track,attitude=dict(peak_absolute_deg=float(np.degrees(absolute).max()),peak_reference_error_deg=float(np.degrees(relative).max()),rms_reference_error_deg=float(np.degrees(np.sqrt(np.mean(relative**2)))),final_reference_error_deg=float(np.degrees(relative[-1]))),
            original_settling=settle,original_route_gates=dict(gates=gates,passed=all(gates.values()),targets=TARGETS,closure_m=closure),
            commands=commands,belts=metrics.belt_metrics(raw,cap),departure=metrics.departure(t,geometry,helper,protocol['evaluation']['early_window_s']),
            turns=metrics.turns(raw,geometry,events,helper),local_path_contexts=contexts,mechanistic=contracts,
            slip_contact=metrics.slip_contact(log,provenance[case]['weight_N']),fixed_design=metadata[case])
        if case=='reference030':cases[case]['mechanistic']['rolling_reference_residual']=current_reference_residual(output,raw,log,model_path)
    comparisons={}
    for baseline,candidate in protocol['pairs']:
        result=metrics.compare(cases[candidate],cases[baseline],protocol['provisional_screen'])
        result['baseline_id']=baseline;result['candidate_id']=candidate
        result['interpretation']='Registered matched-cap one-route mechanism screen only; no combined-candidate or default adoption claim.'
        for row,c,b in zip(result['turn_changes'],cases[candidate]['turns'],cases[baseline]['turns']):
            row.update(baseline_signed_lag_s=b['actual_reversal_lag_s'],candidate_signed_lag_s=c['actual_reversal_lag_s'],
                absolute_lag_error_change_s=None if c['actual_reversal_lag_s'] is None or b['actual_reversal_lag_s'] is None else abs(c['actual_reversal_lag_s'])-abs(b['actual_reversal_lag_s']))
        comparisons[candidate]=result
    report=dict(iteration='I-51',cases=cases,pairs=protocol['pairs'],comparisons=comparisons,
        integrity=dict(i49_common_precision_byte_parity=parity,fixed_design_checks=fixed,initial_state_and_gain_checks=initial,fixed_reference_program=True),
        conventions=dict(held_dt_s=DT,terminal_excluded_from_held_metrics=True,
            raw_request='Common native request means native controller output at that arm state, including the referenceguard whenenabled, before the optional handoffprojection; not an unchanged-baselinecounterfactual.',
            clipping='Original strict rawrequest>ownbounds saturation stays comparable; common clippingdelta isTOTAL actual−raw includingoptionalhandoff. Extra projection is separatelyreported.',
            normalization='Old.30 and eacharm owncap bothreported; absoluteunits retained. No denominator-only improvement.',
            turns='AllsixfixedcorrectedI41events and100msnew-sign dwell. Missing coverage fails screen. Signed increase guard and absolute timingerror are reported separately.',
            geometry='Currentestimated-contactkinematicresidual isnotdynamicfeasibility ortrueslip. Fifth38sphase-schedulecase isoutside thisreport.'),
        limitations=protocol['limitations'])
    save(output/'report.json',report)
    with (output/'case_metrics.csv').open('w',newline='') as stream:
        fields=['case_id','cap_m_s','intervention','completed','moving_rms_m','full_peak_m','final_m','peak_absolute_deg','peak_reference_error_deg','settled','original_route_passed','first20mm_time_s','reference_modified_held_samples','handoff_modified_held_samples']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for case,row in cases.items():
            onset=row['departure']['first_sustained_20mm'];writer.writerow(dict(case_id=case,cap_m_s=row['cap_m_s'],intervention=row['intervention'],completed=row['completed'],
                **{k:row['timed_tracking'][k] for k in ('moving_rms_m','full_peak_m','final_m')},**{k:row['attitude'][k] for k in ('peak_absolute_deg','peak_reference_error_deg')},
                settled=row['original_settling']['passed'],original_route_passed=row['original_route_gates']['passed'],first20mm_time_s=None if onset is None else onset['start_s'],
                reference_modified_held_samples=row['mechanistic']['reference']['modified_held_samples'],handoff_modified_held_samples=row['mechanistic']['handoff']['modified_held_samples']))
    with (output/'turn_metrics.csv').open('w',newline='') as stream:
        fields=['case_id','event_id','reference_time_s','axis','window_complete','geometric_component_overshoot_m','actual_reversal_lag_s','actual_reversal_missing_in_window']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for case,row in cases.items():writer.writerows(dict(case_id=case,**{k:turn[k] for k in fields if k!='case_id'}) for turn in row['turns'])
    plots(output,report,data,geometries)
    for relative,expected in inputs.items():assert sha(ROOT/relative)==expected
    for row in json.loads((output/'declaration_before_execution.json').read_text())['sources']:assert sha(ROOT/row['origin'])==row['sha256']
    for relative,expected in protocol['source_sha256'].items():assert sha(ROOT/relative)==expected
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(output.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']))


def plots(output,report,data,geometries):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(11,7),sharex='col')
    for column,(baseline,candidate) in enumerate(report['pairs']):
        for case,style in ((baseline,'-'),(candidate,'--')):
            raw=data[case];time=raw['time_s'];error=np.linalg.norm(raw['exact_qpos'][:,:2]-raw['exact_reference_xy'],axis=1)*1000
            axes[0,column].plot(time,error,style,label=case,lw=1.2)
            axes[1,column].plot(time,geometries[case]['local_path_distance_m']*1000,style,label=case,lw=1.2)
        axes[0,column].set_title(f'{candidate} vs {baseline} (matched cap)');axes[0,column].legend(fontsize=8)
        axes[1,column].set_xlabel('Recorded time [s]')
    axes[0,0].set_ylabel('Timed position error [mm]');axes[1,0].set_ylabel('Branch-local path distance [mm]')
    for axis in axes.flat:axis.grid(alpha=.25)
    fig.tight_layout();fig.savefig(output/'paired_tracking.png',dpi=160);fig.savefig(output/'paired_tracking.pdf');plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for axis,(baseline,candidate) in zip(axes,report['pairs']):
        x=np.arange(6);a=report['cases'][baseline]['turns'];b=report['cases'][candidate]['turns']
        axis.bar(x-.18,[r['geometric_component_overshoot_m']*1000 if r['geometric_component_overshoot_m'] is not None else np.nan for r in a],.36,label=baseline)
        axis.bar(x+.18,[r['geometric_component_overshoot_m']*1000 if r['geometric_component_overshoot_m'] is not None else np.nan for r in b],.36,label=candidate)
        axis.set_xticks(x,[str(i+1) for i in x]);axis.set_xlabel('Fixed reference component turn');axis.legend(fontsize=8);axis.grid(axis='y',alpha=.25)
    axes[0].set_ylabel('Geometric component overshoot [mm]');fig.tight_layout();fig.savefig(output/'paired_turns.png',dpi=160);fig.savefig(output/'paired_turns.pdf');plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--checks-only',action='store_true');args=parser.parse_args()
    protocol=json.loads((FAMILY/'protocol.json').read_text());assert protocol['iteration']=='I-51' and tuple(j['case_id'] for j in protocol['cases'])==CASES
    output=args.output.resolve();metrics,helper=freeze(output,protocol,args.checks_only)
    try:
        save(output/'pure_checks.json',pure_checks(metrics,helper))
        if not args.checks_only:reduce(output,protocol,metrics,helper)
    except Exception:
        import traceback
        save(output/'failure.json',dict(traceback=traceback.format_exc(),scope='Source/inputs/stage preserved; no automatic overwrite or outcome retry.'))
        raise


if __name__=='__main__':main()
