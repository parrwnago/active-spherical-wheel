"""Separately registered I51 fifth-run late phase-schedule diagnostic.

Fixed38–42/38–44windows and matched history before38s. No modification or
reduction of the four-arm protocol, and no controller/model/physics calls.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse
import csv
import hashlib
import json
import shutil
import types

import numpy as np

HERE=Path(__file__).resolve().parent
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]
PHASE=FAMILY/'phase_schedule_isolation'
PRIMARY=HERE/'analyze_isolation.py'
PRIMARY_SHA='80cab5a1edc6a4783f5647e1c1185b6233048b58c43e1f0c8228fefb5e0be592'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def safe(value):
    if isinstance(value,dict):return {str(k):safe(v)for k,v in value.items()}
    if isinstance(value,(tuple,list)):return [safe(v)for v in value]
    if isinstance(value,np.ndarray):return safe(value.tolist())
    if isinstance(value,np.generic):return safe(value.item())
    if isinstance(value,Path):return str(value)
    if isinstance(value,float)and not np.isfinite(value):return None
    return value
def save(path,value):path.write_text(json.dumps(safe(value),indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False)as arrays:return {key:arrays[key].copy()for key in arrays.files}
def module(path,name):
    result=types.ModuleType(name);result.__file__=str(path);exec(compile(path.read_text(),str(path),'exec'),result.__dict__);return result


def yaw(qpos):
    w,x,y,z=qpos[:,3:7].T
    return np.unwrap(np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z)))


def late_metrics(raw,metrics,geometry,window):
    t=raw['time_s'];mask=(t>=window[0]-1e-9)&(t<=window[1]+1e-9)
    error=np.linalg.norm(raw['exact_qpos'][:,:2]-raw['exact_reference_xy'],axis=1)
    absolute,relative=metrics.attitude(raw);heading=yaw(raw['exact_qpos'])
    rate_error=raw['exact_qvel'][:,6:9]-raw['exact_feedback_reference_qvel'][:,6:9]
    selected=np.flatnonzero(mask)
    if not len(selected):return dict(window_s=window,available=False,window_complete=False)
    local=geometry['local_path_distance_m'][mask]
    return dict(window_s=window,available=True,samples=len(selected),window_complete=bool(t[0]<=window[0]+1e-9 and t[-1]>=window[1]-1e-9),
        timed_rms_m=float(np.sqrt(np.mean(error[mask]**2))),timed_peak_m=float(error[mask].max()),timed_final_m=float(error[selected[-1]]),
        local_path_rms_m=float(np.sqrt(np.mean(local**2))),local_path_peak_m=float(local.max()),
        peak_absolute_lean_deg=float(np.degrees(absolute[mask]).max()),peak_reference_relative_lean_deg=float(np.degrees(relative[mask]).max()),
        rms_reference_relative_lean_deg=float(np.degrees(np.sqrt(np.mean(relative[mask]**2)))),
        yaw_change_from_window_start_deg=float(np.degrees(heading[selected[-1]]-heading[selected[0]])),
        maximum_yaw_change_from_window_start_deg=float(np.degrees(np.max(abs(heading[mask]-heading[selected[0]])))),
        yaw_at_window_start_deg=float(np.degrees(heading[selected[0]])),
        joint_rate_tracking_rms_rad_s=np.sqrt(np.mean(rate_error[mask]**2,axis=0)),joint_rate_tracking_max_abs_rad_s=np.max(abs(rate_error[mask]),axis=0),
        max_abs_joint_reference_rad_s=np.max(abs(raw['exact_feedback_reference_qvel'][mask,6:9]),axis=0),
        max_abs_actual_joint_rate_rad_s=np.max(abs(raw['exact_qvel'][mask,6:9]),axis=0))


def run(output,checks_only):
    output.mkdir(parents=True,exist_ok=False)
    protocol=json.loads((PHASE/'protocol.json').read_text())
    assert protocol['execution']['interventions']['phase_late']['switch_time_s']==38.
    assert protocol['evaluation']['primary_late_window_s']==[38.,42.] and protocol['evaluation']['secondary_late_window_s']==[38.,44.]
    assert sha(PRIMARY)==PRIMARY_SHA
    sources=[Path(__file__).resolve(),PRIMARY,PHASE/'protocol.json',FAMILY/'protocol.json',
        ROOT/'results/belt_capacity_sweep_20261126/analyze_capacity.py',ROOT/protocol['inputs']['event_helper'],
        ROOT/protocol['inputs']['event_turns'],ROOT/'src/spherical_wheel/mobility.py']
    captured=[]
    for source in sources:
        target=output/'source_snapshot'/source.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,target)
        captured.append(dict(origin=str(source.relative_to(ROOT)),snapshot=str(target.relative_to(output)),sha256=sha(source)))
    save(output/'declaration_before_execution.json',dict(captured_utc=datetime.now(timezone.utc).isoformat(),sources=captured,checks_only=checks_only,
        scope='Separate fifth-run addendum. Fixed registered38–42/38–44windows andbefore38prefix. No controller/model/observer/physics/forecast calls.',
        disclosure='Definitions/window fixed in separateprotocol before itsnativeexecution. Root independentheadline results were supplied before this implementation; no metric/window/candidate setting was selected or tuned from them.'))
    primary=module(PRIMARY,'_i51_primary_definitions_readonly')
    assert sha(ROOT/'results/belt_capacity_sweep_20261126/analyze_capacity.py')==primary.I49_SHA256
    for key in ('event_helper','event_turns'):
        assert sha(ROOT/protocol['inputs'][key])==protocol['input_sha256'][key]
    metrics=module(ROOT/'results/belt_capacity_sweep_20261126/analyze_capacity.py','_i51_frozen_i49_phase_metrics')
    helper=module(ROOT/protocol['inputs']['event_helper'],'_i51_phase_corrected_i41')
    # Pure quaternion/window fixtures only; repeated helper checks are not
    # counted as additional repository tests.
    q=np.zeros((3,10));q[:,3]=1.;assert np.array_equal(yaw(q),np.zeros(3))
    save(output/'pure_checks.json',dict(passed=True,scope='Yaw identity andfixedregisteredactivation/window schema. No nativeoutcomes read for this check.'))
    if checks_only:return
    inputs={}
    for key,relative in protocol['inputs'].items():assert sha(ROOT/relative)==protocol['input_sha256'][key]
    for relative,expected in protocol['source_sha256'].items():assert sha(ROOT/relative)==expected
    folders={'base030':FAMILY/'campaign/runs/base030','phase_late030':PHASE/'campaign/runs/phase_late030'}
    for folder in folders.values():
        manifest=json.loads((folder/'artifact_manifest.json').read_text())
        for row in manifest['files']:assert sha(folder/row['path'])==row['sha256']
        for name in ('precision.npz','rollout.npz','summary.json','provenance.json','artifact_manifest.json'):
            source=folder/name;inputs[str(source.relative_to(ROOT))]=sha(source)
    primary_report=HERE/'stage01/report.json';assert primary_report.is_file();inputs[str(primary_report.relative_to(ROOT))]=sha(primary_report)
    save(output/'inputs_before_reduction.json',inputs)
    data={case:load(folder/'precision.npz')for case,folder in folders.items()};base=data['base030'];candidate=data['phase_late030']
    t=candidate['time_s'];np.testing.assert_array_equal(t,base['time_s'])
    before=t<38.-1e-9;activation=int(np.argmin(abs(t-38.)));assert abs(t[activation]-38.)<1e-8
    common=sorted(set(base)&set(candidate));parity={key:primary.same(base[key][before],candidate[key][before])for key in common}
    assert all(parity.values()),'Fifth-run prefixdidnotmatchfreshbase030'
    states={key:primary.same(base[key][activation],candidate[key][activation])for key in ('exact_qpos','exact_qvel','exact_previous_applied','exact_observer_wrench','exact_estimated_normal','exact_nominal_joints_before','exact_reference_xy','exact_reference_qvel')}
    assert all(states.values())
    np.testing.assert_array_equal(candidate['exact_phase_schedule_on_normal'],t>=38.-1e-9)
    np.testing.assert_array_equal(candidate['exact_applied'],np.clip(candidate['exact_native_requested'],-candidate['exact_input_bounds'],candidate['exact_input_bounds']))
    np.testing.assert_array_equal(candidate['exact_previous_applied'][1:],candidate['exact_applied'][:-1])
    program=load(ROOT/protocol['inputs']['historical_precision'])
    canonical=json.loads(primary_report.read_text());cases={};geometries={}
    from spherical_wheel.mobility import TARGETS
    events=json.loads((ROOT/protocol['inputs']['event_turns']).read_text())['rows']
    for case,raw in data.items():
        summary=json.loads((folders[case]/'summary.json').read_text())
        for key in ('time_s','exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2'):assert primary.same(raw[key],program[key])
        geometry=metrics.geometry(raw,program,helper,protocol['evaluation']['timed_moving_window_s']);geometries[case]=geometry
        np.savez_compressed(output/(case+'_geometry.npz'),**geometry)
        absolute,relative=metrics.attitude(raw);track=metrics.tracking_metrics(raw,protocol['evaluation']['timed_moving_window_s'])
        settle=metrics.settling(raw['time_s'],raw['exact_qvel'],relative,summary['completed']);command=metrics.command_metrics(raw,np.array([1.5,.6,.6,.3,.3]))
        closure=float(np.linalg.norm(raw['exact_qpos'][-1,:2]-raw['exact_qpos'][0,:2]))
        gates=dict(completed=bool(summary['completed']),rms=track['moving_rms_m']<=TARGETS['rms_m'],p95=track['moving_p95_m']<=TARGETS['p95_m'],
            max=track['full_peak_m']<=TARGETS['max_m'],final=track['final_m']<=TARGETS['final_m'],attitude=float(np.degrees(relative).max())<=TARGETS['attitude_deg'],
            saturation=command['original_any_request_strict_saturated_fraction']<=TARGETS['saturated_fraction'],settled=settle['passed'],closure=closure<=TARGETS['closure_m'])
        assert gates==summary['mars']['gates']
        cases[case]=dict(completed=bool(summary['completed']),failure=summary['failure'],timed_tracking=track,
            attitude=dict(peak_absolute_deg=float(np.degrees(absolute).max()),peak_reference_error_deg=float(np.degrees(relative).max())),
            original_settling=settle,original_route_gates=dict(gates=gates,passed=all(gates.values())),commands=command,
            turns=metrics.turns(raw,geometry,events,helper),
            late={name:late_metrics(raw,metrics,geometry,protocol['evaluation'][key])for name,key in (('primary38_42','primary_late_window_s'),('secondary38_44','secondary_late_window_s'))})
    # The baseline metrics from the independent addendum must match canonical
    # primary data, but no primary report is mutated or replaced.
    assert cases['base030']['timed_tracking']==canonical['cases']['base030']['timed_tracking']
    gain_diff=np.any(base['exact_feedback_gain']!=candidate['exact_feedback_gain'],axis=(1,2))
    report=dict(iteration='I-51',diagnostic='separately_registered_late_phase_schedule',baseline_id='base030',candidate_id='phase_late030',
        prefix=dict(samples=int(before.sum()),all_common_precision_fields_byte_identical=parity,state_and_reference_at38_equal=states),
        first_gain_difference_s=float(t[np.flatnonzero(gain_diff)[0]])if gain_diff.any()else None,cases=cases,
        interpretation='Late-only scheduling diagnostic aftermatchedhistory. Effectsinthese windows support anepisode-specificcontribution, not globalnormal-phase adoption, a universalcontrollerfix or a newallocationpolicy.',
        limitations=protocol['limitations'],clock='Registered38s threshold uses1e-9roundoff tolerance; latewindows includeendpoints withthe sameclocktolerance. Original movingmetrics retainI49strictbounds.')
    save(output/'report.json',report)
    with(output/'late_metrics.csv').open('w',newline='')as stream:
        fields=['case_id','window','timed_rms_m','timed_peak_m','timed_final_m','local_path_rms_m','peak_reference_relative_lean_deg','yaw_change_from_window_start_deg']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for case,info in cases.items():
            for name,late in info['late'].items():writer.writerow(dict(case_id=case,window=name,**{key:late[key]for key in fields if key not in ('case_id','window')}))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
    for case,raw in data.items():
        mask=raw['time_s']>=37.5
        error=np.linalg.norm(raw['exact_qpos'][:,:2]-raw['exact_reference_xy'],axis=1)
        axes[0].plot(raw['time_s'][mask],error[mask]*1000,label=case)
        axes[1].plot(raw['time_s'][mask],np.degrees(yaw(raw['exact_qpos'])[mask]),label=case)
    axes[0].set_ylabel('Timed error [mm]');axes[1].set_ylabel('Unwrapped yaw [deg]');axes[1].set_xlabel('Recorded time [s]')
    for axis in axes:axis.axvline(38,color='k',ls=':',lw=1);axis.grid(alpha=.25);axis.legend()
    fig.tight_layout();fig.savefig(output/'late_phase_comparison.png',dpi=160);fig.savefig(output/'late_phase_comparison.pdf');plt.close(fig)
    for relative,expected in inputs.items():assert sha(ROOT/relative)==expected
    for row in captured:assert sha(ROOT/row['origin'])==row['sha256']
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),bytes=p.stat().st_size,sha256=sha(p))for p in sorted(output.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--checks-only',action='store_true');args=parser.parse_args()
    try:run(args.output.resolve(),args.checks_only)
    except Exception:
        import traceback
        if args.output.exists():save(args.output/'failure.json',dict(traceback=traceback.format_exc(),scope='Failedstagepreserved;nooverwriteorretrybyoutcome.'))
        raise
