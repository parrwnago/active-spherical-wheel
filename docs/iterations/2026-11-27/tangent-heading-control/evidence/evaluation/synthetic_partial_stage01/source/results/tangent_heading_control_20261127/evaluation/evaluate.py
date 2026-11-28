"""I75 sealed-six-run heading evaluation. No simulation or new inference."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,importlib.util,json,math,shutil
import numpy as np
import metrics

BASE=Path(__file__).resolve().parents[1]
ROOT=Path(__file__).resolve().parents[3]
FRAGMENT=Path(__file__).with_name('protocol_metrics_fragment.json')
ALIASES=dict(time_s='time_s',qpos='exact_qpos',qvel='exact_qvel',reference_xy='exact_reference_xy',
    reference_velocity='exact_reference_velocity_m_s',reference_acceleration='exact_reference_acceleration_m_s2',
    reference_quat='exact_reference_quat',requested='exact_requested',applied='exact_applied',
    previous_applied='exact_previous_applied',caps='exact_input_bounds',
    reference_qvel='exact_reference_qvel',rolling_feedforward='exact_rolling_feedforward',
    observer_update_count='native_observer_update_count',native_feedback_verified='native_feedback_boundary_verified',
    prior_matches_physical='mirror_previous_applied_matches_physical',no_truth_read='no_truth_or_terrain_read_during_command',
    command_held_after_row='command_held_after_row',sync_weights='exact_sync_weights',transition_delta='exact_transition_command_delta',
    raw_heading='heading__raw_heading_rad',raw_rate='heading__raw_heading_rate_rad_s',
    raw_acceleration='heading__raw_heading_acceleration_rad_s2',
    command_heading='heading__command_heading_rad',command_rate='heading__command_heading_rate_rad_s',
    command_acceleration='heading__command_heading_acceleration_rad_s2',
    raw_error_logged='heading__raw_error_rad',command_error_logged='heading__command_error_rad',
    initial_latch='heading__initial_heading_latched_rad',actual_helper_target='heading__actual_helper_target_rad',
    actual_helper_rate_reference='heading__actual_helper_rate_reference_rad_s',
    actual_helper_rate_measurement='heading__actual_helper_rate_measurement_rad_s',
    euler_rate_logged='heading__euler_heading_rate_rad_s',world_z_rate_logged='heading__world_z_rate_rad_s',
    rate_valid_logged='heading__rate_valid',
    heading_command='exact_heading_command',heading_eligible='heading__eligible',
    helper_matches='heading__command_matches_registered_helper',
    vertical_lever='heading__vertical_lever',free_shell_mask='heading__free_shell_mask')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(v):
    if isinstance(v,dict):return {str(k):safe(x) for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [safe(x) for x in v]
    if isinstance(v,np.ndarray):return safe(v.tolist())
    if isinstance(v,np.generic):return safe(v.item())
    if isinstance(v,float) and not math.isfinite(v):return None
    return v


def write(path,value):Path(path).write_text(json.dumps(safe(value),indent=2,allow_nan=False)+'\n')


def load(path):
    with np.load(path,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}


def canonical(raw):
    packet={key:np.asarray(raw[source]).copy() for key,source in ALIASES.items()}
    n=len(packet['time_s'])
    if packet['time_s'].shape!=(n,):raise ValueError('Incorrect time clock shape')
    shapes=dict(qpos=(n,10),qvel=(n,9),reference_xy=(n,2),reference_velocity=(n,2),reference_acceleration=(n,2),
        reference_quat=(n,4),requested=(n,5),applied=(n,5),previous_applied=(n,5),caps=(n,5),
        reference_qvel=(n,9),rolling_feedforward=(n,5),sync_weights=(n,5),transition_delta=(n,5),
        heading_command=(n,5),free_shell_mask=(n,2))
    for key,shape in shapes.items():
        if packet[key].shape!=shape:raise ValueError('Incorrect precision shape for '+key)
    for key in set(packet)-set(shapes)-{'time_s'}:
        if packet[key].shape!=(n,):raise ValueError('Incorrect rowflag shape for '+key)
    return packet


def pair_contracts(packets,jobs,route=None):
    checks=[]
    anchor=packets[jobs[0]['case_id']]
    for job in jobs:
        case=job['case_id'];p=packets[case]
        common=min(len(p['time_s']),len(anchor['time_s']))
        for key in ('time_s','reference_xy','reference_velocity','reference_acceleration','raw_heading','raw_rate','raw_acceleration','command_heading','command_rate','command_acceleration'):
            checks.append(dict(name=case+'_same_registered_raw_'+key+'_commonprefix',
                passed=common>0 and metrics.exact_equal(p[key][:common],anchor[key][:common])))
        if route is not None:
            expected=[route(float(t)) for t in p['time_s']]
            for index,key in enumerate(('reference_xy','reference_velocity','reference_acceleration')):
                values=np.asarray([r[index] for r in expected])
                checks.append(dict(name=case+'_matches_frozen_route_'+key,passed=metrics.exact_equal(p[key],values)))
    phases=sorted({float(j['phase_rad']) for j in jobs})
    for phase in phases:
        group=[j for j in jobs if float(j['phase_rad'])==phase]
        native=next(j for j in group if j['policy']=='N');a=packets[native['case_id']]
        for job in group:
            p=packets[job['case_id']]
            for key in ('qpos','qvel','previous_applied','initial_latch'):
                checks.append(dict(name=job['case_id']+'_same_phase_initial_'+key,passed=metrics.exact_equal(p[key][0],a[key][0])))
    return checks


def verify_release(protocol,protocol_path,seal_path,runs_root,base):
    """Validate all six outputs and their frozen control protocol before math."""
    jobs=protocol['cases'];expected={(p,f) for p in ('N','T') for f in (-1.,.3,.9)}
    if len(jobs)!=6 or len({j['case_id'] for j in jobs})!=6 or {(j['policy'],float(j['phase_rad'])) for j in jobs}!=expected:
        raise ValueError('Exactly N/H times3 registered phases required')
    seal=json.loads(seal_path.read_text())
    if seal.get('protocol_sha256')!=sha(protocol_path):raise ValueError('Outputseal protocol hash mismatch')
    processes=seal.get('processes',[])
    if (not seal.get('all_processes_successful') or len(processes)!=6 or
        {p['case_id'] for p in processes}!={j['case_id'] for j in jobs} or any(p.get('returncode')!=0 for p in processes)):
        raise ValueError('All six process outputs must be sealed successfully')
    files=seal.get('files',{});resolved={}
    for relative,binding in files.items():
        path=(base/relative).resolve()
        if sha(path)!=binding['sha256'] or path.stat().st_size!=binding['bytes']:
            raise ValueError('Sealed output changed: '+relative)
        resolved[path]=binding
    for job in jobs:
        for name in ('precision.npz','summary.json','provenance.json'):
            if (runs_root/job['case_id']/name).resolve() not in resolved:
                raise ValueError('Required case output not bound by all-six seal')
    return seal


def support_diagnostics(raw):
    if 'truth_patch_loads_N' not in raw:return dict(available=False)
    loads=raw['truth_patch_loads_N'];total=loads.sum(axis=1);valid=np.isfinite(loads).all(axis=1)&(loads>=0).all(axis=1)&(total>=1.)
    shares=np.divide(loads,total[:,None],out=np.zeros_like(loads),where=total[:,None]>0)
    bits=np.sum((shares>=.1)*(1<<np.arange(4)),axis=1);bits[~valid]=0
    changes=int(np.sum(bits[1:]!=bits[:-1]))
    return dict(available=True,positive_pressure_set_changes=changes,
        sampled_pressure_singleton_rows=int(np.sum(valid&(np.sum(shares>=.1,axis=1)==1))),
        sampled_pressure_pair_or_more_rows=int(np.sum(valid&(np.sum(shares>=.1,axis=1)>=2))),
        scope='Descriptive last1ms positive-pressure samples; not interval allforce-routing truth or estimatoraccuracy gate.')


def route_module(path):
    spec=importlib.util.spec_from_file_location('i73_metric_route',path);module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module);return module


def verify_frozen_binding(protocol,path):
    """Control registration separates executable sources from frozen inputs."""
    relative=str(path.resolve().relative_to(ROOT))
    expected=[protocol.get(group,{}).get(relative) for group in ('source_sha256','input_sha256')]
    expected=[value for value in expected if value is not None]
    if not expected or len(set(expected))!=1 or expected[0]!=sha(path):
        raise ValueError('Frozen source/input binding changed: '+relative)


def telemetry_contracts(packet,policy,config):
    """Independent target/rate/frame checks against actual recorded states."""
    n=len(packet['time_s']);R=metrics.rotation(packet['qpos'][:,3:7])
    yaw=np.arctan2(R[:,1,0],R[:,0,0]);omega=np.einsum('nij,nj->ni',R,packet['qvel'][:,3:6])
    rate,valid=metrics.euler_heading_rate(R,omega,config['reference_guards']['forward_projection_denominator_min'])
    close=lambda a,b:bool(np.allclose(a,b,atol=1e-12,rtol=1e-12,equal_nan=True))
    angleclose=lambda a,b:bool(np.all(abs(metrics.wrapped(np.asarray(a)-np.asarray(b)))<=1e-12))
    latch=packet['initial_latch'];is_t=policy=='T'
    target=packet['command_heading'] if is_t else latch
    desiredrate=packet['command_rate'] if is_t else np.zeros(n)
    measuredrate=rate if is_t else omega[:,2]
    return [
        dict(name='initial_latch_constant',passed=np.array_equal(latch,np.full(n,latch[0]))),
        dict(name='initial_latch_matches_initial_physical_yaw',passed=angleclose(latch[0],yaw[0])),
        dict(name='raw_error_matches_raw_path',passed=angleclose(packet['raw_error_logged'],yaw-packet['raw_heading'])),
        dict(name='command_error_matches_command_target',passed=angleclose(packet['command_error_logged'],yaw-packet['command_heading'])),
        dict(name='exact_Euler_rate_telemetry',passed=close(packet['euler_rate_logged'],rate)),
        dict(name='physical_world_z_rate_telemetry',passed=close(packet['world_z_rate_logged'],omega[:,2])),
        dict(name='rate_validity_telemetry',passed=np.array_equal(packet['rate_valid_logged'],valid)),
        dict(name='actual_helper_target_policy',passed=angleclose(packet['actual_helper_target'],target)),
        dict(name='actual_helper_rate_reference_policy',passed=close(packet['actual_helper_rate_reference'],desiredrate)),
        dict(name='actual_helper_rate_measurement_frame',passed=close(packet['actual_helper_rate_measurement'],measuredrate)),
        dict(name='native_helper_disabled',passed=is_t or not np.any(packet['heading_command'])),
    ]


def run(protocol_path,runs_root,all_runs_seal,output,fragment=FRAGMENT):
    if not all_runs_seal.is_file():raise ValueError('All-six-runs-sealed release required before evaluation')
    protocol=json.loads(protocol_path.read_text());config=json.loads(fragment.read_text());jobs=protocol['cases']
    if protocol['iteration']!='I-75' or len(jobs)!=6:raise ValueError('Registered I75 six cases required')
    route_path=BASE/'runtime/route.py'
    if config!=protocol['evaluation']:raise ValueError('Metricconfig differs from prephysics protocol')
    for p in (route_path,fragment):
        verify_frozen_binding(protocol,p)
    verify_release(protocol,protocol_path,all_runs_seal,runs_root,BASE)
    paths=[Path(__file__),Path(metrics.__file__),fragment,protocol_path,all_runs_seal,route_path]
    for job in jobs:
        folder=runs_root/job['case_id'];paths += [folder/'precision.npz',folder/'summary.json',folder/'provenance.json']
    bindings=[dict(path=str(p),sha256=sha(p)) for p in paths]
    output.mkdir(parents=True,exist_ok=False)
    write(output/'registered_before_metric_arithmetic.json',dict(iteration='I-75',utc=datetime.now(timezone.utc).isoformat(),
        inputs=bindings,scope='Six fresh paired trajectory-aligned starts; no new physics/inference/tuning/simulatortruthcontrol duringevaluation.'))
    for p in (Path(__file__),Path(metrics.__file__),fragment):shutil.copy2(p,output/p.name)
    route=route_module(route_path).SmoothSPath();goal=np.asarray(route(config['clock']['total_s'])[0])
    route_times=np.arange(int(round((config['clock']['moving_s'][1]-config['clock']['moving_s'][0])/metrics.DT))+1)*metrics.DT+config['clock']['moving_s'][0]
    path=np.array([route(float(t))[0] for t in route_times])
    packets,rows,signals,metadata,targetchecks={},{},{},{},[]
    for job in jobs:
        case=job['case_id'];folder=runs_root/case;raw=load(folder/'precision.npz');p=canonical(raw)
        p.update(goal_xy=goal,path_xy=path);packets[case]=p
        summary=json.loads((folder/'summary.json').read_text());provenance=json.loads((folder/'provenance.json').read_text())
        if provenance.get('protocol_sha256')!=sha(protocol_path) or provenance.get('case')!=job:
            raise ValueError('Caseprovenance differs from registeredprotocol/job')
        if provenance.get('fixed_bank_bytes_sha256')!=protocol['gain_cache']['sha256']:
            raise ValueError('Case used differentgainbank than registeredbytes')
        expected_caps=np.broadcast_to(np.asarray(protocol['bounds'],float),p['caps'].shape)
        if not np.array_equal(p['caps'],expected_caps):raise ValueError('Actualinputbounds differfromregisteredcaps')
        expected_reference=metrics.raw_reference(route,p['time_s'])
        for key,values in expected_reference.items():
            if key=='raw_heading':passed=bool(np.all(abs(metrics.wrapped(p[key]-values))<=1e-12))
            else:passed=bool(np.allclose(p[key],values,atol=1e-12,rtol=1e-12))
            targetchecks.append(dict(name=case+'_matches_independent_'+key,passed=passed))
        expected_initial_yaw=float(np.degrees(metrics.raw_reference(route,np.array([0.]))['raw_heading'][0]))
        targetchecks.append(dict(name=case+'_registered_initial_yaw_aligned_to_raw_tangent',
            passed=bool(abs(job['yaw_deg']-expected_initial_yaw)<=1e-10)))
        for item in telemetry_contracts(p,job['policy'],config):
            targetchecks.append(dict(name=case+'_'+item['name'],passed=bool(item['passed'])))
        row,s=metrics.case_metrics(p,summary,config);row.update(case=job,actual_support=support_diagnostics(raw))
        rows[case]=row;signals[case]=s;metadata[case]=dict(summary_completed=summary.get('completed'),failure=summary.get('failure'),
            fixed_bank_bytes_sha256=provenance.get('fixed_bank_bytes_sha256'),fixed_bank_numeric_sha256=provenance.get('fixed_bank_numeric_sha256'),
            source_hashes_unchanged_after=provenance.get('source_hashes_unchanged_after'),protocol_sha256=provenance.get('protocol_sha256'))
        np.savez_compressed(output/(case+'_signals.npz'),**s)
    setup=pair_contracts(packets,jobs,route)+targetchecks
    for case,row in rows.items():
        setup.append(dict(name=case+'_same_current_state_native_invariants',passed=row['invariant_ok']))
        setup.append(dict(name=case+'_actual_initial_raw_target_alignment',passed=row['heading']['initial_alignment_ok']))
        setup.append(dict(name=case+'_raw_command_reference_guards',passed=row['heading']['reference_guards_ok']))
    banks={v['fixed_bank_bytes_sha256'] for v in metadata.values()};banknums={v['fixed_bank_numeric_sha256'] for v in metadata.values()}
    setup.append(dict(name='all_cases_same_frozen_gain_bank',passed=len(banks)==1 and None not in banks and len(banknums)==1 and None not in banknums))
    setup.append(dict(name='all_sources_unchanged_after_cases',passed=all(v['source_hashes_unchanged_after'] for v in metadata.values())))
    grouped={str(phase):{j['policy']:rows[j['case_id']] for j in jobs if float(j['phase_rad'])==phase}
             for phase in sorted({float(j['phase_rad']) for j in jobs})}
    decision=metrics.decide(grouped,all(x['passed'] for x in setup))
    science=dict(iteration='I-75',new_physics_during_evaluation=0,cases=rows,setup_contracts=setup,
        decision=decision,metadata=metadata,primary_policy='T',comparison_policy='N',metric_config=config,
        interpretation='Trajectory-tangent heading with exact Euler-rate feedback versus native observer baseline. Raw geometric objectives are primary; command-target lag separately reported. Physical world-z angular velocity is diagnostic, not Euler heading rate. No old-baseline replay because both initial yaws changed. No automatic adoption; settling descriptive.')
    for item in bindings:
        if sha(item['path'])!=item['sha256']:raise ValueError('Sealed source/input changed duringevaluation')
    write(output/'science.json',science)
    write(output/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(safe(dict(decision=decision,cases={k:dict(settled=v['settling']['passed'],activity=v['activity'],tracking=v['tracking'],heading=v['heading']) for k,v in rows.items()})),allow_nan=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('protocol','runs-root','all-runs-seal','output'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();run(a.protocol.resolve(),a.runs_root.resolve(),a.all_runs_seal.resolve(),a.output.resolve())
