"""I73 sealed-nine-run evaluation. No new simulation or inference."""
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
    native_vref='contact_reference__native_vref',selected_vref='contact_reference__selected_vref',
    native_ff='contact_reference__native_feedforward',selected_ff='contact_reference__selected_feedforward',
    rolling_called='contact_reference__rolling_called',reference_override='contact_reference__reference_override',
    observer_update_count='native_observer_update_count',native_feedback_verified='native_feedback_boundary_verified',
    prior_matches_physical='mirror_previous_applied_matches_physical',no_truth_read='no_truth_or_terrain_read_during_command',
    command_held_after_row='command_held_after_row',sync_weights='exact_sync_weights',transition_delta='exact_transition_command_delta')


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
    shapes=dict(qpos=(n,10),qvel=(n,9),reference_xy=(n,2),reference_velocity=(n,2),reference_acceleration=(n,2),
        reference_quat=(n,4),requested=(n,5),applied=(n,5),previous_applied=(n,5),caps=(n,5),
        native_vref=(n,9),selected_vref=(n,9),native_ff=(n,5),selected_ff=(n,5),sync_weights=(n,5),transition_delta=(n,5))
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
        for key in ('time_s','reference_xy','reference_velocity','reference_acceleration'):
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
            for key in ('qpos','qvel','previous_applied'):
                checks.append(dict(name=job['case_id']+'_same_phase_initial_'+key,passed=metrics.exact_equal(p[key][0],a[key][0])))
    return checks


def verify_release(protocol,protocol_path,seal_path,runs_root,base):
    """Validate all nine outputs and their frozen control protocol before math."""
    jobs=protocol['cases'];expected={(p,f) for p in ('N','A','C') for f in (-1.,.3,.9)}
    if len(jobs)!=9 or len({j['case_id'] for j in jobs})!=9 or {(j['policy'],float(j['phase_rad'])) for j in jobs}!=expected:
        raise ValueError('ExactlyN/A/C times3registeredphases required')
    seal=json.loads(seal_path.read_text())
    if seal.get('protocol_sha256')!=sha(protocol_path):raise ValueError('Outputseal protocol hash mismatch')
    processes=seal.get('processes',[])
    if (not seal.get('all_processes_successful') or len(processes)!=9 or
        {p['case_id'] for p in processes}!={j['case_id'] for j in jobs} or any(p.get('returncode')!=0 for p in processes)):
        raise ValueError('Allnineprocess outputs must besealed successfully')
    files=seal.get('files',{});resolved={}
    for relative,binding in files.items():
        path=(base/relative).resolve()
        if sha(path)!=binding['sha256'] or path.stat().st_size!=binding['bytes']:
            raise ValueError('Sealed output changed: '+relative)
        resolved[path]=binding
    for job in jobs:
        for name in ('precision.npz','summary.json','provenance.json'):
            if (runs_root/job['case_id']/name).resolve() not in resolved:
                raise ValueError('Requiredcase output notboundbyall-nine seal')
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


def run(protocol_path,runs_root,all_runs_seal,output,fragment=FRAGMENT):
    if not all_runs_seal.is_file():raise ValueError('Allnine-runs-sealed release required before evaluation')
    protocol=json.loads(protocol_path.read_text());config=json.loads(fragment.read_text());jobs=protocol['cases']
    if protocol['iteration']!='I-73' or len(jobs)!=9:raise ValueError('RegisteredI73ninecases required')
    route_path=BASE/'runtime/route.py'
    if config!=protocol['evaluation']:raise ValueError('Metricconfig differs from prephysics protocol')
    for p in (route_path,fragment):
        relative=str(p.resolve().relative_to(ROOT))
        if protocol['source_sha256'].get(relative)!=sha(p):raise ValueError('Frozenroute/gatefragment changed')
    verify_release(protocol,protocol_path,all_runs_seal,runs_root,BASE)
    paths=[Path(__file__),Path(metrics.__file__),fragment,protocol_path,all_runs_seal,route_path]
    for job in jobs:
        folder=runs_root/job['case_id'];paths += [folder/'precision.npz',folder/'summary.json',folder/'provenance.json']
    bindings=[dict(path=str(p),sha256=sha(p)) for p in paths]
    output.mkdir(parents=True,exist_ok=False)
    write(output/'registered_before_metric_arithmetic.json',dict(iteration='I-73',utc=datetime.now(timezone.utc).isoformat(),
        inputs=bindings,scope='Ninefreshclosedloopruns;no newphysics,inference,tuning or simulatortruthcontrol.'))
    for p in (Path(__file__),Path(metrics.__file__),fragment):shutil.copy2(p,output/p.name)
    route=route_module(route_path).SmoothSPath();goal=np.asarray(route(config['clock']['total_s'])[0])
    route_times=np.arange(int(round((config['clock']['moving_s'][1]-config['clock']['moving_s'][0])/metrics.DT))+1)*metrics.DT+config['clock']['moving_s'][0]
    path=np.array([route(float(t))[0] for t in route_times])
    packets,rows,signals,metadata={},{},{},{}
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
        row,s=metrics.case_metrics(p,summary,config);row.update(case=job,actual_support=support_diagnostics(raw))
        rows[case]=row;signals[case]=s;metadata[case]=dict(summary_completed=summary.get('completed'),failure=summary.get('failure'),
            fixed_bank_bytes_sha256=provenance.get('fixed_bank_bytes_sha256'),fixed_bank_numeric_sha256=provenance.get('fixed_bank_numeric_sha256'),
            source_hashes_unchanged_after=provenance.get('source_hashes_unchanged_after'),protocol_sha256=provenance.get('protocol_sha256'))
        np.savez_compressed(output/(case+'_signals.npz'),**s)
    setup=pair_contracts(packets,jobs,route)
    for case,row in rows.items():setup.append(dict(name=case+'_same_current_state_native_invariants',passed=row['invariant_ok']))
    banks={v['fixed_bank_bytes_sha256'] for v in metadata.values()};banknums={v['fixed_bank_numeric_sha256'] for v in metadata.values()}
    setup.append(dict(name='all_cases_same_frozen_gain_bank',passed=len(banks)==1 and None not in banks and len(banknums)==1 and None not in banknums))
    setup.append(dict(name='all_sources_unchanged_after_cases',passed=all(v['source_hashes_unchanged_after'] for v in metadata.values())))
    grouped={str(phase):{j['policy']:rows[j['case_id']] for j in jobs if float(j['phase_rad'])==phase}
             for phase in sorted({float(j['phase_rad']) for j in jobs})}
    decision=metrics.decide(grouped,all(x['passed'] for x in setup))
    science=dict(iteration='I-73',new_physics_during_evaluation=0,cases=rows,setup_contracts=setup,
        decision=decision,metadata=metadata,primary_policy='C',secondary_policy='A',metric_config=config,
        interpretation='Freshphysicalreference-pathscreen;I72failureunchanged.No automaticadoption. Referenceexposureisnotapplied-commandcounterfactual. Worldyawisphysicalomega_world_z,notEulerheadingderivative.')
    for item in bindings:
        if sha(item['path'])!=item['sha256']:raise ValueError('Sealed source/input changed duringevaluation')
    write(output/'science.json',science)
    write(output/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(safe(dict(decision=decision,cases={k:dict(settled=v['settling']['passed'],exposure=v['exposure'],tracking=v['tracking']) for k,v in rows.items()})),allow_nan=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('protocol','runs-root','all-runs-seal','output'):p.add_argument('--'+key,type=Path,required=True)
    a=p.parse_args();run(a.protocol.resolve(),a.runs_root.resolve(),a.all_runs_seal.resolve(),a.output.resolve())
