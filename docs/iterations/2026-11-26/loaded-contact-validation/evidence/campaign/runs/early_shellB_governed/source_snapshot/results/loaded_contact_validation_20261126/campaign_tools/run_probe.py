"""I57 matched-prefix open-loop probes; explicit --run starts fresh Gazebo physics.

No saved-pose reset: replay every original governed5ms hold from the original
placement. Controller is constructed for initialization only; command and
observer update are forbidden for the entire physical replay. Source original
simulation truth and contacts are evaluation data, not a control measurement.
"""
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch
from contextlib import ExitStack
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import time
import traceback
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT=Path(__file__).resolve().parents[1]
DT=.005
BOUNDS=np.array([1.5,.6,.6,.3,.3])
FROZEN_BANK_SHA256='45ab9990aa3961d1202533acf51d5d8a9294eefe5a4c7e6cfde6aa0b3ac462c1'
MODES=('continue_saved','hold_native','hold_governed','hold_midpoint')
SOURCE_FIELDS=('time_s','exact_qpos','exact_qvel','exact_previous_applied','exact_applied','exact_native_applied')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value,dict):return {str(k):safe(v)for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [safe(v)for v in value]
    if isinstance(value,np.ndarray):return safe(value.tolist())
    if isinstance(value,np.generic):return safe(value.item())
    if isinstance(value,Path):return str(value)
    if isinstance(value,float)and not np.isfinite(value):return None
    return value


def save(path,value):Path(path).write_text(json.dumps(safe(value),indent=2,allow_nan=False)+'\n')


def fingerprint(value):return hashlib.sha256(json.dumps(safe(value),sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def bit_exact(a,b):
    a,b=np.asarray(a),np.asarray(b)
    return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()


def read_source(protocol):
    source=protocol['source_data']
    with np.load(ROOT/source['precision_path'],allow_pickle=False)as z:
        arrays={k:z[k].copy()for k in SOURCE_FIELDS}
    provenance=json.loads((ROOT/source['provenance_path']).read_text())
    original=json.loads((ROOT/source['protocol_path']).read_text())
    return arrays,provenance,original


def center_and_sequence(job,event,raw):
    index=event['index'];mode=job['mode']
    if mode=='continue_saved':
        return None,raw['exact_applied'][index:index+4].copy()
    native,governed=raw['exact_native_applied'][index],raw['exact_applied'][index]
    center={'hold_native':native,'hold_governed':governed,'hold_midpoint':.5*(native+governed)}[mode].copy()
    command=center+np.asarray(job.get('delta_input',[0.]*5),float)
    if np.any(abs(command)>BOUNDS):raise ValueError('Registered command would require clipping')
    return center,np.tile(command,(4,1))


def preflight(output,case_id=None):
    protocol=json.loads((output/'protocol.json').read_text())
    if protocol['iteration']!='I-57':raise ValueError('Expected a new I-57 protocol')
    if protocol.get('endpoint_offsets_steps',[0,1,2,3,4])!=[0,1,2,3,4]:
        raise ValueError('All 0/5/10/15/20ms endpoints required')
    if len(protocol['cases'])>30:raise ValueError('At most30 registered branches')
    for relative,expected in protocol['source_sha256'].items():
        if sha(ROOT/relative)!=expected:raise ValueError('Registered source changed: '+relative)
    for relative,expected in protocol['input_sha256'].items():
        if sha(ROOT/relative)!=expected:raise ValueError('Registered input changed: '+relative)
    for kind in ('gain_cache','bridge_binary'):
        item=protocol[kind]
        if sha(ROOT/item['path'])!=item['sha256']:raise ValueError('Registered '+kind+' changed')
    if protocol['gain_cache']['sha256']!=FROZEN_BANK_SHA256:raise ValueError('Wrong original gain bank')
    source=protocol['source_data']
    for key in ('precision_path','provenance_path','protocol_path'):
        if source[key]not in protocol['input_sha256']:raise ValueError('Unbound original source: '+key)
    raw,provenance,original=read_source(protocol)
    if provenance['case']['case_id']!='m100_H' or provenance['case']['variant']!='handoff':
        raise ValueError('This registered diagnostic requires the original I56m100_H trace')
    if original['iteration']!='I-56' or original['parameters']['mode']!='hardware':
        raise ValueError('Unexpected original conditions')
    if not provenance['source_hashes_unchanged_after'] or not provenance['frozen_bank_loaded']:
        raise ValueError('Original source/bank condition failed')
    if original['gain_cache']['sha256']!=protocol['gain_cache']['sha256']:
        raise ValueError('The source and replay banks differ')
    for relative,expected in original['source_sha256'].items():
        if sha(ROOT/relative)!=expected:raise ValueError('Original I56 runtime/source changed: '+relative)
    if raw['time_s'].dtype!=np.float64 or not np.isfinite(raw['time_s']).all() or np.any(abs(np.diff(raw['time_s'])-DT)>1e-9):
        raise ValueError('Source clock must retain every original5ms sample')
    if not np.isfinite(np.r_[raw['exact_applied'].ravel(),raw['exact_native_applied'].ravel()]).all():
        raise ValueError('Source commands must be finite')
    if not bit_exact(raw['exact_previous_applied'][1:],raw['exact_applied'][:-1]):
        raise ValueError('Source prior-input chain does not match applied inputs')
    events={row['event_id']:row for row in protocol['events']}
    if len(events)!=len(protocol['events']):raise ValueError('Duplicate event IDs')
    cases={row['case_id']:row for row in protocol['cases']}
    if len(cases)!=len(protocol['cases']):raise ValueError('Duplicate case IDs')
    for event in events.values():
        index=event['index']
        if not isinstance(index,int)or index<0 or index+4>=len(raw['time_s']):
            raise ValueError('Event needs an exact recorded anchor and20ms continuation')
        if 'time_s'in event and abs(raw['time_s'][index]-event['time_s'])>1e-9:
            raise ValueError('Event index/time disagree')
    for job in cases.values():
        if job['event_id']not in events or job['mode']not in MODES:raise ValueError('Unknown registered event/mode')
        delta=np.asarray(job.get('delta_input',[0.]*5),float)
        if delta.shape!=(5,)or not np.isfinite(delta).all():raise ValueError('Finite delta_input[5] required')
        if job['mode']=='continue_saved'and np.any(delta):raise ValueError('Saved continuation cannot be perturbed')
        center,commands=center_and_sequence(job,events[job['event_id']],raw)
        if commands.shape!=(4,5)or np.any(abs(commands)>BOUNDS):raise ValueError('Physical amplitude caps exceeded')
        if job['mode']=='hold_governed':
            previous=raw['exact_previous_applied'][events[job['event_id']]['index']]
            if np.any(abs(commands[0]-previous)>np.array([60,60,60,6,6])*DT+1e-10):
                raise ValueError('Governed-center probe lies outside the first admissible slew box')
        if 'expected_commands'in job and not bit_exact(commands,np.asarray(job['expected_commands'],np.float64)):
            raise ValueError('Registered expected command does not match source/delta')
    if case_id is not None and case_id not in cases:raise ValueError('Choose exactly one registered case')
    audit=dict(passed=True,scope='Read-only source/data/plugin/bank/command registration; no model creation or physics.',
        protocol_sha256=sha(output/'protocol.json'),source_precision_sha256=sha(ROOT/source['precision_path']),
        verified_sources=len(protocol['source_sha256']),verified_inputs=len(protocol['input_sha256']),
        events=list(events),cases=len(cases),physical_input_bounds=BOUNDS)
    return protocol,cases.get(case_id),events,raw,provenance,original,audit


def forbidden(*unused,**ignored):raise AssertionError('Open-loop replay cannot recompute command/observer or restore a branch state')


def arrays(rows):
    if not rows:return {}
    return {key:np.asarray([row[key]for row in rows])for key in rows[0]}


def seal(directory):
    save(directory/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(directory)),bytes=p.stat().st_size,sha256=sha(p))
        for p in sorted(directory.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))


def run(output,case_id):
    protocol,job,events,raw,source_prov,original,audit=preflight(output,case_id)
    event=events[job['event_id']];anchor=event['index']
    center,commands=center_and_sequence(job,event,raw)
    directory=output/'campaign/runs'/case_id;directory.mkdir(parents=True,exist_ok=False)
    source_paths={ROOT/p for p in protocol['source_sha256']}
    source_paths.update((Path(__file__).resolve(),output/'protocol.json'))
    registered=[]
    for path in sorted(source_paths):
        target=directory/'source_snapshot'/path.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(path,target);registered.append(dict(origin=str(path.relative_to(ROOT)),sha256=sha(path)))
    provenance=dict(iteration='I-57',case=job,event=event,registered_before_execution_utc=datetime.now(timezone.utc).isoformat(),
        pid=os.getpid(),git_head=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),preflight=audit,
        source_provenance_sha256=sha(ROOT/protocol['source_data']['provenance_path']),
        sources=registered,initialization='Original workspace, gain bank, OffroadController construction and place_on_ground formula. No saved-pose branch reset.',
        stepping='Original physical.step -> one original bridge.run(5) per5ms hold. No split/substep instrumentation.',
        estimator_policy='Controller construction/reset for initialization only; command and observer update forbidden thereafter.',
        branch_commands=commands,center_command=center,actual_delta_from_center=None if center is None else commands[0]-center,
        no_hidden_state_restore=True,contacts='Raw last-solve bridge contacts at 0/5/10/15/20ms; evaluation-only. Not interval-integrated solver impulses.')
    save(directory/'provenance_before.json',provenance)
    prefix=[];endpoints=[];contact_rows=[];contact_offsets=[0];physical=None;workspace=None
    started=time.monotonic();success=False;first_mismatch=None
    try:
        import spherical_wheel.gazebo.mars as mars
        from spherical_wheel.gazebo.plant import GazeboPlant
        from spherical_wheel.gazebo.bridge import GazeboSim,plugin_path
        from spherical_wheel.offroad_control import OffroadController,OffroadParams
        from spherical_wheel.contact_sensing import MomentumObserver
        from spherical_wheel.offroad_experiments import place_on_ground
        if mars._WORKSPACE is not None or GazeboSim._servers:
            raise AssertionError('Use one entirely fresh Python process per branch')
        if sha(plugin_path())!=protocol['bridge_binary']['sha256']:raise ValueError('Resolved plugin differs')
        workspace,terrain=mars._workspace(output/'campaign/_workspaces'/case_id,[source_prov['case']['path']['route']])
        if sha(workspace.cache_path)!=protocol['gain_cache']['sha256']or workspace.design.metadata['cache_loaded']is not True:
            raise ValueError('Original frozen gain bank was rebuilt or not loaded')
        physical,mirror=workspace.plant(terrain),workspace.mirror()
        np.testing.assert_array_equal(physical.bounds,BOUNDS)
        if physical.dt!=DT or physical.substeps!=5:raise ValueError('Expected original5ms/5step hold')
        model_paths=dict(mirror_xml=workspace.directory/'mirror/model.xml',
            physical_sdf=physical.world_path.parent/'models/active_spherical_wheel/model.sdf')
        model_hashes={name:sha(path)for name,path in model_paths.items()}
        for name,path in model_paths.items():
            if model_hashes[name]!=protocol['model_sha256'][name]:raise ValueError('Generated model differs: '+name)
            target=directory/'models'/name/path.name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(path,target)
        original_job=source_prov['case'];path,yaw=mars.build_route(original_job['path']['route'])
        if yaw!=original_job['yaw_deg']:raise ValueError('Original route initialization yaw differs')
        controller=OffroadController(mirror,workspace.design,OffroadParams(**original['parameters']),original_job['phase_rad'],record_diagnostics=True)
        place_on_ground(physical,controller,path.origin,original_job['phase_rad'],True,original_job['yaw_deg'])
        initialization_bridge=physical.sim.state().copy()
        reset_count=initialization_bridge[22]
        def row():
            return dict(time_s=np.asarray(physical.data.time,dtype=np.float64),exact_qpos=physical.data.qpos.copy(),
                exact_qvel=physical.data.qvel.copy(),exact_previous_applied=physical.last_input.copy())
        def verify(index,scope):
            nonlocal first_mismatch
            actual=row();flags={key:bit_exact(actual[key],raw[key][index])for key in actual}
            item=dict(index=index,**actual,**{key+'_bit_exact':value for key,value in flags.items()})
            if scope=='prefix':prefix.append(item)
            if not all(flags.values()):
                first_mismatch=dict(scope=scope,index=index,time_s=float(physical.data.time),fields=flags,
                    max_abs_qpos_gap=float(np.max(abs(actual['exact_qpos']-raw['exact_qpos'][index]))),
                    max_abs_qvel_gap=float(np.max(abs(actual['exact_qvel']-raw['exact_qvel'][index]))))
                raise AssertionError('Original physical prefix/continuation mismatch: '+json.dumps(safe(first_mismatch)))
            if physical.sim.state()[22]!=reset_count:raise AssertionError('Unexpected reset during replay')
        def endpoint(offset):
            state=physical.sim.state();contacts=physical.sim.contacts()
            if state[22]!=reset_count or state[23]!=0:raise AssertionError('Reset during branch')
            endpoints.append(dict(offset_steps=offset,offset_s=offset*DT,**row(),bridge_state=state.copy(),
                actual_applied=physical.last_input.copy()))
            contact_rows.append(contacts.copy());contact_offsets.append(contact_offsets[-1]+len(contacts))
        with ExitStack()as stack:
            stack.enter_context(patch.object(OffroadController,'command',forbidden))
            stack.enter_context(patch.object(MomentumObserver,'update',forbidden))
            stack.enter_context(patch.object(GazeboPlant,'restore',forbidden))
            stack.enter_context(patch.object(GazeboPlant,'perturb',forbidden))
            stack.enter_context(patch.object(physical,'reset',forbidden))
            stack.enter_context(patch.object(physical.sim,'reset',forbidden))
            for index in range(anchor):
                verify(index,'prefix')
                physical.step(raw['exact_applied'][index],np.zeros(6))
                if(index+1)%1000==0:print(json.dumps(dict(case_id=case_id,prefix_holds=index+1,elapsed_s=time.monotonic()-started)),flush=True)
            verify(anchor,'prefix');prefix_wall=time.monotonic()-started
            endpoint(0)
            for offset,command in enumerate(commands,1):
                physical.step(command,np.zeros(6))
                endpoint(offset)
                if job['mode']=='continue_saved':verify(anchor+offset,'continuation')
        bank_numeric=fingerprint([{key:point[key]for key in('phase','K','eq','regime','side')}for point in workspace.design.points])
        if bank_numeric!=source_prov['fixed_bank_numeric_sha256']:raise ValueError('Numeric original gain bank differs')
        if sha(workspace.cache_path)!=protocol['gain_cache']['sha256']:raise ValueError('Frozen gain bank changed')
        for item in registered:
            if sha(ROOT/item['origin'])!=item['sha256']:raise ValueError('Executed source changed: '+item['origin'])
        previous=raw['exact_previous_applied'][anchor]
        first_slew_violation=np.maximum(abs(commands[0]-previous)-np.array([60,60,60,6,6])*DT,0.)
        provenance.update(success=True,model_sha256=model_hashes,prefix_wall_seconds=prefix_wall,total_wall_seconds=time.monotonic()-started,
            original_prefix_rows=len(prefix),original_prefix_all_fields_bit_exact=True,
            original_continuation_bit_exact=job['mode']=='continue_saved',frozen_bank_loaded=True,
            fixed_bank_numeric_sha256=bank_numeric,controller_calls_after_initialization=0,observer_updates_after_initialization=0,
            reset_count_initial=float(reset_count),reset_count_final=float(physical.sim.state()[22]),
            first_slew_violation=first_slew_violation,
            first_slew_scope='Native/midpoint amplitude-only counterfactuals may violate candidate first slew. Held column probes are required by protocol to lie in their registered first admissible box; no silent clipping.')
        success=True
    except BaseException:
        save(directory/'failure.json',dict(traceback=traceback.format_exc(),first_mismatch=first_mismatch,
            prefix_rows=len(prefix),endpoint_rows=len(endpoints),elapsed_seconds=time.monotonic()-started))
        raise
    finally:
        if prefix:np.savez_compressed(directory/'prefix.npz',**arrays(prefix))
        if endpoints:
            payload=arrays(endpoints);payload.update(command_sequence=commands,contact_offsets=np.asarray(contact_offsets,np.int64),
                contact_rows=np.concatenate(contact_rows,axis=0)if contact_rows else np.empty((0,16)))
            np.savez_compressed(directory/'endpoints.npz',**payload)
        provenance.update(success=success,prefix_rows=len(prefix),endpoint_rows=len(endpoints),first_mismatch=first_mismatch,
            elapsed_seconds=time.monotonic()-started)
        save(directory/'provenance.json',provenance);seal(directory)
        if physical is not None:physical.close()
    print(json.dumps(dict(case_id=case_id,success=success,prefix_rows=len(prefix),endpoint_rows=len(endpoints),elapsed_seconds=time.monotonic()-started)),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=DEFAULT_OUTPUT)
    parser.add_argument('--case-id')
    group=parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--preflight',action='store_true');group.add_argument('--run',action='store_true')
    args=parser.parse_args();output=args.output.resolve()
    if args.preflight:
        *_,audit=preflight(output,args.case_id);print(json.dumps(safe(audit),indent=2))
    else:
        if not args.case_id:parser.error('--run requires --case-id')
        run(output,args.case_id)


if __name__=='__main__':main()
