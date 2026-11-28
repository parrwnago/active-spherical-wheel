"""I78 one registered native or governed straight Gazebo episode.

The I76 run_case wrapper is imported unchanged. Its guarded native controller,
precision/truth logger, fixed model/gain/terrain and initialization remain in
use. A result-owned copied loop selects coherent references after measurement
and stops after the explicit progress/hold rule. I78 metadata is authoritative.
"""
from contextlib import ExitStack
from datetime import datetime,timezone
from pathlib import Path
from unittest.mock import patch
import argparse
import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys
import traceback
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
BASE=HERE.parent
OLD=ROOT/'results/tangent_heading_routes_20261127'
sys.path.insert(0,str(HERE))
import run_loop
from session import GovernorSession
from spherical_wheel.offroad_control import OffroadController

# Capture before the original parent installs its guarded experiment wrapper.
NATIVE_COMMAND=OffroadController.command

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def safe(value):
    if isinstance(value,dict):return{str(k):safe(v)for k,v in value.items()}
    if isinstance(value,(tuple,list)):return[safe(v)for v in value]
    if isinstance(value,Path):return str(value)
    if isinstance(value,np.ndarray):return safe(value.tolist())
    if isinstance(value,np.generic):return safe(value.item())
    if isinstance(value,float)and not np.isfinite(value):return None
    return value
def save(path,value):
    with Path(path).open('x')as f:json.dump(safe(value),f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec)
    sys.modules[name]=m;spec.loader.exec_module(m);return m
def token(value):
    value=np.asarray(value)
    return value.dtype.str,value.shape,np.ascontiguousarray(value).tobytes()

def preflight(protocol_path,case_id):
    protocol=json.loads(protocol_path.read_text())
    if protocol['iteration']!='I-78' or protocol['status']!='registered_before_physics':
        raise ValueError('Frozen I78 protocol required')
    expected=dict(mode='hardware',rolling_allocation='least_squares',track_contact_point=False)
    if protocol['parameters']!=expected:raise ValueError('Selected native parameters changed')
    matches=[j for j in protocol['cases']if j['case_id']==case_id]
    if len(matches)!=1:raise ValueError('Exactly one registered case required')
    job=matches[0]
    if job['policy']not in ('N','G')or job['route_id']!='straight' or job['yaw_deg']!=113.:
        raise ValueError('Only the fixed straight N/G comparison is registered')
    if job['maximum_duration_s']!=(45. if job['policy']=='N'else 75.)or job['goal_hold_s']!=10.:
        raise ValueError('Maximum budget or terminal hold changed')
    parent_id=case_id[:-1]+'N'
    if parent_id not in ('straight_m100_N','straight_p030_N','straight_p090_N'):
        raise ValueError('Unregistered initial phase')
    phase_by_id=dict(straight_m100_N=-1.,straight_p030_N=.3,straight_p090_N=.9)
    if job['phase_rad']!=phase_by_id[parent_id]:raise ValueError('Case/phase mismatch')
    for group in ('source_sha256','input_sha256'):
        for rel,bound in protocol[group].items():
            if sha(ROOT/rel)!=bound:raise ValueError('Registered source/input differs: '+rel)
    original=ROOT/protocol['original_protocol_path']
    if original!=OLD/'protocol/protocol.json' or sha(original)!=protocol['original_protocol_sha256']:
        raise ValueError('Original I76 protocol differs')
    parent=module(OLD/'campaign_tools/run_case.py','_i78_original_i76_runner')
    parent.preflight(OLD,parent_id)
    baseline=OLD/'campaign/runs'/parent_id/'precision.npz'
    seal_path=OLD/'campaign/outputs_sealed.json';seal=json.loads(seal_path.read_text())
    bound=seal['files'][str(baseline.relative_to(OLD))]
    if sha(baseline)!=bound['sha256']or baseline.stat().st_size!=bound['bytes']:
        raise ValueError('Original sealed native recording differs')
    return protocol,job,parent,parent_id,baseline

def verify_precision(produced,baseline,policy):
    with np.load(produced,allow_pickle=False)as new,np.load(baseline,allow_pickle=False)as old:
        if set(new.files)!=set(old.files):raise AssertionError('Original79 precision fields changed')
        if policy=='N':
            checks={key:token(new[key])==token(old[key])for key in old.files}
            if len(new['time_s'])!=9001 or not all(checks.values()):
                raise AssertionError('Native45s does not reproduce every original precision field')
            return dict(passed=True,policy=policy,fields=checks,rows=9001,
                        all_original_arrays_bit_exact=True,terminal_mask_exception=False)
        n=min(len(new['time_s']),len(old['time_s']))
        np.testing.assert_array_equal(new['time_s'][:n],old['time_s'][:n])
        changed=np.zeros(n,bool)
        for key in ('exact_reference_xy','exact_reference_velocity_m_s','exact_reference_acceleration_m_s2'):
            changed|=np.any(new[key][:n]!=old[key][:n],axis=1)
        first=int(np.flatnonzero(changed)[0])if changed.any()else n
        checks={};mask_exception=False
        for key in old.files:
            a=new[key][:first].copy();b=old[key][:first].copy()
            # If references never change, a longer governed horizon can have a
            # held command at the original N endpoint. That mask is explicit.
            if key=='command_held_after_row' and first:
                if first==len(new['time_s']) and len(new['time_s'])<len(old['time_s']):
                    b[-1]=False;mask_exception=True
                elif first==len(old['time_s']) and len(new['time_s'])>first:
                    b[-1]=a[-1];mask_exception=True
            checks[key]=token(a)==token(b)
        if not all(checks.values()):raise AssertionError('Governed prefix differs before reference intervention')
        first_inputs={}
        if first<n:
            for key in ('exact_qpos','exact_qvel','exact_previous_applied','exact_nominal_joints_before',
                        'exact_observer_wrench','exact_observer_generalized','exact_observer_interval_mean',
                        'exact_estimated_normal','exact_feedback_gain','exact_support_command'):
                first_inputs[key]=token(new[key][first])==token(old[key][first])
            if not all(first_inputs.values()):raise AssertionError('First changed-reference state/stages differ')
        return dict(passed=True,policy=policy,original_prefix_fields=checks,
            exact_complete_command_prefix_rows=first,first_changed_reference_row=first if first<n else None,
            inclusive_first_changed_reference_inputs=first_inputs,
            terminal_mask_exception=mask_exception,
            scope='Observed original precision prefix, not an independent dump of hidden solver state.')

def run(protocol_path,case_id,out):
    protocol,job,parent,parent_id,baseline=preflight(protocol_path,case_id)
    out.mkdir(parents=True,exist_ok=False)
    sources={ROOT/rel for rel in protocol['source_sha256']}
    sources.update([Path(__file__).resolve(),protocol_path.resolve(),OLD/'campaign_tools/run_case.py'])
    for source in sources:
        target=out/'source'/source.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    registration=dict(iteration='I-78',utc=datetime.now(timezone.utc).isoformat(),
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        case=job,parent_case_id=parent_id,protocol_sha256=sha(protocol_path),
        sources={str(p.relative_to(ROOT)):sha(p)for p in sorted(sources)},
        inputs={str(baseline.relative_to(ROOT)):dict(bytes=baseline.stat().st_size,sha256=sha(baseline))},
        no_case_arrays_loaded_before_registration=True,default_changed=False,
        counterfactual='G modifies only coherent path references. One private current-state nominal fullspeed command probe; no model integration, contact oracle, tilt shaping, gain change or finalcommand slew modification.',
        inherited_metadata_override='Child I76 labels identify inherited source/initialization. I78 outcome/protocol define governed references, maximum budget, dynamic requested stop and completion.')
    save(out/'registered_before_arrays_and_physics.json',registration)
    child=out/'child'
    for folder,names in dict(adapter=['tangent_heading_adapter.py'],runtime=['route.py','heading_reference.py'],protocol=['protocol.json']).items():
        (child/folder).mkdir(parents=True)
        for name in names:
            source=OLD/folder/name;target=child/folder/name;shutil.copy2(source,target)
            if sha(source)!=sha(target):raise AssertionError('Copied parent source differs')
    session=GovernorSession(job['policy'],NATIVE_COMMAND,protocol['governor_settings'])
    if session.requested_horizon_s!=job['maximum_duration_s']:raise AssertionError('Session/protocol horizon differs')
    import spherical_wheel.gazebo.experiments as gx
    success=False
    try:
        with patch.object(run_loop,'ACTIVE_SESSION',session),patch.object(gx,'run_offroad',run_loop.run_offroad):
            parent.run_case(child,parent_id)
        success=True
    except BaseException:
        save(out/'failure.json',dict(traceback=traceback.format_exc(),confirmed_governor_rows=len(session.rows),
            reference_selected_without_confirmed_live_command=session.pending is not None))
        raise
    finally:
        if session.rows:
            if session.pending is None:
                arrays=session.arrays()
            else:
                arrays={key:np.asarray([r[key]for r in session.rows])for key in session.rows[0]}
                arrays['command_held_after_row']=np.arange(len(session.rows))<len(session.rows)-1
            np.savez_compressed(out/('governor_diagnostics.npz'if success else'partial_governor_diagnostics.npz'),**arrays)
        for rel,bound in registration['sources'].items():
            if sha(ROOT/rel)!=bound:raise AssertionError('Frozen source changed during episode: '+rel)
    case=child/'campaign/runs'/parent_id
    summary=json.loads((case/'summary.json').read_text())
    outcome=session.outcome(summary);outcome.update(case=job,
        parent_case_directory=str(case.relative_to(out)),protocol_sha256=sha(protocol_path),
        source_hashes_unchanged_after=True)
    save(out/'i78_outcome.json',outcome)
    check=verify_precision(case/'precision.npz',baseline,job['policy'])
    save(out/'precision_prefix_receipt.json',check)
    with np.load(case/'precision.npz',allow_pickle=False)as precision:
        arrays=session.arrays();np.testing.assert_array_equal(arrays['time_s'],precision['time_s'])
        np.testing.assert_array_equal(arrays['governed_xy'],precision['exact_reference_xy'])
        np.testing.assert_array_equal(arrays['governed_velocity_m_s'],precision['exact_reference_velocity_m_s'])
        np.testing.assert_array_equal(arrays['governed_acceleration_m_s2'],precision['exact_reference_acceleration_m_s2'])
        expected=0 if job['policy']=='N'else 1
        if not np.all(arrays['private_observer_updates']==expected):raise AssertionError('Private probe observer count differs')
        if np.any(arrays['private_dynamic_steps']):raise AssertionError('Private reference probe integrated physics')
    preflight(protocol_path,case_id)
    save(out/'artifact_manifest.json',dict(iteration='I-78',files={str(p.relative_to(out)):
        dict(bytes=p.stat().st_size,sha256=sha(p))for p in sorted(out.rglob('*'))
        if p.is_file()and'__pycache__'not in p.parts}))
    print(json.dumps(safe(dict(case_id=case_id,run_returned=True,diagnostic_completed=outcome['diagnostic_completed'],
        survived=outcome['survived_without_native_failure'],actual_end_s=outcome['actual_end_s'],
        reference_complete=outcome['reference_complete'],goal_hold_completed=outcome['full_goal_hold_completed'],
        precision_prefix_passed=True))),flush=True)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',type=Path,required=True);parser.add_argument('--case',required=True)
    parser.add_argument('--output',type=Path);parser.add_argument('--run',action='store_true')
    parser.add_argument('--preflight-only',action='store_true');args=parser.parse_args()
    if args.preflight_only:
        _,job,_,_,_=preflight(args.protocol.resolve(),args.case);print(json.dumps(dict(passed=True,case=job,physics_run=False)));return
    if not args.run or args.output is None:raise SystemExit('Explicit --run and fresh --output are required')
    run(args.protocol.resolve(),args.case,args.output.resolve())

if __name__=='__main__':main()
