"""Freeze I75 before physics; no plant or outcome calculation."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil,subprocess

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
YAW=149.2428430702093
HEADING=dict(heading_reference='start',heading_actuation='shells',heading_gain=.2,
             heading_damping=.06,heading_torque_limit_Nm=.15,heading_roll_budget_Nm=.05)
REFERENCE=dict(max_rate_rad_s=.35,max_acceleration_rad_s2=.5)


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--gates',type=Path,required=True)
    args=parser.parse_args();gates=args.gates.resolve()
    destination=BASE/'protocol/protocol.json'
    if destination.exists() or (BASE/'campaign/runs').exists():raise FileExistsError('Preserve earlier I75 stage')
    old_path=ROOT/'results/heading_hold_control_20261127/protocol/protocol.json'
    old=json.loads(old_path.read_text())
    if sha(old_path)!='aa3d290f3278f515ffdc47042ae8725c69166f28cecf819d26f2541eac4e82a8':
        raise ValueError('I74 control registration changed')
    # Copy the historical registration for portable provenance, not replaying its different-yaw trials.
    old_copy=BASE/'runtime/fixture_stage01/historical_I74_protocol.json'
    if old_copy.exists() and sha(old_copy)!=sha(old_path):raise ValueError('Historical protocol copy conflict')
    if not old_copy.exists():shutil.copy2(old_path,old_copy)
    sources=list((ROOT/'src/spherical_wheel').rglob('*.py'))
    sources += [ROOT/'scripts/run-lqr-contact-forecast.py',ROOT/'config/model.yaml',ROOT/'gazebo/plugin/SphericalWheelBridge.cc']
    sources += list((ROOT/'gazebo/physics').glob('*.cc'))+list((ROOT/'gazebo/physics').glob('*.hh'))
    for folder in ('adapter','runtime','campaign_tools','protocol'):sources+=list((BASE/folder).glob('*.py'))
    sources=sorted(set(sources))
    for required in ('campaign_tools/run_case.py','adapter/tangent_heading_adapter.py','runtime/heading_reference.py'):
        if BASE/required not in sources:raise FileNotFoundError('Required experiment source missing: '+required)
    for path in sources:
        relative=str(path.relative_to(ROOT))
        if relative.startswith(('src/','config/','gazebo/')) and sha(path)!=old['source_sha256'][relative]:
            raise ValueError('Native controller/model source changed: '+relative)
    expected_route='f740cb028cc883ff948ab080390e7a45c580c8f568c55248661556d338222dfd'
    if sha(BASE/'runtime/route.py')!=expected_route:raise ValueError('Same S route bytes required')
    reports=[BASE/'runtime/reference_stage02/report.json',BASE/'runtime/fixture_stage01/historical_selected_map_report.json']
    if not all(json.loads(p.read_text())['passed'] for p in reports):raise ValueError('Reference/fixture screen did not pass')
    map_path=reports[1]
    if sha(map_path)!=old['map_preflight']['sha256']:raise ValueError('Known map report changed')
    inputs=[gates,old_copy,BASE/'protocol/reference_stage01.json',*reports]
    for folder in ('fixture_stage01','environment_stage01','reference_stage02'):
        inputs+=list((BASE/'runtime'/folder).glob('*.json'))
    for relative,digest in old['input_sha256'].items():
        if relative.startswith('assets/') or relative=='build/gazebo/libAswExactPhysics.so':
            if sha(ROOT/relative)!=digest:raise ValueError('Original terrain/runtime changed: '+relative)
            inputs.append(ROOT/relative)
    gain=ROOT/old['gain_cache']['path']
    if sha(gain)!=old['gain_cache']['sha256']:raise ValueError('Original gain bytes changed')
    for name,key in (('libSphericalWheelBridge.so','bridge_binary_sha256'),('libAswExactPhysics.so','exact_physics_binary_sha256')):
        if sha(ROOT/'build/gazebo'/name)!=old[key]:raise ValueError('Original runtime changed')
    cases=[dict(case_id=f'{phase}_{policy}',policy=policy,phase_rad=angle,yaw_deg=YAW,duration_s=45.)
           for phase,angle in (('m100',-1.),('p030',.3),('p090',.9)) for policy in ('N','T')]
    value=dict(iteration='I-75',registered_before_physics_utc=datetime.now(timezone.utc).isoformat(),
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        question='Does planned trajectory-facing heading improve pointing/rate tracking without worse native motion or actuator demands?',
        cases=cases,parameters=old['parameters'],heading_parameters=HEADING,
        heading_reference_parameters=REFERENCE,initial_yaw_deg=YAW,heading_rate_projection_min_norm2=1e-8,
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in sources},
        input_sha256={str(p.relative_to(ROOT)):sha(p) for p in sorted(set(inputs))},
        gain_cache=old['gain_cache'],bridge_binary_sha256=old['bridge_binary_sha256'],
        exact_physics_binary_sha256=old['exact_physics_binary_sha256'],
        map_preflight=dict(report_path=str(map_path.relative_to(ROOT)),sha256=sha(map_path),required_passed=True),
        reference_preflight=dict(report_path=str(reports[0].relative_to(ROOT)),sha256=sha(reports[0]),required_passed=True),
        route=old['route'],bounds=old['bounds'],integration=old['integration'],evaluation=json.loads(gates.read_text()),
        evaluation_code_policy='Definitions fixed before physics; pure evaluator separately sealed/tested before new outcome analysis.',
        intervention='Result-owned T changes heading target and derivative feedback to exact Euler heading-rate error. Native LQR/observer/contacts/rolling/support/preparation/caps unchanged.',
        lqr_rate_reference_policy='Original LQR angular-velocity reference remains zero; original rolling allocation handles translation only. This is a heading-helper overlay, not a coordinated planar-pose reference/allocator. Existing rate feedback may oppose intended turns and its contribution is logged via K/error.',
        target_policy='Analytic planned geometric tangent and its Euler heading derivative; raw=command. Hold endpoint direction at rest. No measured-drift heading target or hidden shaping.',
        initialization='Both N/T yaw arguments equal initial geometric tangent. Same-phase initial q/v required, actual initial raw pointing error <=1degree. New initialcondition vs earlier45degree trials; no numerical oldnative replay gate.',
        configuration_limits='Existing free-shell/lever/error-clamp/torque/roll-budget policy; projected-axis norm2<=1e-8 skips T helper with explicit invalid-rate flag.',
        outcome_policy='One fixed variant, six fresh episodes, all outputs seal before evaluation; no outcome-dependent tuning, candidate substitution or mechanical-holonomy claim.',
        default_changed=False)
    save(destination,value)
    snapshot=BASE/'protocol/source_before_physics';snapshot.mkdir(exist_ok=False)
    for path in sources+[destination,gates]:
        target=snapshot/path.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    save(BASE/'protocol/seal_before_physics.json',dict(protocol_sha256=sha(destination),source_count=len(sources),
         input_count=len(set(inputs)),no_physics=True,captured_utc=datetime.now(timezone.utc).isoformat()))
    print(json.dumps(dict(protocol_sha256=sha(destination),source_count=len(sources),input_count=len(set(inputs)),cases=len(cases))))


if __name__=='__main__':main()
