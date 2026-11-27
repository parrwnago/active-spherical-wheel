"""Register I74 control/fixture/metric definitions before new physics."""
from pathlib import Path
from datetime import datetime, timezone
import argparse, hashlib, json, shutil, subprocess

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
OLD=ROOT/'results/contact_reference_control_20261127'
GAIN_SHA='45ab9990aa3961d1202533acf51d5d8a9294eefe5a4c7e6cfde6aa0b3ac462c1'
HEADING=dict(heading_reference='start',heading_actuation='shells',heading_gain=.2,
             heading_damping=.06,heading_torque_limit_Nm=.15,heading_roll_budget_Nm=.05)


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--gates',type=Path,required=True)
    args=parser.parse_args();gates=args.gates.resolve()
    old_path=BASE/'runtime/fixture_stage01/historical_I73_protocol.json'
    if sha(old_path)!='30f37fabc23b76dab3caf9eca50fa9221c2cfefcae29385978303c57e550fbdf':
        raise ValueError('Historical I73 physics protocol differs')
    old=json.loads(old_path.read_text())
    old_seal_path=BASE/'runtime/fixture_stage01/historical_I73_outputs_sealed.json'
    if sha(old_seal_path)!='54da1644bfd407e955e7f08a064315eb24e4a2405c51d139c25045ad885474ba':
        raise ValueError('Historical I73 output seal differs')
    old_seal=json.loads(old_seal_path.read_text())
    destination=BASE/'protocol/protocol.json'
    if destination.exists() or (BASE/'campaign/runs').exists():raise FileExistsError('Preserve existing I74 stage')
    sources=list((ROOT/'src/spherical_wheel').rglob('*.py'))
    sources += [ROOT/'scripts/run-lqr-contact-forecast.py',ROOT/'config/model.yaml',ROOT/'gazebo/plugin/SphericalWheelBridge.cc']
    sources += list((ROOT/'gazebo/physics').glob('*.cc'))+list((ROOT/'gazebo/physics').glob('*.hh'))
    for folder in ('adapter','runtime','campaign_tools','protocol'):sources+=list((BASE/folder).glob('*.py'))
    sources=sorted(set(sources))
    if BASE/'campaign_tools/run_case.py' not in sources:raise FileNotFoundError('Runner not ready')
    for path in sources:
        relative=str(path.relative_to(ROOT))
        if relative.startswith(('src/','config/','gazebo/')) and sha(path)!=old['source_sha256'][relative]:
            raise ValueError('I73 native control/model source changed: '+relative)
    if sha(BASE/'runtime/route.py')!=old['source_sha256'][str((OLD/'runtime/route.py').relative_to(ROOT))]:
        raise ValueError('Same S route bytes required')
    map_path=BASE/'runtime/fixture_stage01/historical_selected_map_report.json'
    if sha(map_path)!=old['map_preflight']['sha256'] or not json.loads(map_path.read_text())['passed']:
        raise ValueError('Original final site check changed')
    inputs=[gates,old_path,BASE/'runtime/fixture_stage01/historical_I73_outputs_sealed.json',map_path,
            BASE/'runtime/fixture_stage01/receipt.json',BASE/'runtime/environment_stage01/receipt.json',
            BASE/'runtime/fixture_stage01/historical_ground_verification_fixture.json',
            BASE/'runtime/historical_native_stage01/receipt.json']
    for relative,digest in old['input_sha256'].items():
        if relative.startswith('assets/') or relative=='build/gazebo/libAswExactPhysics.so':
            if sha(ROOT/relative)!=digest:raise ValueError('Original terrain/runtime changed: '+relative)
            inputs.append(ROOT/relative)
    historical={}
    for phase in ('m100','p030','p090'):
        for name in ('precision.npz','summary.json','provenance.json'):
            path=BASE/'runtime/historical_native_stage01'/phase/name;inputs.append(path)
            binding=old_seal['files'][f'campaign/runs/{phase}_N/{name}']
            if sha(path)!=binding['sha256'] or path.stat().st_size!=binding['bytes']:
                raise ValueError('Copied historical native input differs: '+str(path))
        historical[f'{phase}_N']=str((BASE/'runtime/historical_native_stage01'/phase/'precision.npz').relative_to(ROOT))
    gain=ROOT/old['gain_cache']['path']
    if sha(gain)!=GAIN_SHA:raise ValueError('Original gain cache changed')
    for name,key in (('libSphericalWheelBridge.so','bridge_binary_sha256'),('libAswExactPhysics.so','exact_physics_binary_sha256')):
        if sha(ROOT/'build/gazebo'/name)!=old[key]:raise ValueError('Original simulator binary changed')
    cases=[dict(case_id=f'{phase}_{policy}',policy=policy,phase_rad=angle,yaw_deg=45.,duration_s=45.)
           for phase,angle in (('m100',-1.),('p030',.3),('p090',.9)) for policy in ('N','H')]
    value=dict(iteration='I-74',registered_before_physics_utc=datetime.now(timezone.utc).isoformat(),
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        question='Can existing bounded starting-heading hold reduce rotation without worsening native tracking, balance or saturation?',
        cases=cases,parameters=old['parameters'],heading_parameters=HEADING,
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in sources},
        input_sha256={str(p.relative_to(ROOT)):sha(p) for p in sorted(set(inputs))},
        gain_cache=old['gain_cache'],bridge_binary_sha256=old['bridge_binary_sha256'],
        exact_physics_binary_sha256=old['exact_physics_binary_sha256'],
        map_preflight=dict(report_path=str(map_path.relative_to(ROOT)),sha256=sha(map_path),required_passed=True),
        historical_native_paths=historical,route=old['route'],bounds=old['bounds'],integration=old['integration'],
        evaluation=json.loads(gates.read_text()),
        evaluation_code_policy='Definitions fixed before physics; pure evaluation source separately sealed/tested before new outcome analysis.',
        intervention='H changes only existing heading parameters. Original native LQR, inferred contacts, rolling/holding feedforward, preparation and final caps retained.',
        target='Actual initial physical Euler heading latched by original reset; fixed across the complete episode. No target switch at final hold.',
        configuration_limits='Original helper eligibility, vertical lever cutoff and error clamp retained. Torque cap .15Nm and horizontal reaction budget .05Nm; configuration-dependent authority.',
        native_reproduction='Compare common full-precision N arrays against I73 N after all six outputs seal. Preserve mismatches rather than tune or overwrite.',
        outcome_policy='One fixed H setting, six fresh episodes, all outputs sealed before analysis, no post-result tuning or winner substitution.',
        default_changed=False)
    save(destination,value)
    snapshot=BASE/'protocol/source_before_physics';snapshot.mkdir(exist_ok=False)
    for path in sources+[destination,gates]:
        target=snapshot/path.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    save(BASE/'protocol/seal_before_physics.json',dict(protocol_sha256=sha(destination),source_count=len(sources),
         input_count=len(set(inputs)),no_physics=True,captured_utc=datetime.now(timezone.utc).isoformat()))
    print(json.dumps(dict(protocol_sha256=sha(destination),source_count=len(sources),input_count=len(set(inputs)),cases=len(cases))))


if __name__=='__main__':main()
