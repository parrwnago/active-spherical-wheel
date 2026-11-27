"""Freeze I73 before physics. Does not construct a plant or inspect outcomes."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+'\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--map-report', type=Path, required=True)
    parser.add_argument('--gates', type=Path, required=True)
    args = parser.parse_args()
    report_path = args.map_report.resolve()
    map_report = json.loads(report_path.read_text())
    if not map_report['passed']:
        raise ValueError('Selected route must pass the registered final site check')
    destination = BASE/'protocol/protocol.json'
    if destination.exists() or (BASE/'campaign/runs').exists():
        raise FileExistsError('Preserve the earlier registration/campaign')
    sources = list((ROOT/'src/spherical_wheel').rglob('*.py'))
    sources += [ROOT/'scripts/run-lqr-contact-forecast.py', ROOT/'config/model.yaml',
                ROOT/'gazebo/plugin/SphericalWheelBridge.cc']
    sources += list((ROOT/'gazebo/physics').glob('*.cc'))
    sources += list((ROOT/'gazebo/physics').glob('*.hh'))
    for folder in ('adapter', 'runtime', 'campaign_tools', 'protocol'):
        sources += list((BASE/folder).glob('*.py'))
    sources = sorted(set(sources))
    required = [BASE/'campaign_tools/run_case.py', BASE/'adapter/contact_reference_adapter.py',
                BASE/'runtime/route.py']
    if not all(path in sources for path in required):
        raise FileNotFoundError('Required registered implementation missing')
    yard = ROOT/'assets/gazebo/third_party/leo_simulator-ros2/leo_gz_worlds/models/marsyard2021_terrain'
    inputs = [yard/'dem/marsyard_terrain_hm.tif',
              *[yard/'meshes'/f'{name}.obj' for name in ('lava','rocks_big','rocks_medium')],
              ROOT/'build/gazebo/libAswExactPhysics.so',
              args.gates.resolve(), report_path]
    inputs += sorted((BASE/'protocol').glob('map_stage*.json'))
    inputs += sorted((BASE/'runtime').glob('map_stage*/report.json'))
    gain = ROOT/'generated/offroad/flat/control_schedule.npz'
    gain_hash = sha(gain)
    if gain_hash != '45ab9990aa3961d1202533acf51d5d8a9294eefe5a4c7e6cfde6aa0b3ac462c1':
        raise ValueError('Original flat gain bytes differ')
    cases = [dict(case_id=f'{name}_{policy}',policy=policy,phase_rad=phase,
                  yaw_deg=45.,duration_s=45.)
             for name, phase in (('m100',-1.),('p030',.3),('p090',.9))
             for policy in ('N','A','C')]
    protocol = dict(iteration='I-73',registered_before_physics_utc=datetime.now(timezone.utc).isoformat(),
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        question='Does a frozen accepted contact point/family intervention in rolling references improve a fresh S and stop?',
        cases=cases,parameters=dict(mode='hardware',rolling_allocation='least_squares',track_contact_point=False),
        source_sha256={str(path.relative_to(ROOT)):sha(path) for path in sources},
        input_sha256={str(path.relative_to(ROOT)):sha(path) for path in sorted(set(inputs))},
        gain_cache=dict(path=str(gain.relative_to(ROOT)),sha256=gain_hash),
        bridge_binary_sha256=sha(ROOT/'build/gazebo/libSphericalWheelBridge.so'),
        exact_physics_binary_sha256=sha(ROOT/'build/gazebo/libAswExactPhysics.so'),
        map_preflight=dict(report_path=str(report_path.relative_to(ROOT)),sha256=sha(report_path),required_passed=True),
        route=map_report['route'],evaluation=json.loads(args.gates.read_text()),
        evaluation_code_policy='Acceptance definitions frozen before physics. Pure evaluation implementation is separately sealed and contract-tested before any new campaign outcome analysis; it cannot affect control.',
        integration=dict(control_interval_s=.005,physics_step_s=.001,final_state_included=True,
                         terminal_unheld_command_excluded_from_command_statistics=True),
        bounds=[1.5,.6,.6,.3,.3],
        fixed_policies=dict(N='Original native inferred records.',A='Frozen I60 accepted fused physical point/family; exact native fallback.',
                            C='Frozen I72 geometry-first fused physical point/family; exact native fallback.'),
        intervention='Only rolling-reference record argument; native observer force/normal/load/gain/support/preparation and final limits retained.',
        stationary_policy='Native zero-velocity branch bypasses rolling selection; final hold tests arrival/history effects.',
        exposure_scope='Same-state native rolling vref/FF counterfactual only. No full post-preparation/clipping native command counterfactual.',
        outcome_policy='Seal all nine outputs before campaign evaluation. Keep setup errors and physical failures. No result-dependent retuning or alternative winner substitution.',
        default_changed=False)
    save(destination,protocol)
    snapshot = BASE/'protocol/source_before_physics'
    snapshot.mkdir(exist_ok=False)
    for path in sources+[destination,args.gates.resolve()]:
        target = snapshot/path.relative_to(ROOT)
        target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(path,target)
    save(BASE/'protocol/seal_before_physics.json',dict(protocol_sha256=sha(destination),
         source_count=len(sources),input_count=len(set(inputs)),no_physics=True,
         captured_utc=datetime.now(timezone.utc).isoformat()))
    print(json.dumps(dict(protocol_sha256=sha(destination),source_count=len(sources),
                         input_count=len(set(inputs)),cases=len(cases)),indent=2))


if __name__ == '__main__':
    main()
