"""Freeze I76 sources, eighteen cases and gates before any physics."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,importlib.util,json,shutil,subprocess
ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True,allow_nan=False)+'\n')
def module(p,name):
    spec=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--gates',type=Path,required=True);args=parser.parse_args()
    dest=BASE/'protocol/protocol.json'
    if dest.exists() or (BASE/'campaign/runs').exists():raise FileExistsError('Preserve previous I76 stage')
    old_path=BASE/'runtime/fixture_stage01/historical_I75_protocol.json';old=json.loads(old_path.read_text())
    if sha(old_path)!='d579e1a21890b2da48dbd5c664ddbb692a1bb5c67f112262fbdf2b95500671ee':raise ValueError('Historical I75 changed')
    runner=module(BASE/'campaign_tools/run_case.py','_i76_register_runner');routes=runner.route_module()
    sources=list((ROOT/'src/spherical_wheel').rglob('*.py'))
    sources += [ROOT/'scripts/run-lqr-contact-forecast.py',ROOT/'config/model.yaml',ROOT/'gazebo/plugin/SphericalWheelBridge.cc']
    sources += list((ROOT/'gazebo/physics').glob('*.cc'))+list((ROOT/'gazebo/physics').glob('*.hh'))
    for folder in ('adapter','runtime','campaign_tools','protocol','runtime/fixture_stage01','runtime/environment_stage01'):
        sources += list((BASE/folder).glob('*.py'))
    sources=sorted(set(sources))
    for p in sources:
        rel=str(p.relative_to(ROOT))
        if rel.startswith(('src/','config/','gazebo/')) and sha(p)!=old['source_sha256'][rel]:raise ValueError('Native source changed:'+rel)
    old_adapter=ROOT/'results/tangent_heading_control_20261127/adapter/tangent_heading_adapter.py'
    if sha(BASE/'adapter/tangent_heading_adapter.py')!=sha(old_adapter):raise ValueError('I75 helper law changed')
    reports=[BASE/'runtime/reference_stage02/report.json',BASE/'runtime/fixture_stage01/map_stage02/report.json',BASE/'campaign_tools/contracts_stage04/receipt.json',BASE/'runtime/fixture_stage01/stress_decision_stage04.json']
    if not all(json.loads(reports[i].read_text())['passed'] for i in (0,2)):raise ValueError('Required reference/runner checks did not pass')
    decision=json.loads(reports[3].read_text())
    if not decision['decision_before_physics'] or decision['physics_steps_at_decision']!=0 or decision['control_acceptance_gates_changed'] or decision['controller_gains_or_limits_changed'] or decision['eight_recorded_geometric_result']['moderate_screen_passed'] or decision['finite_scan_selection']['selected_index_1based'] is not None:raise ValueError('Explicit truthful stress-domain decision missing')
    inputs=[args.gates.resolve(),old_path,BASE/'protocol/reference_stage01.json',BASE/'protocol/map_stage01.json',BASE/'protocol/reference_stage02.json',BASE/'protocol/map_stage02.json',BASE/'protocol/map_scan_stage03.json',*reports]
    for folder in ('runtime/environment_stage01','runtime/fixture_stage01','runtime/reference_stage01','runtime/reference_stage02','runtime/fixture_stage01/map_stage01','runtime/fixture_stage01/map_stage02','runtime/fixture_stage01/map_scan_stage03'):
        inputs+=list((BASE/folder).glob('*.json'))
    for route_id in routes.ROUTE_IDS:inputs+=list((BASE/'runtime/fixture_stage01/map_stage02'/route_id).glob('*.json'))
    for rel,digest in old['input_sha256'].items():
        if rel.startswith('assets/') or rel=='build/gazebo/libAswExactPhysics.so':
            if sha(ROOT/rel)!=digest:raise ValueError('Original asset/runtime changed:'+rel)
            inputs.append(ROOT/rel)
    gain=ROOT/old['gain_cache']['path']
    if sha(gain)!=old['gain_cache']['sha256']:raise ValueError('Frozen gains changed')
    for name,key in [('libSphericalWheelBridge.so','bridge_binary_sha256'),('libAswExactPhysics.so','exact_physics_binary_sha256')]:
        if sha(ROOT/'build/gazebo'/name)!=old[key]:raise ValueError('Runtime changed')
    cases=runner.registered_cases();gate=json.loads(args.gates.read_text())
    map_path=reports[1];reference_path=reports[0]
    value=dict(iteration='I-76',registered_before_physics_utc=datetime.now(timezone.utc).isoformat(),
        base_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        question='Does the unchanged I75 trajectory-facing heading overlay help across straight, quarter-circle and figure-eight routes at allthree starting phases?',
        cases=cases,route_ids=list(routes.ROUTE_IDS),route_specs={r:routes.make_route(r).spec() for r in routes.ROUTE_IDS},
        parameters=runner.BASELINE_SETTINGS,heading_parameters=runner.HEADING_SETTINGS,
        heading_reference_parameters=runner.REFERENCE_SETTINGS,heading_rate_projection_min_norm2=1e-8,
        initial_yaw_deg_by_route={r:routes.make_route(r).heading_deg for r in routes.ROUTE_IDS},
        source_sha256={str(p.relative_to(ROOT)):sha(p) for p in sources},
        input_sha256={str(p.relative_to(ROOT)):sha(p) for p in sorted(set(inputs))},
        gain_cache=old['gain_cache'],bridge_binary_sha256=old['bridge_binary_sha256'],exact_physics_binary_sha256=old['exact_physics_binary_sha256'],
        map_preflight=dict(report_path=str(map_path.relative_to(ROOT)),sha256=sha(map_path),required_passed=False,fixture_categories=runner.FIXTURE_CATEGORIES,allow_outside_moderate_screen_routes=['eight'],domain_decision_path=str(reports[3].relative_to(ROOT)),domain_decision_sha256=sha(reports[3])),
        reference_preflight=dict(report_path=str(reference_path.relative_to(ROOT)),sha256=sha(reference_path),required_passed=True),
        bounds=old['bounds'],integration=old['integration'],evaluation=gate,
        evaluation_code_policy='Fixed definitions before physics; separately sealed/tested pure evaluator before reading new arrays.',
        intervention=old['intervention'],lqr_rate_reference_policy=old['lqr_rate_reference_policy'],
        target_policy='Allroutes analytic planned tangent and exactEuler derivative, raw=command, geometricendpoint targets retained; no measured-drift target or shaping.',
        initialization='Same route/phase N/T initial q/v/previousappliedinput exact, yawargument=geometricinitialtangent; actualinitialerror<=1degree. No oldS replay gate.',
        configuration_limits=old['configuration_limits'],
        outcome_policy='Eighteen fresh episodes, all sealed before scoring. Allnine paired screens required; no result-dependent route/phase/site/setting selection or defaultadoption.',
        route_comparability='Shared siteanchor/rotation and phases, but route shapes,lengths,durations and starting terrain differ. Eight retains stage02 localoffset[.5,0] as an explicit rough_stress case outside unchanged80mm/12deg moderate screens. Allsix finite scan candidates failed (selectionnull); no winning placement. Domain revised beforeanyphysics. Figureeight motion120s tokeep same referenceguards; starts/ends crossing. Report paired method effects withinroute; no claim of equal difficulty.',
        default_changed=False)
    runner.validate_map_report(value,json.loads(map_path.read_text()))
    for job in cases:runner.validate_protocol(value,job['case_id'])
    save(dest,value)
    snap=BASE/'protocol/source_before_physics';snap.mkdir(exist_ok=False)
    for p in sources+[dest,args.gates.resolve()]:
        q=snap/p.relative_to(ROOT);q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
    seal=dict(protocol_sha256=sha(dest),source_count=len(sources),input_count=len(set(inputs)),case_count=len(cases),no_physics=True)
    save(BASE/'protocol/seal_before_physics.json',seal);print(json.dumps(seal))
if __name__=='__main__':main()
