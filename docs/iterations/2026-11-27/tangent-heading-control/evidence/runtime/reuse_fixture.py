"""Bind I75 to the exact I73 route/assets without new map or physics work."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib, json, shutil

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
OLD=ROOT/'results/contact_reference_control_20261127'
OLD_PROTOCOL_SHA='30f37fabc23b76dab3caf9eca50fa9221c2cfefcae29385978303c57e550fbdf'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n')


def main():
    stage=BASE/'runtime/fixture_stage01';stage.mkdir(parents=True,exist_ok=False)
    protocol_path=OLD/'protocol/protocol.json'
    if sha(protocol_path)!=OLD_PROTOCOL_SHA:raise ValueError('Historical I73 protocol changed')
    old=json.loads(protocol_path.read_text())
    yard=ROOT/'assets/gazebo/third_party/leo_simulator-ros2/leo_gz_worlds/models/marsyard2021_terrain'
    files=[Path(__file__).resolve(),BASE/'runtime/route.py',OLD/'runtime/route.py',protocol_path,
           OLD/'runtime/map_stage04/report.json',OLD/'campaign/runs/m100_N/runtime_before_physics.json',
           ROOT/'environment/gazebo.yml',ROOT/'config/model.yaml',
           ROOT/'build/gazebo/libSphericalWheelBridge.so',ROOT/'build/gazebo/libAswExactPhysics.so',
           ROOT/'generated/offroad/flat/control_schedule.npz',yard/'dem/marsyard_terrain_hm.tif',
           *[yard/'meshes'/f'{n}.obj' for n in ('lava','rocks_big','rocks_medium')]]
    bindings={str(p.relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size) for p in files}
    save(stage/'registered_before_checks.json',dict(registered_utc=datetime.now(timezone.utc).isoformat(),
         files=bindings,no_new_physics=True,no_new_map_arithmetic=True,no_new_outcome_queries=True))
    for p in (Path(__file__).resolve(),BASE/'runtime/route.py'):
        target=stage/'source'/p.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
    route_key=str((OLD/'runtime/route.py').relative_to(ROOT))
    route_sha=old['source_sha256'][route_key]
    if sha(BASE/'runtime/route.py')!=route_sha or sha(OLD/'runtime/route.py')!=route_sha:
        raise ValueError('I75 must reuse exact I73 route source bytes')
    for relative,digest in old['input_sha256'].items():
        if any(str(p.relative_to(ROOT))==relative for p in files) and sha(ROOT/relative)!=digest:
            raise ValueError('Executed fixture input changed: '+relative)
    if sha(ROOT/old['gain_cache']['path'])!=old['gain_cache']['sha256']:raise ValueError('Gain bank changed')
    if sha(ROOT/'build/gazebo/libSphericalWheelBridge.so')!=old['bridge_binary_sha256']:raise ValueError('Bridge changed')
    if sha(ROOT/'build/gazebo/libAswExactPhysics.so')!=old['exact_physics_binary_sha256']:raise ValueError('Physics plugin changed')
    runtime=json.loads((OLD/'campaign/runs/m100_N/runtime_before_physics.json').read_text())
    report=json.loads((OLD/'runtime/map_stage04/report.json').read_text())
    save(stage/'historical_ground_verification_fixture.json',runtime)
    save(stage/'historical_selected_map_report.json',report)
    save(stage/'receipt.json',dict(iteration='I-75',passed=True,no_new_physics=True,
         no_new_map_arithmetic=True,no_new_case_outcome_arrays_read=True,
         historical_protocol_sha256=OLD_PROTOCOL_SHA,route_sha256=route_sha,
         route_is_original_I73_source_byte_exact=True,
         map_screen_reused='I73 stage04: 12 degree maximum sampled facet, 80 mm corridor relief, .25 m projected feature clearance.',
         fixture_classification='Moderate faceted Mars ground; unchanged from I73, not a new site search.',
         historical_ground_fixture=runtime,source_and_inputs_unchanged=all(sha(ROOT/k)==v['sha256'] for k,v in bindings.items())))
    print(json.dumps(dict(passed=True,route_sha256=route_sha,files=len(files),no_physics=True),indent=2))


if __name__=='__main__':main()
