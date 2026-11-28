"""I76 byte-only reuse of I75 assets; not a certificate for new route corridors."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import shutil

ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parents[2]
STAGE=Path(__file__).resolve().parent
OLD=ROOT/'results/tangent_heading_control_20261127'
OLD_PROTOCOL_SHA256='d579e1a21890b2da48dbd5c664ddbb692a1bb5c67f112262fbdf2b95500671ee'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path,value):
    with path.open('x') as stream:json.dump(value,stream,indent=2,sort_keys=True,allow_nan=False);stream.write('\n')


def main():
    old_protocol=OLD/'protocol/protocol.json'
    if sha(old_protocol)!=OLD_PROTOCOL_SHA256:raise ValueError('Historical I75 protocol changed')
    protocol=json.loads(old_protocol.read_text())
    yard=ROOT/'assets/gazebo/third_party/leo_simulator-ros2/leo_gz_worlds/models/marsyard2021_terrain'
    historical=OLD/'runtime/fixture_stage01'
    files=[Path(__file__).resolve(),old_protocol,historical/'historical_selected_map_report.json',
        historical/'historical_ground_verification_fixture.json',ROOT/'config/model.yaml',
        ROOT/'environment/gazebo.yml',ROOT/protocol['gain_cache']['path'],
        ROOT/'build/gazebo/libSphericalWheelBridge.so',ROOT/'build/gazebo/libAswExactPhysics.so',
        yard/'dem/marsyard_terrain_hm.tif',*[yard/'meshes'/f'{name}.obj' for name in ('lava','rocks_big','rocks_medium')]]
    bindings={str(p.relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size) for p in files}
    save(STAGE/'registered_before_checks.json',dict(utc=datetime.now(timezone.utc).isoformat(),
        files=bindings,byte_checks_only=True,no_new_dem_arithmetic=True,no_physics=True,
        no_case_outcome_queries=True,new_route_corridors_not_yet_screened=True))
    for path in files:
        relative=str(path.relative_to(ROOT))
        expected=protocol['input_sha256'].get(relative)
        if expected is not None and sha(path)!=expected:raise ValueError('Original I75 input changed: '+relative)
    if sha(ROOT/protocol['gain_cache']['path'])!=protocol['gain_cache']['sha256']:raise ValueError('Gain cache changed')
    for filename,key in [('libSphericalWheelBridge.so','bridge_binary_sha256'),('libAswExactPhysics.so','exact_physics_binary_sha256')]:
        if sha(ROOT/'build/gazebo'/filename)!=protocol[key]:raise ValueError('Runtime plugin changed: '+filename)
    for name in ('historical_selected_map_report.json','historical_ground_verification_fixture.json'):
        shutil.copy2(historical/name,STAGE/name)
    shutil.copy2(old_protocol,STAGE/'historical_I75_protocol.json')
    source=STAGE/'source';source.mkdir();shutil.copy2(Path(__file__),source/Path(__file__).name)
    save(STAGE/'receipt.json',dict(iteration='I-76',passed=True,asset_identity_only=True,
        historical_protocol_sha256=OLD_PROTOCOL_SHA256,fixture_classification='Original moderate faceted Mars site; no new site search.',
        historical_S_screen_not_a_new_route_certificate=True,
        fixed_site_map_only_preflight_required_for_new_routes=True,
        no_new_dem_arithmetic=True,no_physics=True,no_case_outcome_arrays_read=True,
        source_and_inputs_unchanged=all(sha(ROOT/name)==binding['sha256'] for name,binding in bindings.items())))
    print(json.dumps(dict(passed=True,bound_files=len(files),asset_identity_only=True,new_route_screen_still_required=True),indent=2))


if __name__=='__main__':main()
