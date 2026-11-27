"""Mechanically project sealed I-61 truth inputs and frame-byte bindings."""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,json,shutil
import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,x):Path(path).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')


def run(canonical,output):
    output.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((canonical/'artifact_manifest.json').read_text())
    for item in manifest['files']:
        if sha(canonical/item['path'])!=item['sha256']:raise ValueError('Canonicalstagechanged')
    original=json.loads((canonical/'registered_before_truth_read.json').read_text())
    protocol=json.loads((canonical/'protocol.json').read_text());bindings={}
    registration=dict(iteration='I-61',registered_before_projection_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(Path(__file__)),canonical_manifest_sha256=sha(canonical/'artifact_manifest.json'),
        known_prior='Canonicalfailedfivegates; this is lossless mechanical projection, not a new hypothesis or tolerance search.')
    save(output/'registered_before_projection.json',registration)
    shutil.copy2(Path(__file__),output/'pack_evaluation.py')
    for name in ('evaluate_load_gate.py','protocol.json','science.json','registered_before_truth_read.json','artifact_manifest.json'):
        shutil.copy2(canonical/name,output/('original_artifact_manifest.json' if name=='artifact_manifest.json' else name))
    for entry in original['features_and_truth']:
        case=entry['case'];path=Path(entry['precision_path'])
        if sha(path)!=entry['precision_sha256']:raise ValueError('Originaltruthinputchanged')
        names=('truth_time_s','truth_patch_loads_N','truth_patch_points_m','truth_patch_contact_counts','truth_equivalent_point_m')
        with np.load(path,allow_pickle=False) as z:truth={name:z[name].copy() for name in names}
        np.savez_compressed(output/(case+'_truth_projection.npz'),**truth)
        with np.load(canonical/(case+'_evaluation.npz'),allow_pickle=False) as z:
            arrays={name:dict(dtype=z[name].dtype.str,shape=list(z[name].shape),sha256=hashlib.sha256(z[name].tobytes()).hexdigest())for name in z.files}
        bindings[case]=dict(original_precision_sha256=entry['precision_sha256'],
            original_evaluation_npz_sha256=sha(canonical/(case+'_evaluation.npz')),arrays=arrays)
    event=Path(original['frozen_event_selection']['path'])
    if sha(event)!=original['frozen_event_selection']['sha256']:raise ValueError('Oldeventselectionchanged')
    destination=output/'frozen_i60_event_context/analysis/events_stage01/events.json'
    destination.parent.mkdir(parents=True);shutil.copy2(event,destination)
    save(output/'expected_array_bindings.json',dict(cases=bindings))
    save(output/'artifact_manifest.json',dict(iteration='I-61',files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(cases=len(bindings),bytes=sum(p.stat().st_size for p in output.rglob('*')if p.is_file()))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--canonical',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    run(a.canonical.resolve(),a.output.resolve())
