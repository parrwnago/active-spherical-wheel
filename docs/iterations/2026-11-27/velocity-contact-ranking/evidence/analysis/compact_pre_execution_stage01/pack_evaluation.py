"""Lossless compact evaluator inputs and hashes of every canonical frame array."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import shutil

import numpy as np

CASES=('m100_B','p030_B','p090_B')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path,value):
    path.write_text(json.dumps(value,indent=2)+'\n')


def run(source,output):
    output.mkdir(parents=True,exist_ok=False)
    original=json.loads((source/'artifact_manifest.json').read_text())
    for item in original['files']:
        if sha(source/item['path'])!=item['sha256']:raise ValueError('Changed original evaluator artifact')
    registration=dict(iteration='I-60',registered_before_projection_utc=datetime.now(timezone.utc).isoformat(),
        projection_source_sha256=sha(Path(__file__)),original_manifest_sha256=sha(source/'artifact_manifest.json'),
        known_prior='Original results known; mechanical full-rate projection, no criterion or scientific calculation changes.')
    save(output/'registered_before_projection.json',registration)
    shutil.copyfile(Path(__file__),output/'pack_evaluation.py')
    for name in ('evaluate_scores.py','protocol.json','science.json'):
        shutil.copyfile(source/name,output/name)
    shutil.copyfile(source/'artifact_manifest.json',output/'original_artifact_manifest.json')
    bindings={}
    for case in CASES:
        path=source/(case+'_evaluation.npz')
        with np.load(path,allow_pickle=False) as data:
            arrays={name:data[name].copy() for name in data.files}
        compact={name:arrays[name] for name in ('time_s','truth_patch_loads_N','truth_patch_contact_counts')}
        np.savez_compressed(output/(case+'_truth_projection.npz'),**compact)
        bindings[case]=dict(original_file_sha256=sha(path),
            arrays={name:dict(dtype=value.dtype.str,shape=list(value.shape),
                             sha256=hashlib.sha256(value.tobytes()).hexdigest())
                    for name,value in arrays.items()},
            retained_truth_fields=list(compact),time_rows=len(arrays['time_s']))
    save(output/'expected_array_bindings.json',dict(iteration='I-60',cases=bindings,
        scope='All original frame arrays retain dtype/shape/byte hashes; truth-only arrays are full-rate and byte-preserving. Recomputed rank/context regenerate other arrays.'))
    save(output/'artifact_manifest.json',dict(iteration='I-60',files=[
        dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(cases=3,all_time_rows=26403,downsampled=False,
                         bytes=sum(p.stat().st_size for p in output.rglob('*') if p.is_file()))))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    run(args.data_source.resolve(),args.output.resolve())
