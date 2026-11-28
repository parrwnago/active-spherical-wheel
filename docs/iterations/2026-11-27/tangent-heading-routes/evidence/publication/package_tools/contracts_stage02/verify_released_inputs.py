"""Read-only byte verification of root-released I76 outputs/runtime bindings."""
from datetime import datetime,timezone
from pathlib import Path
import argparse
import hashlib
import json


def sha(path):
    result=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):result.update(block)
    return result.hexdigest()


def verify(base,released_seal):
    base=Path(base).resolve();repo=base.parents[1]
    seal_path=base/'campaign/outputs_sealed.json'
    if sha(seal_path)!=released_seal:raise ValueError('All-eighteen release seal changed')
    seal=json.loads(seal_path.read_text());protocol_path=base/'protocol/protocol.json'
    if sha(protocol_path)!=seal['protocol_sha256']:raise ValueError('Original physics protocol changed')
    protocol=json.loads(protocol_path.read_text())
    checks={}
    for group in ('source_sha256','input_sha256'):
        for relative,digest in protocol[group].items():
            actual=sha(repo/relative)
            if actual!=digest:raise ValueError('Registered source/input changed: '+relative)
            checks[group+'/'+relative]=actual
    gain=protocol['gain_cache']
    for relative,digest in [(gain['path'],gain['sha256']),
        ('build/gazebo/libSphericalWheelBridge.so',protocol['bridge_binary_sha256']),
        ('build/gazebo/libAswExactPhysics.so',protocol['exact_physics_binary_sha256'])]:
        actual=sha(repo/relative)
        if actual!=digest:raise ValueError('Frozen gain/runtime binary changed: '+relative)
        checks[relative]=actual
    raw_checks={}
    for relative,expected in seal['files'].items():
        path=base/relative;actual=dict(bytes=path.stat().st_size,sha256=sha(path))
        if actual!=expected:raise ValueError('Sealed original case artifact changed: '+relative)
        raw_checks[relative]=actual
    result=dict(iteration='I-76',verified_utc=datetime.now(timezone.utc).isoformat(),
        released_output_seal_sha256=released_seal,protocol_sha256=sha(protocol_path),
        source_count=len(protocol['source_sha256']),input_count=len(protocol['input_sha256']),
        sealed_case_artifact_count=len(raw_checks),checks=checks,case_artifacts=raw_checks,
        all_original_bindings_unchanged=True,case_arrays_loaded=False,new_metric_or_inference_arithmetic=False,
        scope='Read-only byte hashing after root all-eighteen release. Execution completion is not scientific acceptance.')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,required=True);parser.add_argument('--released-seal',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=verify(args.base,args.released_seal)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream:json.dump(result,stream,indent=2,sort_keys=True);stream.write('\n')
    print(json.dumps({key:result[key] for key in ('source_count','input_count','sealed_case_artifact_count','all_original_bindings_unchanged')}))
