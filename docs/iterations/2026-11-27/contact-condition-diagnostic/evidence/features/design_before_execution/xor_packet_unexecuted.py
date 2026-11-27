"""Lossless I-63 primitive packets and NumPy-only causal feature replay.

Cached kinematic axes are bitwise XOR packed; they are not approximated or
regenerated from the physical model. Observer primitives remain unchanged.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,importlib.util,json,shutil,sys
import numpy as np

CASES=('m100_B','p030_B','p090_B')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def load(path):
    with np.load(path,allow_pickle=False) as value:return {key:value[key].copy() for key in value.files}
def description(value):
    return dict(dtype=str(value.dtype),shape=list(value.shape),sha256=hashlib.sha256(value.tobytes()).hexdigest())
def helper(path):
    spec=importlib.util.spec_from_file_location('_i63_cached_condition_helper',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
def verify_artifacts(directory):
    manifest=read(directory/'artifact_manifest.json')
    for item in manifest['files']:
        if sha(directory/item['path'])!=item['sha256']:raise ValueError('Changed original feature artifact')


def encode(primitives):
    packet={key:value.copy() for key,value in primitives.items() if key!='patch_axis_world'}
    axes=primitives['patch_axis_world']
    if axes.dtype!=np.dtype('float64') or axes.ndim!=3 or axes.shape[1:]!=(4,3):raise ValueError('Expected float64 known patch axes')
    bits=np.ascontiguousarray(axes).view(np.uint64)
    packet['axis_reference_bits']=bits[:,0,:].copy()
    packet['axis_xor_bits']=np.bitwise_xor(bits,bits[:,0:1,:])
    return packet


def decode(packet):
    primitives={key:value.copy() for key,value in packet.items() if key not in ('axis_reference_bits','axis_xor_bits')}
    bits=np.bitwise_xor(packet['axis_xor_bits'],packet['axis_reference_bits'][:,None,:])
    primitives['patch_axis_world']=bits.view(np.float64)
    return primitives


def compare_fields(actual,expected):
    if set(actual)!=set(expected):raise ValueError('Field names differ')
    rows={}
    for key,value in actual.items():
        observed=description(value);reference=expected[key]
        rows[key]=dict(byte_exact=observed==reference,expected=reference,actual=observed)
    return dict(all_byte_exact=all(row['byte_exact'] for row in rows.values()),fields=rows)


def load_and_derive_case(package,case):
    """Return (all full-rate derived arrays, explicit byte-parity report)."""
    package=Path(package);manifest=read(package/'packet_manifest.json')
    if case not in CASES:raise ValueError('Unknown case')
    if sha(package/'condition_features.py')!=manifest['feature_source_sha256']:raise ValueError('Changed frozen feature source')
    entry=manifest['cases'][case];file=package/entry['path']
    if sha(file)!=entry['sha256']:raise ValueError('Changed primitive packet')
    primitives=decode(load(file));primitive_parity=compare_fields(primitives,entry['primitive_fields'])
    if not primitive_parity['all_byte_exact']:raise ValueError('Lossless primitive decoding failed')
    derived=helper(package/'condition_features.py').derive_features(primitives)
    condition_parity=compare_fields(derived,entry['condition_fields'])
    parity=dict(case=case,numpy_version=np.__version__,primitives=primitive_parity,conditions=condition_parity,
        all_byte_exact=primitive_parity['all_byte_exact'] and condition_parity['all_byte_exact'])
    return derived,parity


def pack(source_root,feature_source,output):
    output.mkdir(parents=True,exist_ok=False)
    write(output/'registered_before_packet_arithmetic.json',dict(registered_utc=datetime.now(timezone.utc).isoformat(),
        packet_source_sha256=sha(Path(__file__)),feature_source_sha256=sha(feature_source),numpy_version=np.__version__,
        original_manifests={case:sha(source_root/case/'artifact_manifest.json') for case in CASES},
        operation='Lossless uint64 XOR encoding of cached axes. No feature formula, truth, score or policy changes.'))
    shutil.copy2(Path(__file__),output/'feature_packets.py');shutil.copy2(feature_source,output/'condition_features.py')
    entries={}
    for case in CASES:
        directory=source_root/case;verify_artifacts(directory)
        if sha(directory/'source/condition_features.py')!=sha(feature_source):raise ValueError('Mixed feature formula source')
        primitives=load(directory/'primitives.npz');conditions=load(directory/'conditions.npz');packet=encode(primitives)
        rebuilt=decode(packet)
        primitive_fields={key:description(value) for key,value in primitives.items()}
        condition_fields={key:description(value) for key,value in conditions.items()}
        if not compare_fields(rebuilt,primitive_fields)['all_byte_exact']:raise ValueError('Primitive encoding did not invert exactly')
        file=output/(case+'.npz');np.savez_compressed(file,**packet)
        entries[case]=dict(path=file.name,sha256=sha(file),bytes=file.stat().st_size,
            original_artifact_manifest_sha256=sha(directory/'artifact_manifest.json'),
            primitive_fields=primitive_fields,condition_fields=condition_fields)
    write(output/'packet_manifest.json',dict(iteration='I-63',producer_numpy=np.__version__,
        feature_source_sha256=sha(feature_source),cases=entries,cached_geometry=True,
        qualification='All original primitive bytes must restore. Pure NumPy recomputes features, not kinematic axes. No hardware or new physical run.'))
    checks={case:load_and_derive_case(output,case)[1] for case in CASES}
    write(output/'producer_parity.json',checks)
    if not all(row['all_byte_exact'] for row in checks.values()):raise ValueError('Producer feature replay not exact')
    print(json.dumps(dict(passed=True,total_primitive_packet_bytes=sum(row['bytes'] for row in entries.values()),
        cases=list(CASES),numpy_version=np.__version__)),flush=True)


def check(package,output):
    output.mkdir(parents=True,exist_ok=False)
    write(output/'registered_before_replay.json',dict(replay_source_sha256=sha(Path(__file__)),
        packet_manifest_sha256=sha(package/'packet_manifest.json'),numpy_version=np.__version__,
        registered_utc=datetime.now(timezone.utc).isoformat(),comparison='Strict byte comparison; no normalization, tolerances or replacement.'))
    checks={}
    for case in CASES:
        values,parity=load_and_derive_case(package,case);checks[case]=parity
        np.savez_compressed(output/(case+'_conditions.npz'),**values)
    write(output/'replay_report.json',checks)
    passed=all(row['all_byte_exact'] for row in checks.values())
    print(json.dumps(dict(passed=passed,numpy_version=np.__version__)))
    return 0 if passed else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);sub=parser.add_subparsers(dest='mode',required=True)
    build=sub.add_parser('pack');build.add_argument('--source-root',type=Path,required=True);build.add_argument('--feature-source',type=Path,required=True)
    build.add_argument('--output',type=Path,required=True)
    replay=sub.add_parser('check');replay.add_argument('--package',type=Path,required=True);replay.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.mode=='pack':pack(args.source_root.resolve(),args.feature_source.resolve(),args.output.resolve())
    else:sys.exit(check(args.package.resolve(),args.output.resolve()))
