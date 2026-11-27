"""Standalone NumPy replay of I-63 cached primitives and frozen analysis.

Post-result verification helper: changes no scientific rule or fit. Cached
kinematic axes are restored losslessly, not regenerated from a model.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,importlib.util,json,sys

import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value


def json_differences(expected,actual,path='',rows=None):
    if rows is None:rows=[]
    if isinstance(expected,dict) and isinstance(actual,dict):
        if set(expected)!=set(actual):rows.append(dict(path=path,reason='keys differ'))
        for key in sorted(set(expected)&set(actual)):
            if key=='numpy_version':continue
            json_differences(expected[key],actual[key],path+'.'+key,rows)
    elif isinstance(expected,list) and isinstance(actual,list):
        if len(expected)!=len(actual):rows.append(dict(path=path,reason='list length differs'))
        for i,(a,b) in enumerate(zip(expected,actual)):json_differences(a,b,path+'[%d]'%i,rows)
    elif isinstance(expected,float) and isinstance(actual,(float,int)) and not isinstance(actual,bool):
        if expected!=actual:rows.append(dict(path=path,reason='floating difference',expected=expected,actual=actual,
            absolute_difference=abs(expected-actual),passes=abs(expected-actual)<=1e-12))
    elif type(expected)!=type(actual) or expected!=actual:
        rows.append(dict(path=path,reason='exact scalar differs',expected=expected,actual=actual))
    return rows


def compare_npz(reference,replayed):
    with np.load(reference,allow_pickle=False) as r,np.load(replayed,allow_pickle=False) as a:
        if set(r.files)!=set(a.files):raise ValueError('Replay output array names differ')
        return {key:bool(r[key].dtype==a[key].dtype and r[key].shape==a[key].shape and r[key].tobytes()==a[key].tobytes()) for key in r.files}


def run(args):
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    source=args.analysis_source.resolve();reference=args.reference.resolve();features_root=args.feature_source.resolve()
    registration=read(reference/'registered_before_evaluation.json')
    if sha(source/'condition_diagnostic.py')!=registration['source_sha256']:raise ValueError('Analysis source differs from canonical')
    if sha(source/'i62_metrics.py')!=registration['metrics_dependency_sha256']:raise ValueError('Metric source differs from canonical')
    if sha(args.protocol)!=registration['protocol_sha256']:raise ValueError('Protocol differs from canonical')
    write(output/'registered_before_replay.json',dict(utc=datetime.now(timezone.utc).isoformat(),
        replay_source_sha256=sha(Path(__file__)),numpy_version=np.__version__,
        canonical_source_sha256=registration['source_sha256'],protocol_sha256=sha(args.protocol),
        feature_helper_sha256=sha(args.feature_helper),
        feature_manifests={case:sha(features_root/case/'artifact_manifest.json') for case in ('m100_B','p030_B','p090_B')},
        comparison='Primitive/condition/output arrays byte exact; JSON masks/counts/decisions exact, floats <=1e-12 absolute; NumPy version retained as metadata. No scientific value replacement.'))
    packets=module('_i63_cached_feature_replay',args.feature_helper.resolve())
    features=output/'regenerated_features';features.mkdir();feature_checks={}
    for case in packets.CASES:
        condition,parity=packets.load_and_derive_case(features_root,case);feature_checks[case]=parity
        directory=features/case;directory.mkdir();np.savez_compressed(directory/'conditions.npz',**condition)
    write(output/'feature_parity.json',feature_checks)
    if not all(row['all_byte_exact'] for row in feature_checks.values()):
        write(output/'comparison.json',dict(passed=False,reason='Feature arrays not byte exact; retained failure, no analysis rerun',numpy_version=np.__version__))
        return False
    sys.path.insert(0,str(source));primary=module('_i63_primary_replay',source/'condition_diagnostic.py')
    primary.run(args.protocol.resolve(),args.data_source.resolve(),features,output/'evaluation',args.old_witnesses.resolve())
    expected=read(reference/'science.json');actual=read(output/'evaluation/science.json')
    differences=json_differences(expected,actual)
    numerical_ok=all(row.get('passes',False) for row in differences)
    arrays={}
    for original in sorted(reference.glob('*.npz')):
        arrays[original.name]=compare_npz(original,output/'evaluation'/original.name)
    arrays_ok=all(all(fields.values()) for fields in arrays.values())
    result=dict(passed=numerical_ok and arrays_ok,numpy_version=np.__version__,
        canonical_numpy_version=expected['numpy_version'],condition_arrays_all_byte_exact=True,
        output_arrays_all_byte_exact=arrays_ok,output_arrays=arrays,json_differences=differences,
        scientific_rules_changed=False,new_physics_steps=0)
    write(output/'comparison.json',result)
    print(json.dumps({k:v for k,v in result.items() if k not in ('output_arrays','json_differences')}))
    return result['passed']


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('analysis-source','protocol','data-source','feature-source','feature-helper','old-witnesses','reference','output'):
        p.add_argument('--'+name,type=Path,required=True)
    sys.exit(0 if run(p.parse_args()) else 1)
