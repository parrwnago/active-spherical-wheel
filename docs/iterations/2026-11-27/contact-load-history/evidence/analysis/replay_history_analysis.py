"""Standalone NumPy-only I-64 primitive/filter/analysis replay.

Post-result verifier. Uses fresh derived arrays and canonical byte bindings;
never replaces new values with stored scientific results.
"""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,importlib.util,json,shutil,sys
import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value


def differences(expected,actual,path='',rows=None):
    if rows is None:rows=[]
    if isinstance(expected,dict) and isinstance(actual,dict):
        if set(expected)!=set(actual):rows.append(dict(path=path,reason='keys differ'))
        for key in sorted(set(expected)&set(actual)):
            if key!='numpy_version':differences(expected[key],actual[key],path+'.'+key,rows)
    elif isinstance(expected,list) and isinstance(actual,list):
        if len(expected)!=len(actual):rows.append(dict(path=path,reason='list length differs'))
        for i,(a,b) in enumerate(zip(expected,actual)):differences(a,b,path+'[%d]'%i,rows)
    elif isinstance(expected,float) and isinstance(actual,(float,int)) and not isinstance(actual,bool):
        if expected!=actual:rows.append(dict(path=path,reason='floating difference',expected=expected,actual=actual,
            absolute_difference=abs(expected-actual),passes=abs(expected-actual)<=1e-12))
    elif type(expected)!=type(actual) or expected!=actual:rows.append(dict(path=path,reason='exact scalar differs',expected=expected,actual=actual))
    return rows


def run(args):
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    source=args.analysis_source.resolve();package=args.feature_package.resolve();reference=args.reference.resolve()
    registration=read(reference/'registered_before_evaluation.json')
    if sha(source/'history_diagnostic.py')!=registration['source_sha256']:raise ValueError('Changed canonical analysis source')
    if sha(source/'i62_metrics.py')!=registration['metric_dependency_sha256']:raise ValueError('Changed canonical metric dependency')
    if sha(args.protocol)!=registration['protocol_sha256']:raise ValueError('Changed canonical protocol')
    write(output/'registered_before_replay.json',dict(utc=datetime.now(timezone.utc).isoformat(),numpy_version=np.__version__,
        replay_source_sha256=sha(Path(__file__)),analysis_packet_source_sha256=sha(source/'analysis_packets.py'),
        protocol_sha256=sha(args.protocol),feature_packet_manifest_sha256=sha(package/'packet_manifest.json'),
        analysis_array_bindings_sha256=sha(args.analysis_bindings),
        comparison='All derived feature/output arrays exact dtype/shape/raw-byte hashes; JSON exact except floating differences <=1e-12 and NumPy version metadata. Actual differences preserved, no value replacement.'))
    packets=module('_i64_cached_history_packet',package/'history_packets.py')
    feature_root=output/'regenerated_features';feature_root.mkdir();feature_parity={}
    for case in packets.CASES:
        values,parity=packets.load_and_derive_case(package,case);feature_parity[case]=parity
        case_root=feature_root/case;case_root.mkdir();np.savez_compressed(case_root/'history_features.npz',**values)
        shutil.copy2(package/case/'primitives.npz',case_root/'primitives.npz')
    write(output/'feature_parity.json',feature_parity)
    if not all(row['all_byte_exact'] for row in feature_parity.values()):
        write(output/'comparison.json',dict(passed=False,reason='Feature arrays not byte exact; no analysis rerun; failure retained',numpy_version=np.__version__))
        return False
    sys.path.insert(0,str(source));primary=module('_i64_primary_replay',source/'history_diagnostic.py')
    primary.run(args.protocol.resolve(),args.base_root.resolve(),feature_root,args.old_witnesses.resolve(),output/'evaluation')
    arrays=module('_i64_analysis_packet',source/'analysis_packets.py').compare_outputs(output/'evaluation',args.analysis_bindings)
    write(output/'analysis_array_parity.json',arrays)
    expected=read(reference/'science.json');actual=read(output/'evaluation/science.json');diff=differences(expected,actual)
    passed=arrays['all_byte_exact'] and all(row.get('passes',False) for row in diff)
    result=dict(passed=passed,numpy_version=np.__version__,canonical_numpy_version=expected['numpy_version'],
        feature_arrays_byte_exact=True,analysis_output_arrays_byte_exact=arrays['all_byte_exact'],json_differences=diff,
        scientific_rules_changed=False,new_physics_steps=0)
    write(output/'comparison.json',result);print(json.dumps({k:v for k,v in result.items() if k!='json_differences'}))
    return passed


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('analysis-source','protocol','base-root','feature-package','analysis-bindings','old-witnesses','reference','output'):
        p.add_argument('--'+name,type=Path,required=True)
    sys.exit(0 if run(p.parse_args()) else 1)
