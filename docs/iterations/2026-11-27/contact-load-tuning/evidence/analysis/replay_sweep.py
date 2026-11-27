"""Strict standalone I-62 replay: Python 3 plus NumPy, no robot/ROS imports."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,importlib.util,json,sys
import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,data):Path(path).write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
def read(path):return json.loads(Path(path).read_text())


def differences(expected,actual,path=''):
    if isinstance(expected,dict) and isinstance(actual,dict):
        result=[]
        if set(expected)!=set(actual):result.append(dict(path=path,expected_keys=sorted(expected),actual_keys=sorted(actual)))
        for key in sorted(set(expected)&set(actual)):
            if path=='' and key=='numpy_version':continue
            result.extend(differences(expected[key],actual[key],path+'.'+key))
        return result
    if isinstance(expected,list) and isinstance(actual,list):
        if len(expected)!=len(actual):return [dict(path=path,expected_length=len(expected),actual_length=len(actual))]
        return [d for index,(x,y) in enumerate(zip(expected,actual)) for d in differences(x,y,path+'['+str(index)+']')]
    if type(expected) is not type(actual) or expected!=actual:
        return [dict(path=path,expected=expected,actual=actual)]
    return []


def verify_artifacts(directory):
    manifest=read(directory/'artifact_manifest.json')
    for item in manifest['files']:
        if sha(directory/item['path'])!=item['sha256']:raise ValueError('Changed artifact: '+str(directory/item['path']))


def compare_npz(reference,replayed):
    result=[]
    with np.load(reference,allow_pickle=False) as left,np.load(replayed,allow_pickle=False) as right:
        if set(left.files)!=set(right.files):raise ValueError('Array key mismatch: '+str(reference))
        for key in sorted(left.files):
            x,y=left[key],right[key];equal=x.dtype==y.dtype and x.shape==y.shape and x.tobytes()==y.tobytes()
            item=dict(array=key,dtype=str(x.dtype),shape=list(x.shape),byte_exact=bool(equal),
                expected_sha256=hashlib.sha256(x.tobytes()).hexdigest(),actual_sha256=hashlib.sha256(y.tobytes()).hexdigest())
            if not equal and x.shape==y.shape and np.issubdtype(x.dtype,np.floating) and np.issubdtype(y.dtype,np.floating):
                finite=np.isfinite(x)&np.isfinite(y)
                item['finite_max_absolute_difference']=float(np.max(np.abs(x[finite]-y[finite]))) if finite.any() else None
                item['nonfinite_pattern_exact']=bool(np.array_equal(np.isfinite(x),np.isfinite(y)))
            result.append(item)
    return result


def main(package,output):
    package=package.resolve();output=output.resolve();output.mkdir(parents=True,exist_ok=False)
    analysis=package/'analysis';reference=analysis/'stage02';inputs=analysis/'compact_stage01'
    source=analysis/'tolerance_sweep.py';protocol=package/'protocol/protocol.json'
    verify_artifacts(reference);verify_artifacts(inputs)
    frozen=read(reference/'registered_before_sweep.json')
    for actual,expected,label in [(sha(source),frozen['source_sha256'],'sweep source'),
        (sha(protocol),frozen['protocol_sha256'],'protocol'),
        (sha(inputs/'artifact_manifest.json'),frozen['input_manifest_sha256'],'compact input manifest')]:
        if actual!=expected:raise ValueError('Changed '+label)
    write(output/'registered_before_replay.json',dict(registered_utc=datetime.now(timezone.utc).isoformat(),
        replay_source_sha256=sha(Path(__file__)),sweep_source_sha256=sha(source),protocol_sha256=sha(protocol),
        compact_manifest_sha256=sha(inputs/'artifact_manifest.json'),reference_manifest_sha256=sha(reference/'artifact_manifest.json'),
        numpy_version=np.__version__,comparison='All grid arrays and selected policies byte exact; scientific JSON exactly equal except NumPy version; CSV byte exact. No tolerances, output replacement or value normalization.'))
    spec=importlib.util.spec_from_file_location('_i62_frozen_sweep',source);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.run(protocol,inputs,output/'recomputed',128)
    arrays={p.name:compare_npz(p,output/'recomputed'/p.name) for p in sorted(reference.glob('*.npz'))}
    science_expected=read(reference/'science.json');science_actual=read(output/'recomputed/science.json')
    science_differences=differences(science_expected,science_actual)
    csv_exact=(reference/'all_grid_case_metrics.csv').read_bytes()==(output/'recomputed/all_grid_case_metrics.csv').read_bytes()
    array_exact=all(item['byte_exact'] for group in arrays.values() for item in group)
    passed=array_exact and not science_differences and csv_exact
    report=dict(passed=bool(passed),producer_numpy=science_expected['numpy_version'],replay_numpy=np.__version__,
        arrays_byte_exact=bool(array_exact),array_count=sum(map(len,arrays.values())),arrays=arrays,
        all_science_exact_except_numpy_version=not bool(science_differences),science_differences=science_differences,
        csv_byte_exact=csv_exact,full_refit=True,no_new_physics=True,no_reported_values_changed=True)
    verify_artifacts(reference);verify_artifacts(inputs)
    write(output/'replay_report.json',report)
    print(json.dumps({key:report[key] for key in ['passed','producer_numpy','replay_numpy','arrays_byte_exact','array_count','all_science_exact_except_numpy_version','csv_byte_exact']}))
    return 0 if passed else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package',type=Path,default=Path(__file__).resolve().parent.parent)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();sys.exit(main(args.package,args.output))
