"""Exact cached-output validation; no output values are changed."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def array_binding(array):
    return dict(shape=list(array.shape),dtype=array.dtype.str,
                sha256=hashlib.sha256(np.ascontiguousarray(array).tobytes()).hexdigest())


def main():
    p=argparse.ArgumentParser()
    for name in ('reference','actual','output'):p.add_argument('--'+name,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    result=dict(source_sha256=sha(__file__),numpy_version=np.__version__,arrays={},reports={})
    for name in ('interval_frames.npz','substep_frames.npz'):
        with np.load(args.reference/name,allow_pickle=False) as old,np.load(args.actual/name,allow_pickle=False) as new:
            if set(old.files)!=set(new.files):raise AssertionError('Field-set mismatch')
            differences={k:dict(expected=array_binding(old[k]),actual=array_binding(new[k]))
                         for k in old.files if array_binding(old[k])!=array_binding(new[k])}
            result['arrays'][name]=dict(array_count=len(old.files),all_byte_exact=not differences,differences=differences)
    old=json.loads((args.reference/'report.json').read_text())
    new=json.loads((args.actual/'report.json').read_text())
    old_inputs=old.pop('inputs',None);new_inputs=new.pop('inputs',None)
    result['reports']=dict(all_scientific_values_exact=old==new,input_path_labels_exact=old_inputs==new_inputs,
        reference_input_bindings=old_inputs,actual_input_bindings=new_inputs,
        input_hash_multisets_exact=sorted(old_inputs.values())==sorted(new_inputs.values()))
    result['all_checks_passed']=all(v['all_byte_exact'] for v in result['arrays'].values()) and old==new and result['reports']['input_hash_multisets_exact']
    args.output.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print(json.dumps(result,sort_keys=True))
    if not result['all_checks_passed']:raise AssertionError('Replay output mismatch; original values preserved')


if __name__=='__main__':main()
