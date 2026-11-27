"""Compare fresh I-65 outputs without replacing canonical values."""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np


def load(path):
    with np.load(path,allow_pickle=False) as z:return {k:z[k] for k in z.files}


def compare(reference,actual,output):
    output.mkdir(parents=True,exist_ok=False)
    entries={};passed=True;byte_exact=True;total=0
    expected_files={p.name for p in reference.glob('*.npz')}
    actual_files={p.name for p in actual.glob('*.npz')}
    if expected_files!=actual_files:raise ValueError('NPZ output inventory differs')
    for name in sorted(expected_files):
        old,new=load(reference/name),load(actual/name)
        if set(old)!=set(new):raise ValueError('Array inventory differs')
        report={}
        for key,a in old.items():
            b=new[key];total+=1
            structure=a.dtype==b.dtype and a.shape==b.shape
            exact=structure and a.tobytes()==b.tobytes()
            byte_exact &= exact
            if np.issubdtype(a.dtype,np.floating):
                same_nonfinite=structure and np.array_equal(np.isfinite(a),np.isfinite(b)) and np.array_equal(np.isnan(a),np.isnan(b)) and np.array_equal(np.isposinf(a),np.isposinf(b)) and np.array_equal(np.isneginf(a),np.isneginf(b))
                finite=np.isfinite(a)&np.isfinite(b)
                difference=float(np.max(abs(a[finite]-b[finite]))) if np.any(finite) else 0.
                atol=1e-7 if key.endswith('_score') else 1e-10
                okay=same_nonfinite and np.allclose(a[finite],b[finite],atol=atol,rtol=1e-10)
                report[key]=dict(pass_check=bool(okay),raw_bytes_exact=exact,max_abs_difference=difference,atol=atol,rtol=1e-10)
            else:
                okay=exact
                report[key]=dict(pass_check=bool(okay),raw_bytes_exact=exact)
            passed &= bool(okay)
        entries[name]=report
    old=json.loads((reference/'science.json').read_text());new=json.loads((actual/'science.json').read_text())
    floats=[];json_errors=[]
    def visit(a,b,path):
        if path=='numpy_version':return
        if isinstance(a,dict):
            if not isinstance(b,dict) or set(a)!=set(b):json_errors.append(path+':keys');return
            for k,v in a.items():visit(v,b[k],path+'.'+k if path else k)
        elif isinstance(a,list):
            if not isinstance(b,list) or len(a)!=len(b):json_errors.append(path+':length');return
            for i,(x,y) in enumerate(zip(a,b)):visit(x,y,path+f'[{i}]')
        elif isinstance(a,float):
            atol=1e-7 if ('score' in path or '.scores.' in path) else (1e-9 if ('point_' in path) else 1e-10)
            okay=isinstance(b,(float,int)) and abs(a-b)<=atol+1e-10*abs(a)
            if not okay:json_errors.append(path+':float')
            if a!=b:floats.append(dict(path=path,canonical=a,fresh=b,abs_difference=abs(a-b),atol=atol))
        elif type(a)!=type(b) or a!=b:json_errors.append(path+':value')
    visit(old,new,'')
    passed &= not json_errors
    result=dict(all_checks_pass=bool(passed),all_arrays_raw_bytes_exact=bool(byte_exact),
                array_count=total,canonical_numpy=old['numpy_version'],fresh_numpy=new['numpy_version'],
                no_canonical_values_replaced=True,array_comparison=entries,
                scientific_json_errors=json_errors,scientific_json_float_differences=floats)
    (output/'comparison.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('array_comparison','scientific_json_float_differences')},indent=2))
    if not passed:raise AssertionError('Replay differs beyond registered bounds')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference',type=Path,required=True);p.add_argument('--actual',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();compare(a.reference,a.actual,a.output)
