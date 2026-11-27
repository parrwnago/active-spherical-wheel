"""Separate bounded cross-version validation, preserving the strict failure.

Original mathematical inputs must pass byte bindings; flags/IDs/counts and all
other outputs remain exact. Only listed derived float operations can differ,
bounded by 1e-10*max(1,abs(reference)). No output value is changed or replaced.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).parent/'portable_tools'))
from packet_archive import load_packets,load_trace


ARRAY_PATTERN=re.compile(r'^call_\d+__(?:w_original_Ax_minus_b|unit_step_projected_kkt_residual)$')
FLOAT_PATTERN=re.compile(r'^\.(?:solver_calls\[\d+\]\.(?:w_original_Ax_minus_b\[\d+\]|unit_step_projected_kkt_residual\[\d+\]|max_abs_unit_step_projected_kkt_residual)|applied_contacts\[\d+\]\.(?:force_normal_projection_N|force_tangent_norm_N)|(?:new_trace_matches|historical_trace_matches)\.matches\[\d+\]\.force_match_tolerance_N)$')


def digest(a):return hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('reference','actual','package','output'):p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    packets=load_packets(args.package);load_trace(args.package,'new');load_trace(args.package,'historical')
    result=dict(source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),numpy_version=np.__version__,
        original_mathematical_input_bindings_byte_exact=True,packet_count=len(packets),
        allowance='Derived float differences only: abs(actual-reference)<=1e-10*max(1,abs(reference)); original padding excluded by archive design.',
        array_differences=[],report_differences=[],failures=[])
    with np.load(args.reference/'checks.npz',allow_pickle=False) as a,np.load(args.actual/'checks.npz',allow_pickle=False) as b:
        if set(a.files)!=set(b.files):result['failures'].append('Array field sets differ')
        for key in sorted(set(a.files)&set(b.files)):
            x,y=a[key],b[key]
            if x.shape!=y.shape or x.dtype!=y.dtype:
                result['failures'].append('Array shape/dtype differs: '+key);continue
            if digest(x)==digest(y):continue
            if not ARRAY_PATTERN.fullmatch(key) or x.dtype.kind!='f':
                result['failures'].append('Unallowed nonexact array: '+key);continue
            if not np.array_equal(np.isfinite(x),np.isfinite(y)):
                result['failures'].append('Array finite masks differ: '+key);continue
            finite=np.isfinite(x);delta=np.abs(x[finite]-y[finite]);limit=1e-10*np.maximum(1.,np.abs(x[finite]))
            passed=bool(np.all(delta<=limit))
            result['array_differences'].append(dict(field=key,max_abs=float(delta.max(initial=0.)),bounded=passed))
            if not passed:result['failures'].append('Array rounding bound exceeded: '+key)
    def compare(a,b,path=''):
        if type(a)!=type(b):result['failures'].append('Type mismatch: '+path);return
        if isinstance(a,dict):
            if set(a)!=set(b):result['failures'].append('Report keys differ: '+path);return
            for key in a:compare(a[key],b[key],path+'.'+key)
        elif isinstance(a,list):
            if len(a)!=len(b):result['failures'].append('Report lengths differ: '+path);return
            for i,(x,y) in enumerate(zip(a,b)):compare(x,y,path+f'[{i}]')
        elif a!=b:
            allowed=isinstance(a,float) and FLOAT_PATTERN.fullmatch(path) is not None
            difference=abs(a-b) if isinstance(a,(float,int)) and not isinstance(a,bool) else None
            passed=bool(allowed and np.isfinite(a) and np.isfinite(b) and difference<=1e-10*max(1.,abs(a)))
            result['report_differences'].append(dict(path=path,reference=a,actual=b,absolute_difference=difference,bounded=passed))
            if not passed:result['failures'].append('Unallowed or excessive report difference: '+path)
    compare(json.loads((args.reference/'report.json').read_text()),json.loads((args.actual/'report.json').read_text()))
    result['science_byte_exact']=not result['array_differences'] and not result['report_differences'] and not result['failures']
    result['all_discrete_outputs_exact']=not any('Unallowed' in x or 'Type mismatch' in x or 'keys differ' in x or 'lengths differ' in x for x in result['failures'])
    result['bounded_portability_qualified']=not result['failures']
    result['max_array_abs_difference']=max((x['max_abs'] for x in result['array_differences']),default=0.)
    result['max_report_abs_difference']=max((x['absolute_difference'] or 0. for x in result['report_differences']),default=0.)
    args.output.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n')
    print(json.dumps({key:result[key] for key in ('numpy_version','original_mathematical_input_bindings_byte_exact','science_byte_exact','all_discrete_outputs_exact','bounded_portability_qualified','max_array_abs_difference','max_report_abs_difference')},sort_keys=True))
    if not result['bounded_portability_qualified']:raise AssertionError('Portability bound not qualified; all values preserved')


if __name__=='__main__':main()
