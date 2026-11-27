"""Strict comparison of cached replay outputs; values are never replaced."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def binding(a):return dict(shape=list(a.shape),dtype=a.dtype.str,sha256=hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest())


def main():
    p=argparse.ArgumentParser()
    for key in ('reference','actual','output'):p.add_argument('--'+key,type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    old=json.loads((args.reference/'report.json').read_text());new=json.loads((args.actual/'report.json').read_text())
    with np.load(args.reference/'checks.npz',allow_pickle=False) as a,np.load(args.actual/'checks.npz',allow_pickle=False) as b:
        same_fields=set(a.files)==set(b.files)
        differences={k:dict(reference=binding(a[k]),actual=binding(b[k])) for k in a.files if k not in b.files or binding(a[k])!=binding(b[k])}
        count=len(a.files)
    result=dict(source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),numpy_version=np.__version__,
        report_all_values_exact=old==new,array_field_sets_exact=same_fields,array_count=count,
        all_array_bytes_exact=not differences,array_differences=differences,
        all_checks_passed=old==new and same_fields and not differences)
    args.output.write_text(json.dumps(result,indent=2,sort_keys=True)+'\n');print(json.dumps(result,sort_keys=True))
    if not result['all_checks_passed']:raise AssertionError('Portable output mismatch, all original output values retained')


if __name__=='__main__':main()
