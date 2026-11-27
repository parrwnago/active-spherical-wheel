"""Validation-only local comparison against original canonical arrays.

No data or results are changed. The portable package still checks exact hashes.
This diagnostic needs the large canonical local derived files to quantify
cross-version differences; it is not required for compact package replay.
"""
from pathlib import Path
import argparse,hashlib,importlib.util,json
import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def main(package,canonical_root,output):
    output.mkdir(parents=True,exist_ok=False)
    spec=importlib.util.spec_from_file_location('_i64_packet_comparison',package/'history_packets.py')
    packets=importlib.util.module_from_spec(spec);spec.loader.exec_module(packets)
    results={}
    for case in packets.CASES:
        actual,parity=packets.load_and_derive_case(package,case)
        binding=packets.read(package/case/'array_bindings.json');canonical=canonical_root/case/'history_features.npz'
        if sha(canonical)!=binding['omitted_derived_file']['sha256']:raise ValueError('Changed local canonical file')
        reference=packets.load(canonical);fields={}
        for key,value in actual.items():
            expected=reference[key];row=dict(byte_exact=packets.description(value)==packets.description(expected),dtype=str(value.dtype))
            if np.issubdtype(value.dtype,np.floating):
                finite=np.isfinite(value)&np.isfinite(expected);row['nonfinite_pattern_exact']=bool(np.array_equal(np.isfinite(value),np.isfinite(expected)))
                difference=np.where(finite,np.abs(value-expected),0.)
                row['max_absolute_difference']=float(np.max(difference))
                if row['max_absolute_difference']>0:
                    index=tuple(map(int,np.unravel_index(np.argmax(difference),difference.shape)))
                    row.update(witness_index=list(index),canonical_value=float(expected[index]),recomputed_value=float(value[index]))
            fields[key]=row
        results[case]=dict(parity=parity['all_byte_exact'],fields=fields)
    report=dict(numpy_version=np.__version__,source_sha256=sha(Path(__file__)),results=results,
        no_values_modified=True,no_scientific_criterion_changed=True)
    packets.write(output/'comparison.json',report)
    for case,entry in results.items():
        differing={key:row['max_absolute_difference'] for key,row in entry['fields'].items() if not row['byte_exact']}
        print(json.dumps(dict(case=case,differing_array_count=len(differing),maximum_absolute_difference=max(differing.values(),default=0.),differences=differing)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--package',type=Path,required=True)
    p.add_argument('--canonical-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();main(a.package.resolve(),a.canonical_root.resolve(),a.output.resolve())
