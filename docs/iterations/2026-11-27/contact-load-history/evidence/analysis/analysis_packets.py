"""Post-result exact bindings for omitted I-64 analysis NPZ outputs."""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,json
import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def describe(value):return dict(dtype=value.dtype.str,shape=list(value.shape),raw_sha256=hashlib.sha256(value.tobytes()).hexdigest())
def bind(reference,output):
    output.mkdir(parents=True,exist_ok=False)
    receipt=dict(utc=datetime.now(timezone.utc).isoformat(),source_sha256=sha(Path(__file__)),numpy_version=np.__version__,
        operation='Post-result bindings only; no metric/filter/gate/source changes. Canonical output arrays remain local.')
    (output/'registered_before_binding.json').write_text(json.dumps(receipt,indent=2)+'\n')
    files={}
    for path in sorted(reference.glob('*.npz')):
        with np.load(path,allow_pickle=False) as archive:fields={key:describe(archive[key]) for key in archive.files}
        files[path.name]=dict(original_container_sha256=sha(path),original_bytes=path.stat().st_size,arrays=fields)
    result=dict(iteration='I-64',source_sha256=sha(Path(__file__)),files=files,
        qualification='Reference NPZ containers omitted from compact archive; regenerate arrays from cached inputs and compare these exact dtype/shape/raw-byte hashes.')
    (output/'array_bindings.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(bound_files=len(files),bound_arrays=sum(len(x['arrays']) for x in files.values()))))


def compare_outputs(actual_root,bindings_path):
    bindings=json.loads(Path(bindings_path).read_text());files={}
    for name,entry in bindings['files'].items():
        with np.load(Path(actual_root)/name,allow_pickle=False) as archive:
            names=set(archive.files);expected=entry['arrays'];fields={}
            for key in sorted(names|set(expected)):
                value=describe(archive[key]) if key in names else None
                fields[key]=dict(byte_exact=key in expected and value==expected[key],expected=expected.get(key),actual=value)
        files[name]=dict(all_byte_exact=all(row['byte_exact'] for row in fields.values()),arrays=fields)
    return dict(all_byte_exact=all(row['all_byte_exact'] for row in files.values()),files=files)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--reference',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();bind(a.reference.resolve(),a.output.resolve())
