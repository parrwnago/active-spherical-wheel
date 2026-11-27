"""Verify and load compact measured-state primitives; no contact truth/labels."""
import hashlib
import json
from pathlib import Path

import numpy as np


def binding(array):
    a = np.asarray(array)
    return dict(dtype=a.dtype.str, shape=list(a.shape), sha256=hashlib.sha256(np.ascontiguousarray(a).tobytes()).hexdigest())


def load_primitives(package, case):
    package = Path(package)
    manifest = json.loads((package/'package_manifest.json').read_text())
    if case not in manifest['cases']:
        raise ValueError('case absent from frozen primitive package')
    for relative, expected in manifest['files'].items():
        if relative.startswith(case+'/') or relative.startswith('source/'):
            actual = hashlib.sha256((package/relative).read_bytes()).hexdigest()
            if actual != expected['sha256']:
                raise ValueError('package file binding mismatch: '+relative)
    with np.load(package/case/'primitives.npz', allow_pickle=False) as packet:
        arrays = {key: packet[key] for key in packet.files}
    expected = json.loads((package/case/'primitive_array_bindings.json').read_text())
    if set(arrays) != set(expected):
        raise ValueError('primitive field set changed')
    for key, a in arrays.items():
        if binding(a) != expected[key]:
            raise ValueError('primitive array binding mismatch: '+key)
    if any('truth' in key or 'terrain' in key or 'estimated' in key for key in arrays):
        raise ValueError('unsupported inference primitive field')
    return arrays
