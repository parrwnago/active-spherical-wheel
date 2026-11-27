"""Post-extraction packaging only; frozen scientific feature formulas unchanged."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from portable_inputs import binding, load_primitives


def main():
    p = argparse.ArgumentParser(); p.add_argument('--prepared', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True); args = p.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    args.output.mkdir(parents=True); source = args.output/'source'; source.mkdir()
    for name in ('bank_points.py', 'input_features.py', 'prepare.py', 'portable_inputs.py'):
        shutil.copy2(Path(__file__).with_name(name), source/name)
    cases = ('m100_B', 'p030_B', 'p090_B'); omissions = {}
    for case in cases:
        original, folder = args.prepared/case, args.output/case; folder.mkdir()
        for name in ('primitives.npz', 'metadata.json', 'artifact_manifest.json'):
            shutil.copy2(original/name, folder/name)
        with np.load(original/'primitives.npz', allow_pickle=False) as packet:
            primitives = {key: packet[key] for key in packet.files}
        (folder/'primitive_array_bindings.json').write_text(json.dumps(
            {key: binding(value) for key, value in primitives.items()}, indent=2, sort_keys=True)+'\n')
        feature_path = original/'features.npz'
        with np.load(feature_path, allow_pickle=False) as packet:
            feature_bindings = {key: binding(packet[key]) for key in packet.files}
        (folder/'local_feature_array_bindings.json').write_text(json.dumps(feature_bindings, indent=2, sort_keys=True)+'\n')
        omissions[case] = dict(file='features.npz', bytes=feature_path.stat().st_size,
            sha256=hashlib.sha256(feature_path.read_bytes()).hexdigest(),
            reason='Local convenience cache omitted; derive perframe with frozen helper. Exact producer bindings retained; no cross-version pointwise numerical-bound claim.')
    files = {str(path.relative_to(args.output)): dict(bytes=path.stat().st_size,
             sha256=hashlib.sha256(path.read_bytes()).hexdigest())
             for path in args.output.rglob('*') if path.is_file()}
    manifest = dict(status='post-extraction packaging; no scientific formula changes',
        timestamp_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(), cases=cases,
        files=files, omitted_local_caches=omissions,
        coverage='All26403 frames, full original102 bank reconstructible. OriginalI60featureparity is a producerreceipt, not rerun without old projected-feature files.')
    (args.output/'package_manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True)+'\n')
    for case in cases:
        load_primitives(args.output, case)
    print(json.dumps(dict(cases=cases, total_package_bytes=sum(x['bytes'] for x in files.values()),
                         strict_primitive_bindings='PASS all3')))


if __name__ == '__main__': main()
