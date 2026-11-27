"""NumPy-only reproduction of descriptive I-60 selected-point statistics."""
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(source, output):
    if output.exists():
        raise FileExistsError(output)
    manifest = json.loads((source/'artifact_manifest.json').read_text())
    for item in manifest['files']:
        if sha(source/item['path']) != item['sha256']:
            raise ValueError('Selected-point source changed: '+item['path'])
    spec = importlib.util.spec_from_file_location('_i60_point_stats', source/'check_points.py')
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    saved = json.loads((source/'report.json').read_text())
    cases, arrays = {}, []
    for case in module.CASES:
        with np.load(source/(case+'_selected_points.npz'), allow_pickle=False) as data:
            points = {key: data[key].copy() for key in data.files}
        cases[case] = module.summarize(points);arrays.append(points)
    aggregate = {key: np.concatenate([item[key] for item in arrays]) for key in arrays[0]}
    checks = dict(case_statistics_exact=cases == saved['cases'],
                  aggregate_statistics_exact=module.summarize(aggregate) == saved['aggregate'])
    if not all(checks.values()):
        raise ValueError('Point statistic replay differs')
    output.write_text(json.dumps(dict(iteration='I-60', **checks,
        cases=len(cases), point_source_manifest_sha256=sha(source/'artifact_manifest.json'),
        scope='Distance statistics only; geometric reconstruction remains bound to original raw state and model.',
        no_mujoco_imported=True, new_physics_steps=0), indent=2)+'\n')
    print(json.dumps(checks))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args();run(args.data_source.resolve(), args.output.resolve())
