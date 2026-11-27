"""I71 evaluation-only projection, after all restricted predictions are sealed.

Retains frozen I70 truth/N/F/A fields exactly, but replaces ALL new-method point
fields with I71 outputs.  This output must never be passed to inference.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import shutil
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
BASELINE_FIELDS = ('time_s', 'truth_patch_loads_N', 'truth_patch_points_m', 'N_family',
    'F_family', 'A_family', 'i60_accepted', 'N_points_m', 'F_points_m', 'A_points_m')
PREDICTION_FIELDS = ('time_s', 'emission_points_m', 'state_hmm_points_m', 'hmm_points_m')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_fields(path, fields):
    with np.load(path, allow_pickle=False) as archive:
        return {key:archive[key].copy() for key in fields}


def project(baseline, prediction):
    if not np.array_equal(baseline['time_s'], prediction['time_s']):
        raise ValueError('Frozen baseline and new inference clocks differ')
    result = {key:np.array(baseline[key], copy=True) for key in BASELINE_FIELDS}
    for method in ('emission', 'state_hmm', 'hmm'):
        key = method+'_points_m'
        points = np.asarray(prediction[key])
        if points.shape != (len(result['time_s']), 4, 3):
            raise ValueError('New method points must have shape N by4by3')
        result[key] = points.copy()
    return result


def run(baseline_root, prediction_root, inference_seal, output):
    if not inference_seal.is_file():
        raise ValueError('All-three-inference-sealed receipt required before truth read')
    paths = [Path(__file__), inference_seal]
    for case in CASES:
        paths += [baseline_root/case/'evaluation_packet.npz', prediction_root/case/'predictions.npz']
    inputs = [dict(path=str(p), sha256=sha(p)) for p in paths]
    output.mkdir(parents=True, exist_ok=False)
    (output/'registered_before_truth_read.json').write_text(json.dumps(dict(iteration='I-71',
        utc=datetime.now(timezone.utc).isoformat(), files=inputs,
        baseline_fields_read=list(BASELINE_FIELDS), inference_fields_read=list(PREDICTION_FIELDS),
        boundary='Evaluation only. No I70 emission/HMM point field is read or reused.'), indent=2)+'\n')
    shutil.copy2(Path(__file__), output/'prepare_evaluation.py')
    outputs = []
    for case in CASES:
        packet = project(load_fields(baseline_root/case/'evaluation_packet.npz', BASELINE_FIELDS),
                         load_fields(prediction_root/case/'predictions.npz', PREDICTION_FIELDS))
        folder=output/case;folder.mkdir()
        path=folder/'evaluation_packet.npz'
        np.savez_compressed(path, **packet)
        outputs.append(dict(case=case, path=str(path.relative_to(output)), sha256=sha(path)))
    for item in inputs:
        if sha(item['path']) != item['sha256']:
            raise ValueError('Frozen input changed during evaluation projection')
    (output/'artifact_manifest.json').write_text(json.dumps(dict(iteration='I-71', files=outputs,
        no_new_scoring=True, no_new_physics=True), indent=2)+'\n')
    print(json.dumps(dict(iteration='I-71', cases=len(outputs), evaluation_only=True)))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('baseline-root', 'prediction-root', 'inference-seal', 'output'):
        parser.add_argument('--'+key, type=Path, required=True)
    a=parser.parse_args()
    run(a.baseline_root.resolve(),a.prediction_root.resolve(),a.inference_seal.resolve(),a.output.resolve())
