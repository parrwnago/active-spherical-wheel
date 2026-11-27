"""Project evaluation-only labels/baselines after all inference is sealed.

This program never supplies any array to inference.  It reads no new contacts,
does no model arithmetic and does not fit a scoring rule.  Its output contains
truth and therefore MUST remain outside the inference input boundary.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import shutil
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    with np.load(path, allow_pickle=False) as archive:
        return {key: archive[key].copy() for key in archive.files}


def scatter(family, points):
    family = np.asarray(family)
    if points.shape != (len(family), 3):
        raise ValueError('Selected singleton point must have shape N by3')
    output = np.full((len(family), 4, 3), np.nan)
    valid = (family >= 0) & (family < 4)
    output[np.flatnonzero(valid), family[valid].astype(int)] = points[valid]
    return output


def project(precision, baseline, old_points, predictions):
    time = precision['time_s']
    for other in (precision['truth_time_s'], baseline['time_s'], old_points['time_s'], predictions['time_s']):
        if not np.array_equal(time, other):
            raise ValueError('Truth/baseline/inference row clocks differ')
    N, F = baseline['native_family'], baseline['fused__chosen_family']
    accepted_family = baseline['fused__accepted_family']
    accepted = accepted_family >= 0
    if np.any(accepted & (accepted_family != F)):
        raise ValueError('Frozen accepted family differs from frozen raw chosen family')
    if not np.array_equal(old_points['chosen_family'][:, 3], F):
        raise ValueError('Frozen selected point and raw I60 family differ')
    if not np.array_equal(old_points['accepted'][:, 3], accepted):
        raise ValueError('Frozen selected point and I60 acceptance flag differ')
    Npoint = old_points['native_point_m']
    if not np.array_equal(Npoint, precision['exact_estimated_contact_point_m'], equal_nan=True):
        raise ValueError('Native equivalent point differs from original precision record')
    Fpoint = old_points['candidate_point_m'][:, 3]
    A = np.where(accepted, F, N)
    Apoint = np.where(accepted[:, None], Fpoint, Npoint)
    result = dict(time_s=time.copy(), truth_patch_loads_N=precision['truth_patch_loads_N'].copy(),
        truth_patch_points_m=precision['truth_patch_points_m'].copy(), N_family=N.copy(),
        F_family=F.copy(), A_family=A, i60_accepted=accepted,
        N_points_m=scatter(N, Npoint), F_points_m=scatter(F, Fpoint), A_points_m=scatter(A, Apoint))
    for name in ('emission', 'hmm', 'state_hmm'):
        points = predictions[name+'_points_m']
        if points.shape != (len(time), 4, 3):
            raise ValueError('Inference output points must have shape N by4by3')
        result[name+'_points_m'] = points.copy()
    return result


def run(original_root, baseline_root, point_root, prediction_root, inference_seal, output):
    if not inference_seal.is_file():
        raise ValueError('Root release/all-inference-sealed receipt required before truth read')
    seal_hash = sha(inference_seal)
    sources = []
    for case in CASES:
        for role, path in (
            ('original_precision', original_root/case/'precision.npz'),
            ('frozen_i60_evaluation', baseline_root/(case+'_evaluation.npz')),
            ('frozen_i60_selected_points', point_root/(case+'_selected_points.npz')),
            ('sealed_inference', prediction_root/case/'predictions.npz')):
            sources.append(dict(case=case, role=role, path=str(path), sha256=sha(path)))
    output.mkdir(parents=True, exist_ok=False)
    (output/'registered_before_truth_read.json').write_text(json.dumps(dict(iteration='I-70',
        utc=datetime.now(timezone.utc).isoformat(), source_sha256=sha(Path(__file__)),
        inference_seal_path=str(inference_seal), inference_seal_sha256=seal_hash,
        measurement_boundary='Evaluation-only output; no arrays returned to inference.', sources=sources), indent=2)+'\n')
    shutil.copy2(Path(__file__), output/'prepare_evaluation.py')
    outputs = []
    for case in CASES:
        result = project(load(original_root/case/'precision.npz'), load(baseline_root/(case+'_evaluation.npz')),
            load(point_root/(case+'_selected_points.npz')), load(prediction_root/case/'predictions.npz'))
        folder = output/case; folder.mkdir()
        target = folder/'evaluation_packet.npz'
        np.savez_compressed(target, **result)
        outputs.append(dict(case=case, path=str(target.relative_to(output)), sha256=sha(target)))
    if sha(inference_seal) != seal_hash:
        raise ValueError('Inference release receipt changed during projection')
    for source in sources:
        if sha(source['path']) != source['sha256']:
            raise ValueError('Original input changed during projection')
    (output/'artifact_manifest.json').write_text(json.dumps(dict(iteration='I-70', files=outputs,
        source_sha256=sha(Path(__file__)), no_new_physics=True, no_new_scoring=True), indent=2)+'\n')
    print(json.dumps(dict(iteration='I-70', cases=len(outputs), evaluation_only=True)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ('original-root', 'baseline-root', 'point-root', 'prediction-root', 'inference-seal', 'output'):
        parser.add_argument('--'+key, type=Path, required=True)
    args = parser.parse_args()
    run(args.original_root.resolve(), args.baseline_root.resolve(), args.point_root.resolve(),
        args.prediction_root.resolve(), args.inference_seal.resolve(), args.output.resolve())
