"""Post-primary I-60 selected-point check; no new inference or controller change."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import csv
import hashlib
import importlib.util
import json
import shutil
import sys

import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
MODES = ('geometry', 'wrench', 'velocity', 'fused')
TARGETS = ('equivalent_raw', 'dominant_patch_raw', 'equivalent_radial_R_proxy', 'dominant_patch_radial_R_proxy')
SUBSETS = ('all_matched', 'accepted', 'correct_family', 'strict_one_physical_point')
ROOT = Path(__file__).resolve().parents[3]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False)+'\n')


def load(path):
    with np.load(path, allow_pickle=False) as data:
        return {name: data[name].copy() for name in data.files}


def module(path):
    spec = importlib.util.spec_from_file_location('_i60_point_core', path)
    value = importlib.util.module_from_spec(spec);sys.modules[spec.name] = value
    spec.loader.exec_module(value)
    return value


def stats(values):
    if not len(values):
        return dict(count=0, mean_mm=None, RMS_mm=None, median_mm=None, p95_mm=None, maximum_mm=None)
    values = np.asarray(values)*1000.
    return dict(count=int(len(values)), mean_mm=float(values.mean()), RMS_mm=float(np.sqrt(np.mean(values**2))),
        median_mm=float(np.median(values)), p95_mm=float(np.percentile(values, 95)), maximum_mm=float(values.max()))


def summarize(points):
    base = points['primary_eligible']
    native = points['native_point_m'];candidate = points['candidate_point_m'];targets = points['targets_m']
    native_finite = np.isfinite(native).all(axis=1)
    result = {}
    for mode_index, mode in enumerate(MODES):
        result[mode] = {}
        candidate_finite = np.isfinite(candidate[:, mode_index]).all(axis=1)
        for target_index, target in enumerate(TARGETS):
            target_point = targets[:, target_index]
            target_finite = np.isfinite(target_point).all(axis=1)
            matched = base&native_finite&candidate_finite&target_finite
            masks = (matched, matched&points['accepted'][:, mode_index],
                     matched&(points['chosen_family'][:, mode_index] == points['truth_family']),
                     matched&points['strict_one_physical_point'])
            result[mode][target] = {}
            for subset, mask in zip(SUBSETS, masks):
                native_distance = np.linalg.norm(native[mask]-target_point[mask], axis=1)
                candidate_distance = np.linalg.norm(candidate[mask, mode_index]-target_point[mask], axis=1)
                movement = np.linalg.norm(candidate[mask, mode_index]-native[mask], axis=1)
                result[mode][target][subset] = dict(
                    primary_eligible_count=int(base.sum()), matched_count=int(mask.sum()),
                    missing_native_on_primary=int((base&~native_finite).sum()),
                    missing_candidate_on_primary=int((base&~candidate_finite).sum()),
                    missing_target_on_primary=int((base&~target_finite).sum()),
                    native=stats(native_distance), candidate=stats(candidate_distance), movement_from_native=stats(movement),
                    candidate_minus_native_mean_mm=float(1000.*(candidate_distance.mean()-native_distance.mean())) if mask.any() else None)
    return result


def radial_proxy(point, centre, radius):
    offset = point-centre;length = np.linalg.norm(offset, axis=1)
    valid = np.isfinite(point).all(axis=1)&np.isfinite(length)&(length > 1e-12)
    proxy = np.full_like(point, np.nan)
    proxy[valid] = centre[valid]+radius*offset[valid]/length[valid, None]
    return proxy


def execute(output):
    import mujoco
    if output.exists():
        raise FileExistsError(output)
    base = ROOT/'results/velocity_contact_rank_20261127'
    protocol_path = base/'protocol/post_primary_point_check.json'
    paths = [Path(__file__), protocol_path]
    records = []
    for case in CASES:
        feature = base/'campaign/features_stage01'/case
        original = ROOT/'results/balance_priority_20261126/campaign/runs'/case
        provenance = json.loads((original/'provenance.json').read_text())
        xml = Path(provenance['models']['mirror_xml'])
        source = feature/'source/contact_hypotheses.py'
        record = dict(case=case, feature=feature, original=original, xml=xml, source=source,
            evaluation=base/'analysis/stage01'/(case+'_evaluation.npz'))
        paths += [feature/'measurement_context.npz', feature/'scores.npz', feature/'artifact_manifest.json',
                  original/'precision.npz', original/'provenance.json', xml, source, record['evaluation']]
        records.append(record)
    output.mkdir(parents=True)
    shutil.copy2(Path(__file__), output/'check_points.py')
    shutil.copy2(protocol_path, output/'protocol.json')
    save(output/'registered_before_point_arithmetic.json', dict(iteration='I-60',
        captured_utc=datetime.now(timezone.utc).isoformat(),
        chronology='Post-primary family results known, before any selected-point distance arithmetic.',
        inputs=[dict(path=str(p), sha256=sha(p)) for p in paths],
        no_new_inference=True, no_new_physics=True))
    all_points = [];cases = {}
    for record in records:
        case = record['case'];context = load(record['feature']/'measurement_context.npz')
        scores = load(record['feature']/'scores.npz');evaluation = load(record['evaluation'])
        with np.load(record['original']/'precision.npz', allow_pickle=False) as raw:
            equivalent = raw['truth_equivalent_point_m'].copy()
            patch_points = raw['truth_patch_points_m'].copy()
            counts = raw['truth_patch_contact_counts'].copy()
            native = raw['exact_estimated_contact_point_m'].copy()
        if not np.array_equal(native, context['exact_estimated_contact_point_m']):
            raise ValueError('Native point differs from the bound feature context')
        core = module(record['source']);model = mujoco.MjModel.from_xml_path(str(record['xml']))
        model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        geometry = core.ContactHypothesisFeatures(model)
        qpos = context['exact_qpos'];normal_hint = context['intrinsic_normal_hint'];rows = len(qpos)
        chosen = np.stack([scores[m+'__chosen_family'] for m in MODES], axis=1)
        accepted = np.stack([scores[m+'__accepted_family'] >= 0 for m in MODES], axis=1)
        source_index = np.full((rows, 4), -1, np.int16)
        selected = np.full((rows, 4, 3), np.nan);centre = np.empty((rows, 3))
        for i in range(rows):
            data = geometry.data;data.qpos[:] = qpos[i]
            data.qpos[3:7] /= np.linalg.norm(data.qpos[3:7])
            mujoco.mj_kinematics(model, data)
            centre[i] = data.xpos[geometry.centre_body]
            normal = core._unit(normal_hint[i])
            if normal is None:
                continue
            seeds = core.normal_seeds(normal, geometry.cone_rad)
            for mode_index, mode in enumerate(MODES):
                label = int(chosen[i, mode_index])
                if label < 0:
                    continue
                index = int(scores[mode+'__candidate_index'][i, label])
                if index < 0 or index >= 102 or geometry.family_index[index] != label:
                    raise ValueError('Selected original candidate identity is invalid')
                seed_index, kind_index = divmod(index, 6)
                patch = geometry.patches[label]
                axis = patch['sign']*data.xmat[patch['body']].reshape(3, 3)[:, 0]
                upper = patch['lower'] if kind_index >= 4 else patch['upper']
                offset = core._band_projection(-geometry.radius*seeds[seed_index], axis, geometry.radius,
                                               patch['lower'], upper)
                selected[i, mode_index] = centre[i]+offset;source_index[i, mode_index] = index
        labels = evaluation['truth_family']
        dominant = patch_points[np.arange(rows), np.maximum(labels, 0)].copy()
        dominant[labels < 0] = np.nan
        targets = np.stack([equivalent, dominant, radial_proxy(equivalent, centre, geometry.radius),
                            radial_proxy(dominant, centre, geometry.radius)], axis=1)
        points = dict(time_s=context['time_s'], native_point_m=native, candidate_point_m=selected,
            centre_m=centre, chosen_family=chosen, accepted=accepted,
            original_candidate_index=source_index, targets_m=targets,
            truth_family=labels, primary_eligible=evaluation['primary_mask']&evaluation['truth_eligible'],
            strict_one_physical_point=counts.sum(axis=1) == 1)
        np.savez_compressed(output/(case+'_selected_points.npz'), **points)
        cases[case] = summarize(points);all_points.append(points)
    aggregate = {key: np.concatenate([p[key] for p in all_points]) for key in all_points[0]}
    report = dict(iteration='I-60', chronology='Descriptive post-primary point check; no acceptance or controller change.',
        targets=list(TARGETS), subsets=list(SUBSETS), cases=cases, aggregate=summarize(aggregate),
        qualification='Raw targets are sampled last-1ms load-weighted means; radial-R points are comparison proxies. Strict one-point subset retained. These do not prove future feedforward or control improvement.',
        no_new_physics=True, no_estimator_or_score_change=True)
    save(output/'report.json', report)
    with (output/'aggregate_table.csv').open('w', newline='') as handle:
        writer = csv.writer(handle);writer.writerow(['mode', 'target', 'subset', 'count', 'native_mean_mm',
            'candidate_mean_mm', 'native_RMS_mm', 'candidate_RMS_mm', 'movement_mean_mm'])
        for mode in MODES:
            for target in TARGETS:
                for subset in SUBSETS:
                    row = report['aggregate'][mode][target][subset]
                    writer.writerow([mode, target, subset, row['matched_count'], row['native']['mean_mm'],
                        row['candidate']['mean_mm'], row['native']['RMS_mm'], row['candidate']['RMS_mm'],
                        row['movement_from_native']['mean_mm']])
    for item in json.loads((output/'registered_before_point_arithmetic.json').read_text())['inputs']:
        if sha(item['path']) != item['sha256']:
            raise ValueError('Point-check source/input changed: '+item['path'])
    save(output/'artifact_manifest.json', dict(files=[dict(path=p.name, sha256=sha(p), bytes=p.stat().st_size)
        for p in sorted(output.iterdir()) if p.is_file() and p.name != 'artifact_manifest.json']))
    print(json.dumps(dict(cases=len(cases), primary_eligible=int(aggregate['primary_eligible'].sum()), no_new_physics=True)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args();execute(args.output.resolve())
