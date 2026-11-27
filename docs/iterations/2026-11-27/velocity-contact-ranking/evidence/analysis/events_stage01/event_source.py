"""Explain the fixed I-60 event selections, without changing any ranking."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
FAMILIES = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
MODES = ('geometry', 'wrench', 'velocity', 'fused')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


def safe(value):
    if isinstance(value, dict):
        return {str(k): safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return safe(value.tolist())
    if isinstance(value, np.generic):
        return safe(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    return value


def write(path, value):
    path.write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def run(root, output):
    output.mkdir(parents=True, exist_ok=False)
    paths = [root/'protocol/event_inspection_before_truth.json', root/'protocol/protocol.json']
    for case in CASES:
        paths += [root/'campaign/features_stage01'/case/name for name in
                  ('projected_features.npz', 'measurement_context.npz', 'scores.npz')]
        paths.append(root/'analysis/stage01'/(case+'_evaluation.npz'))
    write(output/'registered_before_event_arithmetic.json', dict(
        captured_utc=datetime.now(timezone.utc).isoformat(), source_sha256=sha(Path(__file__)),
        prior='Primary ranking and post-primary point-summary results known; event-selection rule bound before truth outcomes.',
        inputs=[dict(path=str(path), sha256=sha(path)) for path in paths]))
    shutil.copy2(Path(__file__), output/'event_source.py')
    protocol = json.loads((root/'protocol/protocol.json').read_text())
    scale_n = np.deg2rad(protocol['scales']['normal_angle_deg'])
    scale_v = protocol['scales']['velocity_m_s']
    report = dict(iteration='I-60', cases={}, score_recomposition_max_abs=0.,
        qualification='These explain costs, not a unique physical cause or a tested compensating command.')
    for case in CASES:
        directory = root/'campaign/features_stage01'/case
        features, context, scores = (load(directory/name) for name in
                                    ('projected_features.npz', 'measurement_context.npz', 'scores.npz'))
        frames = load(root/'analysis/stage01'/(case+'_evaluation.npz'))
        eligible = frames['primary_mask'] & frames['truth_eligible']
        wrong = eligible & (frames['fused__accepted_family'] >= 0) & (
                frames['fused__accepted_family'] != frames['truth_family'])
        disagreement = eligible & (frames['fused__chosen_family'] != frames['native_family'])
        first = lambda mask: int(np.flatnonzero(mask)[0]) if mask.any() else None
        maximum = int(np.argmax(np.where(wrong, frames['fused__score_margin'], -np.inf))) if wrong.any() else None
        selections = {'earliest_native_disagreement': first(disagreement),
                      'earliest_accepted_wrong': first(wrong), 'maximum_margin_accepted_wrong': maximum}
        events = {}
        for rule, index in selections.items():
            if index is None:
                events[rule] = None
                continue
            event = dict(row=index, time_s=context['time_s'][index],
                truth_family=int(frames['truth_family'][index]), native_family=int(frames['native_family'][index]),
                fused_raw_family=int(frames['fused__chosen_family'][index]),
                fused_accepted_family=int(frames['fused__accepted_family'][index]),
                truth_loads_N=frames['truth_patch_loads_N'][index],
                estimated_loads_N=context['exact_estimated_support_loads_N'][index],
                body_velocity_world_m_s=context['exact_qvel'][index, :3],
                body_angular_velocity_fork_rad_s=context['exact_qvel'][index, 3:6],
                shaft_shell_joint_rates_rad_s=context['exact_qvel'][index, 6:9],
                previous_held_belt_speeds_m_s=context['exact_previous_applied'][index, 3:],
                observer_wrench=context['exact_observer_wrench'][index],
                motion_availability_m_s=context['measured_excitation_m_s'][index], modes={})
            for mode in MODES:
                candidates = {}
                for family, name in enumerate(FAMILIES):
                    candidate = int(scores[mode+'__candidate_index'][index, family])
                    if candidate < 0:
                        candidates[name] = None
                        continue
                    angle = features['normal_angle_rad'][index, candidate]
                    speed = features['velocity_residual_norm_m_s'][index, candidate]
                    moment = features['moment_residual_norm'][index, candidate]
                    g, v, w = (angle/scale_n)**2, (speed/scale_v)**2, moment**2
                    score = {'geometry': g, 'wrench': g+w, 'velocity': g+v, 'fused': g+w+v}[mode]
                    recorded = scores[mode+'__family_scores'][index, family]
                    discrepancy = abs(float(score-recorded))
                    report['score_recomposition_max_abs'] = max(report['score_recomposition_max_abs'], discrepancy)
                    assert np.isclose(score, recorded, rtol=1e-12, atol=1e-12)
                    candidates[name] = dict(candidate_id=candidate,
                        kind='rim' if features['kind'][candidate] else 'smooth_patch',
                        normal_offset_deg=np.rad2deg(angle), velocity_residual_m_s=speed,
                        velocity_residual_vector_m_s=features['velocity_residual_m_s'][index, candidate],
                        geometry_cost=g, velocity_cost=v, wrench_cost=w, complete_score=recorded)
                event['modes'][mode] = dict(chosen_family=scores[mode+'__chosen_family'][index],
                    accepted_family=scores[mode+'__accepted_family'][index],
                    score_margin=scores[mode+'__score_margin'][index],
                    best_velocity_residual_m_s=scores[mode+'__best_velocity_residual_m_s'][index],
                    residual_separation_m_s=scores[mode+'__residual_separation_m_s'][index],
                    candidates=candidates)
            events[rule] = event
        report['cases'][case] = events
    write(output/'events.json', report)
    write(output/'artifact_manifest.json', dict(files=[dict(path=p.name, sha256=sha(p), bytes=p.stat().st_size)
          for p in sorted(output.iterdir()) if p.is_file() and p.name != 'artifact_manifest.json']))
    print(json.dumps(dict(events={case: {rule: None if event is None else float(event['time_s'])
          for rule, event in events.items()} for case, events in report['cases'].items()},
          score_recomposition_max_abs=report['score_recomposition_max_abs'])))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.root.resolve(), args.output.resolve())
