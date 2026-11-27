"""Independent scalar per-frame I-60 rank audit; no inference helper import."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import numpy as np

MODES = ('geometry', 'wrench', 'velocity', 'fused')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path):
    with np.load(path, allow_pickle=False) as data:
        return {key: data[key].copy() for key in data.files}


def calculate(features, protocol):
    families = features['family_index']
    valid = features['valid']
    frames, count = valid.shape
    a = features['normal_angle_rad']
    v = features['velocity_residual_norm_m_s']
    w = features['moment_residual_norm']
    vectors = features['velocity_residual_m_s']
    frame_valid = features['frame_valid']
    excitation = features['excitation_m_s']
    assert families.shape == (count,) and vectors.shape == (frames, count, 3)
    finite = np.isfinite(a)&np.isfinite(v)&np.isfinite(w)&np.isfinite(vectors).all(axis=2)
    finite &= (a >= 0)&(v >= 0)&(w >= 0)
    active = valid&finite&frame_valid[:, None]&np.isfinite(excitation)[:, None]
    np.testing.assert_allclose(np.linalg.norm(vectors, axis=2)[active], v[active],
                               rtol=1e-10, atol=1e-12)
    normal = (a/math.radians(protocol['scales']['normal_angle_deg']))**2
    velocity = (v/protocol['scales']['velocity_m_s'])**2
    wrench = w*w
    costs = (normal, normal+wrench, normal+velocity, normal+wrench+velocity)
    thresholds = protocol['confidence']
    output = {}
    for mode, cost in zip(MODES, costs):
        result = {
            'family_scores': np.full((frames, 4), np.inf),
            'candidate_index': np.full((frames, 4), -1, int),
            'chosen_family': np.full(frames, -1, int),
            'accepted_family': np.full(frames, -1, int),
            'score_margin': np.full(frames, np.nan),
            'valid_family_count': np.zeros(frames, int),
            'confident': np.zeros(frames, bool),
            'score_tied': np.zeros(frames, bool),
            'geometry_forced': np.zeros(frames, bool),
            'best_velocity_residual_m_s': np.full(frames, np.nan),
            'residual_separation_m_s': np.full(frames, np.nan),
            'excitation_m_s': excitation.copy(),
        }
        for row in range(frames):
            choices = []
            for family in range(4):
                slots = np.flatnonzero(active[row]&(families == family))
                if not len(slots):
                    continue
                # Explicit scalar ordering independently fixes same-family ties.
                index = min(slots, key=lambda index: (float(cost[row, index]), int(index)))
                score = float(cost[row, index])
                if not math.isfinite(score):
                    continue
                result['family_scores'][row, family] = score
                result['candidate_index'][row, family] = index
                choices.append((score, family, index))
            choices.sort()
            number = len(choices)
            result['valid_family_count'][row] = number
            result['geometry_forced'][row] = number == 1
            if not number:
                continue
            best_score, chosen, first = choices[0]
            best = math.sqrt(float(np.sum(vectors[row, first]**2)))
            result['chosen_family'][row] = chosen
            result['best_velocity_residual_m_s'][row] = best
            if number < 2:
                continue
            second_score, _, second = choices[1]
            gap = second_score-best_score
            separated = math.sqrt(float(np.sum((vectors[row, first]-vectors[row, second])**2)))
            tied = first_score_tie = best_score == second_score
            result['score_margin'][row] = gap
            result['score_tied'][row] = tied
            result['residual_separation_m_s'][row] = separated
            accepted = (bool(frame_valid[row]) and not first_score_tie
                        and excitation[row] >= thresholds['minimum_excitation_m_s']
                        and gap >= thresholds['minimum_runnerup_score_gap']
                        and best <= thresholds['maximum_best_velocity_residual_m_s']
                        and separated >= thresholds['minimum_best_runnerup_velocity_vector_separation_m_s'])
            if accepted:
                result['accepted_family'][row] = chosen
                result['confident'][row] = True
        output[mode] = result
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--features-root', type=Path, required=True)
    parser.add_argument('--protocol', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    protocol = json.loads(args.protocol.read_text())
    report = {'iteration': 'I-60', 'physics_runs': 0,
              'completed_utc': datetime.now(timezone.utc).isoformat(),
              'scope': 'Rank arithmetic only; no solver truth, helper import or trajectory inference',
              'protocol_sha256': sha(args.protocol), 'cases': {}}
    for case in protocol['datasets']:
        folder = args.features_root/case
        manifest = json.loads((folder/'artifact_manifest.json').read_text())
        for item in manifest['files']:
            assert sha(folder/item['path']) == item['sha256'], item['path']
        features = load(folder/'projected_features.npz')
        saved = load(folder/'scores.npz')
        result = calculate(features, protocol)
        checks = {}
        for mode in MODES:
            for key, actual in result[mode].items():
                expected = saved[mode+'__'+key]
                assert actual.shape == expected.shape
                if actual.dtype.kind in 'biu':
                    exact = np.array_equal(actual, expected)
                    assert exact, (case, mode, key)
                    checks[mode+'__'+key] = {'exact': True}
                else:
                    assert np.array_equal(np.isnan(actual), np.isnan(expected))
                    assert np.array_equal(np.isposinf(actual), np.isposinf(expected))
                    assert np.array_equal(np.isneginf(actual), np.isneginf(expected))
                    mask = np.isfinite(expected)
                    difference = float(np.max(abs(actual[mask]-expected[mask]))) if mask.any() else 0.
                    np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-12)
                    checks[mode+'__'+key] = {'max_abs_difference': difference}
        report['cases'][case] = {'rows': len(features['valid']), 'checks': checks,
                                'projected_features_sha256': sha(folder/'projected_features.npz'),
                                'scores_sha256': sha(folder/'scores.npz')}
    report['all_checks_pass'] = True
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'all_checks_pass': True, 'cases': len(report['cases'])}))


if __name__ == '__main__':
    main()
