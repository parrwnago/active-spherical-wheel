"""Independent post-primary I44 pair audit; saved-array arithmetic only."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

HERE = Path(__file__).resolve().parent
FAMILY = next(p for p in HERE.parents if p.name == 'terrain_memory_20261126')
PRIMARY = FAMILY / 'campaign/analyze_stage01'
SUPPLEMENT = FAMILY / 'analysis/matched_pairs_stage01'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')


def moments(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return dict(n=0, rmse=None, p90_abs=None, median_abs=None)
    return dict(n=len(a), rmse=float(np.sqrt(np.dot(a, a) / len(a))),
                p90_abs=float(np.quantile(np.abs(a), .9)),
                median_abs=float(np.quantile(np.abs(a), .5)))


def surface_errors(row):
    query, truth = row['surface_query'], row['surface_truth']
    if not query['supported'] or not truth['valid']:
        return None, None
    height = float(query['height_m'] - truth['point_m'][2])
    if truth['normal_ambiguous']:
        return height, None
    a, b = np.asarray(query['normal']), np.asarray(truth['normal'])
    angle = float(np.degrees(np.arctan2(np.linalg.norm(np.cross(a, b)), np.dot(a, b))))
    return height, angle


def point_error(row):
    if not row['future_point_eligible']:
        return None
    target = np.asarray(row['true_future_equivalent_point_m'])
    points = [np.asarray(c['point_m']) for c in row['contact_preview']['candidates']]
    return min((float(np.linalg.norm(p - target)) for p in points), default=None)


def main():
    if (HERE / 'report.json').exists():
        raise FileExistsError('Preserve completed stages')
    source = HERE / 'source'
    source.mkdir(exist_ok=False)
    origins = [Path(__file__), SUPPLEMENT / 'matched_pairs.py', FAMILY / 'protocol.json',
               FAMILY / 'protocol_clarifications.json']
    sources = []
    for path in origins:
        shutil.copyfile(path, source / path.name)
        sources.append(dict(origin=str(path), snapshot='source/' + path.name, sha256=digest(path)))
    inputs = [PRIMARY / 'predictions.json', PRIMARY / 'artifact_manifest.json',
              PRIMARY / 'report.json', SUPPLEMENT / 'report.json']
    save(HERE / 'declaration_before_qa.json', dict(
        declared_utc=datetime.now(timezone.utc).isoformat(),
        scope='Independent post-primary matching arithmetic; no rendering, controller, physics, or parameter changes.',
        rule='Compare both supported surfaces on identical case-anchor-horizon slots; compare both nonempty eligible candidate sets to identical saved future equivalent points. Recompute errors from raw coordinates/normals.',
        sources=sources, input_sha256={str(p.relative_to(FAMILY)): digest(p) for p in inputs}))

    for name, metadata in json.loads((PRIMARY / 'artifact_manifest.json').read_text()).items():
        assert digest(PRIMARY / name) == metadata['sha256']
    rows = json.loads((PRIMARY / 'predictions.json').read_text())
    saved = json.loads((SUPPLEMENT / 'report.json').read_text())
    assert saved['primary_artifact_sha256'] == digest(PRIMARY / 'predictions.json')
    by_key = {}
    for row in rows:
        key = row['anchor_index'], row['horizon_s']
        arms = by_key.setdefault(key, {})
        assert row['arm'] not in arms
        arms[row['arm']] = row
    assert all(set(arms) == {'observer_plane', 'latest_four', 'memory_four'} for arms in by_key.values())
    strata = {}
    maximum_difference = 0.
    for stratum in ('uniform', 'dense_bump'):
        pairs = [arms for arms in by_key.values() if stratum in arms['observer_plane']['strata']]
        surfaces = [arms for arms in pairs if all(arms[a]['surface_query']['supported'] for a in ('observer_plane', 'memory_four'))]
        contacts = []
        for arms in pairs:
            a, b = arms['observer_plane'], arms['memory_four']
            assert a['future_index'] == b['future_index']
            assert a['true_future_equivalent_point_m'] == b['true_future_equivalent_point_m']
            ea, eb = point_error(a), point_error(b)
            if ea is not None and eb is not None:
                contacts.append((ea, eb))
        surface = {}
        for arm in ('observer_plane', 'memory_four'):
            values = [surface_errors(arms[arm]) for arms in surfaces]
            surface[arm] = dict(height_error_m=moments([v[0] for v in values if v[0] is not None]),
                                normal_error_deg=moments([v[1] for v in values if v[1] is not None]))
        errors = dict(observer_plane=moments([p[0] for p in contacts]), memory_four=moments([p[1] for p in contacts]))
        deltas = np.asarray([b - a for a, b in contacts])
        actual = dict(surface_pairs=len(surfaces), surface=surface,
                      actual_future_equivalent_point_pairs=len(contacts), nearest_candidate_error_m=errors,
                      memory_nearest_candidate_better_pairs=int(np.count_nonzero(deltas < 0)),
                      memory_nearest_candidate_worse_pairs=int(np.count_nonzero(deltas > 0)),
                      mean_memory_minus_observer_distance_m=float(np.mean(deltas)) if len(deltas) else None)
        def compare(found, expected):
            nonlocal maximum_difference
            for key, value in found.items():
                wanted = expected[key]
                if isinstance(value, dict):
                    compare(value, wanted)
                elif value is None:
                    assert wanted is None
                elif isinstance(value, int):
                    assert value == wanted
                else:
                    maximum_difference = max(maximum_difference, abs(value - wanted))
                    assert abs(value - wanted) < 1e-8
        compare(actual, saved['strata'][stratum])
        strata[stratum] = actual

    memory = sorted((r for r in rows if r['arm'] == 'memory_four'), key=lambda r: (r['anchor_time_s'], r['horizon_s']))
    first = next(r for r in memory if r['surface_query']['supported'])
    first_value = dict(anchor_time_s=first['anchor_time_s'], horizon_s=first['horizon_s'], first_capture_s=first['surface_query']['first_capture_s'])
    assert first_value == saved['first_supported_memory_query']
    dense = [dict(anchor_time_s=r['anchor_time_s'], horizon_s=r['horizon_s'], first_capture_s=r['surface_query']['first_capture_s'], contact_candidate_available=bool(r['candidate_patches']))
             for r in memory if 'dense_bump' in r['strata'] and r['surface_query']['supported']]
    assert dense == saved['dense_memory_supported_queries']
    common_all = sum(all(r['surface_query']['supported'] for r in arms.values()) for arms in by_key.values())
    assert common_all == saved['all_three_common_surface_queries']
    report = dict(iteration='I-44', passed=True, scope='Independent post-primary matched supplement QA; raw-coordinate arithmetic only.',
                  supplement_report_sha256=digest(SUPPLEMENT / 'report.json'),
                  predictions_sha256=digest(PRIMARY / 'predictions.json'),
                  shared_query_slots=len(by_key), maximum_statistic_disagreement=maximum_difference,
                  independent_strata=strata, first_supported_memory_query=first_value,
                  dense_memory_supported_queries=dense, all_three_common_surface_queries=common_all,
                  interpretation=[
                      'Matched surface RMSE and normal tail error improve, while median absolute height and normal errors worsen.',
                      'Nearest-candidate future-equivalent-point error worsens on the uniform paired sample; geometry-map utility does not establish contact/controller improvement.',
                      'Dense onset coverage is only three surface pairs and one contact pair.',
                      'Nearest-set errors are optimistic for ambiguous candidates and include CV motion error; normals are collision-mesh geometric proxies.',
                      'No parameters, primary artifacts, or physics were changed.'])
    save(HERE / 'report.json', report)
    paths = [p for p in HERE.rglob('*') if p.is_file() and p.name != 'artifact_manifest.json']
    save(HERE / 'artifact_manifest.json', dict(files=[dict(path=str(p.relative_to(HERE)), bytes=p.stat().st_size, sha256=digest(p)) for p in sorted(paths)]))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
