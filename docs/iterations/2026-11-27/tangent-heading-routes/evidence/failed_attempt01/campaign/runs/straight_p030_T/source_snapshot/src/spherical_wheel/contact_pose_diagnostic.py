"""Saved-state diagnostic for terrain versus configuration preview error.

This module does not execute a controller or obtain observations.  The caller
must hold providers fixed while all four conditions are generated.  Saved
future configurations are explicitly hindsight inputs, never deployable
predictions.  Contact records are accepted only by the separate scoring API.
"""
from __future__ import annotations

from collections import Counter

import numpy as np

from .terrain_memory import preview_contacts


PROVIDERS = ('observer_plane', 'memory_four')
ARMS = ('observer_cv', 'memory_cv', 'observer_actual_pose', 'memory_actual_pose')
LABELS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')


class AnchorProvider:
    """Read-only query facade that enforces the information clock, not pose time."""
    def __init__(self, provider, anchor_time_s):
        self._provider = provider
        self.anchor_time_s = float(anchor_time_s)
        if not np.isfinite(self.anchor_time_s) or self.anchor_time_s < 0:
            raise ValueError('Finite nonnegative anchor required')
        if hasattr(provider, 'settings'):
            self.settings = provider.settings
        self.query_count = 0

    def query(self, xy, *, at_time_s):
        if not np.isfinite(at_time_s) or abs(float(at_time_s)-self.anchor_time_s) > 1e-10:
            raise ValueError('Every provider query must use the frozen anchor clock')
        self.query_count += 1
        return self._provider.query(np.asarray(xy, float).copy(), at_time_s=self.anchor_time_s)


def endpoint_indices(times_s, anchor_time_s, horizons_s=(.05, .1, .2), tolerance_s=1e-8):
    """Nearest saved endpoint with an explicit time-match tolerance; no interpolation."""
    times = np.asarray(times_s, float)
    if (times.ndim != 1 or not len(times) or not np.isfinite(times).all()
            or np.any(np.diff(times) <= 0)):
        raise ValueError('Finite strictly increasing saved times required')
    if not np.isfinite(anchor_time_s) or anchor_time_s < 0 or not np.isfinite(tolerance_s) or tolerance_s < 0:
        raise ValueError('Finite nonnegative anchor and tolerance required')
    result = {}
    for horizon in horizons_s:
        horizon = float(horizon)
        if not np.isfinite(horizon) or horizon < 0 or horizon in result:
            raise ValueError('Distinct finite nonnegative horizons required')
        target = float(anchor_time_s)+horizon
        index = int(np.argmin(abs(times-target)))
        if abs(times[index]-target) > tolerance_s:
            raise ValueError('Saved endpoint is outside the registered timing tolerance')
        result[horizon] = index
    return result


def generate_pose_conditions(providers, geometry, qpos, qvel, anchor_time_s,
                             future_qpos_by_horizon, horizons_s=(.05, .1, .2),
                             band_tolerance_m=.001, max_gap_m=.005):
    """Return ``{arm: [preview_row, ...]}`` for the fixed two-by-two comparison.

    CV rows call the unchanged I-44 routine.  Oracle rows call the very same
    routine with a saved endpoint, zero velocity and zero horizon, then relabel
    only the endpoint clock and diagnostic metadata.  This preserves its
    quaternion normalization and geometry semantics without advancing the
    already-future configuration a second time.  No future observation,
    normal, contact identity, load, force, or evaluator is an argument.
    """
    if set(providers) != set(PROVIDERS):
        raise ValueError('Exactly observer_plane and memory_four providers required')
    horizons = tuple(float(h) for h in horizons_s)
    if (not horizons or len(set(horizons)) != len(horizons)
            or any(not np.isfinite(h) or h < 0 for h in horizons)):
        raise ValueError('Distinct finite nonnegative horizons required')
    if set(future_qpos_by_horizon) != set(horizons):
        raise ValueError('Exactly one saved future configuration per horizon required')
    future = {}
    for horizon in horizons:
        value = np.asarray(future_qpos_by_horizon[horizon], float)
        if value.shape != (10,) or not np.isfinite(value).all() or np.linalg.norm(value[3:7]) < 1e-12:
            raise ValueError('Each saved endpoint must be a finite complete qpos10')
        future[horizon] = value.copy()
    result = {}
    for name in PROVIDERS:
        provider = AnchorProvider(providers[name], anchor_time_s)
        cv = preview_contacts(provider, geometry, qpos, qvel, anchor_time_s,
                              horizons_s=horizons, band_tolerance_m=band_tolerance_m,
                              max_gap_m=max_gap_m)['rows']
        result[('observer' if name == 'observer_plane' else 'memory')+'_cv'] = [dict(row, pose_source='constant_velocity',
            uses_future_configuration=False, deployable_input_condition=True) for row in cv]
        oracle = []
        for horizon in horizons:
            row = preview_contacts(provider, geometry, future[horizon], np.zeros(9),
                anchor_time_s, horizons_s=(0.,), band_tolerance_m=band_tolerance_m,
                max_gap_m=max_gap_m)['rows'][0]
            row.update(horizon_s=horizon, time_s=float(anchor_time_s)+horizon,
                pose_source='saved_future_configuration_oracle', uses_future_configuration=True,
                deployable_input_condition=False, endpoint_evaluation_horizon_s=0., endpoint_quaternion_normalized_by_legacy_preview=True,
                assumption='Hindsight saved configuration; anchor-frozen estimated surface; geometry only')
            oracle.append(row)
        result[('observer' if name == 'observer_plane' else 'memory')+'_actual_pose'] = oracle
    return result


def classify_saved_contact_truth(loads_N, contact_counts, patch_points_m,
                                  equivalent_point_m, weight_N,
                                  other_patch_load_max_N=.05):
    """Classify evaluator-only saved records without claiming an exact contact point.

    Counts resolve records above .05 N; patch means include weaker records too.
    Thus the primary target is a *single resolved-contact patch mean*.
    """
    loads = np.asarray(loads_N, float)
    counts = np.asarray(contact_counts)
    points = np.asarray(patch_points_m, float)
    equivalent = np.asarray(equivalent_point_m, float)
    if (loads.shape != (4,) or counts.shape != (4,) or points.shape != (4, 3)
            or equivalent.shape != (3,) or not np.isfinite(loads).all()
            or np.any(loads < 0) or not np.isfinite(counts).all()
            or np.any(counts < 0) or np.any(counts != np.floor(counts))
            or not np.isfinite(weight_N) or weight_N <= 0
            or not np.isfinite(other_patch_load_max_N) or other_patch_load_max_N < 0):
        raise ValueError('Valid four-patch saved force/count/point records and weight required')
    threshold = max(.05, .05*float(weight_N))
    loaded = np.flatnonzero(loads >= threshold)
    positive = np.flatnonzero(loads > 0)
    resolved_count = int(counts.sum())
    patch = int(loaded[0]) if len(loaded) == 1 else None
    other_load = None if patch is None else float(np.sum(np.delete(loads, patch)))
    primary = bool(patch is not None and resolved_count == 1 and counts[patch] == 1
                   and np.isfinite(points[patch]).all()
                   and other_load <= other_patch_load_max_N)
    if primary:
        category = 'single_resolved_patch_mean'
    elif len(loaded) > 1:
        category = 'multiple_loaded_patches'
    elif patch is not None and resolved_count > 1:
        category = 'single_patch_multiple_resolved'
    elif patch is not None and other_load > other_patch_load_max_N:
        category = 'weak_or_unresolved'
    elif patch is not None:
        category = 'invalid_truth' if not np.isfinite(points[patch]).all() else 'weak_or_unresolved'
    elif len(positive):
        category = 'weak_or_unresolved'
    else:
        category = 'unloaded'
    return dict(category=category, primary_eligible=primary,
        primary_patch=LABELS[patch] if primary else None,
        primary_target_m=points[patch].copy() if primary else None,
        legacy_equivalent_eligible=bool(len(loaded) and np.isfinite(equivalent).all()),
        legacy_target_m=equivalent.copy() if len(loaded) and np.isfinite(equivalent).all() else None,
        loaded_patches=[LABELS[k] for k in loaded], positive_load_patches=[LABELS[k] for k in positive],
        resolved_contact_count=resolved_count, load_threshold_N=threshold,
        other_patch_load_N=other_load,
        qualification='Counts include only records above .05 N; patch means include all nonnegative-force records. Weak contacts on the same patch can remain.')


def score_contact_preview(preview, truth):
    """Score all eligible identities, retaining wrong-label candidate distances."""
    candidates = preview.get('candidates', [])
    patches = sorted({candidate['patch'] for candidate in candidates})
    def nearest(target):
        if target is None or not candidates:
            return None
        return min(float(np.linalg.norm(np.asarray(c['point_m'], float)-target)) for c in candidates)
    primary = bool(truth['primary_eligible'])
    legacy = bool(truth['legacy_equivalent_eligible'])
    return dict(contact_preview_complete=bool(preview.get('contact_prediction_supported', False)),
        nonempty_candidates=bool(candidates), candidate_patches=patches,
        candidate_cardinality=len(patches), unique_patch_count=len(patches),
        geometric_candidate_count=len(candidates), candidate_ambiguous=bool(preview.get('ambiguous', False)),
        primary_eligible=primary,
        primary_nearest_candidate_error_m=nearest(truth['primary_target_m']) if primary else None,
        primary_patch_in_candidates=bool(primary and preview.get('contact_prediction_supported', False) and truth['primary_patch'] in patches),
        legacy_equivalent_eligible=legacy,
        legacy_nearest_candidate_error_m=nearest(truth['legacy_target_m']) if legacy else None,
        any_loaded_patch_in_candidates=bool(set(patches)&set(truth['loaded_patches'])))


def common_fourway_rows(rows, target='primary'):
    """Fixed-key common eligibility requires complete AND nonempty all four arms."""
    if target not in ('primary', 'legacy'):
        raise ValueError('target must be primary or legacy')
    groups = {}
    eligible_key = 'primary_eligible' if target == 'primary' else 'legacy_equivalent_eligible'
    for row in rows:
        key = (int(row['anchor_index']), float(row['horizon_s']))
        group = groups.setdefault(key, {})
        if row['arm'] not in ARMS or row['arm'] in group:
            raise ValueError('Unknown or duplicate arm at a fixed query slot')
        group[row['arm']] = row
    selected = []
    excluded = Counter()
    for key, group in groups.items():
        if set(group) != set(ARMS):
            raise ValueError('Every fixed query slot must contain exactly four conditions')
        flags = {bool(row['contact_score'][eligible_key]) for row in group.values()}
        if len(flags) != 1:
            raise ValueError('Truth eligibility must be identical across conditions')
        if not flags.pop():
            excluded['truth_ineligible'] += 1
        elif not all(row['contact_score']['contact_preview_complete'] for row in group.values()):
            excluded['incomplete_preview'] += 1
        elif not all(row['contact_score']['nonempty_candidates'] for row in group.values()):
            excluded['empty_candidates'] += 1
        else:
            selected.append((key, group))
    return dict(groups=selected, query_slots=len(groups), matched_slots=len(selected), exclusions=dict(excluded))


def quality_gates(observer, memory, thresholds):
    """Predeclared paired quality gates; missing metrics always fail safely.

    Inputs contain rmse_m, p95_m, identity_recall and ambiguity_fraction, all
    computed on the SAME four-way fixed-key set.  Coverage is assessed outside
    this helper, because it uses all fixed slots rather than matched ones.
    """
    required = ('rmse_m', 'p95_m', 'identity_recall', 'ambiguity_fraction')
    valid = all(value.get(k) is not None and np.isfinite(value[k])
                for value in (observer, memory) for k in required)
    absolute = observer['rmse_m']-memory['rmse_m'] if valid else None
    relative = (absolute/observer['rmse_m'] if valid and observer['rmse_m'] > 0 else None)
    checks = dict(
        RMS_relative_improvement=bool(relative is not None and relative >= thresholds['RMS_relative_improvement_min']-1e-14),
        RMS_absolute_improvement=bool(valid and absolute >= thresholds['RMS_absolute_improvement_m_min']-1e-14),
        p95_error=bool(valid and memory['p95_m'] <= observer['p95_m']+thresholds['p95_error_increase_m_max']+1e-14),
        identity_recall=bool(valid and memory['identity_recall'] >= observer['identity_recall']-thresholds['matched_identity_recall_loss_max']-1e-14),
        ambiguity=bool(valid and memory['ambiguity_fraction'] <= observer['ambiguity_fraction']+thresholds['matched_ambiguity_fraction_increase_max']+1e-14))
    return dict(checks=checks, passed=all(checks.values()),
        absolute_RMS_improvement_m=absolute, relative_RMS_improvement=relative)


def diagnostic_decision(common_n, eligible_n, cv, oracle, memory_cv_coverage,
                         memory_oracle_coverage, thresholds):
    """Diagnostic prioritization only; no gate authorizes live integration."""
    if common_n < 0 or eligible_n < 0 or common_n > eligible_n:
        raise ValueError('Matched count must lie within fixed truth-eligible count')
    fraction = common_n/eligible_n if eligible_n else None
    support = dict(
        common_count=common_n >= thresholds['primary_common_min_n'],
        common_fraction=fraction is not None and fraction >= thresholds['primary_common_fraction_min'],
        memory_oracle_coverage=memory_oracle_coverage is not None and memory_oracle_coverage >= thresholds['uniform_memory_complete_nonempty_coverage_min']-1e-14,
        memory_coverage_loss=memory_oracle_coverage is not None and memory_cv_coverage is not None
            and memory_oracle_coverage >= memory_cv_coverage-thresholds['oracle_memory_coverage_loss_vs_CV_memory_max']-1e-14)
    cv_quality = quality_gates(cv['observer'], cv['memory'], thresholds)
    oracle_quality = quality_gates(oracle['observer'], oracle['memory'], thresholds)
    if not (support['common_count'] and support['common_fraction']):
        branch = 'inconclusive_insufficient_common_primary'
    elif (oracle['observer'].get('rmse_m') is not None and oracle['memory'].get('rmse_m') is not None
          and oracle['memory']['rmse_m'] > oracle['observer']['rmse_m']):
        branch = 'park_current_map_to_contact_integration_oracle_memory_worse'
    elif oracle_quality['passed'] and all(support.values()):
        branch = ('audit_metric_aggregation_and_stratification' if cv_quality['passed']
                  else 'investigate_motion_prediction_keep_LQR_unchanged')
    else:
        branch = 'insufficient_useful_advantage_no_integration'
    return dict(common_primary_n=int(common_n), fixed_primary_eligible_n=int(eligible_n),
        common_primary_fraction=fraction, support_checks=support,
        cv_quality=cv_quality, oracle_quality=oracle_quality, decision_branch=branch,
        interpretation='Saved-data diagnostic prioritization; no live-controller adoption or hardware claim.')


def compare_values(actual, expected, path='', atol=1e-12):
    """Compact recursive parity failures for preserved JSON/numpy preview records."""
    failures = []
    if isinstance(actual, np.ndarray):
        actual = actual.tolist()
    if isinstance(expected, np.ndarray):
        expected = expected.tolist()
    if isinstance(actual, dict) and isinstance(expected, dict):
        if set(actual) != set(expected):
            failures.append(path+':keys')
        for key in sorted(set(actual)&set(expected)):
            failures.extend(compare_values(actual[key], expected[key], f'{path}/{key}', atol))
    elif isinstance(actual, (list, tuple)) and isinstance(expected, (list, tuple)):
        if len(actual) != len(expected):
            failures.append(path+':length')
        for i, (a, b) in enumerate(zip(actual, expected)):
            failures.extend(compare_values(a, b, f'{path}/{i}', atol))
    elif actual is None or expected is None:
        if actual is not expected:
            failures.append(path+':null')
    elif isinstance(actual, (bool, np.bool_)) or isinstance(expected, (bool, np.bool_)):
        if bool(actual) != bool(expected):
            failures.append(path+':bool')
    elif isinstance(actual, (int, float, np.number)) and isinstance(expected, (int, float, np.number)):
        if not np.isclose(actual, expected, rtol=0., atol=atol, equal_nan=True):
            failures.append(path+':number')
    elif actual != expected:
        failures.append(path+':value')
    return failures
