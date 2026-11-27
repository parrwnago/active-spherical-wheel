"""Pure NumPy I-60 evaluation after all truth-blind score artifacts are sealed."""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import math
import shutil

import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
MODES = ('geometry', 'wrench', 'velocity', 'fused')
ROOT = Path(__file__).resolve().parents[3]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, dict):
        return {str(k): safe(v) for k,v in value.items()}
    if isinstance(value, (tuple, list)):
        return [safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return safe(value.tolist())
    if isinstance(value, np.generic):
        return safe(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def save(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def load_npz(path):
    with np.load(path, allow_pickle=False) as data:
        return {name:data[name].copy() for name in data.files}


def ratio(number, denominator):
    return float(number/denominator) if denominator else None


def mask_window(time_s, window):
    lo, hi = window
    return (time_s >= lo-1e-9)&(time_s < hi-1e-9)


def dominant_labels(loads, minimum_total, minimum_share):
    """Exactly tied or missing positive maxima remain unknown (-1)."""
    loads = np.asarray(loads, float)
    finite = np.isfinite(loads).all(axis=1)&(loads >= 0).all(axis=1)
    clean = np.where(np.isfinite(loads), loads, 0.)
    total, maximum = clean.sum(axis=1), clean.max(axis=1)
    unique = (clean == maximum[:,None]).sum(axis=1) == 1
    share = np.divide(maximum, total, out=np.zeros_like(total), where=total > 0)
    loaded = finite&(total >= minimum_total)&(maximum > 0)
    eligible = loaded&unique&(share >= minimum_share)
    family = np.where(eligible, np.argmax(clean, axis=1), -1)
    return family, eligible, loaded, share, unique, total


def stable_mask(labels, eligible, steps=2):
    result = np.zeros(len(labels), bool)
    for i in range(steps, len(labels)-steps):
        section = slice(i-steps, i+steps+1)
        result[i] = bool(eligible[section].all() and (labels[section] == labels[i]).all())
    return result


def count_switches(label, truth, eligible, window):
    comparison = window[:-1]&window[1:]&eligible[:-1]&eligible[1:]&(truth[:-1] == truth[1:])
    return int(np.sum(comparison&(label[:-1] != label[1:])))


def classification(label, truth, mask):
    count = int(mask.sum())
    errors = int(np.sum(mask&(label != truth)))
    return dict(count=count, correct_count=count-errors, error_count=errors,
        unknown_count=int(np.sum(mask&(label < 0))), error_fraction=ratio(errors, count),
        sampled_disagreement_equivalent_s=.005*errors)


def timing_events(time_s, truth, eligible, predicted):
    """Stable dominant-family runs; unknown gaps remain explicit brackets."""
    runs, start = [], 0
    labels = np.where(eligible, truth, -1)
    while start < len(labels):
        end = start+1
        while end < len(labels) and labels[end] == labels[start]:
            end += 1
        if labels[start] >= 0 and end-start >= 5:
            runs.append((start, end, int(labels[start])))
        start = end
    events = []
    for previous, current in zip(runs, runs[1:]):
        a, b = previous, current
        gap = float(time_s[b[0]]-time_s[a[1]-1]-.005)
        if a[2] == b[2] or gap > .100+1e-9:
            continue
        reference = float(time_s[b[0]])
        if not 1.-1e-9 <= reference < 41.-1e-9:
            continue
        event = dict(old_family=a[2], new_family=b[2],
            old_last_dominant_s=float(time_s[a[1]-1]), new_first_dominant_s=reference,
            unknown_or_ambiguous_gap_s=max(0.,gap),
            gap_qualification='Intervening unscored/transient samples may include a brief unstable third-family run.',
            reference_definition='First sample of new stable dominant family; not physical first touch.', methods={})
        events.append(event)
    for method, label in predicted.items():
        used = set()
        onsets = np.r_[True, label[1:] != label[:-1]]
        for event in events:
            reference, family = event['new_first_dominant_s'], event['new_family']
            possible = np.flatnonzero(onsets&(label == family)&(abs(time_s-reference) <= .100+1e-9))
            available = [int(i) for i in possible if int(i) not in used]
            if available:
                chosen = min(available, key=lambda i:(abs(float(time_s[i])-reference),i))
                used.add(chosen)
                offset = float(time_s[chosen])-reference
                event['methods'][method] = dict(onset_s=float(time_s[chosen]), offset_s=offset,
                    status='early' if offset < -1e-9 else 'late' if offset > 1e-9 else 'same_sample',
                    candidate_onsets_s=[float(time_s[i]) for i in possible],
                    multiple_candidates=bool(len(possible) > 1))
            else:
                event['methods'][method] = dict(onset_s=None, offset_s=None, status='missing',
                    candidate_onsets_s=[float(time_s[i]) for i in possible],
                    multiple_candidates=bool(len(possible) > 1))
    return events


def metrics_for_window(frames, mask):
    truth, eligible = frames['truth_family'], frames['truth_eligible']
    selected = mask&eligible
    native = frames['native_family']
    count = int(selected.sum())
    valid = frames['candidate_family_valid']
    safe_truth = np.maximum(truth, 0)
    covered = np.take_along_axis(valid, safe_truth[:,None], axis=1)[:,0]&eligible
    multiple = frames['valid_family_count'] >= 2
    motion = frames['measured_excitation_m_s'] >= .02
    ambiguous_motion = selected&multiple&motion
    native_metrics = classification(native, truth, selected)
    native_metrics['false_switch_count'] = count_switches(native, truth, eligible, mask)
    result = dict(counts=dict(window_rows=int(mask.sum()), dominant_eligible=count,
        truth_loaded=int(np.sum(mask&frames['truth_loaded'])),
        truth_loaded_nondominant_or_tied=int(np.sum(mask&frames['truth_loaded']&~eligible)),
        truth_unloaded_or_invalid=int(np.sum(mask&~frames['truth_loaded'])),
        candidate_no_family=int(np.sum(selected&(frames['valid_family_count'] == 0))),
        candidate_single_family=int(np.sum(selected&(frames['valid_family_count'] == 1))),
        candidate_multi_family=int(np.sum(selected&multiple)),
        motion_eligible=int(np.sum(selected&motion)),
        ambiguous_motion_eligible=int(ambiguous_motion.sum()),
        physical_solver_point_count_above_one=int(np.sum(mask&(frames['truth_patch_contact_counts'].sum(axis=1)>1)))),
        candidate_coverage=dict(covered_count=int(np.sum(selected&covered)), denominator=count,
            fraction=ratio(int(np.sum(selected&covered)),count)), native=native_metrics, modes={})
    for mode in MODES:
        label = frames[mode+'__chosen_family']
        accepted_label = frames[mode+'__accepted_family']
        accepted = selected&(accepted_label >= 0)
        accepted_metrics = classification(accepted_label, truth, accepted)
        baseline_same = classification(native, truth, accepted)
        sorted_scores = np.sort(frames[mode+'__family_scores'], axis=1)
        tied = np.isfinite(sorted_scores[:,0])&(sorted_scores[:,0] == sorted_scores[:,1])
        forced = selected&(frames['valid_family_count'] == 1)
        raw = classification(label, truth, selected)
        raw['false_switch_count'] = count_switches(label, truth, eligible, mask)
        result['modes'][mode] = dict(raw=raw, accepted=accepted_metrics,
            native_on_same_accepted_subset=baseline_same,
            accepted_count=int(accepted.sum()),
            accepted_ambiguous_motion_count=int(np.sum(accepted&ambiguous_motion)),
            accepted_motion_count=int(np.sum(accepted&motion)),
            accepted_coverage_all=ratio(int(accepted.sum()),count),
            accepted_coverage_motion=ratio(int(np.sum(accepted&motion)),int(np.sum(selected&motion))),
            accepted_coverage_ambiguous_motion=ratio(int(np.sum(accepted&ambiguous_motion)),int(ambiguous_motion.sum())),
            geometry_forced=classification(label,truth,forced),
            score_tie_count=int(np.sum(selected&tied)))
    return result


def evaluate_case(context, scores, truth_loads, truth_counts, protocol):
    time_s = context['time_s']
    evaluation = protocol['evaluation']
    truth, eligible, loaded, share, unique, total = dominant_labels(truth_loads,
        evaluation['truth_minimum_total_load_N'], evaluation['truth_minimum_dominant_fraction'])
    native, _, _, _, native_unique, _ = dominant_labels(context['exact_estimated_support_loads_N'],
        np.finfo(float).tiny, 0.)
    family_valid = np.isfinite(scores['fused__family_scores'])
    frames = dict(time_s=time_s, native_family=native, truth_family=truth,
        truth_eligible=eligible, truth_loaded=loaded, truth_dominant_fraction=share,
        truth_unique_max=unique, truth_patch_loads_N=truth_loads,
        truth_patch_contact_counts=truth_counts, native_unique_max=native_unique,
        candidate_family_valid=family_valid, valid_family_count=family_valid.sum(axis=1),
        measured_excitation_m_s=context['measured_excitation_m_s'], **scores)
    frames['primary_mask'] = mask_window(time_s,evaluation['primary_window_s'])
    frames['stable_truth_mask'] = stable_mask(truth,eligible)
    windows = {'primary': evaluation['primary_window_s'], **evaluation['secondary_windows_s']}
    report = dict(windows={name:metrics_for_window(frames,mask_window(time_s,window))
                          for name,window in windows.items()},
        timing=timing_events(time_s,truth,eligible,
            {'native':native, **{m:scores[m+'__chosen_family'] for m in MODES}}),
        controls=dict(time_grid_rows=len(time_s), primary_rows=int(frames['primary_mask'].sum()),
            terminal_not_in_evaluated_windows=bool(not mask_window(time_s,[0.,44.])[-1]),
            no_candidate_is_in_primary_denominator=True,
            dominant_support_not_unique_physical_contact=True))
    report['windows']['stable_primary'] = metrics_for_window(frames,frames['primary_mask']&frames['stable_truth_mask'])
    if report['controls']['primary_rows'] != 8000:
        raise ValueError('Primary half-open window did not contain 8000 rows')
    return frames,report


def aggregate_primary(cases):
    primary = [value['windows']['primary'] for value in cases.values()]
    count = sum(v['counts']['dominant_eligible'] for v in primary)
    coverage = sum(v['candidate_coverage']['covered_count'] for v in primary)
    native_errors = sum(v['native']['error_count'] for v in primary)
    ambiguous = sum(v['counts']['ambiguous_motion_eligible'] for v in primary)
    motion = sum(v['counts']['motion_eligible'] for v in primary)
    result = dict(dominant_eligible_count=count,
        candidate_true_family_coverage=ratio(coverage,count), candidate_true_family_covered_count=coverage,
        native_error_count=native_errors,native_error_fraction=ratio(native_errors,count),
        ambiguous_motion_eligible_count=ambiguous,motion_eligible_count=motion,modes={})
    for mode in MODES:
        rows = [v['modes'][mode] for v in primary]
        errors = sum(v['raw']['error_count'] for v in rows)
        accepted = sum(v['accepted_count'] for v in rows)
        accepted_errors = sum(v['accepted']['error_count'] for v in rows)
        native_same = sum(v['native_on_same_accepted_subset']['error_count'] for v in rows)
        accepted_ambiguous = sum(v['accepted_ambiguous_motion_count'] for v in rows)
        accepted_motion = sum(v['accepted_motion_count'] for v in rows)
        result['modes'][mode] = dict(error_count=errors,error_fraction=ratio(errors,count),
            accepted_count=accepted,accepted_error_count=accepted_errors,
            accepted_error_fraction=ratio(accepted_errors,accepted),
            native_same_accepted_error_count=native_same,native_same_accepted_error_fraction=ratio(native_same,accepted),
            confidence_coverage_ambiguous_motion=ratio(accepted_ambiguous,ambiguous),
            confidence_coverage_all=ratio(accepted,count),confidence_coverage_motion=ratio(accepted_motion,motion))
    return result


def gate_report(aggregate,cases,protocol):
    config=protocol['gates'];fused=aggregate['modes']['fused'];wrench=aggregate['modes']['wrench']
    gates=[]
    def add(name,actual,threshold,passed):
        gates.append(dict(name=name,actual=actual,threshold=threshold,scorable=actual is not None,
                          passed=bool(actual is not None and passed)))
    coverage=aggregate['candidate_true_family_coverage']
    add('candidate_true_family_coverage',coverage,config['candidate_true_family_coverage_min'],
        coverage is not None and coverage>=config['candidate_true_family_coverage_min'])
    native_error=aggregate['native_error_fraction'];fused_error=fused['error_fraction'];wrench_error=wrench['error_fraction']
    improvement=None if native_error is None or fused_error is None else native_error-fused_error
    required=None if native_error is None else max(config['fused_native_error_reduction_absolute_min'],
        config['fused_native_error_reduction_relative_min']*native_error)
    add('fused_improves_native',improvement,required,
        improvement is not None and required is not None and improvement>=required)
    velocity_benefit=None if wrench_error is None or fused_error is None else wrench_error-fused_error
    add('velocity_adds_over_wrench',velocity_benefit,config['fused_wrench_error_reduction_absolute_min'],
        velocity_benefit is not None and velocity_benefit>=config['fused_wrench_error_reduction_absolute_min'])
    for case,report in cases.items():
        p=report['windows']['primary'];a=p['modes']['fused']['raw']['error_fraction'];b=p['native']['error_fraction']
        difference=None if a is None or b is None else a-b
        allowance=config['per_phase_fused_error_above_native_allowance']
        add(case+'_raw_error_nonregression',difference,allowance,difference is not None and difference<=allowance)
        switches=p['modes']['fused']['raw']['false_switch_count']-p['native']['false_switch_count']
        allowance=config['per_phase_false_switch_above_native_allowance_count']
        add(case+'_false_switch_nonregression',switches,allowance,switches<=allowance)
    coverage=fused['confidence_coverage_ambiguous_motion'];minimum=config['confidence_ambiguous_motion_eligible_coverage_min']
    add('confidence_coverage_ambiguous_motion',coverage,minimum,coverage is not None and coverage>=minimum)
    error=fused['accepted_error_fraction'];maximum=config['confidence_error_max']
    add('confidence_error',error,maximum,error is not None and error<=maximum)
    baseline=fused['native_same_accepted_error_fraction']
    difference=None if error is None or baseline is None else error-baseline
    allowance=config['confidence_error_above_native_same_accepted_subset_allowance']
    add('confidence_error_vs_native_same_subset',difference,allowance,difference is not None and difference<=allowance)
    return gates


def run(protocol_path,feature_root,output):
    protocol=json.loads(protocol_path.read_text())
    if protocol['iteration']!='I-60':raise ValueError('Expected I-60')
    output.mkdir(parents=True,exist_ok=False)
    bindings=[]
    for case in CASES:
        directory=feature_root/case
        manifest=directory/'artifact_manifest.json'
        data=json.loads(manifest.read_text())
        if not data.get('sealed_before_any_truth_read_utc'):raise ValueError('Missing feature/score seal')
        for item in data['files']:
            if sha(directory/item['path'])!=item['sha256']:raise ValueError('Changed sealed feature artifact')
        precision=ROOT/'results/balance_priority_20261126/campaign/runs'/case/'precision.npz'
        registration=json.loads((directory/'registered_before_arithmetic.json').read_text())
        original=next(item for item in registration['inputs']
                      if Path(item['path']).resolve()==precision.resolve())
        if sha(precision)!=original['sha256']:raise ValueError('Historical precision changed before truth read')
        bindings.append(dict(case=case,feature_manifest_sha256=sha(manifest),
            precision_path=str(precision),precision_sha256=original['sha256']))
    # This registration and all three manifests precede opening any truth array.
    save(output/'registered_before_truth_read.json',dict(iteration='I-60',
        registered_utc=datetime.now(timezone.utc).isoformat(),analysis_source_sha256=sha(Path(__file__)),
        protocol_sha256=sha(protocol_path),features=bindings,
        timing_definition='Stable dominant-family runs >=25ms on both sides, <=100ms ambiguous gap; newfirst reference, nearest unused inferred onset within +/-100ms; descriptive only.'))
    shutil.copyfile(Path(__file__),output/'evaluate_scores.py')
    shutil.copyfile(protocol_path,output/'protocol.json')
    cases={}
    for binding in bindings:
        case=binding['case'];directory=feature_root/case
        context=load_npz(directory/'measurement_context.npz');scores=load_npz(directory/'scores.npz')
        with np.load(binding['precision_path'],allow_pickle=False) as original:
            truth_time=original['truth_time_s'].copy()
            truth_loads=original['truth_patch_loads_N'].copy()
            truth_counts=original['truth_patch_contact_counts'].copy()
        if sha(binding['precision_path'])!=binding['precision_sha256']:
            raise ValueError('Historical precision changed during truth read')
        if not np.array_equal(truth_time,context['time_s']):raise ValueError('Truth/state timestamp mismatch')
        frames,report=evaluate_case(context,scores,truth_loads,truth_counts,protocol)
        np.savez_compressed(output/(case+'_evaluation.npz'),**frames)
        cases[case]=report
    aggregate=aggregate_primary(cases);gates=gate_report(aggregate,cases,protocol)
    science=dict(iteration='I-60',new_physics_steps=0,live_controller_changed=False,
        cases=cases,aggregate_primary=aggregate,gates=gates,
        all_gates_passed=all(g['passed'] for g in gates),failed_gates=[g['name'] for g in gates if not g['passed']],
        interpretation='Offline dominant-support ranking only; normal domain is observer-conditioned. No posterior probability, independent wrench replacement, full multicontact estimate or physical tracking benefit is established.',
        truth_clock=protocol['evaluation']['clock'],
        interval_convention='Half-open windows; counts and 5ms sampled disagreement-equivalent seconds, not continuous wrong-contact durations.')
    save(output/'science.json',science)
    save(output/'artifact_manifest.json',dict(iteration='I-60',files=[
        dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(all_gates_passed=science['all_gates_passed'],
        failed_gates=science['failed_gates'],aggregate_primary=science['aggregate_primary'])))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--feature-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    run(args.protocol.resolve(),args.feature_root.resolve(),args.output.resolve())
