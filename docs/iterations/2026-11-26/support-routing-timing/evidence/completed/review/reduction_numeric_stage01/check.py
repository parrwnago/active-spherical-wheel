"""Independent saved-array arithmetic; does not import the canonical reducer."""
from pathlib import Path
import csv
import hashlib
import json

import numpy as np

STAGE = Path(__file__).resolve().parent
FAMILY = STAGE.parents[1]
ROOT = FAMILY.parents[1]
DT = .005


def sha(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


def load(path):
    with np.load(path) as z:
        return {k: z[k].copy() for k in z.files}


def normalize(loads):
    total = np.sum(loads, axis=1)
    valid = np.all(np.isfinite(loads), axis=1) & np.all(loads >= 0, axis=1) & (total > .001)
    fractions = np.zeros_like(loads)
    fractions[valid] = loads[valid]/total[valid, None]
    labels = np.argmax(fractions, axis=1)
    tied = np.sum(np.abs(fractions-fractions.max(axis=1)[:, None]) <= 1e-12, axis=1) > 1
    labels[~valid] = -1
    labels[valid & tied] = -2
    return fractions, valid, labels, total


def events(times, fractions, valid):
    labels = np.full(len(times), -1)
    labels[valid & (fractions[:, :2].sum(axis=1) >= .8)] = 0
    labels[valid & (fractions[:, 2:].sum(axis=1) >= .8)] = 1
    runs, start = [], 0
    for stop in range(1, len(times)+1):
        end = stop == len(times) or labels[stop] != labels[start] or times[stop]-times[stop-1] > DT+1e-9
        if end:
            if labels[start] >= 0 and times[stop-1]-times[start] >= .015-1e-10:
                runs.append((float(times[start]), float(times[stop-1]), int(labels[start])))
            start = stop
    output = []
    for before, after in zip(runs[:-1], runs[1:]):
        if after[0]-before[1] <= .05+1e-10 and after[2] != before[2]:
            output.append(dict(time_s=after[0], direction=('shell_to_belt' if after[2] else 'belt_to_shell')))
    return output


def pair(truth, estimate):
    used, result = set(), {}
    for i, event in enumerate(truth):
        choices = [(j, float(other['time_s']-event['time_s'])) for j, other in enumerate(estimate)
            if j not in used and other['direction'] == event['direction']
            and -.1-1e-10 <= other['time_s']-event['time_s'] <= .15+1e-10]
        if choices:
            nearest = min(abs(delta) for _, delta in choices)
            choices = [(j, delta) for j, delta in choices if abs(delta) <= nearest+1e-10]
            j, delta = min(choices, key=lambda item: (estimate[item[0]]['time_s'], item[0]))
            used.add(j)
            result[i] = delta
    return result, used


def main():
    declaration = json.loads((STAGE/'declaration_before_execution.json').read_text())
    for item in declaration['inputs']:
        assert sha(ROOT/item['path']) == item['sha256']
    canonical = json.loads((FAMILY/'analysis/stage01/report.json').read_text())
    protocol = json.loads((FAMILY/'protocol.json').read_text())
    folders = {'historical_mars_figure8': FAMILY/'campaign/historical_stage01',
        'mars_compact_outback': FAMILY/'campaign/runs/mars_compact_outback',
        'flat_compact_outback': FAMILY/'campaign/runs/flat_compact_outback_setup_corrected'}
    checks, cases, all_common, fresh_common = [], {}, [], []
    def compare(name, actual, expected, atol=1e-12):
        passed = bool(np.allclose(actual, expected, atol=atol, rtol=0, equal_nan=True))
        checks.append(dict(name=name, passed=passed, actual=np.asarray(actual).tolist(), expected=np.asarray(expected).tolist()))
    for case_id, folder in folders.items():
        raw = load(folder/'precision.npz')
        t = raw['time_s']
        original = case_id == 'historical_mars_figure8'
        window = protocol['evaluation']['historical_window_s' if original else 'fresh_window_s']
        if original:
            provenance = json.loads((ROOT/protocol['inputs']['historical_provenance']).read_text())
            truth_load = raw['truth_patch_loads_N']
            truth_valid = np.ones(len(t), bool)
            with np.load(ROOT/protocol['inputs']['historical_rollout']) as z:
                body = z['fork_load_N'].astype(float)
            convention = 'last1ms'
        else:
            provenance = json.loads((folder/'provenance.json').read_text())
            truth_load, truth_valid = raw['truth_interval_mean_loads_N'], raw['truth_interval_valid']
            body = raw['truth_interval_mean_body_load_N']
            convention = 'preceding_mean'
        weight = provenance['weight_N']
        true_share, positive, true_label, total = normalize(truth_load)
        eligible = truth_valid & positive & (total >= max(.05, .05*weight)) & np.isfinite(body) & (body <= .05*weight)
        unique = eligible & (true_share.max(axis=1) >= .8)
        mask = (t >= window[0]-1e-9) & (t < window[1]-1e-9) & (t+DT <= t[-1]+1e-9)
        true_events = events(t, true_share, eligible)
        table = canonical['cases'][case_id]['truth_tables'][convention]
        compare(case_id+'/eligible_count', np.sum(mask & eligible), table['primary_loaded_samples'])
        compare(case_id+'/unique_count', np.sum(mask & unique), table['true_unique_patch_samples'])
        case_result, matches = {}, {}
        for source, key, command_prefix in [('native','native_feedback_effective_loads_N','native'),
                                           ('fast','shadow_used_feedback_effective_loads_N','shadow')]:
            fractions, valid, label, _ = normalize(raw[key])
            wrong = mask & unique & (label != true_label)
            score = np.sum(abs(fractions-true_share), axis=1)/2
            score[~valid] = 1.
            selected = mask & eligible
            edges = selected[1:] & selected[:-1] & valid[1:] & valid[:-1] & (np.diff(t) <= DT+1e-9)
            variation = np.sum(np.abs(np.diff(fractions,axis=0)), axis=1)/2
            expected = table['sources'][source]
            mismatch = int(np.sum(wrong))*DT
            compare(case_id+'/'+source+'/mismatch_s', mismatch, expected['routing_mismatch_equivalent_s'])
            compare(case_id+'/'+source+'/mean_share_error', np.mean(score[selected]), expected['share_error_with_missing_penalty']['mean'])
            compare(case_id+'/'+source+'/adjacent_valid_TV', np.sum(variation[edges]), expected['share_total_variation_adjacent_valid'])
            command_edges = mask[1:] & mask[:-1] & (np.diff(t) <= DT+1e-9)
            maximum = {}
            for view, suffix in [('requested','command'), ('clipped','clipped_command')]:
                steps = np.max(np.abs(np.diff(raw[command_prefix+'_pre_sync_'+suffix],axis=0))/raw['input_bounds'][1:], axis=1)
                maximum[view] = float(np.max(steps[command_edges]))
                compare(case_id+'/'+source+'/'+view+'_max_step', maximum[view], canonical['cases'][case_id]['commands'][source][view]['normalized_adjacent_step']['maximum'])
            estimated_events = events(t, fractions, valid)
            paired, used = pair(true_events, estimated_events)
            matches[source] = paired
            extra = [j for j, e in enumerate(estimated_events) if j not in used and window[0]-1e-9 <= e['time_s'] < window[1]-1e-9]
            compare(case_id+'/'+source+'/extra_events', len(extra), len(expected['extra_estimated_events']))
            case_result[source] = dict(mismatch_s=mismatch, share_TV=float(np.sum(variation[edges])), maximum_normalized_step=maximum)
        common = []
        for i in sorted(set(matches['native']) & set(matches['fast'])):
            event = true_events[i]
            if window[0]-1e-9 <= event['time_s'] < window[1]-1e-9:
                row = dict(case_id=case_id, truth_convention=convention, truth_index=i, truth_time_s=event['time_s'],
                    direction=event['direction'], native_signed_offset_s=matches['native'][i], fast_signed_offset_s=matches['fast'][i],
                    absolute_timing_improvement_s=abs(matches['native'][i])-abs(matches['fast'][i]))
                common.append(row); all_common.append(row)
                if not original:
                    fresh_common.append(row)
        compare(case_id+'/common_event_count', len(common), len(table['common_paired_events']))
        for own, published in zip(common, table['common_paired_events']):
            assert own['direction'] == published['direction'] and own['truth_index'] == published['truth_index']
            for key in ('truth_time_s','native_signed_offset_s','fast_signed_offset_s','absolute_timing_improvement_s'):
                compare(case_id+'/event/'+str(own['truth_index'])+'/'+key, own[key], published[key])
        case_result['common_events'] = common
        cases[case_id] = case_result
        if original:
            _, nv, nl, _ = normalize(raw['native_feedback_effective_loads_N'])
            _, fv, fl, _ = normalize(raw['shadow_used_feedback_effective_loads_N'])
            before_departure = (t >= window[0]-1e-9) & (t <= 6.86+1e-9)
            sat = np.flatnonzero(before_departure & (abs(raw['exact_native_requested'][:,3]) >= raw['input_bounds'][:,3]-1e-12))
            corrected = np.flatnonzero(before_departure & unique & (nl != true_label) & (fl == true_label))
            first_sat, first_cue = float(t[sat[0]]), float(t[corrected[0]])
            compare('historical/first_saturation', first_sat, canonical['historical_causality']['first_native_belt_A_request_saturation_s'])
            compare('historical/first_correcting_route', first_cue, canonical['historical_causality']['first_truth_correcting_effective_route_s'])
            cases[case_id]['causality'] = dict(first_sat_s=first_sat, first_cue_s=first_cue, cue_is_later=first_cue>first_sat)
    compare('fresh/common_event_count', len(fresh_common), canonical['registered_gate']['paired_timing_improvement_s']['n'])
    median = float(np.median([r['absolute_timing_improvement_s'] for r in fresh_common]))
    compare('fresh/common_median_timing_benefit', median, canonical['registered_gate']['paired_timing_improvement_s']['median'])
    with (FAMILY/'analysis/stage01/paired_events.csv').open() as stream:
        csv_rows = [r for r in csv.DictReader(stream) if r['truth_convention'] == ('last1ms' if r['case_id']=='historical_mars_figure8' else 'preceding_mean')]
    compare('CSV/primary_common_event_rows', len(csv_rows), len(all_common))
    noise = load(FAMILY/'analysis/stage01/noise_samples.npz')
    noise_summary = {}
    for k, case_id in enumerate(noise['case_ids']):
        selected = noise['case_number'] == k
        noise_summary[str(case_id)] = {}
        for si, source in enumerate(('native','fast')):
            flip = float(np.mean(noise['labels'][selected,si] != noise['base_labels'][selected,si,None]))
            invalid = float(np.mean(noise['invalid'][selected,si]))
            compare(str(case_id)+'/'+source+'/saved_noise_flip', flip, canonical['cases'][str(case_id)]['noise']['sources'][source]['label_flip_fraction'])
            compare(str(case_id)+'/'+source+'/saved_noise_invalid', invalid, canonical['cases'][str(case_id)]['noise']['sources'][source]['invalid_fraction'])
            noise_summary[str(case_id)][source] = dict(flip=flip, invalid=invalid)
    gate = protocol['registered_gate']
    benefit = []
    robust = []
    for case_id in ('mars_compact_outback','flat_compact_outback'):
        a,b = (cases[case_id][s]['mismatch_s'] for s in ('native','fast'))
        benefit.append(a-b >= gate['wrong_patch_duration_absolute_reduction_s_min']-1e-12 and a>0 and (a-b)/a >= gate['wrong_patch_duration_relative_reduction_min']-1e-12)
        increase = cases[case_id]['fast']['maximum_normalized_step']['requested']-cases[case_id]['native']['maximum_normalized_step']['requested']
        noise_increase = noise_summary[case_id]['fast']['flip']-noise_summary[case_id]['native']['flip']
        robust.append(increase <= gate['max_normalized_command_step_increase']+1e-12 and noise_increase <= gate['max_noise_flip_fraction_increase']+1e-12)
    derived = dict(enough_common_paired_events=len(fresh_common)>=gate['minimum_paired_family_events_total'],
        paired_median_absolute_timing_improvement=median>=gate['minimum_median_abs_timing_improvement_s']-1e-12,
        each_case_wrong_duration_benefit=all(benefit), each_case_command_and_noise_nonregression=all(robust))
    for key,value in derived.items():
        compare('gate/'+key, value, canonical['registered_gate']['checks'][key])
    for item in declaration['inputs']:
        assert sha(ROOT/item['path']) == item['sha256']
    result = dict(passed=all(r['passed'] for r in checks), checks=checks, cases=cases,
        fresh_common_events=fresh_common, fresh_common_event_count=len(fresh_common), fresh_median_timing_improvement_s=median,
        independently_derived_gate_checks=derived, noise_saved_array_reduction=noise_summary,
        inputs_sha256=declaration['inputs'],
        scope='Direct saved-array arithmetic only; canonical reducer not imported. No controller, observer, scene, physics or full noise perturbation/locator campaign executed.')
    (STAGE/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'], checks=len(checks), common_fresh_events=len(fresh_common),median_timing_benefit_s=median)))


if __name__ == '__main__':
    main()
