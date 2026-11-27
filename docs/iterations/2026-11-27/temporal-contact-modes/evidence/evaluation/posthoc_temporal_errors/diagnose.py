"""Post-result explanatory I70 row-set/spell diagnostic, not a new gate.

No force fit, threshold selection, state inference or parameter change.  Source
and inputs are registered before this diagnostic's new arithmetic.  All truth is
retrospective sampled positive-pressure support, not full force routing.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
METHODS = ('N', 'F', 'A', 'emission', 'state_hmm', 'hmm')
ERROR_NAMES = ('correct', 'unknown', 'pair', 'wrong_singleton')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    def convert(v):
        if isinstance(v, dict): return {str(k):convert(x) for k,x in v.items()}
        if isinstance(v, (list, tuple)): return [convert(x) for x in v]
        if isinstance(v, np.ndarray): return convert(v.tolist())
        if isinstance(v, np.generic): return convert(v.item())
        return v
    Path(path).write_text(json.dumps(convert(value), indent=2, allow_nan=False)+'\n')


def load(path):
    with np.load(path, allow_pickle=False) as packet:
        return {key:packet[key].copy() for key in packet.files}


def cardinality(bits):
    return np.sum((np.asarray(bits)[:, None] & (1 << np.arange(4))) != 0, axis=1)


def distribution(values):
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    if not len(a):
        return dict(count=0, mean=None, median=None, p95=None, maximum=None)
    return dict(count=len(a), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), maximum=float(a.max()))


def paired(old_correct, new_correct, mask):
    return dict(denominator=int(mask.sum()),
        both_correct=int(np.sum(mask & old_correct & new_correct)),
        both_wrong=int(np.sum(mask & ~old_correct & ~new_correct)),
        gained=int(np.sum(mask & ~old_correct & new_correct)),
        lost=int(np.sum(mask & old_correct & ~new_correct)),
        old_errors=int(np.sum(mask & ~old_correct)), new_errors=int(np.sum(mask & ~new_correct)))


def error_type(bits, correct, mask):
    output = np.full(len(bits), -1, np.int8)
    output[mask & correct] = 0
    output[mask & ~correct & (bits == 0)] = 1
    output[mask & ~correct & (cardinality(bits) == 2)] = 2
    output[mask & ~correct & (cardinality(bits) == 1)] = 3
    if np.any(mask & (output < 0)):
        raise ValueError('Unexpected prediction expressivity')
    return output


def spells(case, method, time, kinds, bits, truth_family, truth_bits, confidence):
    """Error runs constant in type, predicted support and sampled truth labels."""
    output, ids = [], np.full(len(time), -1, np.int32)
    start = 0
    while start < len(time):
        if kinds[start] <= 0:
            start += 1; continue
        end = start+1
        while (end < len(time) and kinds[end] == kinds[start] and bits[end] == bits[start]
               and truth_family[end] == truth_family[start] and truth_bits[end] == truth_bits[start]
               and abs(float(time[end]-time[end-1])-.005) <= 1e-9):
            end += 1
        ident = len(output); ids[start:end] = ident
        output.append(dict(case=case, method=method, id=ident, error_type=ERROR_NAMES[int(kinds[start])],
            start_row=start, end_row_exclusive=end, start_s=float(time[start]), last_sample_s=float(time[end-1]),
            sampled_duration_s=(end-start)*.005, rows=end-start,
            predicted_support_bits=int(bits[start]), truth_family=int(truth_family[start]),
            truth_support_bits=int(truth_bits[start]), confidence=distribution(confidence[start:end])))
        start = end
    return output, ids


def derived(case, frame, packet):
    if not np.array_equal(frame['time_s'], packet['time_s']):
        raise ValueError('Packet/frame clock mismatch')
    loads = packet['truth_patch_loads_N']
    valid = np.isfinite(loads).all(axis=1) & (loads >= 0).all(axis=1)
    total = np.sum(loads, axis=1)
    shares = np.divide(loads, total[:, None], out=np.zeros_like(loads), where=total[:, None] > 0)
    loaded = valid & (total >= 1.)
    truth_bits = np.sum((shares >= .1)*(1 << np.arange(4)), axis=1).astype(np.int8)
    truth_bits[~loaded] = 0
    if not np.array_equal(truth_bits, frame['truth_support_bits']) or not np.array_equal(loaded, frame['truth_loaded']):
        raise ValueError('Independent pressure-support reconstruction differs from frozen evaluator')
    truth_card = cardinality(truth_bits)
    primary = frame['primary_mask'] & frame['truth_dominant']
    full_loaded = frame['primary_mask'] & loaded
    non_dominant = full_loaded & ~frame['truth_dominant']
    Ncorrect, Acorrect = frame['N__correct_family'], frame['A__correct_family']
    useful = primary & ~Ncorrect & Acorrect
    harmful = primary & Ncorrect & ~Acorrect
    accepted_wrong = primary & packet['i60_accepted'] & ~Acorrect
    output = dict(time_s=frame['time_s'], primary_dominant=primary, primary_loaded=full_loaded,
        primary_loaded_nondominant=non_dominant, truth_family=frame['truth_family'],
        truth_support_bits=truth_bits, truth_support_cardinality=truth_card,
        original_useful=useful, original_harmful=harmful, original_accepted_wrong=accepted_wrong)
    rows, method_reports = [], {}
    for method in METHODS:
        bits, confidence = frame[method+'__bits'], frame[method+'__confidence']
        correct, correct_set = frame[method+'__correct_family'], frame[method+'__correct_set']
        kinds = error_type(bits, correct, primary)
        method_spells, ids = spells(case, method, frame['time_s'], kinds, bits,
                                  frame['truth_family'], truth_bits, confidence)
        rows += method_spells
        for key,value in dict(bits=bits, confidence=confidence, correct_singleton=correct,
                              correct_set=correct_set, error_type=kinds, error_spell_id=ids).items():
            output[method+'__'+key] = value
        errors = {}
        for kind in ERROR_NAMES[1:]:
            k = ERROR_NAMES.index(kind)
            mask = primary & (kinds == k)
            ss = [s for s in method_spells if s['error_type'] == kind]
            errors[kind] = dict(rows=int(mask.sum()), sampled_disagreement_equivalent_s=.005*int(mask.sum()),
                confidence=distribution(confidence[mask]), high_confidence_rows=int(np.sum(mask & (confidence >= .8))),
                spells=len(ss), spell_duration_s=distribution([s['sampled_duration_s'] for s in ss]))
        confident = primary & (confidence >= .8) & (bits != 0)
        method_reports[method] = dict(errors_by_type=errors,
            correct_family_count=int(np.sum(primary & correct)),
            high_confidence_wrong=int(np.sum(confident & ~correct)),
            unknown_high_confidence_error_rows=int(np.sum(primary & (bits == 0) & (confidence >= .8))),
            original_useful=dict(denominator=int(useful.sum()), retained=int(np.sum(useful & correct)), lost=int(np.sum(useful & ~correct))),
            original_harmful=dict(denominator=int(harmful.sum()), repaired=int(np.sum(harmful & correct))),
            original_accepted_wrong=dict(denominator=int(accepted_wrong.sum()), repaired=int(np.sum(accepted_wrong & correct))))
    comparison = {}
    for old,new in (('A','emission'),('A','state_hmm'),('A','hmm'),('emission','state_hmm'),('emission','hmm'),('state_hmm','hmm')):
        comparison[old+'_to_'+new] = dict(singleton=paired(frame[old+'__correct_family'], frame[new+'__correct_family'], primary),
            full_loaded_support=paired(frame[old+'__correct_set'], frame[new+'__correct_set'], full_loaded),
            nondominant_loaded_support=paired(frame[old+'__correct_set'], frame[new+'__correct_set'], non_dominant))
    truth_counts = {domain:{str(c):int(np.sum(mask & (truth_card == c))) for c in range(5)}
        for domain,mask in [('loaded_primary',full_loaded), ('dominant_primary',primary), ('nondominant_loaded_primary',non_dominant)]}
    return output, dict(truth_cardinality_counts=truth_counts, methods=method_reports, paired_comparisons=comparison), rows


def event_row(case, index, arrays):
    f = arrays[case]
    return dict(case=case, row=int(index), time_s=float(f['time_s'][index]),
        truth_family=int(f['truth_family'][index]), truth_support_bits=int(f['truth_support_bits'][index]),
        methods={m:dict(support_bits=int(f[m+'__bits'][index]), confidence=float(f[m+'__confidence'][index]),
            correct_singleton=bool(f[m+'__correct_singleton'][index]), correct_set=bool(f[m+'__correct_set'][index]),
            error_type=ERROR_NAMES[int(f[m+'__error_type'][index])]) for m in METHODS})


def combine(cases, arrays, all_spells):
    joined = {key:np.concatenate([arrays[c][key] for c in CASES]) for key in arrays[CASES[0]]}
    primary, loaded, nondominant = (joined[k] for k in ('primary_dominant','primary_loaded','primary_loaded_nondominant'))
    report = dict(truth_cardinality_counts={domain:{str(c):int(np.sum(mask & (joined['truth_support_cardinality'] == c))) for c in range(5)}
        for domain,mask in [('loaded_primary',loaded), ('dominant_primary',primary), ('nondominant_loaded_primary',nondominant)]},
        methods={}, paired_comparisons={})
    for m in METHODS:
        kind, conf, bits = (joined[m+'__'+key] for key in ('error_type','confidence','bits'))
        errors = {}
        for k,name in enumerate(ERROR_NAMES[1:],1):
            mask = primary & (kind == k)
            ss = [s for s in all_spells if s['method'] == m and s['error_type'] == name]
            errors[name] = dict(rows=int(mask.sum()), sampled_disagreement_equivalent_s=.005*int(mask.sum()),
                confidence=distribution(conf[mask]), high_confidence_rows=int(np.sum(mask & (conf >= .8))),
                spells=len(ss), spell_duration_s=distribution([s['sampled_duration_s'] for s in ss]))
        correct = joined[m+'__correct_singleton']
        report['methods'][m] = dict(errors_by_type=errors,
            correct_family_count=int(np.sum(primary & correct)),
            high_confidence_wrong=int(np.sum(primary & ~correct & (conf >= .8) & (bits != 0))),
            original_useful=dict(denominator=int(joined['original_useful'].sum()),
                retained=int(np.sum(joined['original_useful'] & correct)), lost=int(np.sum(joined['original_useful'] & ~correct))),
            original_harmful=dict(denominator=int(joined['original_harmful'].sum()), repaired=int(np.sum(joined['original_harmful'] & correct))),
            original_accepted_wrong=dict(denominator=int(joined['original_accepted_wrong'].sum()), repaired=int(np.sum(joined['original_accepted_wrong'] & correct))))
    for old,new in (('A','emission'),('A','state_hmm'),('A','hmm'),('emission','state_hmm'),('emission','hmm'),('state_hmm','hmm')):
        report['paired_comparisons'][old+'_to_'+new] = dict(singleton=paired(joined[old+'__correct_singleton'], joined[new+'__correct_singleton'], primary),
            full_loaded_support=paired(joined[old+'__correct_set'], joined[new+'__correct_set'], loaded),
            nondominant_loaded_support=paired(joined[old+'__correct_set'], joined[new+'__correct_set'], nondominant))
    examples = {}
    for method in ('emission','state_hmm','hmm'):
        ss = [s for s in all_spells if s['method'] == method and s['error_type'] == 'pair']
        if ss:
            chosen = min(ss, key=lambda s:(-s['rows'], CASES.index(s['case']), s['start_row']))
            examples[method+'_longest_erroneous_pair_spell'] = dict(spell=chosen,
                start=event_row(chosen['case'],chosen['start_row'],arrays),
                last=event_row(chosen['case'],chosen['end_row_exclusive']-1,arrays))
    for key,covered_only in [('hmm_highest_confidence_error',False), ('hmm_highest_confidence_nonunknown_error',True)]:
        candidates = []
        for c in CASES:
            f=arrays[c];mask=f['primary_dominant'] & ~f['hmm__correct_singleton']
            if covered_only:mask &= f['hmm__bits'] != 0
            for i in np.flatnonzero(mask):candidates.append((-float(f['hmm__confidence'][i]),CASES.index(c),int(i),c))
        if candidates:
            _,_,i,c=min(candidates)
            examples[key]=event_row(c,i,arrays)
    return report,examples


def run(evaluation_root, packet_root, output):
    output.mkdir(parents=True,exist_ok=False)
    paths=[Path(__file__), evaluation_root/'science.json']
    for case in CASES:paths += [evaluation_root/(case+'_frames.npz'),packet_root/case/'evaluation_packet.npz']
    bindings=[dict(path=str(p),sha256=sha(p)) for p in paths]
    write(output/'registered_before_diagnostic_arithmetic.json',dict(iteration='I-70-posthoc',
        utc=datetime.now(timezone.utc).isoformat(),status='Explanatory post-result diagnostic, not a preregistered acceptance gate.',
        fixed_before_this_arithmetic=['Truth support cardinalities counted directly from sampled pressure shares>=.1.',
            'Spells constant error kind/output support/truthfamily/truthsupport,5ms adjacency; duration=rows*.005.',
            'Examples longest erroneouspair pernewmethod, tiecaseorder thenearliestrow; highestconfidenceprimaryHMMerror inclunknown plusnonunknownonly.',
            'No fitting/thresholdselection/stateinference/physics; edge-vs-noedge is descriptive pairedcomparison.'],inputs=bindings))
    cases,arrays,all_spells={},{},[]
    for case in CASES:
        arrays[case],cases[case],ss=derived(case,load(evaluation_root/(case+'_frames.npz')),load(packet_root/case/'evaluation_packet.npz'))
        all_spells += ss
        np.savez_compressed(output/(case+'_diagnostic.npz'),**arrays[case])
    pooled,examples=combine(cases,arrays,all_spells)
    write(output/'report.json',dict(iteration='I-70-posthoc',new_physics_steps=0,parameters_changed=False,
        registered_scientific_gates_changed=False,cases=cases,pooled=pooled,mechanical_examples=examples,
        interpretation='Retrospective row/spell associations on unchanged recorded motion, not a causal intervention on controller or proof of physical slipping/contact identity.'))
    write(output/'spells.json',all_spells)
    for entry in bindings:
        if sha(entry['path'])!=entry['sha256']:raise ValueError('Frozen input changed during diagnostic')
    write(output/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(truth=pooled['truth_cardinality_counts'],paired=pooled['paired_comparisons'],examples=examples)))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation-root',type=Path,required=True)
    parser.add_argument('--packet-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    a=parser.parse_args();run(a.evaluation_root.resolve(),a.packet_root.resolve(),a.output.resolve())
