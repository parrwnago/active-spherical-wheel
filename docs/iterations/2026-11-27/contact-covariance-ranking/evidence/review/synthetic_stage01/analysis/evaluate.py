"""I-65 fixed whole-episode, shared-covariance contact-ranking experiment.

Only archived NumPy maps/loads are used. Training family labels are supervised
calibration labels; held-out truth is evaluated after scoring, never fitted.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import importlib.util
import json
import shutil
import sys
import numpy as np

CASES = ('m100_B', 'p030_B', 'p090_B')
MODELS = ('M1', 'M2')
SCORERS = ('maxabs', 'euclidean', 'shared_covariance')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load(path):
    with np.load(path, allow_pickle=False) as pack:
        return {k: pack[k].copy() for k in pack.files}


def calibrated_rows(base, primitive, feature, model, method):
    """One fixed teacher branch/time, F first; truth point never consulted."""
    normalized, valid = {}, {}
    for branch in ('F', 'Nalt'):
        normalized[branch], finite = method.normalize_residuals(
            feature[model + '_' + branch + '_residual_Nm'],
            base['radius_m'], base['force_norm_N'])
        valid[branch] = (feature[model + '_' + branch + '_valid'] & finite
                         & primitive['material_' + branch + '_valid'])
    eligible = base['primary_eligible'] & base['i60_accepted']
    F = eligible & valid['F'] & (primitive['proposed_family'] == base['truth_family'])
    N = (eligible & ~F & valid['Nalt']
         & (primitive['native_family'] == base['truth_family']))
    chosen = np.full(len(eligible), -1, np.int8)
    chosen[F] = 0
    chosen[N] = 1
    rows = np.where(F[:, None], normalized['F'], normalized['Nalt'])
    return rows[chosen >= 0], chosen


def fit_fold(training, datasets, model, method, protocol):
    """The caller passes ONLY the two training datasets to this function."""
    config = protocol['calibration']
    residuals, weights, selections, counts = [], [], {}, {}
    if set(training) != set(datasets) or len(training) != 2:
        raise ValueError('Exactly the two registered training episodes required')
    for case in training:
        b, p, f = datasets[case]
        values, selected = calibrated_rows(b, p, f, model, method)
        selections[case] = selected
        counts[case] = dict(rows=len(values), F=int(np.sum(selected == 0)),
                            Nalt=int(np.sum(selected == 1)))
        if len(values) < config['minimum_selected_rows_per_episode']:
            return None, selections, counts
        residuals.append(values)
        weights.append(np.full(len(values), 1. / (len(training) * len(values))))
    e = np.concatenate(residuals)
    w = np.concatenate(weights)
    try:
        fitted = method.fit_shared_covariance(e, w, rho=config['shrinkage_rho'],
                                             floor_std=config['floor_std_dimensionless'])
    except (ValueError, np.linalg.LinAlgError, FloatingPointError) as error:
        counts['_calibration_failure'] = str(error)
        return None, selections, counts
    centered = e - fitted['mean']
    whitened = np.linalg.solve(fitted['cholesky'], centered.T).T
    scores = np.sum(whitened * whitened, axis=1)
    order = np.argsort(scores, kind='stable')
    quantile_index = min(int(np.searchsorted(np.cumsum(fitted['normalized_weights'][order]), .95, side='left')), len(e)-1)
    fitted['training_weighted_p95_score'] = float(scores[order[quantile_index]])
    return fitted, selections, counts


def compare_scores(F, N, eF, eN, valid, config):
    """Return -1F,+1N,0tie,2pattern-ambiguous,3unknown; no truth input."""
    result = np.full(len(F), 3, np.int8)
    valid = valid & np.isfinite(F) & np.isfinite(N)
    ambiguous = valid & (np.linalg.norm(eF-eN, axis=1) <= config['pattern_epsilon_dimensionless'])
    result[ambiguous] = 2
    remaining = valid & ~ambiguous
    delta = F-N
    epsilon = config['relative_score_tie_epsilon'] * np.maximum(1., np.maximum(abs(F), abs(N)))
    result[remaining & (delta < -epsilon)] = -1
    result[remaining & (delta > epsilon)] = 1
    result[remaining & (abs(delta) <= epsilon)] = 0
    return result


def score_heldout(base, primitive, feature, model, fitted, method, protocol):
    """No labels are accessed; model/normalization identical for candidates."""
    e, valid = {}, {}
    for branch in ('F', 'Nalt'):
        e[branch], finite = method.normalize_residuals(feature[model+'_'+branch+'_residual_Nm'],
                                                       base['radius_m'], base['force_norm_N'])
        valid[branch] = feature[model+'_'+branch+'_valid'] & finite
    pair_valid = valid['F'] & valid['Nalt']
    output = {'e_F': e['F'], 'e_Nalt': e['Nalt'], 'pair_valid': pair_valid}
    for scorer in SCORERS:
        F = np.full(len(pair_valid), np.nan)
        N = F.copy()
        if scorer == 'maxabs':
            F = np.max(abs(e['F']), axis=1)
            N = np.max(abs(e['Nalt']), axis=1)
        elif scorer == 'euclidean':
            F = np.sum(e['F']**2, axis=1)
            N = np.sum(e['Nalt']**2, axis=1)
        elif fitted is not None:
            for branch, values in (('F', F), ('Nalt', N)):
                finite = np.isfinite(e[branch]).all(axis=1)
                whitened = np.linalg.solve(fitted['cholesky'], (e[branch][finite]-fitted['mean']).T).T
                values[finite] = np.sum(whitened**2, axis=1)
        code = compare_scores(F, N, e['F'], e['Nalt'], pair_valid, protocol['comparison'])
        keep = shadow_keep(base['i60_accepted'], primitive['H_F_valid'], code)
        output.update({scorer+'_F_score': F, scorer+'_Nalt_score': N,
                       scorer+'_code': code, scorer+'_keep': keep})
        # Pair validity and known score are distinct for an invalid calibration.
        output[scorer+'_known'] = pair_valid & np.isfinite(F) & np.isfinite(N)
    return output


def shadow_keep(accepted, valid_F_map, code):
    return accepted & valid_F_map & (code != 1)


def groups(base):
    Nwrong = base['N_family'] != base['truth_family']
    Awrong = base['A_family'] != base['truth_family']
    return dict(useful=Nwrong & ~Awrong, harmful=~Nwrong & Awrong,
                accepted_wrong=base['i60_accepted'] & Awrong)


def discrimination(base, arrays, scorer):
    domain = base['primary_eligible']
    group = groups(base)
    code = arrays[scorer+'_code']
    result = {}
    for name, mask in dict(primary=domain, accepted=domain & base['i60_accepted'],
                           **{k: domain & m for k,m in group.items()}).items():
        known = arrays[scorer+'_known']
        row = dict(denominator=int(mask.sum()), known=int(np.sum(mask & known)),
                   unknown=int(np.sum(mask & (code == 3))),
                   F_preferred=int(np.sum(mask & (code == -1))),
                   Nalt_preferred=int(np.sum(mask & (code == 1))),
                   tied=int(np.sum(mask & (code == 0))),
                   ambiguous=int(np.sum(mask & (code == 2))))
        row['coverage'] = None if not row['denominator'] else row['known']/row['denominator']
        if name in ('useful', 'harmful'):
            row['correct'] = row['F_preferred' if name == 'useful' else 'Nalt_preferred']
            row['correct_fraction'] = None if not row['denominator'] else row['correct']/row['denominator']
        result[name] = row
    return result


def gate_discrimination(bases, outputs, model, protocol, metrics):
    full_base = {k: np.concatenate([b[k] for b in bases.values()]) for k in next(iter(bases.values()))}
    full_arrays = {k: np.concatenate([v[model][k] for v in outputs.values()])
                   for k in next(iter(outputs.values()))[model]}
    reports = {s: discrimination(full_base, full_arrays, s) for s in SCORERS}
    old, new = reports['maxabs'], reports['shared_covariance']
    g = protocol['discrimination_gates']
    domain = full_base['primary_eligible']
    point = domain & full_base['point_mask']
    native_max = float(np.max(full_base['native_distance_m'][point]))
    Adistance = np.where(full_base['i60_accepted'], full_base['proposed_distance_m'], full_base['native_distance_m'])
    critical = domain & (groups(full_base)['harmful'] | groups(full_base)['accepted_wrong']
                         | (full_base['i60_accepted'] & point & (Adistance > native_max+1e-15)))
    flags = dict(accepted_pair_coverage=new['accepted']['coverage'] is not None and new['accepted']['coverage'] >= g['coverage_min'],
                 critical_scorable=bool(np.all(full_arrays['shared_covariance_known'][critical])),
                 half_harmful_correct=new['harmful']['correct_fraction'] is not None and new['harmful']['correct_fraction'] >= g['harmful_correct_fraction_min'],
                 useful_90pct_correct=new['useful']['correct_fraction'] is not None and new['useful']['correct_fraction'] >= g['useful_correct_fraction_min'],
                 harmful_nonregression=new['harmful']['correct'] >= old['harmful']['correct'],
                 useful_nonregression=new['useful']['correct'] >= old['useful']['correct'],
                 strict_improvement=new['harmful']['correct']+new['useful']['correct'] > old['harmful']['correct']+old['useful']['correct'])
    return dict(ranking=reports, gates=flags, promising=all(flags.values()),
                critical_denominator=int(critical.sum()), critical_unknown=int(np.sum(critical & ~full_arrays['shared_covariance_known'])))


def shadow_report(bases, outputs, model, scorer, metrics, protocol):
    domains = {c: metrics.domain([b]) for c,b in bases.items()}
    full = metrics.domain(list(bases.values()))
    phase = {c: metrics.score_keeps(d, outputs[c][model][scorer+'_keep'][bases[c]['primary_eligible']])
             for c,d in domains.items()}
    keep = np.concatenate([outputs[c][model][scorer+'_keep'][bases[c]['primary_eligible']] for c in bases])
    pooled = metrics.score_keeps(full, keep)
    base = metrics.constants(full)
    phase_bases = {c: metrics.constants(d) for c,d in domains.items()}
    passed, flags = metrics.training_feasibility(pooled, base, phase, phase_bases, protocol)
    return dict(metrics=metrics.scalar_metrics(pooled), baseline=base,
                gates={k: bool(v[0]) for k,v in flags.items()}, all_gates_pass=bool(passed[0]),
                per_case={c:dict(metrics=metrics.scalar_metrics(phase[c]), baseline=phase_bases[c],
                                harmful_accuracy_scorable=phase_bases[c]['A_harmful_count']>0) for c in domains})


def json_model(fitted):
    return None if fitted is None else {k: v.tolist() if isinstance(v,np.ndarray) else v for k,v in fitted.items() if k!='normalized_weights'}


def run(protocol_path, previous, source_root, output):
    protocol = read(protocol_path)
    if protocol['iteration'] != 'I-65' or tuple(protocol['cases']) != CASES:
        raise ValueError('Unregistered protocol')
    seal = read(protocol_path.with_name('input_source_seal.json'))
    for row in seal['files']:
        p = (source_root / row['path']) if row['root'] == 'experiment' else (previous / row['path'])
        if sha(p) != row['sha256']:
            raise ValueError('Sealed source/input changed: '+str(p))
    output.mkdir(parents=True, exist_ok=False)
    write(output/'registered_before_fit.json', dict(utc=datetime.now(timezone.utc).isoformat(),
        protocol_sha256=sha(protocol_path), input_source_seal_sha256=sha(protocol_path.with_name('input_source_seal.json')),
        numpy_version=np.__version__, new_physics_steps=0, live_controller_changes=False))
    method = module(source_root/'method/covariance_score.py', '_i65_covariance')
    history = module(previous/'features/portable_stage01/m100_B/source/history_features.py', '_i65_fixed_history')
    metrics = module(previous/'analysis/i62_metrics.py', '_i65_fixed_metrics')
    datasets = {}
    for case in CASES:
        base = load(previous/'portable_inputs/i62_base'/f'{case}.npz')
        primitive = load(previous/'features/portable_stage01'/case/'primitives.npz')
        if not np.array_equal(base['time_s'],primitive['time_s']):
            raise ValueError('Mismatched original timestamps')
        feature = history.derive_history(primitive)
        for values in (base,primitive,feature):
            for value in values.values(): value.flags.writeable=False
        datasets[case] = base,primitive,feature
    outputs = {c:{} for c in CASES}
    models = {}
    for heldout in CASES:
        training = [c for c in CASES if c != heldout]
        models[heldout] = {}
        selections = {}
        for model in MODELS:
            fit, select, counts = fit_fold(training,{c:datasets[c] for c in training},model,method,protocol)
            models[heldout][model] = dict(training=training, training_counts=counts,
                                         calibration_failed=fit is None, fitted=json_model(fit))
            selections.update({model+'__'+c+'__teacher_branch':v for c,v in select.items()})
            b,p,f = datasets[heldout]
            result = score_heldout(b,p,f,model,fit,method,protocol)
            if fit is not None:
                result['both_above_training_p95'] = result['shared_covariance_known'] & (result['shared_covariance_F_score'] > fit['training_weighted_p95_score']) & (result['shared_covariance_Nalt_score'] > fit['training_weighted_p95_score'])
            else:
                result['both_above_training_p95'] = np.zeros(len(b['time_s']),bool)
            outputs[heldout][model] = result
        np.savez_compressed(output/f'{heldout}_training_selections.npz',**selections)
        frame = dict(time_s=datasets[heldout][0]['time_s'])
        for model,res in outputs[heldout].items():
            frame.update({model+'__'+k:v for k,v in res.items()})
        np.savez_compressed(output/f'{heldout}_oof.npz',**frame)
    bases = {c:v[0] for c,v in datasets.items()}
    report = dict(iteration='I-65',protocol_sha256=sha(protocol_path),numpy_version=np.__version__,
                  qualification='Retrospective three previously inspected whole episodes; no unseen validation, new driving or live adoption.',
                  models=models, discrimination={}, shadows={}, per_case={}, events=[])
    for model in MODELS:
        report['discrimination'][model] = gate_discrimination(bases,outputs,model,protocol,metrics)
        report['shadows'][model] = {s:shadow_report(bases,outputs,model,s,metrics,protocol) for s in SCORERS}
    for case,(b,p,f) in datasets.items():
        report['per_case'][case] = {model:dict(ranking={s:discrimination(b,outputs[case][model],s) for s in SCORERS},
            both_above_training_p95_accepted=int(np.sum(b['primary_eligible'] & b['i60_accepted'] & outputs[case][model]['both_above_training_p95']))) for model in MODELS}
        selected = b['primary_eligible']
        # Fixed, teacher-free old references reproduce I-64 before interpretation.
        for model in ('M0','M1','M2'):
            eF,_=method.normalize_residuals(f[model+'_F_residual_Nm'],b['radius_m'],b['force_norm_N'])
            eN,_=method.normalize_residuals(f[model+'_Nalt_residual_Nm'],b['radius_m'],b['force_norm_N'])
            scoreF=np.max(abs(eF),axis=1);scoreN=np.max(abs(eN),axis=1)
            pair=f[model+'_F_valid'] & f[model+'_Nalt_valid']
            g=groups(b)
            report['per_case'][case][model+'_I64_original_rank'] = dict(
                useful_correct=int(np.sum(selected & g['useful'] & pair & (scoreF < scoreN-1e-9))),
                harmful_correct=int(np.sum(selected & g['harmful'] & pair & (scoreN < scoreF-1e-9))))
        event_reasons = {}
        def add(indices, reason):
            for i in indices: event_reasons.setdefault(int(i),[]).append(reason)
        add(np.flatnonzero(selected & groups(b)['harmful']), 'every_harmful')
        useful=np.flatnonzero(selected & groups(b)['useful'])
        if len(useful): add(useful[:1],'first_useful')
        if case=='m100_B': add(np.flatnonzero(abs(b['time_s']-7.875)<=1e-9),'known_useful_7.875')
        distance=np.where(b['i60_accepted'],b['proposed_distance_m'],b['native_distance_m'])
        point=np.flatnonzero(selected & b['point_mask'])
        if len(point): add([point[np.argmax(distance[point])]],'worst_A_point')
        lost=np.flatnonzero(selected & groups(b)['useful'] & ~outputs[case]['M1']['shared_covariance_keep'])
        if len(lost): add(lost[:1],'first_lost_useful_primary')
        for i,reasons in sorted(event_reasons.items()):
            row=dict(case=case,index=i,time_s=float(b['time_s'][i]),reasons=reasons,
                     N_family=int(b['N_family'][i]),F_family=int(p['proposed_family'][i]),truth_family=int(b['truth_family'][i]),
                     native_point_mm=float(b['native_distance_m'][i]*1000.),proposal_point_mm=float(b['proposed_distance_m'][i]*1000.),scores={})
            for model in MODELS:
                arr=outputs[case][model]
                row['scores'][model]={s:dict(F=None if not np.isfinite(arr[s+'_F_score'][i]) else float(arr[s+'_F_score'][i]),
                    Nalt=None if not np.isfinite(arr[s+'_Nalt_score'][i]) else float(arr[s+'_Nalt_score'][i]),
                    code=int(arr[s+'_code'][i]),keep=bool(arr[s+'_keep'][i])) for s in SCORERS}
                row['scores'][model]['normalized_F_residual']=arr['e_F'][i].tolist()
                row['scores'][model]['normalized_Nalt_residual']=arr['e_Nalt'][i].tolist()
            report['events'].append(row)
    report['primary_qualified'] = report['discrimination']['M1']['promising'] and report['shadows']['M1']['shared_covariance']['all_gates_pass']
    report['decision'] = 'Offline promise requires future frozen fresh-episode validation; no live adoption.' if report['primary_qualified'] else 'Rejected for integration; retain live baseline and investigate candidate/load assumptions.'
    write(output/'science.json',report)
    # Record per-array hashes for compact deterministic replay without copying old inputs.
    bindings={}
    for file in sorted(output.glob('*.npz')):
        arrays=load(file)
        bindings[file.name]={k:dict(dtype=v.dtype.str,shape=list(v.shape),sha256=hashlib.sha256(v.tobytes()).hexdigest()) for k,v in arrays.items()}
    write(output/'array_bindings.json',bindings)
    write(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size) for p in sorted(output.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(iteration='I-65',primary_qualified=report['primary_qualified'],
                         discrimination=report['discrimination']['M1']['gates'],
                         shadow=report['shadows']['M1']['shared_covariance']['metrics'])),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--previous-package',type=Path,required=True)
    parser.add_argument('--source-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    run(args.protocol.resolve(),args.previous_package.resolve(),args.source_root.resolve(),args.output.resolve())
