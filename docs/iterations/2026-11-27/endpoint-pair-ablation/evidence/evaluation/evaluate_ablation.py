"""I71 wrapper around exact frozen I70 scientific metrics and adoption gates.

Only the separately declared continuation decision is new.  It does not relax
any adoption gate and cannot authorize a live controller change.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import importlib.util
import json
import shutil
import numpy as np

REFERENCE = Path(__file__).with_name('reference_i70')/'evaluate_temporal.py'
spec = importlib.util.spec_from_file_location('i71_frozen_i70_metrics', REFERENCE)
metrics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)
EXPECTED_REFERENCE_SHA = '33993c7fc892d71a4530666090af957cb77614543b03fe3b4316bfdc8b351008'
CASES = ('m100_B', 'p030_B', 'p090_B')


def continuation_gates(pooled, cases):
    rows=[]
    def add(name, actual, limit, passed):
        rows.append(dict(name=name,actual=actual,limit=limit,passed=bool(passed)))
    H,E=pooled['methods']['hmm']['singleton'],pooled['methods']['emission']['singleton']
    if H['denominator'] != E['denominator']:
        raise ValueError('Continuation requires the same primary denominator')
    add('pooled_errors_strictly_less_than_matching_frame', H['errors'],E['errors'],H['errors'] < E['errors'])
    for case in CASES:
        h,e=cases[case]['methods']['hmm']['singleton'],cases[case]['methods']['emission']['singleton']
        if h['denominator'] != e['denominator']:
            raise ValueError('Per-case continuation denominators differ')
        add(case+'_errors_no_more_than_matching_frame',h['errors'],e['errors'],h['errors'] <= e['errors'])
    H,E=pooled['timing']['hmm'],pooled['timing']['emission']
    for name in ('missed','reverted_attempts','false_switches'):
        add('timing_'+name+'_no_more_than_matching_frame',H[name],E[name],H[name] <= E[name])
    p95=H['absolute_latency_s']['p95']
    add('timing_absolute_p95_at_most50ms',p95,.050,p95 is not None and p95 <= .050+1e-9)
    return rows


def continuation_context(pooled,cases):
    def context(row):
        return {m:dict(primary=row['methods'][m]['singleton'],confidence=row['methods'][m]['confidence'])
                for m in ('emission','state_hmm','hmm')}
    return dict(pooled=context(pooled),cases={c:context(cases[c]) for c in CASES},
        unknown_scope='Unknown is an error on the fixed primary domain and never provides confident coverage; endpoint pairs are deliberately unrepresentable.',
        no_adoption_authority=True)


def check_restricted_api(prediction):
    n=len(prediction['time_s'])
    for key in ('emission_probability21','state_hmm_probability21','posterior21'):
        values=prediction[key]
        if values.shape != (n,21) or np.any(values[:,8:20] != 0.):
            raise ValueError('Restricted experiment must pad all endpoint pair states with exactzero probability')
    for method in ('emission','state_hmm','hmm'):
        points=prediction[method+'_points_m']
        if points.shape != (n,4,3) or np.any(np.isfinite(points).all(axis=2).sum(axis=1) > 1):
            raise ValueError('Restricted output has more than one finite endpoint-family point')


def parent_comparison(parent,pooled,cases):
    for method in ('N','F','A'):
        if pooled['methods'][method]['singleton'] != parent['pooled']['methods'][method]['singleton']:
            raise ValueError('Frozen N/F/A singleton counts differ from I70')
    result={}
    for name,current,previous in [('pooled',pooled,parent['pooled'])]+[(c,cases[c],parent['cases'][c]) for c in CASES]:
        if current['counts'] != previous['counts']:
            raise ValueError('I70/I71 evaluation truth domains differ')
        result[name]={m:dict(I70_singleton=previous['methods'][m]['singleton'],
            I71_singleton=current['methods'][m]['singleton'],
            I70_loaded_set=previous['methods'][m]['multiset'],I71_loaded_set=current['methods'][m]['multiset'])
            for m in metrics.METHODS}
    return result


def run(protocol_path,packet_root,prediction_root,parent_science,output):
    if metrics.sha(REFERENCE) != EXPECTED_REFERENCE_SHA:
        raise ValueError('Frozen I70 metric source changed')
    protocol=json.loads(protocol_path.read_text())
    if protocol['iteration'] != 'I-71':
        raise ValueError('Wrong experiment protocol')
    paths=[Path(__file__),REFERENCE,protocol_path,parent_science]
    for case in CASES:paths += [packet_root/case/'evaluation_packet.npz',prediction_root/case/'predictions.npz']
    bindings=[dict(path=str(p),sha256=metrics.sha(p)) for p in paths]
    output.mkdir(parents=True,exist_ok=False)
    metrics.write(output/'registered_before_truth_read.json',dict(iteration='I-71',
        utc=datetime.now(timezone.utc).isoformat(),files=bindings,
        metric_scope='Exact frozen I70 metric/evaluation functions and16adoption gates; separate8continuation checks.'))
    shutil.copy2(Path(__file__),output/'evaluate_ablation.py')
    shutil.copy2(REFERENCE,output/'frozen_i70_metrics.py')
    shutil.copy2(protocol_path,output/'protocol.json')
    frames,cases={},{}
    for case in CASES:
        packet=metrics.load(packet_root/case/'evaluation_packet.npz')
        prediction=metrics.load(prediction_root/case/'predictions.npz')
        check_restricted_api(prediction)
        if len(packet['time_s']) != 8801 or int(metrics.primary_window(packet['time_s']).sum()) != 8000:
            raise ValueError('Complete registered44s episode required')
        frames[case],cases[case]=metrics.evaluate_case(packet,prediction,protocol)
        np.savez_compressed(output/(case+'_frames.npz'),**frames[case])
    pooled=metrics.aggregate(cases,frames)
    adoption=metrics.gates(pooled,cases)
    continuation=continuation_gates(pooled,cases)
    parent=json.loads(parent_science.read_text())
    continued=all(row['passed'] for row in continuation)
    science=dict(iteration='I-71',new_physics_steps=0,live_controller_changed=False,
        metric_reference_sha256=EXPECTED_REFERENCE_SHA,cases=cases,pooled=pooled,
        gates=adoption,all_gates_passed=all(row['passed'] for row in adoption),
        failed_gates=[row['name'] for row in adoption if not row['passed']],
        continuation_gates=continuation,continuation_all_gates_passed=continued,
        failed_continuation_gates=[row['name'] for row in continuation if not row['passed']],
        continuation_decision='DIAGNOSTIC_CONTINUATION_PASSED_NO_ADOPTION' if continued else 'STOP_THIS_HMM_VERSION',
        continuation_context=continuation_context(pooled,cases),
        parent_I70_comparison=parent_comparison(parent,pooled,cases),
        fixed_events=metrics.fixed_event_report(frames,protocol.get('fixed_events',[])),
        point_outliers_top10=metrics.outliers(frames),
        interpretation='Stopping ablation, not a general multicontact estimator. All471recorded mixed-pressure truth rows remain in loaded-set accuracy as errors when singleton-onlyoutput cannot match. No parameter fitting or controller adoption.',
        truth_scope='Original sampled final1ms positive-pressure labels every5ms; not full force-routing or interval contact truth.')
    for item in bindings:
        if metrics.sha(item['path']) != item['sha256']:
            raise ValueError('Frozen input/source changed during I71 evaluation')
    metrics.write(output/'science.json',science)
    metrics.write(output/'artifact_manifest.json',dict(files=[dict(path=p.name,sha256=metrics.sha(p),bytes=p.stat().st_size)
        for p in sorted(output.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(metrics.safe(dict(failed_adoption=science['failed_gates'],failed_continuation=science['failed_continuation_gates'],
        continuation_decision=science['continuation_decision'],pooled=pooled)),allow_nan=False))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('protocol','packet-root','prediction-root','parent-science','output'):
        parser.add_argument('--'+key,type=Path,required=True)
    a=parser.parse_args()
    run(a.protocol.resolve(),a.packet_root.resolve(),a.prediction_root.resolve(),a.parent_science.resolve(),a.output.resolve())
