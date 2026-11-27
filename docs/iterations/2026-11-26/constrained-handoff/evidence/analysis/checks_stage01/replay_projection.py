"""Replay every I56 metric from a portable full-rate saved-array projection.

Usage: python replay_projection.py --data-source PATH/TO/ANALYSIS_STAGE --output NEW_JSON
NumPy is the only nonstandard dependency. The stage carries copied pure metric
source, fixed reference events/program, protocol, and lossless metric input fields.
"""
from pathlib import Path
import argparse
import gzip
import hashlib
import json
import types
import numpy as np


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def module(path, name):
    result=types.ModuleType(name);result.__file__=str(path)
    exec(compile(path.read_text(),str(path),'exec'),result.__dict__)
    return result


def replay(stage, output):
    index=json.loads((stage/'portable_inputs.json').read_text())
    manifest=json.loads((stage/'artifact_manifest.json').read_text())
    for row in manifest['files']:
        if sha(stage/row['path'])!=row['sha256']:
            raise ValueError('Changed portable artifact: '+row['path'])
    reducer=module(stage/index['sources']['reducer'],'_portable_i56_reducer')
    legacy=module(stage/index['sources']['legacy_metrics'],'_portable_i49_metrics')
    helper=module(stage/index['sources']['event_helper'],'_portable_i41_events')
    program=reducer.load(stage/index['reference_program'])
    events=json.loads((stage/index['events']).read_text())
    report=dict(cases=dict(index['unavailable_cases']),comparisons={})
    protocol=None
    for case,paths in index['cases'].items():
        for key in ('projection','diagnostics','metadata'):
            if sha(stage/paths[key])!=paths[key+'_sha256']:
                raise ValueError('Changed input projection: '+case+'/'+key)
        raw=reducer.load(stage/paths['projection'])
        metadata=json.loads((stage/paths['metadata']).read_text())
        with gzip.open(stage/paths['diagnostics'],'rt',encoding='utf8') as stream:
            governor=json.load(stream)
        row,_,_=reducer.case_metrics(raw,{},metadata['summary'],metadata['provenance'],
            governor,metadata['protocol'],program,events,legacy,helper)
        row.update(case_id=case,phase_rad=metadata['provenance']['case']['phase_rad'],
            variant=metadata['provenance']['case']['variant'],
            case_source=str(Path(metadata['original_precision_path']).parents[3]))
        row['historical_parity']=metadata['historical_parity']
        report['cases'][case]=row
        if case.endswith('_B'):protocol=metadata['protocol']
    if protocol is None:raise ValueError('No baseline protocol in portable inputs')
    for phase in reducer.PHASES:
        b,h=phase+'_B',phase+'_H'
        if b in report['cases'] and h in report['cases']:
            report['comparisons'][h+'_vs_'+b]=reducer.gate_comparison(report['cases'][h],report['cases'][b],protocol['acceptance'])
    if 'm100_A' in report['cases']:
        report['comparisons']['m100_A_vs_m100_B']=reducer.gate_comparison(report['cases']['m100_A'],report['cases']['m100_B'],protocol['acceptance'])
    original=json.loads((stage/'report.json').read_text())
    case_exact={case:reducer.safe(row)==original['cases'][case] for case,row in report['cases'].items()}
    core_keys=('available','passes_all_conditions','checks','failed_checks','turn_changes','reason','scope')
    comparison_exact={case:{k:v for k,v in row.items() if k in core_keys}=={k:v for k,v in original['comparisons'][case].items() if k in core_keys}
        for case,row in report['comparisons'].items()}
    report['case_metrics_exact']=case_exact
    report['comparison_metrics_exact']=comparison_exact
    report['all_replayed_metrics_exact']=all(case_exact.values()) and all(comparison_exact.values())
    reducer.save(output,report)
    if not report['all_replayed_metrics_exact']:
        raise ValueError('Portable replay differs: '+str(case_exact)+' '+str(comparison_exact))
    print(json.dumps(dict(all_replayed_metrics_exact=True,cases=list(report['cases']))))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    replay(args.data_source.resolve(),args.output.resolve())
