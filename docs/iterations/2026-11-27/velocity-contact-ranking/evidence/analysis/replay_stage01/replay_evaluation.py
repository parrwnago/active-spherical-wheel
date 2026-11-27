"""Standalone NumPy replay of fixed I-60 rank/evaluation evidence, no physics.

The 16-slot rank package preserves the original fixed-mode family minima.
Historical truth loads are evaluator-only inputs from the sealed score report.
This does not regenerate geometry candidates without the historical model.
"""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import importlib.util
import json
import shutil
import sys

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def module(path, name):
    spec=importlib.util.spec_from_file_location(name,path)
    value=importlib.util.module_from_spec(spec)
    sys.modules[spec.name]=value
    spec.loader.exec_module(value)
    return value


def exact(a,b):
    return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()


def run(rank_source,evaluation_source,output):
    output.mkdir(parents=True,exist_ok=False)
    # Verify the complete canonical evaluation stage before using its labels.
    manifest=json.loads((evaluation_source/'artifact_manifest.json').read_text())
    for item in manifest['files']:
        if sha(evaluation_source/item['path'])!=item['sha256']:
            raise ValueError('Changed evaluation artifact: '+item['path'])
    evaluation=module(evaluation_source/'evaluate_scores.py','_i60_portable_evaluation')
    rank=module(rank_source/'replay_rank.py','_i60_portable_rank')
    protocol=json.loads((evaluation_source/'protocol.json').read_text())
    expected=json.loads((evaluation_source/'science.json').read_text())
    registration=dict(iteration='I-60',registered_before_replay_utc=datetime.now(timezone.utc).isoformat(),
        replay_source_sha256=sha(Path(__file__)),evaluation_source_sha256=sha(evaluation_source/'evaluate_scores.py'),
        canonical_science_sha256=sha(evaluation_source/'science.json'),
        known_prior='Canonical I-60 results are known; this is an exact mechanical reproduction, not a new blinded hypothesis test.',
        new_physics_steps=0)
    evaluation.save(output/'registered_before_replay.json',registration)
    shutil.copyfile(Path(__file__),output/'replay_evaluation.py')
    cases,comparisons={},{}
    for case in evaluation.CASES:
        context,scores,rank_parity=rank.load_and_rank_case(rank_source,case)
        canonical=evaluation.load_npz(evaluation_source/(case+'_evaluation.npz'))
        if not exact(context['time_s'],canonical['time_s']):
            raise ValueError('Rank-context and truth-evaluation clocks differ')
        frames,report=evaluation.evaluate_case(context,scores,
            canonical['truth_patch_loads_N'],canonical['truth_patch_contact_counts'],protocol)
        comparison={name:exact(frames[name],canonical[name]) for name in frames}
        if set(frames)!=set(canonical) or not all(comparison.values()):
            raise ValueError('Full-rate evaluation arrays differ: '+case+' '+str(comparison))
        comparisons[case]=dict(rank=rank_parity,evaluation_arrays_byte_exact=comparison)
        cases[case]=report
    aggregate=evaluation.aggregate_primary(cases)
    gates=evaluation.gate_report(aggregate,cases,protocol)
    science=dict(iteration='I-60',new_physics_steps=0,live_controller_changed=False,
        cases=cases,aggregate_primary=aggregate,gates=gates,
        all_gates_passed=all(g['passed'] for g in gates),failed_gates=[g['name'] for g in gates if not g['passed']],
        interpretation='Offline dominant-support ranking only; normal domain is observer-conditioned. No posterior probability, independent wrench replacement, full multicontact estimate or physical tracking benefit is established.',
        truth_clock=protocol['evaluation']['clock'],
        interval_convention='Half-open windows; counts and 5ms sampled disagreement-equivalent seconds, not continuous wrong-contact durations.')
    same=science==expected
    evaluation.save(output/'science.json',science)
    evaluation.save(output/'replay_comparison.json',dict(iteration='I-60',science_exact=same,
        all_evaluation_arrays_byte_exact=True,cases=comparisons,new_physics_steps=0,
        scope='Fixed-mode minima-preserving rank plus every canonical evaluation metric/gate; no full geometry extraction or new robot trajectory.'))
    if not same:
        raise ValueError('Canonical scientific report differs')
    evaluation.save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(science_exact=True,all_evaluation_arrays_byte_exact=True,cases=len(cases),new_physics_steps=0)))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rank-data-source',type=Path,required=True)
    parser.add_argument('--evaluation-data-source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    run(args.rank_data_source.resolve(),args.evaluation_data_source.resolve(),args.output.resolve())
