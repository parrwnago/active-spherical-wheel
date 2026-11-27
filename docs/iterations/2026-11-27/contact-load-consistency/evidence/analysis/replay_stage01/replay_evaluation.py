"""Standalone NumPy replay of I-61 load veto, every frame and all fixed gates."""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,importlib.util,json,shutil,sys
import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m


def run(gate_source,evaluation_source,output):
    output.mkdir(parents=True,exist_ok=False)
    manifest=json.loads((evaluation_source/'artifact_manifest.json').read_text())
    for item in manifest['files']:
        if sha(evaluation_source/item['path'])!=item['sha256']:raise ValueError('Changedcompactevaluationartifact')
    evaluate=module(evaluation_source/'evaluate_load_gate.py','_i61_portable_eval')
    gate=module(gate_source/'replay_gate.py','_i61_portable_gate')
    protocol=json.loads((evaluation_source/'protocol.json').read_text())
    expected=json.loads((evaluation_source/'science.json').read_text())
    array_bindings=json.loads((evaluation_source/'expected_array_bindings.json').read_text())['cases']
    evaluate.save(output/'registered_before_replay.json',dict(iteration='I-61',registered_utc=datetime.now(timezone.utc).isoformat(),
        replay_source_sha256=sha(Path(__file__)),evaluator_source_sha256=sha(evaluation_source/'evaluate_load_gate.py'),
        canonical_science_sha256=sha(evaluation_source/'science.json'),
        known_prior='Canonicalnegativeoutcomeisknown; thresholds/data/scienceareunchanged mechanical replay.',new_physics_steps=0))
    shutil.copy2(Path(__file__),output/'replay_evaluation.py')
    packs,truths,comparisons={},{},{}
    for case in evaluate.CASES:
        pack,parity=gate.load_and_gate_case(gate_source,case);packs[case]=pack
        truths[case]=evaluate.load(evaluation_source/(case+'_truth_projection.npz'))
        comparisons[case]=dict(gate=parity)
    frames,science=evaluate.evaluate_all(packs,truths,protocol,evaluation_source/'frozen_i60_event_context')
    for case,values in frames.items():
        expected_arrays=array_bindings[case]['arrays']
        same={name:(value.dtype.str==expected_arrays[name]['dtype']and list(value.shape)==expected_arrays[name]['shape']
            and hashlib.sha256(value.tobytes()).hexdigest()==expected_arrays[name]['sha256'])for name,value in values.items()}
        if set(values)!=set(expected_arrays)or not all(same.values()):raise ValueError('Replayedframearraydiffers: '+case+' '+str(same))
        comparisons[case]['evaluation_arrays_byte_exact']=same
        np.savez_compressed(output/(case+'_evaluation.npz'),**values)
    identical=science==expected;evaluate.save(output/'science.json',science)
    evaluate.save(output/'replay_comparison.json',dict(iteration='I-61',science_exact=identical,
        all_evaluation_arrays_byte_exact=True,cases=comparisons,new_physics_steps=0,
        scope='Fixedloadgate/policies/oraclegeometry arithmeticandallcanonicalmetrics. DoesnotregenerategeometricJacobiansfromhistoricalmodelwithoutlocaloriginals.'))
    if not identical:raise ValueError('Replayedscientificreportdiffers')
    evaluate.save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*'))if p.is_file()and '__pycache__'not in p.parts and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(science_exact=True,all_evaluation_arrays_byte_exact=True,cases=len(comparisons),new_physics_steps=0)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--gate-data-source',type=Path,required=True)
    p.add_argument('--evaluation-data-source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    run(a.gate_data_source.resolve(),a.evaluation_data_source.resolve(),a.output.resolve())
