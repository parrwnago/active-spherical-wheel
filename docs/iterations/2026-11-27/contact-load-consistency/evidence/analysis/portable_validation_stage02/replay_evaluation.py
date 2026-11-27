"""Standalone NumPy replay of I-61 load veto, every frame and all fixed gates."""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,importlib.util,json,math,shutil,sys
import numpy as np


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m


def differences(a,b,path=()):
    if type(a) is not type(b):return [(path,a,b)]
    if isinstance(a,dict):
        if set(a)!=set(b):return [(path,sorted(a),sorted(b))]
        return [item for key in a for item in differences(a[key],b[key],path+(str(key),))]
    if isinstance(a,list):
        if len(a)!=len(b):return [(path,len(a),len(b))]
        return [item for i,(x,y) in enumerate(zip(a,b)) for item in differences(x,y,path+(str(i),))]
    return [] if a==b else [(path,a,b)]


def verify_science(actual,expected):
    """Keep every value; permit only declared cross-NumPy scalar-event roundoff."""
    tolerance_mm=1e-12
    names={'earliest_native_disagreement','earliest_accepted_wrong',
           'maximum_margin_accepted_wrong','i60_worst_selected_point'}
    changes=differences(expected,actual);allowed=[]
    for path,old,new in changes:
        permitted=(len(path)==6 and path[0]=='cases' and path[1] in ('m100_B','p030_B','p090_B')
            and path[2]=='fixed_events' and path[3] in names
            and path[4]=='distances_to_equivalent_reference_mm' and path[5] in ('N','A','C')
            and isinstance(old,float) and isinstance(new,float) and math.isfinite(old) and math.isfinite(new)
            and abs(old-new)<=tolerance_mm)
        if not permitted:raise ValueError('Scientificreportdiffersoutsidepermittedscalar-eventroundoff: '+str(path))
        allowed.append(dict(path='/'+('/'.join(path)),canonical_value_mm=old,replayed_value_mm=new,absolute_difference_mm=abs(old-new)))
    metric_exact=(actual['aggregate_primary']==expected['aggregate_primary'] and actual['gates']==expected['gates']
        and all(actual['cases'][c]['windows']==expected['cases'][c]['windows'] for c in actual['cases']))
    if not metric_exact:raise ValueError('Primary/fixed-windowmetricsorgatesdiffer')
    return dict(science_exact=not changes,full_science_exact=not changes,
        primary_and_fixed_window_metrics_and_gates_exact=metric_exact,
        science_equivalent_with_declared_event_roundoff=True,
        event_scalar_roundoff_tolerance_mm=tolerance_mm,
        event_scalar_roundoff_max_mm=max((item['absolute_difference_mm'] for item in allowed),default=0.),
        event_scalar_roundoff=allowed,
        qualification='Allnumbersarepreserved. Onlylistedfixed-eventscalarpointnormsmayrounddifferentlyacrossNumPyversions; noframe,metric,gate,criterionorcanonicalvalueisnormalizedorchanged.')


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
        numpy_version=np.__version__,known_prior='Canonicalnegativeoutcomeisknown; thresholds/data/scienceareunchanged mechanical replay. NumPy1.21.5hasfiveknownscalar-eventnormroundoffs~e-15mm; validationpermitsONLYthosefixed-eventdistancepathswithin1e-12mm.',new_physics_steps=0))
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
    science_check=verify_science(science,expected);evaluate.save(output/'science.json',science)
    evaluate.save(output/'replay_comparison.json',dict(iteration='I-61',**science_check,
        all_evaluation_arrays_byte_exact=True,cases=comparisons,new_physics_steps=0,
        scope='Fixedloadgate/policies/oraclegeometry arithmeticandallcanonicalmetrics. DoesnotregenerategeometricJacobiansfromhistoricalmodelwithoutlocaloriginals.'))
    evaluate.save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*'))if p.is_file()and '__pycache__'not in p.parts and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(science_exact=science_check['science_exact'],
        primary_and_fixed_window_metrics_and_gates_exact=True,event_scalar_roundoff_max_mm=science_check['event_scalar_roundoff_max_mm'],
        all_evaluation_arrays_byte_exact=True,cases=len(comparisons),new_physics_steps=0)))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--gate-data-source',type=Path,required=True)
    p.add_argument('--evaluation-data-source',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    run(a.gate_data_source.resolve(),a.evaluation_data_source.resolve(),a.output.resolve())
