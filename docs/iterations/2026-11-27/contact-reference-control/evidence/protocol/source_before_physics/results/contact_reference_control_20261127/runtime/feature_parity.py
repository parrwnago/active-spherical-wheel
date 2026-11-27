"""I73 limited historical, truth-free feature/ranker implementation parity.

This checks the proposed live kinematics path at 13 fixed timestamps per native
episode. It does not evaluate accuracy, tune a score, or step any simulator.
Candidate-ID differences are reported explicitly even for nearly equal points.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import importlib.util
import json
import shutil
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
CASES=('m100_B','p030_B','p090_B')
INDICES=np.array([0,1,200,1000,1575,2400,4000,6000,6422,7011,7041,8200,8800])
FIELDS=('time_s','exact_qpos','exact_qvel','exact_previous_applied',
        'exact_observer_wrench','intrinsic_normal_hint','exact_estimated_contact_point_m')
PROJECTED=('excitation_m_s','family_index','frame_valid','moment_residual_norm',
           'normal_angle_rad','valid','velocity_residual_m_s','velocity_residual_norm_m_s','kind')
RANK=('candidate_index','chosen_family','accepted_family','confident','score_tied',
      'family_scores','score_margin')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n')


def load_module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    value=importlib.util.module_from_spec(spec);sys.modules[name]=value;spec.loader.exec_module(value)
    return value


def paths_for(case):
    old60=ROOT/'results/velocity_contact_rank_20261127/campaign/features_stage01'/case
    old72=ROOT/'results/geometry_first_contact_20261127'
    provenance=ROOT/'results/balance_priority_20261126/campaign/runs'/case/'provenance.json'
    xml=Path(json.loads(provenance.read_text())['models']['mirror_xml'])
    return dict(context=old60/'measurement_context.npz',projected=old60/'projected_features.npz',
                scores=old60/'scores.npz',packet=old72/'inputs/stage01'/case/'rank_packet.npz',
                predictions=old72/'inference/stage01'/case/'predictions.npz',
                provenance=provenance,xml=xml)


def register(stage,protocol):
    stage.mkdir(parents=True,exist_ok=False)
    sources=[Path(__file__).resolve(),BASE/'adapter/frozen_i60_rank.py',
             BASE/'adapter/geometry_first_contact.py',ROOT/'src/spherical_wheel/contact_sensing.py']
    inputs=[protocol]
    for case in CASES:
        files=paths_for(case);inputs+=list(files.values())
        for element in ET.parse(files['xml']).getroot().iter():
            if element.get('file'):
                p=Path(element.get('file'))
                inputs.append(p if p.is_absolute() else files['xml'].parent/p)
    for p in sources:
        target=stage/'source'/p.relative_to(ROOT);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
    bindings={str(p.resolve().relative_to(ROOT)):dict(sha256=sha(p),bytes=p.stat().st_size)
              for p in set(sources+inputs)}
    save(stage/'registered_before_arithmetic.json',dict(
        registered_utc=datetime.now(timezone.utc).isoformat(),paths=bindings,
        indices=INDICES.tolist(),cases=CASES,allowed_context_fields=FIELDS,
        no_truth_fields=True,no_new_physics=True,no_accuracy_metrics=True,
        numeric_tolerance_abs=1e-12,numeric_tolerance_relative=1e-12,
        discrete_policy='Candidate IDs/accepted labels/masks exact; differences are never silently accepted.',
        purpose='Limited implementation parity, not new contact inference evaluation.'))


def compare(actual,expected):
    a,b=np.asarray(actual),np.asarray(expected)
    shape=a.shape==b.shape
    if not shape:return dict(passed=False,shape_match=False,actual_shape=list(a.shape),expected_shape=list(b.shape))
    if a.dtype.kind not in 'fc' and b.dtype.kind not in 'fc':
        return dict(passed=bool(np.array_equal(a,b)),exact=bool(np.array_equal(a,b)),shape_match=True,
                    mismatched_elements=int(np.sum(a!=b)))
    finite=np.isfinite(a)&np.isfinite(b)
    patterns=bool(np.array_equal(np.isnan(a),np.isnan(b)) and np.array_equal(np.isposinf(a),np.isposinf(b)) and np.array_equal(np.isneginf(a),np.isneginf(b)))
    diff=np.abs(a[finite]-b[finite]);allowed=1e-12+1e-12*np.abs(b[finite])
    return dict(passed=patterns and bool((diff<=allowed).all()),shape_match=True,
                nonfinite_patterns_exact=patterns,maximum_absolute_difference=float(diff.max()) if len(diff) else 0.,
                exact=bool(np.array_equal(a,b,equal_nan=True)))


def execute(stage):
    reg=json.loads((stage/'registered_before_arithmetic.json').read_text())
    for p,binding in reg['paths'].items():
        if sha(ROOT/p)!=binding['sha256']:raise ValueError('Registered parity input changed: '+p)
    if (stage/'report.json').exists():raise FileExistsError('Preserve earlier parity outcome')
    import mujoco
    from spherical_wheel.contact_sensing import intrinsic_normal
    frozen=load_module('_i73_parity_frozen',BASE/'adapter/frozen_i60_rank.py')
    geometry=load_module('_i73_parity_geometry',BASE/'adapter/geometry_first_contact.py')
    reports={}
    for case in CASES:
        paths=paths_for(case)
        with np.load(paths['context'],allow_pickle=False) as packet:
            data={k:packet[k][INDICES].copy() for k in FIELDS}
        with np.load(paths['projected'],allow_pickle=False) as packet:
            expected={k:packet[k].copy() if k in ('family_index','kind') else packet[k][INDICES].copy() for k in PROJECTED}
        with np.load(paths['scores'],allow_pickle=False) as packet:
            oldA={k:packet['fused__'+k][INDICES].copy() for k in RANK}
        with np.load(paths['predictions'],allow_pickle=False) as packet:
            oldC={k:packet['fused__'+k][INDICES].copy() for k in RANK}
            oldIDs=packet['selected_candidate_id'][INDICES].copy()
        with np.load(paths['packet'],allow_pickle=False) as packet:
            ids=packet['candidate_ids'].copy()
            anchor=packet['native_anchor_point_m'][INDICES].copy()
            shape=packet['point_shape'].copy()
            oldpoints=np.full(tuple(shape),np.nan)
            oldpoints.reshape(-1,3)[packet['point_flat_index']]=packet['point_values_m']
            oldpoints=oldpoints[INDICES]
        model=mujoco.MjModel.from_xml_path(str(paths['xml']))
        model.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        extractor=frozen.ContactHypothesisFeatures(model)
        values=[];hints=[];points=[]
        for i in range(len(INDICES)):
            wrench=data['exact_observer_wrench'][i]
            hint=intrinsic_normal(wrench[:3],wrench[3:],extractor.radius)
            hint=np.full(3,np.nan) if hint is None else hint
            result=extractor.evaluate(data['exact_qpos'][i],data['exact_qvel'][i],
                                      data['exact_previous_applied'][i],hint,wrench)
            projected=frozen.project_features(result);projected['kind']=extractor.kind.copy()
            values.append(projected);hints.append(hint);points.append(result['point_m'])
        current={k:values[0][k] if k in ('family_index','kind') else np.stack([v[k] for v in values]) for k in PROJECTED}
        points=np.stack(points)
        A=frozen.rank_hypotheses(current)['fused']
        C=geometry.geometry_first_rank(current,points,anchor,ids,rank_hypotheses=frozen.rank_hypotheses)
        checks={'intrinsic_normal':compare(np.stack(hints),data['intrinsic_normal_hint']),
                'native_anchor':compare(anchor,data['exact_estimated_contact_point_m'])}
        checks.update({'feature__'+k:compare(current[k],expected[k]) for k in PROJECTED})
        checks.update({'A__'+k:compare(A[k],oldA[k]) for k in RANK})
        checks.update({'C__'+k:compare(C['ranking']['fused'][k],oldC[k]) for k in RANK})
        checks['C__selected_candidate_id']=compare(C['selected_candidate_id'],oldIDs)
        valid=expected['valid']
        checks['candidate_point_on_original_valid']=compare(points[valid],oldpoints[valid])
        mismatches=[]
        for i,family in np.argwhere(C['selected_candidate_id']!=oldIDs):
            oldid,newid=int(oldIDs[i,family]),int(C['selected_candidate_id'][i,family])
            candidates=np.flatnonzero(expected['family_index']==family)
            point_distance=np.sum((points[i,candidates]-anchor[i])**2,axis=1)
            finite=np.sort(point_distance[np.isfinite(point_distance)])
            mismatches.append(dict(frame_index=int(INDICES[i]),time_s=float(data['time_s'][i]),family=int(family),
                                   cached_id=oldid,recomputed_id=newid,
                                   closest_two_recomputed_squared_distance_gap_m2=float(finite[1]-finite[0]) if len(finite)>1 else None,
                                   cached_point_m=oldpoints[i,oldid].tolist() if oldid>=0 else None,
                                   recomputed_point_m=points[i,newid].tolist() if newid>=0 else None))
        reports[case]=dict(passed=all(v['passed'] for v in checks.values()),checks=checks,
                           candidate_id_mismatches=mismatches,frames=len(INDICES))
    unchanged=all(sha(ROOT/p)==b['sha256'] for p,b in reg['paths'].items())
    report=dict(passed=unchanged and all(x['passed'] for x in reports.values()),cases=reports,
                source_and_inputs_unchanged=unchanged,no_truth_fields_read=True,
                no_new_physics=True,no_new_accuracy_statistics=True,
                total_fixed_frames=3*len(INDICES),
                limitation='Only fixed implementation-parity frames, not full-episode parity or contact accuracy.')
    save(stage/'report.json',report)
    manifest={n:dict(sha256=sha(stage/n),bytes=(stage/n).stat().st_size) for n in ('registered_before_arithmetic.json','report.json')}
    save(stage/'artifact_manifest.json',dict(files=manifest,inputs_unchanged=unchanged))
    print(json.dumps(dict(passed=report['passed'],case_passed={k:v['passed'] for k,v in reports.items()},
                         id_mismatches={k:len(v['candidate_id_mismatches']) for k,v in reports.items()}),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--stage',type=Path,required=True)
    parser.add_argument('--protocol',type=Path,required=True);parser.add_argument('--register',action='store_true');parser.add_argument('--run',action='store_true')
    args=parser.parse_args()
    if args.register:register(args.stage.resolve(),args.protocol.resolve())
    if args.run:execute(args.stage.resolve())
