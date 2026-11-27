"""I-61 truth-blind selected-contact load features and veto policies.

No historical truth field is loaded. All three case artifacts must be sealed
before a separate evaluator opens contact truth. No controller or physics runs.
"""
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from unittest.mock import patch
import argparse
import hashlib
import importlib.util
import json
import shutil
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
CASES = ('m100_B', 'p030_B', 'p090_B')
CONTEXT_FIELDS = ('time_s', 'exact_qpos', 'exact_qvel', 'exact_previous_applied',
    'exact_observer_wrench', 'exact_observer_generalized', 'exact_observer_interval_mean',
    'exact_estimated_support_loads_N', 'exact_estimated_contact_point_m', 'intrinsic_normal_hint')
SCORE_FIELDS = ('fused__chosen_family', 'fused__accepted_family', 'fused__confident',
                'fused__candidate_index', 'fused__score_margin')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, dict): return {str(k):safe(v) for k,v in value.items()}
    if isinstance(value, (list,tuple)): return [safe(v) for v in value]
    if isinstance(value, np.ndarray): return safe(value.tolist())
    if isinstance(value, np.generic): return safe(value.item())
    if isinstance(value, Path): return str(value)
    if isinstance(value, float) and not np.isfinite(value): return None
    return value


def save(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, allow_nan=False)+'\n')


def module(path, name):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);sys.modules[name]=result
    spec.loader.exec_module(result)
    return result


def load_allowed(path, fields):
    with np.load(path, allow_pickle=False) as raw:
        return {name:raw[name].copy() for name in fields}


def native_labels(loads):
    finite=np.isfinite(loads).all(axis=1)&(loads>=0).all(axis=1)
    clean=np.where(np.isfinite(loads),loads,0.)
    maximum=clean.max(axis=1)
    unique=(clean==maximum[:,None]).sum(axis=1)==1
    return np.where(finite&(maximum>0)&unique,np.argmax(clean,axis=1),-1)


def policies(native_family, native_point, proposed_family, proposed_point, accepted, load_passes):
    accepted=np.asarray(accepted,bool);gated=accepted&np.asarray(load_passes,bool)
    return dict(N_family=native_family.copy(), A_family=np.where(accepted,proposed_family,native_family),
        C_family=np.where(gated,proposed_family,native_family), N_point_m=native_point.copy(),
        A_point_m=np.where(accepted[:,None],proposed_point,native_point),
        C_point_m=np.where(gated[:,None],proposed_point,native_point),
        A_uses_proposal=accepted.copy(), C_uses_proposal=gated)


def run(protocol_path, case, output):
    protocol=json.loads(protocol_path.read_text())
    if protocol['iteration']!='I-61' or case not in CASES: raise ValueError('Unregistered I-61 case')
    mapping_path=protocol_path.parent/'api_mapping_before_execution.json'
    mapping=json.loads(mapping_path.read_text())
    if sha(protocol_path)!=mapping['protocol_sha256']:
        raise ValueError('I-61 protocol differs from its pre-execution binding')
    # Root schema mapping is explicit; there is no runtime score/tolerance override.
    gate_options=dict(absolute_tolerance_Nm=protocol['primary_veto']['absolute_tolerance_Nm'],
                      relative_tolerance=protocol['primary_veto']['relative_force_lever_tolerance'])
    output.mkdir(parents=True,exist_ok=False);source=output/'source';source.mkdir()
    old=ROOT/'results/velocity_contact_rank_20261127/campaign/features_stage01'/case
    old_protocol=json.loads((old/'source/protocol.json').read_text())
    minimum_force=float(old_protocol['candidate_search']['minimum_compressive_force_N'])
    original=ROOT/'results/balance_priority_20261126/campaign/runs'/case
    provenance_path=original/'provenance.json'
    provenance=json.loads(provenance_path.read_text());xml=Path(provenance['models']['mirror_xml'])
    core_source=ROOT/'src/spherical_wheel/contact_load_consistency.py'
    geometry_source=old/'source/contact_hypotheses.py'
    bound=[Path(__file__).resolve(),protocol_path,mapping_path,core_source,geometry_source,xml,provenance_path,
           old/'measurement_context.npz',old/'scores.npz',old/'artifact_manifest.json']
    root_seal_path=protocol_path.parent/'input_source_seal.json'
    if root_seal_path.exists():
        root_seal=json.loads(root_seal_path.read_text())
        for item in root_seal.get('sources',[])+root_seal.get('inputs',[]):
            path=Path(item['path']);path=path if path.is_absolute() else ROOT/path
            if sha(path)!=item['sha256']:raise ValueError('Root-bound input/source changed: '+str(path))
        bound.append(root_seal_path)
    old_manifest=json.loads((old/'artifact_manifest.json').read_text())
    for item in old_manifest['files']:
        if sha(old/item['path'])!=item['sha256']:raise ValueError('Changed I-60 feature artifact')
    registration=dict(iteration='I-61',case=case,registered_before_load_arithmetic_utc=datetime.now(timezone.utc).isoformat(),
        inputs=[dict(path=str(path),sha256=sha(path)) for path in bound],
        allowed_context_fields=CONTEXT_FIELDS,allowed_score_fields=SCORE_FIELDS,
        truth_fields_read=[],gate_options=gate_options,minimum_force_N=minimum_force,
        native_relative_comparison='diagnostic only',new_physics_steps=0,live_controller_changed=False,
        oracle_map='Truth-blind per-family affine joint-force maps; future evaluator alone inserts logged truth geometry.')
    save(output/'registered_before_arithmetic.json',registration)
    for path,name in ((Path(__file__),'extract_features.py'),(core_source,'contact_load_consistency.py'),
                      (geometry_source,'contact_hypotheses.py'),(protocol_path,'protocol.json')):
        shutil.copy2(path,source/name)
    context=load_allowed(old/'measurement_context.npz',CONTEXT_FIELDS)
    scores=load_allowed(old/'scores.npz',SCORE_FIELDS)
    for value in context.values():value.flags.writeable=False
    for value in scores.values():value.flags.writeable=False
    t=context['time_s'];n=len(t)
    if n!=8801 or abs(t[0])>1e-12 or abs(t[-1]-44)>1e-8 or not np.all(abs(np.diff(t)-.005)<1e-9):
        raise ValueError('Expected all 8801 original 5-ms states')
    if not np.array_equal(scores['fused__confident'],scores['fused__accepted_family']>=0):
        raise ValueError('Frozen I-60 acceptance fields disagree')
    if not np.array_equal(scores['fused__accepted_family'][scores['fused__confident']],
                          scores['fused__chosen_family'][scores['fused__confident']]):
        raise ValueError('Frozen I-60 accepted family differs from its raw winner')
    if not np.array_equal(context['exact_observer_wrench'],context['exact_observer_generalized'][:,:6]):
        raise ValueError('Mixed observer-frame fields do not match')
    import mujoco
    core=module(source/'contact_load_consistency.py','_i61_load_core')
    geometry=module(source/'contact_hypotheses.py','_i61_frozen_geometry')
    model=mujoco.MjModel.from_xml_path(str(xml));model.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    features=core.ContactLoadFeatures(model,geometry_module=geometry,minimum_force_N=minimum_force)
    if features.radius_m != protocol['primary_veto']['radius_m']:
        raise ValueError('Model radius differs from the fixed load tolerance scale')
    native=native_labels(context['exact_estimated_support_loads_N'])
    proposed=scores['fused__chosen_family'];accepted=scores['fused__confident']
    winner_index=np.full(n,-1,np.int64);native_index=np.full(n,-1,np.int64)
    good=proposed>=0;winner_index[good]=scores['fused__candidate_index'][np.flatnonzero(good),proposed[good]]
    good=native>=0;native_index[good]=scores['fused__candidate_index'][np.flatnonzero(good),native[good]]
    points=np.full((n,2,3),np.nan);rows=[[],[]];maps=[]
    def forbidden(*args,**kwargs):raise RuntimeError('I-61 must not run physics or query contacts')
    started=time.monotonic()
    with ExitStack() as guards:
        for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_collision','mj_contactForce'):
            guards.enter_context(patch.object(mujoco,name,side_effect=forbidden))
        for i in range(n):
            q=context['exact_qpos'][i];hint=context['intrinsic_normal_hint'][i]
            wrench=context['exact_observer_wrench'][i];gamma=context['exact_observer_generalized'][i]
            for kind,(index,label) in enumerate(((winner_index[i],proposed[i]),(native_index[i],native[i]))):
                if index>=0 and int(features.geometry.family_index[index])!=label:
                    raise ValueError('Frozen candidate index belongs to another family')
                point=features.reconstruct_point(q,hint,index);points[i,kind]=point
                rows[kind].append(features.evaluate(q,point,label,wrench,gamma))
            maps.append(features.joint_maps(q,wrench))
            if (i+1)%2000==0:print(json.dumps(dict(case=case,rows=i+1,elapsed_s=round(time.monotonic()-started,3))),flush=True)
    pack=dict(time_s=t.copy(),native_family=native,proposed_family=proposed.copy(),i60_accepted=accepted.copy(),
        proposed_point_m=points[:,0],native_alternative_point_m=points[:,1],
        native_point_m=context['exact_estimated_contact_point_m'].copy(),
        proposed_candidate_index=winner_index,native_candidate_index=native_index,
        observer_generalized=context['exact_observer_generalized'].copy(),observer_wrench=context['exact_observer_wrench'].copy(),
        observer_interval_mean=context['exact_observer_interval_mean'].copy())
    for prefix,values in zip(('proposed','native_alternative'),rows):
        arrays={key:np.asarray([row[key] for row in values]) for key in values[0]}
        compatibility=core.load_compatibility(arrays,**gate_options)
        for key,value in arrays.items():pack[prefix+'__'+key]=value
        for key,value in compatibility.items():pack[prefix+'_gate__'+key]=value
    for key in maps[0]:pack['map__'+key]=np.asarray([row[key] for row in maps])
    pack['native_relative_max_ratio_difference']=(pack['proposed_gate__max_normalized_joint_residual']
        -pack['native_alternative_gate__max_normalized_joint_residual'])
    pack.update(policies(native,pack['native_point_m'],proposed,pack['proposed_point_m'],accepted,pack['proposed_gate__passes']))
    np.savez_compressed(output/'load_features_and_policies.npz',**pack)
    save(output/'metadata.json',dict(iteration='I-61',case=case,rows=n,
        feature_shapes={key:list(value.shape) for key,value in pack.items()},
        elapsed_s=time.monotonic()-started,model_xml_sha256=sha(xml),
        no_truth_read=True,no_physics_steps=True,same_frozen_i60_candidates=True,
        note='Native physical alternative is the frozen fused arm minimum within the native family; fallback geometry remains the original native equivalent point.'))
    for entry in registration['inputs']:
        if sha(entry['path'])!=entry['sha256']:raise ValueError('Bound input/source changed during extraction')
    save(output/'artifact_manifest.json',dict(iteration='I-61',sealed_before_any_truth_read_utc=datetime.now(timezone.utc).isoformat(),
        files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
               for p in sorted(output.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(case=case,rows=n,sealed_before_truth=True,new_physics_steps=0)),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',type=Path,required=True);parser.add_argument('--case',choices=CASES,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    run(args.protocol.resolve(),args.case,args.output.resolve())
