"""I67 blind ROI bank reconstruction and full-history selected-point parity.

Packets contain evaluation fields, but this extractor reads only the explicit
measurement/pose/old-geometry and inferred-selection allowlist below. ROI
selection is predeclared evaluation bookkeeping, not a hypothesis input.
"""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,importlib.util,json,shutil
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
CASES=('m100_B','p030_B','p090_B')
POSE_FIELDS=('time_s','centre_m','patch_axis_world','patch_lower_m','patch_upper_m','radius_m','valid')
PACKET_FIELDS=('source_index','time_s','intrinsic_normal_hint','observer_generalized',
    'centre_m','patch_axis_world','patch_lower_m','patch_upper_m','radius_m','valid',
    'old__frame_valid','old__valid','old__normal_angle_rad','old__family_index','old__kind',
    'saved_F_index','saved_Nalt_index','saved_F_family','saved_Nalt_family','saved_point_F_m','saved_point_Nalt_m')
I64_FIELDS=('time_s','proposed_candidate_index','native_candidate_index','proposed_family','native_family','point_F_m','point_Nalt_m')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def array_hash(value):return hashlib.sha256(np.asarray(value).tobytes()).hexdigest()
def load_fields(path,fields):
    with np.load(path,allow_pickle=False) as value:return {key:value[key].copy() for key in fields}
def choose_path(paths):
    found=[path for path in paths if path.is_file()]
    if len(found)!=1:raise ValueError('Pass a unique experiment root or direct portable stage root: '+str(paths))
    return found[0]
def verify_seal(path):
    value=read(path)
    if not value.get('files'):raise ValueError('Expected explicit input source seal')
    for row in value['files']:
        file=Path(row['path']);file=file if file.is_absolute() else ROOT/file
        if sha(file)!=row['sha256']:raise ValueError('Sealed source/input changed: '+str(file))


def frame(pose,index):
    return dict(centre_m=pose['centre_m'][index],patch_axis_world=pose['patch_axis_world'][index],
        valid=pose['valid'][index],radius_m=pose['radius_m'],patch_lower_m=pose['patch_lower_m'],patch_upper_m=pose['patch_upper_m'])


def point_parity(actual,expected,tolerance=1e-12):
    pattern=bool(np.array_equal(np.isfinite(actual),np.isfinite(expected)))
    finite=np.isfinite(actual)&np.isfinite(expected)
    delta=float(np.max(abs(actual[finite]-expected[finite]))) if np.any(finite) else 0.
    exact=actual.dtype==expected.dtype and actual.shape==expected.shape and actual.tobytes()==expected.tobytes()
    return dict(finite_patterns_exact=pattern,byte_exact=bool(exact),max_absolute_difference_m=delta,
        tolerance_m=tolerance,passes=pattern and delta<=tolerance)


def family_check(ids,families,layout):
    if not np.issubdtype(ids.dtype,np.integer) or np.any(ids>=102):raise ValueError('Invalid original candidate IDs')
    known=ids>=0
    if not np.array_equal(layout['family_index'][ids[known]],families[known]):raise ValueError('Original ID belongs to another inferred family')
    return dict(known_ids=int(known.sum()),unknown_ids=int((~known).sum()),ids_sha256=array_hash(ids),families_sha256=array_hash(families))


def full_selected_parity(helper,case,args,layout):
    p64=choose_path([args.i64/'features/portable_stage01'/case/'primitives.npz',args.i64/case/'primitives.npz'])
    p66=choose_path([args.i66/'features/stage01'/case/'pose_maps.npz',args.i66/case/'pose_maps.npz'])
    context_path=args.contexts/case/'measurement_context.npz'
    primitive=load_fields(p64,I64_FIELDS);pose=load_fields(p66,POSE_FIELDS)
    context=load_fields(context_path,('time_s','intrinsic_normal_hint'));t=pose['time_s'];n=len(t)
    if n!=8801 or not np.array_equal(t,primitive['time_s']) or not np.array_equal(t,context['time_s']):raise ValueError('Full original clocks differ')
    ids=np.stack([primitive['proposed_candidate_index'],primitive['native_candidate_index']],axis=1)
    points=np.full((n,2,3),np.nan)
    for i in range(n):points[i]=helper.reconstruct_selected(frame(pose,i),context['intrinsic_normal_hint'][i],ids[i])
    checks={}
    for column,(branch,family_key) in enumerate([('F','proposed_family'),('Nalt','native_family')]):
        checks[branch]=dict(**point_parity(points[:,column],primitive['point_'+branch+'_m']),
            **family_check(ids[:,column],primitive[family_key],layout))
        if not checks[branch]['passes']:raise ValueError('Full selected point differs: '+case+' '+branch)
    return dict(rows=n,checks=checks,inputs=[dict(path=str(path),sha256=sha(path)) for path in [p64,p66,context_path]])


def run(args):
    protocol=read(args.protocol)
    if protocol['iteration']!='I-67':raise ValueError('Expected frozen I67 protocol')
    seal=args.protocol.parent/'input_source_seal.json';verify_seal(seal)
    helper_path=Path(__file__).resolve().with_name('bank_points.py')
    spec=importlib.util.spec_from_file_location('_i67_original_bank_points',helper_path);helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    layout=helper.candidate_layout()
    args.output.mkdir(parents=True,exist_ok=False);source=args.output/'source';source.mkdir()
    write(args.output/'registered_before_bank_arithmetic.json',dict(iteration='I-67',registered_utc=datetime.now(timezone.utc).isoformat(),
        extractor_sha256=sha(Path(__file__)),helper_sha256=sha(helper_path),protocol_sha256=sha(args.protocol),
        input_source_seal_sha256=sha(seal),allowed_packet_fields=list(PACKET_FIELDS),allowed_full_i64_fields=list(I64_FIELDS),
        allowed_pose_fields=list(POSE_FIELDS),allowed_context_fields=['time_s','intrinsic_normal_hint'],
        truth_fields_read=[],new_scores=False,new_selection=False,new_kinematics=False,new_physics=False,
        interpretation='All102pre-admission slots reconstructed only for frozenROI; old masks remain authoritative. Full8801 work reconstructs only original F/Nalt IDs.'))
    for path in [Path(__file__),helper_path,args.protocol]:shutil.copy2(path,source/path.name)
    cases={}
    for case in CASES:
        packet_path=args.packets/(case+'.npz');packet=load_fields(packet_path,PACKET_FIELDS)
        count=len(packet['time_s'])
        if packet['old__valid'].shape!=(count,102) or packet['old__normal_angle_rad'].shape!=(count,102):raise ValueError('Old candidate shape differs')
        if not np.array_equal(packet['old__family_index'],layout['family_index']) or not np.array_equal(packet['old__kind'],layout['kind']):raise ValueError('Original layout differs')
        rows=[]
        for i in range(count):
            rows.append(helper.reconstruct_bank(frame(packet,i),packet['intrinsic_normal_hint'][i],
                observer_force_N=packet['observer_generalized'][i,:3],original_frame_valid=packet['old__frame_valid'][i]))
        cache=dict(source_index=packet['source_index'].copy(),time_s=packet['time_s'].copy(),
            **{key:np.asarray([row[key] for row in rows]) for key in ['points_m','reconstructed','normal_angle_rad','normal_force_N','original_geometry_valid']},
            **{key:layout[key].copy() for key in ['candidate_index','family_index','kind']})
        mask_exact=bool(np.array_equal(cache['original_geometry_valid'],packet['old__valid']))
        if not mask_exact:
            where=np.argwhere(cache['original_geometry_valid']!=packet['old__valid'])
            write(args.output/(case+'_mask_failure.json'),dict(mismatch_count=len(where),first_roi_row_candidate=where[:20].tolist(),
                source_index=packet['source_index'][where[:20,0]].tolist()))
            raise ValueError('Original geometry validity differs: '+case)
        old_valid=packet['old__valid'];angle_delta=float(np.max(abs(cache['normal_angle_rad'][old_valid]-packet['old__normal_angle_rad'][old_valid]))) if old_valid.any() else 0.
        if not np.isfinite(angle_delta) or angle_delta>1e-12:raise ValueError('Old valid-candidate normal angle differs: '+case)
        roi_checks={}
        for branch in ['F','Nalt']:
            ids=packet['saved_'+branch+'_index'];families=packet['saved_'+branch+'_family']
            known=ids>=0;reconstructed=np.full((count,3),np.nan)
            reconstructed[known]=cache['points_m'][np.flatnonzero(known),ids[known]]
            roi_checks[branch]=dict(**point_parity(reconstructed,packet['saved_point_'+branch+'_m']),**family_check(ids,families,layout))
            if not roi_checks[branch]['passes']:raise ValueError('ROI selected point differs: '+case+' '+branch)
        full=full_selected_parity(helper,case,args,layout)
        file=args.output/(case+'.npz');np.savez_compressed(file,**cache)
        case_report=dict(roi_rows=count,full_selected_rows=full['rows'],original_geometry_mask_exact=mask_exact,
            normal_angle_max_absolute_difference_rad=angle_delta,roi_selected_parity=roi_checks,full_selected_parity=full,
            roi_packet_sha256=sha(packet_path),bank_packet_sha256=sha(file),
            compression_control='No oldper-candidate compressionvalues were saved; original cone/compression admission mask reproduced exactly.',
            truth_fields_read=[])
        write(args.output/(case+'_verification.json'),case_report);cases[case]=case_report
        print(json.dumps(dict(case=case,roi_rows=count,full_selected_rows=full['rows'],mask_exact=mask_exact,
            ROI_F_byte_exact=roi_checks['F']['byte_exact'],ROI_Nalt_byte_exact=roi_checks['Nalt']['byte_exact'])),flush=True)
    verify_seal(seal)
    write(args.output/'bank_manifest.json',dict(iteration='I-67',cases=cases,all_cases_complete=True,truth_fields_read=[],new_physics=False,
        files=[dict(path=str(path.relative_to(args.output)),sha256=sha(path),bytes=path.stat().st_size)
            for path in sorted(args.output.rglob('*')) if path.is_file() and '__pycache__' not in path.parts and path.name!='bank_manifest.json']))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ['protocol','packets','i64','i66','contexts','output']:parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    for name in ['protocol','packets','i64','i66','contexts','output']:setattr(args,name,getattr(args,name).resolve())
    run(args)
