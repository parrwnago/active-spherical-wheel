"""I-66 load-free pose caches and previous-map identity controls; no truth."""
from contextlib import ExitStack
from pathlib import Path
from datetime import datetime,timezone
from unittest.mock import patch
import argparse,hashlib,importlib.util,json,shutil,sys,xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
CASES=('m100_B','p030_B','p090_B')
I64_FIELDS=('time_s','point_F_m','point_Nalt_m','proposed_family','native_family','H_F','H_Nalt','H_F_valid','H_Nalt_valid')
I61_FIELDS=('time_s','map__centre_m','map__valid','map__joint_moment_map','map__joint_at_centre','observer_wrench')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);sys.modules[name]=value;spec.loader.exec_module(value);return value
def load_fields(path,fields):
    with np.load(path,allow_pickle=False) as value:return {key:value[key].copy() for key in fields}


def verify_seal(path,verified=None,seen=None):
    verified={} if verified is None else verified;seen=set() if seen is None else seen
    path=Path(path).resolve()
    if str(path) in seen:return verified
    seen.add(str(path));seal=read(path)
    entries=seal.get('files',seal.get('sources',[])+seal.get('inputs',[]))
    if not entries:raise ValueError('Expected explicit sealed input paths')
    for item in entries:
        file=Path(item['path']);file=file if file.is_absolute() else ROOT/file
        if sha(file)!=item['sha256']:raise ValueError('Changed sealed input: '+str(file))
        verified[str(file.resolve())]=item['sha256']
    if seal.get('parent_seal'):
        parent=seal['parent_seal'];file=Path(parent['path']);file=file if file.is_absolute() else ROOT/file
        if sha(file)!=parent['sha256']:raise ValueError('Changed parent input seal')
        verify_seal(file,verified,seen)
    return verified


def resolve_i64(path,case):
    if path.is_file():return path
    options=[path/case/'primitives.npz',path/'features/portable_stage01'/case/'primitives.npz',path/'primitives.npz']
    matches=[file for file in options if file.exists()]
    if len(matches)!=1:raise ValueError('Pass a unique I64 primitive file or portable/evidence root')
    return matches[0]


def load_i61(path,case):
    if path.is_file():return load_fields(path,I61_FIELDS),[path]
    package=path if (path/'load_gate_manifest.json').exists() else path/'portable_tools/gate_stage01'
    decoder_path=package/'replay_gate.py';decoder=module(decoder_path,'_i66_old_i61_decoder');decoder.verify_manifest(package)
    metadata_path=package/case/'metadata.json';inputs_path=package/case/'inputs.npz'
    metadata=read(metadata_path)
    decoded=decoder.decode_inputs(decoder.load_npz(inputs_path),metadata['input_recipes'])
    return {key:decoded[key].copy() for key in I61_FIELDS},[decoder_path,metadata_path,inputs_path,package/'load_gate_manifest.json']


def assert_close(name,first,second,valid,tolerance=1e-10):
    if not np.any(valid):raise ValueError('Empty verification control: '+name)
    delta=np.abs(np.asarray(first)[valid]-np.asarray(second)[valid])
    if not np.isfinite(delta).all() or float(np.max(delta))>tolerance:raise ValueError('Map identity failed: '+name)
    return dict(rows=int(np.sum(valid)),max_absolute_difference=float(np.max(delta)),absolute_tolerance=tolerance)


def run(args):
    protocol=read(args.protocol)
    if protocol['iteration']!='I-66' or args.case not in CASES:raise ValueError('Unregistered iteration/case')
    seal_path=args.protocol.parent/'input_source_seal.json';verified=verify_seal(seal_path)
    # These generated XML files are self-contained except explicit mesh files.
    document=ET.parse(args.model);asset_paths=[]
    for element in document.iter():
        if element.tag=='include':raise ValueError('This audit requires the original generated self-contained model XML')
        if 'file' in element.attrib:asset_paths.append((args.model.parent/element.attrib['file']).resolve())
    for file in [args.model,*asset_paths]:
        if str(file.resolve()) not in verified:raise ValueError('Model asset not covered by input seal: '+str(file))
    source=Path(__file__).resolve();helper_path=source.with_name('geometry_maps.py')
    core_path=ROOT/'src/spherical_wheel/contact_load_consistency.py'
    geometry_path=ROOT/'results/velocity_contact_rank_20261127/campaign/features_stage01'/args.case/'source/contact_hypotheses.py'
    i64_path=resolve_i64(args.previous_i64,args.case)
    args.output.mkdir(parents=True,exist_ok=False);snapshot=args.output/'source';snapshot.mkdir()
    # Old portable packet decoding is lossless input restoration, not new scoring.
    previous61,extra_paths=load_i61(args.previous_i61,args.case)
    bound=[source,helper_path,core_path,geometry_path,args.protocol,seal_path,args.qpos_input,args.model,i64_path,*extra_paths]
    bindings=[dict(path=str(file),sha256=sha(file)) for file in bound]
    write(args.output/'registered_before_pose_arithmetic.json',dict(iteration='I-66',case=args.case,
        registered_utc=datetime.now(timezone.utc).isoformat(),sources_and_inputs=bindings,
        qpos_fields=['time_s','exact_qpos'],i64_fields=list(I64_FIELDS),i61_fields=list(I61_FIELDS),
        truth_fields_read=[],model_assets_verified=len(asset_paths),pose_cache_inputs='Known model and recorded q only; no forces used in map construction.',
        old_force_usage='Previous I61 filtered force is used only to verify its already recorded centre-load identity.',
        new_physics_steps=0,live_controller_changes=False))
    for file in [source,helper_path,core_path,geometry_path,args.protocol]:shutil.copy2(file,snapshot/file.name)
    context=load_fields(args.qpos_input,('time_s','exact_qpos'));previous64=load_fields(i64_path,I64_FIELDS)
    t=context['time_s'];n=len(t)
    if n!=8801 or abs(t[0])>1e-12 or abs(t[-1]-44)>1e-8 or not np.all(abs(np.diff(t)-.005)<1e-9):raise ValueError('Expected full original 5ms episode')
    if not np.array_equal(t,previous64['time_s']) or not np.array_equal(t,previous61['time_s']):raise ValueError('Mismatched previous/current timebases')
    import mujoco
    helper=module(helper_path,'_i66_geometry_maps');core=module(core_path,'_i66_load_core');geometry=module(geometry_path,'_i66_frozen_geometry')
    model=mujoco.MjModel.from_xml_path(str(args.model));model.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    generator=helper.EndpointPoseMaps(model,core,geometry);rows=[]
    def forbidden(*a,**k):raise RuntimeError('I66 cannot run physics or query contact')
    with ExitStack() as guard:
        for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_collision','mj_contactForce'):guard.enter_context(patch.object(mujoco,name,side_effect=forbidden))
        for q in context['exact_qpos']:rows.append(generator.evaluate(q))
    dynamic=('centre_m','linear_at_centre','angular_world','valid','patch_axis_world','hinge_axis_world','hinge_anchor_m')
    pose=dict(time_s=t.copy(),**{key:np.asarray([row[key] for row in rows]) for key in dynamic},
        radius_m=np.asarray(generator.radius_m),patch_lower_m=generator.lower.copy(),patch_upper_m=generator.upper.copy(),ancestor_active=generator.active.copy())
    controls={}
    for branch,family_key in [('F','proposed_family'),('Nalt','native_family')]:
        points=previous64['point_'+branch+'_m'];family=previous64[family_key]
        mapped=helper.assigned_point_maps(pose,points,family)
        if not np.array_equal(mapped['valid']&mapped['material_valid'],previous64['H_'+branch+'_valid']):raise ValueError('Previous physical map availability differs: '+branch)
        mask=previous64['H_'+branch+'_valid']
        controls['I64_'+branch]=assert_close(branch,mapped['H'],previous64['H_'+branch],mask)
        all_points=np.broadcast_to(points[:,None,:],(n,4,3));analytic=helper.analytical_family_point_maps(pose,all_points)
        safe=np.clip(family,0,3);Hanalytic=analytic['H'][np.arange(n),safe]
        controls['analytic_'+branch]=assert_close('analytic '+branch,Hanalytic,previous64['H_'+branch],mask)
    old_valid=previous61['map__valid']&pose['valid']
    controls['I61_centre']=assert_close('I61 centre',pose['centre_m'],previous61['map__centre_m'],old_valid)
    controls['I61_angular']=assert_close('I61 angular',pose['angular_world'],previous61['map__joint_moment_map'],old_valid)
    old_force=previous61['observer_wrench'][:,:3]
    centre_load=np.einsum('nkij,nj->nki',pose['linear_at_centre'],old_force)
    controls['I61_centre_load']=assert_close('I61 centre load',centre_load,previous61['map__joint_at_centre'],old_valid)
    centres=np.broadcast_to(pose['centre_m'][:,None,:],(n,4,3));analytic_centre=helper.analytical_family_point_maps(pose,centres)
    controls['analytic_centre_linear']=assert_close('analytic centre',analytic_centre['H'],pose['linear_at_centre'],pose['valid'])
    expected_angular=np.where(generator.active[None,:,:,None],pose['hinge_axis_world'][:,None,:,:],0.)
    controls['analytic_angular']=assert_close('analytic angular',expected_angular,pose['angular_world'],pose['valid'])
    np.savez_compressed(args.output/'pose_maps.npz',**pose)
    write(args.output/'verification.json',dict(iteration='I-66',case=args.case,numpy_version=np.__version__,controls=controls,
        rows=n,pose_valid_count=int(np.sum(pose['valid'])),truth_fields_read=[],no_new_residuals=True,
        scope='Force-independent endpoint pose cache; sampled family means must remain evaluation-only, with material flags separate from map validity.'))
    for item in bindings:
        if sha(item['path'])!=item['sha256']:raise ValueError('Bound source/input changed')
    verify_seal(seal_path)
    write(args.output/'artifact_manifest.json',dict(iteration='I-66',files=[dict(path=str(file.relative_to(args.output)),sha256=sha(file),bytes=file.stat().st_size)
        for file in sorted(args.output.rglob('*')) if file.is_file() and '__pycache__' not in file.parts and file.name!='artifact_manifest.json']))
    print(json.dumps(dict(case=args.case,rows=n,pose_maps_sealed=True,new_physics_steps=0,no_truth_read=True)),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',choices=CASES,required=True)
    for name in ['qpos-input','previous-i64','previous-i61','model','output','protocol']:p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    for name in ['qpos_input','previous_i64','previous_i61','model','output','protocol']:setattr(a,name,getattr(a,name).resolve())
    run(a)
