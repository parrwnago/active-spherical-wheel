"""I-64 truth-blind endpoint maps at frozen inference points; no physics."""
from contextlib import ExitStack
from datetime import datetime,timezone
from pathlib import Path
from unittest.mock import patch
import argparse,hashlib,importlib.util,json,shutil,time
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
CASES=('m100_B','p030_B','p090_B')
CONTEXT_FIELDS=('time_s','exact_qpos','exact_observer_generalized','exact_observer_interval_mean')
I61_FIELDS=('time_s','proposed_family','native_family','proposed_candidate_index','native_candidate_index',
    'i60_accepted','proposed_point_m','native_alternative_point_m','native_point_m',
    'N_family','A_family','A_point_m','proposed__residual','proposed__valid',
    'native_alternative__residual','native_alternative__valid','C_uses_proposal')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value
def load_allowed(path,fields):
    with np.load(path,allow_pickle=False) as value:return {key:value[key].copy() for key in fields}
def verify_root_seal(path):
    seal=read(path);entries=seal.get('files',seal.get('sources',[])+seal.get('inputs',[]))
    if not entries:raise ValueError('Expected explicit root input/source seal')
    for item in entries:
        file=Path(item['path']);file=file if file.is_absolute() else ROOT/file
        if sha(file)!=item['sha256']:raise ValueError('Root-bound source/input changed: '+str(file))


def run(protocol_path,case,output):
    protocol=read(protocol_path)
    if protocol['iteration']!='I-64' or case not in CASES:raise ValueError('Unregistered iteration or case')
    seal=protocol_path.parent/'input_source_seal.json';verify_root_seal(seal)
    source=Path(__file__).resolve();helper_source=source.with_name('history_features.py')
    i60=ROOT/'results/velocity_contact_rank_20261127/campaign/features_stage01'/case
    context_path=i60/'measurement_context.npz'
    i61_path=ROOT/'results/contact_load_consistency_20261127/campaign/features_stage01'/case/'load_features_and_policies.npz'
    provenance_path=ROOT/'results/balance_priority_20261126/campaign/runs'/case/'provenance.json'
    xml=Path(read(provenance_path)['models']['mirror_xml'])
    core_source=ROOT/'src/spherical_wheel/contact_load_consistency.py';geometry_source=i60/'source/contact_hypotheses.py'
    bound=[source,helper_source,protocol_path,seal,context_path,i61_path,provenance_path,xml,core_source,geometry_source]
    output.mkdir(parents=True,exist_ok=False);snapshot=output/'source';snapshot.mkdir()
    bindings=[dict(path=str(path),sha256=sha(path)) for path in bound]
    write(output/'registered_before_arithmetic.json',dict(iteration='I-64',case=case,
        registered_utc=datetime.now(timezone.utc).isoformat(),sources_and_inputs=bindings,
        allowed_context_fields=list(CONTEXT_FIELDS),allowed_i61_fields=list(I61_FIELDS),truth_fields_read=[],
        constants=dict(cutoff_rad_s=100.,warmup_s=.2,sample_period_s=.005,time_tolerance_s=1e-9),
        history='Frozen inferred A and separate native-equivalent shadow paths. Observed seeds,200ms warmup; current alternatives never modify prior history.',
        geometry='Endpoint only: frozen F and physical native alternative plus unprojected virtual native equivalent.',
        new_physics_steps=0,live_controller_changed=False))
    for file in [source,helper_source,protocol_path,core_source,geometry_source]:shutil.copy2(file,snapshot/file.name)
    context=load_allowed(context_path,CONTEXT_FIELDS);old=load_allowed(i61_path,I61_FIELDS)
    for values in [context,old]:
        for value in values.values():value.flags.writeable=False
    t=context['time_s'];n=len(t)
    if n!=8801 or not np.array_equal(t,old['time_s']) or abs(t[0])>1e-12 or abs(t[-1]-44)>1e-8 or not np.all(abs(np.diff(t)-.005)<1e-9):raise ValueError('Expected matched original complete episodes')
    if not np.array_equal(old['native_family'],old['N_family']):raise ValueError('Frozen native families disagree')
    expected_family=np.where(old['i60_accepted'],old['proposed_family'],old['native_family'])
    expected_point=np.where(old['i60_accepted'][:,None],old['proposed_point_m'],old['native_point_m'])
    if not np.array_equal(expected_family,old['A_family']) or not np.array_equal(expected_point,old['A_point_m'],equal_nan=True):raise ValueError('Frozen A path changed')
    helper=module(helper_source,'_i64_history_helper');core=module(core_source,'_i64_load_core');geometry=module(geometry_source,'_i64_frozen_geometry')
    import mujoco
    model=mujoco.MjModel.from_xml_path(str(xml));model.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    maps=helper.FrozenEndpointMaps(model,core,geometry)
    pack=dict(time_s=t.copy(),observer_generalized=context['exact_observer_generalized'].copy(),
        observer_interval_mean=context['exact_observer_interval_mean'].copy(),radius_m=np.asarray(maps.radius_m),
        i60_accepted=old['i60_accepted'].copy(),proposed_family=old['proposed_family'].copy(),native_family=old['native_family'].copy(),
        proposed_candidate_index=old['proposed_candidate_index'].copy(),native_candidate_index=old['native_candidate_index'].copy(),
        I61_F_residual_Nm=old['proposed__residual'][:,6:9].copy(),I61_F_valid=old['proposed__valid'].copy(),
        I61_Nalt_residual_Nm=old['native_alternative__residual'][:,6:9].copy(),I61_Nalt_valid=old['native_alternative__valid'].copy(),
        I61_C_uses_proposal=old['C_uses_proposal'].copy())
    choices=[('F','proposed_point_m','proposed_family'),('Nalt','native_alternative_point_m','native_family'),('Neq','native_point_m','native_family')]
    for label,point_key,_ in choices:
        pack['point_'+label+'_m']=old[point_key].copy();pack['H_'+label]=np.full((n,3,3),np.nan)
        pack['H_'+label+'_valid']=np.zeros(n,bool);pack['material_'+label+'_valid']=np.zeros(n,bool)
    def forbidden(*args,**kwargs):raise RuntimeError('I-64 cannot run physics or query contacts')
    started=time.monotonic()
    with ExitStack() as guards:
        for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_collision','mj_contactForce'):guards.enter_context(patch.object(mujoco,name,side_effect=forbidden))
        for i in range(n):
            for label,point_key,family_key in choices:
                value=maps.point_map(context['exact_qpos'][i],old[point_key][i],old[family_key][i])
                pack['H_'+label][i]=value['H'];pack['material_'+label+'_valid'][i]=value['material_point_valid']
                pack['H_'+label+'_valid'][i]=value['map_valid'] and (value['material_point_valid'] or label=='Neq')
    derived=helper.derive_history(pack)
    np.savez_compressed(output/'primitives.npz',**pack);np.savez_compressed(output/'history_features.npz',**derived)
    write(output/'metadata.json',dict(iteration='I-64',case=case,rows=n,numpy_version=np.__version__,
        elapsed_s=time.monotonic()-started,primitive_shapes={key:list(value.shape) for key,value in pack.items()},
        derived_shapes={key:list(value.shape) for key,value in derived.items()},no_truth_read=True,no_physics_steps=True,
        kinematic_replay='Cached H maps; NumPy replay does not regenerate endpoint Jacobians.',
        endpoint_qualification='Raw interval-average force times endpoint Jacobian is an approximation; actual within-interval contact history is unknown.'))
    for item in bindings:
        if sha(item['path'])!=item['sha256']:raise ValueError('Bound input/source changed during extraction')
    verify_root_seal(seal)
    write(output/'artifact_manifest.json',dict(iteration='I-64',sealed_before_truth_utc=datetime.now(timezone.utc).isoformat(),
        files=[dict(path=str(file.relative_to(output)),sha256=sha(file),bytes=file.stat().st_size)
            for file in sorted(output.rglob('*')) if file.is_file() and '__pycache__' not in file.parts and file.name!='artifact_manifest.json']))
    print(json.dumps(dict(case=case,rows=n,sealed_before_truth=True,new_physics_steps=0)),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--case',choices=CASES,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run(args.protocol.resolve(),args.case,args.output.resolve())
