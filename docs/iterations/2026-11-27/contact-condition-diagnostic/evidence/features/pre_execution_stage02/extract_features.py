"""I-63 truth-blind causal signal extraction; private kinematics, no physics."""
from contextlib import ExitStack
from datetime import datetime,timezone
from pathlib import Path
from unittest.mock import patch
import argparse,hashlib,importlib.util,json,shutil,sys,time
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
CASES=('m100_B','p030_B','p090_B')
CONTEXT_FIELDS=('time_s','exact_qpos','exact_observer_generalized','exact_observer_interval_mean','intrinsic_normal_hint')


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def read(path):return json.loads(Path(path).read_text())
def load_allowed(path):
    with np.load(path,allow_pickle=False) as raw:return {key:raw[key].copy() for key in CONTEXT_FIELDS}


def verify_root_seal(path):
    seal=read(path);entries=seal.get('files',seal.get('sources',[])+seal.get('inputs',[]))
    if not entries:raise ValueError('Expected explicit frozen root inputs')
    for entry in entries:
        file=Path(entry['path']);file=file if file.is_absolute() else ROOT/file
        if sha(file)!=entry['sha256']:raise ValueError('Root-bound source/input changed: '+str(file))


def run(protocol_path,case,output):
    protocol=read(protocol_path)
    if protocol['iteration']!='I-63' or case not in CASES:raise ValueError('Unregistered case or iteration')
    source_path=Path(__file__).resolve();helper_path=source_path.with_name('condition_features.py')
    seal_path=protocol_path.parent/'input_source_seal.json';verify_root_seal(seal_path)
    old=ROOT/'results/velocity_contact_rank_20261127/campaign/features_stage01'/case
    context_path=old/'measurement_context.npz'
    provenance_path=ROOT/'results/balance_priority_20261126/campaign/runs'/case/'provenance.json'
    xml=Path(read(provenance_path)['models']['mirror_xml'])
    bound=[source_path,helper_path,protocol_path,seal_path,context_path,provenance_path,xml]
    output.mkdir(parents=True,exist_ok=False);source=output/'source';source.mkdir()
    bindings=[dict(path=str(path),sha256=sha(path)) for path in bound]
    write(output/'registered_before_arithmetic.json',dict(iteration='I-63',case=case,
        registered_utc=datetime.now(timezone.utc).isoformat(),sources_and_inputs=bindings,
        allowed_context_fields=list(CONTEXT_FIELDS),truth_fields_read=[],candidate_fields_read=[],
        feature_constants=dict(minimum_force_N=1.,sample_period_s=.005,time_tolerance_s=1e-9,
            joint_innovation_absolute_scale_Nm=.03,joint_innovation_relative_scale=.10,
            seam_distance_scale_m=.012,seam_rate_preview_s=.12),
        seam_definition='Signed meridional spherical arclength; shellUpper/beltLower seams; positive equatorward, negative poleward.',
        frame_definition='Observer first3 world force, next3 world moment; last3 joint scalars. Interval mean is available at interval end.',
        source_state='Simulator q is used only for known patch axes; no true contacts, terrain or loads are queried.',
        new_physics_steps=0,live_controller_changed=False))
    for file in [source_path,helper_path,protocol_path]:shutil.copy2(file,source/file.name)
    values=load_allowed(context_path)
    for value in values.values():value.flags.writeable=False
    t=values['time_s'];n=len(t)
    if n!=8801 or abs(t[0])>1e-12 or abs(t[-1]-44)>1e-8 or not np.all(np.abs(np.diff(t)-.005)<1e-9):raise ValueError('Expected complete original 5-ms episode')
    spec=importlib.util.spec_from_file_location('_i63_condition_features',helper_path);helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    import mujoco
    model=mujoco.MjModel.from_xml_path(str(xml));model.opt.disableflags|=int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    kinematics=helper.PatchAxisKinematics(model)
    axes=np.full((n,4,3),np.nan)
    def forbidden(*args,**kwargs):raise RuntimeError('I-63 extraction cannot run physics or query contact')
    started=time.monotonic()
    with ExitStack() as guards:
        for name in ('mj_step','mj_step1','mj_step2','mj_forward','mj_collision','mj_contactForce'):
            guards.enter_context(patch.object(mujoco,name,side_effect=forbidden))
        for i in range(n):axes[i]=kinematics.axes(values['exact_qpos'][i])
    primitives=dict(time_s=t.copy(),observer_generalized=values['exact_observer_generalized'].copy(),
        observer_interval_mean=values['exact_observer_interval_mean'].copy(),
        intrinsic_normal_hint=values['intrinsic_normal_hint'].copy(),patch_axis_world=axes,
        patch_lower_m=kinematics.lower.copy(),patch_upper_m=kinematics.upper.copy(),radius_m=np.asarray(kinematics.radius_m))
    conditions=helper.derive_features(primitives)
    np.savez_compressed(output/'primitives.npz',**primitives)
    np.savez_compressed(output/'conditions.npz',**conditions)
    write(output/'metadata.json',dict(iteration='I-63',case=case,rows=n,
        primitive_shapes={key:list(value.shape) for key,value in primitives.items()},
        condition_shapes={key:list(value.shape) for key,value in conditions.items()},
        patch_names=list(helper.PATCH_NAMES),minimum_force_N=1.,
        elapsed_s=time.monotonic()-started,no_truth_read=True,no_candidate_read=True,no_physics_steps=True,
        replay_qualification='Primitive axes are cached private kinematics; pure NumPy replay regenerates features, not model axes.',
        qvel_qualification='Original q/qvel remain in hash-bound measurement context; qvel is not required by this registered risk.'))
    for entry in bindings:
        if sha(entry['path'])!=entry['sha256']:raise ValueError('Bound input/source changed during extraction')
    verify_root_seal(seal_path)
    write(output/'artifact_manifest.json',dict(iteration='I-63',sealed_before_truth_utc=datetime.now(timezone.utc).isoformat(),
        files=[dict(path=str(file.relative_to(output)),sha256=sha(file),bytes=file.stat().st_size)
            for file in sorted(output.rglob('*')) if file.is_file() and '__pycache__' not in file.parts and file.name!='artifact_manifest.json']))
    print(json.dumps(dict(case=case,rows=n,primitive_bytes=(output/'primitives.npz').stat().st_size,sealed_before_truth=True,new_physics_steps=0)),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--case',choices=CASES,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();run(args.protocol.resolve(),args.case,args.output.resolve())
