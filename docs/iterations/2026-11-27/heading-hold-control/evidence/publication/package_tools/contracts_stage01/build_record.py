"""Build compact I74 evidence and a separate lossless local raw-data bundle.

Requires the root-released all-six output seal hash. Plot-only decimation is
explicitly separated from the full raw metric inputs. No inference or metrics
are recomputed, and no simulator/controller package is imported. Execute only
after the separate root post-render release so final figures/metadata are fixed.
"""
from datetime import datetime, timezone
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import tarfile

import numpy as np
from verify_released_inputs import verify as verify_released


CASE_IDS = tuple(f'{phase}_{policy}' for phase in ('m100','p030','p090') for policy in ('N','H'))
RESULT_RELATIVE = Path('results/heading_hold_control_20261127')
SMALL_SUFFIXES = {'.py','.cc','.hh','.json','.md','.log','.txt','.yaml','.yml'}
LARGE_RUN_NAMES = {'precision.npz','rollout.npz','partial_precision.npz','partial_truth.npz'}
PLOT_FIELDS = ('time_s','timed_xy_error_m','geometric_cross_track_m','goal_distance_m',
    'body_xy_speed_m_s','world_z_angular_speed_rad_s','reference_up_error_deg',
    'absolute_tilt_deg','body_up_rate_rad_s','joint_reference_delta_norm_rad_s',
    'belt_reference_delta_norm_m_s','rolling_torque_delta_norm_Nm',
    'reference_boundary_exposed','held_command_interval','any_requested_clipping',
    'heading_error_deg','heading_error_rad','absolute_heading_error_deg',
    'heading_target_rad','heading_helper_active','heading_eligible')
PRECISION_PLOT_FIELDS = ('exact_qpos','exact_qvel','exact_reference_xy','exact_reference_quat',
    'exact_requested','exact_applied','exact_reference_velocity_m_s',
    'exact_heading_reference_rad','exact_heading_error_rad','exact_heading_command',
    'heading__eligible','heading__vertical_lever','heading__free_shell_mask2')


def sha(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''): result.update(block)
    return result.hexdigest()


def save_new(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x') as stream:json.dump(value,stream,indent=2,sort_keys=True,allow_nan=False);stream.write('\n')


def record(path):
    path=Path(path)
    return dict(bytes=path.stat().st_size,sha256=sha(path))


def array_binding(value):
    value=np.asarray(value)
    return dict(dtype=value.dtype.str,shape=list(value.shape),
        sha256=hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest())


def decimation_indices(count,stride=10):
    if count<=0:raise ValueError('Nonempty trace required')
    return np.unique(np.r_[np.arange(0,count,stride,dtype=np.int64),count-1])


def selected_small_files(base,output,bundle):
    result=set()
    for folder in ('adapter','campaign_tools','protocol','runtime','evaluation','review'):
        for path in (base/folder).rglob('*'):
            if (path.is_file() and '__pycache__' not in path.parts and path.suffix in SMALL_SUFFIXES
                    and not path.is_relative_to(output) and path!=bundle):result.add(path)
    # Include failed map samples and final map fixture, but no full campaign arrays.
    result.update((base/'runtime').glob('map_stage*/map_samples.npz'))
    result.update(path for path in (base/'campaign/process_logs').glob('*.log') if path.is_file())
    result.add(base/'campaign/outputs_sealed.json')
    for case in CASE_IDS:
        folder=base/'campaign/runs'/case
        result.update(path for path in folder.iterdir() if path.is_file() and path.suffix in SMALL_SUFFIXES)
    # Tools are copied directly, excluding archive/QA directories built later.
    for path in Path(__file__).parent.iterdir():
        if path.is_file() and path.suffix in SMALL_SUFFIXES:result.add(path)
    for pattern in ('contracts_stage*','verify_stage*','source_i73'):
        for folder in Path(__file__).parent.glob(pattern):
            result.update(path for path in folder.rglob('*') if path.is_file() and path.suffix in SMALL_SUFFIXES)
    # Root-owned rendering is complete before the post-render packaging release.
    for pattern in ('plot_tools','plots_stage*','video_tools','videos_stage*','video_stage*',
                    'video_preview_stage*','video_quality_stage*','video_visual_review_stage*'):
        for folder in (base/'publication').glob(pattern):
            result.update(path for path in folder.rglob('*') if path.is_file()
                and '__pycache__' not in path.parts and path.suffix in SMALL_SUFFIXES|{'.png','.pdf','.jpg'})
    return sorted(result)


def large_files(base,repo,output,bundle):
    paths=set()
    for case in CASE_IDS:
        paths.update(path for path in (base/'campaign/runs'/case).iterdir()
                     if path.is_file() and (path.name in LARGE_RUN_NAMES or path.suffix in {'.mp4','.webm','.avi'}))
    paths.update((base/'evaluation').glob('*/*_signals.npz'))
    # Historical matched N precision inputs are explicit I74 dependencies,
    # carried losslessly without putting their large arrays in tracked evidence.
    for folder in (base/'runtime').glob('historical_native_stage*'):
        paths.update(folder.rglob('*.npz'))
    for path in base.rglob('*'):
        if (path.is_file() and path.suffix in {'.mp4','.webm','.avi'}
                and not path.is_relative_to(output) and path!=bundle):paths.add(path)
    for name in ('libSphericalWheelBridge.so','libAswExactPhysics.so'):
        path=repo/'build/gazebo'/name
        if path.is_file():paths.add(path)
    return sorted(paths)


def build(base,output,bundle,released_seal,evaluation_stage='completed_stage01'):
    base=Path(base).resolve();output=Path(output).resolve();bundle=Path(bundle).resolve()
    repo=base.parents[1]
    if base!=repo/RESULT_RELATIVE:raise ValueError('Only the dedicated original I74 result root is accepted')
    if output.exists() or bundle.exists():raise FileExistsError('Preserve existing archive/bundle stages')
    verified=verify_released(base,released_seal)
    seal_path=base/'campaign/outputs_sealed.json'
    if sha(seal_path)!=released_seal:raise ValueError('All-six seal differs from root-released hash')
    seal=json.loads(seal_path.read_text())
    expected_cases={item['case_id'] for item in seal['processes']}
    if expected_cases!=set(CASE_IDS):raise ValueError('All six process records required')
    for relative,expected in seal['files'].items():
        if record(base/relative)!=expected:raise ValueError('Sealed campaign output changed: '+relative)
    protocol_path=base/'protocol/protocol.json';protocol=json.loads(protocol_path.read_text())
    if sha(protocol_path)!=seal['protocol_sha256']:raise ValueError('Physics protocol changed')
    small=selected_small_files(base,output,bundle)
    large=large_files(base,repo,output,bundle)
    gain=repo/protocol['gain_cache']['path']
    if sha(gain)!=protocol['gain_cache']['sha256']:raise ValueError('Frozen gain cache changed')
    sources=sorted(set(small+large+[gain]))
    original={str(path.relative_to(repo)):record(path) for path in sources}
    output.mkdir(parents=True,exist_ok=False)
    save_new(output/'publication/package_tools/registered_before_packaging.json',dict(
        registered_utc=datetime.now(timezone.utc).isoformat(),all_six_released_seal_sha256=released_seal,
        original_runtime_source_count=verified['source_count'],original_runtime_input_count=verified['input_count'],
        all_original_source_gain_binary_and_sealed_raw_bindings_verified=True,
        files=original,operation='Copy bytes and plot-only every10frames decimation; no inference, metric scoring or physics.'))
    for source in small:
        target=output/source.relative_to(base);target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    cache_relative=Path('publication/frozen_inputs/control_schedule.npz')
    (output/cache_relative).parent.mkdir(parents=True,exist_ok=True);shutil.copy2(gain,output/cache_relative)
    projections={}
    for case in CASE_IDS:
        signal=base/'evaluation'/evaluation_stage/(case+'_signals.npz')
        precision=base/'campaign/runs'/case/'precision.npz'
        if not signal.exists() or not precision.exists():
            projections[case]=dict(available=False,reason='No complete canonical signal/precision pair; original failure records retained.')
            continue
        with np.load(signal,allow_pickle=False) as data:
            count=len(data['time_s']);index=decimation_indices(count)
            projected={key:data[key][index].copy() for key in PLOT_FIELDS if key in data.files}
        with np.load(precision,allow_pickle=False) as data:
            if len(data['time_s'])!=count:raise ValueError('Signal/precision clock length differs')
            if not np.array_equal(data['time_s'][index],projected['time_s']):raise ValueError('Signal/precision clock differs')
            projected.update({key:data[key][index].copy() for key in PRECISION_PLOT_FIELDS if key in data.files})
        projected['source_frame_index']=index
        relative=Path('publication/plot_samples_50ms')/(case+'.npz')
        (output/relative).parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(output/relative,**projected)
        projections[case]=dict(available=True,path=str(relative),source_signal=str(signal.relative_to(repo)),
            source_precision=str(precision.relative_to(repo)),full_frames=count,plot_frames=len(index),
            nominal_stride_s=.050,frame_stride=10,terminal_state_retained=True,
            arrays={key:array_binding(value) for key,value in projected.items()},
            scope='Plot-only decimation; NOT raw metric input, no metric replay/acceptance claim.')
    save_new(output/'publication/plot_samples_50ms/metadata.json',dict(cases=projections,
        scope='Explicit plot-only projections. Full raw metric signals/precision remain in the lossless bundle.'))
    inventory=[]
    for source in large:
        relative=source.relative_to(repo)
        inventory.append(dict(origin=str(relative),archive_member='repository/'+str(relative),**record(source),
            restore_kind='result_raw' if source.is_relative_to(base) else 'runtime_binary_verify_only'))
    bundle.parent.mkdir(parents=True,exist_ok=True)
    with tarfile.open(bundle,'w:gz') as archive:
        for item in inventory:archive.add(repo/item['origin'],arcname=item['archive_member'],recursive=False)
    bundle_record=dict(name=bundle.name,local_path=str(bundle),**record(bundle),files=inventory,
        all_six_full_metric_input_sets_present=all((base/'campaign/runs'/case/name).is_file()
            for case in CASE_IDS for name in ('precision.npz','summary.json','provenance.json')),
        scope='Lossless local/shareable bundle. Large original raw metric inputs and runtime binaries remain out of compact tracked evidence; runtime binaries are never auto-installed.')
    save_new(output/'publication/local_raw_bundle_inventory.json',bundle_record)
    dependencies=dict(sources=protocol['source_sha256'],inputs=protocol['input_sha256'],
        gain_cache=protocol['gain_cache'],bridge_binary=dict(path='build/gazebo/libSphericalWheelBridge.so',sha256=protocol['bridge_binary_sha256']),
        exact_physics_binary=dict(path='build/gazebo/libAswExactPhysics.so',sha256=protocol['exact_physics_binary_sha256']),
        policy='Restore only dedicated I74 result-owned files. Verify external source/assets/binaries; never overwrite them. Optional frozen gain cache installation refuses conflicts.')
    save_new(output/'publication/runtime_dependency_bindings.json',dependencies)
    for relative,expected in original.items():
        if record(repo/relative)!=expected:raise ValueError('Source/input changed while packaging: '+relative)
    copied={str(path.relative_to(output)):record(path) for path in sorted(output.rglob('*')) if path.is_file()}
    save_new(output/'compact_record_manifest.json',dict(iteration='I-74',
        created_utc=datetime.now(timezone.utc).isoformat(),files=copied,result_relative=str(RESULT_RELATIVE),
        frozen_gain_cache_relative=str(cache_relative),bundle_inventory='publication/local_raw_bundle_inventory.json',
        every_copied_original_byte_exact=True,original_inputs_unchanged=True,
        full_raw_metric_inputs_in_compact_record=False,
        scope='Compact research record, source/test/map history, small run reports and plot-only50ms samples; exact full-metric replay requires the separate original raw bundle.',
        root_owned_final_render_artifacts_included=True))
    print(json.dumps(dict(files=len(copied),compact_bytes=sum(item['bytes'] for item in copied.values()),
        bundle_bytes=bundle_record['bytes'],plot_cases=sum(bool(item['available']) for item in projections.values())),sort_keys=True))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--bundle',type=Path,required=True);parser.add_argument('--released-seal',required=True)
    parser.add_argument('--evaluation-stage',default='completed_stage01')
    args=parser.parse_args();build(args.base,args.output,args.bundle,args.released_seal,args.evaluation_stage)
