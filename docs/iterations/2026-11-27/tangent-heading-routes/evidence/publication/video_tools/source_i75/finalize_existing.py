"""Recover I75 post-encode media checks without altering encoded video bytes.

Two original render processes encoded all frames but failed while seeking for
their final decoded-frame checks. Preserve that history, decode sequentially
with one FFmpeg input/output thread, and reconstruct the missing display-only
metadata from the successful shared-camera run and recorded clocks. No physics,
controller call, image alteration, inference or metric scoring is performed.
"""
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import shutil

import imageio_ffmpeg
import numpy as np
from PIL import Image

import render


BASE=render.BASE
ROOT=render.ROOT


def record(path):
    path=Path(path)
    return dict(bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def save(path,value):
    path.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+'\n')


def main():
    stage=BASE/'publication/video_decode_recovery_stage01'
    stage.mkdir(exist_ok=False)
    source=Path(__file__).resolve()
    inputs=[source,source.parent/'render.py',BASE/'campaign/outputs_sealed.json',
            BASE/'protocol/protocol.json']
    before={}
    for phase in render.PHASES:
        folder=BASE/'publication/videos_stage01'/phase
        paths=[p for p in folder.rglob('*') if p.is_file() and 'scene' not in p.relative_to(folder).parts]
        inputs.extend(paths)
        before[phase]={str(p.relative_to(folder)):record(p) for p in paths}
        for path in paths:
            if path.suffix in {'.json','.py','.jpg'}:
                target=stage/'before'/phase/path.relative_to(folder)
                target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
    bindings={str(p.relative_to(ROOT)):record(p) for p in inputs}
    shutil.copy2(source,stage/source.name)
    save(stage/'registered_before_decode.json',dict(utc=datetime.now(timezone.utc).isoformat(),
        files=bindings,original_stage_inventory=before,decode_policy='Sequential stream; FFmpeg input/output threads=1.',
        encoded_video_bytes_will_not_change=True,physics_steps=0))
    render.verify_root_seal(BASE/'campaign/outputs_sealed.json')
    render.verify_manifests()
    # All three original renderers loaded/validated the same six recorded runs.
    # The successful p090 metadata contains shared geometry and all-six checks.
    template=json.loads((stage/'before/p090/metadata.json').read_text())
    clocks={}
    for phase in render.PHASES:
        for method in render.METHODS:
            case=phase+'_'+method
            with np.load(BASE/'campaign/runs'/case/'precision.npz',allow_pickle=False) as data:
                clocks[case]=data['time_s'].copy()
    reports={}
    for phase in render.PHASES:
        folder=BASE/'publication/videos_stage01'/phase
        video=folder/f'{phase}_N_T.mp4';video_before=record(video)
        output=stage/'decoded'/phase;output.mkdir(parents=True)
        frames=imageio_ffmpeg.read_frames(str(video),input_params=['-threads','1'],output_params=['-threads','1'])
        info=next(frames);count=0
        assert info['size']==(1600,960) and info['fps']==20.
        try:
            for index,pixels in enumerate(frames):
                count=index+1
                if index in (0,450,900):
                    Image.frombytes('RGB',info['size'],pixels).save(output/f'decoded_{index:04d}.jpg',quality=95)
        finally:
            frames.close()
        assert count==901 and abs(info['duration']-45.05)<.03
        assert record(video)==video_before
        metadata=deepcopy(template)
        metadata.update(phase=phase,requested_start_phase_rad=render.PHASES[phase])
        metadata['video']=dict(path=str(video.relative_to(ROOT)),**record(video),decoded_frames=count,duration_s=info['duration'])
        metadata['post_encode_recovery']=dict(stage='publication/video_decode_recovery_stage01',
            original_render_returncode=0 if phase=='p090' else 1,
            metadata_source='Shared-camera/all-six checks from successful original p090 render; per-phase frame clocks and exact existing encoded bytes independently verified.',
            no_new_rendering=True,encoded_video_unchanged=True,decode_policy='Sequential, one FFmpeg input/output thread')
        metadata['terminal_sample_policy']='The frame at t=45.000 s is the recorded terminal state. Its heading target/helper output is logged but UNHELD: no simulation interval follows it, and no post-45s actuation is claimed.'
        metadata['heading_motor_norm']='Euclidean norm of the logged helper motor-command torques; not a measured world yaw torque or necessarily applied torque. Terminal t=45s output is UNHELD.'
        schedule=[]
        for frame,moment in enumerate(np.arange(901)/20.):
            cases=[]
            for method in render.METHODS:
                case=phase+'_'+method;times=clocks[case];index=render.nearest(times,moment)
                cases.append(dict(case=case,requested_time_s=float(moment),pose_index=index,
                    recorded_pose_time_s=float(times[index]),run_ended_pose_frozen=bool(moment>times[-1]+1e-8)))
            schedule.append(dict(frame=frame,cases=cases))
        if (folder/'display_schedule.json').exists():
            assert json.loads((folder/'display_schedule.json').read_text())==schedule
        for image in output.glob('*.jpg'):shutil.copy2(image,folder/image.name)
        save(folder/'display_schedule.json',schedule);save(folder/'metadata.json',metadata)
        artifacts={str(p.relative_to(folder)):record(p) for p in sorted(folder.rglob('*'))
                   if p.is_file() and 'scene' not in p.relative_to(folder).parts and p.name!='artifact_manifest.json'}
        save(folder/'artifact_manifest.json',dict(files=artifacts,inputs_unchanged=True,no_physics=True,
             post_encode_recovery='publication/video_decode_recovery_stage01'))
        reports[phase]=dict(video=video_before,decoded_frames=count,duration_s=info['duration'],
             encoded_bytes_unchanged=True,terminal_output_explicitly_unheld=True)
    for name,binding in bindings.items():
        # Only decoded JPEG/metadata/manifest completion is authorized to change.
        relative=Path(name)
        if 'videos_stage01' in relative.parts and relative.name in {
                'metadata.json','display_schedule.json','artifact_manifest.json','decoded_0000.jpg','decoded_0450.jpg','decoded_0900.jpg'}:
            continue
        assert record(ROOT/relative)==binding
    save(stage/'report.json',dict(passed=True,phases=reports,all_original_encoded_video_bytes_unchanged=True,
        no_new_rendering=True,no_new_physics=True,source_and_recorded_inputs_unchanged=True,
        failures_preserved='publication/video_tools/setup_failures.json'))
    print(json.dumps(dict(passed=True,phases=reports),indent=2),flush=True)


if __name__=='__main__':main()
