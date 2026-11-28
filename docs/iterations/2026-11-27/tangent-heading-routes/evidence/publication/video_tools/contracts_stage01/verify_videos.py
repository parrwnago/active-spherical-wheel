"""Byte/frame/camera QA for all nine I76 videos; no metric scoring."""
from datetime import datetime,timezone
from pathlib import Path
import hashlib,json,shutil
import imageio_ffmpeg
from PIL import Image

BASE=Path(__file__).resolve().parents[2]
ROOT=Path(__file__).resolve().parents[4]
ROUTES={'straight':45.,'arc':45.,'eight':133.}
PHASES=('m100','p030','p090')
FPS=10

def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,indent=2,sort_keys=True)+'\n')

def main():
    stage=BASE/'publication/video_quality_stage01';stage.mkdir(exist_ok=False)
    paths=[Path(__file__).resolve(),BASE/'campaign/outputs_sealed.json',BASE/'protocol/protocol.json']
    for route,end in ROUTES.items():
        count=int(end*FPS)+1
        for phase in PHASES:
            folder=BASE/'publication/videos_stage01'/route/phase
            paths += [folder/name for name in ('metadata.json','display_schedule.json',f'{route}_{phase}_N_T.mp4',
                *[f'decoded_{index:04d}.jpg' for index in (0,count//2,count-1)])]
    bindings={str(p.relative_to(BASE)):dict(sha256=sha(p),bytes=p.stat().st_size) for p in paths}
    shutil.copy2(Path(__file__),stage/'verify_videos.py')
    save(stage/'registered_before_checks.json',dict(utc=datetime.now(timezone.utc).isoformat(),
        inputs=bindings,media_only=True,decode_policy='Sequential stream; one FFmpeg input/output thread.'))
    videos={};cameras=[]
    for route,end in ROUTES.items():
        count=int(end*FPS)+1;sheet=Image.new('RGB',(2400,1440),(0,0,0))
        for row,phase in enumerate(PHASES):
            folder=BASE/'publication/videos_stage01'/route/phase
            metadata=json.loads((folder/'metadata.json').read_text())
            video=folder/f'{route}_{phase}_N_T.mp4';registered_video=metadata['video']
            assert sha(video)==registered_video['sha256'] and video.stat().st_size==registered_video['bytes']
            assert metadata['fps']==FPS and metadata['frames']==count and metadata['dimensions']==[1600,960]
            assert metadata['full_recorded_window_s']==[0.,end] and metadata['playback_speed']==1.
            assert metadata['root_output_seal_verification']['passed'] and registered_video['all_frames_decoded']
            assert 'UNHELD' in metadata['terminal_sample_policy']
            schedule=json.loads((folder/'display_schedule.json').read_text());assert len(schedule)==count
            for index,frame in enumerate(schedule):
                assert frame['frame']==index and {r['case'] for r in frame['cases']}=={f'{route}_{phase}_{n}' for n in 'NT'}
                assert all(r['requested_time_s']==index/FPS for r in frame['cases'])
                assert all(abs(r['recorded_pose_time_s']-index/FPS)<=.0025+1e-8 or r['run_ended_pose_frozen'] for r in frame['cases'])
            frames=imageio_ffmpeg.read_frames(str(video),input_params=['-threads','1'],output_params=['-threads','1'])
            info=next(frames);decoded=0
            assert info['size']==(1600,960) and info['fps']==FPS
            try:
                for index,pixels in enumerate(frames):decoded=index+1
            finally:frames.close()
            assert decoded==count and abs(info['duration']-count/FPS)<.03
            for col,index in enumerate((0,count//2,count-1)):
                decoded_image=Image.open(folder/f'decoded_{index:04d}.jpg').convert('RGB').resize((800,480))
                sheet.paste(decoded_image,(col*800,row*480))
            cameras.append(metadata['shared_camera'])
            videos[f'{route}_{phase}']=dict(path=str(video.relative_to(ROOT)),sha256=sha(video),bytes=video.stat().st_size,
                fps=FPS,frames=count,duration_s=info['duration'],dimensions=[1600,960],
                decoded_review_indices=[0,count//2,count-1],full_recorded_window_s=[0.,end],all_frames_decoded=True)
        sheet.save(stage/f'{route}_decoded_contact_sheet.jpg',quality=95)
    assert len(videos)==9 and all(c==cameras[0] for c in cameras)
    assert all(sha(BASE/path)==binding['sha256'] for path,binding in bindings.items())
    report=dict(passed=True,videos=videos,one_shared_camera_all_eighteen_paths=True,
        full_motion_and_goal_holds_preserved=True,all_18_cases_preserved=True,
        source_and_inputs_unchanged=True,no_new_physics_or_scientific_metrics=True,
        contact_sheets='Per route: rows m100,p030,p090; columns first/middle/final encoded frame.',
        terminal_outputs_explicitly_logged_unheld=True,setup_failures=[])
    save(stage/'report.json',report)
    save(stage/'artifact_manifest.json',dict(files={p.name:dict(sha256=sha(p),bytes=p.stat().st_size)
        for p in stage.iterdir() if p.is_file()},media_only=True))
    print(json.dumps(dict(passed=True,videos=len(videos),frames=sum(v['frames'] for v in videos.values()),
        all_shared_camera=True),indent=2),flush=True)

if __name__=='__main__':main()
