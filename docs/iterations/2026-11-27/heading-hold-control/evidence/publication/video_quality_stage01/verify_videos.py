"""Byte/frame/camera QA for I74 media, without scientific metric computation."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,shutil
import imageio.v2 as imageio
from PIL import Image,ImageDraw,ImageFont

BASE=Path(__file__).resolve().parents[2]
ROOT=Path(__file__).resolve().parents[4]
PHASES=('m100','p030','p090')


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2,sort_keys=True)+'\n')


def main():
    stage=BASE/'publication/video_quality_stage01';stage.mkdir(exist_ok=False)
    paths=[Path(__file__).resolve(),BASE/'campaign/outputs_sealed.json',
           BASE/'protocol/protocol.json']
    for phase in PHASES:
        folder=BASE/'publication/videos_stage01'/phase
        paths += [folder/n for n in ('metadata.json','display_schedule.json',f'{phase}_N_H.mp4',
                                     'decoded_0000.jpg','decoded_0450.jpg','decoded_0900.jpg')]
    bindings={str(p.relative_to(BASE)):dict(sha256=sha(p),bytes=p.stat().st_size) for p in paths}
    shutil.copy2(Path(__file__),stage/'verify_videos.py')
    save(stage/'registered_before_checks.json',dict(registered_utc=datetime.now(timezone.utc).isoformat(),
                                                   inputs=bindings,media_only=True))
    videos={};cameras=[]
    sheet=Image.new('RGB',(2400,1440),(0,0,0))
    for row,phase in enumerate(PHASES):
        folder=BASE/'publication/videos_stage01'/phase
        m=json.loads((folder/'metadata.json').read_text());v=m['video'];path=folder/f'{phase}_N_H.mp4'
        assert sha(path)==v['sha256'] and path.stat().st_size==v['bytes']
        assert m['fps']==20 and m['frames']==901 and m['dimensions']==[1600,960]
        assert m['full_recorded_window_s']==[0.,45.] and m['playback_speed']==1.
        assert m['root_output_seal_verification']['passed']
        schedule=json.loads((folder/'display_schedule.json').read_text())
        assert len(schedule)==901
        for i,frame in enumerate(schedule):
            assert frame['frame']==i and {r['case'] for r in frame['cases']}=={phase+'_'+n for n in 'NH'}
            assert all(r['requested_time_s']==i/20 for r in frame['cases'])
            assert all(abs(r['recorded_pose_time_s']-i/20)<=.0025+1e-8 or r['run_ended_pose_frozen'] for r in frame['cases'])
        with imageio.get_reader(path) as reader:
            assert reader.count_frames()==901
            assert reader.get_data(0).shape==(960,1600,3)
            duration=reader.get_meta_data()['duration'];assert abs(duration-901/20)<.03
        for col,index in enumerate((0,450,900)):
            image=Image.open(folder/f'decoded_{index:04d}.jpg').convert('RGB').resize((800,480))
            sheet.paste(image,(col*800,row*480))
        cameras.append(m['shared_camera'])
        videos[phase]=dict(path=str(path.relative_to(ROOT)),sha256=sha(path),bytes=path.stat().st_size,
                           fps=20,frames=901,duration_s=duration,dimensions=[1600,960],
                           decoded_review_indices=[0,450,900],full_recorded_window_s=[0.,45.])
    assert all(c==cameras[0] for c in cameras)
    assert all(sha(BASE/p)==b['sha256'] for p,b in bindings.items())
    sheet.save(stage/'decoded_contact_sheet.jpg',quality=95)
    report=dict(passed=True,videos=videos,one_shared_camera_all_six_paths=True,
                full_motion_and_goal_hold_preserved=True,source_and_inputs_unchanged=True,
                no_new_physics_or_scientific_metrics=True,
                contact_sheet='Rows:m100,p030,p090; columns:decodedframe0,450,900.',
                setup_failures=({'record':'publication/video_tools/setup_failures.json'} if (BASE/'publication/video_tools/setup_failures.json').exists() else {'failed_preview_invocations':0}))
    save(stage/'report.json',report)
    manifest={p.name:dict(sha256=sha(p),bytes=p.stat().st_size) for p in stage.iterdir() if p.is_file()}
    save(stage/'artifact_manifest.json',dict(files=manifest,media_only=True))
    print(json.dumps(report,indent=2),flush=True)


if __name__=='__main__':main()
