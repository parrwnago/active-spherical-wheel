"""Verify I44 presentation decoding, ffprobe metadata, inputs and causal display schedule."""
from fractions import Fraction
from pathlib import Path
import hashlib
import json
import subprocess

import imageio.v2 as imageio
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    target=HERE/'qa.json'
    if target.exists():raise FileExistsError('Preserve previous final video QA')
    source=Path(__file__);snapshot=HERE/'source_snapshot'/f'verify_video_{sha(source)[:12]}.py'
    assert not snapshot.exists();snapshot.write_bytes(source.read_bytes())
    metadata=json.loads((HERE/'video_metadata.json').read_text())
    for rel,wanted in metadata['input_sha256'].items():assert sha(ROOT/rel)==wanted,rel
    video=ROOT/metadata['video']['path'];assert sha(video)==metadata['video']['sha256']
    analysis=HERE.parent/'campaign/analyze_stage01';depth_stage=HERE.parent/'campaign/depth_stage01'
    anchors=json.loads((analysis/'anchor_records.json').read_text());times=np.array([a['anchor_time_s']for a in anchors])
    with np.load(depth_stage/'depth_batches.npz')as z:
        captures=z['capture_time_s'].copy();deliveries=z['delivered_at_s'].copy()
    schedule=[]
    for chapter in metadata['chapters']:
        moments=chapter['start_s']+np.arange(0.,chapter['stop_s']-chapter['start_s']+1e-9,chapter['speed']/20)
        for moment in moments:
            ai=int(np.searchsorted(times,moment+1e-9,side='right')-1);anchor=anchors[ai];batch=anchor['latest_batch_index']
            assert times[ai]<=moment+1e-9 and deliveries[batch]<=times[ai]+1e-9
            assert batch==int(np.searchsorted(deliveries,times[ai]+1e-9,side='right')-1)
            assert abs(deliveries[batch]-captures[batch]-.05)<1e-12
            schedule.append({'display_time_s':float(moment),'anchor_time_s':float(times[ai]),
                             'latest_batch_index':batch,'capture_time_s':float(captures[batch]),'delivery_time_s':float(deliveries[batch])})
    (HERE/'causal_display_schedule.json').write_text(json.dumps({'scope':'Independent reconstruction of declared stored-anchor display policy, not a pixel-level causal proof.','frames':schedule},indent=2)+'\n')
    expected=len(schedule)+60
    ffprobe=ROOT/'build/gazebo-env/bin/ffprobe'
    process=subprocess.run([str(ffprobe),'-v','error','-select_streams','v:0','-show_entries',
        'stream=codec_name,width,height,avg_frame_rate,nb_frames,duration','-of','json',str(video)],
        check=True,capture_output=True,text=True)
    probe=json.loads(process.stdout);stream=probe['streams'][0]
    assert stream['codec_name']=='h264'and(stream['width'],stream['height'])==(1600,1200)
    assert Fraction(stream['avg_frame_rate'])==20 and int(stream['nb_frames'])==expected==326
    (HERE/'ffprobe.json').write_text(json.dumps(probe,indent=2)+'\n')
    with imageio.get_reader(video)as reader:
        info=reader.get_meta_data();count=reader.count_frames()
        frames={'early':reader.get_data(0),'late':reader.get_data(145),'last_motion':reader.get_data(count-61),'endcard':reader.get_data(count-1)}
    assert count==expected==metadata['video']['frames']
    assert info['fps']==20 and tuple(info['size'])==(1600,1200)
    for name,frame in frames.items():
        assert frame.std()>10,name
        imageio.imwrite(HERE/f'qa_decoded_{name}.jpg',frame,quality=92)
    assert np.mean(abs(frames['early'].astype(float)-frames['late'].astype(float)))>.5
    result={'iteration':'I-44','passed':True,'scope':__doc__,'source_sha256':sha(source),
        'video_sha256':sha(video),'input_hashes_verified':len(metadata['input_sha256']),
        'frames':count,'fps':20,'dimensions':[1600,1200],'duration_s':info['duration'],
        'ffprobe_verified':True,'decoded_stills':['early','late','last_motion','endcard'],
        'causal_display_schedule_frames':len(schedule),'all_schedule_arrivals_available_at_stored_anchor':True,
        'map_policy':'Only stored past-only cell snapshots; unknown regions remain explicit. No future contact/terrain marker is inserted.',
        'visual_review':'Root agent inspected initial6.4/30s previews. Final decoded screenshots require campaign reviewer inspection.',
        'limits':'Presentation integrity only. No scene raycast, map integration, controller command, dynamics or new Gazebo trial is performed.'}
    target.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':main()
