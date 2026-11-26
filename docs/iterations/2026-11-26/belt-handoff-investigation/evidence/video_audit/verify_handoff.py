"""Bounded saved-media QA for the I50 slow replay; no dynamics or controller."""
from pathlib import Path
import hashlib
import json
import shutil

import imageio.v2 as imageio

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    if(HERE/'qa.json').exists():raise FileExistsError('Preserve prior QA stage')
    source=Path(__file__).resolve();snapshot=HERE/'source_snapshot'/f'{source.stem}_{sha(source)[:12]}.py';shutil.copy2(source,snapshot)
    meta=read(HERE/'slow_video_metadata.json');schedule=read(HERE/'slow_video_schedule.json');probe=read(HERE/'ffprobe.json');scope=read(HERE.parent/'scope.json');audit=read(HERE/'event_motion_report.json')
    stream=next(s for s in probe['streams']if s['codec_type']=='video')
    original_mapping=next(r for r in audit['video_mapping']if r['frame']==360)
    checks={
        'all_original_scope_inputs_unchanged':all(sha(ROOT/p)==h for p,h in scope['inputs_sha256'].items()),
        'all_final_render_inputs_unchanged':all(sha(ROOT/p)==h for p,h in meta['input_sha256'].items()),
        'all_audit_inputs_unchanged':all(sha(ROOT/p)==h for p,h in audit['input_sha256'].items()),
        'video_hash_matches':sha(ROOT/meta['video']['path'])==meta['video']['sha256'],
        'playback18_maps_recorded32':original_mapping['display_time_s']==32. and all(abs(t-32.)<1e-8 for t in original_mapping['case_state_times_s'].values()),
        'slow_full_window':len(schedule)==301 and abs(schedule[0]['display_recorded_time_s']-30.5)<1e-8 and abs(schedule[-1]['display_recorded_time_s']-33.5)<1e-8,
        'all_3_actual_state_clocks_match':all(set(r['case_state_times_s'])=={'mars_cap030','mars_cap060','mars_cap090'}and all(abs(t-r['display_recorded_time_s'])<1e-8 for t in r['case_state_times_s'].values())for r in schedule),
        'quarter_speed_10ms_pose_sampling':meta['playback_speed']==.25 and all(abs(b['display_recorded_time_s']-a['display_recorded_time_s']-.01)<1e-8 for a,b in zip(schedule,schedule[1:])),
        'same_camera_explicit':bool(meta['shared_camera']),
        'original_actual_states':meta['motion_is_original_saved_states'],
        'no_new_physics_or_controller':not meta['new_physics']and not meta['new_controller'],
        'truth_not_controller_input':not meta['truth_controller_input'],
        'no_unique_cause_claim':not meta['causal_claim'],
        'h264':stream['codec_name']=='h264',
        'dimensions':(stream['width'],stream['height'])==(1800,1200),
        'fps':stream['avg_frame_rate']=='25/1',
        'frames':int(stream['nb_frames'])==426==meta['frames'],
        'duration':abs(float(probe['format']['duration'])-17.04)<1e-8,
    }
    stills={}
    for index in (0,50,200,425):
        p=HERE/f'slow_decoded_{index:04d}.jpg';pixels=imageio.imread(p);stills[p.name]={'sha256':sha(p),'shape':list(pixels.shape),'pixel_std':float(pixels.std())}
    checks['decoded_frames_nonblank']=all(s['shape']==[1200,1800,3]and s['pixel_std']>20 for s in stills.values())
    result={'iteration':'I-50','passed':all(checks.values()),'checks':checks,'video':meta['video'],'decoded_frames':stills,
        'limitations':['Visual poses are sampled10ms from original5ms logs; plots retain all5ms samples. Video alone cannot resolve every5ms contact toggle.',
            'Truth support is current evaluator endpoint, not5ms averaged load or hardware measurement.',
            'Preparation blend weight is not proof of achieved surface speed matching.',
            'Absolute tilt is relative to world vertical; body-up rotation speed is not an Euler-angle derivative.',
            'This is an unchanged-trajectory observational replay; no unique physical cause or improved controller is claimed.'],
        'visual_review':{'inspected':['First preview31.670s and intro/endcard','Final decoded recorded32.000s and resultcard'],
            'findings':['Shared camera and three-arm labels readable. Recorded and original-playback clocks explicit.',
                'Truth/observer support and preparation weights separate. Requested/applied beltB curves separate.',
                'Initial preview footer clipping corrected before final encoding; prior source/previews preserved.']}}
    (HERE/'qa.json').write_text(json.dumps(result,indent=2)+'\n')
    files=[]
    for p in sorted(HERE.rglob('*')):
        if p.is_file()and not any(k in p.parts for k in ('_render_scenes','__pycache__'))and p.name not in ('artifact_manifest.json','qa.log'):
            files.append({'path':str(p.relative_to(ROOT)),'sha256':sha(p),'bytes':p.stat().st_size})
    (HERE/'artifact_manifest.json').write_text(json.dumps({'iteration':'I-50','files':files},indent=2)+'\n')
    print(json.dumps({'passed':result['passed'],'checks':checks,'video':meta['video'],'manifest_files':len(files)},indent=2))
    if not result['passed']:raise RuntimeError('Media QA failed; preserve failed stage')

if __name__=='__main__':main()
