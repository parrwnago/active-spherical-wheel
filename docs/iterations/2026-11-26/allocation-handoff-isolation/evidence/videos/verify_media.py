"""Bounded I51 saved-media hashes, clocks, decoded-frame and encoding QA."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil

import imageio.v2 as imageio
import numpy as np

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def probe_checks(meta,probe,width,height,frames,fps,duration):
    stream=next(r for r in probe['streams'] if r['codec_type']=='video')
    return {'all_inputs_unchanged':all(sha(ROOT/name)==h for name,h in meta['input_sha256'].items()),
        'video_hash_matches':sha(ROOT/meta['video']['path'])==meta['video']['sha256'],
        'h264':stream['codec_name']=='h264','resolution':(stream['width'],stream['height'])==(width,height),
        'fps':stream['avg_frame_rate']==f'{fps}/1','frame_count':int(stream['nb_frames'])==frames==meta['frames'],
        'duration':abs(float(probe['format']['duration'])-duration)<1e-8,
        'no_physics_controller_or_scientific_reduction':meta['no_new_physics']and meta['no_controller_execution']and meta['no_scientific_reduction'],
        'defaults_unchanged':not meta['default_changed']}

def still_checks(names,shape):
    result={}
    for name in names:
        pixels=imageio.imread(HERE/name);result[name]={'sha256':sha(HERE/name),'shape':list(pixels.shape),'pixel_std':float(pixels.std())}
    return all(v['shape']==shape and v['pixel_std']>20 for v in result.values()),result

def primary():
    meta=read(HERE/'video_metadata.json');schedule=read(HERE/'video_display_schedule.json');checks=probe_checks(meta,read(HERE/'primary_ffprobe.json'),1800,1500,561,20,28.05)
    old=read(ROOT/'results/belt_capacity_sweep_20261126/videos/video_metadata.json')['shared_camera'];cam=meta['shared_camera']
    checks.update(primary_four_cases_only=meta['primary_cases']==['base030','reference030','base060','handoff060'],
        full44s_schedule=len(schedule)==441 and abs(schedule[0]['recorded_time_s'])<1e-8 and abs(schedule[-1]['recorded_time_s']-44)<1e-8,
        actual_clocks_match=all(set(row['case_state_times_s'])==set(meta['primary_cases'])and all(abs(t-row['recorded_time_s'])<1e-8 for t in row['case_state_times_s'].values())for row in schedule),
        all4_end_at44=all(abs(t-44.)<1e-8 for t in meta['case_end_times_s'].values()),
        same_original_camera=all(cam[k]==old[k]for k in ('azimuth_deg','elevation_deg','distance_m'))and np.allclose(cam['lookat'],old['lookat'],rtol=0,atol=1e-12)and np.allclose(cam['world_xy_bounds'],old['world_xy_bounds_m'],rtol=0,atol=1e-12),
        playback2x=meta['playback_speed']==2.,raw_integrity_passed=read(HERE.parent/'review/native_qa/stage01/report.json')['integrity_passed'])
    ok,stills=still_checks([f'decoded_{i:04d}.jpg'for i in (0,40,280,560)],[1500,1800,3]);checks['decoded_frames_nonblank']=ok
    return {'passed':all(checks.values()),'checks':{k:bool(v)for k,v in checks.items()},'video':meta['video'],'stills':stills,
        'visual_review':'Preview32.170s and finaldecoded24s/endcard inspected: fixedcamera, pairedrows, actualdivergence and mixedoutcomes readable. Screensfalse and handoff11.68deg lean retained.',
        'limitations':['Video poses sampled100ms from original5ms native logs; fulltrace error strips retain allsamples.','No combined candidate or separate fifth phasecase appears.','Mechanistic modifierbounds are separate from physical performance and acceptancegates.']}

def phase():
    meta=read(HERE/'phase_video_metadata.json');schedule=read(HERE/'phase_video_display_schedule.json');checks=probe_checks(meta,read(HERE/'phase_ffprobe.json'),1800,1200,421,20,21.05)
    primarycam=read(HERE/'video_metadata.json')['shared_camera']
    checks.update(only_separate_matched_history_pair=meta['cases']==['base030','phase_late030'],
        fixed36_44_schedule=len(schedule)==321 and schedule[0]['recorded_time_s']==36. and schedule[-1]['recorded_time_s']==44.,
        actual_clocks_match=all(set(row['case_state_times_s'])==set(meta['cases'])and all(abs(t-row['recorded_time_s'])<1e-8 for t in row['case_state_times_s'].values())for row in schedule),
        exact38_switch_labels=all(row['phase_flag']==(row['recorded_time_s']>=38.-1e-8)for row in schedule),
        same_camera_as_primary=meta['shared_camera']==primarycam,
        half_speed=meta['playback_speed']==.5,explicit_diagnostic_time_gate=meta['diagnostic_time_gate_s']==38. and not meta['deployable_policy_claim'],
        root_matched_history_QA_passed=read(HERE.parent/'review/late_phase_stage01/report.json')['passed'])
    ok,stills=still_checks([f'phase_decoded_{i:04d}.jpg'for i in (0,40,120,260,420)],[1200,1800,3]);checks['decoded_frames_nonblank']=ok
    return {'passed':all(checks.values()),'checks':{k:bool(v)for k,v in checks.items()},'video':meta['video'],'stills':stills,
        'visual_review':'Two-panel preview and finaldecoded38s/endcard inspected; clocks/switchmarker, matchedcamera and diagnostic-only qualification readable.',
        'limitations':['The38s gate is a selected episode-isolation intervention, not a deployable controller or default.','Improved late/final error in one episode does not imply a fullroute fix; movingRMS is essentiallyunchanged and slightlyworse.','Pose frames sampled25ms from original5ms native logs; plots retain allsamples.']}

def manifest(path,files):
    rows=[{'path':str(p.relative_to(ROOT)),'sha256':sha(p),'bytes':p.stat().st_size}for p in sorted(files)]
    path.write_text(json.dumps({'iteration':'I-51','files':rows},indent=2)+'\n');return len(rows)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--primary-only',action='store_true');args=parser.parse_args()
    source=Path(__file__).resolve();dest=HERE/'source_snapshot'/f'{source.stem}_{sha(source)[:12]}.py'
    if not dest.exists():shutil.copy2(source,dest)
    if not(HERE/'primary_qa.json').exists():
        p=primary();(HERE/'primary_qa.json').write_text(json.dumps(p,indent=2)+'\n')
        if not p['passed']:raise AssertionError('Preserve failed primary media QA')
    else:p=read(HERE/'primary_qa.json')
    if args.primary_only:
        selected=[f for f in HERE.iterdir()if f.is_file()and not(f.name.startswith('phase_')or f.name.startswith('late_phase')or f.name in ('render_late_phase.py','qa_execution.log','primary_media_manifest.json','artifact_manifest.json','qa.json'))]
        count=manifest(HERE/'primary_media_manifest.json',selected)
        print(json.dumps({'passed':p['passed'],'primary':p['video'],'checks':len(p['checks']),'manifest_files':count},indent=2));return
    if(HERE/'qa.json').exists():raise FileExistsError('Preserve prior final media QA')
    q=phase();(HERE/'phase_qa.json').write_text(json.dumps(q,indent=2)+'\n')
    aggregate={'iteration':'I-51','passed':p['passed']and q['passed'],'primary':p,'late_phase':q,'source_sha256':sha(source),'scope':__doc__}
    (HERE/'qa.json').write_text(json.dumps(aggregate,indent=2)+'\n')
    selected=[f for f in HERE.rglob('*')if f.is_file()and not any(part.endswith('render_scenes')or part=='__pycache__'for part in f.parts)and f.name not in ('artifact_manifest.json','qa_execution.log')]
    count=manifest(HERE/'artifact_manifest.json',selected)
    print(json.dumps({'passed':aggregate['passed'],'primary':p['video'],'phase':q['video'],'manifest_files':count},indent=2))
    if not aggregate['passed']:raise AssertionError('Preserve failed media QA')

if __name__=='__main__':main()
