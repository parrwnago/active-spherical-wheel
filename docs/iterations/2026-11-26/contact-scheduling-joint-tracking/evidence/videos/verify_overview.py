"""Bounded I52 overview encoding/hash/clock/factual-caption QA only."""
from pathlib import Path
import hashlib
import json
import shutil

import imageio.v2 as imageio
import numpy as np

HERE=Path(__file__).resolve().parent
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]
PHASES=('m100','p030','p090')
VARIANTS=('B','R','N','NR')
def read(p):return json.loads(Path(p).read_text())
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    if(HERE/'qa.json').exists():raise FileExistsError('Preserve prior media QA stage')
    source=Path(__file__).resolve();snap=HERE/'source_snapshot'/f'{source.stem}_{sha(source)[:12]}.py';shutil.copy2(source,snap)
    meta=read(HERE/'video_metadata.json');schedule=read(HERE/'video_display_schedule.json');probe=read(HERE/'ffprobe.json');canonical=read(FAMILY/'analysis/stage01/report.json')
    review_path=FAMILY/'frame_audit/reducer_review/final_report.json';review=read(review_path)
    stream=next(row for row in probe['streams']if row['codec_type']=='video')
    original=read(ROOT/'results/belt_capacity_sweep_20261126/videos/video_metadata.json')['shared_camera'];camera=meta['shared_camera']
    phase_checks={}
    for chapter,phase in enumerate(PHASES):
        rows=[r for r in schedule if r['phase_key']==phase];expected=[phase+'_'+v for v in VARIANTS]
        phase_checks[phase]=dict(full_recorded0_44=len(rows)==221 and rows[0]['recorded_time_s']==0. and rows[-1]['recorded_time_s']==44.,
            all4cells_present=all(set(row['case_state_times_s'])==set(expected)for row in rows),
            actual_state_clocks_match=all(abs(t-row['recorded_time_s'])<1e-8 for row in rows for t in row['case_state_times_s'].values()),
            exact_playback_frame_map=all(row['frame']==40+chapter*251+i and abs(row['playback_time_s']-row['frame']/20)<1e-12 for i,row in enumerate(rows)),
            pose_interval200ms=all(abs(b['recorded_time_s']-a['recorded_time_s']-.2)<1e-8 for a,b in zip(rows,rows[1:])))
    checks={
        'all_render_inputs_unchanged':all(sha(ROOT/relative)==h for relative,h in meta['input_sha256'].items()),
        'video_hash_matches':sha(ROOT/meta['video']['path'])==meta['video']['sha256'],
        'all3chapters_all12cases':meta['phase_chapters']==list(PHASES)and meta['all_twelve_cases']==sorted(phase+'_'+variant for phase in PHASES for variant in VARIANTS),
        'all_phase_clock_checks':all(all(v.values())for v in phase_checks.values()),
        'same_original_camera_worldXY':all(camera[k]==original[k]for k in ('distance_m','azimuth_deg','elevation_deg'))and np.allclose(camera['lookat'],original['lookat'],rtol=0,atol=1e-12)and np.allclose(camera['world_xy_bounds'],original['world_xy_bounds_m'],rtol=0,atol=1e-12),
        'shared_plot_axes_declared':'shared_plot_limits'in meta,
        'explicit_retry_resolver_used':meta['explicit_resolver_used'],
        'raw_integrity_passed':read(FAMILY/'review/native_qa/final_stage01/report.json')['passed'],
        'canonical_integrity_passed':canonical['integrity']['passed'],
        'independent_metrics_QA_passed_for_same_canonical':review['status'].startswith('pass ')and review['canonical_report_sha256']==sha(FAMILY/'analysis/stage01/report.json'),
        'caption_all12complete44':all(c['completed']and abs(c['completed_duration_s']-44.)<1e-8 for c in canonical['cases'].values()),
        'caption_no_policy_passes_all3':not any(p['passes_all_three_phases']for p in canonical['policy_screens'].values()),
        'positive_route_results_retained':canonical['cases']['m100_N']['original_route_gates']['passed']and canonical['cases']['p090_NR']['original_route_gates']['passed'],
        'p030_NR_settled_routefail_distinct':canonical['cases']['p030_NR']['original_settling']['passed']and not canonical['cases']['p030_NR']['original_route_gates']['passed'],
        'no_new_physics_controller_or_scientific_reduction':meta['no_new_physics']and meta['no_controller_observer_forecast']and meta['no_scientific_reduction'],
        'defaults_unchanged':not meta['default_changed'],
        'h264':stream['codec_name']=='h264',
        'resolution':(stream['width'],stream['height'])==(1800,1500),
        'fps':stream['avg_frame_rate']=='20/1',
        'frame_count':int(stream['nb_frames'])==873==meta['frames'],
        'duration':abs(float(probe['format']['duration'])-43.65)<1e-8,
        '4x_overview_explicit':meta['playback_speed']==4.and meta['overview_pose_interval_s']==.2,
    }
    stills={}
    for i in (0,40,291,542,872):
        p=HERE/f'decoded_{i:04d}.jpg';pixels=imageio.imread(p);stills[p.name]=dict(sha256=sha(p),shape=list(pixels.shape),pixel_std=float(pixels.std()))
    checks['decoded_stills_nonblank']=all(v['shape']==[1500,1800,3]and v['pixel_std']>20 for v in stills.values())
    result=dict(iteration='I-52',passed=all(checks.values()),checks={k:bool(v)for k,v in checks.items()},phase_checks=phase_checks,video=meta['video'],stills=stills,
        scientific_QA_source=dict(path=str(review_path.relative_to(ROOT)),sha256=sha(review_path),meaning='Saved-metric assertions, not extra repository tests or physical trials.'),
        visual_review=dict(inspected=['Firstpreview m10032s, p090phasecard andfinalcard','Finaldecodedchapterstarts andfinalcard'],
            findings=['Alltwelve actual trajectories remain visible in fixedphaseorder with sharedcamera/scales.','N/NRoriginalroute/settling successes are separate fromfailedregisteredturnscreens.','m100Rregression is retained; no pooledaverage hidesphasefailure.','Per-frame phase is labeledrequestedlookup because a gaincanbeheld whileunloaded.']),
        limitations=['4x overview poses sample200ms of recordedtime, soindividual5ms twitches cannot be assessed from thismovie.','Full5ms plots andearlierslowclips remain available.','Reference assumptions and currentkinematicresidual are not motor-response identification, achievednoslip orhardwarevalidation.','Thirteenprocessattemptsincludeonepreservednon-scoredtechnicalcrash andoneexact-source retry; twelvevalidcells are shown.'])
    (HERE/'qa.json').write_text(json.dumps(result,indent=2)+'\n')
    files=[]
    for p in sorted(HERE.rglob('*')):
        if p.is_file()and not any(part.endswith('render_scenes')or part=='__pycache__'for part in p.parts)and p.name not in ('artifact_manifest.json','qa_execution.log'):
            files.append(dict(path=str(p.relative_to(ROOT)),sha256=sha(p),bytes=p.stat().st_size))
    (HERE/'artifact_manifest.json').write_text(json.dumps(dict(iteration='I-52',files=files),indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],checks=len(checks),phase_checks=phase_checks,video=meta['video'],manifest_files=len(files)),indent=2))
    if not result['passed']:raise AssertionError('Preserve failed media QA stage')

if __name__=='__main__':main()
