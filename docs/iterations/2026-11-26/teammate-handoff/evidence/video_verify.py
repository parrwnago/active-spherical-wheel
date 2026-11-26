"""Bounded I53 saved-media/clock/encoding QA, no simulation or controller."""
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
    if(HERE/'qa.json').exists():raise FileExistsError('Preserve previous mediaQA attempt')
    source=Path(__file__).resolve();dest=HERE/'source_snapshot'/f'{source.stem}_{sha(source)[:12]}.py';shutil.copy2(source,dest)
    meta=read(HERE/'video_metadata.json');schedule=read(HERE/'video_display_schedule.json');selection=read(HERE/'selection.json');probe=read(HERE/'ffprobe.json')
    event_path=HERE.parent/'event_audit/final_report.json';event=read(event_path);compact_path=HERE.parent/'event_audit/compact_stage02/summary.json';compact=read(compact_path)
    stream=next(s for s in probe['streams']if s['codec_type']=='video')
    cases=['m100_B','m100_R','m100_N','m100_NR']
    mapping=selection['mapping'];cue=next(r for r in mapping if r['frame']==200)
    checks=dict(
        render_inputs_unchanged=all(sha(ROOT/p)==h for p,h in meta['input_sha256'].items()),
        original_overview_unchanged=sha(ROOT/selection['source_video'])==selection['source_video_sha256'],
        exact_cue10s_is_recorded32s=cue['recorded_time_s']==32. and cue['playback_time_s']==10.,
        fixed_context9_11_maps28_36=selection['fixed_recorded_window_s']==[28,36],
        all4original_histories=meta['cases']==cases,
        full8s_slow_schedule=len(schedule)==321 and schedule[0]['recorded_time_s']==28. and schedule[-1]['recorded_time_s']==36.,
        saved_state_clocks_match=all(set(row['case_state_times_s'])==set(cases)and all(abs(t-row['recorded_time_s'])<1e-8 for t in row['case_state_times_s'].values())for row in schedule),
        exact_half_speed_clock=all(abs(row['frame']/20-row['slow_clip_time_s'])<1e-8 for row in schedule),
        continuous_original4x_clock_qualified='Continuous4x'in meta['original_fractional_clock_scope']and all(abs(row['original_overview_time_s']-(2+row['recorded_time_s']/4))<1e-10 for row in schedule),
        no_video_pose_interpolation=meta['pose_spacing_s']==.025 and meta['full_signal_spacing_s']==.005,
        evaluation_truth_only=meta['truth_evaluation_only'],
        no_new_physics_or_control=meta['no_new_physics']and meta['no_controller_observer_forecast'],
        no_scientific_gate_rerun_or_unique_gain_claim=meta['no_scientific_gate_reduction']and meta['no_unique_physical_cause_or_gain_improvement_claim'],
        defaults_unchanged=not meta['default_changed'],
        bound_event_summary=sha(compact_path)=='609b9874b5e79e24335905920638129da8a3f2e1502b26b0d622190acb8ec75a',
        bound_final_event_review=sha(event_path)=='2c1360f28b3dffa151a8099c1c041d32ee95b20401d7e0be59bd6fad88f785c3',
        source_speed_caption=abs(compact['cases']['m100_R']['windows']['playback9_11']['actual_planar_speed_max_m_s']-1.0254316598342363)<1e-12,
        video_hash_matches=sha(ROOT/meta['video']['path'])==meta['video']['sha256'],
        h264=stream['codec_name']=='h264',resolution=(stream['width'],stream['height'])==(1800,1800),
        fps=stream['avg_frame_rate']=='20/1',frames=int(stream['nb_frames'])==421==meta['frames'],
        duration=abs(float(probe['format']['duration'])-21.05)<1e-8)
    stills={}
    for i in (0,40,210,420):
        p=HERE/f'decoded_{i:04d}.jpg';pixels=imageio.imread(p);stills[p.name]=dict(sha256=sha(p),shape=list(pixels.shape),pixel_std=float(pixels.std()))
    checks['decoded_frames_nonblank']=all(v['shape']==[1800,1800,3]and v['pixel_std']>20 for v in stills.values())
    result=dict(iteration='I-53',passed=all(checks.values()),checks={k:bool(v)for k,v in checks.items()},video=meta['video'],stills=stills,
        event_review=dict(path=str(event_path.relative_to(ROOT)),sha256=sha(event_path)),
        visual_review=dict(inspected=['Preview recorded32.250s and resultcard','Final decoded trigger32.250s and resultcard'],findings=['Actual/native/used joint rates and requested/applied torque/belts clearly separated.','Observer support and saved truth endpoint separated; no solved-contact control input.','Reference bounds not conflated with actual acceleration limits.','Continuous original4x clock qualified; exact10s cue mapped by savedframe200.']),
        limitations=['25ms visual poses cannot resolve every5ms contact pulse; trace strips retain full5ms data.','50ms acceleration is backward finite difference of saved worldXY velocity, not an actuator acceleration measurement.','Different policies have distinct prior histories; observed gain/support/command chronology is not a unique physical-cause proof.'])
    (HERE/'qa.json').write_text(json.dumps(result,indent=2)+'\n')
    files=[]
    for p in sorted(HERE.rglob('*')):
        if p.is_file()and not any(part.endswith('render_scenes')or part=='__pycache__'for part in p.parts)and p.name not in ('artifact_manifest.json','qa_execution.log'):
            files.append(dict(path=str(p.relative_to(ROOT)),sha256=sha(p),bytes=p.stat().st_size))
    (HERE/'artifact_manifest.json').write_text(json.dumps(dict(iteration='I-53',files=files),indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],checks=len(checks),video=meta['video'],manifest_files=len(files)),indent=2))
    if not result['passed']:raise AssertionError('Preserve failed mediaQA stage')

if __name__=='__main__':main()
