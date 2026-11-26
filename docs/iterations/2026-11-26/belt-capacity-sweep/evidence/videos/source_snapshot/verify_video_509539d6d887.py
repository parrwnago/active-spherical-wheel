"""Bounded I49 media verification; no numerical campaign or controller rerun."""
from pathlib import Path
import hashlib
import json

import imageio.v2 as imageio


HERE=Path(__file__).resolve().parent
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())


def main():
    if(HERE/'qa.json').exists():raise FileExistsError('Preserve earlier media QA before a new stage')
    meta=read(HERE/'video_metadata.json');schedule=read(HERE/'video_display_schedule.json');probe=read(HERE/'ffprobe.json')
    stream=next(row for row in probe['streams']if row['codec_type']=='video')
    checks=dict(input_hashes_unchanged=all(sha(ROOT/name)==digest for name,digest in meta['input_sha256'].items()),
        video_hash_matches=sha(HERE/'belt_capacity_comparison.mp4')==meta['video']['sha256'],
        full44s_actual_schedule=len(schedule)==441 and schedule[0]['display_time_s']==0. and schedule[-1]['display_time_s']==44.,
        actual_state_clocks_match_program=all(set(row['case_state_times_s'])=={'mars_cap030','mars_cap060','mars_cap090'}and all(abs(t-row['display_time_s'])<1e-8 for t in row['case_state_times_s'].values())for row in schedule),
        same_fixed_camera_and_worldxy='shared_camera'in meta,
        original_design_and_actual_caps_explicit=meta['gain_library_and_design_unchanged']and meta['operating_caps_m_s']==[.3,.6,.9],
        motion_identity_not_input_identity=meta['motion_060_090_identical']and not meta['applied_060_090_identical'],
        no_new_simulation_or_controller=not meta['simulation_or_controller_execution'],
        no_scientific_reduction_rerun=not meta['scientific_reduction_recomputed'],
        defaults_unchanged=not meta['default_changed'],
        h264=stream['codec_name']=='h264',resolution=(stream['width'],stream['height'])==(1800,1200),
        fps=stream['avg_frame_rate']=='20/1',frames=int(stream['nb_frames'])==meta['frames']==561,
        duration=abs(float(probe['format']['duration'])-28.05)<1e-8)
    stills={}
    for name in ('decoded_frame_0000.jpg','decoded_frame_0040.jpg','decoded_frame_0280.jpg','decoded_frame_0560.jpg'):
        pixels=imageio.imread(HERE/name);stills[name]=dict(sha256=sha(HERE/name),dimensions=list(pixels.shape),pixel_std=float(pixels.std()))
    checks['decoded_frames_nonblank']=all(row['dimensions']==[1200,1800,3]and row['pixel_std']>20 for row in stills.values())
    result=dict(iteration='I-49',passed=all(checks.values()),checks=checks,video=meta['video'],stills=stills,
        scope='Only saved-media/hash/clock verification; no physics/controller/numerical analysis rerun.',
        visual_review=dict(inspected='Three-arm6.825s preview, resultcard, tracking summary, and final decoded intro/actual/endcard.',
            findings=['Same fixed camera/worldXY and chart axes across three operatingcaps.',
                'Physical motion is actual native Gazebo/DART data; title explicitly says original0.30design.',
                'Requested/applied speeds and old/owncaps separate;0.60/0.90motionidentity not conflated with appliedinputs.',
                'Mixed outcome retained: peak/final/lean improve, movingRMS worsens, settling/routes/provisional screen fail.',
                'Ideal surface-speed capacity assumption does not claim actual motor response or motor sizing validation.']))
    (HERE/'qa.json').write_text(json.dumps(result,indent=2)+'\n')
    files=[]
    for path in sorted(HERE.rglob('*')):
        if(path.is_file()and '_render_scenes'not in path.parts and '__pycache__'not in path.parts and path!=HERE/'artifact_manifest.json'and path!=HERE/'qa.log'):
            files.append(dict(path=str(path.relative_to(ROOT)),sha256=sha(path),bytes=path.stat().st_size))
    (HERE/'artifact_manifest.json').write_text(json.dumps(dict(iteration='I-49',files=files),indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'],checks=checks,video=meta['video'],manifest_files=len(files)),indent=2))
    if not result['passed']:raise RuntimeError('Media QA failed; preserve this attempt')


if __name__=='__main__':main()
