"""Bounded read-only I46 media QA: decode, clocks, hashes and presentation scope."""
from pathlib import Path
import hashlib
import json

import imageio.v2 as imageio
import numpy as np


HERE = Path(__file__).resolve().parent
FAMILY = HERE.parent
ROOT = FAMILY.parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def main():
    if (HERE/'qa.json').exists():
        raise FileExistsError('Preserve prior media QA stage before repeating')
    metadata = read(HERE/'video_metadata.json')
    schedule = read(HERE/'video_display_schedule.json')
    probe = read(HERE/'ffprobe.json')
    stream = next(row for row in probe['streams'] if row['codec_type'] == 'video')
    checks = dict(
        input_hashes_unchanged=all(sha(ROOT/name) == digest for name, digest in metadata['input_sha256'].items()),
        video_hash_matches=sha(HERE/'support_routing_timing.mp4') == metadata['video']['sha256'],
        expected_full16s_schedule=len(schedule) == 321 and schedule[0]['display_time_s'] == 0. and schedule[-1]['display_time_s'] == 16.,
        both_actual_case_clocks=all(set(row['case_state_times_s']) == {'mars_compact_outback', 'flat_compact_outback'} and
            all(abs(value-row['display_time_s']) < 1e-8 for value in row['case_state_times_s'].values()) for row in schedule),
        zero_simulation_and_controller_calls=metadata['gazebo_physics_calls'] == metadata['controller_calls'] == 0,
        shadow_unactuated=metadata['shadow_actuated'] is False,
        no_scientific_pipeline_rerun=not metadata['predictions_or_scientific_reduction_recomputed'],
        resolution=(stream['width'], stream['height']) == (1600, 1200),
        fps=stream['avg_frame_rate'] == '20/1',
        frames=int(stream['nb_frames']) == metadata['frames'] == 441,
        duration=abs(float(probe['format']['duration'])-22.05) < 1e-8,
        codec=stream['codec_name'] == 'h264',
        raw_secondary_QA_passed=read(FAMILY/'review/campaign_raw_qa_stage01/report.json')['passed'])
    stills = {}
    for name in ('decoded_frame_0000.jpg', 'decoded_frame_0040.jpg', 'decoded_frame_0220.jpg', 'decoded_frame_0440.jpg'):
        pixels = imageio.imread(HERE/name)
        stills[name] = dict(sha256=sha(HERE/name), dimensions=list(pixels.shape), pixel_std=float(pixels.std()))
    checks['decoded_stills_nonblank'] = all(row['dimensions'] == [1200, 1600, 3] and row['pixel_std'] > 20 for row in stills.values())
    report = dict(iteration='I-46', passed=all(checks.values()), checks=checks, video=metadata['video'], stills=stills,
        scope='Media/file/clock verification only; canonical scientific results unchanged.',
        visual_review=dict(
            inspected='Source previews of actual Mars7s, combined actual7s, site intro, resultcard and decoded intro/native/endcard frames.',
            parent_preview_review='Root inspected Marsactual7s+diagnostics: readable and native pose good; full16s actual video + intro/outro approved.',
            findings=['Both columns show saved native simulation; orange candidate commands are labeled never actuated.',
                'Site map retains failed strict criteria and pre-control geometry revision.',
                'Flat setup-corrected attempt is labeled; original failed setup retained separately.',
                'Endcard separates preceding-mean discrepancy from following-hold auxiliary, with no improved actuation claim.',
                'Historical last1ms-only cue follows saturation; largest command step remains unchanged; retain-filter decision explicit.']))
    (HERE/'qa.json').write_text(json.dumps(report, indent=2)+'\n')
    files = []
    for path in sorted(HERE.rglob('*')):
        if (path.is_file() and '__pycache__' not in path.parts and '_render_scenes' not in path.parts
                and path != HERE/'artifact_manifest.json' and path != HERE/'qa.log'):
            files.append(dict(path=str(path.relative_to(ROOT)), bytes=path.stat().st_size, sha256=sha(path)))
    (HERE/'artifact_manifest.json').write_text(json.dumps(dict(iteration='I-46', files=files), indent=2)+'\n')
    print(json.dumps(dict(passed=report['passed'], checks=checks, video=metadata['video'], manifest_files=len(files)), indent=2))
    if not report['passed']:
        raise RuntimeError('Media QA failed; preserve this stage')


if __name__ == '__main__':
    main()
