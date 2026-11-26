"""Read-only saved-media and causal-display QA for I45; no experiment calls."""
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
    metadata = read(HERE/'video_metadata.json')
    schedule = read(HERE/'video_display_schedule.json')
    stage = FAMILY/'campaign/stage01'
    report = read(stage/'report.json')
    manifest = read(stage/'artifact_manifest.json')
    anchors = read(stage/'anchor_records.json')
    times = np.array([row['anchor_time_s'] for row in anchors])
    probe = read(HERE/'ffprobe.json')
    stream = next(row for row in probe['streams'] if row['codec_type'] == 'video')
    checks = dict(
        main_integrity_passed=report['integrity_passed'],
        main_artifacts_unchanged=all(sha(stage/name) == entry['sha256'] for name, entry in manifest.items()),
        render_inputs_unchanged=all(sha(ROOT/name) == digest for name, digest in metadata['input_sha256'].items()),
        video_hash_matches=sha(HERE/'contact_pose_diagnostic.mp4') == metadata['video']['sha256'],
        expected_264_diagnostic_frames=len(schedule) == 264,
        expected_344_total_frames=metadata['frames'] == 344,
        fixed_windows=set(row['chapter'] for row in schedule) == {'early_bump', 'late_revisit'},
        no_prediction_recomputation=not metadata['recomputed_predictions'],
        explicit_hindsight_label=metadata['hindsight_future_pose_explicit'],
        no_physics_or_controller_calls=metadata['new_physics'] == metadata['new_controller_calls'] == 0,
        video_dimensions=(stream['width'], stream['height']) == (1600, 1200),
        video_fps=stream['avg_frame_rate'] == '20/1',
        video_frame_count=int(stream['nb_frames']) == 344,
        video_duration=abs(float(probe['format']['duration'])-17.2) < 1e-6)
    violations = []
    for row in schedule:
        index = int(np.searchsorted(times, row['display_time_s']+1e-9, side='right')-1)
        expected = anchors[index]
        if (row['anchor_index'] != expected['anchor_index']
                or row['anchor_time_s'] != expected['anchor_time_s']
                or row['anchor_time_s'] > row['display_time_s']+1e-9
                or abs(row['endpoint_time_s']-row['anchor_time_s']-.2) > 1e-8):
            violations.append(row)
    checks['latest_past_anchor_for_every_display_frame'] = not violations
    stills = {}
    for name in ('decoded_frame_0000.jpg', 'decoded_frame_0172.jpg', 'decoded_frame_0343.jpg',
                 'preview_early_bump_6.400s.jpg', 'preview_late_revisit_30.000s.jpg', 'video_resultcard.jpg'):
        pixels = imageio.imread(HERE/name)
        stills[name] = dict(dimensions=list(pixels.shape), pixel_standard_deviation=float(pixels.std()), sha256=sha(HERE/name))
    checks['decoded_and_preview_stills_nonblank'] = all(row['dimensions'] == [1200, 1600, 3] and row['pixel_standard_deviation'] > 20 for row in stills.values())
    plots = {}
    for name in ('primary_and_coverage.png', 'primary_and_coverage.pdf', 'primary_vs_legacy.png', 'primary_vs_legacy.pdf'):
        path = FAMILY/'plots'/name
        plots[name] = dict(sha256=sha(path), bytes=path.stat().st_size)
    checks['four_scientific_plot_artifacts'] = all(row['bytes'] > 1000 for row in plots.values())
    review = dict(
        author='campaign agent, direct visual inspection of decoded/pre-encode panels and report table',
        parent_preview_review='Root inspected fixed30s2x2 preview and primary_and_coverage figure; readable, truthful, wrong belt retained, fixed matching and hindsight clear; no further layout change requested.',
        findings=[
            'Fixed early6.4s panel exposes absent memory support and labels legacy target because primary is excluded.',
            'Fixed late30s panel retains wrong belt_B candidate/identity in both memory conditions; no convenient identity filter.',
            'Hindsight is explicit in lower-row titles; target is marked evaluator-only; no robot-motion improvement is presented.',
            'Result card separates all327 coverage from244/292 primary matched and legacy271 denominators.',
            'Result card retains both memory11.49→7.72mm pose benefit and absent RMS advantage plus better oracle P95; dense0/33 remains visible.',
            'No overlapping text or clipped substantive content found in inspected source and decoded stills.'
        ])
    (HERE/'visual_review.json').write_text(json.dumps(review, indent=2)+'\n')
    result = dict(iteration='I-45', passed=all(checks.values()), checks=checks,
                  display_violations=violations, stills=stills, plots=plots,
                  main_report_sha256=sha(stage/'report.json'), video=metadata['video'],
                  scope='Media/clock/table QA only. No physics, depth, mapping, forecast, controller or scientific reduction rerun.')
    (HERE/'qa.json').write_text(json.dumps(result, indent=2)+'\n')
    entries = []
    for path in sorted(HERE.rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts and path.name != 'artifact_manifest.json':
            entries.append(dict(path=str(path.relative_to(ROOT)), sha256=sha(path), bytes=path.stat().st_size))
    (HERE/'artifact_manifest.json').write_text(json.dumps(dict(iteration='I-45', files=entries), indent=2)+'\n')
    print(json.dumps(dict(passed=result['passed'], checks=checks, files=len(entries), video=metadata['video']), indent=2))
    if not result['passed']:
        raise RuntimeError('Saved-media QA failed; preserve this attempt')


if __name__ == '__main__':
    main()
