"""Render saved baseline/bounded Gazebo figure-eight runs with honest rate labels.

MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 .venv/bin/python -u results/weighted_allocation_20261125/videos/render_bounded_comparisons.py
Only saved data are rendered. The allocator limits shown are assumed reference
constraints; measured joint rates are reported separately and need not obey them.
"""
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import argparse
import json

import imageio.v2 as imageio
import mujoco
import numpy as np

from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.offroad_view import _camera, _caption, _draw
from spherical_wheel.gazebo.mars import MarsYard, build_route, route_samples
from spherical_wheel.gazebo.mars_compare import replay_plant

OUTPUT = Path(__file__).resolve().parent
STUDY = OUTPUT.parent
BASELINE = STUDY/'baseline'
ROUTE = 'plain_figure8'
CANDIDATE = 'bounded'


def load_log(directory):
    with np.load(directory/'rollout.npz') as saved:
        return {key: np.asarray(value, dtype=np.float64) if value.dtype.kind == 'f' else np.asarray(value)
                for key, value in saved.items()}


def nearest(times, moment):
    index = min(int(np.searchsorted(times, moment)), len(times)-1)
    if index and abs(times[index-1]-moment) <= abs(times[index]-moment):
        index -= 1
    return index


def vector(values, digits=2):
    return '['+', '.join(f'{value:+.{digits}f}' for value in values)+']'


def render(plant, runs, target, start_s, end_s, speed, preview_s, fps=20):
    width, height = 960, 720
    times = np.arange(start_s, end_s+1e-8, speed/fps)
    option = mujoco.MjvOption()
    option.geomgroup[3:] = 0
    footer = f'Gazebo/DART | Exact state inputs; contacts estimated | Playback {speed:g}x | One run per setting'
    saved = plant.capture()
    writer = imageio.get_writer(target, fps=fps, codec='libx264', quality=8,
                                macro_block_size=2, ffmpeg_log_level='error')
    try:
        with mujoco.Renderer(plant.model, height=height, width=width) as renderer:
            for moment in times:
                panels = []
                for label, log, bounded in runs:
                    i = nearest(log['time_s'], moment)
                    camera = _camera(log['qpos'][i, :3], 112., -24., 1.8)
                    frame = _draw(plant, renderer, log, i, camera, option, shadows=False)
                    error = np.linalg.norm(log['qpos'][i, :2]-log['reference_xy'][i])*1000.
                    reference = (log['allocation_drive_reference'][i] if bounded else
                                 np.r_[log['reference_qvel'][i, 6:9], log['rolling_feedforward'][i, 3:]])
                    clip = ','.join(name for name, active in zip(
                        ('shaft', 'shell A', 'shell B', 'belt A', 'belt B'), log['actuator_saturated'][i]) if active) or 'none'
                    stopped = moment > log['time_s'][-1]+.005
                    status = f"sim time {log['time_s'][i]:.3f} s | tracking error {error:.0f} mm"
                    if stopped:
                        status += ' | RUN STOPPED'
                    lines = [status,
                             'joint references [shaft,A,B] rad/s: '+vector(reference[:3], 1),
                             'measured joint rates rad/s: '+vector(log['qvel'][i, 6:9], 1),
                             'belt references m/s: '+vector(reference[3:])+
                             ' | applied speed command: '+vector(log['input'][i, 3:]),
                             'torques req/applied N m: '+vector(log['command'][i, :3])+' / '+vector(log['input'][i, :3]),
                             f"measured rolling mismatch RMS {log['contact_velocity_residual_rms_m_s'][i]:.3f} m/s | clipping: {clip}"]
                    if bounded:
                        limits = log['allocation_speed_limits'][i]
                        accelerations = log['allocation_acceleration_limits'][i]
                        lines += [f'assumed reference limits: joints +/-{limits[0]:g} rad/s, belts +/-{limits[3]:g} m/s',
                                  f'reference acceleration limits: {accelerations[0]:g} rad/s2, {accelerations[3]:g} m/s2'
                                  +f" | fit residual {log['allocation_residual_rms_m_s'][i]:.3f} m/s"]
                        if CANDIDATE == 'bounded_adaptive':
                            lines.append('geometry-adaptive speed cost 0.001 to 0.005; '
                                         f"now {log['allocation_effort_weight_m_s'][i]:.4f}; change cost 0.001")
                    else:
                        lines += ['baseline allocation has no explicit reference-speed or acceleration bounds',
                                  'Reference rates are targets; measured rates are shown separately above.']
                        if CANDIDATE == 'bounded_adaptive':
                            lines.append('Baseline speed cost: none; no adaptive geometry cost.')
                    panels.append(_caption(frame, label, lines, footer))
                writer.append_data(np.concatenate(panels, axis=1))
    finally:
        writer.close()
        plant.restore(saved)
    with imageio.get_reader(target) as reader:
        metadata = reader.get_meta_data()
        count = reader.count_frames()
        reader.get_data(count-1)
        preview = int(np.clip(round((preview_s-start_s)*fps/speed), 0, count-1))
        imageio.imwrite(target.with_suffix('.jpg'), reader.get_data(preview))
    return {'file': str(target), 'fps': metadata['fps'], 'size': list(metadata['size']),
            'video_seconds': metadata['duration'], 'frames': count,
            'sim_start_s': start_s, 'sim_end_s': end_s, 'playback_speed': speed}


def render_phase(phase):
    phase_name = f'phase_{phase:+.2f}'
    baseline_dir = BASELINE/ROUTE/phase_name
    bounded_dir = STUDY/CANDIDATE/ROUTE/phase_name
    # A summary is written after the full simulator run has ended.
    assert (bounded_dir/'summary.json').is_file(), f'Run not complete: {bounded_dir}'
    baseline, bounded = load_log(baseline_dir), load_log(bounded_dir)
    path, _ = build_route(ROUTE)
    points = np.vstack([route_samples(path, 400), baseline['qpos'][:, :2], bounded['qpos'][:, :2]])
    bounds = (float(points[:, 0].min()-1.6), float(points[:, 0].max()+1.6),
              float(points[:, 1].min()-1.6), float(points[:, 1].max()+1.6))
    for log in (baseline, bounded):
        log['planned_xy'] = route_samples(path, 400)
    plant = replay_plant(load_config(), MarsYard(), bounds, OUTPUT/'_scenes'/phase_name)
    effort = float(bounded['allocation_effort_weight_m_s'][0])
    change = float(bounded['allocation_change_weight_m_s'][0])
    candidate_label = (f'Geometry-adaptive bounded allocation | phase {phase:+.2f}'
                       if CANDIDATE == 'bounded_adaptive' else
                       f'Bounded experiment: effort {effort:.3f}, change {change:.3f} | phase {phase:+.2f}')
    runs = [(f'Baseline allocation | figure-eight, phase {phase:+.2f}', baseline, False),
            (candidate_label, bounded, True)]
    token = ('p090' if phase > 0 else 'm100')
    if CANDIDATE != 'bounded':
        token = CANDIDATE+'_'+token
    if phase > 0:
        rates = np.max(np.abs(baseline['reference_qvel'][:, 6:9]), axis=1)
        event_s = float(baseline['time_s'][np.argmax(rates)])
        half_width, speed = .45, .1
        reason = 'Largest baseline joint-speed reference; same absolute simulation time in both runs.'
    else:
        # This phase was a regression in the earlier damped experiment. Show
        # the largest tracking deviation from either of these two full runs.
        candidates = []
        selected_runs = [runs[1]] if CANDIDATE == 'bounded_adaptive' else runs
        for _, log, _ in selected_runs:
            errors = np.linalg.norm(log['qpos'][:, :2]-log['reference_xy'], axis=1)
            if CANDIDATE == 'bounded_adaptive':
                i = int(np.argmax(errors))
            else:
                moving = (log['time_s'] >= 1.) & (log['time_s'] <= log['time_s'][-1]-1.)
                indices = np.flatnonzero(moving)
                i = indices[np.argmax(errors[moving])]
            candidates.append((float(errors[i]), float(log['time_s'][i])))
        _, event_s = max(candidates)
        half_width, speed = 1., .5
        reason = ('Largest adaptive-run tracking deviation; same absolute time in both panels.'
                  if CANDIDATE == 'bounded_adaptive' else
                  'Largest tracking deviation across either saved run during motion; same time in both panels.')
    end = max(float(log['time_s'][-1]) for _, log, _ in runs)
    reports = []
    if CANDIDATE != 'bounded_adaptive' or phase < 0:
        reports.append(render(plant, runs, OUTPUT/f'figure8_{token}_event.mp4',
                              max(0., event_s-half_width), min(end, event_s+half_width), speed, event_s))
    reports.append(render(plant, runs, OUTPUT/f'figure8_{token}_comparison.mp4', 0., end, 2., event_s))
    result = {'candidate': CANDIDATE, 'phase_rad': phase, 'event_time_s': event_s, 'event_selection': reason,
              'videos': reports}
    (OUTPUT/f'metadata_{token}.json').write_text(json.dumps(result, indent=2)+'\n')
    return result


def main():
    global CANDIDATE
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--phase', nargs='+', type=float, default=[.90, -1.])
    parser.add_argument('--candidate', default='bounded')
    args = parser.parse_args()
    CANDIDATE = args.candidate
    OUTPUT.mkdir(parents=True, exist_ok=True)
    with ProcessPoolExecutor(max_workers=1) as pool:
        results = list(pool.map(render_phase, args.phase))
    results = [json.loads(path.read_text()) for path in sorted(OUTPUT.glob('metadata_*.json'))]
    (OUTPUT/'video_metadata.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results, indent=2), flush=True)


if __name__ == '__main__':
    main()
