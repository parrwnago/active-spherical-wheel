"""Render saved Gazebo trajectories with estimator/control diagnostics.

From the repository root:
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 .venv/bin/python -u results/contact_diagnostics_20261125/videos/render_diagnostic_comparison.py
"""
from pathlib import Path
import json

import imageio.v2 as imageio
import mujoco
import numpy as np

from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.offroad_view import _camera, _caption, _draw
from spherical_wheel.gazebo.mars import MarsYard, build_route, route_samples
from spherical_wheel.gazebo.mars_compare import replay_plant

ROOT = REPO_ROOT/'results/contact_diagnostics_20261125'
OUTPUT = ROOT/'videos'
ROUTE = 'plain_figure8'
PHASE = 'phase_+0.90'


def load_log(variant):
    with np.load(ROOT/variant/ROUTE/PHASE/'rollout.npz') as saved:
        return {key: np.asarray(value, dtype=np.float64) if value.dtype.kind == 'f' else np.asarray(value)
                for key, value in saved.items()}


def nearest(times, moment):
    index = min(int(np.searchsorted(times, moment)), len(times)-1)
    if index and abs(times[index-1]-moment) <= abs(times[index]-moment):
        index -= 1
    return index


def vector(values, digits=2):
    return '['+', '.join(f'{value:+.{digits}f}' for value in values)+']'


def render(plant, runs, target, start_s, end_s, speed, fps=20, width=960, height=720):
    target.parent.mkdir(parents=True, exist_ok=True)
    times = np.arange(start_s, end_s+1e-8, speed/fps)
    option = mujoco.MjvOption()
    option.geomgroup[3:] = 0
    footer = f'Gazebo/DART motion | Exact state inputs | Contacts estimated | Playback {speed:g}x | One run per setting'
    saved = plant.capture()
    writer = imageio.get_writer(target, fps=fps, codec='libx264', quality=8,
                                macro_block_size=2, ffmpeg_log_level='error')
    try:
        with mujoco.Renderer(plant.model, height=height, width=width) as renderer:
            for moment in times:
                panels = []
                for label, log in runs:
                    i = nearest(log['time_s'], moment)
                    camera = _camera(log['qpos'][i, :3], 112., -24., 1.8)
                    frame = _draw(plant, renderer, log, i, camera, option, shadows=False)
                    error = np.linalg.norm(log['qpos'][i, :2]-log['reference_xy'][i])*1000.
                    joint = log['reference_qvel'][i, 6:9]
                    clip = ','.join(name for name, active in zip(
                        ('shaft', 'shell A', 'shell B', 'belt A', 'belt B'), log['actuator_saturated'][i]) if active) or 'none'
                    mismatch = log['contact_velocity_residual_rms_m_s'][i]
                    lines = [
                        f"absolute sim time {log['time_s'][i]:.3f} s | tracking error {error:.0f} mm | body lean {np.degrees(log['tilt_rad'][i]):.1f} deg",
                        'joint reference [shaft, A, B] rad/s: '+vector(joint, 1),
                        'torque request N m: '+vector(log['command'][i, :3]),
                        'torque applied N m: '+vector(log['input'][i, :3]),
                        f'rolling mismatch RMS {mismatch:.3f} m/s | command clipped: {clip}',
                    ]
                    panels.append(_caption(frame, label, lines, footer))
                writer.append_data(np.concatenate(panels, axis=1))
    finally:
        writer.close()
        plant.restore(saved)
    with imageio.get_reader(target) as reader:
        metadata = reader.get_meta_data()
        count = reader.count_frames()
        reader.get_data(count-1)
        # Select the known allocator event exactly for a useful review image.
        event_index = int(np.clip(round((31.65-start_s)*fps/speed), 0, count-1))
        imageio.imwrite(target.with_suffix('.jpg'), reader.get_data(event_index))
    return {'file': str(target), 'fps': metadata['fps'], 'size': list(metadata['size']),
            'video_seconds': metadata['duration'], 'frames': count,
            'sim_start_s': start_s, 'sim_end_s': end_s, 'playback_speed': speed}


def main():
    baseline, damped = load_log('baseline'), load_log('damped')
    path, _ = build_route(ROUTE)
    points = np.vstack([route_samples(path, 400), baseline['qpos'][:, :2], damped['qpos'][:, :2]])
    bounds = (float(points[:, 0].min()-1.6), float(points[:, 0].max()+1.6),
              float(points[:, 1].min()-1.6), float(points[:, 1].max()+1.6))
    for log in (baseline, damped):
        log['planned_xy'] = route_samples(path, 400)
    plant = replay_plant(load_config(), MarsYard(), bounds, OUTPUT/'_scene')
    runs = [('Baseline rolling allocation | figure-eight, phase +0.90', baseline),
            ('Experimental damped allocation | figure-eight, phase +0.90', damped)]
    reports = []
    reports.append(render(plant, runs, OUTPUT/'figure8_phase090_event_slow.mp4', 31.2, 32.1, .1))
    print(json.dumps(reports[-1]), flush=True)
    end = max(float(log['time_s'][-1]) for _, log in runs)
    reports.append(render(plant, runs, OUTPUT/'figure8_phase090_comparison.mp4', 0., end, 2.))
    print(json.dumps(reports[-1]), flush=True)
    (OUTPUT/'video_metadata.json').write_text(json.dumps(reports, indent=2)+'\n')


if __name__ == '__main__':
    main()
