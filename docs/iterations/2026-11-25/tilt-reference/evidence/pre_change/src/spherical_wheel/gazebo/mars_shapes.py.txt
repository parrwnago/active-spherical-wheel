"""Shape trajectories on the Mars yard with the contact-aware controller, with replays.

``python -m spherical_wheel.gazebo mars-shapes`` runs each route of
``mars.SHAPE_ROUTES`` (a circle, a triangle, a straight line out and back,
and a zigzag) in Gazebo with the contact-aware controller (``terrain_aware``,
default settings), starting at rest with the shaft-driven direction along the
first leg and wheel phase 0.30, as in the main Mars suite. Each run is saved
like any other Mars route and gets a single-panel MP4 replay rendered from
the logged Gazebo states with the scene of ``mars_compare`` (the same DEM
triangulation, Leo's lava and rock meshes cropped around the route).
"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import os
from pathlib import Path
import time

import numpy as np

from ..model import REPO_ROOT, load_config
from .mars import SHAPE_ROUTES, MarsYard, build_route, route_item, route_samples, run_route

CONTROLLER = 'terrain_aware'
LABEL = 'contact-aware (force sensing)'
DEFAULT_OUTPUT = REPO_ROOT/'results'/'gazebo'/'mars_shapes'


def _job(name, output):
    return {'route': name, 'controller': CONTROLLER, 'output': str(output), 'workspace': str(output/'_workspaces'),
            'all_routes': list(SHAPE_ROUTES)}


def render_shape(job):
    """Render the single-panel replay of one shape's contact-aware run."""
    os.environ.setdefault('MUJOCO_GL', 'egl')
    from ..offroad_view import render_replay
    from .mars_compare import replay_plant
    name, output = job['route'], Path(job['output'])
    path, _ = build_route(name)
    with np.load(output/name/CONTROLLER/'rollout.npz') as saved:
        log = {key: np.asarray(value, dtype=np.float64) if value.dtype.kind == 'f' else np.asarray(value)
               for key, value in saved.items()}
    log['planned_xy'] = route_samples(path, 400)
    points = np.vstack([route_samples(path, 400), log['qpos'][:, :2]])
    margin = 1.6
    bounds = (float(points[:, 0].min()-margin), float(points[:, 0].max()+margin),
              float(points[:, 1].min()-margin), float(points[:, 1].max()+margin))
    plant = replay_plant(load_config(), MarsYard(), bounds, output/'_replay_scenes'/name)
    speed = 1. if float(log['time_s'][-1]) <= 45. else 2.
    started = time.time()
    video = render_replay(plant, [(LABEL, log)], output/name/'replay.mp4', fps=20, speed=speed, width=960, height=720,
                          azimuth=112., elevation=-24., distance=1.9, title=name.removeprefix('shape_').replace('_', ' '))
    return {'route': name, 'replay': str(video), 'speed': speed, 'render_s': time.time()-started}


def write_report(output):
    output = Path(output)
    lines = ['# Shape trajectories on the ERC 2021 Mars yard: contact-aware controller (Gazebo)', '',
             'Contact-aware controller (`terrain_aware`, default settings). Each run starts at rest in static balance '
             'with the shaft-driven direction along the first leg, wheel phase 0.30, and cruises at 0.1 m/s; polyline '
             'routes stop at every corner. Physics: Gazebo Harmonic, DART, 1 ms step. Replays are rendered from the '
             'logged Gazebo states (MuJoCo renderer, same DEM triangulation, Leo lava and rock meshes). Gates are the '
             'mobility evaluation targets (GAZEBO.md).', '',
             '| Route | Length m | Finished | Passed all gates | Failed gates | Time s | Moving RMS / max / final mm '
             '| Closure mm | Max attitude error deg | Heading excursion deg | Replay |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for name in SHAPE_ROUTES:
        summary_path = output/name/CONTROLLER/'summary.json'
        if not summary_path.is_file():
            continue
        s = json.loads(summary_path.read_text())
        t, m = s['tracking'], s['mars']
        failed = ', '.join(gate for gate, ok in m['gates'].items() if not ok) or '-'
        closure = f"{1e3*m['loop_closure_m']:.0f}" if 'closure' in m['gates'] else '-'
        video = output/name/'replay.mp4'
        lines.append(f"| {name} | {m['reference_length_m']:.1f} | {'yes' if s['completed'] else 'no: ' + str(s['failure'])} | "
                     f"{'yes' if m['passed'] else 'no'} | {failed} | {s['completed_duration_s']:.1f} | "
                     f"{1e3*t['rms_error_while_moving_m']:.0f} / {1e3*t['max_error_m']:.0f} / {1e3*t['final_error_m']:.0f} | "
                     f"{closure} | {s['attitude']['max_error_from_reference_deg']:.1f} | "
                     f"{m['max_heading_excursion_deg']:.0f} | "
                     + (f'[mp4]({name}/replay.mp4)' if video.is_file() else '-') + ' |')
    lines += ['', '| Route | What it is |', '| --- | --- |']
    lines += [f"| {name} | {route_item(name)['about']} |" for name in SHAPE_ROUTES]
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')


def run(output=DEFAULT_OUTPUT, routes=None, workers=4, render=True, render_workers=4):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    names = list(routes or SHAPE_ROUTES)
    unknown = [name for name in names if name not in SHAPE_ROUTES]
    if unknown:
        raise ValueError(f'Unknown shape routes {unknown}; choose from {list(SHAPE_ROUTES)}')
    jobs = [_job(name, output) for name in names]
    pending = [job for job in jobs if not (output/job['route']/CONTROLLER/'summary.json').is_file()]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for future in as_completed([pool.submit(run_route, job) for job in pending]):
            r = future.result()
            print(f"{r['route']}: finished={r['completed']} passed={r['passed']} failure={r['failure']} "
                  f"({r['wall_s']:.0f} s)", flush=True)
            write_report(output)
    write_report(output)
    if render:
        with ProcessPoolExecutor(max_workers=render_workers) as pool:
            for future in as_completed([pool.submit(render_shape, job) for job in jobs]):
                r = future.result()
                print(f"replay {r['route']}: {r['replay']} ({r['speed']:g}x, {r['render_s']:.0f} s)", flush=True)
        write_report(output)
    return output
