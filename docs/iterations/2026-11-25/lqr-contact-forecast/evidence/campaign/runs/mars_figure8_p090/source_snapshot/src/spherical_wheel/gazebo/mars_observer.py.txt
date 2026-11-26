"""Contact-aware control on the Mars yard with and without force sensing, with replays.

``python -m spherical_wheel.gazebo mars-observer`` runs every route of
``mars.ROUTES`` and ``mars.SHAPE_ROUTES`` in Gazebo twice from the identical
starting state:

* ``terrain_aware``: the contact-aware controller, reading the ground force
  and moment from Gazebo's solved contacts;
* ``momentum_observer``: the same controller and gains without that force
  sensing. The force and moment behind the sensed ground normal and the load
  fraction come from ``contact_sensing.MomentumObserver``, which reads only
  the state, the drive torques and the MuJoCo dynamics model. Gazebo (DART)
  integrates the motion, so the observer's model is not the one producing the
  motion.

Both controllers still read Gazebo's contacts for the per-surface loads (gain
blending), the rolling feedforward and the incoming-drive preview, and both
use the simulator state. Each route gets a side-by-side MP4 replay rendered
from the logged Gazebo states, with the sensed ground slope and its error in
each panel's caption.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import os
from pathlib import Path
import time

import numpy as np

from ..model import REPO_ROOT, load_config
from .mars import CONTROLLERS as SETTINGS
from .mars import (ROUTES, SHAPE_ROUTES, MarsYard, _workspace, build_route, evaluate_route, plot_route,
                   route_closed, route_item, route_samples)

CONTROLLERS = ('terrain_aware', 'momentum_observer')
LABELS = {'terrain_aware': 'force sensing (solved contacts)',
          'momentum_observer': 'momentum observer (no force sensing)'}
ALL_ROUTES = list(ROUTES)+list(SHAPE_ROUTES)
DEFAULT_OUTPUT = REPO_ROOT/'results'/'gazebo'/'mars_observer'


def run_job(job):
    """One route with one controller, in this worker's Gazebo Mars yard."""
    from ..offroad_control import OffroadParams
    from ..offroad_experiments import save_run
    from .experiments import run_gazebo
    name, mode, root = job['route'], job['controller'], Path(job['output'])
    workspace, yard = _workspace(job['workspace'], ALL_ROUTES)
    path, yaw = build_route(name)
    start = time.time()
    log, summary = run_gazebo(workspace, yard, path, OffroadParams(**SETTINGS[mode]), yaw_deg=yaw)
    summary = evaluate_route(log, summary, path, yard, route_closed(name))
    summary['route'] = {'name': name, 'about': route_item(name)['about'], 'initial_yaw_deg': yaw,
                        'path': path.spec(), 'start_phase_rad': .30}
    summary['wall_time_s'] = time.time()-start
    directory = save_run(root/name/mode, log, summary, yard)
    plot_route(log, summary, path, yard, directory/'plots.png', f'{name}: {LABELS[mode]}')
    return {'route': name, 'controller': mode, 'completed': summary['completed'], 'failure': summary['failure'],
            'start_qpos': np.asarray(log['qpos'][0]).tolist(), 'wall_s': summary['wall_time_s']}


def _load(directory):
    with np.load(Path(directory)/'rollout.npz') as saved:
        return {key: np.asarray(value, dtype=np.float64) if value.dtype.kind == 'f' else np.asarray(value)
                for key, value in saved.items()}


def _annotate(log, index):
    true, sensed = log['ground_normal'][index], log['estimated_normal'][index]
    slope = np.degrees(np.arccos(np.clip(sensed[2], -1., 1.)))
    error = np.degrees(np.arccos(np.clip(true@sensed/np.linalg.norm(sensed), -1., 1.)))
    return f'sensed slope {slope:4.1f} deg    sensed normal off the true ground by {error:4.1f} deg'


def render_job(job):
    """Side-by-side replay of one route's two runs."""
    os.environ.setdefault('MUJOCO_GL', 'egl')
    from ..offroad_view import render_replay
    from .mars_compare import replay_plant
    name, output = job['route'], Path(job['output'])
    path, _ = build_route(name)
    runs = []
    for mode in CONTROLLERS:
        log = _load(output/name/mode)
        log['planned_xy'] = route_samples(path, 400)
        runs.append((LABELS[mode], log))
    points = np.vstack([route_samples(path, 400)]+[log['qpos'][:, :2] for _, log in runs])
    margin = 1.6
    bounds = (float(points[:, 0].min()-margin), float(points[:, 0].max()+margin),
              float(points[:, 1].min()-margin), float(points[:, 1].max()+margin))
    plant = replay_plant(load_config(), MarsYard(), bounds, output/'_replay_scenes'/name)
    duration = max(float(log['time_s'][-1]) for _, log in runs)
    speed = 1. if duration <= 45. else 2.
    started = time.time()
    video = render_replay(plant, runs, output/name/'replay.mp4', fps=20, speed=speed, width=720, height=540,
                          azimuth=112., elevation=-24., distance=1.8, title=name.removeprefix('shape_').replace('_', ' '),
                          annotate=_annotate)
    return {'route': name, 'replay': str(video), 'speed': speed, 'render_s': time.time()-started}


def _cell(summary):
    t = summary['tracking']
    outcome = 'yes' if summary['completed'] else f"no ({summary['failure']} at {summary['completed_duration_s']:.1f} s)"
    c = summary['contact']
    return (f"{outcome} | {1e3*t['max_error_m']:.0f} / {1e3*t['final_error_m']:.0f} | "
            f"{summary['attitude']['max_error_from_reference_deg']:.1f} | "
            f"{c['normal_estimate_error_median_deg']:.2f} / {c['normal_estimate_error_p95_deg']:.1f}")


def write_report(output):
    output = Path(output)
    lines = ['# Mars yard: contact-aware control with force sensing and with the momentum observer (Gazebo)', '',
             'Each route runs twice from the identical starting state (at rest in static balance, wheel phase 0.30, '
             'shaft-driven direction along the route). **Force sensing** (`terrain_aware`) reads the ground force '
             "and moment from Gazebo's solved contacts; **momentum observer** (`momentum_observer`) gets them from "
             "`MomentumObserver` (state, drive torques "
             'and the MuJoCo dynamics model only; no contact force). Same controller, gains and settings otherwise. '
             'Both still read the solved contacts for per-surface loads, rolling feedforward and incoming-drive '
             'preview, and both use the simulator state. Physics: Gazebo Harmonic, DART, 1 ms step.', '',
             'Columns per controller: finished | max / final tracking error mm | max attitude error deg | '
             'sensed normal against the true ground normal, median / 95th percentile deg (loaded samples; on the '
             'shell rims and DEM facet edges the force-and-moment normal is not the ground normal for either source).',
             '',
             '| Route | Force sensing | | | | Momentum observer | | | | Observer vs solved contact: force median / p95 N, '
             'moment median / p95 N m | Replay |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for name in ALL_ROUTES:
        paths = [output/name/mode/'summary.json' for mode in CONTROLLERS]
        if not all(path.is_file() for path in paths):
            continue
        force, observer = (json.loads(path.read_text()) for path in paths)
        sensing = observer.get('sensing') or {}
        agreement = (f"{sensing['observer_force_error_median_N']:.2f} / {sensing['observer_force_error_p95_N']:.2f}, "
                     f"{sensing['observer_moment_error_median_Nm']:.3f} / {sensing['observer_moment_error_p95_Nm']:.3f}"
                     if sensing else '-')
        video = f'[mp4]({name}/replay.mp4)' if (output/name/'replay.mp4').is_file() else '-'
        lines.append(f'| {name} | {_cell(force)} | {_cell(observer)} | {agreement} | {video} |')
    lines += ['', '| Route | What it is |', '| --- | --- |']
    lines += [f"| {name} | {route_item(name)['about']} |" for name in ALL_ROUTES]
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')


def run(output=DEFAULT_OUTPUT, routes=None, workers=5, render=True, render_workers=4):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    names = list(routes or ALL_ROUTES)
    unknown = [name for name in names if name not in ALL_ROUTES]
    if unknown:
        raise ValueError(f'Unknown routes {unknown}; choose from {ALL_ROUTES}')
    jobs = [{'route': name, 'controller': mode, 'output': str(output), 'workspace': str(output/'_workspaces')}
            for name in names for mode in CONTROLLERS
            if not (output/name/mode/'summary.json').is_file()]
    starts = {}
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(run_job, job): job for job in jobs}
        for future in as_completed(futures):
            result = future.result()
            starts.setdefault(result['route'], []).append(np.asarray(result['start_qpos']))
            print(f"{result['route']:>22} {result['controller']:>17}: finished={result['completed']} "
                  f"failure={result['failure']} ({result['wall_s']:.0f} s)", flush=True)
            write_report(output)
    for name, pair in starts.items():
        if len(pair) == 2 and not np.allclose(pair[0], pair[1], atol=1e-9, rtol=0):
            raise RuntimeError(f'{name}: the two controllers did not start from the same state')
    write_report(output)
    if render:
        with ProcessPoolExecutor(max_workers=render_workers) as pool:
            futures = [pool.submit(render_job, {'route': name, 'output': str(output)}) for name in names]
            for future in as_completed(futures):
                result = future.result()
                print(f"replay {result['route']}: {result['replay']} ({result['speed']:g}x, "
                      f"{result['render_s']:.0f} s)", flush=True)
        write_report(output)
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python -m spherical_wheel.gazebo.mars_observer',
                                     description=__doc__.split('\n')[0])
    parser.add_argument('--routes', nargs='*', default=None, help=f'Default: all of {ALL_ROUTES}')
    parser.add_argument('--workers', type=int, default=5)
    parser.add_argument('--render-workers', type=int, default=4)
    parser.add_argument('--no-render', action='store_true')
    parser.add_argument('--output', default=str(DEFAULT_OUTPUT))
    args = parser.parse_args(argv)
    run(args.output, args.routes, args.workers, not args.no_render, args.render_workers)


if __name__ == '__main__':
    main()
