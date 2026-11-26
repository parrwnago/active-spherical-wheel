"""Exercise existing Mars routes with the unchanged hardware controller.

All 15 routes start at shaft phase 0.30. Figure-eight, triangle, and
out-and-back also start at phases 0.90 and -1.00. Real motion is simulated
by Gazebo/DART. Controller contacts are estimated; state inputs are exact.
"""
from concurrent.futures import ProcessPoolExecutor, as_completed
from contextlib import ExitStack
import csv
import hashlib
import json
import logging
import os
from pathlib import Path
import subprocess
import time
from unittest.mock import patch

import numpy as np

from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import save_run
from spherical_wheel.gazebo.mars import (
    ROUTES, SHAPE_ROUTES, MarsYard, _workspace, build_route, evaluate_route,
    plot_route, route_closed, route_item, route_samples,
)
from spherical_wheel.gazebo.experiments import run_gazebo

OUTPUT = Path(__file__).resolve().parent
NAMES = tuple(ROUTES)+tuple(SHAPE_ROUTES)
REPEATED = ('plain_figure8', 'shape_triangle', 'shape_line_and_back')


def case_id(name, phase):
    return f'{name}/phase_{phase:+.2f}'


def run_case(job):
    name, phase = job
    identity = case_id(name, phase)
    directory = OUTPUT/'runs'/identity
    if (directory/'summary.json').is_file():
        return brief(json.loads((directory/'summary.json').read_text()))
    logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
    workspace, yard = _workspace(OUTPUT/'_workspaces', list(NAMES))
    path, yaw = build_route(name)
    original = OffroadController.command
    calls = 0

    def forbidden(*args, **kwargs):
        raise AssertionError('Controller tried to read simulator contacts')

    def guarded(self, *args, **kwargs):
        nonlocal calls
        assert self.params.mode == 'hardware' and self.locator is not None
        assert self.plant.ground is None and self.plant.terrain is None
        assert self.plant.data.ncon == 0
        with ExitStack() as stack:
            for target in (
                'spherical_wheel.offroad_control.measure_contact',
                'spherical_wheel.offroad_control.rolling_feedforward',
                'spherical_wheel.synchronization.ContactSynchronizer._normal_loads',
                'mujoco.mj_contactForce',
            ):
                stack.enter_context(patch(target, forbidden))
            result = original(self, *args, **kwargs)
        calls += 1
        return result

    started = time.monotonic()
    with patch.object(OffroadController, 'command', guarded):
        log, summary = run_gazebo(workspace, yard, path,
                                  OffroadParams(mode='hardware'), phase=phase, yaw_deg=yaw)
    summary = evaluate_route(log, summary, path, yard, route_closed(name))
    summary['route'] = {'name': name, 'about': route_item(name)['about'],
                        'initial_yaw_deg': yaw, 'path': path.spec(),
                        'start_phase_rad': phase}
    summary['wall_time_s'] = time.monotonic()-started
    summary['contact_input_audit'] = {
        'checked_controller_calls': calls, 'forbidden_contact_api_accesses': 0,
        'controller_has_terrain_or_ground': False,
        'state_inputs': 'Exact simulated pose, velocities, encoders, and ideal applied inputs.',
        'initialization': 'Terrain-informed static balance, as in existing Mars tests.',
    }
    save_run(directory, log, summary, yard)
    plot_route(log, summary, path, yard, directory/'plots.png', f'{name}, phase {phase:+.2f}')
    return brief(summary)


def brief(summary):
    route = summary['route']
    return {
        'case': case_id(route['name'], route['start_phase_rad']),
        'route': route['name'], 'phase_rad': route['start_phase_rad'],
        'completed': summary['completed'], 'failure': summary['failure'],
        'simulated_s': summary['completed_duration_s'],
        'requested_s': summary['requested_duration_s'],
        'rms_moving_mm': 1000*summary['tracking']['rms_error_while_moving_m'],
        'p95_moving_mm': 1000*summary['mars']['p95_moving_error_m'],
        'max_error_mm': 1000*summary['tracking']['max_error_m'],
        'final_error_mm': 1000*summary['tracking']['final_error_m'],
        'max_attitude_error_deg': summary['attitude']['max_error_from_reference_deg'],
        'saturated_fraction': summary['actuation']['saturated_time_fraction'],
        'passed_all_gates': summary['mars']['passed'],
        'failed_gates': [key for key, passed in summary['mars']['gates'].items() if not passed],
        'contact_checked_calls': summary['contact_input_audit']['checked_controller_calls'],
        'wall_s': summary['wall_time_s'],
    }


def load_log(directory):
    with np.load(Path(directory)/'rollout.npz') as data:
        return {key: np.asarray(value, dtype=float) if value.dtype.kind == 'f' else np.asarray(value)
                for key, value in data.items()}


def render_case(name):
    import imageio.v2 as imageio
    from spherical_wheel.offroad_view import render_replay
    from spherical_wheel.gazebo.mars_compare import replay_plant
    directory = OUTPUT/'runs'/case_id(name, .30)
    summary = json.loads((directory/'summary.json').read_text())
    log = load_log(directory)
    path, _ = build_route(name)
    log['planned_xy'] = route_samples(path, 400)
    points = np.vstack([log['planned_xy'], log['qpos'][:, :2]])
    bounds = (float(points[:, 0].min()-1.6), float(points[:, 0].max()+1.6),
              float(points[:, 1].min()-1.6), float(points[:, 1].max()+1.6))
    plant = replay_plant(load_config(), MarsYard(), bounds, OUTPUT/'_replay_scenes'/name)
    speed = 4. if name == 'plain_patrol' else (2. if log['time_s'][-1] > 30 else 1.)
    outcome = ('Completed' if summary['completed'] else
               f"Stopped {summary['completed_duration_s']:.1f}s: {summary['failure']}")
    video = render_replay(plant, [('Estimated contacts, phase 0.30', log)],
                          OUTPUT/'videos'/f'{name}.mp4', fps=20, speed=speed,
                          width=960, height=720, azimuth=112., elevation=-24., distance=1.8,
                          title=name.replace('_', ' '),
                          annotate=lambda log, i: f'{outcome} | Exact state inputs; no sensor noise/delay.')
    with imageio.get_reader(video) as reader:
        meta = reader.get_meta_data()
        count = reader.count_frames()
        reader.get_data(count-1)
        imageio.imwrite(video.with_suffix('.jpg'), reader.get_data(count//2))
    return {'route': name, 'video': str(video), 'speed': speed,
            'video_seconds': meta['duration'], 'frames': count}


def write_results(results, videos, errors):
    ordered = [results[key] for key in sorted(results)]
    report = {
        'controller': 'Existing hardware mode: gain-scheduled LQR with estimated-contact feedforward.',
        'runs': ordered, 'videos': videos, 'execution_errors': errors,
        'finished': sum(row['completed'] for row in ordered), 'evaluated': len(ordered),
        'passed_all_gates': sum(row['passed_all_gates'] for row in ordered),
        'contact_checked_calls': sum(row['contact_checked_calls'] for row in ordered),
    }
    (OUTPUT/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
    if ordered:
        with (OUTPUT/'summary.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(ordered[0]))
            writer.writeheader()
            writer.writerows(ordered)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    jobs = [(name, .30) for name in NAMES]
    jobs += [(name, phase) for name in REPEATED for phase in (.90, -1.)]
    source_names = ('offroad_control.py', 'contact_sensing.py', 'control.py',
                    'recovery_control.py', 'synchronization.py')
    source_hashes = {name: hashlib.sha256((REPO_ROOT/'src/spherical_wheel'/name).read_bytes()).hexdigest()
                     for name in source_names}
    provenance = {
        'git_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'source_sha256': source_hashes,
        'map_commit': subprocess.check_output(['git', '-C',
            'assets/gazebo/third_party/leo_simulator-ros2', 'rev-parse', 'HEAD'], text=True).strip(),
        'jobs': jobs, 'controller_parameters': OffroadParams(mode='hardware').as_dict(),
        'simulation_workers': 4, 'render_workers': 2,
    }
    (OUTPUT/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    results, videos, errors = {}, {}, {}
    print(f'Running {len(jobs)} existing trajectory cases across {len(NAMES)} Mars routes.', flush=True)
    with ProcessPoolExecutor(max_workers=2) as rendering:
        render_futures = {}
        with ProcessPoolExecutor(max_workers=4) as simulation:
            futures = {simulation.submit(run_case, job): job for job in jobs}
            for future in as_completed(futures):
                name, phase = futures[future]
                try:
                    result = future.result()
                    results[result['case']] = result
                    print(json.dumps(result), flush=True)
                    if phase == .30:
                        render_futures[rendering.submit(render_case, name)] = name
                except Exception as error:
                    errors[case_id(name, phase)] = repr(error)
                    print(f'EXECUTION ERROR {case_id(name, phase)}: {error!r}', flush=True)
                write_results(results, videos, errors)
        for future in as_completed(render_futures):
            name = render_futures[future]
            try:
                video = future.result()
                videos[name] = video
                print('VIDEO '+json.dumps(video), flush=True)
            except Exception as error:
                errors[f'video/{name}'] = repr(error)
                print(f'RENDER ERROR {name}: {error!r}', flush=True)
            write_results(results, videos, errors)
    for name, digest in source_hashes.items():
        assert hashlib.sha256((REPO_ROOT/'src/spherical_wheel'/name).read_bytes()).hexdigest() == digest
    print(f"Finished {sum(row['completed'] for row in results.values())}/{len(results)} runs; "
          f"{sum(row['passed_all_gates'] for row in results.values())} passed all gates; "
          f'{len(videos)} videos; {len(errors)} execution errors.', flush=True)
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
