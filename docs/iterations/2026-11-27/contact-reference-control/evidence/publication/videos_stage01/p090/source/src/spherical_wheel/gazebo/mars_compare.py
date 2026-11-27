"""Original lab local LQR against the contact-aware controller on the Mars yard, with replays.

``python -m spherical_wheel.gazebo mars-lqr`` runs every Mars route twice in
Gazebo from one identical starting state: once with the original interactive
lab's ``local_lqr`` command law and gain selector (the unmodified
``LocalLQRAdapter`` of ``scripts/compare-local-lqr-offroad.py``) and once with
the contact-aware controller. Both start at heading 0 and wheel phase 0.30:
the local LQR uses raw world-frame errors with gains designed at heading 0,
so a turned start would not be its own operating point.

Replays are rendered from the Gazebo-simulated logs with the project's MuJoCo
renderer (``offroad_view.render_replay``): the triangulated Mars DEM the
physics used, plus Leo's lava and rock meshes cropped around the route. The
renderer only draws logged states; it simulates nothing.
"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, as_completed
import copy
import importlib.util
import json
import os
from pathlib import Path
import time
from types import FunctionType
import xml.etree.ElementTree as ET

import numpy as np

from ..control import jsonable
from ..model import REPO_ROOT, load_config
from ..offroad_control import OffroadParams
from .mars import ROUTES, MarsYard, build_route, evaluate_route, feature_triangles, route_samples

CONTROLLERS = ('local_lqr', 'terrain_aware')
# Every pair starts at heading 0, so a route that differs from another only in
# its starting heading would repeat that run.
COMPARED_ROUTES = [name for name in ROUTES if name != 'plain_traverse_shell']
LABELS = {'local_lqr': 'original local LQR', 'terrain_aware': 'contact-aware (force sensing)'}
DEFAULT_OUTPUT = REPO_ROOT/'results'/'gazebo'/'mars_local_lqr'
START_PHASE = .30
START_YAW_DEG = 0.


def _adapter_class():
    """The comparison script's LocalLQRAdapter, loaded from its file unmodified."""
    script = REPO_ROOT/'scripts'/'compare-local-lqr-offroad.py'
    spec = importlib.util.spec_from_file_location('compare_local_lqr_offroad', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.LocalLQRAdapter


def run_local_lqr(workspace, terrain, path, phase=START_PHASE, yaw_deg=START_YAW_DEG):
    """``run_offroad`` with the original local LQR, as the comparison script binds it."""
    from .. import offroad_experiments as experiments
    adapter = _adapter_class()

    class BoundLocalLQR(adapter):
        pass
    BoundLocalLQR.path = path
    created = []

    def factory(*args, **kwargs):
        controller = BoundLocalLQR(*args, **kwargs)
        created.append(controller)
        return controller
    original = experiments.run_offroad
    runner = FunctionType(original.__code__, dict(original.__globals__, OffroadController=factory),
                          original.__name__, original.__defaults__, original.__closure__)
    log, summary = runner(workspace, terrain, path, OffroadParams(mode='flat_baseline', synchronize=False),
                          phase=phase, yaw_deg=yaw_deg)
    summary['controller'] = 'local_lqr'
    summary['parameters']['mode'] = 'local_lqr'
    lab = created[0].lab
    summary['local_lqr_schedule'] = {'phase_rad': [p['phase'] for p in lab.schedule],
                                     'regimes': [p['regime'] for p in lab.schedule],
                                     'rejected_points': lab.failed_gain_points,
                                     'rule': 'Unmodified SimulationLab._select_gain; planner extended to both path x extrema.'}
    return log, summary


def run_pair(job):
    """Both controllers on one route from the same start, in this worker's Gazebo yard."""
    from ..offroad_experiments import run_offroad, save_run
    from .experiments import GAZEBO_VERSIONS
    from .mars import _workspace
    name, output = job['route'], Path(job['output'])
    workspace, yard = _workspace(job['workspace'], list(ROUTES))
    path, _ = build_route(name)
    closed = ROUTES[name]['path'][0] in ('circle', 'figure8') or (ROUTES[name]['path'][0] == 'spline' and ROUTES[name]['path'][2])
    results, starts = [], []
    for mode in CONTROLLERS:
        start = time.time()
        if mode == 'local_lqr':
            log, summary = run_local_lqr(workspace, yard, path)
        else:
            log, summary = run_offroad(workspace, yard, path, OffroadParams(), phase=START_PHASE, yaw_deg=START_YAW_DEG)
        plant = workspace.plant(yard)
        summary['engine'] = dict(GAZEBO_VERSIONS, ground=plant.ground_kind, world_info=plant.world_info)
        summary['engine_version'] = GAZEBO_VERSIONS['simulator']
        summary = evaluate_route(log, summary, path, yard, closed)
        summary['route'] = {'name': name, 'about': ROUTES[name]['about'], 'initial_yaw_deg': START_YAW_DEG,
                            'start_phase_rad': START_PHASE, 'path': path.spec()}
        summary['wall_time_s'] = time.time()-start
        save_run(output/name/mode, log, summary, yard)
        starts.append(np.asarray(log['qpos'][0]))
        results.append({'controller': mode, 'completed': summary['completed'], 'failure': summary['failure'],
                        'passed': summary['mars']['passed'], 'duration_s': summary['completed_duration_s'],
                        'max_mm': 1e3*summary['tracking']['max_error_m'], 'final_mm': 1e3*summary['tracking']['final_error_m']})
    if not np.allclose(starts[0], starts[1], atol=1e-9, rtol=0):
        raise RuntimeError(f'{name}: the two controllers did not start from the same state')
    return {'route': name, 'runs': results}


# -- replay scene --------------------------------------------------------------------------
def _crop_features(bounds, directory):
    """Write the lava/rock triangles near ``bounds`` as OBJ meshes; return their paths."""
    x0, x1, y0, y1 = bounds
    paths = []
    for name, triangles in feature_triangles().items():
        inside = ((triangles[:, :, 0] >= x0) & (triangles[:, :, 0] <= x1) &
                  (triangles[:, :, 1] >= y0) & (triangles[:, :, 1] <= y1)).any(axis=1)
        if inside.sum() < 4:
            continue
        kept = triangles[inside].reshape(-1, 3)
        path = Path(directory)/f'{name}_crop.obj'
        with path.open('w') as out:
            np.savetxt(out, kept, fmt='v %.5f %.5f %.5f')
            faces = np.arange(len(kept)).reshape(-1, 3)+1
            np.savetxt(out, faces, fmt='f %d %d %d')
        paths.append((name, path))
    return paths


def replay_plant(config, yard, bounds, directory):
    """A MuJoCo terrain plant over ``bounds`` with the yard's lava and rocks added as visuals.

    The DEM height field is the same triangulation the Gazebo collision mesh
    uses (``MarsYard``). The features are display-only geoms in one extra body
    appended after every other body, so the plant's body and geom ids keep
    their meanings. Used only to render logged states.
    """
    import mujoco
    from ..offroad import TerrainPlant
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    plant = TerrainPlant(copy.deepcopy(config), yard, directory, visual_bounds=bounds)
    xml = directory/'model_offroad.xml'
    root = ET.parse(xml).getroot()
    asset, world = root.find('asset'), root.find('worldbody')
    body = ET.SubElement(world, 'body', name='mars_features')
    colours = {'lava': '0.30 0.25 0.22 1', 'rocks_big': '0.42 0.33 0.27 1', 'rocks_medium': '0.48 0.38 0.30 1'}
    for name, path in _crop_features(bounds, directory):
        ET.SubElement(asset, 'mesh', name=f'{name}_crop', file=str(path))
        ET.SubElement(body, 'geom', name=f'{name}_visual', type='mesh', mesh=f'{name}_crop',
                      contype='0', conaffinity='0', rgba=colours.get(name, '0.4 0.35 0.3 1'))
    # Mars-toned ground in place of the grey-brown checker (still 0.1 m squares).
    for texture in asset.findall('texture'):
        if texture.get('name') == 'terrain_tex':
            texture.set('rgb1', '0.62 0.40 0.27')
            texture.set('rgb2', '0.68 0.45 0.31')
    path = directory/'model_replay.xml'
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='unicode', xml_declaration=True)
    model = mujoco.MjModel.from_xml_path(str(path))
    model.hfield_data[:] = plant.model.hfield_data
    if model.nbody != plant.model.nbody+1:
        raise RuntimeError('Replay model must only append the feature body')
    plant.model, plant.data = model, mujoco.MjData(model)
    plant.reset()
    return plant


def render_pair(job):
    """Render the side-by-side replay of one route's two runs."""
    os.environ.setdefault('MUJOCO_GL', 'egl')
    from ..offroad_view import render_replay
    name, output = job['route'], Path(job['output'])
    path, _ = build_route(name)
    yard = MarsYard()
    runs = []
    for mode in CONTROLLERS:
        with np.load(output/name/mode/'rollout.npz') as saved:
            log = {key: np.asarray(value, dtype=np.float64) if value.dtype.kind == 'f' else np.asarray(value)
                   for key, value in saved.items()}
        log['planned_xy'] = route_samples(path, 240)
        runs.append((LABELS[mode], log))
    points = np.vstack([route_samples(path, 400)]+[log['qpos'][:, :2] for _, log in runs])
    margin = 1.6
    bounds = (float(points[:, 0].min()-margin), float(points[:, 0].max()+margin),
              float(points[:, 1].min()-margin), float(points[:, 1].max()+margin))
    plant = replay_plant(load_config(), yard, bounds, output/'_replay_scenes'/name)
    duration = max(float(log['time_s'][-1]) for _, log in runs)
    speed = 1. if duration <= 45. else 2.
    started = time.time()
    video = render_replay(plant, runs, output/name/'replay.mp4', fps=20, speed=speed, width=640, height=480,
                          azimuth=112., elevation=-24., distance=1.7, title=name.replace('_', ' '))
    return {'route': name, 'replay': str(video), 'speed': speed, 'render_s': time.time()-started}


def write_report(output):
    output = Path(output)
    lines = ['# Original local LQR against the contact-aware controller on the ERC 2021 Mars yard (Gazebo)', '',
             'Both controllers start from the identical state on each route: heading 0, wheel phase 0.30, at rest in '
             'static balance. Physics: Gazebo Harmonic, DART, 1 ms step. Replays are rendered from the logged Gazebo '
             'states (MuJoCo renderer, same DEM triangulation, Leo lava and rock meshes).', '',
             '| Route | Controller | Finished | Passed all gates | Time s | Moving RMS / max / final mm | Max attitude error deg | Failure | Replay |',
             '| --- | --- | --- | --- | --- | --- | --- | --- | --- |']
    for name in COMPARED_ROUTES:
        for mode in CONTROLLERS:
            summary_path = output/name/mode/'summary.json'
            if not summary_path.is_file():
                continue
            s = json.loads(summary_path.read_text())
            t = s['tracking']
            video = output/name/'replay.mp4'
            lines.append(f"| {name} | {LABELS[mode]} | {'yes' if s['completed'] else 'no'} | "
                         f"{'yes' if s['mars']['passed'] else 'no'} | {s['completed_duration_s']:.1f} | "
                         f"{1e3*t['rms_error_while_moving_m']:.0f} / {1e3*t['max_error_m']:.0f} / {1e3*t['final_error_m']:.0f} | "
                         f"{s['attitude']['max_error_from_reference_deg']:.1f} | {s['failure'] or '-'} | "
                         + (f'[mp4]({name}/replay.mp4)' if video.is_file() else '-') + ' |')
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')


def run(output=DEFAULT_OUTPUT, routes=None, workers=5, render=True, render_workers=6):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    names = list(routes or COMPARED_ROUTES)
    jobs = [{'route': name, 'output': str(output), 'workspace': str(output/'_workspaces')} for name in names]
    pending = [job for job in jobs if not all((output/job['route']/mode/'summary.json').is_file() for mode in CONTROLLERS)]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for future in as_completed([pool.submit(run_pair, job) for job in pending]):
            result = future.result()
            print(result['route'], ' | '.join(f"{r['controller']}: finished={r['completed']} max {r['max_mm']:.0f} mm "
                                               f"({r['failure'] or 'ok'})" for r in result['runs']), flush=True)
            write_report(output)
    write_report(output)
    if render:
        with ProcessPoolExecutor(max_workers=render_workers) as pool:
            for future in as_completed([pool.submit(render_pair, job) for job in jobs]):
                result = future.result()
                print(f"replay {result['route']}: {result['replay']} ({result['speed']:g}x, {result['render_s']:.0f} s)", flush=True)
                write_report(output)
    (output/'results.json').write_text(json.dumps(jsonable({'routes': names}), indent=2)+'\n')
    return output
