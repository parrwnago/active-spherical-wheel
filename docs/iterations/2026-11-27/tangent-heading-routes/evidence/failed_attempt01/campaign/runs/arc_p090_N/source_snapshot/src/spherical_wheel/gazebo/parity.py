"""Gazebo against MuJoCo: the same controller and gains on the same scenarios.

``python -m spherical_wheel.gazebo parity`` runs every off-road scenario
(``offroad_experiments.SCENARIOS``) in Gazebo with both controllers, from the
same starting state as the saved MuJoCo suite (first start, phase 0.30), and
compares each run with ``results/offroad/suite/<scenario>/<controller>``.

The controller code, the gain library and the runner are identical. What
differs is the simulator: DART with rigid, box-friction contacts against a
triangulated terrain, where MuJoCo has soft elliptic-cone contacts against the
exact tangent plane. Matching numbers therefore show the port is faithful; the
remaining differences are the physics engines'.
"""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import os
from pathlib import Path
import time

import numpy as np

from ..control import jsonable
from ..model import REPO_ROOT
from ..offroad_control import OffroadParams
from ..offroad_experiments import SCENARIOS, save_run, scenario

CONTROLLERS = ('terrain_aware', 'flat_baseline')
LABELS = {'terrain_aware': 'contact-aware', 'flat_baseline': 'flat baseline',
          'momentum_observer': 'momentum observer (no force sensing)'}
MUJOCO_SUITE = REPO_ROOT/'results'/'offroad'/'suite'
SEED_CACHE = REPO_ROOT/'generated'/'offroad'/'flat'/'control_schedule.npz'

_WORKSPACE = None


def worker_workspace(root):
    """One Gazebo workspace per process; the bridge plugin is one per process."""
    global _WORKSPACE
    if _WORKSPACE is None:
        from .experiments import GazeboWorkspace
        os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
        _WORKSPACE = GazeboWorkspace(directory=Path(root)/f'worker_{os.getpid()}',
                                     seed_cache=SEED_CACHE if SEED_CACHE.is_file() else None)
    return _WORKSPACE


def metrics(summary):
    """The figures compared between simulators."""
    t, a, c = summary['tracking'], summary['attitude'], summary['contact']
    return {'completed': summary['completed'], 'failure': summary['failure'],
            'duration_s': summary['completed_duration_s'],
            'rms_mm': 1e3*t['rms_error_m'], 'max_mm': 1e3*t['max_error_m'], 'final_mm': 1e3*t['final_error_m'],
            'attitude_max_deg': a['max_error_from_reference_deg'], 'heading_change_deg': a['heading_change_deg'],
            'load_max': c['normal_load_fraction_max'], 'load_min': c['normal_load_fraction_min'],
            'friction_use_p95': c['utilization_p95'], 'slip_mm': 1e3*c['slip_distance_m'],
            'normal_estimate_rms_deg': c['normal_estimate_error_rms_deg'],
            'support_share': c['support_time_share'], 'handovers': c['support_handovers'],
            'saturated_fraction': summary['actuation']['saturated_time_fraction']}


def run_case(job):
    """Run one scenario/controller in Gazebo in this worker; return metrics."""
    from ..offroad_analysis import plot_run
    from .experiments import run_gazebo
    name, mode, root = job['scenario'], job['controller'], Path(job['output'])
    workspace = worker_workspace(job['workspace'])
    terrain, path = scenario(name)
    start = time.time()
    log, summary = run_gazebo(workspace, terrain, path, OffroadParams(mode))
    directory = save_run(root/name/mode, log, summary, terrain)
    try:
        plot_run(log, summary, terrain, path, directory/'plots.png',
                 title=f'Gazebo: {name}, {LABELS[mode]} controller')
    except Exception as error:  # a figure must not lose a finished run
        (directory/'plot_error.txt').write_text(repr(error))
    reference = MUJOCO_SUITE/name/mode/'summary.json'
    mujoco = metrics(json.loads(reference.read_text())) if reference.is_file() else None
    return {'scenario': name, 'controller': mode, 'gazebo': metrics(summary), 'mujoco': mujoco,
            'wall_s': time.time()-start, 'bridge_counters': summary.get('bridge_counters')}


def _mujoco_case(job):
    """The same scenario in the MuJoCo plant on this machine (reference for the report)."""
    from ..offroad_experiments import Workspace, run_offroad
    global _MUJOCO_WORKSPACE
    if _MUJOCO_WORKSPACE is None:
        os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
        _MUJOCO_WORKSPACE = Workspace(directory=Path(job['workspace'])/f'mujoco_{os.getpid()}',
                                      seed_cache=SEED_CACHE if SEED_CACHE.is_file() else None)
    terrain, path = scenario(job['scenario'])
    log, summary = run_offroad(_MUJOCO_WORKSPACE, terrain, path, OffroadParams(job['controller']))
    save_run(Path(job['output'])/job['scenario']/job['controller']/'mujoco', log, summary, terrain)
    return {'scenario': job['scenario'], 'controller': job['controller'], 'mujoco': metrics(summary)}


_MUJOCO_WORKSPACE = None


def run_mujoco_reference(output, scenarios=None, controllers=CONTROLLERS, workers=4):
    """Run the MuJoCo suite here, so the comparison shares machine and libraries."""
    output = Path(output).resolve()
    jobs = [{'scenario': name, 'controller': mode, 'output': str(output), 'workspace': str(output/'_workspaces')}
            for name in (scenarios or SCENARIOS) for mode in controllers]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        for future in as_completed([pool.submit(_mujoco_case, job) for job in jobs]):
            result = future.result()
            print(f"mujoco {result['scenario']:>13} {result['controller']:>13}: max {result['mujoco']['max_mm']:.1f} mm", flush=True)
    results = json.loads((output/'results.json').read_text()) if (output/'results.json').is_file() else []
    write_report(results, output)


def run_suite(output, scenarios=None, controllers=CONTROLLERS, workers=4):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    jobs = [{'scenario': name, 'controller': mode, 'output': str(output), 'workspace': str(output/'_workspaces')}
            for name in (scenarios or SCENARIOS) for mode in controllers]
    results = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(run_case, job): job for job in jobs}
        for future in as_completed(futures):
            job = futures[future]
            try:
                result = future.result()
            except Exception as error:
                result = {'scenario': job['scenario'], 'controller': job['controller'], 'error': repr(error)}
            results.append(result)
            g = result.get('gazebo') or {}
            print(f"{job['scenario']:>13} {job['controller']:>13}: "
                  + (f"error {result['error']}" if 'error' in result else
                     f"completed={g['completed']} max {g['max_mm']:.1f} mm final {g['final_mm']:.1f} mm "
                     f"({result['wall_s']:.0f} s wall)"), flush=True)
            (output/'results.json').write_text(json.dumps(jsonable(results), indent=2)+'\n')
    order = {name: i for i, name in enumerate(SCENARIOS)}
    results.sort(key=lambda r: (order.get(r['scenario'], 99), list(LABELS).index(r['controller'])))
    (output/'results.json').write_text(json.dumps(jsonable(results), indent=2)+'\n')
    write_report(results, output)
    return results


def _fmt(value, digits=1):
    if value is None or (isinstance(value, float) and not np.isfinite(value)):
        return 'n/a'
    return f'{value:.{digits}f}'


def write_report(results, output):
    output = Path(output)
    # Prefer MuJoCo references run on this machine when they exist.
    for r in results:
        local = output/r['scenario']/r['controller']/'mujoco'/'summary.json'
        if local.is_file():
            r['mujoco'] = metrics(json.loads(local.read_text()))
            r['mujoco_source'] = 'this machine'
        elif r.get('mujoco'):
            r['mujoco_source'] = 'saved results/offroad/suite'
    sources = sorted({r.get('mujoco_source', 'n/a') for r in results})
    lines = ['# Gazebo against MuJoCo: off-road scenario suite', '',
             'Same controller code, gains, runner and starting state (phase 0.30). MuJoCo reference: '
             + ', '.join(sources) + '. Gazebo: gz-sim 8.10, DART 6.19, ODE collision, triangulated terrain.', '',
             '| Scenario | Controller | Finished (Gz / MJ) | Max error mm (Gz / MJ) | Final error mm (Gz / MJ) | '
             'Max attitude error deg (Gz / MJ) | Peak load x weight (Gz / MJ) | Heading change deg (Gz / MJ) |',
             '| --- | --- | --- | --- | --- | --- | --- | --- |']
    for r in results:
        if 'error' in r:
            lines.append(f"| {r['scenario']} | {LABELS[r['controller']]} | run error: {r['error']} | | | | | |")
            continue
        g, m = r['gazebo'], r['mujoco'] or {}
        lines.append(f"| {r['scenario']} | {LABELS[r['controller']]} | {'yes' if g['completed'] else 'no'} / "
                     f"{'yes' if m.get('completed') else 'no' if m else 'n/a'} | {_fmt(g['max_mm'])} / {_fmt(m.get('max_mm'))} | "
                     f"{_fmt(g['final_mm'])} / {_fmt(m.get('final_mm'))} | {_fmt(g['attitude_max_deg'], 2)} / "
                     f"{_fmt(m.get('attitude_max_deg'), 2)} | {_fmt(g['load_max'], 2)} / {_fmt(m.get('load_max'), 2)} | "
                     f"{_fmt(g['heading_change_deg'])} / {_fmt(m.get('heading_change_deg'))} |")
    (output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    plot_comparison(results, output/'comparison.png')


def plot_comparison(results, path):
    from ..offroad_analysis import _pyplot
    plt = _pyplot()
    rows = [r for r in results if 'error' not in r and r.get('mujoco')]
    if not rows:
        return
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.2), constrained_layout=True)
    for axis, key, label in ((axes[0], 'max_mm', 'Largest tracking error (mm)'),
                             (axes[1], 'final_mm', 'Final error (mm)'),
                             (axes[2], 'attitude_max_deg', 'Largest attitude error (deg)')):
        for mode, colour in (('terrain_aware', '#1b6ca8'), ('momentum_observer', '#1baf7a'),
                             ('flat_baseline', '#c8553d')):
            subset = [r for r in rows if r['controller'] == mode]
            x = [max(r['mujoco'][key], 1e-2) for r in subset]
            y = [max(r['gazebo'][key], 1e-2) for r in subset]
            done = [r['gazebo']['completed'] for r in subset]
            axis.scatter([a for a, d in zip(x, done) if d], [b for b, d in zip(y, done) if d], color=colour,
                         label=f'{LABELS[mode]}', s=36)
            axis.scatter([a for a, d in zip(x, done) if not d], [b for b, d in zip(y, done) if not d], color=colour,
                         marker='x', s=50, label=f'{LABELS[mode]}: Gazebo run did not finish')
            for r, a, b in zip(subset, x, y):
                axis.annotate(r['scenario'], (a, b), fontsize=6.5, alpha=.7, xytext=(3, 2), textcoords='offset points')
        low = min(min(max(r['mujoco'][key], 1e-2), max(r['gazebo'][key], 1e-2)) for r in rows)
        high = max(max(r['mujoco'][key], r['gazebo'][key]) for r in rows)
        axis.plot([low, high], [low, high], color='0.6', lw=1, ls='--', label='equal')
        axis.set(xscale='log', yscale='log', xlabel=f'MuJoCo: {label}', ylabel=f'Gazebo: {label}')
        axis.grid(alpha=.25, which='both')
    axes[0].legend(fontsize=7, loc='upper left')
    fig.suptitle('Same controller, same gains, same scenarios: Gazebo (DART) against MuJoCo')
    fig.savefig(path, dpi=140)
    plt.close(fig)
