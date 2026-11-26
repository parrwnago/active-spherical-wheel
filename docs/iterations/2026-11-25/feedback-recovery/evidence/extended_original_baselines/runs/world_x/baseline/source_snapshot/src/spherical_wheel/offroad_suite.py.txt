"""Batches of off-road runs: scenario suites, controller tuning and slope limits.

Runs are independent, so a batch is spread over worker processes. Each worker
owns a private model directory; results do not depend on the worker count.
"""
from __future__ import annotations

import copy
import multiprocessing
import os
from pathlib import Path
import shutil

import numpy as np

from .model import load_config, mass_properties
from .offroad_control import OffroadParams
from .offroad_experiments import (DEFAULT_WORKSPACE, SCENARIOS, ReferencePath, Workspace, run_offroad, save_run)
from .terrain import Terrain, make_terrain

_WORKER = {}


def _start_worker(config, directory, seed_cache, counter):
    with counter.get_lock():
        slot = counter.value
        counter.value += 1
    _WORKER['workspace'] = Workspace(config, Path(directory)/f'worker_{slot:02d}', seed_cache, plant_limit=8)


def run_job(workspace, job):
    """Run one job dictionary and return its summary, with ``label`` attached.

    Keys: ``terrain`` and ``path`` (their ``spec()`` dictionaries), ``params``
    (``OffroadParams`` fields), and optionally ``pushes``, ``phase``,
    ``balanced_lean``, ``settle_s``, ``duration_s``, ``label``, ``title`` and
    ``save`` (a directory for the rollout, summary and figure).
    """
    terrain, path = Terrain.from_spec(job['terrain']), ReferencePath.from_spec(job['path'])
    params = OffroadParams(**job.get('params', {}))
    log, summary = run_offroad(workspace, terrain, path, params, job.get('duration_s'), job.get('phase'),
                               job.get('pushes', ()), job.get('balanced_lean', True), job.get('settle_s', 3.))
    summary['label'] = job.get('label')
    if job.get('save'):
        from .offroad_analysis import plot_run
        directory = save_run(job['save'], log, summary, terrain)
        plot_run(log, summary, terrain, path, directory/'plots.png', job.get('title'))
    return summary


def _worker_job(job):
    return run_job(_WORKER['workspace'], job)


class Pool:
    """Worker processes kept alive across several batches.

    Use as a context manager. ``run(jobs)`` returns summaries in job order.
    With one worker everything runs in this process.
    """

    def __init__(self, workers=None, directory=None, config=None):
        self.config = copy.deepcopy(config if isinstance(config, dict) else load_config(config))
        self.directory = Path(directory or DEFAULT_WORKSPACE).resolve()
        self.workers = max(1, workers or max(1, (os.cpu_count() or 2)-2))
        self._workspace = self._executor = None

    def __enter__(self):
        self._workspace = Workspace(self.config, self.directory)
        if self.workers > 1:
            from concurrent.futures import ProcessPoolExecutor
            counter = multiprocessing.Value('i', 0)
            # Workers build models under a directory of this pool's own, so two
            # batches started at once never write the same files.
            self._scratch = self.directory/f'pool_{os.getpid()}'
            self._executor = ProcessPoolExecutor(
                self.workers, initializer=_start_worker,
                initargs=(self.config, self._scratch, self._workspace.cache_path, counter))
        return self

    def __exit__(self, *exception):
        if self._executor is not None:
            self._executor.shutdown()
            shutil.rmtree(self._scratch, ignore_errors=True)
        self._executor = self._workspace = None

    def run(self, jobs):
        jobs = list(jobs)
        if self._executor is None or len(jobs) == 1:
            return [run_job(self._workspace, job) for job in jobs]
        return list(self._executor.map(_worker_job, jobs))


def run_jobs(jobs, workers=None, directory=None, config=None, pool=None):
    """Run jobs and return their summaries in order; ``pool`` reuses live workers."""
    jobs = list(jobs)
    if pool is not None:
        return pool.run(jobs)
    with Pool(min(len(jobs), workers) if workers else min(len(jobs), max(1, (os.cpu_count() or 2)-2)),
              directory, config) as temporary:
        return temporary.run(jobs)


def scenario_job(name, params=None, seed=7, label=None, phase=None, distance=None, duration=None, **extra):
    """Job for a named scenario, optionally shortened or started on another part of the wheel."""
    item = SCENARIOS[name]
    kind, options = item['terrain']
    terrain = make_terrain(kind, heading_deg=item['course'], seed=seed, **options)
    path = ReferencePath(distance or item['distance'], item['travel'], duration or item['duration'])
    job = {'terrain': terrain.spec(), 'path': path.spec(), 'params': dict(params or {}),
           'label': label if label is not None else name, **extra}
    if phase is not None:
        job['phase'] = float(phase)
    return job


# metric: (how it is read from a summary, the value that costs 1)
COST_TERMS = {
    'rms_tracking_error_m': (lambda s: s['tracking']['rms_error_m'], .03),
    'max_tracking_error_m': (lambda s: s['tracking']['max_error_m'], .10),
    'final_error_m': (lambda s: s['tracking']['final_error_m'], .02),
    'max_attitude_error_deg': (lambda s: s['attitude']['max_error_from_reference_deg'], 4.),
    'impact_load_above_weight': (lambda s: max(0., s['contact']['normal_load_fraction_max']-1.), 2.),
    'sliding_distance_m': (lambda s: s['contact']['slip_distance_m'], .05),
}
FAILURE_COST = 30.


def run_cost(summary):
    """Scalar cost of one run: 1 per term at its reference value, plus 30 for a fall.

    A fall keeps the terms it accumulated, so an early fall is not cheaper
    than a late one. Each term is capped at 10. The reference values in
    ``COST_TERMS`` decide what the tuning trades against what; they are a
    stated choice, not a measurement.
    """
    cost = sum(min(getter(summary)/scale, 10.) for getter, scale in COST_TERMS.values())
    return float(cost+(0. if summary['completed'] else FAILURE_COST))


TUNING_SCENARIOS = ('ramp_forward', 'ramp_lateral', 'cross_slope', 'hills', 'washboard', 'bumps', 'rough')
# Each scenario is run from two starts: another random-terrain seed and another
# part of the wheel on the ground, so surface handovers fall at different places.
TUNING_VARIANTS = ((7, .30), (23, .90))
# Shortened travel for tuning; the reported suite uses the full scenarios.
TUNING_DISTANCE_M, TUNING_DURATION_S = 1.1, 12.
TUNING_GRID = (
    ('normal_time_constant_s', (.02, .04, .1, .25)),
    ('rim_aware', (False, True)),
    ('rim_time_constant_s', (.1, .2, .5)),
    ('lean_gain', (0., .5, .9, 1., 1.1)),
    ('support_feedforward', (0., .5, 1.)),
    ('shaft_error', ('material', 'free', 'rolling')),
    ('schedule_on_ground_normal', (False, True)),
    ('unload_fraction', (0., .3)),
    ('synchronize', (False, True)),
    ('track_contact_point', (False, True)),
    ('heading_gain', (0., .2)),
    ('normal_source', ('intrinsic', 'solver')),
)


def evaluate(parameter_sets, scenarios=TUNING_SCENARIOS, variants=TUNING_VARIANTS, shortened=True,
             workers=None, directory=None, config=None, pool=None):
    """Mean suite cost for each parameter set; returns one record per set."""
    parameter_sets = list(parameter_sets)
    cases = [(name, seed, phase) for name in scenarios for seed, phase in variants]
    travel = {'distance': TUNING_DISTANCE_M, 'duration': TUNING_DURATION_S} if shortened else {}
    jobs = [scenario_job(name, params, seed, label=index, phase=phase, **travel)
            for index, params in enumerate(parameter_sets) for name, seed, phase in cases]
    summaries = run_jobs(jobs, workers, directory, config, pool)
    records = []
    for index, params in enumerate(parameter_sets):
        chunk = summaries[index*len(cases):(index+1)*len(cases)]
        costs = [run_cost(summary) for summary in chunk]
        records.append({'params': dict(params), 'cost': float(np.mean(costs)),
                        'cost_standard_error': float(np.std(costs, ddof=1)/np.sqrt(len(costs))) if len(costs) > 1 else 0.,
                        'failures': int(sum(not summary['completed'] for summary in chunk)),
                        'runs': [{'scenario': name, 'seed': seed, 'phase_rad': phase, 'cost': cost,
                                  'completed': summary['completed'],
                                  'max_error_m': summary['tracking']['max_error_m'],
                                  'final_error_m': summary['tracking']['final_error_m'],
                                  'max_attitude_error_deg': summary['attitude']['max_error_from_reference_deg'],
                                  'max_load_fraction': summary['contact']['normal_load_fraction_max']}
                                 for (name, seed, phase), summary, cost in zip(cases, chunk, costs)]})
    return records


def tune(start=None, grid=TUNING_GRID, margin=.05, workers=None, directory=None, config=None, progress=print):
    """One-factor-at-a-time sweep around the starting settings, then a check.

    Every setting in ``grid`` is varied alone with the others held at the
    start, and each candidate is run on the tuning suite. A value replaces the
    incumbent only if it lowers the mean cost by more than ``margin`` with no
    additional falls: the suite's outcome is sensitive to small changes, so
    smaller differences are not treated as real. The combined choice is then
    run once more against the start, and kept only if it is better.

    Returns ``(chosen parameters, report)``. The report holds every evaluation,
    so the same data shows what each feature contributes when it is removed.
    """
    incumbent = OffroadParams(**(start or {})).as_dict()
    candidates, index = [incumbent], {}
    for name, values in grid:
        for value in values:
            if value == incumbent[name]:
                index[name, value] = 0
            else:
                index[name, value] = len(candidates)
                candidates.append(dict(incumbent, **{name: value}))
    with Pool(workers, directory, config) as pool:
        records = evaluate(candidates+[{'mode': 'flat_baseline'}], pool=pool)
        baseline = records.pop()
        base = records[0]
        sweeps, chosen = [], dict(incumbent)
        for name, values in grid:
            costs = [records[index[name, value]]['cost'] for value in values]
            failures = [records[index[name, value]]['failures'] for value in values]
            best = min(range(len(values)), key=lambda i: (failures[i], costs[i]))
            better = (failures[best] <= base['failures'] and costs[best] < base['cost']*(1.-margin))
            pick = values[best] if better else incumbent[name]
            chosen[name] = pick
            sweeps.append({'parameter': name, 'values': list(values), 'costs': costs, 'failures': failures,
                           'standard_errors': [records[index[name, value]]['cost_standard_error'] for value in values],
                           'start': incumbent[name], 'chosen': pick})
            progress(f'{name}: '+', '.join(f'{value}={cost:.2f}'+(f' ({fell} fell)' if fell else '')
                                           for value, cost, fell in zip(values, costs, failures))+f' -> {pick}')
        confirmation = None
        if chosen != incumbent:
            combined = evaluate([chosen], pool=pool)[0]
            confirmation = {'start_cost': base['cost'], 'start_failures': base['failures'],
                            'combined_cost': combined['cost'], 'combined_failures': combined['failures'],
                            'combined_record': combined}
            kept = combined['failures'] <= base['failures'] and combined['cost'] < base['cost']
            confirmation['kept'] = bool(kept)
            progress(f"combined choice: {combined['cost']:.2f} against start {base['cost']:.2f}"
                     f" -> {'kept' if kept else 'rejected, start retained'}")
            if not kept:
                chosen = dict(incumbent)
    return chosen, {'start': incumbent, 'chosen': chosen, 'margin': margin, 'sweeps': sweeps,
                    'confirmation': confirmation, 'flat_baseline': baseline, 'records': records}


def slope_envelope(grades_deg=(0., 4., 8., 12., 16., 20.), phases=(.30, np.pi/2), hold_s=6.,
                   push_N=.5, params=None, workers=None, directory=None, config=None):
    """Hold position on uniform slopes of increasing steepness, with a small push.

    Each case starts balanced on the slope, receives ``push_N`` for 0.1 s at
    1 s along world x, and is run to ``hold_s``. ``direction`` is the slope's
    direction relative to the fork: ``along`` loads the common shaft and
    ``across`` loads a shell or belt. Both controllers run every case from the
    same starting state.
    """
    jobs, cases = [], []
    for direction in ('along', 'across'):
        for phase in phases:
            for grade in grades_deg:
                options = ({'grade_deg': grade, 'cross_deg': 0.} if direction == 'along'
                           else {'grade_deg': 0., 'cross_deg': grade})
                terrain = make_terrain('slope', **options)
                path = ReferencePath(0., 0., hold_s-.5, start_s=.5, profile='quintic')
                for mode in ('flat_baseline', 'terrain_aware'):
                    settings = dict(params or {}, mode=mode) if mode == 'terrain_aware' else {'mode': mode}
                    cases.append({'direction': direction, 'phase_rad': float(phase), 'grade_deg': float(grade),
                                  'controller': mode})
                    jobs.append({'terrain': terrain.spec(), 'path': path.spec(), 'params': settings,
                                 'phase': float(phase), 'pushes': [(1., .1, [push_N, 0., 0.])],
                                 'settle_s': 0., 'duration_s': hold_s})
    for case, summary in zip(cases, run_jobs(jobs, workers, directory, config)):
        case.update(completed=summary['completed'], failure=summary['failure'], settled=summary['settled_at_end'],
                    final_error_m=summary['tracking']['final_error_m'], max_error_m=summary['tracking']['max_error_m'],
                    max_attitude_error_deg=summary['attitude']['max_error_from_reference_deg'],
                    utilization_p95=summary['contact']['utilization_p95'],
                    max_input_fraction=summary['actuation']['max_input_fraction'],
                    rms_torque_Nm=summary['actuation']['rms_torque_Nm'])
    return cases


def slope_limits(config=None):
    """Closed-form static figures of the declared model on a uniform slope.

    The friction and torque entries bound what any controller can hold at
    rest. The belt-cap entry is different: it is the side slope at which the
    polar belt stops covering the pole, not a holding limit. The envelope runs
    hold position beyond it, with larger error. None of these say anything
    about dynamics, and all inherit the assumed masses, friction and bounds.
    """
    config = copy.deepcopy(config if isinstance(config, dict) else load_config(config))
    geometry, mu = config['geometry'], float(config['contact']['friction'][0])
    properties = mass_properties(config)
    radius = geometry['sphere_radius_m']
    weight = properties['total_mass_kg']*abs(config['simulation']['gravity_m_s2'][2])
    upper = properties['fork']
    lean_factor = properties['total_mass_kg']*radius/(upper['mass_kg']*upper['com_m'][2])
    shaft, shell = config['actuation']['torque_limits_Nm'][:2]
    phase = config['simulation'].get('initial_shaft_phase_rad', .30)
    return {
        'weight_N': float(weight),
        'lean_per_unit_slope': float(lean_factor),
        'friction_limit_deg': float(np.degrees(np.arctan(mu))),
        'shaft_torque_limit_deg': float(np.degrees(np.arcsin(min(1., shaft/(weight*radius))))),
        'shell_torque_limit_deg_at_initial_phase':
            float(np.degrees(np.arcsin(min(1., shell/(weight*radius*abs(np.cos(phase))))))),
        'belt_cap_side_slope_limit_deg': float(geometry['polar_cap_angle_deg']/(1.+lean_factor)),
        'notes': {
            'lean_per_unit_slope': 'Balancing lean from vertical divided by slope angle, for small angles.',
            'friction_limit_deg': 'Slope whose tangent equals the nominal friction coefficient.',
            'shaft_torque_limit_deg': 'Grade along the shaft direction at which static holding torque equals the shaft bound.',
            'shell_torque_limit_deg_at_initial_phase': 'Side slope at which static holding torque equals the shell bound, at the initial shaft phase.',
            'belt_cap_side_slope_limit_deg': 'Side slope at which slope plus balancing lean carries the support point off the polar belt cap when a pole is down. Not a holding limit: beyond it the sideways drive depends on the support staying off the exact pole line.',
        },
    }
