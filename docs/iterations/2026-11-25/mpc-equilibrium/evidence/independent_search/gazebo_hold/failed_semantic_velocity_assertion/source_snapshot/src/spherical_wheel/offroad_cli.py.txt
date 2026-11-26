"""Command-line entry points for the off-road study.

``python -m spherical_wheel offroad <action>`` where action is one of
``limits``, ``run``, ``replay``, ``suite``, ``envelope``, ``holdout`` or
``tune``. Every batch action writes its numbers as JSON before any figure.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from .control import jsonable
from .model import load_config
from .offroad_control import OffroadParams
from .offroad_experiments import SCENARIOS, ReferencePath, Workspace, run_offroad, save_run, scenario
from .offroad_suite import (COST_TERMS, FAILURE_COST, TUNING_DISTANCE_M, TUNING_DURATION_S, TUNING_GRID,
                            TUNING_SCENARIOS, TUNING_VARIANTS, run_cost, run_jobs, scenario_job, slope_envelope,
                            slope_limits, tune)
from .terrain import make_terrain

MODES = ('flat_baseline', 'terrain_aware')

# One setting changed at a time from the tuning's starting point. The first
# group removes a use of the ground reaction; the second adds an option.
REMOVALS = (
    ('No balancing lean from the sensed slope', 'lean_gain', 0.),
    ('No static holding torque', 'support_feedforward', 0.),
    ('Shaft angle held to its flat-floor bookkeeping', 'shaft_error', 'material'),
    ('Rim contact read like the spherical surface', 'rim_aware', False),
    ('No torque fade while unloaded', 'unload_fraction', 0.),
    ('No incoming-drive preparation', 'synchronize', False),
    ('Normal read from the solver, not force and moment', 'normal_source', 'solver'),
    ('Gains scheduled against the sensed ground', 'schedule_on_ground_normal', True),
    ('Reference applied to the contact point', 'track_contact_point', True),
    ('Heading held with the idle shell', 'heading_gain', .2),
)

# Terrains and headings the tuning never saw.
HOLDOUT = (
    ('rough, new seed', 'rough', {'seed': 41}),
    ('bumps, new seed', 'bumps', {'seed': 41}),
    ('hills, diagonal travel', 'hills', {'course': 35., 'travel': 35.}),
    ('washboard, shaft-driven', 'washboard', {'course': 0., 'travel': 0.}),
    ('ramp 12 deg, shell-driven', 'ramp', {'course': 90., 'travel': 90., 'options': {'grade_deg': 12.},
                                         'distance': 1.0, 'duration': 12.}),
    ('course, new seed, reversed', 'course', {'seed': 41, 'course': 180., 'travel': 180.}),
)


def _write(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(jsonable(payload), indent=2)+'\n')
    return path


def _brief(summary):
    return {'completed': summary['completed'], 'failure': summary['failure'],
            'final_error_mm': round(summary['tracking']['final_error_m']*1000, 2),
            'max_error_mm': round(summary['tracking']['max_error_m']*1000, 2),
            'rms_error_mm': round(summary['tracking']['rms_error_m']*1000, 2),
            'max_attitude_error_deg': round(summary['attitude']['max_error_from_reference_deg'], 3),
            'heading_change_deg': round(summary['attitude']['heading_change_deg'], 1),
            'max_slope_under_wheel_deg': round(summary['ground']['max_slope_under_wheel_deg'], 2),
            'normal_load_range_x_weight': [round(summary['contact']['normal_load_fraction_min'], 2),
                                           round(summary['contact']['normal_load_fraction_max'], 2)],
            'friction_use_p95': round(summary['contact']['utilization_p95'], 3),
            'sliding_distance_mm': round(summary['contact']['slip_distance_m']*1000, 2),
            'normal_estimate_error_rms_deg': round(summary['contact']['normal_estimate_error_rms_deg'], 3),
            'saturated_time_fraction': round(summary['actuation']['saturated_time_fraction'], 4),
            'support_handovers': summary['contact']['support_handovers'],
            'cost': round(run_cost(summary), 3)}


def _cost_definition():
    return {'terms_and_the_value_that_costs_1': {key: scale for key, (_, scale) in COST_TERMS.items()},
            'failure_cost': FAILURE_COST}


def _holdout_job(label, kind, changes, params):
    item = dict(SCENARIOS[kind if kind in SCENARIOS else 'ramp_forward'])
    options = dict(item['terrain'][1] if kind in SCENARIOS else {})
    options.update(changes.get('options', {}))
    course, travel = changes.get('course', item['course']), changes.get('travel', item['travel'])
    terrain = make_terrain(kind, heading_deg=course, seed=changes.get('seed', 7), **options)
    path = ReferencePath(changes.get('distance', item['distance']), travel, changes.get('duration', item['duration']))
    return {'terrain': terrain.spec(), 'path': path.spec(), 'params': dict(params), 'label': label}


# Every scenario is run from several places on the wheel, so surface handovers
# fall at different points of the terrain. A single run's worst-case error is
# sensitive to such details; the spread over starts is the honest figure.
START_PHASES = (.30, .90, 2.20, -1.00)


def _grouped(jobs, labels, workers, config):
    """Run jobs labelled ``[case, controller, start]`` into ``{case: {controller: [summaries]}}``."""
    results = {label: {mode: [None]*len(START_PHASES) for mode in MODES} for label in labels}
    for summary in run_jobs(jobs, workers, config=config):
        case, mode, start = summary['label']
        results[case][mode][start] = summary
    return results


def _spread(summaries):
    """Median and worst case of the headline metrics over one case's starts."""
    rows = [_brief(summary) for summary in summaries]

    def both(values):
        return {'median': round(float(np.median(values)), 3), 'worst': round(float(np.max(values)), 3)}

    return {'completed': f"{sum(row['completed'] for row in rows)}/{len(rows)}",
            'failures': [row['failure'] for row in rows if not row['completed']],
            'final_error_mm': both([row['final_error_mm'] for row in rows]),
            'max_error_mm': both([row['max_error_mm'] for row in rows]),
            'rms_error_mm': both([row['rms_error_mm'] for row in rows]),
            'max_attitude_error_deg': both([row['max_attitude_error_deg'] for row in rows]),
            'friction_use_p95': both([row['friction_use_p95'] for row in rows]),
            'sliding_distance_mm': both([row['sliding_distance_mm'] for row in rows]),
            'largest_normal_load_x_weight': both([row['normal_load_range_x_weight'][1] for row in rows]),
            'heading_change_deg_abs': both([abs(row['heading_change_deg']) for row in rows]),
            'mean_cost': round(float(np.mean([row['cost'] for row in rows])), 3),
            'runs': [dict(row, start_phase_rad=phase) for row, phase in zip(rows, START_PHASES)]}


def _overall(table):
    def count(mode, part):
        return sum(int(case[mode]['completed'].split('/')[part]) for case in table.values())

    return {mode: {'completed': f'{count(mode, 0)}/{count(mode, 1)}',
                   'mean_cost': round(float(np.mean([case[mode]['mean_cost'] for case in table.values()])), 3)}
            for mode in MODES}


def _headline(table):
    return {'overall': _overall(table),
            'cases': {label: {mode: {key: case[mode][key] for key in
                                     ('completed', 'final_error_mm', 'max_error_mm', 'mean_cost')}
                              for mode in MODES} for label, case in table.items()}}


def main(args):
    output, workers = Path(args.output), args.workers
    config = load_config(args.config)
    if args.action == 'limits':
        print(json.dumps(slope_limits(config), indent=2))
        return
    if args.action == 'run':
        terrain, path = scenario(args.scenario, args.seed)
        workspace = Workspace(config)
        log, summary = run_offroad(workspace, terrain, path, OffroadParams(mode=args.controller))
        directory = save_run(output/'runs'/f'{args.scenario}_{args.controller}', log, summary, terrain)
        from .offroad_analysis import plot_run
        plot_run(log, summary, terrain, path, directory/'plots.png')
        view = None
        try:
            from .offroad_view import render_view
            moment = args.view_time if args.view_time is not None else .5*float(log['time_s'][-1])
            view = str(render_view(workspace.plant(terrain), log, moment, directory/'view.png'))
        except Exception as error:  # no display or no GL context: the run itself is unaffected
            view = f'not rendered: {error}'
        print(json.dumps({'directory': str(directory), 'view': view, **_brief(summary)}, indent=2))
        return
    if args.action == 'replay':
        terrain, path = scenario(args.scenario, args.seed)
        workspace = Workspace(config)
        runs, table = [], {}
        for label, mode in (('Flat-floor baseline', 'flat_baseline'), ('Contact-aware controller', 'terrain_aware')):
            log, summary = run_offroad(workspace, terrain, path, OffroadParams(mode=mode))
            runs.append((label, log))
            table[mode] = _brief(summary)
        from .offroad_view import render_replay
        target = render_replay(workspace.plant(terrain), runs, output/'replay'/f'{args.scenario}.mp4',
                               speed=args.speed, title=args.scenario.replace('_', ' '))
        print(json.dumps({'replay': str(target), **table}, indent=2))
        return
    if args.action == 'suite':
        names = list(SCENARIOS)
        jobs = [scenario_job(name, {'mode': mode}, args.seed, label=[name, mode, start], phase=phase,
                             **({'save': str(output/'suite'/name/mode)} if start == 0 else {}))
                for name in names for mode in MODES for start, phase in enumerate(START_PHASES)]
        results = _grouped(jobs, names, workers, config)
        table = {name: {mode: _spread(results[name][mode]) for mode in MODES} for name in names}
        # Numbers are written before any figure, so a drawing error cannot lose a batch.
        _write(output/'suite'/'comparison.json', {
            'scope': 'Simulated provisional model. Each scenario is run with both controllers from the same '
                     'starting states, once for each start phase.',
            'start_phases_rad': START_PHASES,
            'scenarios': {name: {'terrain': SCENARIOS[name]['terrain'][0],
                                 'terrain_options': SCENARIOS[name]['terrain'][1],
                                 'course_heading_deg': SCENARIOS[name]['course'],
                                 'travel_heading_deg': SCENARIOS[name]['travel'],
                                 'distance_m': SCENARIOS[name]['distance'], 'duration_s': SCENARIOS[name]['duration'],
                                 'ground': results[name]['terrain_aware'][0]['ground']} for name in names},
            'cost': _cost_definition(), 'contact_aware_parameters': OffroadParams().as_dict(),
            'overall': _overall(table), 'results': table})
        from .offroad_analysis import plot_comparison
        from .offroad_geometry import plot_support_map
        plot_comparison(results, output/'suite'/'comparison.png')
        logs = {}
        for label, name in (('Mixed course', 'course'), ('Side slope', 'cross_slope')):
            with np.load(output/'suite'/name/'terrain_aware'/'rollout.npz') as saved:
                logs[label] = {key: saved[key] for key in ('support_direction', 'normal_force_N')}
        plot_support_map(config, output/'suite'/'support_map.png', logs)
        print(json.dumps(_headline(table), indent=1))
        return
    if args.action == 'envelope':
        cases = slope_envelope(workers=workers, config=config)
        limits = slope_limits(config)
        _write(output/'envelope'/'envelope.json', {
            'scope': 'Hold on a uniform slope for 6 s with a 0.5 N, 0.1 s push at 1 s. Simulated provisional model.',
            'static_limits': limits, 'cases': cases})
        from .offroad_geometry import plot_envelope
        plot_envelope(cases, limits, output/'envelope'/'envelope.png')
        for case in cases:
            print(f"{case['direction']:6s} phase={case['phase_rad']:.2f} grade={case['grade_deg']:4.1f} "
                  f"{case['controller']:13s} completed={case['completed']!s:5s} settled={case['settled']!s:5s} "
                  f"final={case['final_error_m']*1000:8.1f} mm")
        return
    if args.action == 'holdout':
        labels = [label for label, _, _ in HOLDOUT]
        jobs = [dict(_holdout_job(label, kind, changes, {'mode': mode}), label=[label, mode, start], phase=phase,
                     **({'save': str(output/'holdout'/label.replace(',', '').replace(' ', '_')/mode)}
                        if start == 0 else {}))
                for label, kind, changes in HOLDOUT for mode in MODES for start, phase in enumerate(START_PHASES)]
        results = _grouped(jobs, labels, workers, config)
        table = {label: {mode: _spread(results[label][mode]) for mode in MODES} for label in labels}
        _write(output/'holdout'/'comparison.json', {
            'scope': 'Not used for tuning. Simulated provisional model. Each case is run from each start phase.',
            'start_phases_rad': START_PHASES,
            'cases': [{'label': label, 'terrain': kind, 'changes': changes} for label, kind, changes in HOLDOUT],
            'cost': _cost_definition(), 'overall': _overall(table), 'results': table})
        from .offroad_analysis import plot_comparison
        plot_comparison(results, output/'holdout'/'comparison.png', 'Terrains and headings the tuning never saw')
        print(json.dumps(_headline(table), indent=1))
        return
    if args.action == 'tune':
        chosen, report = tune(workers=workers, config=config)
        lookup = {(sweep['parameter'], value): (cost, fell) for sweep in report['sweeps']
                  for value, cost, fell in zip(sweep['values'], sweep['costs'], sweep['failures'])}
        start = report['records'][0]
        rows = [{'label': 'Contact-aware controller, settings at the start of the sweep', 'cost': start['cost'],
                 'failures': start['failures'], 'reference': True}]
        rows += [{'label': label, 'cost': lookup[name, value][0], 'failures': lookup[name, value][1],
                  'parameter': name, 'value': value} for label, name, value in REMOVALS if (name, value) in lookup]
        rows.append({'label': 'Flat-floor baseline', 'cost': report['flat_baseline']['cost'],
                     'failures': report['flat_baseline']['failures']})
        _write(output/'tuning'/'tuning.json', {
            'method': 'One setting varied at a time from the starting settings; a value is adopted only if it '
                      'lowers the mean cost by more than the margin with no added falls; the combined choice is '
                      'then checked against the start.',
            'scenarios': TUNING_SCENARIOS, 'variants_seed_and_start_phase': TUNING_VARIANTS,
            'travel': {'distance_m': TUNING_DISTANCE_M, 'duration_s': TUNING_DURATION_S},
            'grid': {name: list(values) for name, values in TUNING_GRID}, 'cost': _cost_definition(),
            'chosen': chosen, 'defaults_in_code': OffroadParams().as_dict(), 'ablation': rows, **report})
        from .offroad_analysis import plot_ablation, plot_sweep
        plot_sweep(report['sweeps'], output/'tuning'/'sweep.png')
        plot_ablation(rows, output/'tuning'/'ablation.png')
        print(json.dumps({'chosen': chosen, 'ablation': rows, 'confirmation': report['confirmation']
                          and {key: value for key, value in report['confirmation'].items() if key != 'combined_record'}},
                         indent=2))
