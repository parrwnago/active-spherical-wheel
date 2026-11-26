"""Record and analyze existing Mars correction cases using estimated contacts.

Run inside the Gazebo environment with MUJOCO_GL=egl. This diagnostic study
uses the figure-eight at three wheel phases and triangle/out-and-back as
regressions. Ground truth stays in the evaluator; controller contact reads
are forbidden during every command.
"""
from concurrent.futures import ProcessPoolExecutor, as_completed
from contextlib import ExitStack
import argparse
import hashlib
import json
import logging
from pathlib import Path
import subprocess
from unittest.mock import patch

from spherical_wheel.contact_diagnostics import analyze_run
from spherical_wheel.model import REPO_ROOT
from spherical_wheel.offroad_control import OffroadController, OffroadParams
from spherical_wheel.offroad_experiments import save_run
from spherical_wheel.gazebo.experiments import run_gazebo
from spherical_wheel.gazebo.mars import (
    ROUTES as MARS_ROUTES, SHAPE_ROUTES, _workspace, build_route, evaluate_route, route_closed, route_item,
)

ROUTES = ('plain_figure8', 'shape_triangle', 'shape_line_and_back')


def run_case(job):
    name, phase = job['route'], job['phase']
    directory = Path(job['output'])/name/f'phase_{phase:+.2f}'
    logging.getLogger('matplotlib.font_manager').setLevel(logging.ERROR)
    workspace, yard = _workspace(Path(job['output'])/'_workspaces', job.get('workspace_routes', list(ROUTES)))
    path, yaw = build_route(name)
    original = OffroadController.command
    calls = 0

    def forbidden(*args, **kwargs):
        raise AssertionError('Controller attempted to read simulator contacts')

    def guarded(self, *args, **kwargs):
        nonlocal calls
        assert self.plant.ground is None and self.plant.terrain is None and self.plant.data.ncon == 0
        with ExitStack() as stack:
            for target in (
                'spherical_wheel.offroad_control.measure_contact',
                'spherical_wheel.offroad_control.rolling_feedforward',
                'spherical_wheel.offroad_control.records_from_mujoco',
                'spherical_wheel.synchronization.ContactSynchronizer._normal_loads',
                'mujoco.mj_contactForce',
            ):
                stack.enter_context(patch(target, forbidden))
            output = original(self, *args, **kwargs)
        calls += 1
        return output

    parameters = dict(mode='hardware')
    if job['variant'] != 'baseline':
        parameters['rolling_allocation'] = job['variant']
    parameters.update(job.get('parameters', {}))
    with patch.object(OffroadController, 'command', guarded):
        log, summary = run_gazebo(workspace, yard, path, OffroadParams(**parameters),
                                  phase=phase, yaw_deg=yaw, record_diagnostics=True)
    summary = evaluate_route(log, summary, path, yard, route_closed(name))
    summary.update(route={'name': name, 'about': route_item(name)['about'],
                          'start_phase_rad': phase, 'initial_yaw_deg': yaw, 'path': path.spec()},
                   contact_input_audit={'checked_controller_calls': calls,
                                        'forbidden_contact_reads': 0,
                                        'state_inputs': 'Exact simulated measurements; no noise/delay.'})
    save_run(directory, log, summary, yard)
    analyze_run(log, directory/'diagnostics', title=f'{name}, phase {phase:+.2f}')
    return {'route': name, 'phase_rad': phase, 'variant': job['variant'],
            'completed': summary['completed'], 'failure': summary['failure'],
            'rms_moving_mm': 1000*summary['tracking']['rms_error_while_moving_m'],
            'max_error_mm': 1000*summary['tracking']['max_error_m'],
            'final_error_mm': 1000*summary['tracking']['final_error_m'],
            'max_attitude_error_deg': summary['attitude']['max_error_from_reference_deg'],
            'saturated_fraction': summary['actuation']['saturated_time_fraction'],
            'passed_all_gates': summary['mars']['passed'],
            'guarded_calls': calls, 'directory': str(directory)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--variant', choices=('baseline', 'damped', 'bounded', 'feasible'), default='baseline')
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--suite', choices=('focused', 'broad'), default='focused')
    parser.add_argument('--route', action='append', choices=tuple(MARS_ROUTES)+tuple(SHAPE_ROUTES))
    parser.add_argument('--phase', type=float, action='append')
    parser.add_argument('--effort-weight', type=float, help='Bounded allocation speed cost in m/s per normalized unit')
    parser.add_argument('--change-weight', type=float, help='Bounded allocation change cost in m/s per normalized unit')
    parser.add_argument('--adaptive-effort', action='store_true')
    parser.add_argument('--catchup-gain', type=float, help='Feasible-reference position-to-velocity gain; zero disables it')
    parser.add_argument('--position-error-limit', type=float, help='Feasible-reference projected position-error cap in metres')
    parser.add_argument('--balance-guard', action='store_true', help='Smoothly relax travel setpoints as reference lean error/rate grows')
    args = parser.parse_args()
    if args.variant not in ('bounded', 'feasible') and (args.adaptive_effort or args.effort_weight is not None or args.change_weight is not None):
        parser.error('Effort/change settings require --variant bounded or feasible')
    if args.variant != 'feasible' and (args.catchup_gain is not None or args.position_error_limit is not None or args.balance_guard):
        parser.error('Reference settings require --variant feasible')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    routes = args.route or (list(MARS_ROUTES)+list(SHAPE_ROUTES) if args.suite == 'broad' else list(ROUTES))
    repeated = ROUTES if args.suite == 'broad' else ('plain_figure8',)
    parameters = {name: value for name, value in (
        ('rolling_effort_m_s', args.effort_weight), ('rolling_change_m_s', args.change_weight)) if value is not None}
    if args.adaptive_effort:
        parameters['rolling_adaptive_effort'] = True
    if args.catchup_gain is not None:
        parameters['rolling_reference_catchup_gain_s_inv'] = args.catchup_gain
    if args.position_error_limit is not None:
        parameters['rolling_reference_position_limit_m'] = args.position_error_limit
    if args.balance_guard:
        parameters['rolling_reference_balance_guard'] = True
    jobs = [{'route': name, 'phase': phase, 'variant': args.variant, 'output': str(output),
             'parameters': parameters, 'workspace_routes': routes}
            for name in routes for phase in (args.phase or ((.30, .90, -1.) if name in repeated else (.30,)))]
    sources = ('offroad_control.py', 'offroad_experiments.py', 'contact_kinematics.py',
               'contact_diagnostics.py', 'experiments.py', 'synchronization.py', 'rolling_allocation.py', 'feasible_reference.py')
    provenance = {'git_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
                  'runner_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  'variant': args.variant, 'jobs': jobs,
                  'source_sha256': {name: hashlib.sha256((REPO_ROOT/'src/spherical_wheel'/name).read_bytes()).hexdigest()
                                    for name in sources}}
    (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    results = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(run_case, job) for job in jobs]
        for future in as_completed(futures):
            result = future.result()
            results.append(result)
            print(json.dumps(result), flush=True)
            (output/'summary.json').write_text(json.dumps(results, indent=2)+'\n')
    print(f'Completed diagnostic study: {len(results)} cases.', flush=True)


if __name__ == '__main__':
    main()
