"""Repeat terrain runs at half the physics step, with gains designed for each step.

Run from the repository root with the project environment:

    python results/offroad/numerical_step_check/reproduce.py

Writes ``summary.json`` beside this file. Generated models go under
``generated/offroad/step_check``, which is not tracked.
"""
import copy
import json
from pathlib import Path

import mujoco

from spherical_wheel.control import jsonable
from spherical_wheel.model import REPO_ROOT, load_config
from spherical_wheel.offroad_control import OffroadParams
from spherical_wheel.offroad_experiments import Workspace, run_offroad, scenario

SCENARIOS = ('ramp_forward', 'cross_slope', 'course')


def main():
    report = {
        'scope': 'Same scenarios and controller settings at two physics steps. The flat-floor gain library is '
                 'designed separately for each step. Controller interval 5 ms in both. Simulated provisional model.',
        'engine_version': mujoco.__version__, 'controller_parameters': OffroadParams().as_dict(), 'runs': []}
    for step in (.001, .0005):
        config = copy.deepcopy(load_config())
        config['simulation']['timestep_s'] = step
        workspace = Workspace(config, REPO_ROOT/'generated'/'offroad'/'step_check'/f'step_{step}')
        for name in SCENARIOS:
            terrain, path = scenario(name)
            _, summary = run_offroad(workspace, terrain, path, OffroadParams())
            row = {'scenario': name, 'timestep_s': step, 'completed': summary['completed'],
                   'rms_error_mm': summary['tracking']['rms_error_m']*1000,
                   'max_error_mm': summary['tracking']['max_error_m']*1000,
                   'final_error_mm': summary['tracking']['final_error_m']*1000,
                   'max_attitude_error_deg': summary['attitude']['max_error_from_reference_deg'],
                   'friction_use_p95': summary['contact']['utilization_p95'],
                   'sliding_distance_mm': summary['contact']['slip_distance_m']*1000,
                   'normal_estimate_error_rms_deg': summary['contact']['normal_estimate_error_rms_deg'],
                   'largest_normal_load_x_weight': summary['contact']['normal_load_fraction_max'],
                   'heading_change_deg': summary['attitude']['heading_change_deg']}
            report['runs'].append(row)
            print(json.dumps(jsonable(row)), flush=True)
    target = Path(__file__).resolve().parent/'summary.json'
    target.write_text(json.dumps(jsonable(report), indent=2)+'\n')
    print(target)


if __name__ == '__main__':
    main()
