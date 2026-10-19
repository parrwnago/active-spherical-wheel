"""Record the 12 s recovery regressions and their longer tails, with sync on/off."""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
from pathlib import Path
import importlib.util
import json

import numpy as np
from spherical_wheel.lab_runtime import SimulationLab
from spherical_wheel.model import generate_model


def main():
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('transient_helper', root/'tests/test_recovery.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    lab = SimulationLab(model_path=generate_model(output_dir=root/'generated/lab'))
    results = {}
    cases = [('shell', .5, .3, .15, [(0., 1., 0.)]),
             ('polar', 1., .2, .25, [(1., 0., 0.), (0., 1., 0.), (-1., 0., 0.), (0., -1., 0.)])]
    for scenario, strength, duration, spacing, directions in cases:
        for enabled in (False, True):
            result = helper._repeated_pushes(lab, scenario, strength, duration, spacing, directions, enabled)
            result['impulse_Ns'] = result['impulse_Ns'].tolist()
            points = {'12': result}
            for end in (20, 30, 60):
                while lab.plant.data.time < end-1e-8 and not lab.failed:
                    lab.advance(steps=min(2000, round((end-lab.plant.data.time)/lab.dt)))
                state = lab.state()
                points[str(end)] = {key: state[key] for key in (
                    'time_s', 'failed', 'tilt_deg', 'angular_speed_rad_s',
                    'yaw_rate_rad_s', 'tracking_error_m', 'motion_settled')}
            results[f'{scenario}_sync{int(enabled)}'] = points
            print(scenario, enabled, json.dumps(points), flush=True)
    results['cap_exit'] = {}
    for label, enabled in (('off', False), ('on', True)):
        _, trace, first = helper._controlled_cap_exit(lab, enabled)
        results['cap_exit'][label] = {
            'first_load_time_s': float(trace[first, 0]), 'pre5ms_m_s': float(trace[first-1, 1]),
            'pre20ms_m_s': float(trace[first-4, 1]), 'actual_slip_m_s': float(trace[first, 3]),
            'peak_blend': float(max(trace[:first, 4]))}
    (Path(__file__).parent/'transient_checks.json').write_text(json.dumps(results, indent=2)+'\n')


if __name__ == '__main__':
    main()
