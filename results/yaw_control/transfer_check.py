"""Recheck the controlled cap exit at 1 ms and 0.5 ms physics steps.

Run from the repository root with the project Python environment. The existing
test helper supplies matched initial states, reference motion and physical
guards. Gains are redesigned at each physics step; this is not a fixed-gain
discretization study.
"""
import os
os.environ['OPENBLAS_NUM_THREADS'] = '1'
os.environ['OMP_NUM_THREADS'] = '1'
from pathlib import Path
import hashlib
import importlib.util
import json
import tempfile

import numpy as np
from spherical_wheel.lab_runtime import SimulationLab
from spherical_wheel.model import generate_model, load_config
import spherical_wheel.recovery_control as feedback


def main():
    root = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('transfer_helper', root/'tests/test_recovery.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    report = {'feedback_sha256': hashlib.sha256(Path(feedback.__file__).read_bytes()).hexdigest(),
              'scope': 'Matched cap exit; redesign gains at each physics step; provisional physical model.',
              'runs': []}
    for step in (.001, .0005):
        config = load_config()
        config['simulation']['timestep_s'] = step
        with tempfile.TemporaryDirectory(prefix='yaw-transfer-') as temp:
            lab = SimulationLab(config, generate_model(config, Path(temp)/'model'))
            starts = []
            for enabled in (False, True):
                start, trace, first = helper._controlled_cap_exit(lab, enabled)
                starts.append(start)
                row = {'timestep_s': step, 'synchronization_enabled': enabled,
                       'first_1N_load_time_s': float(trace[first, 0]),
                       'candidate_speed_5ms_before_m_s': float(trace[first-1, 1]),
                       'candidate_speed_20ms_before_m_s': float(trace[first-4, 1]),
                       'normal_load_5ms_before_N': float(trace[first-1, 2]),
                       'actual_slip_at_first_load_m_s': float(trace[first, 3]),
                       'peak_preload_blend': float(max(trace[:first, 4]))}
                report['runs'].append(row)
                print(json.dumps(row), flush=True)
            for before, after in zip(*starts):
                np.testing.assert_array_equal(before, after)
    Path(__file__).with_suffix('.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
