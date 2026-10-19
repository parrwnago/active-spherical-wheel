"""Regenerate physical yaw evidence from the real MuJoCo plant.

Run from the project root with the test environment:
    .venv/bin/python results/yaw_control/mechanics/reproduce.py

Models are generated in a temporary directory; production configuration and
geometry are untouched. The paired free-space experiments disable gravity and
contacts only in their private plant copies, retaining default joint damping.
"""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile

import mujoco

from spherical_wheel.model import generate_model, load_config


ROOT = Path(__file__).resolve().parents[3]
test_path = ROOT/'tests'/'test_yaw_mechanics.py'
spec = importlib.util.spec_from_file_location('yaw_mechanics_checks', test_path)
checks = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checks)

with tempfile.TemporaryDirectory(prefix='spherical_yaw_mechanics_') as temporary:
    model_path = generate_model(output_dir=Path(temporary)/'model')
    result = {
        'engine': 'MuJoCo', 'engine_version': mujoco.__version__,
        'configuration': load_config(),
        'generated_model_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest(),
        'regression_source_sha256': hashlib.sha256(test_path.read_bytes()).hexdigest(),
        'scope': 'One free spherical wheel assembly; real internal joint torques and actual ground contact forces. No yaw constraint or external yaw torque.',
        'private_free_space_conditions': 'Gravity and collision masks disabled only in the two isolated test plant copies. Default masses, inertias, damping and actuator bounds retained.',
        'polar_single_shell': checks.internal_pulse(model_path),
        'polar_opposing_shells': checks.internal_pulse(model_path, opposing=True),
        'equatorial_opposing_shells': checks.dual_rim_pulse(model_path),
    }
    checks.test_polar_internal_motor_yaws_free_platform_and_conserves_total_momentum(model_path)
    checks.test_opposite_polar_shell_torques_cancel_platform_reaction(model_path)
    checks.test_dual_equatorial_rims_generate_yaw_couple_without_net_horizontal_force(model_path)
    result['mechanics_regressions'] = '3 passed'
destination = Path(__file__).resolve().parent/'summary.json'
destination.write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps({
    'polar_single_shell_yaw_deg': result['polar_single_shell']['fork_yaw_deg'],
    'polar_opposing_shells_yaw_deg': result['polar_opposing_shells']['fork_yaw_deg'],
    'equatorial_opposing_shells_yaw_deg': result['equatorial_opposing_shells']['fork_yaw_deg'],
    'equatorial_ground_yaw_moment_Nm': result['equatorial_opposing_shells']['final_ground_contacts']['moment_about_wheel_world_Nm'][2],
    'mechanics_regressions': result['mechanics_regressions'], 'saved': str(destination),
}, indent=2))
