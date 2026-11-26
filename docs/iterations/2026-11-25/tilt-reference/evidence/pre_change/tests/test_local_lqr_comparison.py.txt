"""Check the off-road adapter against the original lab's actual control step."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from spherical_wheel.lab_runtime import SimulationLab
from spherical_wheel.mobility import MobilityPath
from spherical_wheel.offroad_experiments import Workspace, place_on_ground
from spherical_wheel.offroad_control import OffroadParams
from spherical_wheel.terrain import make_terrain
from spherical_wheel.model import REPO_ROOT

spec = importlib.util.spec_from_file_location('local_lqr_comparison', REPO_ROOT/'scripts'/'compare-local-lqr-offroad.py')
comparison = importlib.util.module_from_spec(spec)
spec.loader.exec_module(comparison)


@pytest.fixture(scope='module')
def workspace(tmp_path_factory):
    cache = REPO_ROOT/'generated'/'offroad'/'mobility'/'design_0.001'/'flat'/'control_schedule.npz'
    return Workspace(directory=tmp_path_factory.mktemp('local_lqr'), seed_cache=cache)


@pytest.mark.parametrize('terrain', ['flat', 'slope'])
@pytest.mark.parametrize('velocity,acceleration', [([0., 0.], [0., 0.]), ([.03, -.02], [.01, .02])])
def test_adapter_matches_original_lab_step(workspace, terrain, velocity, acceleration):
    plant = workspace.plant(make_terrain(terrain))
    class Adapter(comparison.LocalLQRAdapter):
        path = MobilityPath('circle', duration_s=4., radius_m=.015)
    adapter = Adapter(plant, workspace.design, OffroadParams(mode='flat_baseline', synchronize=False), .3)
    place_on_ground(plant, adapter, adapter.path.origin, .3)
    nominal = adapter.nominal_joints.copy()
    xy = np.array([.02, -.01])
    velocity, acceleration = np.array(velocity), np.array(acceleration)
    applied, diagnostics = adapter.command(xy, velocity, acceleration)
    lab = SimulationLab.__new__(SimulationLab)
    lab.plant, lab.config, lab.dt = plant, plant.config, plant.dt
    lab.failed = False
    lab.controller_mode = 'local_lqr'
    lab.controller_enabled = lab.belt_enabled = True
    lab.synchronization_enabled = False
    lab.schedule, lab.selected = adapter.lab.schedule, adapter.lab.selected
    lab.nominal_joints = nominal
    lab.trajectory = {'kind': 'curve'}
    lab._reference = lambda: (xy, velocity, acceleration)
    lab._record = lambda now: None
    lab.pulses, lab.events = [], []
    lab._disturbance_end_s = None
    lab._advance_one()
    np.testing.assert_allclose(applied, lab._last_input, rtol=0, atol=1e-12)
    np.testing.assert_allclose(diagnostics['requested'], lab._last_command, rtol=0, atol=1e-12)
    np.testing.assert_allclose(diagnostics['reference_qpos'], lab._last_ref, rtol=0, atol=1e-12)
    np.testing.assert_allclose(diagnostics['reference_qvel'], lab._last_vref, rtol=0, atol=1e-12)
    adapter.after_step()
    np.testing.assert_allclose(adapter.nominal_joints, lab.nominal_joints, rtol=0, atol=1e-12)
