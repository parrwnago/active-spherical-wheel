"""Checks for the material risks in a free-base, internally driven wheel."""

import mujoco
import numpy as np
import pytest

from spherical_wheel.model import generate_model
from spherical_wheel.validation import (
    belt_wrapper_response,
    floor_contacts,
    inspect_topology,
    isolated_reaction_test,
    phase_contact_sweep,
    polar_shell_leverage,
)


@pytest.fixture(scope="module")
def model_path(tmp_path_factory):
    return generate_model(output_dir=tmp_path_factory.mktemp("physics_model"))


def test_free_base_and_physical_inertias(model_path):
    report = inspect_topology(mujoco.MjModel.from_xml_path(str(model_path)))
    assert report["status"] == "passed", report


def test_each_internal_motor_reacts_and_conserves_momentum(model_path):
    report = isolated_reaction_test(model_path)
    assert report["status"] == "passed", report


def test_polar_openings_expose_both_belts_to_the_floor(model_path):
    report = phase_contact_sweep(model_path)
    assert report["both_polar_belts_reach_floor"], report


def test_shell_spin_loses_leverage_at_its_pole(model_path):
    report = polar_shell_leverage(model_path)
    assert report["status"] == "passed", report


def test_belt_zero_to_nonzero_command_moves_the_free_plant(model_path):
    report = belt_wrapper_response(model_path)
    assert report["status"] == "passed", report


def test_fork_floor_contact_is_identified_as_balance_failure(model_path):
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    data.qpos[3:7] = [np.sqrt(0.5), np.sqrt(0.5), 0, 0]
    # Deliberately place the tilted assembly through the ground.  This is solely
    # a failure detector check and never a balance initial condition.
    data.qpos[2] = 0.005
    mujoco.mj_forward(model, data)
    assert floor_contacts(model, data), "Fork/platform ground contact must end a balance trial."
    from spherical_wheel.sim import Plant
    plant = Plant(model_path=model_path)
    plant.reset(qpos=data.qpos)
    assert plant.snapshot()["balance_failed"]
