"""Check that smooth support cannot fill a shell opening or equatorial gap."""

import copy

import mujoco
import numpy as np
import pytest

from spherical_wheel.control import find_equilibrium, linearize
from spherical_wheel.model import generate_model, load_config
from spherical_wheel.sim import Plant
from spherical_wheel.validation import belt_wrapper_response


@pytest.fixture(scope="module")
def clipped_model(tmp_path_factory):
    config = copy.deepcopy(load_config())
    config["contact"]["mode"] = "analytic_clipped_floor"
    path = generate_model(config, output_dir=tmp_path_factory.mktemp("clipped_model"))
    return config, path


def _floor_geoms(plant):
    names = set()
    for c in plant.data.contact:
        pair = [mujoco.mj_id2name(plant.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in c.geom]
        if "floor" in pair:
            names.update(n for n in pair if n != "floor")
    return names


@pytest.mark.parametrize("phase,expected", [(-60, "shell_B_smooth"), (-30, "shell_B_smooth"),
                                            (30, "shell_A_smooth"), (60, "shell_A_smooth")])
def test_interior_support_uses_only_the_exposed_shell(clipped_model, phase, expected):
    config, path = clipped_model
    plant = Plant(config=config, model_path=path)
    plant.reset(shaft_phase=np.deg2rad(phase))
    plant.data.qpos[2] -= 0.0005
    mujoco.mj_forward(plant.model, plant.data)
    assert _floor_geoms(plant) == {expected}


@pytest.mark.parametrize("phase,expected", [(90, "belt_A_smooth"), (-90, "belt_B_smooth")])
def test_polar_opening_exposes_belt_and_excludes_shell(clipped_model, phase, expected):
    config, path = clipped_model
    plant = Plant(config=config, model_path=path)
    plant.reset(shaft_phase=np.deg2rad(phase))
    plant.data.qpos[2] -= 0.0005
    mujoco.mj_forward(plant.model, plant.data)
    assert _floor_geoms(plant) == {expected}


def test_equatorial_gap_uses_finite_rims_without_sphere_support(clipped_model):
    config, path = clipped_model
    plant = Plant(config=config, model_path=path)
    plant.reset(shaft_phase=0)
    plant.data.qpos[2] -= 0.001
    mujoco.mj_forward(plant.model, plant.data)
    names = _floor_geoms(plant)
    assert names
    assert not any(n.endswith("_smooth") for n in names)
    assert any(n.startswith("shell_A_sector") for n in names)
    assert any(n.startswith("shell_B_sector") for n in names)


def test_smooth_contacts_respect_the_physical_axial_bands(clipped_model):
    config, path = clipped_model
    plant = Plant(config=config, model_path=path)
    geometry = config["geometry"]
    radius = geometry["sphere_radius_m"]
    rim = radius * np.cos(np.deg2rad(geometry["polar_cap_angle_deg"]))
    gap = geometry["equatorial_gap_half_m"]
    for phase in np.deg2rad(np.linspace(-90, 90, 37)):
        plant.reset(tilt=(0.03, -0.02), shaft_phase=phase)
        plant.data.qpos[2] -= 0.0008
        mujoco.mj_forward(plant.model, plant.data)
        for name in _floor_geoms(plant):
            if not name.endswith("_smooth"):
                continue
            geom = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_GEOM, name)
            body = int(plant.model.geom_bodyid[geom])
            axis = plant.data.xmat[body].reshape(3, 3)[:, 0]
            signed_axial_distance = (-1 if "_B_" in name else 1) * (-radius * axis[2])
            lower, upper = (rim, radius) if name.startswith("belt") else (gap, rim)
            assert lower - 1e-10 <= signed_axial_distance <= upper + 1e-10


def test_shell_material_rotation_does_not_create_artificial_support_force(clipped_model):
    config, path = clipped_model
    plant = Plant(config=config, model_path=path)
    equilibrium = find_equilibrium(plant)
    plant.reset(qpos=equilibrium["qpos"], qvel=equilibrium["qvel"])
    mujoco.mj_forward(plant.model, plant.data)
    reference_acceleration = plant.data.qacc.copy()
    for phase in (0.123456, 0.8, 2.4):
        qpos = equilibrium["qpos"].copy()
        qpos[8:10] = [phase, -0.7 * phase]
        plant.reset(qpos=qpos, qvel=equilibrium["qvel"])
        assert _floor_geoms(plant) == {"shell_A_smooth"}
        assert np.linalg.norm(plant.data.qacc - reference_acceleration) < 1e-8
    A, _, _ = linearize(plant, equilibrium)
    for state_index in (7, 8):
        expected = np.eye(18)[:, state_index]
        assert np.linalg.norm(A[:, state_index] - expected) < 1e-6


def test_clipped_belts_work_through_the_complete_command_wrapper(clipped_model):
    config, path = clipped_model
    report = belt_wrapper_response(path, config=config)
    assert report["status"] == "passed", report


def test_raw_mjcf_does_not_enable_unfiltered_full_spheres(clipped_model):
    _, path = clipped_model
    # The global callback can already be installed by earlier tests.  A raw
    # compiled model must still keep every proxy disabled until installation.
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    data.qpos[7] = 0.30
    data.qpos[2] -= 0.001
    mujoco.mj_forward(model, data)
    for c in data.contact:
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in c.geom]
        assert not any(n and n.endswith("_smooth") for n in names)
