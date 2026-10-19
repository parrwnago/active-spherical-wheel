"""Local controller checks; these do not certify tracking across contact regimes."""

import numpy as np
import pytest

from spherical_wheel.control import design_balance_feedback, design_lqr, find_equilibrium
from spherical_wheel.model import generate_model
from spherical_wheel.sim import Plant


@pytest.fixture(scope="module")
def local_design(tmp_path_factory):
    path = generate_model(output_dir=tmp_path_factory.mktemp("control_model"))
    plant = Plant(model_path=path)
    equilibrium = find_equilibrium(plant)
    design = design_lqr(plant, equilibrium)
    baseline = design_balance_feedback(plant, equilibrium)
    return path, equilibrium, design, baseline


def test_equilibrium_is_static_for_the_complete_controller_interval(local_design):
    _, equilibrium, _, _ = local_design
    assert equilibrium["optimizer_success"]
    assert np.linalg.norm(equilibrium["acceleration"]) < 1e-6
    assert np.linalg.norm(equilibrium["step_residual"]) < 1e-7
    assert sum(c["normal_force_N"] for c in equilibrium["contacts"]) > 1.0


def test_complete_step_linearization_predicts_an_independent_perturbation(local_design):
    path, equilibrium, design, _ = local_design
    plant = Plant(model_path=path)
    rng = np.random.default_rng(73)
    state_direction = rng.normal(size=2 * plant.model.nv)
    state_direction /= np.linalg.norm(state_direction)
    input_direction = rng.normal(size=5)
    input_direction /= np.linalg.norm(input_direction)
    # This combination was not used for any individual finite-difference column.
    dx, du = 1e-5 * state_direction, 1e-5 * input_direction
    plant.reset(qpos=equilibrium["qpos"], qvel=equilibrium["qvel"])
    plant.perturb(dx)
    plant.step(equilibrium["u"] + du)
    nonlinear = plant.error(equilibrium["qpos"], equilibrium["qvel"])
    predicted = design["f0"] + design["A"] @ dx + design["B"] @ du
    relative_error = np.linalg.norm(nonlinear - predicted) / np.linalg.norm(nonlinear - design["f0"])
    assert relative_error < 0.02, relative_error


def test_full_closed_model_has_no_concealed_growing_mode(local_design):
    _, _, design, _ = local_design
    # Inspect every coordinate, including yaw and all evolving material phases.
    # Unit eigenvalues are permitted for explicitly unregulated cyclic states.
    full_eigenvalues = np.linalg.eigvals(design["A"] - design["B"] @ design["K"])
    growing = full_eigenvalues[np.abs(full_eigenvalues) > 1.0001]
    assert growing.size == 0, growing


def _assert_two_degree_recovery(path, equilibrium, design):
    plant = Plant(model_path=path)
    plant.reset(qpos=equilibrium["qpos"], qvel=equilibrium["qvel"])
    dx = np.zeros(2 * plant.model.nv)
    dx[3:5] = np.deg2rad([2.0, 2.0])
    plant.perturb(dx)
    for _ in range(round(5.0 / plant.dt)):
        u = equilibrium["u"] - design["K"] @ plant.error(equilibrium["qpos"], equilibrium["qvel"])
        state = plant.step(u)
        assert not state["balance_failed"]
        assert np.all(np.isfinite(state["qpos"]))
        assert np.all(np.abs(state["input"]) <= plant.bounds + 1e-12)
        assert np.all(state["disturbance"] == 0)
    # Baseline project targets: under half a degree and slow body rotation by 5 s.
    # Position tracking, material phase and motor speed are separate milestones.
    assert state["tilt_rad"] < np.deg2rad(0.5), np.rad2deg(state["tilt_rad"])
    assert np.linalg.norm(state["qvel"][3:6]) < 0.05


def test_full_state_lqr_recovers_two_degree_tilt_without_support(local_design):
    path, equilibrium, design, _ = local_design
    _assert_two_degree_recovery(path, equilibrium, design)


def test_baseline_feedback_recovers_two_degree_tilt_without_support(local_design):
    path, equilibrium, _, design = local_design
    _assert_two_degree_recovery(path, equilibrium, design)
