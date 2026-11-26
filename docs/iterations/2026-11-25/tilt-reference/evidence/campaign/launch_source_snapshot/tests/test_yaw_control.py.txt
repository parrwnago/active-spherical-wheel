"""Physical shell-angle invariance and the actual heading-following loop."""
from pathlib import Path
import shutil

import mujoco
import numpy as np
import pytest

from spherical_wheel.control import design_lqr, find_equilibrium
from spherical_wheel.model import generate_model
from spherical_wheel.recovery_control import ScheduledBalanceFeedback, build_gain_library
from spherical_wheel.sim import Plant


@pytest.fixture(scope='module')
def local_feedback(tmp_path_factory):
    path = generate_model(output_dir=tmp_path_factory.mktemp('yaw_control_model'))
    plant = Plant(model_path=path)
    designer = Plant(plant.config, path)
    points = []
    ordinary = None
    for phase, regime, side in ((.3, 'shell', 'A'), (-.3, 'shell', 'B'),
                                (np.pi/2, 'belt', 'A'), (-np.pi/2, 'belt', 'B')):
        equilibrium = find_equilibrium(designer, phase)
        design = design_lqr(designer, equilibrium)
        points.append({'phase': phase, 'regime': regime, 'side': side,
                       'K': design['K'], 'eq': equilibrium})
        if phase == .3:
            ordinary = equilibrium, design
    return plant, ScheduledBalanceFeedback(plant, points), ordinary


@pytest.fixture(scope='module')
def complete_step_feedback(local_feedback, tmp_path_factory):
    plant, local, _ = local_feedback
    designer = Plant(plant.config, plant.model_path)
    cache = tmp_path_factory.mktemp('yaw_actual_schedule')/'control_schedule.npz'
    warm_cache = Path(__file__).resolve().parents[1]/'generated'/'lab'/'control_schedule.npz'
    if warm_cache.is_file():
        shutil.copy2(warm_cache, cache)
    # Read a validated production schedule, or rebuild a private test copy.
    # Tests never rewrite the running lab's gain cache.
    points, _ = build_gain_library(designer, cache_path=cache)
    production = ScheduledBalanceFeedback(plant, points)
    equilibria, designs = {}, {}
    for phase in (.3, -.3, np.pi/2, -np.pi/2, 1.1, 1.2, 1.4):
        equilibria[phase] = find_equilibrium(designer, phase)
        if phase in (.3, -.3, np.pi/2, -np.pi/2, 1.1):
            designs[phase] = design_lqr(designer, equilibria[phase])
    return plant, local, production, equilibria, designs


def test_shell_material_angle_is_not_a_balance_or_heading_target(local_feedback):
    plant, feedback, (equilibrium, design) = local_feedback
    plant.reset(qpos=equilibrium['qpos'], qvel=equilibrium['qvel'])
    perturbation = np.zeros(18)
    perturbation[:2] = [.01, -.006]
    perturbation[3:5] = [.015, -.02]
    perturbation[9:] = [.02, -.01, 0., .03, -.02, .04, .2, -.5, .7]
    plant.perturb(perturbation)
    original = plant.capture()
    first, first_info = feedback.compute(equilibrium['qpos'], equilibrium['qvel'],
                                        equilibrium['u'], design['K'])
    plant.step(first)
    expected_qpos, expected_qvel = plant.data.qpos.copy(), plant.data.qvel.copy()

    plant.restore(original)
    # Arbitrary fractions of a turn, rather than a 2*pi wrap, must be physically
    # irrelevant for axisymmetric shells. Retain all measured drive velocities.
    plant.data.qpos[8:10] += [1.1, -2.3]
    mujoco.mj_forward(plant.model, plant.data)
    reference = equilibrium['qpos'].copy()
    reference[8:10] += [.8, 1.6]
    rotated, info = feedback.compute(reference, equilibrium['qvel'],
                                     equilibrium['u'], design['K'])
    np.testing.assert_allclose(rotated, first, atol=1e-9, rtol=1e-9)
    np.testing.assert_array_equal(info['error'][7:9], 0.)
    np.testing.assert_allclose(info['error'][6], first_info['error'][6], atol=1e-12)
    np.testing.assert_allclose(info['error'][9:], first_info['error'][9:], atol=1e-12)
    assert np.max(np.abs(info['error'][16:18])) > .1
    plant.step(rotated)
    np.testing.assert_allclose(plant.data.qpos[:8], expected_qpos[:8], atol=1e-9, rtol=1e-9)
    np.testing.assert_allclose(plant.data.qvel, expected_qvel, atol=1e-8, rtol=1e-8)


@pytest.mark.parametrize('phase,production_schedule', [
    (.3, False), (-.3, False), (np.pi/2, False), (-np.pi/2, False),
    (1.1, True), (1.2, True), (1.4, True),
])
def test_complete_heading_following_feedback_step_has_no_growing_local_mode(
        complete_step_feedback, phase, production_schedule):
    plant, local, production, equilibria, designs = complete_step_feedback
    feedback = production if production_schedule else local
    equilibrium = equilibria[phase]
    # The transition preset keeps its initial 1.1-rad ordinary shaft row while
    # the other rows follow current contact. Initial-support tests use the
    # ordinary row of their own reset pose, including either polar module.
    ordinary_phase = 1.1 if production_schedule else phase
    design = designs[ordinary_phase]
    plant.reset(qpos=equilibrium['qpos'], qvel=equilibrium['qvel'])
    base = plant.capture()
    epsilon = 1e-6
    columns = []
    try:
        for index in range(2*plant.model.nv):
            values = []
            for sign in (1., -1.):
                plant.restore(base)
                tangent = np.zeros(2*plant.model.nv)
                tangent[index] = sign*epsilon
                plant.perturb(tangent)
                command, _ = feedback.compute(equilibrium['qpos'], equilibrium['qvel'],
                                              equilibrium['u'], design['K'])
                plant.step(command)
                values.append(plant.error(equilibrium['qpos'], equilibrium['qvel']))
            columns.append((values[0]-values[1])/(2*epsilon))
    finally:
        plant.restore(base)
    closed_loop = np.column_stack(columns)
    # Differentiate the actual controller, whose reference yaw follows measured
    # heading. Removing shell columns from a fixed-heading K would test a
    # different feedback law. Unregulated cyclic coordinates remain neutral.
    assert max(abs(np.linalg.eigvals(design['A']))) > 1.001
    radius = max(abs(np.linalg.eigvals(closed_loop)))
    print(f'Actual feedback step phase={phase:.8f}, ordinary={ordinary_phase:.8f}, '
          f'production_schedule={production_schedule}, spectral_radius={radius:.12f}')
    assert radius < 1.00001
