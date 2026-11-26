"""I52 results-owned phase diagnostics and exact I51 reference-guard binding.

The normal flag changes only a lookup angle in the existing FLAT gain bank.
It does not redesign dynamics/gains or transform all terrain error/input frames.
No controller/model/observer/physics methods are invoked here.
"""
from pathlib import Path
import hashlib
import importlib.util
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
GUARD_SOURCE = ROOT/'results/allocation_handoff_isolation_20261126/reference_candidate/joint_reference_guard.py'
GUARD_SHA256 = '80c01ce649ce507dde6e96950fbfeb8b9b7de269a5d51c8980529d5970d5a2a4'
_BOUND_GUARD = None


def bind_guard(source=GUARD_SOURCE):
    """Verify exact frozen bytes before loading; no silent helper revision."""
    global _BOUND_GUARD
    source = Path(source)
    if hashlib.sha256(source.read_bytes()).hexdigest() != GUARD_SHA256:
        raise ValueError('I52 requires the exact frozen I51 reference-guard source')
    if _BOUND_GUARD is None:
        name = '_i52_exact_i51_joint_reference_guard'
        spec = importlib.util.spec_from_file_location(name, source)
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
        _BOUND_GUARD = module
    return _BOUND_GUARD


def guard_identity():
    bind_guard()
    return dict(origin=str(GUARD_SOURCE.relative_to(ROOT)), sha256=GUARD_SHA256,
        speed_limits_rad_s=[20.,20.,20.], reference_acceleration_limits_rad_s2=[100.,100.,100.])


def phase_diagnostics(rotation, normal, *, normal_phase):
    """Use the exact native atan2 convention without changing pose or normal."""
    rotation, normal = np.asarray(rotation, float), np.asarray(normal, float)
    if rotation.shape != (3,3) or normal.shape != (3,) or not np.isfinite(rotation).all() or not np.isfinite(normal).all():
        raise ValueError('Expected finite carrier rotation[3,3] and estimated normal[3]')
    if np.linalg.norm(normal) <= 1e-12:
        raise ValueError('Estimated normal cannot vanish')
    vertical_support = rotation.T@np.array([0.,0.,1.])
    normal_support = rotation.T@normal
    vertical = float(np.arctan2(-vertical_support[0], vertical_support[2]))
    estimated = float(np.arctan2(-normal_support[0], normal_support[2]))
    used = estimated if normal_phase else vertical
    return dict(phase_world_z_rad=vertical, phase_estimated_normal_rad=estimated, phase_used_rad=used,
        phase_normal_minus_world_z_wrapped_rad=float((estimated-vertical+np.pi)%(2*np.pi)-np.pi),
        normal_phase_enabled=bool(normal_phase), estimated_normal_world=normal.copy(),
        world_z_phase_coordinate_defined=bool(np.linalg.norm(vertical_support[[0,2]])>1e-12),
        normal_phase_coordinate_defined=bool(np.linalg.norm(normal_support[[0,2]])>1e-12),
        scope='Current-carrier lookup phases only; flat gains and native error/input frames are unchanged. The original feedback can hold a prior gain while unloaded.')


def prepare(controller, vref, normal, *, reference_guard, normal_phase, touching=None):
    """Prepare immediately before one original _terrain_feedback invocation.

    Root configures schedule_on_ground_normal before controller construction.
    Only the optional exact old guard writes anything: vref[6:9] and its owned
    controller history. Baseline/phase-only calls do not create guard history.
    """
    if bool(controller.params.schedule_on_ground_normal) != bool(normal_phase):
        raise ValueError('Controller phase flag must be configured before the run')
    if not isinstance(vref,np.ndarray) or vref.shape != (9,) or not np.isfinite(vref).all():
        raise ValueError('Expected finite native velocity reference[9]')
    rotation = controller.plant.data.xmat[controller.carrier_id].reshape(3,3)
    diagnostic = phase_diagnostics(rotation, normal, normal_phase=normal_phase)
    native = vref[6:9].copy()
    guard = None
    if reference_guard:
        module = _BOUND_GUARD if _BOUND_GUARD is not None else bind_guard()
        guard = module.apply(controller, vref)
    diagnostic.update(reference_guard_enabled=bool(reference_guard), native_joint_reference_rad_s=native,
        used_joint_reference_rad_s=vref[6:9].copy(), actual_joint_rates_rad_s=controller.plant.data.qvel[6:9].copy(),
        guard_diagnostics=guard, current_gain_lookup_requested=None if touching is None else bool(touching),
        original_feedback_calls_by_this_helper=0)
    return diagnostic


def validate_feedback(diagnostic, feedback, controller=None):
    """Check the one native return, without modifying gains or references."""
    gap = float((feedback['effective_phase_rad']-diagnostic['phase_used_rad']+np.pi)%(2*np.pi)-np.pi)
    if abs(gap) > 1e-12:
        raise AssertionError('Native feedback did not use the logged phase convention')
    np.testing.assert_array_equal(np.asarray(feedback['reference_qvel'])[6:9], diagnostic['used_joint_reference_rad_s'])
    if controller is not None:
        np.testing.assert_array_equal(feedback['K'][0],controller.ordinary['design']['K'][0])
    return dict(passed=True, phase_gap_rad=gap, joint_reference_return_matches=True,
        ordinary_shaft_row_checked=controller is not None)


def reset(controller):
    """Proxy only the old helper's owned state; no native controller reset."""
    if getattr(controller,'_i51_joint_reference_guard',None) is not None:
        module = _BOUND_GUARD if _BOUND_GUARD is not None else bind_guard()
        module.reset(controller)
