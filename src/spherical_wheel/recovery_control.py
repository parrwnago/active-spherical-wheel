"""Contact-scheduled feedback for the nonlinear interactive balance experiments.

The controller preserves the requested world-space position and physical input
limits. It schedules feedback using the carrier's orientation and the actual
normal loads, follows the platform's heading, and wraps cyclic joint errors.
The companion library contains locally designed gains, not a global stability
certificate. Finite-rim models with extreme shaft gains are rejected.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import tempfile

import mujoco
import numpy as np


_CACHE_VERSION = 1
_SUPPORTS = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
_SOURCE_FILES = ('control.py', 'model.py', 'sim.py', 'contact.py', 'recovery_control.py')


def _json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f'Cannot encode {type(value).__name__} in gain-library metadata')


def _fingerprint(designer):
    directory = Path(__file__).resolve().parent
    sources = {name: hashlib.sha256((directory/name).read_bytes()).hexdigest()
               for name in _SOURCE_FILES}
    model_path = getattr(designer, 'model_path', None)
    payload = {
        'cache_version': _CACHE_VERSION,
        'mujoco_version': mujoco.__version__,
        'configuration': designer.config,
        'source_sha256': sources,
        'model_xml_sha256': hashlib.sha256(Path(model_path).read_bytes()).hexdigest()
                            if model_path is not None else None,
        'nq': int(designer.model.nq), 'nv': int(designer.model.nv),
        'input_bounds': np.asarray(designer.bounds).tolist(),
        'controller_interval_s': float(designer.dt),
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'),
                         allow_nan=False, default=_json_default).encode()
    return hashlib.sha256(encoded).hexdigest(), payload


def _initial_phases():
    # Keep the experimental grid's floating-point phases: the same grid produced
    # the validated 80 accepted default-model points, including the pole itself.
    return sorted(set(np.r_[np.arange(-.4, .901, .05),
                            np.arange(1.10, 2.051, .05), .30, np.pi/2]))


def _point(designer, result, phase):
    equilibrium = result.get('equilibrium', result.get('eq'))
    design = result.get('design', result)
    if equilibrium is None:
        raise ValueError('A gain design must include its equilibrium')
    K = np.asarray(design['K'], dtype=float)
    if K.shape != (5, 2*designer.model.nv) or not np.all(np.isfinite(K)):
        raise ValueError('The local feedback matrix has invalid dimensions or values')
    radius = float(design.get('full_closed_spectral_radius', np.nan))
    if not np.isfinite(radius) or radius > 1.0001:
        raise ValueError(f'Local closed-loop spectral radius is {radius}')
    qpos = np.asarray(equilibrium['qpos'], dtype=float)
    qvel = np.asarray(equilibrium['qvel'], dtype=float)
    effort = np.asarray(equilibrium['u'], dtype=float)
    residual = np.asarray(equilibrium.get('step_residual', np.zeros(2*designer.model.nv)), dtype=float)
    if (qpos.shape != (designer.model.nq,) or qvel.shape != (designer.model.nv,)
            or effort.shape != (5,) or residual.shape != (2*designer.model.nv,)
            or not all(np.all(np.isfinite(array)) for array in (qpos, qvel, effort, residual))):
        raise ValueError('The local equilibrium has invalid dimensions or values')
    if np.max(np.abs(residual)) > 1e-5:
        raise ValueError('The equilibrium does not remain static over a controller interval')
    contacts = equilibrium.get('contacts', [])
    belt_loaded = any(any(name.startswith('belt_') for name in contact['geoms'])
                      and contact['normal_force_N'] > 1. for contact in contacts)
    loaded_names = [name for contact in contacts if contact['normal_force_N'] > 1.
                    for name in contact['geoms'] if name.startswith(('shell_', 'belt_'))]
    return {
        'phase': float(phase), 'K': K.copy(),
        'regime': 'belt' if belt_loaded else 'shell',
        'side': 'A' if any(name.startswith(('shell_A', 'belt_A')) for name in loaded_names) else 'B',
        'eq': {'qpos': qpos.copy(), 'qvel': qvel.copy(), 'u': effort.copy(),
               'step_residual': residual.copy()},
        'full_closed_spectral_radius': radius,
    }


def _validate_groups(points):
    names = {('belt_' if point['regime'] == 'belt' else 'shell_')+point['side']
             for point in points}
    missing = set(_SUPPORTS)-names
    if missing:
        raise RuntimeError(f'No usable local feedback for {", ".join(sorted(missing))}')


def _write_cache(path, points, metadata):
    path.parent.mkdir(parents=True, exist_ok=True)
    arrays = {
        'phase_rad': np.array([point['phase'] for point in points], dtype=np.float64),
        'K': np.array([point['K'] for point in points], dtype=np.float64),
        'qpos': np.array([point['eq']['qpos'] for point in points], dtype=np.float64),
        'qvel': np.array([point['eq']['qvel'] for point in points], dtype=np.float64),
        'equilibrium_u': np.array([point['eq']['u'] for point in points], dtype=np.float64),
        'step_residual': np.array([point['eq']['step_residual'] for point in points], dtype=np.float64),
        'spectral_radius': np.array([point['full_closed_spectral_radius'] for point in points], dtype=np.float64),
        'regime_code': np.array([point['regime'] == 'belt' for point in points], dtype=np.int8),
        'side_code': np.array([point['side'] == 'B' for point in points], dtype=np.int8),
        # Unicode, rather than an object array, keeps allow_pickle=False safe.
        'metadata_json': np.array(json.dumps(metadata, sort_keys=True, allow_nan=False,
                                             default=_json_default)),
    }
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, prefix=path.name+'.', suffix='.tmp', delete=False) as stream:
            temporary = Path(stream.name)
            np.savez_compressed(stream, **arrays)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _read_cache(path, fingerprint, designer):
    with np.load(path, allow_pickle=False) as saved:
        metadata = json.loads(str(saved['metadata_json'].item()))
        if metadata.get('fingerprint') != fingerprint:
            raise ValueError('Gain-library fingerprint changed')
        names = ('phase_rad', 'K', 'qpos', 'qvel', 'equilibrium_u', 'step_residual',
                 'spectral_radius', 'regime_code', 'side_code')
        arrays = {name: np.array(saved[name], copy=True) for name in names}
    count = arrays['phase_rad'].size
    expected = {'phase_rad': (count,), 'K': (count, 5, 2*designer.model.nv),
                'qpos': (count, designer.model.nq), 'qvel': (count, designer.model.nv),
                'equilibrium_u': (count, 5), 'step_residual': (count, 2*designer.model.nv),
                'spectral_radius': (count,), 'regime_code': (count,), 'side_code': (count,)}
    for name, array in arrays.items():
        if (array.shape != expected[name] or array.dtype.kind not in 'fiu'
                or not np.all(np.isfinite(array))):
            raise ValueError(f'Invalid cached gain-library array: {name}')
    if (not count or np.any(arrays['spectral_radius'] > 1.0001)
            or np.any(np.abs(arrays['K'][:, 0, 4]) > 100.)
            or np.any(np.abs(arrays['step_residual']) > 1e-5)
            or np.any(~np.isin(arrays['regime_code'], (0, 1)))
            or np.any(~np.isin(arrays['side_code'], (0, 1)))):
        raise ValueError('Cached gain-library acceptance checks failed')
    points = []
    for index in range(count):
        points.append({
            'phase': float(arrays['phase_rad'][index]), 'K': arrays['K'][index],
            'regime': 'belt' if arrays['regime_code'][index] else 'shell',
            'side': 'B' if arrays['side_code'][index] else 'A',
            'eq': {'qpos': arrays['qpos'][index], 'qvel': arrays['qvel'][index],
                   'u': arrays['equilibrium_u'][index], 'step_residual': arrays['step_residual'][index]},
            'full_closed_spectral_radius': float(arrays['spectral_radius'][index]),
        })
    _validate_groups(points)
    return points, {**metadata, 'cache_loaded': True}


def build_gain_library(designer, design_callback=None, cache_path=None):
    """Build or safely load gains for all four contact supports.

    ``design_callback(phase)`` may return SimulationLab._design's
    ``{equilibrium, design}`` result. With no callback the ordinary local LQR
    designer is used. Numeric arrays and Unicode metadata are stored in NPZ;
    cached values are always read with ``allow_pickle=False`` and validated.
    Returns ``(points, metadata)``. The caller owns synchronization of designer.
    """
    fingerprint, provenance = _fingerprint(designer)
    path = Path(cache_path) if cache_path is not None else None
    rebuild_reason = None
    if path is not None and path.is_file():
        try:
            return _read_cache(path, fingerprint, designer)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
            rebuild_reason = str(error)
    if design_callback is None:
        from .control import find_equilibrium, design_lqr

        def design_callback(phase):
            equilibrium = find_equilibrium(designer, phase)
            return {'equilibrium': equilibrium, 'design': design_lqr(designer, equilibrium)}

    points, rejected = [], []
    attempts = 0

    def attempt(phase):
        nonlocal attempts
        attempts += 1
        try:
            points.append(_point(designer, design_callback(float(phase)), float(phase)))
        except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
            rejected.append({'phase_rad': float(phase), 'reason': str(error)})

    initial_state = designer.capture()
    initial_surface_velocity = designer.model.geom_surfacevel.copy()
    try:
        for phase in _initial_phases():
            attempt(phase)
        # Extend to both polar modules and the opposite hemisphere. Preserve
        # the dense local points; do not redesign their nearly equal phases.
        for phase in np.arange(-3., 3.051, .1):
            if not any(abs(point['phase']-phase) < .025 for point in points):
                attempt(phase)
    finally:
        designer.model.geom_surfacevel[:] = initial_surface_velocity
        designer.restore(initial_state)

    accepted = []
    for point in points:
        if abs(point['K'][0, 4]) > 100.:
            rejected.append({'phase_rad': point['phase'],
                             'reason': 'Extreme finite-rim shaft pitch gain exceeds 100 N m/rad'})
        else:
            accepted.append(point)
    _validate_groups(accepted)
    metadata = {'fingerprint': fingerprint, 'provenance': provenance,
                'cache_version': _CACHE_VERSION, 'cache_loaded': False,
                'attempted_points': attempts, 'accepted_points': len(accepted),
                'rejected_points': rejected,
                'limits': 'Local feedback with ideal belt speed commands; global recovery is not certified.'}
    if rebuild_reason is not None:
        metadata['cache_rebuild_reason'] = rebuild_reason
    if path is not None:
        try:
            _write_cache(path, accepted, metadata)
        except OSError as error:
            metadata['cache_write_error'] = str(error)
    return accepted, metadata


class ScheduledBalanceFeedback:
    """Heading-aware, load-weighted phase scheduling with cyclic joint errors."""

    def __init__(self, plant, points):
        self.plant = plant
        self.carrier_id = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_BODY, 'carrier')
        if self.carrier_id < 0:
            raise ValueError('Scheduled balance feedback needs the carrier body')
        self.supports = {}
        for point in points:
            if abs(point['K'][0, 4]) > 100.:
                continue
            name = ('belt_' if point['regime'] == 'belt' else 'shell_')+point['side']
            self.supports.setdefault(name, []).append(point)
        _validate_groups(points)
        for name, group in list(self.supports.items()):
            group = sorted(group, key=lambda point: point['phase'])
            self.supports[name] = (np.array([point['phase'] for point in group]),
                                   np.array([point['K'] for point in group]))

    @staticmethod
    def _phase_near_interval(phase, low, high):
        # At +pi/-pi, a residual load on the outgoing hemisphere should use
        # that support's nearby endpoint, rather than jump to its other end.
        def distance(value):
            return max(low-value, 0., value-high)
        return min((phase, phase-2*np.pi, phase+2*np.pi), key=distance)

    def _gain(self, phase, loads):
        gain = np.zeros((5, 2*self.plant.model.nv))
        total = sum(loads.values())
        for name, normal_force in loads.items():
            if normal_force < .001 or name not in self.supports:
                continue
            phases, gains = self.supports[name]
            local_phase = self._phase_near_interval(phase, phases[0], phases[-1])
            index = int(np.searchsorted(phases, local_phase))
            low, high = max(0, index-1), min(len(phases)-1, index)
            fraction = np.clip((local_phase-phases[low])/max(phases[high]-phases[low], 1e-9), 0., 1.)
            gain += normal_force/total*((1-fraction)*gains[low]+fraction*gains[high])
        return gain

    def compute(self, qref, vref, equilibrium_u, ordinary_K, feedforward=None):
        """Return an unclipped five-input command plus scheduling diagnostics.

        ``ordinary_K`` supplies a well-conditioned ordinary-pose shaft row.
        The caller retains input clipping and any actuator-enabled switches.
        No reference translation, state reset, or external force is applied.
        """
        plant = self.plant
        rotation = plant.data.xmat[self.carrier_id].reshape(3, 3)
        phase = float(np.arctan2(-rotation[2, 0], rotation[2, 2]))
        loads = {}
        for contact in plant.contact_summary():
            for geom in contact['geoms']:
                if geom.startswith(('shell_', 'belt_')):
                    name = geom[:7] if geom.startswith('shell_') else geom[:6]
                    loads[name] = loads.get(name, 0.)+contact['normal_force_N']
        gain = self._gain(phase, loads) if sum(loads.values()) > .001 else np.array(ordinary_K, copy=True)
        gain[0] = ordinary_K[0]
        quat = plant.data.qpos[3:7]
        yaw = float(np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]),
                              1-2*(quat[2]**2+quat[3]**2)))
        c, s = np.cos(yaw), np.sin(yaw)
        world_to_heading = np.array([[c, s], [-s, c]])
        target = np.array(qref, copy=True)
        yaw_quat = np.array([np.cos(yaw/2), 0., 0., np.sin(yaw/2)])
        # Right-multiplication preserves a world-space acceleration lean's
        # up direction while allowing the unregulated heading to follow yaw.
        mujoco.mju_mulQuat(target[3:7], np.array(qref[3:7]), yaw_quat)
        error = plant.error(target, vref)
        error[:2] = world_to_heading@error[:2]
        error[9:11] = world_to_heading@error[9:11]
        error[6:9] = (error[6:9]+np.pi) % (2*np.pi)-np.pi
        # Both shells are axisymmetric about their drive axes. Their material
        # angles do not specify a physical balance target; even wrapped angle
        # feedback can keep unwinding them and inject persistent yaw energy.
        # Retain shaft phase and every velocity term, including yaw-rate and
        # shell-speed feedback, while leaving shell material position free.
        error[7:9] = 0.
        command = np.asarray(equilibrium_u)-gain@error
        if feedforward is not None:
            command += np.asarray(feedforward)
        return command, {'effective_phase_rad': phase, 'support_loads_N': loads,
                         'yaw_rad': yaw, 'K': gain, 'error': error}
