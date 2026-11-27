"""Result-owned I73 rolling-reference routing; native sensing stays native.

Only the local records argument to the stateless least-squares rolling call
changes. Selected records contain kinematics only, never force/load estimates.
The original controller continues to use its own records everywhere else.
"""
from contextlib import contextmanager
import importlib.util
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np


FAMILIES = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
POLICIES = ('N', 'A', 'C')


def _snapshot(value):
    if isinstance(value, dict):
        return tuple((key, _snapshot(value[key])) for key in sorted(value))
    if isinstance(value, (tuple, list)):
        return tuple(_snapshot(item) for item in value)
    if value is None or isinstance(value, str):
        return value
    array = np.asarray(value)
    if array.dtype.hasobject:
        raise TypeError('Unsupported mutable boundary value')
    return array.dtype.str, array.shape, array.tobytes()


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def create(controller, policy):
    """Production factory; copied frozen modules never enter the live package."""
    from spherical_wheel import offroad_control
    from spherical_wheel.contact_sensing import intrinsic_normal, measure_records
    here = Path(__file__).resolve().parent
    frozen = _load('_i73_frozen_i60_rank', here / 'frozen_i60_rank.py')
    geometry = _load('_i73_geometry_first_contact', here / 'geometry_first_contact.py')
    feature_model = frozen.ContactHypothesisFeatures(controller.plant.model)
    return ContactReferenceAdapter(policy, feature_model, frozen.project_features,
        frozen.rank_hypotheses, geometry.geometry_first_rank, intrinsic_normal,
        measure_records, offroad_control)


class ContactReferenceAdapter:
    def __init__(self, policy, feature_model, project_features, rank_hypotheses,
                 geometry_first_rank, intrinsic_normal, measure_records, rolling_module):
        if policy not in POLICIES:
            raise ValueError('Registered policies are N, A and C')
        self.policy = policy
        self.features = feature_model
        self.project_features = project_features
        self.rank_hypotheses = rank_hypotheses
        self.geometry_first_rank = geometry_first_rank
        self.intrinsic_normal = intrinsic_normal
        self.measure_records = measure_records
        self.rolling_module = rolling_module
        self.native_rolling = rolling_module.rolling_feedforward_records
        self._active = False
        self._row = None
        self._controller = None

    @contextmanager
    def command_context(self, controller, velocity):
        """Wrap exactly one native command; no observer update or plant step."""
        if self._active or self.rolling_module.rolling_feedforward_records is not self.native_rolling:
            raise RuntimeError('Nested or externally changed rolling hook')
        if controller.rolling_allocator is not None or controller.motion_reference is not None:
            raise ValueError('I73 requires stateless original least-squares rolling allocation')
        if controller.params.rolling_allocation != 'least_squares':
            raise ValueError('I73 keeps the original least-squares allocation')
        if controller.observer is None or controller.locator is None:
            raise ValueError('Observer and native support locator are required')
        moving = bool(np.linalg.norm(velocity) > 0)
        self._row = dict(policy_index=np.int8(POLICIES.index(self.policy)),
            rolling_called=False, moving=moving, selection_evaluated=False,
            reference_override=False, dropout_hold=False, branch='pending',
            native_family=np.int64(-1), native_anchor_point_m=np.full(3, np.nan),
            raw_family=np.int64(-1), accepted_family=np.int64(-1),
            chosen_candidate_id=np.int64(-1), raw_point_m=np.full(3, np.nan),
            raw_family_scores=np.full(4, np.inf), score_margin=np.nan,
            selected_candidate_id=np.full(4, -1, np.int64),
            selected_family_distance_squared_m2=np.full(4, np.nan),
            observer_wrench6=np.full(6, np.nan), intrinsic_normal_hint=np.full(3, np.nan),
            native_record_count=np.int64(0), selected_record_count=np.int64(0),
            native_vref=np.full(9, np.nan), selected_vref=np.full(9, np.nan),
            native_feedforward=np.full(5, np.nan), selected_feedforward=np.full(5, np.nan))
        self._active, self._controller = True, controller
        try:
            with patch.object(self.rolling_module, 'rolling_feedforward_records', self._rolling):
                yield self
        finally:
            self._active, self._controller = False, None

    def _boundary(self, records):
        controller = self._controller
        plant = controller.plant
        return _snapshot(dict(qpos=plant.data.qpos, qvel=plant.data.qvel,
            time_s=float(plant.data.time), previous_applied=plant.last_input,
            nominal_joints=controller.nominal_joints, records=records,
            observer_generalized=controller.observer.generalized,
            observer_interval_mean=controller.observer.interval_mean,
            observer_previous=controller.observer._previous,
            surface_velocity=getattr(plant.model, 'geom_surfacevel', None)))

    def _rolling(self, plant, records, velocity, regularization_m_s=0., **kwargs):
        if not self._active or plant is not self._controller.plant or self._row['rolling_called']:
            raise RuntimeError('Unexpected rolling-call boundary')
        if regularization_m_s != 0. or any(value is not None for value in kwargs.values()):
            raise ValueError('Unregistered stateful/regularized rolling options')
        controller = self._controller
        before = self._boundary(records)
        row = self._row
        row['rolling_called'] = True
        row['native_record_count'] = np.int64(len(records))
        native = self.native_rolling(plant, records, np.asarray(velocity, float), 0.)
        row['native_vref'], row['native_feedforward'] = (np.array(value, copy=True) for value in native)
        measurement = self.measure_records(plant, records)
        loads = np.asarray([measurement['loads_N'][family] for family in FAMILIES], float)
        maximum = float(loads.max())
        native_family = int(np.argmax(loads)) if maximum > 0 and np.sum(loads == maximum) == 1 else -1
        row['native_family'] = np.int64(native_family)
        row['native_anchor_point_m'] = np.asarray(measurement['point_m'], float).copy()
        selected = native
        row['branch'] = 'native_policy' if self.policy == 'N' else 'native_fallback_unaccepted'
        if self.policy != 'N':
            wrench = np.r_[controller.observer.wrench()]
            hint = self.intrinsic_normal(wrench[:3], wrench[3:], self.features.radius)
            if hint is None:
                hint = np.full(3, np.nan)
            row['observer_wrench6'] = wrench.copy()
            row['intrinsic_normal_hint'] = np.asarray(hint, float).copy()
            value = self.features.evaluate(plant.data.qpos, plant.data.qvel,
                plant.last_input, hint, wrench)
            projected = self.project_features(value)
            row['selection_evaluated'] = True
            ids = np.arange(len(value['family_index']), dtype=np.int64)
            if self.policy == 'A':
                rank = self.rank_hypotheses(projected)['fused']
            else:
                selection = self.geometry_first_rank(projected, value['point_m'],
                    row['native_anchor_point_m'], ids, rank_hypotheses=self.rank_hypotheses)
                rank = selection['ranking']['fused']
                row['selected_candidate_id'] = selection['selected_candidate_id'].copy()
                row['selected_family_distance_squared_m2'] = selection['family_distance_squared_m2'].copy()
            family, accepted = int(rank['chosen_family']), int(rank['accepted_family'])
            row['raw_family'], row['accepted_family'] = np.int64(family), np.int64(accepted)
            row['raw_family_scores'] = np.asarray(rank['family_scores']).copy()
            row['score_margin'] = float(rank['score_margin'])
            if family >= 0:
                index = int(rank['candidate_index'][family])
                if not (0 <= index < len(ids)) or int(value['family_index'][index]) != family or not value['valid'][index]:
                    raise ArithmeticError('Ranked candidate is not a valid physical family point')
                point = value['point_m'][index].copy()
                if not np.isfinite(point).all():
                    raise ArithmeticError('Ranked physical point is not finite')
                row['chosen_candidate_id'], row['raw_point_m'] = ids[index], point
            if accepted >= 0:
                if accepted != family:
                    raise ArithmeticError('Accepted family must equal the raw chosen family')
                patch_metadata = self.features.patches[family]
                kinematic_record = dict(name=patch_metadata['name'], body=int(patch_metadata['body']),
                    geom=int(patch_metadata['geom']), pos=row['raw_point_m'].copy())
                # Do not manufacture normal force, load shares, contact forces or friction.
                selected = self.native_rolling(plant, [kinematic_record], np.asarray(velocity, float), 0.)
                row['reference_override'], row['selected_record_count'], row['branch'] = True, np.int64(1), 'accepted_override'
        if not row['reference_override']:
            row['selected_record_count'] = np.int64(len(records))
        row['selected_vref'], row['selected_feedforward'] = (np.array(value, copy=True) for value in selected)
        if self._boundary(records) != before:
            raise AssertionError('Reference adapter mutated native records, observer, model or measured state')
        if not np.isfinite(np.r_[row['selected_vref'], row['selected_feedforward']]).all():
            raise ArithmeticError('Nonfinite rolling reference')
        return tuple(np.array(value, copy=True) for value in selected)

    def diagnostics(self, info):
        """Call after native command; bypassed branches keep original semantics."""
        if self._active or self._row is None:
            raise RuntimeError('Diagnostics require a completed native command context')
        row = self._row
        if not row['rolling_called']:
            row['dropout_hold'] = bool(row['moving'])
            row['branch'] = 'native_dropout_hold' if row['moving'] else 'native_not_moving'
            row['native_vref'] = np.asarray(info['reference_qvel']).copy()
            row['selected_vref'] = np.asarray(info['reference_qvel']).copy()
            row['native_feedforward'] = np.asarray(info['rolling_feedforward']).copy()
            row['selected_feedforward'] = np.asarray(info['rolling_feedforward']).copy()
            row['native_anchor_point_m'] = np.asarray(info['estimated_contact_point_m']).copy()
            loads = np.asarray(info['estimated_support_loads_N'])
            top = float(loads.max())
            row['native_family'] = np.int64(np.argmax(loads) if top > 0 and np.sum(loads == top) == 1 else -1)
        if not np.array_equal(np.asarray(info['reference_qvel']), row['selected_vref']):
            raise AssertionError('Unexpected downstream modification of the selected rolling reference')
        if not np.array_equal(np.asarray(info['rolling_feedforward']), row['selected_feedforward']):
            raise AssertionError('Unexpected downstream modification of selected rolling feedforward')
        row['delta_vref'] = row['selected_vref'] - row['native_vref']
        row['delta_feedforward'] = row['selected_feedforward'] - row['native_feedforward']
        return {key: np.array(value, copy=True) for key, value in row.items()}
