"""Synthetic reference routing/boundary contracts; no physics or saved logs."""
import copy
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from contact_reference_adapter import ContactReferenceAdapter


class FeatureModel:
    radius = .12
    patches = [dict(name=family+'_smooth', body=index+10, geom=index+20)
               for index, family in enumerate(('shell_A','shell_B','belt_A','belt_B'))]

    def __init__(self):
        self.calls = []

    def evaluate(self, *args):
        self.calls.append(tuple(np.array(value, copy=True) for value in args))
        return dict(family_index=np.arange(4), valid=np.ones(4, bool),
                    point_m=np.arange(12, dtype=float).reshape(4, 3)/100)


def fixture(policy='A', family=2, accepted=2):
    plant = NS(model=NS(geom_surfacevel=np.zeros((4, 6))),
        data=NS(qpos=np.arange(10, dtype=float), qvel=np.arange(9, dtype=float), time=1.),
        last_input=np.arange(5, dtype=float)/10)
    controller = NS(plant=plant, rolling_allocator=None, motion_reference=None,
        params=NS(rolling_allocation='least_squares'), nominal_joints=np.zeros(3),
        observer=NS(generalized=np.arange(9, dtype=float), interval_mean=np.ones(9),
                    _previous=(0., np.zeros(10)), wrench=lambda:(np.array([0.,0.,10.]),np.zeros(3))),
        locator=object())
    native_records = [dict(name='shell_A_smooth', body=10, geom=20, pos=np.zeros(3),
                          force=np.array([0.,0.,10.]), normal_force=10.)]
    calls = []
    def rolling(plant, records, velocity, damping):
        calls.append(copy.deepcopy(records))
        marker = float(records[0]['body']) if records else 0.
        return np.r_[velocity, np.zeros(4), np.full(3,marker)], np.r_[np.zeros(3), marker, -marker]
    module = NS(rolling_feedforward_records=rolling)
    features = FeatureModel()
    ranks = dict(chosen_family=np.int64(family), accepted_family=np.int64(accepted),
                 candidate_index=np.arange(4), family_scores=np.arange(4,dtype=float), score_margin=.5)
    geometry_calls = []
    def geometry(projected, points, anchor, ids, **kwargs):
        geometry_calls.append((copy.deepcopy(projected),points.copy(),anchor.copy(),ids.copy(),set(kwargs)))
        return dict(ranking=dict(fused=copy.deepcopy(ranks)), selected_candidate_id=np.arange(4),
                    family_distance_squared_m2=np.arange(4,dtype=float))
    def measure(plant, records):
        return dict(loads_N=dict(shell_A=10.,shell_B=0.,belt_A=0.,belt_B=0.),point_m=np.array([.01,.02,.03]))
    adapter=ContactReferenceAdapter(policy, features, lambda x:x, lambda x:dict(fused=copy.deepcopy(ranks)),
        geometry, lambda force,moment,radius:np.array([0.,0.,1.]), measure, module)
    return NS(adapter=adapter,controller=controller,records=native_records,calls=calls,module=module,
              features=features,ranks=ranks,geometry_calls=geometry_calls)


def execute(f):
    with f.adapter.command_context(f.controller,np.array([.1,.02])):
        vref,ff=f.module.rolling_feedforward_records(f.controller.plant,f.records,np.array([.1,.02]),0.)
    info=dict(reference_qvel=vref,rolling_feedforward=ff,
              estimated_contact_point_m=np.array([.01,.02,.03]),estimated_support_loads_N=np.array([10.,0.,0.,0.]))
    return vref,ff,f.adapter.diagnostics(info)


class AdapterContracts(unittest.TestCase):
    def test_native_is_one_unmodified_call_without_feature_eval(self):
        f=fixture('N');v,ff,row=execute(f)
        self.assertEqual(len(f.calls),1);self.assertEqual(len(f.features.calls),0)
        self.assertFalse(row['reference_override']);self.assertFalse(row['selection_evaluated'])
        np.testing.assert_array_equal(row['delta_vref'],np.zeros(9))
        np.testing.assert_array_equal(v[6:9],[10.,10.,10.])

    def test_accepted_override_uses_physical_patch_kinematics_no_force_split(self):
        f=fixture();before=copy.deepcopy(f.records);v,ff,row=execute(f)
        self.assertEqual(len(f.calls),2)
        self.assertEqual(set(f.calls[1][0]),{'name','body','geom','pos'})
        self.assertEqual(f.calls[1][0]['name'],'belt_A_smooth')
        self.assertEqual(f.calls[1][0]['body'],12);self.assertEqual(f.calls[1][0]['geom'],22)
        np.testing.assert_array_equal(f.calls[1][0]['pos'],f.features.evaluate()[ 'point_m'][2])
        np.testing.assert_array_equal(f.records[0]['force'],before[0]['force'])
        np.testing.assert_array_equal(v[6:9],[12.,12.,12.]);self.assertTrue(row['reference_override'])

    def test_unaccepted_raw_choice_keeps_native_records_and_outputs(self):
        f=fixture(accepted=-1);v,ff,row=execute(f)
        self.assertEqual(len(f.calls),1);self.assertEqual(row['raw_family'],2)
        self.assertFalse(row['reference_override']);self.assertEqual(row['chosen_candidate_id'],2)
        np.testing.assert_array_equal(row['delta_feedforward'],np.zeros(5))

    def test_geometry_receives_native_equivalent_anchor_without_native_identity(self):
        f=fixture('C');_,_,row=execute(f)
        self.assertEqual(len(f.geometry_calls),1)
        projected,points,anchor,ids,keywords=f.geometry_calls[0]
        np.testing.assert_array_equal(anchor,[.01,.02,.03]);self.assertEqual(keywords,{'rank_hypotheses'})
        self.assertNotIn('native_family',projected)
        np.testing.assert_array_equal(row['selected_candidate_id'],np.arange(4))

    def test_measurements_are_current_and_previous_input_is_actually_held(self):
        f=fixture();execute(f);q,v,previous,hint,wrench=f.features.calls[0]
        np.testing.assert_array_equal(q,f.controller.plant.data.qpos)
        np.testing.assert_array_equal(v,f.controller.plant.data.qvel)
        np.testing.assert_array_equal(previous,f.controller.plant.last_input)
        np.testing.assert_array_equal(wrench,[0.,0.,10.,0.,0.,0.])

    def test_feature_state_mutation_is_rejected(self):
        f=fixture();original=f.features.evaluate
        def bad(*args):
            result=original(*args);args[0][0]+=1.;return result
        f.features.evaluate=bad
        with self.assertRaisesRegex(AssertionError,'mutated native'):execute(f)

    def test_native_record_mutation_is_rejected(self):
        f=fixture();original=f.adapter.measure_records
        def bad(plant,records):
            records[0]['force'][0]=1.;return original(plant,records)
        f.adapter.measure_records=bad
        with self.assertRaisesRegex(AssertionError,'mutated native'):execute(f)

    def test_accepted_family_must_equal_physical_candidate_family(self):
        f=fixture(family=1,accepted=2)
        with self.assertRaisesRegex(ArithmeticError,'Accepted family'):execute(f)

    def test_stationary_bypass_never_activates_stale_override(self):
        f=fixture();execute(f)
        with f.adapter.command_context(f.controller,np.zeros(2)):pass
        row=f.adapter.diagnostics(dict(reference_qvel=np.zeros(9),rolling_feedforward=np.zeros(5),
            estimated_contact_point_m=np.ones(3),estimated_support_loads_N=np.array([10.,0.,0.,0.])))
        self.assertFalse(row['reference_override']);self.assertFalse(row['selection_evaluated'])
        self.assertFalse(row['dropout_hold']);self.assertEqual(row['branch'],'native_not_moving')
        np.testing.assert_array_equal(row['delta_vref'],np.zeros(9))

    def test_dropout_bypass_keeps_current_native_held_reference(self):
        f=fixture();held=np.arange(9,dtype=float)
        with f.adapter.command_context(f.controller,np.ones(2)):pass
        row=f.adapter.diagnostics(dict(reference_qvel=held,rolling_feedforward=np.ones(5),
            estimated_contact_point_m=np.ones(3),estimated_support_loads_N=np.zeros(4)))
        self.assertTrue(row['dropout_hold']);self.assertFalse(row['selection_evaluated'])
        np.testing.assert_array_equal(row['native_vref'],held)
        np.testing.assert_array_equal(row['delta_vref'],np.zeros(9))

    def test_hook_restores_after_exception(self):
        f=fixture();original=f.module.rolling_feedforward_records
        with self.assertRaisesRegex(RuntimeError,'fixture'):
            with f.adapter.command_context(f.controller,np.ones(2)):raise RuntimeError('fixture')
        self.assertIs(f.module.rolling_feedforward_records,original)

    def test_stateful_allocation_and_nested_hooks_rejected(self):
        f=fixture();f.controller.rolling_allocator=object()
        with self.assertRaises(ValueError):
            with f.adapter.command_context(f.controller,np.ones(2)):pass
        f.controller.rolling_allocator=None
        with f.adapter.command_context(f.controller,np.ones(2)):
            with self.assertRaises(RuntimeError):
                with f.adapter.command_context(f.controller,np.ones(2)):pass

    def test_more_than_one_rolling_call_rejected(self):
        f=fixture()
        with f.adapter.command_context(f.controller,np.ones(2)):
            f.module.rolling_feedforward_records(f.controller.plant,f.records,np.ones(2))
            with self.assertRaisesRegex(RuntimeError,'boundary'):
                f.module.rolling_feedforward_records(f.controller.plant,f.records,np.ones(2))


if __name__=='__main__':unittest.main()
