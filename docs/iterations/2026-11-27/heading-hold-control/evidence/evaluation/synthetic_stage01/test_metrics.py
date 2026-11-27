"""Focused I74 contracts; synthetic data only, never reads new campaign arrays."""
from pathlib import Path
import copy
import ast
import importlib.util
import json
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace as NS
import numpy as np
import metrics
import evaluate


CONFIG=json.loads(Path(__file__).with_name('protocol_metrics_fragment.json').read_text())


def fixture():
    t=np.arange(9001,dtype=float)*.005;n=len(t)
    q=np.zeros((n,10));q[:,2]=1.;q[:,3]=1.
    caps=np.broadcast_to([1.5,.6,.6,.3,.3],(n,5)).copy()
    return dict(time_s=t,qpos=q,qvel=np.zeros((n,9)),reference_xy=np.zeros((n,2)),
        reference_velocity=np.zeros((n,2)),reference_acceleration=np.zeros((n,2)),
        reference_quat=np.broadcast_to([1.,0.,0.,0.],(n,4)).copy(),requested=np.zeros((n,5)),
        applied=np.zeros((n,5)),previous_applied=np.zeros((n,5)),caps=caps,
        reference_qvel=np.zeros((n,9)),rolling_feedforward=np.zeros((n,5)),
        observer_update_count=np.ones(n,int),native_feedback_verified=np.ones(n,bool),
        prior_matches_physical=np.ones(n,bool),no_truth_read=np.ones(n,bool),
        command_held_after_row=np.arange(n)<n-1,sync_weights=np.zeros((n,5)),transition_delta=np.zeros((n,5)),
        heading_target=np.zeros(n),heading_command=np.zeros((n,5)),heading_eligible=np.zeros(n,bool),
        helper_matches=np.ones(n,bool),vertical_lever=np.ones(n),free_shell_mask=np.ones((n,2),bool),
        goal_xy=np.zeros(2),path_xy=np.array([[0.,0.],[.01,0.]]))


def result(p=None):
    p=fixture() if p is None else p
    # Geometry is irrelevant to heading contracts; exercise its algorithm separately.
    with patch.object(metrics,'polyline_distance',lambda xy,path:np.zeros(len(xy))):
        return metrics.case_metrics(p,dict(completed=True,failure=None),CONFIG)


def nominal_groups():
    n,_=result();n['heading']['error_rms_rad']=1.;n['heading']['late_physical_absolute_rotation_rad']=1.
    h=copy.deepcopy(n);h['heading']['error_rms_rad']=.7;h['heading']['late_physical_absolute_rotation_rad']=.7
    h['activity']['passed']=True
    return {str(f):dict(N=copy.deepcopy(n),H=copy.deepcopy(h)) for f in (-1.,.3,.9)}


class HeadingMetricContracts(unittest.TestCase):
    def test_physical_rotation_integral_is_not_Euler_heading_travel(self):
        p=fixture();angle=np.deg2rad(60.)
        p['qpos'][:,3:7]=[np.cos(angle/2),np.sin(angle/2),0.,0.]
        p['qvel'][:,5]=2.
        row,s=result(p)
        self.assertAlmostEqual(row['heading']['late_physical_absolute_rotation_rad'],16.,places=10)
        self.assertAlmostEqual(row['motion']['late_heading_absolute_travel_rad'],0.,places=12)
        np.testing.assert_allclose(s['world_z_angular_speed_rad_s'],1.,atol=1e-12)

    def test_world_rotation_uses_full_body_vector(self):
        p=fixture();a=np.pi/2;p['qpos'][:,3:7]=[np.cos(a/2),0.,np.sin(a/2),0.]
        p['qvel'][:,3]=2.
        _,s=result(p)
        np.testing.assert_allclose(s['world_z_angular_speed_rad_s'],-2.,atol=1e-12)

    def test_wrapped_heading_error_and_reference_up_have_distinct_roles(self):
        p=fixture();a=.3;p['qpos'][:,3:7]=[np.cos(a/2),0.,0.,np.sin(a/2)]
        p['heading_target'][:]=a+2*np.pi
        row,s=result(p)
        self.assertLess(row['heading']['error_rms_rad'],1e-12)
        self.assertLess(row['attitude']['raw_reference_peak_deg'],1e-6)
        np.testing.assert_allclose(s['heading_rad'],a,atol=1e-12)

    def test_trapezoid_integral_includes_both_physical_endpoints(self):
        p=fixture();p['qvel'][-1,5]=2.
        row,_=result(p)
        self.assertAlmostEqual(row['heading']['late_physical_absolute_rotation_rad'],.005,places=12)

    def test_terminal_command_is_unheld_but_terminal_state_counts(self):
        p=fixture();p['heading_command'][-1,1]=.5;p['heading_eligible'][-1]=True
        p['requested'][-1,:]=100.;p['applied'][-1,:]=p['caps'][-1]
        p['qvel'][-1,5]=.2
        row,_=result(p)
        self.assertEqual(row['activity']['seconds'],0.)
        self.assertEqual(row['commands']['clipped_held_fraction'],0.)
        self.assertFalse(row['settling']['passed'])

    def test_eligibility_is_not_actual_helper_activity(self):
        p=fixture();p['heading_eligible'][:]=True
        row,_=result(p)
        self.assertAlmostEqual(row['activity']['eligible_held_seconds'],45.)
        self.assertEqual(row['activity']['seconds'],0.)
        self.assertFalse(row['activity']['passed'])

    def test_held_helper_activity_counts_actual_intervals(self):
        p=fixture();p['heading_command'][600:620,1]=1e-4;p['heading_eligible'][600:620]=True
        row,_=result(p)
        self.assertAlmostEqual(row['activity']['seconds'],.1,places=12)
        self.assertTrue(row['activity']['passed'])

    def test_four_criterion_settling_is_distinct_from_heading_hold(self):
        p=fixture();p['heading_target'][:]=np.pi/4
        row,_=result(p)
        self.assertTrue(row['settling']['passed'])
        self.assertFalse(row['settling']['with_heading_passed'])

    def test_early_stop_keeps_frozen_goal_and_complete_path(self):
        p=fixture();p={k:(v[:201].copy() if isinstance(v,np.ndarray) and len(v)==9001 else v) for k,v in p.items()}
        p['goal_xy']=np.array([1.,0.]);p['path_xy']=np.array([[0.,0.],[1.,0.]])
        p['command_held_after_row']=np.arange(201)<200
        row,s=result(p)
        self.assertAlmostEqual(row['tracking']['final_xy_m'],0.)
        self.assertAlmostEqual(s['goal_distance_m'][-1],1.)
        self.assertFalse(row['completed'])
        self.assertFalse(row['settling']['final_window_complete'])

    def test_50ms_acceleration_uses_all_three_world_components(self):
        p=fixture();p['qvel'][:,2]=p['time_s']*3.
        row,_=result(p)
        self.assertAlmostEqual(row['motion']['full_50ms_acceleration_peak_m_s2'],3.,places=9)

    def test_clipping_excess_is_normalized_by_per_channel_caps(self):
        p=fixture();p['requested'][:,0]=3.;p['requested'][:,3]=.6
        p['applied']=np.clip(p['requested'],-p['caps'],p['caps']);p['previous_applied'][1:]=p['applied'][:-1]
        row,_=result(p)
        self.assertAlmostEqual(row['commands']['normalized_clipping_excess']['rms'],np.sqrt(2.))
        self.assertTrue(row['applied_caps_ok']);self.assertTrue(row['actual_prior_history_exact'])

    def test_each_channel_saturation_is_not_hidden_by_any_channel_fraction(self):
        n,_=result();h=copy.deepcopy(n)
        n['commands']['clipped_held_fraction']=.5;h['commands']['clipped_held_fraction']=.5
        n['commands']['clipped_per_channel_fraction']=np.array([.5,0.,0.,0.,0.])
        h['commands']['clipped_per_channel_fraction']=np.array([0.,.5,0.,0.,0.])
        checks={c['name']:c['passed'] for c in metrics.compare(h,n)}
        self.assertTrue(checks['held_any_clipping_fraction'])
        self.assertFalse(checks['held_clipping_fraction_channel1'])

    def test_each_phase_requires_both_primary_benefits(self):
        groups=nominal_groups();self.assertEqual(metrics.decide(groups,True)['status'],'PASS_HEADING_SCREEN')
        groups['0.3']['H']['heading']['late_physical_absolute_rotation_rad']=.81
        self.assertEqual(metrics.decide(groups,True)['status'],'FAIL_HEADING_SCREEN')

    def test_activity_inconclusive_and_failure_precedence(self):
        groups=nominal_groups();groups['0.9']['H']['activity']['passed']=False
        self.assertEqual(metrics.decide(groups,True)['status'],'INCONCLUSIVE')
        groups['0.9']['H']['heading']['error_rms_rad']=.9
        self.assertEqual(metrics.decide(groups,True)['status'],'FAIL_HEADING_SCREEN')
        self.assertEqual(metrics.decide(groups,False)['status'],'INVALID_SETUP')

    def test_geometric_path_distance_handles_vertices_and_zero_segments(self):
        got=metrics.polyline_distance(np.array([[.5,.3],[1.1,0.]]),np.array([[0.,0.],[0.,0.],[1.,0.]]))
        np.testing.assert_allclose(got,[.3,.1],atol=1e-12)


class EvaluationBoundaryContracts(unittest.TestCase):
    def test_schema_maps_runner_heading_and_original_fields(self):
        p=fixture();raw={source:p[name].copy() for name,source in evaluate.ALIASES.items()}
        restored=evaluate.canonical(raw)
        for key in evaluate.ALIASES:self.assertTrue(metrics.exact_equal(restored[key],p[key]))
        raw['heading__free_shell_mask']=np.zeros((9001,3),bool)
        with self.assertRaises(ValueError):evaluate.canonical(raw)

    def test_schema_matches_actual_runner_and_executed_telemetry_keys(self):
        base=Path(__file__).resolve().parents[1]
        def module(path,name):
            spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec)
            spec.loader.exec_module(m);return m
        telemetry=module(base/'adapter/heading_telemetry.py','i74_schema_telemetry')
        runner=module(base/'campaign_tools/run_case.py','i74_schema_runner')
        controller=NS(plant=NS(model=None,data=NS(xmat=np.eye(3).reshape(1,9)),fork_id=0),
            params=NS(heading_torque_limit_Nm=.15,heading_gain=0.,heading_actuation='shells',
                      heading_reference='none',heading_roll_budget_Nm=0.,heading_min_lever=.12),
            carrier_id=0,heading_reference=0.,_held=None,_yaw_error=None)
        info=dict(yaw_rad=0.,heading_command=np.zeros(5),heading_error_rad=0.)
        with patch.object(telemetry.mujoco,'mj_objectVelocity',lambda model,data,kind,body,out,local:out.fill(0.)):
            actual=telemetry.heading_diagnostics(controller,info)
        tree=ast.parse((base/'campaign_tools/run_case.py').read_text())
        fixed=set()
        for node in ast.walk(tree):
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id=='dict':
                names={kw.arg for kw in node.keywords if kw.arg}
                if 'exact_qpos' in names:fixed|=names
        available=fixed|{'exact_'+name for name in runner.INFO_FIELDS}|{'heading__'+name for name in actual}|{'command_held_after_row'}
        self.assertTrue(set(evaluate.ALIASES.values()).issubset(available),set(evaluate.ALIASES.values())-available)
        self.assertEqual(np.asarray(actual['free_shell_mask']).shape,(2,))

    def test_common_prefix_and_phase_initial_state_contracts(self):
        jobs=[dict(case_id=f'{f}_{policy}',phase_rad=f,policy=policy) for f in (-1.,.3,.9) for policy in ('N','H')]
        packets={j['case_id']:fixture() for j in jobs}
        packets['0.3_H']={k:(v[:301] if isinstance(v,np.ndarray) and len(v)==9001 else v) for k,v in packets['0.3_H'].items()}
        self.assertTrue(all(c['passed'] for c in evaluate.pair_contracts(packets,jobs)))
        packets['0.3_H']['reference_xy'][20,0]=.1
        self.assertFalse(all(c['passed'] for c in evaluate.pair_contracts(packets,jobs)))

    def test_baseline_common_numeric_arrays_are_exact_without_fabricated_fields(self):
        p=fixture();raw={source:p[name].copy() for name,source in evaluate.ALIASES.items()}
        historical={k:v.copy() for k,v in raw.items() if not k.startswith('heading__')}
        historical['contact_reference__policy_index']=np.zeros(9001,int)
        self.assertTrue(evaluate.baseline_replay(raw,historical)['passed'])
        historical['exact_qvel'][2,0]=1e-15
        replay=evaluate.baseline_replay(raw,historical)
        self.assertFalse(replay['passed']);self.assertIn('exact_qvel',replay['changed_fields'])

    def test_frozen_fragment_may_be_an_input_binding(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);p=root/'fragment.json';p.write_text('{}')
            digest=evaluate.sha(p)
            with patch.object(evaluate,'ROOT',root):
                evaluate.verify_frozen_binding({'source_sha256':{},'input_sha256':{'fragment.json':digest}},p)
                p.write_text('{"changed":true}')
                with self.assertRaises(ValueError):evaluate.verify_frozen_binding({'input_sha256':{'fragment.json':digest}},p)

    def test_all_six_seal_rejects_mutation_and_missing_case(self):
        with tempfile.TemporaryDirectory() as temp:
            base=Path(temp);jobs=[dict(case_id=f'{f}_{p}',phase_rad=f,policy=p) for f in (-1.,.3,.9) for p in ('N','H')]
            protocol=dict(cases=jobs);protocol_path=base/'protocol.json';protocol_path.write_text(json.dumps(protocol))
            files={}
            for job in jobs:
                d=base/'runs'/job['case_id'];d.mkdir(parents=True)
                for name in ('precision.npz','summary.json','provenance.json'):
                    p=d/name;p.write_text('fixture');files[str(p.relative_to(base))]=dict(sha256=evaluate.sha(p),bytes=p.stat().st_size)
            seal=dict(protocol_sha256=evaluate.sha(protocol_path),all_processes_successful=True,
                      processes=[dict(case_id=j['case_id'],returncode=0) for j in jobs],files=files)
            path=base/'seal.json';path.write_text(json.dumps(seal))
            evaluate.verify_release(protocol,protocol_path,path,base/'runs',base)
            (base/'runs'/jobs[0]['case_id']/'summary.json').write_text('changed')
            with self.assertRaises(ValueError):evaluate.verify_release(protocol,protocol_path,path,base/'runs',base)


if __name__=='__main__':unittest.main()
