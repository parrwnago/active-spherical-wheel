"""I76 contracts: synthetic states/planned references only, no new outcome reads."""
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import ast,copy,importlib.util,json,tempfile,unittest
import numpy as np
import metrics,evaluate

BASE=Path(__file__).resolve().parents[1]
FULL_CONFIG=json.loads(Path(__file__).with_name('protocol_metrics_fragment.json').read_text())
CONFIG=metrics.case_config(FULL_CONFIG,'straight')


def module(path,name):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m);return m


def fixture():
    t=np.arange(9001)*.005;n=len(t);q=np.zeros((n,10));q[:,2]=1.;q[:,3]=1.
    p=dict(time_s=t,qpos=q,qvel=np.zeros((n,9)),reference_xy=np.zeros((n,2)),
        reference_velocity=np.zeros((n,2)),reference_acceleration=np.zeros((n,2)),
        reference_quat=np.broadcast_to([1.,0.,0.,0.],(n,4)).copy(),requested=np.zeros((n,5)),
        applied=np.zeros((n,5)),previous_applied=np.zeros((n,5)),
        caps=np.broadcast_to([1.5,.6,.6,.3,.3],(n,5)).copy(),reference_qvel=np.zeros((n,9)),
        rolling_feedforward=np.zeros((n,5)),observer_update_count=np.ones(n,int),
        native_feedback_verified=np.ones(n,bool),prior_matches_physical=np.ones(n,bool),
        no_truth_read=np.ones(n,bool),command_held_after_row=np.arange(n)<n-1,
        sync_weights=np.zeros((n,5)),transition_delta=np.zeros((n,5)),
        heading_command=np.zeros((n,5)),heading_eligible=np.zeros(n,bool),
        helper_matches=np.ones(n,bool),vertical_lever=np.ones(n),free_shell_mask=np.ones((n,2),bool),
        rate_valid_logged=np.ones(n,bool),goal_xy=np.zeros(2),path_xy=np.array([[0.,0.],[.01,0.]]))
    for key in ('raw_heading','raw_rate','raw_acceleration','command_heading','command_rate','command_acceleration',
                'raw_error_logged','command_error_logged','initial_latch','actual_helper_target',
                'actual_helper_rate_reference','actual_helper_rate_measurement','euler_rate_logged','world_z_rate_logged'):
        p[key]=np.zeros(n)
    return p


def result(p=None):
    p=fixture() if p is None else p
    with patch.object(metrics,'polyline_distance',lambda xy,path:np.zeros(len(xy))):
        return metrics.case_metrics(p,dict(completed=True,failure=None),CONFIG)


def nominal_groups():
    n,_=result();n['heading']['error_rms_rad']=1.;n['heading']['raw_euler_rate_error_rms_rad_s']=1.
    t=copy.deepcopy(n);t['heading']['error_rms_rad']=.7;t['heading']['raw_euler_rate_error_rms_rad_s']=.7
    t['activity']['passed']=True
    return {r+'_'+str(f):dict(N=copy.deepcopy(n),T=copy.deepcopy(t)) for r in ('straight','arc','eight') for f in (-1.,.3,.9)}


def quat(roll,pitch,yaw):
    a,b,c=np.array([roll,pitch,yaw])/2.;ca,cb,cc=np.cos([a,b,c]);sa,sb,sc=np.sin([a,b,c])
    return np.array([ca*cb*cc+sa*sb*sc,sa*cb*cc-ca*sb*sc,ca*sb*cc+sa*cb*sc,ca*cb*sc-sa*sb*cc])


class TangentMetricContracts(unittest.TestCase):
    def test_exact_Euler_rate_matches_independent_finite_rotation(self):
        R=metrics.rotation(quat(.35,-.2,1.1)[None])[0];omega=np.array([.3,-.4,.5])
        rate,valid=metrics.euler_heading_rate(R[None],omega[None]);self.assertTrue(valid[0])
        axis=omega/np.linalg.norm(omega);x,y,z=axis;K=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
        eps=1e-6;theta=np.linalg.norm(omega)*eps
        plus=(np.eye(3)+np.sin(theta)*K+(1-np.cos(theta))*(K@K))@R
        minus=(np.eye(3)-np.sin(theta)*K+(1-np.cos(theta))*(K@K))@R
        numeric=metrics.wrapped(np.arctan2(plus[1,0],plus[0,0])-np.arctan2(minus[1,0],minus[0,0]))/(2*eps)
        self.assertAlmostEqual(rate[0],numeric,places=8);self.assertGreater(abs(rate[0]-omega[2]),.01)

    def test_near_vertical_projection_is_invalid_without_fabricated_rate(self):
        R=metrics.rotation(quat(0.,np.pi/2,0.)[None]);rate,valid=metrics.euler_heading_rate(R,np.zeros((1,3)))
        self.assertFalse(valid[0]);self.assertTrue(np.isnan(rate[0]))

    def test_independent_raw_tangent_matches_real_reference_program(self):
        factory=module(BASE/'runtime/route.py','i76_test_route').make_route
        heading=module(BASE/'runtime/heading_reference.py','i76_test_heading').PlannedTangentReference
        for route_id in ('straight','arc','eight'):
            route=factory(route_id);generator=heading(.35,.5,route)
            total=CONFIG['clock']['total_s'] if route_id!='eight' else 133.
            times=np.arange(round(total/.005)+1)*.005;expected=metrics.raw_reference(route,times)
            actual=[generator.sample(float(t)) for t in times]
            for key,source in [('raw_heading','raw_heading_rad'),('raw_rate','raw_heading_rate_rad_s'),('raw_acceleration','raw_heading_acceleration_rad_s2')]:
                got=np.array([row[source] for row in actual])
                if key=='raw_heading':self.assertLess(np.max(abs(metrics.wrapped(got-expected[key]))),1e-12)
                else:np.testing.assert_allclose(got,expected[key],atol=1e-12,rtol=1e-12)
            self.assertLess(np.max(abs(expected['raw_rate'])),.35);self.assertLess(np.max(abs(expected['raw_acceleration'])),.5)

    def test_tangent_derivatives_match_planned_velocity_acceleration(self):
        factory=module(BASE/'runtime/route.py','i76_test_kinematic_route').make_route
        for route_id in ('straight','arc','eight'):
            route=factory(route_id);times=3.+route.duration_s*np.array([.1,.3,.5,.7,.9]);ref=metrics.raw_reference(route,times)
            values=[route(float(t)) for t in times];vel=np.array([v[1] for v in values]);acc=np.array([v[2] for v in values])
            expected_rate=(vel[:,0]*acc[:,1]-vel[:,1]*acc[:,0])/np.sum(vel*vel,axis=1)
            np.testing.assert_allclose(metrics.wrapped(ref['raw_heading']-np.arctan2(vel[:,1],vel[:,0])),0.,atol=1e-12)
            np.testing.assert_allclose(ref['raw_rate'],expected_rate,atol=1e-12,rtol=1e-12)
            eps=1e-4;before=metrics.raw_reference(route,times-eps)['raw_rate'];after=metrics.raw_reference(route,times+eps)['raw_rate']
            np.testing.assert_allclose(ref['raw_acceleration'],(after-before)/(2*eps),atol=1e-9,rtol=1e-6)

    def test_geometric_tangent_defined_at_zero_speed_and_end_hold(self):
        factory=module(BASE/'runtime/route.py','i76_test_stop_route').make_route
        for route_id in ('straight','arc','eight'):
            route=factory(route_id);end=3.+route.duration_s
            ref=metrics.raw_reference(route,np.array([0.,3.,end,end+10.]))
            np.testing.assert_allclose(metrics.wrapped(ref['raw_heading'][:2]-np.radians(route.heading_deg)),0.,atol=1e-12)
            self.assertLess(abs(metrics.wrapped(ref['raw_heading'][2]-route.tangent_heading(1.))),1e-12)
            np.testing.assert_array_equal(ref['raw_rate'],np.zeros(4));np.testing.assert_array_equal(ref['raw_acceleration'],np.zeros(4))

    def test_independent_XY_V_A_and_dense_uniform_geometric_path(self):
        factory=module(BASE/'runtime/route.py','i76_test_geometry_route').make_route
        for route_id in ('straight','arc','eight'):
            route=factory(route_id);times=np.linspace(0.,route.total_duration_s,1001)
            expected=metrics.planned_xy_reference(route,times);actual=[route(float(t)) for t in times]
            for i,key in enumerate(('reference_xy','reference_velocity','reference_acceleration')):
                np.testing.assert_allclose(expected[key],np.array([a[i] for a in actual]),atol=1e-12,rtol=1e-12)
            path=metrics.geometric_polyline(route_id);self.assertEqual(path.shape,(4097,2))
            np.testing.assert_allclose(path[0],route(0.)[0],atol=1e-12)
            np.testing.assert_allclose(path[-1],route(route.total_duration_s)[0],atol=1e-12)
            u=np.linspace(0.,1.,4097);local=metrics.shape_derivatives(route_id,u)[0]
            np.testing.assert_allclose(path,local@route.rotation.T+route.origin,atol=1e-12)
            np.testing.assert_allclose(metrics.planned_origin(route_id),route.origin,atol=1e-12)
        np.testing.assert_array_equal(metrics.planned_origin('straight'),metrics.planned_origin('arc'))
        self.assertAlmostEqual(np.linalg.norm(metrics.planned_origin('eight')-metrics.planned_origin('straight')),.5,places=12)
        self.assertGreater(np.min(np.linalg.norm(metrics.shape_derivatives('eight',np.linspace(0.,1.,4097))[1],axis=1)),0.)

    def test_figure_eight_crossing_distance_does_not_replace_timed_error(self):
        path=metrics.geometric_polyline('eight');crossing=path[0]
        self.assertLess(metrics.polyline_distance(crossing[None],path)[0],1e-12)
        self.assertGreater(np.linalg.norm(crossing-path[1024]),.1)

    def test_route_clocks_keep_long_eight_and_final_settling_window(self):
        self.assertEqual(metrics.case_config(FULL_CONFIG,'eight')['clock']['total_s'],133.)
        self.assertEqual(metrics.case_config(FULL_CONFIG,'eight')['clock']['moving_s'],[3.,123.])
        self.assertEqual(metrics.case_config(FULL_CONFIG,'eight')['clock']['final_settling_s'],[131.,133.])
        self.assertEqual(CONFIG['clock']['total_s'],45.)
        factory=module(BASE/'runtime/route.py','i76_test_clock_route').make_route
        for route_id in ('straight','arc','eight'):
            route=factory(route_id);clock=metrics.case_config(FULL_CONFIG,route_id)['clock']
            self.assertEqual(route.start_s,clock['moving_s'][0])
            self.assertEqual(route.duration_s,clock['moving_s'][1]-clock['moving_s'][0])
            self.assertEqual(route.total_duration_s,clock['total_s'])

    def test_raw_objective_not_hidden_by_command_target_lag(self):
        p=fixture();p['raw_heading'][:]=.2
        row,_=result(p)
        self.assertAlmostEqual(row['heading']['error_rms_rad'],.2)
        self.assertEqual(row['heading']['command_error_rms_rad'],0.)
        self.assertFalse(row['heading']['reference_guards_ok'])

    def test_primary_rate_error_is_exact_Euler_not_world_z(self):
        p=fixture();q=quat(.35,-.2,.4);R=metrics.rotation(q[None])[0];omega=np.array([.3,-.4,.5])
        rate,_=metrics.euler_heading_rate(R[None],omega[None])
        p['qpos'][:,3:7]=q;p['qvel'][:,3:6]=R.T@omega;p['raw_heading'][:]=.4;p['command_heading'][:]=.4
        p['raw_rate'][:]=rate[0];p['command_rate'][:]=rate[0]
        row,s=result(p)
        self.assertLess(row['heading']['raw_euler_rate_error_rms_rad_s'],1e-12)
        np.testing.assert_allclose(s['world_z_angular_speed_rad_s'],.5,atol=1e-12)
        self.assertGreater(abs(rate[0]-.5),.01)

    def test_initial_alignment_allows_small_lean_composition_offset(self):
        p=fixture();p['raw_heading'][:]=np.radians(.5);p['command_heading'][:]=p['raw_heading']
        row,_=result(p);self.assertTrue(row['heading']['initial_alignment_ok'])
        p['raw_heading'][:]=np.radians(1.1);row,_=result(p);self.assertFalse(row['heading']['initial_alignment_ok'])

    def test_world_rotation_integral_remains_physical_diagnostic(self):
        p=fixture();p['qpos'][:,3:7]=quat(np.pi/3,0.,0.);p['qvel'][:,5]=2.
        row,s=result(p);self.assertAlmostEqual(row['heading']['late_physical_absolute_rotation_rad'],16.,places=10)
        self.assertAlmostEqual(row['motion']['late_heading_absolute_travel_rad'],0.,places=12)

    def test_terminal_command_unheld_but_terminal_physical_state_counts(self):
        p=fixture();p['heading_command'][-1,1]=.5;p['heading_eligible'][-1]=True
        p['requested'][-1]=100.;p['applied'][-1]=p['caps'][-1];p['qvel'][-1,5]=.2
        row,_=result(p);self.assertEqual(row['activity']['seconds'],0.);self.assertEqual(row['commands']['clipped_held_fraction'],0.)
        self.assertFalse(row['settling']['passed'])

    def test_eligibility_is_not_helper_activity(self):
        p=fixture();p['heading_eligible'][:]=True;row,_=result(p)
        self.assertAlmostEqual(row['activity']['eligible_held_seconds'],45.);self.assertEqual(row['activity']['seconds'],0.)

    def test_actual_helper_activity_counts_held_intervals(self):
        p=fixture();p['heading_command'][600:620,1]=1e-4;p['heading_eligible'][600:620]=True;row,_=result(p)
        self.assertAlmostEqual(row['activity']['seconds'],.1,places=12);self.assertTrue(row['activity']['passed'])

    def test_four_condition_settling_separate_from_correct_pointing(self):
        p=fixture();p['raw_heading'][:]=np.pi/4;p['command_heading'][:]=np.pi/4;row,_=result(p)
        self.assertTrue(row['settling']['passed']);self.assertFalse(row['settling']['with_heading_passed'])

    def test_early_stop_uses_frozen_full_goal_and_path(self):
        p=fixture();p={k:(v[:201].copy() if isinstance(v,np.ndarray) and len(v)==9001 else v) for k,v in p.items()}
        p['goal_xy']=np.array([1.,0.]);p['path_xy']=np.array([[0.,0.],[1.,0.]])
        p['command_held_after_row']=np.arange(201)<200;row,s=result(p)
        self.assertAlmostEqual(s['goal_distance_m'][-1],1.);self.assertFalse(row['completed']);self.assertFalse(row['settling']['final_window_complete'])

    def test_50ms_acceleration_uses_vertical_world_velocity(self):
        p=fixture();p['qvel'][:,2]=p['time_s']*3.;row,_=result(p)
        self.assertAlmostEqual(row['motion']['full_50ms_acceleration_peak_m_s2'],3.,places=9)

    def test_clipping_magnitude_retains_large_requested_demands(self):
        p=fixture();p['requested'][:,0]=3.;p['requested'][:,3]=.6
        p['applied']=np.clip(p['requested'],-p['caps'],p['caps']);p['previous_applied'][1:]=p['applied'][:-1]
        row,_=result(p);self.assertAlmostEqual(row['commands']['normalized_clipping_excess']['rms'],np.sqrt(2.))
        self.assertTrue(row['applied_caps_ok']);self.assertTrue(row['actual_prior_history_exact'])

    def test_per_channel_clipping_not_hidden_by_total_fraction(self):
        n,_=result();t=copy.deepcopy(n);n['commands']['clipped_held_fraction']=t['commands']['clipped_held_fraction']=.5
        n['commands']['clipped_per_channel_fraction']=np.array([.5,0.,0.,0.,0.]);t['commands']['clipped_per_channel_fraction']=np.array([0.,.5,0.,0.,0.])
        checks={c['name']:c['passed'] for c in metrics.compare(t,n)}
        self.assertTrue(checks['held_any_clipping_fraction']);self.assertFalse(checks['held_clipping_fraction_channel1'])

    def test_both_raw_heading_and_rate_benefits_required_each_phase(self):
        groups=nominal_groups();self.assertEqual(metrics.decide(groups,True,FULL_CONFIG)['status'],'PASS_TANGENT_SCREEN')
        groups['arc_0.3']['T']['heading']['raw_euler_rate_error_rms_rad_s']=.81
        self.assertEqual(metrics.decide(groups,True,FULL_CONFIG)['status'],'FAIL_TANGENT_SCREEN')

    def test_intentional_rotation_is_not_absolute_peak_or_travel_regression(self):
        n,_=result();t=copy.deepcopy(n);t['motion']['full_world_z_peak_rad_s']=20.;t['motion']['late_world_z_peak_rad_s']=20.;t['motion']['late_heading_absolute_travel_rad']=50.
        self.assertTrue(all(c['passed'] for c in metrics.compare(t,n)))
        t['heading']['raw_euler_rate_error_peak_rad_s']=.1
        self.assertFalse(all(c['passed'] for c in metrics.compare(t,n)))

    def test_activity_and_invalid_setup_precedence(self):
        groups=nominal_groups();groups['eight_0.9']['T']['activity']['passed']=False
        self.assertEqual(metrics.decide(groups,True,FULL_CONFIG)['status'],'INCONCLUSIVE')
        groups['eight_0.9']['T']['heading']['error_rms_rad']=.9
        self.assertEqual(metrics.decide(groups,True,FULL_CONFIG)['status'],'FAIL_TANGENT_SCREEN')
        self.assertEqual(metrics.decide(groups,False,FULL_CONFIG)['status'],'INVALID_SETUP')

    def test_zero_baseline_absolute_floor_pass_is_not_relative_benefit(self):
        groups=nominal_groups()
        for arm in groups.values():
            for policy in ('N','T'):
                arm[policy]['heading']['error_rms_rad']=0.;arm[policy]['heading']['raw_euler_rate_error_rms_rad_s']=0.
        decision=metrics.decide(groups,True,FULL_CONFIG)
        primary=[x for x in decision['physical_control_checks'] if 'pass_basis' in x]
        self.assertEqual(decision['status'],'PASS_TANGENT_SCREEN');self.assertEqual(len(primary),18)
        self.assertTrue(all(x['pass_basis']=='acceptable_absolute_error' and not x['achieved_relative_benefit'] for x in primary))
        groups['straight_-1.0']['T']['heading']['error_rms_rad']=np.radians(5.1)
        self.assertEqual(metrics.decide(groups,True,FULL_CONFIG)['status'],'FAIL_TANGENT_SCREEN')

    def test_geometric_tracking_and_settling_preservation_are_gates(self):
        n,_=result();t=copy.deepcopy(n);t['tracking']['geometric_moving_m']['rms']=.002
        self.assertFalse(n['settling']['descriptive_only']);self.assertTrue(n['settling']['five_condition_descriptive_only'])
        checks={x['name']:x['passed'] for x in metrics.compare(t,n)}
        self.assertFalse(checks['moving_geometric_cross_track_RMS'])
        t=copy.deepcopy(n);t['settling']['passed']=False
        self.assertFalse(all(x['passed'] for x in metrics.compare(t,n)))
        n['settling']['passed']=False
        self.assertTrue(all(x['passed'] for x in metrics.compare(t,n)))

    def test_every_nine_pair_is_required_and_no_route_winner_substitution(self):
        groups=nominal_groups();self.assertEqual(len(metrics.decide(groups,True,FULL_CONFIG)['physical_control_checks']),207)
        groups['eight_0.9']['T']['heading']['error_rms_rad']=.81
        self.assertEqual(metrics.decide(groups,True,FULL_CONFIG)['status'],'FAIL_TANGENT_SCREEN')
        del groups['eight_0.9']
        self.assertEqual(metrics.decide(groups,True,FULL_CONFIG)['status'],'INVALID_SETUP')

    def test_geometric_distance_handles_zero_segments(self):
        got=metrics.polyline_distance(np.array([[.5,.3],[1.1,0.]]),np.array([[0.,0.],[0.,0.],[1.,0.]]))
        np.testing.assert_allclose(got,[.3,.1],atol=1e-12)


class EvaluationBoundaryContracts(unittest.TestCase):
    def test_canonical_shapes_and_alias_round_trip(self):
        p=fixture();raw={source:p[name].copy() for name,source in evaluate.ALIASES.items()}
        restored=evaluate.canonical(raw)
        for key in evaluate.ALIASES:self.assertTrue(metrics.exact_equal(restored[key],p[key]))
        raw['heading__free_shell_mask']=np.zeros((9001,3),bool)
        with self.assertRaises(ValueError):evaluate.canonical(raw)

    def test_schema_uses_executed_actual_adapter_and_actual_runner_keys(self):
        adapter=module(BASE/'adapter/tangent_heading_adapter.py','i76_test_schema_adapter')
        runner=module(BASE/'campaign_tools/run_case.py','i76_test_schema_runner')
        reference=module(BASE/'runtime/heading_reference.py','i76_test_schema_reference').PlannedTangentReference(.35,.5,module(BASE/'runtime/route.py','i76_test_schema_route').make_route('straight'))
        controller=NS(plant=NS(model=None,data=NS(xmat=np.eye(3).reshape(1,9),time=0.),fork_id=0),
            params=NS(heading_torque_limit_Nm=.15,heading_gain=0.,heading_actuation='shells',heading_reference='none',
                heading_roll_budget_Nm=0.,heading_min_lever=.12,heading_error_limit_rad=.5),
            carrier_id=0,heading_reference=0.,_held=None,_yaw_error=None,
            _update_heading_target=lambda normal,yaw:None,_heading_torque=lambda yaw:(np.zeros(5),0.))
        info=dict(yaw_rad=0.,heading_command=np.zeros(5),heading_error_rad=0.)
        obj=adapter.TangentHeadingAdapter(controller,'N',reference)
        with patch.object(adapter.mujoco,'mj_objectVelocity',lambda model,data,kind,body,out,local:out.fill(0.)):
            with obj.command_context():pass
            actual=obj.diagnostics(info)
        fixed=set();tree=ast.parse((BASE/'campaign_tools/run_case.py').read_text())
        for node in ast.walk(tree):
            if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id=='dict':
                keys={kw.arg for kw in node.keywords if kw.arg}
                if 'exact_qpos' in keys:fixed|=keys
        available=fixed|{'exact_'+key for key in runner.INFO_FIELDS}|{'heading__'+key for key in actual}|{'command_held_after_row'}
        self.assertTrue(set(evaluate.ALIASES.values()).issubset(available),set(evaluate.ALIASES.values())-available)

    def test_telemetry_reference_frame_and_raw_error_checks(self):
        p=fixture();self.assertTrue(all(c['passed'] for c in evaluate.telemetry_contracts(p,'N',CONFIG)))
        p['raw_error_logged'][:]=.1
        self.assertFalse(all(c['passed'] for c in evaluate.telemetry_contracts(p,'N',CONFIG)))
        p=fixture();p['actual_helper_rate_measurement'][:]=.1
        self.assertFalse(all(c['passed'] for c in evaluate.telemetry_contracts(p,'T',CONFIG)))

    def test_paired_initial_states_and_raw_reference_prefixes(self):
        jobs=[dict(case_id=f'{r}_{f}_{policy}',route_id=r,phase_rad=f,policy=policy) for r in ('straight','arc','eight') for f in (-1.,.3,.9) for policy in ('N','T')]
        packets={j['case_id']:fixture() for j in jobs}
        packets['arc_0.3_T']={k:(v[:301] if isinstance(v,np.ndarray) and len(v)==9001 else v) for k,v in packets['arc_0.3_T'].items()}
        # Distinct route clocks/streams are allowed; only each route's common prefix must match.
        for job in jobs:
            if job['route_id']=='arc':packets[job['case_id']]['raw_heading'][:]=.2
        self.assertTrue(all(c['passed'] for c in evaluate.pair_contracts(packets,jobs)))
        packets['arc_0.3_T']['raw_heading'][20]=.1
        self.assertFalse(all(c['passed'] for c in evaluate.pair_contracts(packets,jobs)))

    def test_source_or_input_binding_is_supported(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);p=root/'fragment.json';p.write_text('{}');digest=evaluate.sha(p)
            with patch.object(evaluate,'ROOT',root):
                evaluate.verify_frozen_binding({'input_sha256':{'fragment.json':digest}},p)
                stage=root/'evaluation/synthetic_stage01';stage.mkdir(parents=True)
                (stage/'receipt.json').write_text(json.dumps(dict(exit_code=0,source_inputs_unchanged=True,protocol_contract_omitted_until_registration=False)))
                (stage/'registered_before_tests.json').write_text(json.dumps(dict(files={'fragment.json':dict(sha256=digest,bytes=p.stat().st_size)})))
                self.assertEqual(len(evaluate.verify_contract_release(root)),2)
                p.write_text('{"changed":true}')
                with self.assertRaises(ValueError):evaluate.verify_frozen_binding({'input_sha256':{'fragment.json':digest}},p)
                with self.assertRaises(ValueError):evaluate.verify_contract_release(root)

    def test_actual_I76_protocol_and_route_initialconditions(self):
        protocol=json.loads((BASE/'protocol/protocol.json').read_text())
        self.assertEqual(protocol['iteration'],'I-76');self.assertEqual(protocol['evaluation'],FULL_CONFIG)
        self.assertEqual({(j['route_id'],j['policy'],j['phase_rad']) for j in protocol['cases']},{(r,p,f) for r in ('straight','arc','eight') for p in ('N','T') for f in (-1.,.3,.9)})
        factory=module(BASE/'runtime/route.py','i76_test_protocol_route').make_route
        self.assertTrue(all(abs(j['yaw_deg']-factory(j['route_id']).heading_deg)<1e-12 for j in protocol['cases']))
        self.assertNotIn('historical_native_paths',protocol)
        for path in (BASE/'runtime/route.py',BASE/'evaluation/protocol_metrics_fragment.json'):
            evaluate.verify_frozen_binding(protocol,path)

    def test_all_eighteen_output_seal_rejects_mutation(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);protocol_path=root/'protocol.json';protocol_path.write_text('{}');jobs=[];files={};processes=[]
            for route_id in ('straight','arc','eight'):
                for phase in (-1.,.3,.9):
                    for policy in ('N','T'):
                        case=f'{route_id}_{phase}_{policy}';jobs.append(dict(case_id=case,route_id=route_id,policy=policy,phase_rad=phase));processes.append(dict(case_id=case,returncode=0))
                        folder=root/'runs'/case;folder.mkdir(parents=True)
                        for name in ('precision.npz','summary.json','provenance.json'):
                            p=folder/name;p.write_text('synthetic');files[str(p.relative_to(root))]=dict(sha256=evaluate.sha(p),bytes=p.stat().st_size)
            seal=root/'seal.json';seal.write_text(json.dumps(dict(protocol_sha256=evaluate.sha(protocol_path),processes=processes,all_processes_successful=True,files=files)))
            evaluate.verify_release(dict(cases=jobs),protocol_path,seal,root/'runs',root)
            (root/'runs'/jobs[0]['case_id']/'summary.json').write_text('changed')
            with self.assertRaises(ValueError):evaluate.verify_release(dict(cases=jobs),protocol_path,seal,root/'runs',root)


if __name__=='__main__':unittest.main()
