"""Synthetic I73 metric/contracts only; no campaign arrays or physics."""
from pathlib import Path
from copy import deepcopy
import importlib.util,json,tempfile,unittest
import numpy as np
import metrics
import evaluate

CONFIG=json.loads(Path(__file__).with_name('protocol_metrics_fragment.json').read_text())


def fixture():
    c=deepcopy(CONFIG);c['clock'].update(total_s=.2,moving_s=[.02,.1],hold_s=[.1,.2],late_s=[.05,.2],final_settling_s=[.15,.2])
    c['settling']['sustained_s']=.05;c['reference_exposure']['minimum_held_moving_seconds_per_C_phase']=.01
    t=np.arange(41)*.005;n=len(t);q=np.zeros((n,10));q[:,3]=1.
    p=dict(time_s=t,qpos=q,qvel=np.zeros((n,9)),reference_xy=np.zeros((n,2)),reference_velocity=np.zeros((n,2)),
        reference_acceleration=np.zeros((n,2)),reference_quat=np.tile([1.,0.,0.,0.],(n,1)),
        requested=np.zeros((n,5)),applied=np.zeros((n,5)),previous_applied=np.zeros((n,5)),
        caps=np.tile([1.5,.6,.6,.3,.3],(n,1)),native_vref=np.zeros((n,9)),selected_vref=np.zeros((n,9)),
        native_ff=np.zeros((n,5)),selected_ff=np.zeros((n,5)),rolling_called=np.ones(n,bool),reference_override=np.ones(n,bool),
        observer_update_count=np.ones(n,int),native_feedback_verified=np.ones(n,bool),prior_matches_physical=np.ones(n,bool),
        no_truth_read=np.ones(n,bool),command_held_after_row=np.arange(n)<n-1,sync_weights=np.zeros((n,5)),
        transition_delta=np.zeros((n,5)),goal_xy=np.zeros(2),path_xy=np.array([[0.,0.],[1.,0.]]))
    return p,c


def row():
    p,c=fixture();r,_=metrics.case_metrics(p,dict(completed=True,failure=None),c)
    r['exposure']['passed']=True
    return r


def groups():
    g={str(x):{a:row() for a in ('N','A','C')} for x in (-1.,.3,.9)}
    g['-1.0']['N']['settling'].update(passed=False,nontrivial_failure=True)
    return g


class ControlMetricContracts(unittest.TestCase):
    def test_yaw_heading_difference_does_not_become_lean_error(self):
        p,c=fixture();p['qpos'][:,3:7]=[np.sqrt(.5),0.,0.,np.sqrt(.5)]
        r,s=metrics.case_metrics(p,dict(completed=True,failure=None),c)
        self.assertTrue(r['settling']['passed']);self.assertAlmostEqual(r['attitude']['raw_reference_peak_deg'],0.)

    def test_world_angular_velocity_uses_body_rotation(self):
        R=metrics.rotation(np.array([[np.sqrt(.5),0.,np.sqrt(.5),0.]]))[0]
        world=R@np.array([0.,0.,1.])
        np.testing.assert_allclose(world,[1.,0.,0.],atol=1e-15)

    def test_terminal_physical_state_counts_for_settling(self):
        p,c=fixture();p['qpos'][-1,0]=.021
        r,_=metrics.case_metrics(p,dict(completed=True,failure=None),c)
        self.assertFalse(r['settling']['passed']);self.assertEqual(r['settling']['final_samples'],11)

    def test_early_goal_entry_then_exit_is_not_final_settling(self):
        p,c=fixture();p['qpos'][p['time_s']>.16,0]=.1
        r,_=metrics.case_metrics(p,dict(completed=True,failure=None),c)
        self.assertAlmostEqual(r['settling']['first_sustained_entry_s'],.1)
        self.assertTrue(r['settling']['exited_after_first_entry']);self.assertFalse(r['settling']['passed'])

    def test_early_stop_keeps_true_final_goal_and_full_path(self):
        p,c=fixture();n=15
        for k,v in list(p.items()):
            if isinstance(v,np.ndarray) and v.shape and len(v)==41:p[k]=v[:n].copy()
        p['command_held_after_row']=np.arange(n)<n-1;p['qpos'][:,0]=.5
        p['reference_xy']=np.column_stack([p['time_s'],p['time_s']])
        p['goal_xy']=np.array([1.,0.]);p['path_xy']=np.array([[0.,0.],[1.,0.]])
        r,s=metrics.case_metrics(p,dict(completed=False,failure='syntheticearlystop'),c)
        self.assertTrue(np.all(s['goal_distance_m']==.5));self.assertFalse(r['completed'])
        self.assertTrue(np.allclose(s['geometric_cross_track_m'][np.isfinite(s['geometric_cross_track_m'])],0.))

    def test_final_unheld_command_is_excluded(self):
        p,c=fixture();p['requested'][-1]=1e6;p['applied'][-1]=1e6
        r,_=metrics.case_metrics(p,dict(completed=True,failure=None),c)
        self.assertTrue(r['applied_caps_ok']);self.assertTrue(r['applied_matches_clipped_request'])
        self.assertEqual(r['commands']['held_intervals'],40);self.assertEqual(r['commands']['normalized_step']['maximum'],0.)

    def test_first_command_step_uses_actual_prior(self):
        p,c=fixture();p['previous_applied'][0,0]=.75
        r,_=metrics.case_metrics(p,dict(completed=True,failure=None),c)
        self.assertEqual(r['commands']['normalized_step']['maximum'],.5)

    def test_acceleration_guard_includes_vertical_velocity(self):
        p,c=fixture();p['qvel'][:,2]=2.*p['time_s']
        r,_=metrics.case_metrics(p,dict(completed=True,failure=None),c)
        self.assertAlmostEqual(r['motion']['full_50ms_acceleration_peak_m_s2'],2.)

    def test_exposure_counts_only_used_held_moving_reference(self):
        p,c=fixture();p['selected_vref'][:,6]=.01;p['reference_override'][:]=False
        r,_=metrics.case_metrics(p,dict(completed=True,failure=None),c);self.assertEqual(r['exposure']['seconds'],0.)
        p['reference_override'][:]=True
        r,_=metrics.case_metrics(p,dict(completed=True,failure=None),c)
        self.assertAlmostEqual(r['exposure']['seconds'],.08);self.assertTrue(r['exposure']['passed'])

    def test_body_reference_mutation_fails_invariance(self):
        p,c=fixture();p['selected_vref'][:,0]=.1
        r,_=metrics.case_metrics(p,dict(completed=True,failure=None),c);self.assertFalse(r['invariant_ok'])

    def test_physical_failure_precedes_no_exposure(self):
        g=groups();g['0.3']['C']['completed']=False;g['0.3']['C']['exposure']['passed']=False
        self.assertEqual(metrics.decide(g,True)['status'],'FAIL_CONTROL_SCREEN')

    def test_secondary_A_cannot_rescue_C(self):
        g=groups();g['0.3']['C']['settling']['passed']=False
        self.assertEqual(metrics.decide(g,True)['status'],'FAIL_CONTROL_SCREEN')

    def test_no_nontrivial_native_failure_is_inconclusive(self):
        g=groups();g['-1.0']['N']['settling']['nontrivial_failure']=False
        self.assertEqual(metrics.decide(g,True)['status'],'INCONCLUSIVE')

    def test_missing_exposure_is_inconclusive_when_C_otherwise_passes(self):
        g=groups();g['0.3']['C']['exposure']['passed']=False
        self.assertEqual(metrics.decide(g,True)['status'],'INCONCLUSIVE')

    def test_all_checks_pass_is_screen_not_adoption(self):
        d=metrics.decide(groups(),True);self.assertEqual(d['status'],'PASS_CONTROL_SCREEN');self.assertTrue(d['no_automatic_adoption'])

    def test_one_phase_tracking_regression_fails(self):
        g=groups();g['0.3']['C']['tracking']['moving_timed_rms_m']=.002
        self.assertEqual(metrics.decide(g,True)['status'],'FAIL_CONTROL_SCREEN')

    def test_setup_violation_never_becomes_control_pass(self):
        self.assertEqual(metrics.decide(groups(),False)['status'],'INVALID_SETUP')

    def test_raw_reference_equality_uses_common_prefix_for_early_stop(self):
        packet,c=fixture();jobs=[];packets={}
        for phase in (-1.,.3,.9):
            for a in ('N','A','C'):
                name=str(phase)+a;jobs.append(dict(case_id=name,phase_rad=phase,policy=a));x=deepcopy(packet)
                if a=='C':
                    for k,v in list(x.items()):
                        if isinstance(v,np.ndarray) and v.shape and len(v)==41:x[k]=v[:20].copy()
                packets[name]=x
        self.assertTrue(all(r['passed'] for r in evaluate.pair_contracts(packets,jobs)))
        packets['0.3C']['qpos'][0,0]=.01
        self.assertFalse(all(r['passed'] for r in evaluate.pair_contracts(packets,jobs)))

    def test_polyline_distance_is_not_timed_tracking(self):
        xy=np.array([[.5,0.],[1.2,.3]])
        np.testing.assert_allclose(metrics.polyline_distance(xy,np.array([[0.,0.],[1.,0.]])),[0.,np.sqrt(.13)])

    def test_nonfinite_statistic_does_not_drop_bad_sample(self):
        got=metrics.stats([1.,np.nan]);self.assertFalse(got['complete']);self.assertIsNone(got['maximum'])

    def test_release_seal_binds_all_outputs_and_protocol(self):
        with tempfile.TemporaryDirectory() as tmp:
            base=Path(tmp);runs=base/'campaign/runs';jobs=[];files={};processes=[]
            for phase in (-1.,.3,.9):
                for a in ('N','A','C'):
                    name=str(phase)+a;jobs.append(dict(case_id=name,phase_rad=phase,policy=a));processes.append(dict(case_id=name,returncode=0))
                    folder=runs/name;folder.mkdir(parents=True)
                    for f in ('precision.npz','summary.json','provenance.json'):
                        path=folder/f;path.write_bytes(b'synthetic');files[str(path.relative_to(base))]=dict(sha256=evaluate.sha(path),bytes=path.stat().st_size)
            protocol=dict(cases=jobs);pp=base/'protocol.json';pp.write_text(json.dumps(protocol));sp=base/'seal.json'
            seal=dict(protocol_sha256=evaluate.sha(pp),processes=processes,all_processes_successful=True,files=files);sp.write_text(json.dumps(seal))
            evaluate.verify_release(protocol,pp,sp,runs,base)
            (runs/jobs[0]['case_id']/'precision.npz').write_bytes(b'changed')
            with self.assertRaises(ValueError):evaluate.verify_release(protocol,pp,sp,runs,base)

    def test_actual_protocol_separates_route_source_and_gate_input(self):
        protocol=json.loads((evaluate.BASE/'protocol/protocol.json').read_text())
        route=evaluate.BASE/'runtime/route.py';fragment=evaluate.FRAGMENT
        route_key=str(route.relative_to(evaluate.ROOT));fragment_key=str(fragment.relative_to(evaluate.ROOT))
        self.assertIn(route_key,protocol['source_sha256'])
        self.assertIn(fragment_key,protocol['input_sha256'])
        evaluate.verify_frozen_binding(protocol,route)
        evaluate.verify_frozen_binding(protocol,fragment)
        self.assertEqual(json.loads(fragment.read_text()),protocol['evaluation'])


if __name__=='__main__':unittest.main(verbosity=2)
