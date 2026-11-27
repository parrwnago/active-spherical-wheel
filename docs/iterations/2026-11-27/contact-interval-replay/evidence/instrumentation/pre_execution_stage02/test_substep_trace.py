"""Fake-bridge instrumentation and coordinate contracts; no physics launch."""
from pathlib import Path
from types import SimpleNamespace
import importlib.util,tempfile,unittest,json,ast
import numpy as np

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('_i68_trace',HERE/'substep_trace.py');T=importlib.util.module_from_spec(spec);spec.loader.exec_module(T)


class Sim:
    def __init__(self):
        self.clock=1000.;self.iteration=10;self.run_calls=[];self.commands=[];self.events=[]
        self.state_reads=0;self.contact_reads=0;self.fail_at=None;self.reset_at=None;self.mutate=None
    def command(self,torque,belt,wrench,friction):
        self.events.append('command');self.commands.append((torque.copy(),belt.copy(),wrench.copy(),friction))
    def run(self,n):
        self.events.append('run'+str(n));self.run_calls.append(n);self.clock+=n*.001;self.iteration+=n
        if self.mutate:self.mutate()
        if self.fail_at==len(self.run_calls):raise RuntimeError('fake run failure')
    def state(self):
        self.events.append('state');self.state_reads+=1;s=np.zeros(24);s[0]=self.clock;s[1]=self.iteration
        s[2:5]=[self.iteration*.001,0.,.12];s[5]=1.;s[9:12]=[.1,.2,.3];s[12:15]=[.4,.5,.6]
        s[15:18]=[.1,.2,.3];s[18:21]=[.7,.8,.9];s[21]=.001
        s[22]=int(self.reset_at is not None and len(self.run_calls)>=self.reset_at);return s
    def contacts(self):
        self.events.append('contacts');self.contact_reads+=1;rows=np.zeros((3,16));rows[:,0]=[0,6,2]
        rows[:,1]=self.iteration*.002;rows[:,6]=1.;rows[:,9]=[2.,3.,99.]
        rows[:,10]=.001;rows[:,11]=.7;rows[:,12]=[1.,1.,0.];return rows


class Plant:
    def __init__(self,time=0.):
        self.sim=Sim();self.data=SimpleNamespace(time=time);self.dt=.005;self.substeps=5
        self.last_input=np.zeros(5);self.last_disturbance=np.zeros(6);self.ground={'friction':.61};self.nominal_friction=.7
        self.pull_calls=0;self.forward_calls=0;self.received=[];self.result=object();self.batch=5
    def step(self,u,disturbance=None):
        self.received.append((u,disturbance));self.last_input=np.clip(u,-1.,1.)
        self.last_disturbance=np.zeros(6) if disturbance is None else np.asarray(disturbance).copy()
        self.sim.command(self.last_input[:3],self.last_input[3:],self.last_disturbance,self.ground['friction'])
        self.sim.run(self.batch);self.data.time+=self.dt;self.pull_calls+=1;self.forward_calls+=1;return self.result


class TraceContracts(unittest.TestCase):
    def test_one_native_command_and_pull_with_five_held_solves(self):
        p=Plant();trace=T.SubstepTrace(p);u=np.array([2.,-.4,.3,.2,-.2]);d=np.arange(6.)
        original_step=p.step;original_run=p.sim.run
        with trace.instrument():self.assertIs(p.step(u,d),p.result)
        self.assertEqual(p.step,original_step);self.assertEqual(p.sim.run,original_run)
        self.assertEqual(p.sim.run_calls,[1]*5);self.assertEqual(len(p.sim.commands),1)
        self.assertEqual((p.pull_calls,p.forward_calls),(1,1));self.assertIs(p.received[0][0],u);self.assertIs(p.received[0][1],d)
        a=trace.arrays();np.testing.assert_array_equal(a['held_input'],np.tile(np.clip(u,-1,1),(5,1)))
        np.testing.assert_array_equal(a['disturbance_world'],np.tile(d,(5,1)))
        self.assertEqual(p.sim.events[:4],['command','state','contacts','run1'])

    def test_before_after_state_and_contact_snapshots_are_coherent(self):
        p=Plant();trace=T.SubstepTrace(p)
        with trace.instrument():p.step(np.zeros(5))
        a=trace.arrays();self.assertEqual((p.sim.state_reads,p.sim.contact_reads),(6,6))
        np.testing.assert_array_equal(a['bridge_state_before'][1:],a['bridge_state'][:-1])
        for i in range(1,5):
            before=a['contact_rows_before'][a['contact_offsets_before'][i]:a['contact_offsets_before'][i+1]]
            previous=a['contact_rows'][a['contact_offsets'][i-1]:a['contact_offsets'][i]]
            np.testing.assert_array_equal(before,previous)

    def test_all_body_and_rejected_records_are_preserved_without_force_filtering(self):
        p=Plant();trace=T.SubstepTrace(p)
        with trace.instrument():p.step(np.zeros(5))
        a=trace.arrays();self.assertEqual(len(a['contact_rows']),15);self.assertEqual(len(a['contact_rows_before']),15)
        self.assertIn(6,a['contact_tags']);self.assertEqual(int((~a['contact_admitted']).sum()),5)
        np.testing.assert_array_equal(a['contact_rows'][2::3,9],99.)

    def test_native_and_bridge_clocks_actual_dt_and_end_index_are_distinct(self):
        p=Plant();trace=T.SubstepTrace(p)
        with trace.instrument():p.step(np.zeros(5));p.step(np.zeros(5))
        a=trace.arrays();np.testing.assert_allclose(a['time_s'],np.arange(1,11)*.001,atol=1e-17)
        self.assertGreater(float(a['bridge_time_s'][0]),999.)
        np.testing.assert_array_equal(a['step_dt_s'],.001);np.testing.assert_array_equal(a['control_end_index'],[1]*5+[2]*5)
        np.testing.assert_allclose(a['bridge_time_after_s']-a['bridge_time_before_s'],a['step_dt_s'],atol=1e-12)

    def test_setup_and_outside_window_runs_stay_batched_without_truth_queries(self):
        p=Plant();trace=T.SubstepTrace(p,start_s=.005,end_s=.01)
        with trace.instrument():p.sim.run(7);p.step(np.zeros(5));p.step(np.zeros(5));p.step(np.zeros(5))
        self.assertEqual(p.sim.run_calls,[7,5,1,1,1,1,1,5]);self.assertEqual(trace.completed_holds,1)
        self.assertEqual(len(trace.arrays()['time_s']),5)

    def test_empty_recording_has_fixed_shapes_and_no_capture_reads(self):
        p=Plant();trace=T.SubstepTrace(p,start_s=1.,end_s=2.)
        with trace.instrument():p.step(np.zeros(5))
        a=trace.arrays();self.assertEqual(a['qpos_before'].shape,(0,10));self.assertEqual(a['qvel_after'].shape,(0,9))
        self.assertEqual(a['contact_rows'].shape,(0,16));self.assertEqual(a['contact_rows_before'].shape,(0,16))
        self.assertEqual((p.sim.state_reads,p.sim.contact_reads),(0,0))

    def test_failed_hold_preserves_partial_rows_and_restores_hooks(self):
        p=Plant();p.sim.fail_at=2;trace=T.SubstepTrace(p);old_step=p.step;old_run=p.sim.run
        with self.assertRaisesRegex(RuntimeError,'fake run failure'):
            with trace.instrument():p.step(np.zeros(5))
        self.assertEqual(p.step,old_step);self.assertEqual(p.sim.run,old_run)
        self.assertEqual(trace.failed_holds,1);a=trace.arrays();self.assertEqual(len(a['time_s']),1)
        self.assertFalse(a['hold_completed'].any())

    def test_wrong_batch_is_rejected_before_any_extra_physics(self):
        p=Plant();p.batch=4;trace=T.SubstepTrace(p)
        with self.assertRaisesRegex(AssertionError,'run'):
            with trace.instrument():p.step(np.zeros(5))
        self.assertEqual(p.sim.run_calls,[])

    def test_reset_and_mid_hold_command_mutation_are_rejected(self):
        p=Plant();p.sim.reset_at=2;trace=T.SubstepTrace(p)
        with self.assertRaisesRegex(AssertionError,'reset'):
            with trace.instrument():p.step(np.zeros(5))
        p=Plant();p.sim.mutate=lambda:p.last_input.__setitem__(0,.2);trace=T.SubstepTrace(p)
        with self.assertRaisesRegex(AssertionError,'held input'):
            with trace.instrument():p.step(np.zeros(5))

    def test_arrays_are_independent_and_save_is_pickle_free_and_nonoverwriting(self):
        p=Plant();trace=T.SubstepTrace(p)
        with trace.instrument():p.step(np.zeros(5))
        first=trace.arrays();first['contact_rows'][:]=0.;first['held_input'][:]=9.
        self.assertGreater(trace.arrays()['contact_rows'][0,9],0.);self.assertEqual(np.max(trace.arrays()['held_input']),0.)
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'trace.npz';trace.save(path);before=path.read_bytes()
            with np.load(path,allow_pickle=False) as z:
                metadata=json.loads(str(z['metadata_json']));self.assertFalse(metadata['per_contact_couple_available'])
                self.assertFalse(metadata['total_constraint_wrench_available']);self.assertEqual(z['qpos_before'].shape,(5,10))
            with self.assertRaises(FileExistsError):trace.save(path)
            self.assertEqual(path.read_bytes(),before)

    def test_qvel_world_to_body_conversion_matches_native_quaternion_matrix(self):
        import mujoco
        rng=np.random.default_rng(68);states=np.zeros((12,24));states[:,5:9]=rng.normal(size=(12,4))
        states[:,2:5]=rng.normal(size=(12,3));states[:,9:21]=rng.normal(size=(12,12));before=states.copy()
        q,v=T.bridge_state_to_qv(states)
        for i in range(len(states)):
            matrix=np.empty(9);quat=states[i,5:9]/np.linalg.norm(states[i,5:9]);mujoco.mju_quat2Mat(matrix,quat)
            np.testing.assert_allclose(v[i,3:6],matrix.reshape(3,3).T@states[i,12:15],atol=1e-14)
            # Batched and scalar norm reductions differ at roundoff; these
            # offline coordinates are bounded comparisons, not parity proof.
            np.testing.assert_allclose(q[i,3:7],quat,rtol=0.,atol=5e-16)
        np.testing.assert_array_equal(states,before)

    def test_static_tag_mapping_labels_mirror_bodies_without_gazebo_entity_claim(self):
        p=Plant();p._geom_ids=list(range(7));body_ids=np.array([3,4,2,2,3,4,1])
        p.model=SimpleNamespace(geom_bodyid=body_ids,geom=lambda i:SimpleNamespace(name=('platform' if i==6 else 'geom'+str(i))),body=lambda i:SimpleNamespace(name={1:'fork',2:'shaft',3:'shell_A',4:'shell_B'}[i]))
        trace=T.SubstepTrace(p)
        with trace.instrument():p.step(np.zeros(5))
        a=trace.arrays();self.assertEqual(a['tag_mirror_body_names'][6],'fork')
        np.testing.assert_array_equal(a['contact_mirror_body_ids'][a['contact_tags']==6],1)

    def test_nested_install_and_invalid_cadence_are_refused(self):
        p=Plant();trace=T.SubstepTrace(p)
        with trace.instrument():
            with self.assertRaises(RuntimeError):
                with trace.instrument():pass
        with self.assertRaises(ValueError):T.SubstepTrace(p,physics_dt_s=.002)
        with self.assertRaises(ValueError):T.SubstepTrace(p,start_s=44.,end_s=0.)

    def test_state_schema_matches_native_bridge_literal_without_gazebo_import(self):
        root=HERE.parents[2];tree=ast.parse((root/'src/spherical_wheel/gazebo/bridge.py').read_text());fields=None
        for node in tree.body:
            if isinstance(node,ast.Assign) and any(isinstance(target,ast.Name) and target.id=='STATE_FIELDS' for target in node.targets):fields=ast.literal_eval(node.value)
        self.assertEqual(fields,T.STATE_FIELDS);self.assertEqual(len(T.CONTACT_FIELDS),16)


if __name__=='__main__':unittest.main(verbosity=2)
