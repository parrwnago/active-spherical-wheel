"""Pure fake-bridge instrumentation contracts; no simulator/model imports."""
from pathlib import Path
from types import SimpleNamespace,ModuleType
import hashlib,json,ast
import numpy as np
import pytest
ROOT=Path.cwd();SOURCE=ROOT/'results/causal_replay_20261126/substep_trace.py'
helper=ModuleType('_i55_substep_trace_checks');helper.__file__=str(SOURCE)
exec(compile(SOURCE.read_text(),str(SOURCE),'exec'),helper.__dict__)
class Sim:
 def __init__(self):
  self.clock=1000.;self.iteration=10;self.run_calls=[];self.commands=[];self.events=[];self.state_calls=0;self.contact_calls=0;self.fail_at=None;self.reset_at=None;self.mutate=None
 def command(self,torque,belt,wrench,friction):
  self.events.append('command');self.commands.append((torque.copy(),belt.copy(),wrench.copy(),friction))
 def run(self,n):
  self.events.append('run'+str(n));self.run_calls.append(n);self.clock+=n*.001;self.iteration+=n
  if self.mutate:self.mutate()
  if self.fail_at==len(self.run_calls):raise RuntimeError('preserved fake failure')
 def state(self):
  self.events.append('state');self.state_calls+=1;s=np.zeros(24);s[0]=self.clock;s[1]=self.iteration;s[2:5]=[self.iteration*.001,0,.12];s[5]=1;s[21]=.001;s[22]=int(self.reset_at is not None and len(self.run_calls)>=self.reset_at);return s
 def contacts(self):
  self.events.append('contacts');self.contact_calls+=1;n=self.iteration%3;rows=np.zeros((n,16))
  for k,row in enumerate(rows):row[0]=(self.iteration+k)%7;row[1]=self.iteration;row[6]=1;row[9]=k+1;row[10]=.001;row[11]=.7;row[12]=k%2
  return rows
class Plant:
 def __init__(self,time=31.4):
  self.sim=Sim();self.data=SimpleNamespace(time=time);self.dt=.005;self.substeps=5;self.last_input=np.zeros(5);self.last_disturbance=np.zeros(6);self.ground={'friction':.61};self.nominal_friction=.7;self.pull_calls=0;self.forward_calls=0;self.received=[];self.result=object();self.batch=5
 def step(self,u,disturbance=None):
  self.received.append((u,disturbance));self.last_input=np.clip(u,-1,1);self.last_disturbance=np.zeros(6)if disturbance is None else np.asarray(disturbance).copy();self.sim.command(self.last_input[:3],self.last_input[3:],self.last_disturbance,self.ground['friction']);self.sim.run(self.batch);self.data.time+=self.dt;self.pull_calls+=1;self.forward_calls+=1;return self.result

def test_split_preserves_command_hold_order_and_native_step_result():
 p=Plant();t=helper.SubstepTrace(p);u=np.array([2.,-.4,.3,.2,-.2]);d=np.arange(6.)
 original_step=p.step;original_run=p.sim.run
 with t.instrument():assert p.step(u,d)is p.result
 assert p.step==original_step and p.sim.run==original_run
 assert p.sim.run_calls==[1]*5 and len(p.sim.commands)==1
 assert p.pull_calls==p.forward_calls==1 and p.received[0][0]is u and p.received[0][1]is d
 assert p.sim.events[:3]==['command','state','run1']
 a=t.arrays();assert a['bridge_state'].shape==(5,24);assert np.array_equal(a['held_input'],np.tile(np.clip(u,-1,1),(5,1)))
 assert np.array_equal(a['disturbance_world'],np.tile(d,(5,1)))and np.all(a['friction']==.61)
 assert np.array_equal(a['substep_index'],np.arange(1,6))and np.all(a['hold_completed'])
 np.testing.assert_allclose(a['bridge_state'][1:],a['bridge_state_before'][1:]+np.tile(np.r_[.001,1,.001,np.zeros(21)],(4,1)),atol=1e-10)

def test_outside_window_and_direct_runs_remain_batched():
 p=Plant(time=31.395);t=helper.SubstepTrace(p,start_s=31.4,end_s=31.405)
 with t.instrument():
  p.sim.run(7);p.step(np.zeros(5));p.step(np.zeros(5));p.step(np.zeros(5))
 assert p.sim.run_calls==[7,5,1,1,1,1,1,5]
 assert len(p.sim.commands)==3 and p.pull_calls==3 and p.forward_calls==3
 assert len(t.arrays()['time_s'])==5 and t.completed_holds==1

def test_contact_rows_preserve_body_and_rejected_records_and_copies():
 p=Plant();t=helper.SubstepTrace(p)
 with t.instrument():p.step(np.zeros(5))
 a=t.arrays();assert a['contact_offsets'].shape==(6,);assert a['contact_offsets'][-1]==len(a['contact_rows'])
 assert 6 in a['contact_tags']and np.any(~a['contact_admitted'])
 before=t.arrays()['bridge_state'].copy();a['bridge_state'][:]=0;assert np.array_equal(t.arrays()['bridge_state'],before)
 a['held_input'][:]=9;assert np.max(t.arrays()['held_input'])==0

def test_save_npz_is_pickle_free_and_refuses_overwrite(tmp_path):
 p=Plant();t=helper.SubstepTrace(p)
 with t.instrument():p.step(np.zeros(5))
 output=tmp_path/'new.npz';report=t.save(output);before=output.read_bytes()
 with np.load(output,allow_pickle=False)as z:
  assert z['contact_rows'].shape[1]==16 and z['bridge_state'].shape[1]==24
  metadata=json.loads(str(z['metadata_json']));assert metadata['completed_holds']==1 and metadata['failed_holds']==0
  assert metadata['parity_status'].startswith('Caller must verify')
 with pytest.raises(FileExistsError):t.save(output)
 assert output.read_bytes()==before and report['substep_rows']==5

def test_empty_recording_has_fixed_shapes_without_queries():
 p=Plant(time=0);t=helper.SubstepTrace(p)
 with t.instrument():p.step(np.zeros(5))
 a=t.arrays();assert a['bridge_state'].shape==(0,24)and a['contact_rows'].shape==(0,16)and a['contact_offsets'].tolist()==[0]
 assert p.sim.run_calls==[5]and p.sim.state_calls==p.sim.contact_calls==0

def test_partial_physics_failure_preserves_rows_and_restores_hooks(tmp_path):
 p=Plant();p.sim.fail_at=2;t=helper.SubstepTrace(p);original_step=p.step;original_run=p.sim.run
 with pytest.raises(RuntimeError,match='preserved fake'):
  with t.instrument():p.step(np.zeros(5))
 assert p.step==original_step and p.sim.run==original_run
 a=t.arrays();assert len(a['time_s'])==1 and not a['hold_completed'].any()and t.failed_holds==1
 assert len(p.sim.commands)==1 and p.sim.run_calls==[1,1]
 assert t.save(tmp_path/'failed.npz')['metadata']['failed_holds']==1

def test_wrong_native_batch_is_rejected_before_extra_physics():
 p=Plant();p.batch=4;t=helper.SubstepTrace(p)
 with pytest.raises(AssertionError,match='run'):
  with t.instrument():p.step(np.zeros(5))
 assert p.sim.run_calls==[]and len(p.sim.commands)==1 and t.failed_holds==1

def test_reset_or_hold_mutation_is_rejected():
 p=Plant();p.sim.reset_at=2;t=helper.SubstepTrace(p)
 with pytest.raises(AssertionError,match='reset'):
  with t.instrument():p.step(np.zeros(5))
 assert t.failed_holds==1 and len(t.arrays()['time_s'])==1
 q=Plant();q.sim.mutate=lambda:q.last_input.__setitem__(0,.2);r=helper.SubstepTrace(q)
 with pytest.raises(AssertionError,match='held input'):
  with r.instrument():q.step(np.zeros(5))
 assert len(q.sim.commands)==1 and r.failed_holds==1

def test_nested_install_is_rejected_without_losing_outer_hook():
 p=Plant();t=helper.SubstepTrace(p)
 with t.instrument():
  with pytest.raises(RuntimeError):
   with t.instrument():pass
  p.step(np.zeros(5))
 assert p.sim.run_calls==[1]*5

@pytest.mark.parametrize('start,end,dt',[(np.nan,32.4,.001),(32.4,31.4,.001),(31.4,32.4,0),(31.4,32.4,.002)])
def test_invalid_window_or_cadence_refused(start,end,dt):
 with pytest.raises(ValueError):helper.SubstepTrace(Plant(),start_s=start,end_s=end,physics_dt_s=dt)

def test_schema_equals_native_literal_without_importing_simulator():
 tree=ast.parse((ROOT/'src/spherical_wheel/gazebo/bridge.py').read_text());value=None
 for node in tree.body:
  if isinstance(node,ast.Assign)and any(isinstance(t,ast.Name)and t.id=='STATE_FIELDS'for t in node.targets):value=ast.literal_eval(node.value)
 assert value==helper.STATE_FIELDS and len(helper.CONTACT_FIELDS)==16
