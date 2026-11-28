"""I77 causal-hook contracts without physical integration or case-array reads."""
import copy
from types import SimpleNamespace,MethodType
import unittest
import numpy as np
import causal_branch as branch
from spherical_wheel.offroad_control import OffroadController

class CausalHookContracts(unittest.TestCase):
 def fixture(self):
  c=SimpleNamespace(plant=SimpleNamespace(data=SimpleNamespace(qvel=np.arange(9.)/10),bounds=np.array([1.5,.6,.6,.3,.3])),ordinary=dict(equilibrium=dict(u=np.array([.01,-.02,.03,0.,0.]))),_held=dict(vref=np.arange(9.)/11))
  ref=np.arange(9.)/20;error=np.arange(18.)/100;error[15:18]=c.plant.data.qvel[6:9]-ref[6:9]
  fb=dict(error=error,reference_qvel=ref,K=np.arange(90.).reshape(5,18)/700,support_torque=np.array([.02,-.01,.02,0.,0.]))
  ff=np.array([0.,0.,0.,.1,-.1]);cmd=c.ordinary['equilibrium']['u']+ff+fb['support_torque']-fb['K']@fb['error']
  return c,cmd,fb,ff
 def test_inactive_and_identical_reference_return_original_objects_and_bytes(self):
  c,cmd,fb,ff=self.fixture()
  a,b=branch.replace_feedback_reference(c,cmd,fb,ff,None)
  self.assertIs(a,cmd);self.assertIs(b,fb)
  a,b=branch.replace_feedback_reference(c,cmd,fb,ff,fb['reference_qvel'][6:9].copy())
  self.assertIs(a,cmd);self.assertIs(b,fb)
 def test_active_override_changes_only_joint_rate_feedback_and_keeps_native_cache(self):
  c,cmd,fb,ff=self.fixture();old=copy.deepcopy(fb);held=c._held['vref'].copy();target=np.array([.2,-.1,.3])
  used,info=branch.replace_feedback_reference(c,cmd,fb,ff,target)
  np.testing.assert_array_equal(c._held['vref'],held)
  for key in fb:np.testing.assert_array_equal(fb[key],old[key])
  np.testing.assert_array_equal(info['error'][:15],old['error'][:15]);np.testing.assert_array_equal(info['reference_qvel'][:6],old['reference_qvel'][:6])
  np.testing.assert_array_equal(info['reference_qvel'][6:9],target)
  np.testing.assert_array_equal(info['error'][15:18],c.plant.data.qvel[6:9]-target)
  np.testing.assert_array_equal(used,c.ordinary['equilibrium']['u']+ff+fb['support_torque']-fb['K']@info['error'])
 def test_fixed_windows_and_stateless_same_state_command_stage_propagation(self):
  ref=np.array([7.,-2.,-8.]);held=np.array([1.,0.,-3.])
  self.assertIsNone(branch.treatment('one_pulse',4116,ref,held));np.testing.assert_array_equal(branch.treatment('one_pulse',4117,ref,held),held);self.assertIsNone(branch.treatment('one_pulse',4118,ref,held))
  self.assertIsNone(branch.treatment('cluster_cap5',4079,ref,held));np.testing.assert_array_equal(branch.treatment('cluster_cap5',4080,ref,held),[5.,-2.,-5.]);np.testing.assert_array_equal(branch.treatment('cluster_cap5',4129,ref,held),[5.,-2.,-5.]);self.assertIsNone(branch.treatment('cluster_cap5',4130,ref,held))
  c=SimpleNamespace(plant=SimpleNamespace(data=SimpleNamespace(qvel=np.array([0,0,0,0,0,0,.2,-.4,.1])),bounds=np.array([1.5,.6,.6,.3,.3])),params=SimpleNamespace(unload_fraction=.3,airborne_damping_Nms=.01),_sync=dict(blend_weights=[0.,.35,.35,1.,0.],commands=[2.,.2,-.2,.15,-.4]))
  c._fade_when_unloaded=MethodType(OffroadController._fade_when_unloaded,c)
  before=branch.snapshot(dict(sync=c._sync,qvel=c.plant.data.qvel,params=vars(c.params)))
  request,applied=branch.native_final_counterfactual(c,np.array([2.,.8,-.8,.4,-.4]),dict(heading_command=np.zeros(5),estimate=dict(load_fraction=.15),reference_qvel=np.r_[np.zeros(6),[.1,.2,-.1]]))
  np.testing.assert_allclose(request,[.9995,.298,-.296,.15,-.4],rtol=0,atol=1e-15);np.testing.assert_allclose(applied,[.9995,.298,-.296,.15,-.3],rtol=0,atol=1e-15)
  self.assertEqual(before,branch.snapshot(dict(sync=c._sync,qvel=c.plant.data.qvel,params=vars(c.params))))

if __name__=='__main__':unittest.main()
