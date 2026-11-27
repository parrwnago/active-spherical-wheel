"""Independent I58 source-binding and nonbinding-box objective equivalence check.

No simulator/model/observer evaluation. These synthetic comparisons are not the
repository suite, physical tests, or validation of stability.
"""
from pathlib import Path
import hashlib,json
import numpy as np
from spherical_wheel.balance_priority_control import BalancePriorityAllocator
from spherical_wheel.handoff_control import BoundedHandoffAllocator,HandoffSettings

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
registration=json.loads((HERE/'source_review_registered.json').read_text())
for name,digest in registration['source_sha256'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
caps=np.array([1.5,.6,.6,.3,.3]);settings=HandoffSettings(yaw_weight=0,momentum_weight=0);rng=np.random.default_rng(5803)
maximum=0.;recoveries=0
for rank in (0,1,2):
 for case in range(10):
  G=rng.normal(size=(9,5))
  if rank==0:G[3:5]=0
  if rank==1:G[4]=-2*G[3]
  args=dict(native_requested=caps*rng.uniform(-.02,.02,5),input_bounds=caps,previous_applied=caps*rng.uniform(-.02,.02,5),dt_s=.005,qvel=np.zeros(9),response=G,fork_rotation_world=np.eye(3),shell_axis_world=np.array([0.,0.,1.]),shell_axial_inertia=np.ones(2)*.0013,support_loads_N=np.array([0.,18.,0.,0.]),reference_shell_rates=np.zeros(2),readiness=rng.uniform(0,1,5))
  old=BoundedHandoffAllocator(settings).solve(**args);new=BalancePriorityAllocator(settings).solve(**args)
  assert not np.any(old.diagnostics['active_slew'])
  difference=float(np.max(abs(new.command-old.command)));maximum=max(maximum,difference)
  assert difference<1e-6,(rank,case,difference)
  assert np.max(abs(G[3:5]@(new.command-args['native_requested'])))<=1e-8
  recoveries+=int(new.diagnostics['native_fallback_used'])
for name,digest in registration['source_sha256'].items():assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
result=dict(status='pass',synthetic_nonbinding_rate_box_comparisons=30,maximum_command_difference=maximum,P_fallbacks=recoveries,physics_executions=0,scope='Independent synthetic objective equivalence where oldA rateboxisnonbinding; notfullrepoorphysical validation. OtherPtestsareseparateoverlappingtestscopes.')
(HERE/'check_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
