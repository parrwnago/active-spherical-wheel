"""Independent source-bound algebra review; no physics and no I56 outcomes."""
import hashlib, importlib.util, json, sys
from pathlib import Path
from dataclasses import replace
from datetime import datetime, timezone
import numpy as np
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
registration=json.loads((HERE/'registered_before_review.json').read_text())
for name,digest in registration['input_sha256'].items():
    assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    assert (ROOT/name).read_bytes()==(HERE/'source_snapshot'/name).read_bytes(),name
path=HERE/'source_snapshot/src/spherical_wheel/handoff_control.py'
spec=importlib.util.spec_from_file_location('i56_current_bound_review',path)
m=importlib.util.module_from_spec(spec);sys.modules[spec.name]=m;spec.loader.exec_module(m)
main=json.loads((HERE/'source_snapshot/results/constrained_handoff_20261126/protocol.json').read_text())
abl=json.loads((HERE/'source_snapshot/results/constrained_handoff_20261126/ablation/protocol.json').read_text())
settings=main['candidate_settings']['allocator']; asettings=abl['candidate_settings']['allocator']
assert {k for k in settings if settings[k]!=asettings[k]}=={'yaw_weight','momentum_weight'}
assert asettings['yaw_weight']==asettings['momentum_weight']==0
execution=json.loads((HERE/'source_snapshot/results/constrained_handoff_20261126/execution_registration.json').read_text())
# Four focused deterministic synthetic probes, distinct from all prior checks.
caps=np.array([3.,3.,3.,.3,.3]);v=np.zeros(9);R=np.eye(3);axis=np.array([0.,0.,1.]);inertia=np.ones(2)
loads=np.array([10.,10.,0.,0.]);refs=np.zeros(2)
def solve(G,native=None,previous=None,velocity=None,loads_=None,settings_=None):
    return m.BoundedHandoffAllocator(settings_ or m.HandoffSettings()).solve(
        np.zeros(5) if native is None else native,caps,np.zeros(5) if previous is None else previous,.005,
        v if velocity is None else velocity,G,R,axis,inertia,loads if loads_ is None else loads_,refs)
G=np.zeros((9,5));G[3,0]=1;G[4,1]=1;G[5,2]=1
velocity=v.copy();velocity[5]=1
out=solve(G,velocity=velocity,settings_=m.HandoffSettings(travel_weight=0,momentum_weight=0))
assert out.command[2]<-.01
np.testing.assert_allclose(G[3:5]@out.command,0,atol=1e-10)
bodyxy=dict(command=out.command.tolist(),locked_response=(G[3:5]@out.command).tolist(),yaw_delta=float(G[5]@out.command))
native=np.array([1.5,0,0,0,0.]);out=solve(G,native=native)
assert np.linalg.norm(out.diagnostics['first_stage_balance_delta'])>1
assert np.max(abs(out.diagnostics['secondary_balance_delta']))<1e-8
forced=dict(first_stage=out.diagnostics['first_stage_balance_delta'].tolist(),secondary=out.diagnostics['secondary_balance_delta'].tolist(),command=out.command.tolist())
previous=np.array([1.2,.4,-.4,.2,-.2]);out=solve(np.zeros((9,5)),previous=previous)
np.testing.assert_allclose(out.diagnostics['lower_command'],np.maximum(-caps,previous-np.array([.3,.3,.3,.03,.03])),atol=1e-12)
assert np.max(abs(out.command-previous)-np.array([.3,.3,.3,.03,.03]))<1e-12
previous_probe=dict(previous=previous.tolist(),lower=out.diagnostics['lower_command'].tolist(),command=out.command.tolist())
zero=solve(G,velocity=velocity,loads_=np.zeros(4));disabled=solve(G,velocity=velocity,loads_=np.zeros(4),settings_=m.HandoffSettings(yaw_weight=0,momentum_weight=0))
np.testing.assert_array_equal(zero.command,disabled.command)
assert not zero.diagnostics['auxiliary_damping_enabled']
report=dict(completed_utc=datetime.now(timezone.utc).isoformat(),status='pass',source_sha256=registration['input_sha256'],physics_executions=0,focused_synthetic_probes=4,probes=dict(body_transverse_only=bodyxy,unavoidable_primary_slew_loss=forced,actual_previous_first_call=previous_probe,no_support_auxiliary_disabled=True),protocol_ablation_selective=True,scope='Four deterministic algebra probes; not full repository tests or physical balance validation.')
(HERE/'check_result.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k] for k in ('status','focused_synthetic_probes','physics_executions','protocol_ablation_selective')}))
