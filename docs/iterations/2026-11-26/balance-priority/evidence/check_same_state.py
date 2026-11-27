"""Static I57 anchor regression: compare A/P at the identical saved H state."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import shutil
import mujoco
import numpy as np
from spherical_wheel.handoff_control import BoundedHandoffAllocator, HandoffSettings
from spherical_wheel.balance_priority_control import BalancePriorityAllocator
from spherical_wheel.handoff_model import MeasuredContactVelocityModel

ROOT = Path.cwd()
BASE = ROOT/'results/balance_priority_20261126'
OLD = ROOT/'results/constrained_handoff_20261126/campaign/runs/m100_H'
ANCHORS = ROOT/'results/loaded_contact_validation_20261126/model_tools/predictions_stage01'
OUT = BASE/'same_state_stage01'
OUT.mkdir(exist_ok=False)
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
protocol = json.loads((BASE/'protocol.json').read_text())
provenance = json.loads((OLD/'provenance.json').read_text())
xml = Path(provenance['models']['mirror_xml'])
inputs = [Path(__file__),BASE/'protocol.json',xml,OLD/'precision.npz',OLD/'governor_diagnostics.json',
    ROOT/'src/spherical_wheel/handoff_control.py',ROOT/'src/spherical_wheel/balance_priority_control.py',
    ROOT/'src/spherical_wheel/handoff_model.py']
for event in ('early_shellB','late_beltB'):
    inputs.extend([ANCHORS/event/'matrices.npz',ANCHORS/event/'predictions.json'])
hashes = {str(p.relative_to(ROOT)):sha(p) for p in inputs}
(OUT/'registered_before_arithmetic.json').write_text(json.dumps(dict(
    utc=datetime.now(timezone.utc).isoformat(), input_sha256=hashes,
    scope='No Gazebo, integration or observer update. Same known I56H state/prior input used for both A/P; not either controller trajectory.',
    criterion='P preserves Gbodyxy clipped-native total response within1e-8 and caps; oldA shows nontrivial late primary loss. No required individual actuator value if allocation can redistribute.'),indent=2)+'\n')
shutil.copyfile(__file__,OUT/'check_same_state.py')
with np.load(OLD/'precision.npz',allow_pickle=False) as z:
    data={k:z[k].copy() for k in ('time_s','exact_native_requested','exact_previous_applied',
        'exact_qpos','exact_qvel','exact_input_bounds','exact_reference_qvel','exact_estimated_support_loads_N')}
governor=json.loads((OLD/'governor_diagnostics.json').read_text())
settings=HandoffSettings(**protocol['adapters']['hard_slew']['settings']['allocator'])
model=mujoco.MjModel.from_xml_path(str(xml));helper=MeasuredContactVelocityModel(model,contact_strength=10.)
rows=[]
for event in ('early_shellB','late_beltB'):
    prediction=json.loads((ANCHORS/event/'predictions.json').read_text());i=prediction['saved_index']
    response=helper.evaluate(data['exact_qpos'][i],data['exact_qvel'][i],prediction['observer_records'],
        horizon_s=.02,tangent_confidence=prediction['tangent_confidence'])
    with np.load(ANCHORS/event/'matrices.npz',allow_pickle=False) as z:
        np.testing.assert_allclose(response.response,z['h020ms_response'],atol=1e-12,rtol=0.)
    diagnostic=governor[i]['diagnostics'];ready=np.asarray(diagnostic['readiness'])
    rates=data['exact_reference_qvel'][i,7:9];target=np.asarray(diagnostic['auxiliary_shell_target_rad_s'])
    prep=rates.copy()
    active=ready[1:3]>0
    prep[active]=(target[active]-(1-ready[1:3][active])*rates[active])/ready[1:3][active]
    kwargs=dict(native_requested=data['exact_native_requested'][i], input_bounds=data['exact_input_bounds'][i],
        previous_applied=data['exact_previous_applied'][i],dt_s=.005,qvel=data['exact_qvel'][i],
        response=response.response,fork_rotation_world=response.fork_rotation_world,
        shell_axis_world=response.shell_axis_world,shell_axial_inertia=response.shell_axial_inertias_kg_m2,
        support_loads_N=data['exact_estimated_support_loads_N'][i],reference_shell_rates=rates,
        readiness=ready,preparation_shell_rates=prep)
    a=BoundedHandoffAllocator(settings).solve(**kwargs)
    p=BalancePriorityAllocator(settings).solve(**kwargs)
    nominal=np.clip(kwargs['native_requested'],-kwargs['input_bounds'],kwargs['input_bounds'])
    da=response.response[3:5]@(a.command-nominal);dp=response.response[3:5]@(p.command-nominal)
    assert np.max(abs(dp))<=1e-8 and np.all(abs(p.command)<=kwargs['input_bounds'])
    rows.append(dict(event=event,index=i,time_s=float(data['time_s'][i]),
        previous=kwargs['previous_applied'].tolist(),native_clipped=nominal.tolist(),
        A_command=a.command.tolist(),P_command=p.command.tolist(),A_total_balance_delta=da.tolist(),
        P_total_balance_delta=dp.tolist(),P_native_fallback=p.diagnostics['native_fallback_used']))
assert np.linalg.norm(rows[-1]['A_total_balance_delta'])>.01
assert hashes=={str(p.relative_to(ROOT)):sha(p) for p in inputs}
report=dict(iteration='I-58',passed=True,physics_executions=0,source_inputs_unchanged=True,
    rows=rows,limitations='Static local-model and integration regression at known H states; no new physical trajectory or stability evidence.')
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
(OUT/'artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size)
    for p in sorted(OUT.iterdir()) if p.is_file()]),indent=2)+'\n')
print(json.dumps(report,indent=2))
