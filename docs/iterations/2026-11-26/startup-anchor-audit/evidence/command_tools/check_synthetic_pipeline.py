"""Nonzero synthetic stage/ambiguity checks, independent of actual anchors."""
from pathlib import Path
import hashlib
import json
import shutil
import numpy as np
from native_pipeline import native_stages, infer_fade, propagate_fixed_state

HERE = Path(__file__).resolve().parent
OUT = HERE/'synthetic_control_stage01'
OUT.mkdir(exist_ok=False)
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
(OUT/'registered_before_arithmetic.json').write_text(json.dumps(dict(
    scope='Synthetic nonzero-command, command-dependent transition/fade, clipping and degenerate-fade checks; no actual anchor comparisons or physics.',
    source_sha256=sha(Path(__file__)), helper_sha256=sha(HERE/'native_pipeline.py')), indent=2)+'\n')
shutil.copy2(__file__, OUT/'check_synthetic_pipeline.py')
shutil.copy2(HERE/'native_pipeline.py', OUT/'native_pipeline.py')

caps = np.array([1.5, .6, .6, .3, .3])
gain = np.arange(90, dtype=float).reshape(5, 18)*.0001
error = np.linspace(-.2, .3, 18)
eq, ff, support = np.array([.3, -.2, .1, .05, -.04]), np.ones(5)*.01, np.arange(5)*.02
lqr = -gain@error
pre = eq+ff+support+lqr
weights = np.array([0., .35, .6, .9, .2])
target = np.array([0., .3, -.2, .1, -.1])
post = (1.-weights)*pre+weights*target
heading = np.array([0., .02, -.01, 0., 0.])
prefade = post+heading
qvel, ref = np.arange(9)*.03, np.zeros(9)
damping = -.01*qvel[6:9]
fade = .37
request = prefade.copy(); request[:3] = fade*prefade[:3]+(1.-fade)*damping
row = {'exact_'+key: value for key,value in dict(feedback_gain=gain,error=error,
    equilibrium_command=eq,rolling_feedforward=ff,support_command=support,lqr_command=lqr,
    transition_command_delta=post-pre,heading_command=heading,fade_command_delta=request-prefade,
    sync_weights=weights,input_bounds=caps,native_requested=request,
    native_applied=np.clip(request,-caps,caps),native_clipping_command_delta=np.clip(request,-caps,caps)-request).items()}
row['exact_clipping_command_delta'] = np.full(5, 999.)  # Final-P field must be ignored.
stages = native_stages(row)
f = infer_fade(stages,qvel,ref,.01,np.float32(fade))
delta = np.array([4.,-.7,.2,-.4,.5])
result = propagate_fixed_state(stages,delta,f)
counter = (1.-weights)*(pre+delta)+weights*target+heading
counter[:3] = fade*counter[:3]+(1.-fade)*damping
np.testing.assert_allclose(result['native_request'],counter,atol=1e-12,rtol=0)
np.testing.assert_allclose(result['native_clipped'],np.clip(counter,-caps,caps),atol=1e-12,rtol=0)
np.testing.assert_allclose(result['transition_delta'],weights*(target-pre-delta),atol=1e-12,rtol=0)
assert result['native_clipped'][0] == caps[0]
assert result['request_lower'][0]-1e-12 <= counter[0] <= result['request_upper'][0]+1e-12
identity = propagate_fixed_state(stages,np.zeros(5),f)
assert identity['native_request'].tobytes() == row['exact_native_requested'].tobytes()
degenerate = dict(stages); degenerate['pre_fade'] = prefade.copy(); degenerate['pre_fade'][:3] = damping
degenerate['fade_delta'] = np.zeros(5)
unknown = infer_fade(degenerate,qvel,ref,.01,np.float32(.41))
assert not unknown['identifiable'] and unknown['upper'] > unknown['lower']
belt_only = propagate_fixed_state(degenerate,np.array([0.,0.,0.,.1,-.1]),unknown)
assert belt_only['fade_effect_independent']
np.testing.assert_array_equal(belt_only['request_lower'],belt_only['request_upper'])
torque = propagate_fixed_state(degenerate,np.array([.2,0.,0.,0.,0.]),unknown)
assert torque['request_upper'][0] > torque['request_lower'][0]
report = dict(passed=True,nonzero_fixed_target_and_fade_recomposition=True,
    saturation_recomputed=True,wrong_final_P_clipping_ignored=True,zero_swap_bitwise=True,
    degenerate_fade_bounds_retained=True,belt_only_delta_independent_of_unknown_fade=True,
    actual_anchor_comparisons=0,physics_runs=0)
(OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
(OUT/'artifact_manifest.json').write_text(json.dumps(dict(files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size) for p in sorted(OUT.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']),indent=2)+'\n')
print(json.dumps(report))
