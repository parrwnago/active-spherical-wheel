# I59 native-command boundary

This source-only mapping was prepared before computing anchor coefficient differences. The hypothesis permits either a material command effect or identical rows/trims and a null result. No direction or benefit is assumed.

In `src/spherical_wheel/offroad_control.py`, `_terrain_feedback` selects a contact-scheduled gain and then replaces **row zero** with the retained ordinary design's row (`gain[0] = design['K'][0]`, line 786). Its return command is `equilibrium_u + rolling_feedforward + support_torque - gain @ error` (line 816). The exact line numbers below are bound by the accompanying source hashes.

`command` records `lqr_command = -K @ error` before synchronization (lines 686–691). It then blends `(1-w)*pre_sync + w*sync_commands` (lines 692–711), records the resulting transition delta, adds heading (lines 712–719), fades the first three torque channels (lines 720–723), and finally amplitude-clips the native request (lines 724–727).

`ContactSynchronizer.preview` in `synchronization.py` initializes its commands by clipping the primary command (line 174). However, it then overwrites every shell/belt target at indices 1–4 using fixed pose/velocity/contact geometry (lines 221–245). These targets and their blend weights do not depend on the primary command at a held state. Shaft index 0 has no patch and its blend weight stays zero. Thus the relevant fixed-state synchronization derivative is `diag(1-w)`. This does **not** mean keeping its additive transition delta fixed: that delta changes by `-w*delta_pre`.

Heading uses pose, velocity, held loads and heading history, and has no primary-command dependence (`offroad_control.py`, lines 820–871). Holding these quantities fixed holds heading torque fixed. `_fade_when_unloaded` (lines 873–890) uses the fixed filtered load fraction and damping target `-airborne_damping*(qvel[6:9]-reference_qvel[6:9])`. Its derivative is `D = diag(f,f,f,1,1)`; the additive fade delta changes when its incoming command changes.

For an intervention that preserves state, references, contacts, history, synchronization targets/weights and fade factor:

```
delta_pre = delta_equilibrium_u - delta_K @ stored_error
delta_native_request = D @ diag(1-w) @ delta_pre
new_native_applied = clip(old_native_request + delta_native_request, -bounds, bounds)
```

The primary intervention has nonzero `delta_K` only in row 0. The trim-only diagnostic changes all five equilibrium-command components while preserving the recorded reference/error. The combined diagnostic changes both. These are partial, same-state interventions, not reinitializations of the ordinary design.

`results/balance_priority_20261126/campaign_tools/run_case.py` preserves native request/applied and `exact_native_clipping_command_delta` before the candidate (lines 248–251, 277–283). It overwrites the generic `clipping_command_delta` with the final candidate's clipping (lines 269–272). Consequently the **native** audit must use `exact_native_clipping_command_delta`. On P traces the resulting counterfactual is the native policy evaluated at the observed P state. It is not a reoptimized final P command.

Minimal exact fields are `exact_feedback_gain`, `exact_error`, `exact_equilibrium_command`, `exact_rolling_feedforward`, `exact_support_command`, `exact_lqr_command`, `exact_transition_command_delta`, `exact_heading_command`, `exact_fade_command_delta`, `exact_sync_weights`, `exact_native_requested`, `exact_native_applied`, `exact_native_clipping_command_delta`, `exact_input_bounds`, `exact_qvel`, and `exact_reference_qvel`. The scalar fade was not stored in precision. Ordinary `rollout.npz/drive_fade` is float32 (`offroad_experiments.py`, lines 401–410).

Where the exact damping-minus-prefade vector is nonzero, exact recorded stage deltas constrain fade through `fade_delta[:3] = (1-f)*(damping-prefade[:3])`. Validate every component and its float32 recording. If that vector is zero or numerically uninformative, fade is not identifiable from baseline output. Retain its float32 rounding interval intersected with `[0,1]` and componentwise closure constraints. Always propagate the resulting factor interval; a small closure residual alone is not a factor-accuracy certificate. If the perturbed post-sync torque is zero, unknown fade cannot affect that intervention.

No observer update, gain lookup, LQR design, state/reference reset, physics, P reoptimization, trajectory prediction or default change belongs to this audit.
