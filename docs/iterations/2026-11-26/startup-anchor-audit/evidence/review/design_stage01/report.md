# I59 independent preregistered startup-anchor design review

This zero-physics audit can test whether the proposed persistent shaft-gain/trim path materially changes the native command at already-recorded states. It cannot isolate the entire effect of starting mechanical phase, execute a changed controller, or demonstrate a trajectory benefit. Keep the raw state, observer-derived contact information, already-transformed feedback error, reference/history, current scheduled rows 1–4, synchronization targets/weights, heading addition and unloaded fade conditions fixed.

The primary intervention changes only ordinary shaft-feedback row 0. Trim-only and row-plus-trim are separate secondary interventions. Do not call `reset`, recompute a local design, replace the whole scheduled matrix, regenerate references or invoke preview/estimator/observer updates. These operations would change the question and introduce additional state/history paths. The source permits a persistent ordinary row, but does not establish that its numerical coefficients differ materially across these three actual recorded anchors. A null or negligible result must be retained and should qualify the earlier explanation.

## Source map and exact command propagation

The native pre-synchronization command is equilibrium trim + rolling feedforward + support term − gain × feedback error (`offroad_control.py:816`). The gain's row 0 is overwritten by the ordinary local design at line 786; the other four rows use current phase/load scheduling. The recorded error already includes yaw-frame transformation, zeroed shell material angles, and the rolling-consistent shaft-error replacement. Use that exact logged error rather than reconstructing a different state error.

For new trim `a`, old trim `b`, new row `Ka`, old row `Kb`, and fixed logged error `e`, form:

`delta_preSync = (a-b) - [ (Ka-Kb) @ e, 0, 0, 0, 0 ]`.

In row-only mode `a-b=0`; trim-only mode has no row difference. With fixed preparation target `s` and blend vector `w`, synchronization is `(1-w)*preSync + w*s`, so its perturbation is `(1-w)*delta_preSync` (`offroad_control.py:698–700`). The logged transition additive delta cannot simply be reused as invariant under a changed preSync command.

Heading contributes a fixed additive term at lines 712–713, so it cancels from the difference. Unloaded fading scales the first three commands by factor `f` and adds fixed damping; the belts are unchanged (`offroad_control.py:883–889`). Thus:

`delta_requested = diag(f,f,f,1,1) * (1-w) * delta_preSync`.

Add this to the recorded native request and clip once to the same actual amplitude caps. Reusing the original fade additive delta unchanged would be incorrect. Verify the baseline stage sums and native clipping independently before interpreting any swapped result.

The audited outputs are **native requested** and **native amplitude-clipped** commands on all six B/P traces. In B, native clipped is the actual final output. In P, the physical final output additionally passes through the priority allocator (`run_case.py:252–268`). This audit does not resolve that allocator, so it must not label its output a counterfactual final P-applied command. Use `exact_native_applied` and `exact_native_clipping_command_delta`; the generic clipping fields were overwritten for the P boundary.

## Fade identification and ambiguity

Full-rate precision records exact pre-fade components and fade deltas, but not a float64 fade factor. Ordinary rollout contains float32 `drive_fade`. For each torque component, the exact baseline relation is:

`native_request = f*preFade + (1-f)*damping`.

The damping comes from exact current joint rates and exact reference rates with the unchanged parameter. Infer/check f where informative, using the component with the largest `preFade-damping` magnitude and verifying all components, [0,1] bounds and baseline closure. A tiny denominator makes factor inference poorly conditioned even when baseline command closure is excellent.

When all pre-fade torque components equal damping, baseline output is independent of f. Do not silently treat the rounded float32 factor as exact. Report the ambiguity or bound the candidate over its float32 rounding interval intersected with [0,1] and the baseline closure constraints. If the candidate torque perturbation itself is zero, its output remains independent of f and a null result can still be certified. If possible factor values straddle a materiality threshold, retain an ambiguous classification rather than selecting a favorable point.

## Controls and fixed materiality classifications

Before extracting one representative row/trim from each native B phase, verify their within-trace constancy. First record byte equality and actual maximum differences; then verify matching B/P coefficients at the same nominal phase. Do not silently select a convenient state if a supposedly constant anchor changes. Compare all three anchor pairs, preserving coefficients and derived command differences even if they are identical or negligible.

Require native-anchor reproduction and full decomposition/clip closure within **1e-10 absolute command units**, with explicit failures and immutable source/input hashes. This is a numerical consistency condition, not a physical accuracy claim. If a fade interval is required, closure and effect bounds must reflect it rather than claiming exact factor recovery.

The parent selected separate engineering materiality reports before arithmetic:

- Per-input absolute effect: **0.001 N m** for each torque and **0.0001 m/s** for each belt.
- Caps-normalized infinity effect: **0.001** of an input cap.

Report both classifications independently for raw and native-clipped commands; do not combine them opportunistically after results. Always retain actual magnitudes. Effects below these choices are negligible for this declared screening resolution, not mathematically zero or proved irrelevant to every possible trajectory.

Use all 8,801 saved states and the fixed full/moving/goal windows, making their overlap explicit. Maxima and state fractions can include the terminal unheld command, while any held-duration exposure must exclude it. These are correlated counterfactual evaluations of existing logs, not new trials, added integration tests or independent statistical samples.

Even a material command effect would establish only this frozen intermediate mapping at sampled states. Current phase/load scheduling, mechanical geometry, contact sequence, terrain encounter timing, actual motion and other initialization paths remain outside this intervention. A negligible result would weaken the startup-row/trim explanation and direct attention to those remaining paths; it would not prove a unique alternative cause.

The independent review is source-only. No recorded K/trim numerical values or counterfactual results were accessed before this registration, and no runtime source was edited. New audit-helper source inspection should be bound separately before executing its arithmetic.
