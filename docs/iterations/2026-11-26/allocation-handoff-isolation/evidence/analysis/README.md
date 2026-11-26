# I51 — paired mechanism tests and late phase diagnostic

All four primary44s runs and the separately registered fifth44s diagnostic complete without falling. The0.30/0.60 baselines reproduce every common I49 precision field exactly. Initial states, original time/XY/V/A program and fixed gain-bank/ordinary design match. No runtime/config/default files changed.

The original least-squares controller does not deliberately move the path target or relax its timing. The reference candidate keeps those targets and permits internal rolling-reference mismatch to avoid extreme joint-reference changes. It is not a path-following governor or a guarantee of total-command feasibility.

## Primary matched-cap comparisons

| Arm | Moving RMS mm | Peak mm | Final mm | Relative lean max ° | Settled / old gates |
|---|---:|---:|---:|---:|---|
| base030 | 50.486 | 190.901 | 190.191 | 8.709 | False / False |
| reference030 | 44.213 | 97.186 | 32.687 | 6.346 | False / False |
| base060 | 52.521 | 110.009 | 49.231 | 6.053 | False / False |
| handoff060 | 42.056 | 92.651 | 15.575 | 11.683 | False / False |

Compare `reference030` only with `base030`, and `handoff060` only with `base060`. Both improve moving RMS by more than the registered10% threshold, but both fail the complete multiobjective screen. Favorable peak/final reductions remain useful evidence despite those failures.

The reference arm reduces moving RMS12.43%, peak/final errors and relative lean. Its references are modified on53 held samples(265ms), beginning6.700s. Native reference maximum5.380rad/s never reaches the20rad/s ceiling: the observed intervention is reference-slew limiting and its subsequent changed state history. It is not a same-history demonstration of directly clipping the baseline31.650s singular request. Guarded maximum is3.119rad/s and reference acceleration100rad/s². Modeled introduced rolling residual reaches0.09005m/s on modified rows; it is not measured tracking error or dynamic feasibility. Turn-four overshoot worsens by11.468mm; settling and old route gates fail.

The handoff arm reduces moving RMS19.92% and final error49.231→15.575mm. It changes496 held samples(2.480s), also beginning6.700s, and respects3m/s² during active episodes. Peak reference-relative lean worsens6.053→11.683°, and several turns regress. Native joint references still reach143.914rad/s; this intervention does not solve reference singularities. Raw native box clipping and additional handoff projection are reported separately. The common clipping delta is total actual-minus-raw, not solely actuator saturation.

All six fixed turn reversals remain covered. Signed-lag changes and absolute timing error have different meanings: reference turn3 changes−0.469→−0.183s and turn4−0.444→+0.374s, both closer to zero in absolute terms while failing the registered signed-increase guard. Reference turn6 instead worsens+0.569→+0.903s. No unfavorable turn is discarded.

## Separately registered late phase scheduling

The fifth case changes only world-Z versus sensed-normal gain scheduling from38s. All47 common precision fields match the fresh0.30 baseline for7,600 preactivation observations; state/reference at38s match and gain differences begin at38s.

| Metric | base030 | phase_late030 |
|---|---:|---:|
| Timed RMS38–42s |57.857mm|48.323mm|
| Timed RMS38–44s |84.652mm|56.707mm|
| Full moving RMS |50.486mm|50.495mm|
| Full peak error |190.901mm|96.309mm|
| Final error44s |190.191mm|53.993mm|
| Full peak relative lean |8.709°|6.089°|

This matched-history intervention supports a contribution from gain scheduling in this late episode. It does not validate globally changing the phase definition: earlier global normal-relative scheduling failures remain relevant. It remains unsettled and fails the original route gates. Free-heading yaw change over38–44s grows in magnitude from−109.755° to−142.269°, so improved position does not imply uniformly better behavior. Joint tracking RMS in the addendum is a three-element per-joint vector, not the aggregate vector-norm RMS used in some earlier audits.

No combined candidate, outcome-based parameter tuning or broader trajectory expansion was performed. The native defaults remain unchanged. Reference/episode bounds are mechanistic successes; tracking, balance, friction and actual motor capability remain separate questions.

[Primary report](stage01/report.json) · [Turn table](stage01/turn_metrics.csv) · [Tracking plot](stage01/paired_tracking.png) · [Turn plot](stage01/paired_turns.png) · [Separate phase addendum](phase_stage01/report.json) · [Late error/yaw plot](phase_stage01/late_phase_comparison.png) · [Compact JSON](summary.json).
