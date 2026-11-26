# I-49 — fixed-gain belt-speed capacity sensitivity

2026-11-26. **Three native runs completed and analyzed; capacity tradeoffs observed. Defaults unchanged.**

The user proposed increasing assumed actuator speed to see whether additional
capacity reduces the saturated balance corrections. This is a bounded
simulation experiment, not a motor recommendation or a change to defaults.
The earlier 6.825 s decomposition establishes that attitude/angular-rate terms
dominate that one LQR belt request; it does not classify all saturation events.

## Registered comparison

Three fresh isolated Gazebo/DART cases use the original 44 s Mars figure-eight,
phase +0.90 rad, yaw 45°, 5 ms control and 1 ms physics. Both belts have an
operating speed cap of **0.30**, **0.60** or **0.90 m/s**. The complete reference,
observer, support locator, incoming-drive algorithm, torque bounds and LQR
gains remain the same. Each arm has its own native closed-loop history.

The original 0.30 m/s configuration, gain library and ordinary local design
remain fixed. Only physical-plant and measured-state-mirror operating belt
bounds change, after the standard design is constructed. Updating both is
necessary because mirror state transfer re-applies prior inputs through its
own bounds. Incoming-belt preparation also uses the operating cap; this is a
capacity-interface comparison, not an isolated final-clamp-only ablation.

Changing the configuration itself would affect the LQR design's input
normalization and cache fingerprint. That would mix actuator capacity and
controller redesign, so it is deliberately deferred. Repository defaults and
model files remain unchanged.

## Measurements and provisional screen

Report complete-route original timed moving RMS, peak/final error, original
settling/route gates, absolute and reference-relative lean, first sustained
20 mm local-path departure, all six fixed turn overshoot/lag events, requested
versus applied inputs and command steps, joint-torque clipping, slip and contact
loads. Report belt saturation against both the arm's own cap and the original
0.30 m/s cap; a larger normalization denominator alone is not improvement.
Terminal unactuated observations are excluded from held-duration metrics.

The provisional screen requires completion without falling, at least 10% lower
moving RMS, peak/final error no more than 1 mm worse, peak reference-relative
lean no more than 0.5° worse, every turn overshoot no more than 5 mm worse and
lag no more than 0.1 s worse. Missing reversal coverage is not silently dropped.
Original settling and route gates are separate requirements for a broader
candidate claim. All outcomes are reported even if the screen fails.

The reference geometry/turn helper is the preserved corrected I-41 version,
not its initial buggy helper. Fresh 0.30 behavior is checked against the
historical native trace; fixed gain/ordinary-design hashes and initial states
are compared across arms.

## Interpretation limits

The belts still use ideal prescribed surface-speed actuation. Raising a cap
does not add motor-load, torque-speed, acceleration, current or thermal dynamics.
A better result would support an actuation-capacity hypothesis in this model,
not prove that a selected faster motor can deliver the motion. A worse result
would expose a control/capacity tradeoff, not justify discarding the run.
One terrain/reference/phase cannot support default adoption or a universal
motor requirement. No outcome-based retuning or retries are part of this screen.

Local family: [results](../../../../results/belt_capacity_sweep_20261126).
The protocol, jobs and inputs were registered before native outcomes.


## Results

All three runs complete 44 s without falling, each with 8,801 observed control
calls. The 0.30 m/s arm reproduces all 17 common historical precision arrays
bit-for-bit. Gain-bank, ordinary-design, seed-cache and initial-state/reference
checks pass across arms. Fixed physical gains do not mean time-series scheduled
gains must stay identical once each arm follows a different state history.

| Metric | 0.30 m/s | 0.60 m/s | 0.90 m/s |
| --- | ---: | ---: | ---: |
| Moving timed position RMS | 50.486 mm | 52.521 mm | 52.521 mm |
| Peak timed position error | 190.901 mm | 110.009 mm | 110.009 mm |
| Final error at 44 s | 190.191 mm | 49.231 mm | 49.231 mm |
| Peak reference-relative lean error | 8.709° | 6.053° | 6.053° |
| First sustained 20 mm local departure | 6.860 s | 6.850 s | 6.850 s |
| Original settling criterion | Fail | Fail | Fail |
| Original full-route gates | Fail | Fail | Fail |

The larger capacity improves worst/final error and peak relative lean, while
moving RMS worsens by 4.03%. It does not remove the initial departure. Some
turns improve; others regress. Turn four's overshoot increases 9.836→17.412 mm.
The signed-lag guard also fails at turns three/four. At turn four, signed lag
changes −0.444→+0.074 s, which is closer to zero in absolute value despite
failing that declared signed-increase guard. Both signed and absolute meanings
are retained rather than calling every timing result worse.

Both raised-cap arms fail the provisional multiobjective screen because of
moving RMS, settling/route gates and turn nonregression. These gates do not
erase the useful peak/final/lean improvements. This is an observed tradeoff,
not a complete solution or grounds for a general capacity recommendation.

## Why 0.60 and 0.90 produce the same motion

Saved qpos/qvel, requested commands, observer wrench, contact force/moment,
loads and utilization are byte-identical between the two raised-cap arms.
Their **applied belt commands are not identical**: belt A differs on nine
held samples, totaling 45 ms between 6.825–6.910 s. Its maximum applied speed
is 0.600 versus 0.834742 m/s. The 0.60 cap was therefore active.

Seven of those holds have loaded belt-A contact at both saved endpoints;
there are no holds unloaded at both endpoints. For example, at 6.825 s the
applied speeds are 0.600/0.834742, with current/next belt load 20.872/12.004 N.
The next recorded slip is 0.45699/0.68374 m/s while the saved body state and
contact force remain identical. Increasing the target changes calculated slip
without further body-motion improvement on this run.

This is consistent with sliding and a force-limited contact response, not
proof of one unique mechanism over every unrecorded 1 ms solve. The 5 ms
endpoint utilization diagnostic can exceed one under DART's box friction
formulation; it must not be misread as proof of violating a unit circular cone.
No new finer-physics run was made to resolve that hypothesis.

## Interpretation and decision

The user's capacity hypothesis is useful: the assumed belt-speed cap affects
closed-loop performance. Extra capacity improves some substantial errors, but
speed alone does not fix average tracking, initial departure, settling or all
turns. Preserve 0.60/0.90 as experimental capacity arms and leave defaults at
0.30 for reproducible comparison. This does not decide that a real design
should use a 0.30 belt, nor does it validate 0.60/0.90 under motor load.

Before motor selection, the model needs loaded torque/force-speed response,
gearing/pulley conversion and transient/thermal requirements. Nominal no-load
speed is insufficient: see the manufacturer explanation of
[speed/torque characteristics](https://support.maxongroup.com/hc/en-us/articles/360013761160-Motor-data-and-simulation).
This is a technical reference, not a recommendation for that manufacturer's
products. No motor or component was selected.

## Validation and retained corrections

- Six pure runner contracts and three independent mirror/preparation checks
  pass; the three integration checks pass in both numerical environments.
  These are nine distinct targeted checks, not a full repo suite or nine
  additional robot trials. No existing runtime source was edited.
- Exact raw clipping, previous-applied-input alignment, mirror transfer,
  baseline/reference/fixed-design identities and all 17 baseline fields pass.
- Independent direct arithmetic reproduces tracking/up-vector lean, all six
  geometric overshoots and belt maxima. One checker initially used a wrong
  JSON key name; its source/log remain in stage01 and a stage02 schema-only
  correction passes. No physics was rerun for it.
- The metric reducer uses the immutable corrected I-41 helper and all fixed
  turn anchors. Source-only independent review precedes first reduction.
- A renderer syntax error stopped before data loading/rendering. Its original
  source/log are retained, and the corrected renderer uses only saved states.
- Full raw/protocol/source and all unfavorable results are retained. No
  outcome-based cap retuning, extra route or default promotion was performed.

[Comparison video](../../../../results/belt_capacity_sweep_20261126/videos/belt_capacity_comparison.mp4)
· [Canonical report](../../../../results/belt_capacity_sweep_20261126/analysis/stage01/report.json)
· [Applied-command/motion sidecar](../../../../results/belt_capacity_sweep_20261126/review/cap060_cap090_equivalence/report.json).

The video shows actual saved Gazebo/DART states at 2× playback, with equal
view/plot scales across the three cases. It does not simulate a physical motor
or show a retuned controller. Compact evidence and a presentation bundle are
preserved with this record.


[Compact evidence inventory](evidence/manifest.json)
· [Presentation/source bundle](../../../../results/belt_capacity_sweep_20261126/presentation-pack.zip).
