# I-43: bounded tilt reference and matching tilt rate

2026-11-25. **Implemented and evaluated; not promoted.** The optional generator
reduces early path-error amplitude and belt demand, but fails the declared
whole-route/raw-reference and turn-regression screen. The user authorized the
I-42 proposal: keep LQR, smooth the desired tilt and supply its corresponding
tilt rate. The native controller remains default.

[Three-arm comparison video](../../../../results/tilt_reference_20261125/videos/tilt_reference_comparison.mp4)
· [Shareable evidence bundle](../../../../results/tilt_reference_20261125/presentation-pack.zip)
· [Authoritative main report](evidence/analysis/stage01/report.json).

[Declared protocol](../../../../results/tilt_reference_20261125/protocol.json)
· [I-42 evidence and negative results](../handoff-factorial/README.md).

I-42 found that all tested support/preparation branches still departed at
6.860 s. At 6.825 s every branch correctly selected belt A yet requested about
0.83 m/s against its 0.30 m/s simulated limit, mostly attitude/rate feedback.
The desired lean changed while desired angular rate was zero. This motivates
a reference experiment; it does not prove the reference is the cause.

The new feature is opt-in. A smooth bounded two-stage filter generates tilt
and its analytic rate from the current observer-derived raw target. The
declared time constant is 0.08 s and maximum tilt-coordinate rate 60°/s.
Its derived coordinate-acceleration bound is 1500°/s². These are numerical
design assumptions, not measured actuator limits or proof of attainable motion.
The existing final static ground lean, yaw composition, gain bank, observer,
contact geometry, rolling feedforward and incoming-drive preparation are kept.

The generator advances the previous held target to the current command time,
then records the current target for the next interval. It updates once per
command, independently of calls to the pure raw-attitude accessor. It starts
at 4 s with the exact raw quaternion and zero rate, avoiding an activation
jump, and stays enabled through the route. The desired tilt rate is mapped
through the rotation Jacobian into the actual body's angular-velocity frame.
Measured yaw rate is not added; existing heading behavior remains intentional.

Three fresh matched 44 s Gazebo/DART runs compare native control, shaped angle
with zero angular-rate target, and the same generator with consistent rate.
All use phase +0.9, the same timed Mars figure-eight and the same bounds.
The angle-only branch is an ablation, not a fallback candidate chosen after
seeing results. Exact default/saved-history behavior, warm prefixes, generator
state/reset/cloning and quaternion/rate consistency are checked before physics.

Results will include lean error against **both raw and shaped targets**,
absolute tilt, nominal path tracking, belt demand, saturation and all six
fixed I-41 turns. The raw target uses the unchanged native formula from each
arm's own observer; its quaternion trace need not remain identical once robot
motion differs. A smaller error against a modified target alone is insufficient.
Raw-reference summaries and route gates will be recomputed separately.

The consistent-rate candidate must pass the declared early-error, belt-demand,
raw-balance, whole-route and turn nonregression screen before broader tests.
If it passes, the historical five-case suite reuses the main +0.9 pair and
adds eight fresh runs for the other four route/phase pairs. This means at most
11 new Gazebo executions, not 13 independent cases. No retuning or automatic
default promotion is authorized by the experimental screen.

The exact algorithm is a bounded tanh cascade in a 2D tilt chart. A convex
SSP-Heun integration with at most 1 ms substeps preserves the tilt disk without
clipping or arrival resets. Analytical rate/acceleration avoid differentiating
noisy contact estimates. Coordinate acceleration is distinct from transported
body-frame component acceleration. Independent mathematical and finite-
difference checks supplement controller integration and forecast-clone tests.

Eighty-three pre-change source/test origins and seven older presentation
bundles are preserved before implementation. Exact tested source stages,
failures, reports, plots and comparison videos are retained, including the
failed performance screen. No particle filter, iLQR,
LQR-weight change or physical-robot validation is part of this iteration.

## Implementation and compatibility

The new [tilt_reference.py](../../../../src/spherical_wheel/tilt_reference.py)
contains immutable settings, the causal bounded cascade and its rotation-
Jacobian mapping. [offroad_control.py](../../../../src/spherical_wheel/offroad_control.py)
adds a nested optional parameter, one update per command, reset handling and
reference diagnostics. It overwrites stale angular references during a
contact dropout. The pure raw-attitude accessor remains unchanged.

The feature is disabled by default. Disabled settings are omitted from the
legacy as_dict output. Direct dataclasses.asdict includes the new None field;
the compatibility guarantee applies to the controller's public as_dict method.
Enabled settings round-trip as a nested dictionary. An experimental opt-in is:

```python
from spherical_wheel.offroad_control import OffroadParams
from spherical_wheel.tilt_reference import TiltReferenceSettings

params = OffroadParams(
    mode="hardware",
    tilt_reference=TiltReferenceSettings(
        time_constant_s=0.08,
        max_rate_deg_s=60.0,
        activation_time_s=4.0,
        use_rate_reference=True,
    ),
)
```

This reproduces the tested settings, which are not recommended as a new
default. The campaign does not establish safe cold activation or performance
on the conditional five-route suite.

The full disabled-feature saved-history replay processes all 8,801 original
measurements with no Gazebo run or private forecast. Requested/applied inputs,
observer wrench, normal, nominal joint history and timed references are
bit-identical at every row. Fresh Gazebo baseline also reproduces all 17 common
I-40 precision arrays exactly. No observer, gain bank, allocator, synchronizer
or contact-geometry source is changed. The only modified existing source
origin is offroad_control.py; the module and two test files are new.

## Three fresh Gazebo results

All three 44 s cases complete without falling, with the same nominal time/XY/
velocity/acceleration program. Native and comparable generator-history prefixes
match through the 4 s activation command, which emits the exact original
quaternion and zero rate. The enabled generator's auxiliary last-seen clock
differs from the disabled controller before activation; true full fingerprints
record this intentional bookkeeping difference separately.

| Metric | Native baseline | Shaped angle, zero rate | Shaped angle, consistent rate |
| --- | ---: | ---: | ---: |
| Primary local-path RMS, 6.4–7.4 s | 38.661 mm | 25.889 mm | 29.893 mm |
| Primary local-path peak | 67.410 mm | 52.835 mm | 59.909 mm |
| Peak requested belt-A speed, all 6.6–6.9 s samples | 0.835 m/s | 0.376 m/s | 0.491 m/s |
| Primary requested-clipping fraction | 9.0% | 3.0% | 4.5% |
| Primary raw-reference peak lean error | 5.328° | 6.555° | 5.580° |
| First sustained 20 mm departure | 6.860 s | 6.830 s | 6.820 s |
| First departure sample span | 2.565 s | 1.610 s | 1.990 s |

The consistent-rate candidate reduces early RMS **22.7%** and peak requested
belt-A speed **41.2%**, but demand still exceeds the unchanged 0.300 m/s bound.
The departure is smaller/shorter; it is not prevented or delayed. Its threshold
onset is 40 ms earlier. All demand samples are included, without selecting only
favorable contact labels afterward.

| Full-route metric | Baseline | Angle only | Consistent rate |
| --- | ---: | ---: | ---: |
| Moving timed-position RMS | 50.486 mm | 61.863 mm | 52.180 mm |
| Peak timed-position error | 190.901 mm | 130.644 mm | 115.390 mm |
| Final timed-position error | 190.191 mm | 63.058 mm | 35.605 mm |
| Peak error from raw/native lean target | 8.709° | 13.135° | 13.581° |
| Peak error from used/shaped lean target | 8.709° | 2.399° | 6.973° |
| Peak absolute platform tilt | 13.784° | 13.647° | 13.652° |
| Settled against raw target | No | No | Yes |
| Settled against used target | No | No | No |

Shaped-target errors alone would hide the raw-target regression. The increase
to 13.581° is an error against the original target, not evidence of toppling;
absolute platform tilt is similar and every run finishes. Consistent-rate
moving RMS is 3.35% worse, within the declared 5% tolerance, and endpoint/peak
tracking improve. These benefits do not override the failed raw-reference
balance and turn requirements. None passes every original route gate under
either target convention.

Angle shaping alone reduces early demand/error more strongly, but worsens
primary raw-reference lean and whole-route RMS. Rate feedback improves those
quantities relative to angle-only without dominating every metric. Angle-only
remains the predeclared ablation; it is not substituted as a winning candidate.

## All six turns and the acceptance decision

| Fixed reference turn | Baseline overshoot | Consistent-rate overshoot |
| --- | ---: | ---: |
| 11.775 s / y | 0 mm | 24.469 mm |
| 15.377 s / x | 0 mm | 0 mm |
| 18.301 s / y | 0 mm | 33.272 mm |
| 23.699 s / y | 9.836 mm | 35.306 mm |
| 26.623 s / x | 28.022 mm | 11.098 mm |
| 30.225 s / y | 60.204 mm | 94.125 mm |

Four of six overshoots worsen. The first reversal is nearly on time (−0.035 s
versus baseline +1.101 s) but gains 24.469 mm geometric overshoot. At the fifth
event no valid 100 ms reversal is observed within the fixed window; a smaller
overshoot there is not complete evidence of a better turn. Complete windows,
multiple/missing reversal flags and every arm remain in the numeric report.

All implementation/input/cadence/frame prerequisites pass. The chosen
consistent-rate candidate passes the early RMS, belt demand, primary raw
lean and nominal full-route error conditions. It fails **full raw-reference
peak lean** and **all-six-turn nonregression**. Therefore the main screen fails,
and the conditional eight executions do not launch: exactly **three** fresh
Gazebo runs, zero retries, zero conditional runs.

Keep the native controller as default and retain this initial setting as an
opt-in research prototype. The result supports a tradeoff between reducing
early requested motion and keeping up with the evolving raw lean/turn targets.
It does not justify weakening balance feedback, globally suppressing drive
preparation, or claiming the initial failure solved. No parameters were
retuned after these outcomes. Further candidate selection is separate work.

## Verification and retained corrections

- [Core verification](evidence/core_development/report.json): 109 unique
  selected tests pass in both environments—32 new (20 pure math and 12
  controller/clone/forecast integration) plus 77 existing cases. This is a
  relevant subset, not the full repository suite. Environment repetitions and
  the independent 20-test stage overlap and are not added again.
- [Independent math](evidence/independent_checks/stage02/math_report.json):
  100 deterministic synthetic cases, not 100 additional robot trials. Maximum
  Rodrigues rate discrepancy 7.08e−11 rad/s; flow-Jacobian discrepancy 2.35e−9/s;
  SSP versus high-accuracy ODE state discrepancy 4.27e−7 rad; constant-target
  residual 8.44e−15 rad.
- [Default replay](evidence/default_replay/stage01/report.json): all six named
  comparisons are bit-identical over 8,801 guarded saved-measurement calls.
- [Campaign contracts](evidence/harness_checks/stage04_clock/report.json):
  13 unique pure bookkeeping/evaluation/clearance checks pass in both
  environments. Earlier 9/12-test stages overlap rather than add coverage.
- [Independent raw QA](evidence/independent_raw_QA/stage02_model_constants/report.json)
  verifies exact generator replay, one initialization plus 8,000 advances per
  enabled arm, all substeps at most 1 ms, body-frame rate errors <=3.34e−16 rad/s,
  target-error reconstruction, prefix behavior and source hashes. Observed
  chart rate maxima 36.25/45.12°/s and acceleration maxima 823.89/813.42°/s²
  remain below the declared 60/1500 bounds.

Review found a preactivation backwards-clock validation gap after the initial
19 passing pure tests. That source/coverage stage is preserved; corrected
20-test coverage includes the adversarial clock/reset case. Auxiliary clock
logging was then distinguished from the comparable canonical reference state
before any physics. The initial ambient ROS pytest plugin failed before
collection in one environment; isolated plugin loading corrected setup without
installing unrelated dependencies. These failures/stages remain retained.

Initial raw QA reconstructed constants from full-precision mass metadata,
whereas the actual model uses rounded XML inertial constants. Its 3.5e−13 rad
check discrepancy and source are preserved. Corrected QA reads the actual XML;
no controller, raw output or simulation changed. Initial renderer compile and
successful preview stages are also retained before presentation refinements.

[Early comparison plot](evidence/analysis/stage01/event_comparison.png)
· [All-six-turn plot](evidence/analysis/stage01/turn_comparison.png)
· [Video QA](evidence/videos/qa.json)
· [Explicit conditional-run decision](evidence/regression_decision.json).

The 16.3 s video replays the actual early event and first turn with fixed
camera/world XY, then shows the full outcome table. It retains earlier onset,
turn regressions, raw-versus-shaped lean and the failed screen. It is rendered
from recorded Gazebo/DART states, with no additional robot simulation.

Exact protocol, launch arguments, versions, implementation patch, tested
sources and failed stages are archived. Restore .py.txt snapshots to their
recorded origin for reproduction, using the repository terrain assets and
numerical environments and a fresh output family. Large rollouts/precision
arrays and MP4 remain in the shareable bundle; generated workspaces/meshes are
inventoried. No old result or bundle is replaced.

[Compact evidence manifest](evidence/manifest.json)
· [Payload manifest included in ZIP](../../../../results/tilt_reference_20261125/presentation_manifest.json)
· [Post-package verification sidecar, outside ZIP](presentation_verification.json).
