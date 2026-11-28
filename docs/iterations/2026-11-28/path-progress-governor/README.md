# I-78 — managed path progress helps one start, fails the comparison screen

2026-11-28. **Completed and analyzed; candidate not adopted.** The original native
N gain-scheduled LQR, momentum observer and contact feedforward remain selected.
Production source and defaults are unchanged.

The user approved slowing path progress during difficult configurations, and
asked whether the advancing path clock really caused the departures. **That
sole-cause claim is not established.** [I-77](../../2026-11-27/baseline-deviation-cause/README.md)
showed that selected reference interventions influence later departures. I-78
tests an online remedy around the unchanged controller, rather than assuming
that remedy already works.

## Results and media

[Two-page PDF](evidence/evaluation/publication_stage01/progress_comparison.pdf) ·
[path comparison](evidence/evaluation/publication_stage01/motion_comparison.png) ·
[clock and clipping](evidence/evaluation/publication_stage01/timing_commands.png) ·
[canonical report](evidence/evaluation/completed_stage01/report.json).

| Shaft start | Peak distance from finite path, N → G | Progress-weighted RMS, N → G | Added reference travel time |
| --- | ---: | ---: | ---: |
| −1.0 rad | 148.9 → 102.0 mm | 50.0 → 34.0 mm | 0.985 s |
| +0.3 rad | 124.6 → 124.4 mm | 42.9 → 39.7 mm | 0.600 s |
| +0.9 rad | 59.7 → 59.2 mm | 21.3 → 21.2 mm | 0.005 s |

Six fresh Gazebo/DART runs finish. All three G runs complete the path and then
a full physical ten-second goal-reference hold; none stalls. All six meet final
two-second position/speed goal-convergence criteria. Only G−1 passes the stricter
simultaneous position, speed, rotation and attitude settling criteria; the
others retain rotation. Completion, approaching the goal and fully settling
are different outcomes.

The frozen comparison has **180/180 setup checks passing and 80/82 performance
checks passing**. These are separate scopes, not the full repository suite.
At +0.3 rad, peak absolute tilt increases **9.397 → 9.970°**, exceeding the
native+0.5° margin. Peak planar speed increases **0.1819 → 0.2284 m/s**, exceeding
the 0.2101 m/s margin. Both failures remain in the result.

At −1 rad, any-channel clipping falls from **0.680 to 0.240 s** and the largest
departure falls about31%. Requests still exceed the applied bounds at some
instants. This has not made LQR a constrained optimizer or removed the poorly
conditioned rolling inverse.

Local comparison videos, sharing physical elapsed time at1×:

- [−1.0 rad](../../../../results/path_progress_governor_20261128/publication/video_m100_stage01/straight_m100_N_G.mp4)
- [+0.3 rad](../../../../results/path_progress_governor_20261128/publication/video_p030_stage01/straight_p030_N_G.mp4)
- [+0.9 rad](../../../../results/path_progress_governor_20261128/publication/video_p090_stage01/straight_p090_N_G.mp4)
- [Shareable visual bundle, 15.6 MB](../../../../results/path_progress_governor_20261128/publication/visual_bundle_stage01.zip)

Physics is recorded Gazebo/DART; MuJoCo supplies visuals without reintegrating
physics or rerunning controller/observer. Ended native panels explicitly freeze
at their actual45s endpoint while G continues. No progress alignment or speed
change conceals delay. Media/rollouts remain local; compact plots, reports and
exact source/metadata evidence are preserved in this repository.

## Implementation and limitations

N uses the original timed reference. G advances a virtual clock τ at rate λ in
[0,1], with no positive floor. For the unchanged path g:

```text
p = g(τ)
v = g′(τ) λ
a = g″(τ) λ² + g′(τ) λdot
```

Position, velocity and acceleration slow consistently. The initial3s hold stays
exact. A nominal32s traversal finishes at virtual time35s, then holds the goal
for ten physical seconds. G has at most75s; N retains45s. G reference-arrival
samples are35.985/35.600/35.005s. The +0.9 delay includes the ordinary first
sample after arrival; actual slowing is small.

Target rate is the minimum of one and three ratios:

1. Minimum singular value of the inferred-load-weighted mean planar rolling
   matrix, column-scaled by [5,5,5 rad/s,.3,.3 m/s], divided by .15 m/s.
2. 5 rad/s divided by largest nominal full-speed joint-rate reference.
3. .85 divided by largest absolute nominal full native request normalized by
   its applied bound.

These are fixed simulation design scales, not motor ratings or a guaranteed
balance reserve. A bounded two-stage cascade, .10s time constant,4s⁻¹ stage-rate
bound and ≤1ms substeps smooths λ and its derivative. Each sample advances the
previous target before registering the new one. A spike can remain while slowing;
the mean matrix can also conceal disagreement between contacts.

One private native-controller clone evaluates hypothetical full-speed demand at
the **current** virtual position and measured state. History is refreshed from
the live controller; private commands are never applied. No private physics,
future terrain query or contact truth is used. One live controller/observer
update commands each held interval. This is a reactive progress policy, **not
a future contact forecast or a guarantee of feasible future commands**.

Native lean, gains, observer, support/load routing, unbounded rolling inverse,
feedforward, incoming preparation and clipping remain unchanged. No heading
helper, camera, dynamic-contact forecast, new tilt shaper or motor-cap increase.

## Conditions, evidence and decision

Initial revision `9f7cfc87b9a5b12f45c0a0158fda0f5dcb8c73f3`; original I-76 straight
campaign, Gazebo/DART Mars terrain, origin[−2.4,10.0]m, yaw113°,1.2m line,
shaft starts−1.0/+0.3/+0.9rad. Exact simulated body state is allowed; controller
contacts stay inferred. Original gain bank,5ms loop and bounds[1.5,.6,.6 N m,
.3,.3 m/s].

[Frozen protocol](evidence/protocol/protocol.json) binds100 sources and66 inputs,
SHA256 `cf868e800031bee3dc8f503269dffdd461067feff6fdf4bf33bd5e417a0fb112`.
One registered serial launcher observes actual zero exits for all six cases;
sources/inputs remain unchanged. All three fresh N recordings match all79
original I-76 precision fields exactly.

The fixed evaluation uses the same finite path: geometric peak, progress-weighted
RMS and physical-time RMS, with pauses retained in the latter. Old-clock XY error
is secondary schedule lag. An incomplete path cannot pass. A continuous5s with
virtual progress<.05s while unfinished and >20mm from goal fails no-progress.
Extra time, clipping seconds/fractions, balance, motion, steps and settling stay
separate. All phases must pass; the lead−1 peak must fall at least20%. No
post-outcome phase, parameter, horizon or gate tuning.

**Decision:** keep original N selected and retain G as an isolated failed-screen
experiment. The evidence supports reference-generation contributions and one
useful retiming effect. It does not prove an advancing clock explains every
departure or contact estimation is always correct. Two starts have almost
unchanged peaks. A defensible next analysis would separate remaining rolling
demand from the normal-derived attitude target, rather than assume more threshold
tuning will solve it. Earlier I-17–I-21, I-43, I-51 and I-56 regressions remain.

## Preservation and reproduction

[Iteration log](../../../../ITERATION_LOG.md) retains proposal, registration,
corrections and final outcomes. Compact evidence records local origins/hashes;
full raw data/media live in `results/path_progress_governor_20261128/`.

The registered original evaluator is retained. Separately sealed stage02 fixes
a conditional probe-identity check and protocol-subdirectory base path. Terminal
arrival accepts the first5ms sample after floating-point crossing, without
extending the75s budget or ten-second hold. Corrections were registered before
reading candidate arrays and change no numerical performance gates. Earlier
setup/contract failures remain preserved. A later +0.9 equality concern was
resolved by direct sealed raw checks: references first differ at19.815s, states
at19.820s, and independent finite-line peaks match the canonical report.

Two independent audits reproduce the scalar scores and every fixed decision:
[564-leaf/82-gate audit and peak contexts](evidence/evaluation/scalar_context_stage01/summary.md),
[independent command/clock diagnostics](evidence/governor/verification/scalar_diagnostic_stage01/report.json).
At +0.3, G's speed peaks at19.730s while desired speed is only .03869m/s and
no same-row command clips. Tilt peaks at20.070s during clock reacceleration,
with desired acceleration .07785m/s² and motor request−.6594Nm clipped to−.6Nm.
These associations do not isolate the full cause of the regression.

[Compact manifest](evidence/manifest.json) binds159 evidence files plus itself.
Its first152-file capture is retained before adding final raw-bundle closure
metadata. [Exact source/metadata archive](evidence/source-and-metadata.tar.gz)
contains1,543 independently hash-checked members. The separate
[full raw evidence bundle, 172.7MB](../../../../results/path_progress_governor_20261128/publication/full_evidence_stage01/i78_full_evidence.tar.gz)
contains1,962 verified members, including original I-76 inputs, all six new
rollouts and videos. SHA256:
`31a82810e8981312ae79e2fb9d781fb0cef55f21f62aaedec231e782a0fa369e`.
[Archive verification](evidence/publication/full_evidence_stage01/archive_verification.json) ·
[inventory](evidence/publication/full_evidence_stage01/archive_inventory.json) ·
[prerequisites](evidence/publication/full_evidence_stage01/external_prerequisites.json) ·
[omissions](evidence/publication/full_evidence_stage01/excluded_files.json) ·
[restoration instructions](evidence/publication/full_evidence_stage01/REPRODUCE.md).
Three scoped archive-integrity contracts also pass; they are not physics tests.

Re-evaluate existing sealed data into a **new** output directory:

```bash
.venv/bin/python results/path_progress_governor_20261128/evaluation/evaluate_stage02.py \
  --protocol results/path_progress_governor_20261128/protocol/protocol.json \
  --seal results/path_progress_governor_20261128/campaign/outputs_sealed.json \
  --output results/path_progress_governor_20261128/evaluation/new_review
```

Exact result-owned sources are archived. Restore them to recorded paths with
separately identified I-76 prerequisites; do not overwrite closed trials.
Current-source reruns are new experiments. No full repository suite is claimed.
