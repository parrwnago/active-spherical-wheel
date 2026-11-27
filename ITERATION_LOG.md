# Controller and contact-estimation iteration log

This records the research process: what we expected, what we tried, what the
evidence showed, and why the next experiment changed. Unsuccessful candidates
remain part of the record.

First backfill compiled **2026-11-25, America/New_York**, from the conversation,
source review, and saved artifact families `20261124` and `20261125`. The IDs
describe the research sequence; they do not assert exact local start times.
The two-contact fixtures were developed alongside the diagnostic study.

- [Presentation notes and selected clips](docs/iterations/2026-11-25/README.md)
- [Evidence manifest, source paths, sizes, and hashes](docs/iterations/2026-11-25/evidence_manifest.json)
- [All 30 focused trial rows, with campaign and iteration IDs](docs/iterations/2026-11-25/all_trial_metrics.csv)
- [Controller details and reproduction options](OFFROAD.md#estimated-contact-correction-diagnostics)

## Research question and starting point

Can the spherical wheel balance and follow trajectories using contact estimates
and inputs that could eventually be available on a real robot?

The existing controller is **gain-scheduled LQR with contact-aware feedforward**.
This work did not introduce LQR. There is no physical robot yet. Simulated body
pose/velocity and joint measurements are exact, and the belt drives are ideal.
The simulation does not provide electrical motor-current measurements.

The controller mode matters:

| Mode | Contact information available to control |
| --- | --- |
| `terrain_aware` | Solver wrench, contact locations, and load shares. This remains the global `OffroadParams.mode` default. |
| `momentum_observer` | Estimated net wrench, but solver contacts still inform rolling feedforward, gain blending, and synchronization. |
| `hardware` | Estimated wrench, contact records, and load shares through a contact-free `StateMirror`. Explicitly selected for our estimated-contact studies. The name does not imply hardware validation. |

The rolling allocation default remains `least_squares`. Every damping/bounded
candidate below is an opt-in experiment.

## Iteration overview

| ID | Question or change | Observation | Decision |
| --- | --- | --- | --- |
| I-01 | Audit measurement sources and run the existing modes | An observer wrench alone did not remove all solver-contact dependencies; estimated-contact mode could be tested separately. Initial MuJoCo evidence was distinguished from Gazebo evidence. | Establish the correct controller/data baseline. |
| I-02 | Add more Mars trajectories and starting phases | 19/21 runs completed, but only 2/21 passed every quality gate. | Investigate corrections rather than equating completion with good tracking. |
| I-03 | Log command components and velocity consistency | A modest requested body velocity could produce extreme opposing speed references near aligned drive directions. | Isolate allocation demand and estimator/transition behavior. |
| I-04 | Test controlled two-contact geometry | Two rims were represented well; a V-groove exposed the ambiguity of one equivalent support point. | Document the estimator's multi-contact limitation. |
| I-05 | Add uniform damping, coefficient 0.01 | Reference spikes decreased, but some tracking and attitude results worsened. | Retain the original allocation default. |
| I-06 | Bound references; add speed/change costs, both 0.001 | Bounds held, but the difficult +0.90 figure-eight regressed. | Test whether a higher speed cost avoids excessive demand. |
| I-07 | Increase the speed cost to 0.005 | +0.90 improved substantially; triangle tracking worsened. | Do not apply the stronger cost globally by default. |
| I-08 | Adapt speed cost from 0.001 to 0.005 using geometry | Triangle tracking recovered, but the -1.00 figure-eight regressed. | Preserve this unsuccessful candidate and investigate its correction event. |
| I-09 | Decompose the remaining large correction | Support identity switched rapidly while attitude/velocity feedback dominated belt demand; allocation residual was tiny at the inspected onset. | Inspect support transitions and gain blending next. |
| I-10 | Next experiments | Proposed, not executed. | Test one transition-continuity change against the retained baseline. |
| I-11 | Review the user's tilt/configuration hypothesis | Same platform attitude permits strong or singular geometry; earlier weak geometry preceded lean-error growth in the adaptive regression. Offline review only. | Map configuration, requested direction, and reference-relative balance error before choosing the next weight policy. |
| I-12 | Execute the configuration/direction map | 17,739 kinematic configurations, 141,912 direction cases, and 6,560 boundary samples show direction-dependent feasibility and finite shell/belt boundary jumps. | Prefer configuration-and-direction evidence to a tilt-only schedule. |
| I-13 | Check whether the smooth coefficient produces smooth behavior | Saved contact-map and feedback changes remain abrupt; several switches are actual physical contact changes. | A smooth scalar formula alone does not make the whole controller continuous. |
| I-14 | Run short controlled Gazebo comparisons | 40/40 completed; difficult-direction tracking remains poor with the existing adaptive allocator. Initialization also confounds the nominal pitch perturbation. | Retain the limitation and all cases; do not claim a new successful weight law. |
| I-15 | Next targeted continuity experiment | Proposed, not implemented. | Test one smooth shell/belt feedback-transfer rule while keeping the contact estimator and allocator fixed; particle filtering remains deferred. |
| I-16 | User correction: the original singularity involves shaft and hemisphere drives at one contact | I-15 focused on a separate handover issue. Fixed-hemisphere evidence already demonstrates the geometric loss of a travel direction. | Prioritize configuration/direction-aware allocation and consistent motion references at fixed hemisphere contact; test handovers separately afterward. |

## I-01 — Audit the controller and establish simulation evidence

**Question.** Does using a momentum observer mean the controller no longer uses
simulator-only contact information?

**Work.** Reviewed the existing data paths, distinguished the three modes above,
and ran existing controller tests. The first four ramp/hills runs were MuJoCo
runs, not Mars-yard Gazebo runs. That scope distinction was corrected explicitly.
They completed, and the associated selected test run passed 19 checks.

The subsequent Gazebo setup checks passed 9 tests. Four circle/crater runs
compared `terrain_aware` and `momentum_observer`: both circle runs completed, but
the observer crater run fell at 25.35 s while the force-aware run completed
55.5 s. These modes still used solver contact records for rolling feedforward.

Two separate Gazebo demonstrations explicitly selected `hardware`. Both completed:
circle peak/final error 128.9/2.1 mm; crater climb 674.5/31.3 mm. Contact-truth API
guards checked 19,102 controller updates. The crater's large peak error remained
visible despite completion.

**Learning and decision.** Keep simulator/measurement provenance explicit. Use
the estimated-contact mode for the next experiments; do not describe it as a
tested real-robot sensor pipeline. Physical encoder/body-motion capture is
future work because there is no robot.

**Evidence:** [initial MuJoCo report](docs/iterations/2026-11-25/initial_mujoco_summary.json),
[Gazebo observer crater failure](docs/iterations/2026-11-25/initial_gazebo/crater_rim_climb_momentum_observer.json),
[Gazebo force-aware crater run](docs/iterations/2026-11-25/initial_gazebo/crater_rim_climb_terrain_aware.json),
[estimated-contact demonstration report](docs/iterations/2026-11-25/feedforward_summary.json).

## I-02 — Broaden the trajectory baseline

**Hypothesis.** More trajectory shapes and starting phases will reveal where
corrections occur and how much the outcome depends on contact transitions.

**Experiment.** Existing Gazebo/DART Mars trajectories, explicitly using
`hardware`: all 15 routes at phase +0.30 rad, plus the figure-eight, triangle,
and out-and-back at +0.90 and -1.00 rad, for 21 runs.

**Result.** 19 completed; 2 passed all acceptance gates. The steep descent fell
at 9.785 s and the lava crossing at 11.240 s. Guards checked 260,353 controller
updates. The extra phases are different initial configurations, not repeated
identical-start trials.

**Learning and decision.** Route completion was encouraging but insufficient.
Retain attitude, settling, saturation, and trajectory-error gates. Use three
figure-eight phases and two simpler shapes as a focused correction study.

**Evidence:** [21-run report](docs/iterations/2026-11-25/trajectory_summary.json),
[trajectory figure](docs/iterations/2026-11-25/figures/trajectory_paths.png),
[original run definition](docs/iterations/2026-11-25/reproduction/trajectory_suite.py).

## I-03 — Make the corrections observable

**Hypothesis.** A correction could start with contact-estimate changes,
unachievable rolling commands, actuator clipping, or reference behavior. Logging
the separate contributions should distinguish these possibilities.

**Change.** Added optional estimated-contact/load logging, rolling references,
LQR/feedforward/support/transition components, clipping, and input alignment.
Added the proposed velocity-consistency diagnostic using measured motion,
estimated contact geometry, joint rates, and preceding applied belt speeds.
It initially only records disagreement; it does not alter commands.

**Finding.** In the D10 baseline +0.90 figure-eight at 31.650 s, desired horizontal
velocity was about 0.066 m/s, but joint references included shaft +488.6 and
shell B -1760.5 rad/s. Both available motions at the estimated loaded point were
almost aligned. The reconstructed normalized Jacobian condition number was
about 3961. Only shell B was estimated to carry load; this did not require both
hemispheres to touch the ground. Large torque requests were then clipped to
the existing actuator limits.

These were **speed references, not measured motor speeds**. Geometry reconstructed
from saved float32 data is approximate; its inverse differs slightly from the
original logged inverse in this ill-conditioned configuration.

**Engineering corrections.** Logging now distinguishes allocator reference
velocity from the slope-adjusted feedback reference and uses the preceding
requested command for preceding-interval saturation. An initial instrumentation
test run had 1 failure/12 passes due to an invalid short-path/ramp test setup;
the saved trace also reflects source edits made while the old test process was
running. Later alignment checks passed. This was not evidence that the path
constructor ignored its arguments.

**Learning and decision.** Numerical feasibility without actuator/reference
limits is not enough. A velocity mismatch is not uniquely contact-radius error:
slip, clipping, support ambiguity, and motion uncertainty can also contribute.
No particle filter or mismatch feedback was implemented here.

**Evidence:** [singularity event](docs/iterations/2026-11-25/allocation_event.json),
[velocity-consistency figure](docs/iterations/2026-11-25/figures/velocity_consistency.png),
[initial test failure](docs/iterations/2026-11-25/tests/instrumentation_initial_failure.log),
[later alignment checks](docs/iterations/2026-11-25/tests/reference_alignment.log).

## I-04 — Check the multi-contact limitation

**Question.** Can a net force/moment estimate locate more than one actual support?

**Experiment.** Two controlled Gazebo fixtures: simultaneous equatorial-rim
support, and two contacts on one shell in a V-groove whose local normals differ
by 50°. These use exact solved net wrench to isolate contact-location geometry
from observer error.

**Result.** Rim points/load shares were represented within the tested tolerances.
In the groove, the two actual normals differed by 50°, but the locator returned
one equivalent support point more than 40 mm from either actual point. The net
wrench remained consistent with that equivalent point. Both fixture tests passed
because they checked these expected properties, including the limitation.

**Learning and decision.** A good net wrench/equivalent contact is not proof of a
correct contact distribution. General multi-contact traversal was not demonstrated.
Contact particle filtering remains a proposal to evaluate after diagnostics,
not an implemented estimator or an established solution.

**Evidence:** [fixture tests](tests/test_multicontact_diagnostics.py),
[saved fixture test run](docs/iterations/2026-11-25/tests/multicontact.log).

## I-05 — Uniformly damp the allocation

**Hypothesis.** A normalized drive-speed penalty can prevent large opposing rates
when exact velocity matching requires an ill-conditioned inverse.

**Experiment.** D10 campaign: the five focused baseline cases from I-03 and five
matching runs with damping coefficient 0.01 m/s per normalized drive unit.
Normalization was 5 rad/s for joints and 0.3 m/s for belts; these were preferences,
not hardware speed limits. Original timed trajectories were retained.

**Result.** All 10 completed; none passed every gate. At +0.90, maximum joint
reference fell from 1760.5 to 4.4 rad/s. But -1.00 figure-eight moving RMS error
increased from 51.1 to 86.4 mm, with peak error increasing from 116.3 to 361.8 mm.

**Learning and decision.** Suppressing the inverse's extreme requests did not
consistently improve the coupled controller. Keep `least_squares` as the default.
The next experiment adds explicit bounds and separates speed/change costs.

**Evidence:** [D10 table](docs/iterations/2026-11-25/diagnostic_comparison.csv),
[comparison figure](docs/iterations/2026-11-25/figures/uniform_damping_comparison.png),
[slow event clip](results/contact_diagnostics_20261125/videos/figure8_phase090_event_slow.mp4).

## I-06 — Bound references and penalize reference changes

**Hypothesis.** Tracking, normalized speed demand, and reference changes can be
traded explicitly while enforcing a finite reference envelope.

**Change.** Bounded least squares with load-normalized contact weights. Speed
limits `(20,20,20,.3,.3)`; acceleration limits `(100,100,100,3,3)` per second,
with joint channels in rad/s and belt channels in m/s. These are simulation
reference assumptions. Speed and change coefficients both 0.001. The LQR gains,
holding torques, and final plant actuator limits remain in place.

**Experiment.** W20 campaign began with a fresh five-case baseline and five
bounded runs. The complete W20 campaign also includes I-07 and I-08: 20 runs
total, not 20 additional runs at each iteration.

**Result.** All five bounded runs completed and obeyed both reference bounds.
Four cases had lower moving RMS error than the fresh baseline. The +0.90 case
regressed from 50.5 to 67.6 mm RMS and 190.9 to 246.7 mm peak error. An unloaded
shell reached 44.3 rad/s despite the 20 rad/s reference cap.

**Engineering corrections during review.** Fixed stale last-loaded references
being restored after stopping/restarting during a contact dropout. Fixed the
nonhardware Gazebo path to use its external contact provider. Added regression
checks and explicit matrix validation. Per-run source hashes retain these code
differences; later variants were not merely relabelled copies of this source.

**Learning and decision.** Reference feasibility is enforced, but actual joint
speeds and final belt-command acceleration are not bounded by this allocator.
Speed demand is a proxy, not torque/current/energy cost. Test stronger demand
penalization without promoting this candidate.

**Evidence:** [initial bounded report](docs/iterations/2026-11-25/comparison_initial.json),
[comparison figure](docs/iterations/2026-11-25/figures/comparison_initial.png),
[44.3 rad/s excursion still](docs/iterations/2026-11-25/figures/bounded_initial_excursion.jpg).

## I-07 — Increase the fixed speed cost

**Hypothesis.** The 0.001 speed cost still permits excessive demands near difficult
geometry; 0.005 may make temporary tracking error preferable.

**Experiment.** Five W20 runs, speed cost 0.005 and change cost 0.001, with the
same bounds, original timed paths, starting states, and fresh baseline.

**Result.** The +0.90 figure-eight improved: peak error 190.9 → 85.3 mm, final
error 190.2 → 19.9 mm, and maximum joint reference 1760.5 → 2.1 rad/s. However,
triangle moving RMS increased 24.4 → 35.2 mm and peak error 68.8 → 166.2 mm.

**Learning and decision.** One fixed penalty did not produce the desired tradeoff
on every route. Preserve both the improvement and regression; test a geometry
dependent coefficient rather than choosing a coefficient by route name.

**Evidence:** [fixed-cost report](docs/iterations/2026-11-25/comparison_effort005.json),
[all-case figure](docs/iterations/2026-11-25/figures/comparison_effort005.png),
[+0.90 event clip](results/weighted_allocation_20261125/videos/figure8_bounded_effort005_p090_event.mp4).

## I-08 — Adapt the speed cost to estimated geometry

**Hypothesis.** Keep a low cost when control authority is good, increasing it as
the horizontal drive directions align or their levers vanish.

**Change.** Blend speed-cost coefficient from 0.001 to 0.005 using
`threshold²/(s²+threshold²)`, with threshold 0.02 and `s` the smaller singular
value of the normalized, load-averaged horizontal contact Jacobian. The blend
changes the penalty on all normalized drive rates; it is not an independent
penalty applied only to the weakest singular direction.

**Experiment.** Five W20 cases, same timed references and bounds. No reference
governor or trajectory-clock slowing was applied.

**Result.** Triangle RMS returned to 23.4 mm, versus fresh baseline 24.4 mm.
The -1.00 figure-eight regressed: RMS 41.3 → 77.1 mm, peak error 104.9 → 317.8 mm,
and final error 63.9 → 87.7 mm. Bounds still held. Maximum actual joint speed
across the adaptive cases reached 32.85 rad/s.

**Learning and decision.** The plausible geometry-based rule did not consistently
improve closed-loop behavior. Keep it opt-in, retain the original default, and
inspect the regression rather than continuing coefficient searches. Each W20
setting completed 5/5 runs and passed every acceptance gate on only 1/5.

**Evidence:** [adaptive report](docs/iterations/2026-11-25/comparison_adaptive.json),
[all-case figure](docs/iterations/2026-11-25/figures/comparison_adaptive.png),
[negative-phase regression clip](results/weighted_allocation_20261125/videos/figure8_bounded_adaptive_m100_event.mp4).

## W20 comparison on the original timed trajectories

Moving RMS error, millimetres; lower means closer tracking. These are single
runs per setting, not estimates of statistical significance.

| Case | Fresh baseline | Fixed 0.001 | Fixed 0.005 | Adaptive 0.001–0.005 |
| --- | ---: | ---: | ---: | ---: |
| Figure-eight +0.30 | 57.0 | 51.8 | 62.7 | 60.7 |
| Figure-eight +0.90 | 50.5 | 67.6 | 46.2 | 55.5 |
| Figure-eight -1.00 | 41.3 | 38.6 | 38.8 | 77.1 |
| Triangle +0.30 | 24.4 | 23.8 | 35.2 | 23.4 |
| Out-and-back +0.30 | 34.0 | 30.0 | 33.5 | 33.9 |

W20 completed 20/20 runs with 214,020 guarded controller updates and zero
reference-speed/acceleration violations in all 15 bounded runs. D10 and W20
have separate baselines: for example, -1.00 baseline RMS was 51.1 mm in D10
and 41.3 mm in W20. Small Gazebo numerical changes can alter later handovers;
do not attribute that baseline rerun difference to an algorithm improvement.

## I-09 — Trace the adaptive regression

**Question.** Was the remaining large correction simply unachievable allocation
or the position loop fighting accepted velocity error?

**Evidence.** The adaptive -1.00 case peaked at 317.8 mm at 29.975 s. At the
inspected onset, 28.975 s, allocation residual was only 5.66e-7 m/s with no
active allocation bounds. Yet estimated support was belt A while instantaneous
solver support was shell A. The estimated identity switched shell/belt 18 times
over 28.900–29.385 s, despite a small equivalent-point error at the onset.

Belt-A LQR demand was -1.659 m/s, versus feedforward -0.0478 m/s, with the applied
command clipped at -0.3 m/s. Its largest reconstructed LQR contributions were
attitude (-1.006), linear velocity (-0.391), and angular velocity (-0.219 m/s);
position contributed -0.041 m/s. The reconstruction matched saved LQR commands
within 8.0e-6.

**Interpretation.** These observations do not establish a single cause. They do
show that allocation infeasibility or position error alone does not explain this
onset. Accurate equivalent-point geometry can coexist with uncertain surface
identity near a patch boundary.

**Decision.** Inspect estimated support transitions and the associated gain
blending before promoting an allocator or adding a more complex estimator.

**Evidence:** [event decomposition](docs/iterations/2026-11-25/adaptive_event.json),
[annotated event plot](docs/iterations/2026-11-25/figures/adaptive_event.png).

**Follow-up:** I-11 inspected earlier samples. The 28.975 s sample above was
already partway through the disturbance; its small residual does not rule out
an earlier period of weak allocation geometry. Retain this event decomposition
and use the extended timing in I-11 when discussing the full sequence.

## Verification record

These saved runs overlap. **Do not add their counts or call them one full-repo
test-suite result.** Simulations, fixture tests, and selected regression tests
answer different questions.

| Saved check | Outcome | Scope/evidence |
| --- | --- | --- |
| Initial observer/controller tests | 19 passed | [log](docs/iterations/2026-11-25/tests/initial_mujoco.log) |
| Initial Gazebo tests | 9 passed | [log](docs/iterations/2026-11-25/tests/initial_gazebo.log) |
| First instrumentation run | 1 failed, 12 passed | Invalid short-path test setup; [original failure retained](docs/iterations/2026-11-25/tests/instrumentation_initial_failure.log). |
| Diagnostic regression run | 74 passed | [log](docs/iterations/2026-11-25/tests/diagnostic_regression.log) |
| Later reference alignment checks | 2 passed | [log](docs/iterations/2026-11-25/tests/reference_alignment.log) |
| Two-contact fixtures | 2 passed | [log](docs/iterations/2026-11-25/tests/multicontact.log) |
| Weighted-allocation selected regression run | 97 passed | Earlier source/test snapshot; [log](docs/iterations/2026-11-25/tests/weighted_regression.log). |
| Final allocator tests | 40 passed | [log](docs/iterations/2026-11-25/tests/weighted_final_allocator.log) |
| Final hardware/kinematics/diagnostics integration checks | 24 passed | [log](docs/iterations/2026-11-25/tests/weighted_final_integration.log) |

## I-10 — Proposed next iteration, not executed

1. Quantify shell/belt identity changes, load/gain changes, and their timing
   relative to attitude corrections across all five focused cases.
2. Make one continuity/confidence change to support transitions or gain blending.
   Retain balance authority and the original baseline for comparison.
3. Reuse the five cases and inspect event decomposition as well as trajectory,
   attitude, demand, and settling metrics. Expand the route suite after a
   candidate passes the focused comparison criteria.

Particle/contact filtering, a reference-progress governor, real sensor noise and
delay, and general closed-loop multi-contact traversal remain proposals or
unvalidated extensions. No real-robot experiments have occurred.

## I-11 — User hypothesis: schedule weights by tilt or configuration

**Status:** Existing-log analysis and two instantaneous kinematic checks completed;
the broader configuration sweep and new controller policy remain proposed.

**Observation and hypothesis.** The user noted that platform tilt in the earlier
example changed the available drive directions, and suggested tilt-dependent or
configuration-dependent allocation weights. Tilt can change support geometry;
the question is whether tilt alone is a sufficient scheduling variable.

**Controlled kinematic check.** On flat ground, hold the complete platform
attitude fixed at roll -20°, pitch 0°, with the contact on shell B. Changing only
the common-shaft phase changes the two contact-motion directions:

| Shaft phase | Angle between shaft/shell contact motions | Normalized weak authority |
| --- | ---: | ---: |
| -34.38° | 76.83° | 0.46534 m/s |
| -90° | 0° | approximately zero |

Both points lie inside shell B's valid surface band. These are admissible
instantaneous poses, not claimed balanced equilibria or new dynamic tests.
Even the complete platform attitude does not uniquely determine rolling authority.

**Saved-trace check.** In the original +0.90 baseline, estimated support changes
belt B → shell B from 31.645 to 31.650 s. Reconstructed weak authority drops from
0.29888 to 0.00015122 m/s, while absolute platform tilt changes only 7.151° →
7.171°. The reference-relative lean error slightly decreases. An absolute-tilt
threshold would not explain this abrupt estimated-geometry change.

Looking earlier in the adaptive -1.00 regression refines I-09:

| Time | Absolute tilt | Reference-relative lean error | Logged allocation residual | Logged weak authority |
| --- | ---: | ---: | ---: | ---: |
| 28.425 s | 10.67° | 0.34° | 0.06000 m/s | 0.001516 m/s |
| 28.975 s | 2.96° | 9.69° | 0.000000566 m/s | 0.29987 m/s |

Lean error first exceeds 3°, 5°, and 8° at approximately 28.585, 28.770, and
28.895 s in the inspected window. Rapid estimated shell/belt switching begins
around 28.900 s. Thus weak geometry and a large allocation residual occur before
substantial lean error and before the recorded burst of switching. Later larger
absolute tilt is part of the subsequent motion. This timing does not prove one
causal chain; it changes which controlled experiment is most informative.

**Interpretation.** Use different signals for different questions:

- Estimated contact geometry, shaft configuration, and requested velocity
  direction describe which travel motions are difficult to produce.
- Error from the intended balanced lean, together with falling/lean rate,
  describes balance urgency. Absolute tilt alone can penalize an intentional
  slope-compensating lean or miss a platform that is too upright for the slope.
- Contact-identity confidence and transition history describe whether the
  geometry and gain selection are trustworthy.

The previous adaptive candidate already uses configuration through the contact
Jacobian, but changes one scalar penalty on all normalized rates. It does not
independently weight singular directions or add a reference-relative balance
priority. Merely substituting tilt for that scalar is not an established fix.

**Recommended next experiment.** Build a controlled map of platform roll/pitch,
shaft phase, ground normal, support identity, and requested travel direction.
Separate valid patch interiors from shell/belt boundaries. Compare tilt-only
predictions with Jacobian-based reachable-motion predictions, then run short
matched closed-loop cases to separate poor geometry from balance recovery and
contact handover effects. Keep the existing LQR gains as the baseline.

Use the map to select one subsequent change: direction-specific relaxation of
infeasible travel, a smooth reduction of travel priority when reference-relative
lean error/rate grows, or transition-continuity treatment when support identity
is uncertain. Do not change all three at once. This refines the experiment order
proposed in I-10; it does not claim a new successful weight schedule.

Direction-selective filtering has precedent in
[Buss and Kim's inverse-kinematics work](https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/SdlsPaper.pdf).
That is background for a candidate, not validation of this rolling/balancing robot.

**Evidence:** [offline review and source-input hashes](docs/iterations/2026-11-25/tilt-configuration-review/review.json),
[analysis source snapshot](docs/iterations/2026-11-25/tilt-configuration-review/analyze.py.txt),
[follow-up manifest](docs/iterations/2026-11-25/tilt-configuration-review/manifest.json).
Baseline authority is reconstructed from float32 state/contact logs; its exact
original allocation residual was not logged and is not claimed recovered.

## I-12 — Execute the configuration-and-direction map

**Hypothesis.** The geometry and requested travel direction explain difficult
motion more precisely than platform tilt alone. Small configuration changes
near a shell/belt boundary may also change the available channels abruptly.

**Experiment.** Offline kinematics with roll/pitch from -20° to +20° in 5°
increments, shaft phase -180° to +180° in 5° increments, zero yaw and shell
material angles, and flat plus two 10° slope normals. Eight horizontal travel
directions request 0.06 m/s. This gives 17,739 configurations and 141,912
direction cases; four refined boundary scans add 6,560 samples. Synthetic
contact geometry stays entirely offline and is never given to a controller.

**Result.** At the same -20° platform roll and shell-B support, shaft -90° still
permits x travel but loses the entire requested 60 mm/s in y. One shell/belt
boundary shows a roughly 30.16 mm/s bounded-feasibility jump across only
±0.0001° of shaft phase. The selector changes available channels even when
pose changes continuously.

**Checks and limitation.** Geometry, unique direction coverage, opposite-direction
symmetry, phase periodicity, and bounded-residual consistency checks passed.
There are 15,914 clear patch-interior states; rim fallback candidates are
separately labelled and excluded from the interior interpretation because their
candidate plane-height spread reaches 1.666 mm. Kinematic feasibility does not
establish dynamic balance, friction feasibility, or actual motor capabilities.
Uniform grid fractions are not probabilities of failures during motion.

**Pipeline correction.** The first coarse smoke run computed geometry but failed
in plotting because `Axes.set` was given `linthresh`. The call was corrected
and the smoke/full map completed. The original failed log was overwritten;
the preserved issue record states that it is reconstructed, not an original
trace. The numeric-run and later plot-only source versions are both preserved.

**Decision.** Use configuration and travel direction for the next policy design;
do not substitute an absolute-tilt threshold for the existing geometry metric.

**Evidence:** [map summary](docs/iterations/2026-11-25/configuration-study/map_summary.json),
[directional example](docs/iterations/2026-11-25/configuration-study/figures/directional_example.png),
[boundary scans](docs/iterations/2026-11-25/configuration-study/figures/boundary_continuity.png),
[audit](docs/iterations/2026-11-25/configuration-study/map_audit.json),
[smoke issue](docs/iterations/2026-11-25/configuration-study/smoke_plot_issue.json).

## I-13 — Distinguish a continuous formula from continuous control

**Question.** Does the existing smoothly varying geometry coefficient make the
recorded transition commands smooth?

**Analysis.** Reconstructed the contact maps and compared per-step changes in
desired velocity, estimated support, Jacobian, coefficient, references, and LQR
commands in the saved baseline/adaptive event windows. No physics was rerun.
Counterfactual baseline coefficients are labelled as diagnostics, not commands
that the baseline actually used.

**Result.** Median normalized contact-map changes are about 128–140 times larger
at patch switches than between switches. Desired velocity remains gradual.
The active adaptive coefficient changes by as much as 0.001025 in 5 ms, even
within one patch during rapid loss of leverage. Its formula is continuous in
its scalar input but does not constrain its rate of change over time.

The bounded drive references stay within their declared acceleration limits,
while adaptive LQR-command changes reach 5.79 actuator bounds per step. All
six estimated dominant-patch switches in the inspected baseline window match
actual dominant-patch changes. This is not exclusively estimator misclassification.

**Learning and decision.** Smoothing a scalar penalty cannot by itself smooth a
hard contact-map/feedback transfer. Investigate the feedback handover without
freezing an old contact identity or imposing a blanket torque slew limit.
The observations identify a hypothesis, not a proven cause of the excursion.

**Evidence:** [continuity analysis](docs/iterations/2026-11-25/configuration-study/continuity_summary.json),
[continuity figure](docs/iterations/2026-11-25/configuration-study/figures/continuity.png).

## I-14 — Controlled short Gazebo comparisons (C40)

**Experiment.** Existing least-squares versus existing geometry-adaptive bounded
allocation. Flat ground and `z = tan(10°) * y`; phases 0.60 and π/2; nominal
fork-pitch offsets 0° and +3°; hold or world-x/world-y travel. Travel ramps
smoothly to 0.06 m/s, starting at 0.5 s. Every case lasts four seconds.
Eight holds use the common baseline because zero allocator history/request
gives identical allocation references; 16 moving pairs give 40 runs total.

Fork pitch is compensated by the opposite shaft-angle change, preserving
initial physical carrier/contact geometry. Observer initialization is reset
consistently in every case. The first 0.3 s is reported separately as a cold
observer window. Initial slope placement also gives approximately 20.83 mm of
body-center/reference offset, shared within each matched allocator pair.

**Result.** 40/40 completed, with 32,040 guarded controller calls. Initial states,
estimated geometry, time grids, and references match within numerical precision
inside allocator pairs. All 16 adaptive runs obey reference speed/acceleration
bounds; actual joint speed still reaches 25.51 rad/s despite a 20 rad/s
reference cap. These are mechanism tests, not Mars-route acceptance results.

At the side-slope pole with zero nominal pitch offset:

| Travel | Baseline RMS error | Existing adaptive RMS error | Estimated switches, baseline/adaptive |
| --- | ---: | ---: | ---: |
| World x | 16.26 mm | 16.36 mm | 0 / 0 |
| World y | 275.07 mm | 278.52 mm | 66 / 74 |

At phase 0.60 on the same slope, x is instead the difficult transient and y
remains modest. Difficulty depends on the evolving configuration and requested
direction; world y is not universally the weak direction. The adaptive bundle
changes bounds, effort/change costs, and scheduling together, so these
comparisons do not isolate the scalar coefficient alone.

The illustrated poor y run also changes heading by about 87.8°, versus 4.5°
for x. Heading is free in these controller settings. These are evolving dynamic
configurations, not four seconds at a fixed Jacobian; handover, yaw evolution,
and direction-dependent authority remain coupled.

**Design limitation discovered.** The nominal +3° fork perturbation preserves
the true initial contact point but changes the cold-start estimated point by
about 6.45 mm and changes the lean reference. Every initial reference-relative
lean error is below 1.3e-6°. Consequently these cases do not isolate a large
balance-error priority condition. Retain them as initialization-sensitivity
evidence; a future balance-priority test needs a separate warm-observer design.

**Decision.** No controller/default change. The short runs support a
configuration/direction/transition investigation, but do not demonstrate a new
smooth schedule or general robustness. Keep particle filtering deferred.

**Evidence:** [C40 analysis](docs/iterations/2026-11-25/configuration-study/controlled_analysis.json),
[all runs](docs/iterations/2026-11-25/configuration-study/controlled_runs.csv),
[matched pairs](docs/iterations/2026-11-25/configuration-study/controlled_pairs.csv),
[directional traces](docs/iterations/2026-11-25/configuration-study/figures/controlled_directions.png).

## I-15 — Next single policy experiment, proposed only

Test a smooth shell/belt feedback-transfer rule, keeping the current estimated
contact geometry, gain library, reference bounds, and allocator coefficients
fixed. This targets an observed feedback discontinuity rather than assuming a
more elaborate contact estimator is required first. Continuous blending may
weaken a necessary balance response, so compare attitude recovery, command
steps, saturation, tracking, and completion on the controlled cases before the
five trajectory regressions. Do not freeze the contact identity or reduce all
balance torque to obtain a smoother trace.

Direction-specific travel relaxation, a balance-error/rate priority rule, and
anticipatory configuration margins remain distinct candidates. Do not combine
them into this first handover test. No new weight law, feedback-transfer policy,
reference governor, or particle filter was implemented in I-12–I-14.

## I-16 — Correct the next-test priority: common shaft and hemisphere

**Status:** User correction and revised experiment proposal; no controller change
or new simulation run.

The user correctly pointed out that the original issue also involves the common
shaft and the loaded hemisphere rotating about different axes but producing
nearly parallel motion at the same ground contact. A shell/belt handover can
bring the robot into such a configuration, but a handover is not required for
that geometric loss of authority. The fixed-shell-B poses and directional map
in I-11/I-12 already demonstrate it. The saved 31.650 s event did occur at a
selected support transition; this does not make transition smoothing a remedy
for the same-contact geometry itself.

**Correction to the recommendation.** The assistant over-focused the next step
on the later contact/feedback discontinuity. I-15 remains a recorded hypothesis,
but it is no longer the first proposed experiment for the original problem.

**Revised next experiment.** Use the existing valid fixed-shell-B configuration
sweep as an allocator regression fixture, varying shaft phase and requested
travel direction without changing the support patch. Compare a candidate smooth
configuration/direction-aware allocation policy with the existing bounded
scalar-cost allocation under identical reference limits and history. Permit
velocity error in the poorly driven direction while retaining useful motion
in the well-driven direction. Large opposing shaft/hemisphere references should
not be required just to preserve an infeasible body-velocity request.

The body-motion reference used by feedback must also be consistent with what
the allocation can achieve. Smaller joint references alone leave the body
velocity/position loops pursuing the original motion. If a managed trajectory
or progress governor is tested, record nominal and managed progress separately
and require eventual destination completion; it must not hide tracking loss or
stop forever in a weak configuration. This is a proposed design consideration,
not an implemented governor or validated solution.

Check demand, attainable velocity, and balance response in the fixed-support
case first. Then assess shell/belt handovers as a separate regression. Neither
direction-aware weighting nor a contact estimator can create an instantaneous
drive direction absent from the current geometry; reconfiguration may also be
needed. Keep particle filtering deferred and preserve the existing baseline.

**Evidence reused, unchanged:** [fixed-support directional example](docs/iterations/2026-11-25/configuration-study/same_tilt_example.json),
[directional feasibility figure](docs/iterations/2026-11-25/configuration-study/figures/directional_example.png),
[original shaft/hemisphere reference event](docs/iterations/2026-11-25/allocation_event.json).

## I-17 — Implement directional allocation and accepted motion feedback

**Date/status:** 2026-11-25; implemented as an opt-in prototype, not promoted.
This follows the user's authorization to proceed with I-16. It addresses the
original common-shaft/hemisphere same-contact geometry before a separate
handover policy. Particle filtering remains deferred.

**Change.** Add `rolling_allocation='feasible'`: a normalized Jacobian SVD
assigns smooth .001–.005 modal speed costs (authority threshold .02, change
cost .001), retaining the declared `(20,20,20,.3,.3)` reference speed and
`(100,100,100,3,3)` acceleration bounds. Feed feedback the allocated horizontal
body velocity rather than the original infeasible request. Project travel
position error and nominal lean acceleration using the geometric modal
response. Preserve original timed goals and report their errors independently.
Initial position-error cap .05 m, projected catch-up gain 1/s, speed cap .06 m/s.
The existing gain-scheduled LQR, holding torques and global defaults remain.

**Scope.** Exact reference agreement applies to the supplied single-contact
horizontal kinematics. Multi-contact agreement is only with the load-weighted
mean; per-contact residuals are logged. The response used for position and
acceleration is unconstrained geometry, not the bounded solution's derivative.
Projected nominal acceleration is not the derivative of accepted velocity.
No autonomous reconfiguration, friction guarantee, sensor realism or physical
robot validation is claimed.

**Evidence:** [implementation, settings and source provenance](docs/iterations/2026-11-25/fixed-contact-allocation/README.md).

## I-18 — Fixed-support regression and warm-observer physics fixtures

**Date/status:** 2026-11-25; run and analyzed; mechanism verified within scope.

**Experiment.** An 8 s synthetic phase sweep at fixed -20° roll holds an
interior shell-B contact through all 1,601 samples. Compare LS, bounded scalar
adaptive, and feasible allocation for x, y and diagonal requests (.06 m/s).
All 14 assertions pass: bounded references obey speed/acceleration limits,
accepted velocity matches allocated kinematics, the missing y component is
rejected at the pole, and useful x motion remains. LS reaches 3.31 million
rad/s reference near the singularity. Both bounded variants can reach 20
rad/s; the new evidence concerns feedback-reference consistency, not a newly
created direction. The sweep externally restores phase; the rejected-y
velocity integral (about 53.65 mm) is not measured travel or autonomous recovery.

**Physics limitation retained.** Eight warm-observer Gazebo cases compare LS
and the initial feasible setting on a 10° side slope after identical .8 s
warm-up, without resetting the live estimator or physical state. Long .60 s
and short .16 s pulses in x/y keep actual shell-A support with zero handovers,
but both families fail the declared sustained-weak-geometry gate. For the
16 mm y request, achieved travel is only 8.5%/8.9% (LS/candidate); for the
4.4 mm request, 69.4%/63.7% at motion end. These are transient mechanism tests,
not evidence of completing a blocked destination. Preserve both families.
The initial `.venv` invocation lacked `gz` bindings and stopped before physics;
the micromamba Gazebo environment corrected setup, with the traceback retained.

**Evidence:** [fixed-support plot and warm videos](docs/iterations/2026-11-25/fixed-contact-allocation/README.md#mechanism-tests--i-18).

## I-19 — Fresh five-case Gazebo baseline and initial candidate

**Date/status:** 2026-11-25; ten route executions completed, analyzed, not promoted.

Compare identical original timed Mars paths, starting states, engine/terrain
settings and exact-state `hardware` measurement surrogate. Three figure-eight
phases (+.30,+.90,-1.00), triangle and out-and-back use guarded controller calls
that forbid simulator contact APIs. Reference bounds pass for every candidate.
Four of five moving-RMS errors improve, but +.90 regresses from 50.49 to
161.62 mm (peak 190.90 to 504.18 mm; attitude 8.71° to 14.71°). Neither family
passes all acceptance gates on any case. Completion means duration without a
fall, not adequate tracking or settling. Actual candidate joint speed reaches
45.81 rad/s despite bounded references.

**Finding.** The +.90 projected-reference RMS is only 35.35 mm while original
RMS is 161.62 mm, with 454 mm maximum separation of the references. Reporting
only the modified reference would hide the regression. Keep nominal metrics.
This campaign's -1.00 baseline RMS is 56.05 mm; do not substitute older D10/W20
baselines. Single runs per setting do not establish statistical robustness.

**Evidence:** [complete matched table](docs/iterations/2026-11-25/fixed-contact-allocation/README.md#trajectory-results--i-19i-21).

## I-20 — Catch-up and position-cap ablations; wider-cap regression

**Date/status:** 2026-11-25; three targeted and five V2 executions completed;
no setting promoted. Original failed trials remain intact.

At +.90, disable catch-up, widen the position cap to 1 m, then combine those
changes. Moving RMS becomes 45.80, 40.23 and 46.97 mm respectively; none passes
all gates. No-catch-up ends 50.19 mm from the goal; the combined trial ends
139.28 mm away. Widening alone improves the initial candidate's lag, so test
that setting (V2: gain 1/s, cap 1 m) across all five cases.

V2 improves RMS in three cases, but +.30 RMS becomes 64.31 versus 56.96 mm
baseline, peak 295.91 versus 154.28 mm; out-and-back RMS becomes 72.34 versus
35.57 mm, peak 444.49 versus 99.98 mm. All five final errors are below 50 mm,
but 0/5 pass all gates. Endpoints alone conceal excursions. The V2 +.90 case
was separately executed and produced a byte-identical saved rollout to the
wide-cap ablation; this is not an additional statistically varied trial.

**Evidence:** [ablations and annotated regression video](docs/iterations/2026-11-25/fixed-contact-allocation/README.md#trajectory-results--i-19i-21).

## I-21 — Balance-based travel priority; retain baseline and record limits

**Date/status:** 2026-11-25; implemented default-off, five executions completed,
analyzed; prototype not promoted. No further coefficient search in this task.

Retain V2 and scale travel requests/projected errors/nominal lean acceleration
smoothly by `1/(1+((lean_error+.1*lean_rate)/3deg)^4)`. Existing LQR gains and
holding torques stay in place. The unsigned lean-rate metric can also reduce
travel during recovery; this is not a stability certificate. Some attitude
peaks improve (+.90: 8.27° V2 to 5.77°), but original-path RMS is worse than
baseline in all five cases. Triangle alone passes all gates (1/5). Nominal
goals and timing remain unchanged; improved final distances do not erase lag.

**Validation.** The synthetic fixture passes 14 assertions. The selected
pre-guard regression passes 145 tests in 244.24 s; after the optional guard,
the final targeted integration passes 110 tests in 50.01 s, including 46
feasible-reference tests. Counts overlap and are not a full-repo suite or an
additive total. One initial guard test expected suppression for a tilted pose,
but the startup observer's inferred normal made that pose match its intended
lean, so priority correctly stayed at 1. Supplying the fixture's intended flat
normal resolved the test without changing control. The setup failure in I-18
and test-fixture correction are recorded as iteration evidence; the latter's
initial failing stdout was not saved as a separate artifact.

**Decision.** Keep `least_squares` and the guard off by default. The original
same-contact reference inconsistency now has an implemented, kinematically
verified treatment, but the overall robot-level failure remains unresolved.
Twenty-three Mars executions produced 22 distinct NPZ payload hashes and
240,423 guarded controller calls; eight warm fixtures are separate. No change
here establishes autonomous escape from a persistent missing drive direction.
The next design question is deliberate configuration recovery/progress while
preserving balance, informed by these retained counterexamples. It remains
proposed, alongside the separate handover investigation; no particle filter
or new LQR has been added.

**Evidence:** [presentation record, plots, source manifests, tests and media](docs/iterations/2026-11-25/fixed-contact-allocation/README.md).

## I-22 — Proposed next study: separate wrench, contact model and control

**Date/status:** 2026-11-25; recommendation following the user's assessment of
I-17–I-21. Proposed only: no controller implementation or new simulation run.
The existing experimental settings and default are unchanged. Earlier
presentation archives remain frozen at their recorded stages.

**Priority correction:** I-23 below reviews the prior accuracy evidence raised
by the user and de-prioritizes this broad observer-first study. The proposal
is retained here as history, not the current first recommendation.

**Question.** Does better contact information materially improve the original
weak-direction response, or does the controller/contact model still fail with
accurate inputs? Kinematic allocation consistency alone does not establish
dynamic balance or explain the trajectory regressions. Current evidence also
includes real physical support changes; do not label all switching estimator
chatter or treat handover as the original same-contact mechanism.

**First experiment.** Keep the baseline controller, gains, actuator limits,
nominal goals and reference policy fixed. Compare normal momentum-observer
outputs with exact simulator net ground force/moment passed through the SAME
SupportLocator and ContactEstimator. Align wrench origin, frame and sampling
interval, preserve identical warm controller/filter history, and record the
injection point. This isolates wrench estimation more narrowly than changing
the entire controller mode or injecting all true contacts at once.

Use three separate conditions: a warm single-hemisphere weak-direction request,
a well-conditioned same-hemisphere control, and one isolated shell/belt crossing.
This gives six initial screening executions, not a robustness qualification.
Verify actual support, load and directional authority over each evaluated
window. The earlier warm fixture did not sustain weak geometry; do not count
another exit from the weak band as proof of persistent rank-loss recovery.
Avoid physical state resets, pinned shafts or imposed contacts that conceal
the actual balancing behavior. Warm-prefix equality and baseline repeat
variability must be checked before attributing small differences to the input.

**Conditional second comparison.** If exact wrench does not explain the
failure, separately supply true contact points/normals/load shares using the
same downstream conventions. This deliberately changes the locator/contact
representation as well as its accuracy, so label that contrast separately.
It is a diagnostic use of simulator truth; the intended controller retains
inputs that could eventually be estimated from robot measurements.

**Outcomes and decision.** Evaluate reference-relative lean/rate recovery,
original-goal progress/error, actuator saturation and command steps. Improvement
with exact wrench implicates wrench/normal estimation, not automatically a
need for particle filtering. Additional improvement with true contacts points
toward locator/geometry/contact representation. Comparable failure even with
true contacts prioritizes balance/reference/actuation dynamics and feasible
configuration recovery; it does not prove any one controller replacement.
Do not infer dynamic controllability solely from the horizontal drive Jacobian.
Select the next policy from these contrasts before another weight search.

**Rationale/evidence:** [I-17–I-21 results](docs/iterations/2026-11-25/fixed-contact-allocation/README.md),
[continuity audit and its causal limits](docs/iterations/2026-11-25/configuration-study/continuity_summary.json),
and [MIT notes on contact dynamics](https://underactuated.mit.edu/contact.html).
The literature supplies the modeling distinction between contact modes and
continuous dynamics; the proposed experiment and priority are our inference
from this repository's results, not a literature-validated fix for this robot.

## I-23 — Accuracy was already validated; correct the next-step emphasis

**Date/status:** 2026-11-25; user correction and review of existing evidence.
No new physics run, test, estimator or controller change. I-22's broad
observer-first experiment is not the current priority.

**Correction.** The user correctly recalled that we had already checked
contact estimation against simulation truth. The assistant's I-22
recommendation gave that evidence too little weight. Good point/normal
accuracy does not explain away every possible estimation effect, but the
existing results do not justify treating large estimation error as the leading
explanation for the original geometric failure.

**Verified evidence and scope:**

- Combined estimated-contact MuJoCo hardware-mode ramp run: observer force
  median error 0.00101 N and ground-normal RMS error 0.0832°. The hills run
  gives 0.00251 N and 0.2448°. These use exact simulated state/model; they are
  not physical sensor/model-mismatch tests.
- Passing hills/washboard locator tests require dominant-patch agreement
  above 98%, median load-share error below .001, and single-dominant-patch
  point-error median below .2 mm / 95th percentile below 2 mm. These isolate
  the locator using exact supplied wrench; do not present those tolerances
  as end-to-end observer accuracy on every terrain.
- In the original Gazebo +.90 failure window at 31.655 s, logged equivalent
  point error is 0.1518 mm and normal error 0.0280°. This is one instant,
  adjacent to the extreme-reference event, not a bound for the whole window.
  Full-run point-error p95 is 7.03 mm; window maximum is 25.13 mm.
- In a separate adaptive-regression onset at 28.975 s, point error is
  0.3155 mm and normal error 0.0198° while lean error is already 9.69°.
  Estimated support is belt A while instantaneous truth is shell A. Small
  equivalent-point error need not imply correct instantaneous drive-surface
  selection. Conversely, all six estimated switches in the inspected baseline
  continuity window match actual dominant-patch changes.
- The V-groove fixture still exposes a multi-contact representation limitation:
  even exact net wrench can yield one equivalent point over 40 mm from either
  true contact. Keep this separate from ordinary single-contact accuracy.

**Decision.** Retain the estimator for the next controller investigation.
Prioritize its dynamic balance/reference response in the original same-contact
weak-direction condition; assess shell/belt feedback transfer separately.
Use an oracle comparison only as a targeted sensitivity check if a particular
event's existing accuracy data warrants it. This changes the investigation
priority; it does not establish one cause or validate a replacement controller.
Particle filtering remains deferred. Existing source and presentation archives
are unchanged; this correction is a later addition to the research record.

**Evidence:** [preserved accuracy reports and manifest](docs/iterations/2026-11-25/contact-accuracy-review/manifest.json),
[locator test](tests/test_hardware_mode.py),
[separate observer impulse tests](tests/test_momentum_observer.py),
[adaptive event](docs/iterations/2026-11-25/adaptive_event.json), and
[continuity evidence](docs/iterations/2026-11-25/configuration-study/continuity_summary.json).

## I-24 — Targeted support-disagreement audit and saved-state command probe

**Date/status:** 2026-11-25; implemented, run offline and analyzed following the
user's authorization to investigate the quoted shell-A/belt-A event. No physics
or controller change in this stage. Original files and older archives retained.

**Question.** How long does the disagreement last, and how much does it affect
the balance command? Revisit the W20 adaptive-bounded figure-eight, phase -1,
using its saved states and exact archived feedback-law source.

**Timing correction.** Rows contain state, estimates and the next command
before stepping, so this is not a one-row logging offset. However, Gazebo's
reported contact is from the last completed 1 ms physics step; the observer
reconstructs the preceding 5 ms interval wrench and filters it at 100/s
(10 ms time constant). A different sampled support label is therefore a
support-information disagreement, not automatically geometric misclassification.

**Observed duration and order.** The 28.975 s label difference occupies one
sample; labels agree at 28.970 and 28.980. Its sample-held indicator lasts
5 ms, not proof of continuously wrong support between samples. In 28.8–29.45 s,
30 of 130 loaded comparisons disagree (150 ms sample-held), with 18 estimated
and 30 last-substep-truth switches. Lean error already exceeds 3° at 28.585 s,
5° at 28.770 and 8° at 28.895; the first sampled disagreement in this transition
window is 28.925. The recorded disagreement burst follows the initial lean
growth. Both load-share margins are 1 at the quoted event; that margin is not
a calibrated confidence measure.

**Geometry caveat.** At 28.975 s, the inferred point lies about 12.59 micrometres
inside the modeled belt band and the true point 16.34 micrometres inside the
shell band. Simply relabeling the same inferred point would violate its patch
definition. First isolate feedback load routing; treat a complete valid
contact-record substitution as a separate broader intervention.

**Validated fixed-state counterfactual.** Across 141 saved states, native
feedback reconstruction passes its checks (maximum LQR reconstruction error
1.33e-5 in command units). Replace only feedback support shares with last-step
truth, preserving inferred total load and all saved references/geometry. At
28.975 s, belt-A request changes -1.7065 to -0.04782 m/s; shell-A torque request
changes -0.06144 to +0.68338 N m, clipped at +0.6 N m. Demand moves onto the
loaded shell but saturation remains. Requested saturation occurs in 90/141
native versus 86/141 independent counterfactual states. These corrected
commands were not integrated; they do not demonstrate improved recovery.

**Evidence:** [targeted presentation/evidence index](docs/iterations/2026-11-25/support-identity-probe/README.md).

## I-25 — Four matched Gazebo runs: targeted feedback and full-record oracles

**Date/status:** 2026-11-25; four executions completed and analyzed. Diagnostic
comparisons only; no new deployable controller or default change.

**Conditions.** Same Mars figure-eight at phase -1.00, Gazebo/DART, exact-state
`hardware` surrogate, adaptive bounded allocation from the W20 event. Here
"baseline" means that existing experimental setting, not global least-squares
control. Gain library, path timing, initial conditions and actuator bounds stay
fixed. Preserve warm observer/controller/simulator history with no reset.

The declared intervention computes commands at 28.900–29.385 s inclusive,
whose last hold ends at 29.390 s. Collect actual support loads at all five
1 ms substeps per control interval in 28.8–29.5 s for every branch. No command
changes occur between those substeps. Fresh baseline saved float32 qpos, qvel,
commands, applied inputs, estimated loads and goals exactly match the historical
run over all 44 s. This is output reproduction, not a claim that every original
uncommitted source byte was recovered.

**Interventions.** First replace only feedback load shares, preserving inferred
total load, using (a) last-1-ms truth or (b) the full preceding-5-ms mean. Both
omit the observer's exponential wrench filter; estimated records, rolling feedforward and
synchronization stay native. Then test (c) the full valid last-step contact
record bundle, remapped by name onto the contact-free mirror. That changes
points, forces/load shares, rolling allocation, feedback, synchronization and
slip/rim inputs. The observer wrench remains native; its downstream normal/
friction filters can respond to changed slip/rim inputs. The latter comparison
does not isolate identity alone. Outside the window, return to native inputs;
physical and retained controller history naturally carry intervention effects.

| Full 44 s run | Moving RMS, mm | Peak path error, mm | Final error, mm | Peak reference-relative lean, deg |
| --- | ---: | ---: | ---: | ---: |
| Existing adaptive baseline | 77.08 | 317.78 | 87.66 | 13.91 |
| Last-step feedback shares | 73.54 | 311.58 | 34.44 | 14.48 |
| Interval-mean feedback shares | 75.11 | 340.87 | 206.53 | 16.76 |
| Complete last-step contact records | 79.52 | 322.67 | 48.61 | 14.56 |

**Outcome.** All four complete; all four fail existing acceptance gates. The
last-step feedback branch improves some tracking quantities but worsens peak
lean; the mean-share branch worsens peak error, peak lean and destination recovery.
Complete contact records improve final error but worsen RMS, peak error and
peak lean. Better endpoint accuracy alone is not acceptance. These single-case
diagnostic trials do not qualify general performance or a new estimator.

During the intervention itself, peak lean is 13.91° baseline, 14.48° last-step
feedback, 14.35° mean feedback and 14.56° complete records. Requested-saturation
fractions fall from .908 to .816/.837/.857, but immediate peak lean error does
not improve. Window lean RMS decreases slightly for mean feedback and complete
records; this is a tradeoff, not uniform deterioration. The complete-record
bundle differs from its own native locator
by up to 9.15 mm in equivalent point and 95.27 N in normal load during the
window; this is a broad instantaneous-truth versus filtered-estimate contrast,
not an isolated identity-label change.

At the original 28.975 s baseline sample, preceding-5-ms mean support is also
100% shell A (about 22.02 N), so that particular disagreement persists when
comparing with interval-mean truth. Wrench filtering and hard geometric patch
selection remain different operations from averaging true load shares.

**Validation and retained failure.** Four simulations exercise command-time
input guards and intervention/invariance assertions. Source stages, full-
precision warm-prefix comparisons, scope checks and first divergence times
are in the comparison report. There are 35,204 controller calls and 700 new
physics-substep load samples per run; these are not additional unit tests.
The first comparison analysis selected an oracle shadow even outside its
active window and failed an assertion. Correcting that analysis selection
resolved the check; failed output is preserved and physics was not rerun.
An additional analysis-only precision correction replaces a float32-derived
median timestep with the validated .005 s control period. Elapsed timestamps,
sample-indicator duration and actual held-command duration are now distinct;
the terminal observation is not counted as actuated. The previous analysis
stage is retained, with no physics rerun.
The fourth run has a later harness source snapshot; post-run documentation/
metadata clarifications are distinguished from exact launch sources.

Probe archives retain per-support and equivalent-point/load summaries, record
counts and force differences. Individual raw contact-record dictionaries and
all per-contact normals were not serialized; source and scope are recorded.

**Decision/limits.** The support-information difference demonstrably changes
commands. Correcting the selected feedback path, and then the full contact
bundle, in this late window does not resolve the excursion. This does not
rule out effects of earlier estimation errors, timing, or other contact
episodes. It also does not prove a particular dynamic-control or configuration-
recovery policy. Keep estimator and defaults unchanged; prioritize the
already-observed earlier balance/weak-geometry response when selecting the next
controller experiment. No particle filter or contact-smoothing policy is
adopted from this probe.

**Evidence:** [reports, comparison plots, videos, source stages and reproduction](docs/iterations/2026-11-25/support-identity-probe/README.md).

## I-26 — Proposed alternative: constrained predictive balance control

**Date/status:** 2026-11-25; recommendation requested by the user after I-25.
Proposed only: no MPC implementation, new test or simulation in this entry.
Existing estimator, controller and defaults remain unchanged. Earlier bundles
retain their recorded source/results; this proposal is a later addition.

**Rationale.** Instantaneous allocation/reference changes and the selected late
contact-input corrections did not consistently improve balance and tracking.
This supports testing a different control formulation, without establishing
that LQR or estimation is the sole cause. The current LQR already uses a
dynamic model. The proposed difference is optimizing a bounded sequence of
physical inputs over predicted state evolution, including input constraints
inside the optimization and allowing a temporary configuration/position change
whose benefit occurs later. An inverse rolling Jacobian is not a dynamic
controllability or balance-recovery model.

**Candidate.** A short-horizon constrained MPC prototype predicts platform
tilt/angular rate, body motion, common-shaft phase and relevant drive rates.
Optimize the actual simulated inputs: three motor torques and two ideal belt
surface-speed commands. Do not substitute arbitrary desired body acceleration
or pretend the belt inputs are torque-controlled motors. Retain the existing
LQR as the comparison and available fallback. Include balance, original-goal
progress, normalized input demand and command changes in the objective, with
the existing actuator limits. State/lean restrictions may need explicit slack
to report infeasibility rather than invent an attainable action. MPC does not
restore a missing instantaneous drive direction or guarantee stability merely
by its name.

**Small first experiment.** Reuse a valid warm-start same-hemisphere fixture
on simple terrain: weak world-Y request and a strong world-X control, with
the same .8 s observer warm-up and .6 s / 16 mm nominal motion pulse. Extend
recovery long enough to measure arrival at the retained original goal. Compare
one declared MPC setting with existing LQR before any broader route sweep.
A tentative prediction horizon is .3–.5 s, subject to short-rollout model
validation and measured solve cost, not an assumed real-time capability.

Warm observer/controller history, goals, actuator limits and plant assumptions
must match. No prescribed phase reset, pinned support or forced upright state:
allow the robot to move/reconfigure to balance. Record actual contact and
directional authority over time; the earlier warm fixture did not sustain weak
geometry and cannot become such evidence just by changing the controller.
The predictor must use its model and estimated support/ground, not live Gazebo
contact truth. Validate its predicted response locally before tuning policy
weights or interpreting a poor closed-loop trial.

**Go/no-go.** Predeclare balance recovery and original-goal error criteria.
Require useful progress/recovery in weak Y without worse peak lean or a
material strong-X regression; a controller that simply stops travel does not
pass. Report both original-clock error and eventual goal completion, actual
actuator-limit occupancy/speed, predicted-versus-measured motion, solve time,
infeasibility and fallback usage. Passing input bounds alone is not success.
Only after a convincing small-case comparison test the earlier five-route
set and then contact transitions. Preserve unsuccessful trials and fixed
candidate settings instead of starting another broad coefficient search.

**External context and limitation.** [MIT's trajectory-optimization/MPC notes](https://underactuated.mit.edu/trajopt.html)
describe dynamic prediction with input/state constraints.
[Karachalios and Abbas's ballbot predictive-control study](https://arxiv.org/abs/2402.12092)
demonstrates a related stabilization/reference-tracking formulation. These
support the method as a candidate; they do not validate our hemisphere/belt
mechanism, changing-contact model or proposed implementation. The recommendation
is an inference from this repo's results. Particle filtering remains a separate
estimation direction and is not the first change in this proposed comparison.

## I-27 — Implement estimated-plane dynamic prediction and constrained MPC

**Date/status:** 2026-11-25; implemented and unit-tested following the user's
authorization of I-26. Experimental modules and harness only; existing controller
defaults remain unchanged. OSQP is an optional `mpc` dependency, not a new
requirement for ordinary runs.

**Predictor.** Copy the measured-state model into private MuJoCo model/data,
enable its own physical contacts, and pose a plane from estimated point/normal.
Fit only the private plane offset within +/-3 mm to the inferred normal load.
No force is injected, no live terrain/contact truth is read, and no physical
robot/simulator state is reset. Optimize actual three torque and two ideal
belt-speed inputs. Private actuator clipping is disabled for finite differences;
the optimizer owns the real input bounds. Integration state is reset
deterministically between independent derivative evaluations.

**Initial optimizer/controller.** Condensed affine MPC, 20 stages x .02 s,
replanned every .02 s with observer updates every .005 s. Normalize costs on
body position/velocity, roll/pitch/rates, rotor rates, physical inputs and input
changes. Yaw and cyclic joint-position costs are zero. Height/normal-speed
coordinates follow the estimated plane. Native LQR is retained as shadow and
fallback; successful MPC directly commands final physical inputs without later
synchronizer/heading/fade modification. Bounds are hard; tilt and slew are
costs, not guaranteed hard constraints. A finite QP is not robot-level success.

**Validation.** Initial predictor 12 tests and optimizer 29 tests pass together
(41 tests). The selected Gazebo-environment integration passes 51, including
those tests and hardware-mode regressions. Counts overlap. An initial optimizer
test expected a negative command while a large previous positive input and
change cost correctly favored the positive bound; correcting that fixture did
not change optimization math. The failed stage is preserved.

On ten recorded warm X/Y states, replaying applied inputs through the estimated
plane model gives maximum 20 ms XY/attitude errors .0302 mm/.00774° and 100 ms
.374 mm/.1074°. This validates those local recorded-command predictions, not
aggressive .4 s optimized plans or general contact transitions. The predictor
and optimizer have separately preserved source/test stages.

**Evidence:** [MPC experiment record](docs/iterations/2026-11-25/mpc-balance/README.md).

## I-28 — Initial affine MPC: balance improves, original goal still fails

**Date/status:** 2026-11-25; four matched Gazebo executions completed and
analyzed; no candidate promoted.

Use the same .8 s LQR/observer warm-up on a 10° side slope, phase pi/2, then
request a .6 s / 16 mm pulse in weak world Y or strong world X and allow 4 s
recovery (5.4 s total). Both branches anchor the nominal body-center path once
to the measured center at .8 s. The original goal then stays fixed; no later
reference translation, prescribed phase change or physical state reset.
Exact warm-prefix state/input/observer comparisons and activation-state checks
pass. State measurements are still exact simulation surrogates, not hardware.

| Case | Active path RMS mm | Peak reference-relative lean deg | Final original-goal error mm | Final travel toward +16 mm goal |
| --- | ---: | ---: | ---: | ---: |
| Y LQR | 235.63 | 9.36 | 98.83 | -74.47 mm |
| Y affine MPC | 20.07 | 1.77 | 25.08 | -8.75 mm |
| X LQR | 10.37 | 1.39 | 5.30 | 15.52 mm |
| X affine MPC | 13.58 | .92 | 1.79 | 15.27 mm |

Y improves lean integral from 13.57 to .615 degree-seconds and solves every
generated plan without fallback, but ends displaced opposite the requested
direction, behind its starting position. This describes displacement, not
the sign of its final instantaneous velocity.
It fails the predeclared <=5 mm goal criterion. X improves final error and peak
lean but regresses active path RMS by about 31% and uses LQR fallback for
628/920 actuated active samples (68.26%). It fails the tracking/fallback gates.
All four runs complete without a fall; neither MPC case passes the protocol.
Fallback-assisted results must not be credited as MPC-only performance.

Planning cost is measured offline: successful-plan median update about .36 s
for Y and .43 s for X versus .02 s simulated replanning. X's all-update median
about .009 s is misleading because early calibration failures skip planning.
Do not present the implementation as real-time control.

**Model lesson.** Optimized-plan nonlinear replay shows much larger .4 s
disagreement than local recorded-command tests: Y p95 XY/rotation disagreement
about 13.85 mm/6.69°, X 33.99 mm/18.79°. A frozen local model can miss the
phase-dependent future control authority needed for goal recovery. Preserve
the improved balancing result and failed destination/tracking outcomes together.

**Evidence:** [initial cases, reports and videos](docs/iterations/2026-11-25/mpc-balance/README.md).

## I-29 — Investigate calibration failures; reject a gate-only change

**Date/status:** 2026-11-25; offline audits plus one new calibration-only X
Gazebo execution completed and analyzed. No cost-weight search or adoption.

The strict .2% static load-fit gate rejects 156/231 initial X plans. At four
exact failed states, normal-force versus plane-offset curves are deterministic
and monotone but jump across contact admission (up to 9.52 N across a 10 nm
scan interval). Nearest-load residuals of roughly .4–1.72 N occur at micrometre
offsets, far from the +/-3 mm limit. There is no ordinary root-fitting bug.

Replaying actual commands from those states still gives max 20 ms errors
.0370 mm/.0292° and 100 ms .2204 mm/.1250°. Thus a strict static-force fit
alone over-rejects some good nominal dynamic predictions. However, finite-
difference A matrices near the force gaps change drastically with epsilon:
spectral radii and long-horizon amplification can be enormous. Nominal
prediction accuracy does not establish derivative/model-planning accuracy.
Dropping normal-height/normal-velocity coordinates did not fix this in an
offline test; that proposed reduction was not adopted.

Test one engineering change only: permit a bracketed nearest-load fit with
residual <=10% of estimated load, away from the offset bound, retaining the
strict-fit diagnostic. Keep affine cost/horizon/control settings unchanged.
Reuse the original X LQR baseline, explicitly not a new baseline execution.
Fallback falls to 28/920 samples (3.04%), but final goal error is 29.54 mm,
path RMS 19.73 mm and peak lean 3.82° versus baseline 5.30 mm/10.37 mm/1.39°.
This trial fails. Do not adopt gate relaxation by itself. A duplicate Y run
was unnecessary because every original Y plan already passed the strict gate.

An additional solver correctness check accepts backend success only when a
normalized external box-KKT stationarity test passes. A mock stale-Hessian
backend verifies rejection; no PSD repair or objective rescaling is hidden.
Retrospective checking finds none of 304 accepted initial plans would fail
the new gate. That does not certify their model accuracy. LTV/KKT solver
tests and previous source snapshots are separately preserved.

**Decision.** Preserve the rejected gate-only trial. The next targeted change
is trajectory relinearization with explicit nonlinear candidate scoring,
rather than another weight adjustment or reliance on QP status alone.

**Evidence:** [force/derivative audits and gate-only trial](docs/iterations/2026-11-25/mpc-balance/README.md).

## I-30 — Trajectory relinearization and nonlinear plan screening

**Date/status:** 2026-11-25; implemented, two targeted executions completed and
analyzed. No default change; both candidates still fail original-goal recovery.

Retain the .4 s horizon, .02 s stage/replanning periods, costs, limits, goals
and estimator. Recompute stage-dependent dynamics along the nonlinear warm
trajectory in a common root tangent chart. Evaluate Jacobians at nodes
0/5/10/15/19 and interpolate between them; retain exact nominal nonlinear drift
at every one of 20 stages. This is an explicitly approximate five-knot SQP
model, not a claim to exact hybrid/contact optimization.

At the first plan only, include two bounded predicted torque seeds: common-
shaft +/- .03 N m for .12 s. They never impose a pose or phase on the physical
plant. Refine seeds/warm start with one QP step, evaluate line-search fractions
1/.5/.25 with the private nonlinear dynamics, and select the lowest finite
nonlinear cost among the bounded sequences. Later updates shift the previous
plan and repeat one refinement. Record seed/warm/SQP command-source counts,
QP failures and actual fallback separately. An accepted warm/seed sequence is
not a solved QP or a stability certificate. Use the audited nearest-load gate
with its remaining derivative limitations visible.

Optimizer LTV tests use independent uncondensed-KKT examples and preserve
bit-identical LTI outputs on tested cases. Predictor tests check common-chart
derivatives, exact nominal replay and interpolation knots. Initial versions,
failed fixtures, force/derivative audits and new launch snapshots are retained.
Weak Y and strong X are the same matched fixtures; the original LQR baselines
are reused and their histories must match. No further policy/weight search is
planned in this experiment.

**Results versus the reused original LQR baselines:**

| Case | Active path RMS mm | Peak reference-relative lean deg | Final goal error mm | Final travel toward +16 mm |
| --- | ---: | ---: | ---: | ---: |
| Y trajectory MPC | 19.63 | 1.72 | 24.97 | -8.79 mm |
| X trajectory MPC | 9.18 | .84 | 8.16 | 8.87 mm |

Both finish without physical failure and with zero LQR fallback. Each accepts
230 actuated nonlinear sequences, with 232 solved actuated QP calls (three
initial seeds), not 232 independent accepted plans. Counts including the
unactuated terminal observation are 231 accepted plans/233 QP calls. Y applies
900 SQP-refined and 20 warm-sequence control intervals; X 908/12. No initial
seed sequence alone is applied. Peak lean and active path RMS improve in both
directions, but neither meets the original <=5 mm destination criterion. No
broader Mars suite or default promotion follows these failed small-case gates.

For chosen plans, private affine/nonlinear .4 s disagreement p95 improves to
Y .247 mm/.079° and X .537 mm/.197°. This is internal plan consistency, not
full-horizon validation against Gazebo truth. Median update cost is about
1.64 s/Y and 2.02 s/X per .02 s simulated replan: explicitly offline. Current
integration passes 69 selected tests (43 optimizer +16 predictor +10 existing
hardware-mode tests), overlapping earlier stages, not a full-repo suite.

**Remaining question.** Saved Y nonlinear plans themselves forecast retreat
from the goal during recovery, even as they improve the declared aggregate
cost. This is not solely unexpected plant drift. The finite-horizon objective,
terminal tracking design, local optimization and feasible recovery trajectory
need further investigation; neither physical goal infeasibility nor a specific
replacement objective is established. Preserve the real balancing improvement
alongside the failed goal test. No further controller/weight search in this task.

All seven new Gazebo executions are retained: two initial baselines, two initial
MPCs, one gate-only X trial and two trajectory MPCs. Later comparisons reuse
the initial baselines explicitly. Exact warm histories, input bounds and
diagnostic-component reconstruction pass. All motion pulses retain actual
shell-A support, but none sustains the declared weak-authority band throughout
peak demand. Do not claim permanent-singularity recovery or hardware robustness.
Post-run CLI/error-path/metadata clarifications are distinguished from seven
immutable prelaunch source snapshots; physical control math is unchanged.

**Evidence:** [MPC iteration/evidence index](docs/iterations/2026-11-25/mpc-balance/README.md).

## I-31 — Proposed next experiment: reach and hold a consistent goal state

**Date/status:** 2026-11-25; recommendation after the user reviewed I-30 as
promising. Proposed only; no new controller, equilibrium solve or simulation
performed in this entry. The I-27–I-30 presentation pack remains frozen.

**Observed motivation.** After the nominal motion pulse has stopped, chosen Y
plans sometimes predict increasing goal distance: at 1.4 s about 6.39 to
9.69 mm, and at 2 s 11.88 to 15.82 mm. Thus this behavior is present in the
controller's own nonlinear forecast, not solely unexpected plant drift. Slowing
the path clock alone does not address a rest-goal tracking problem.

**Concrete design concerns, not proven causes.** The target builder combines
goal XY with current height/internal configuration, inherited analytic lean and
zero rates without verifying bounded-input equilibrium. The effort cost
penalizes absolute input toward zero, while a slope can require nonzero holding
torque. Therefore arriving at the desired rest state is not necessarily the
zero-error/zero-regulation-cost operating point of the existing formulation.
Neither concern proves why the observed trajectory retreats.

**One scoped next experiment.** From preserved post-pulse states and the same
estimated plane, check whether the original goal/rest reference admits bounded
holding inputs. Evaluate all acceleration residuals, especially unactuated
base rows, support/slip feasibility and short hold drift. If inconsistent,
find a nearby balanced pose/configuration and holding input with original goal
XY fixed. Check local recovery/stabilizability rather than accepting a static
fit alone. No reset or prescribed phase change is applied to a live run.

Then compare one equilibrium-centered tracking formulation: track the verified
state and holding inputs, penalize unnecessary input deviations, and derive a
terminal value from its local dynamics where valid. Keep the estimator, horizon,
stage-weight magnitudes, actuator bounds and original goal/timing fixed. An
equilibrium does not establish reachability from the current state or within
the .4 s horizon; do not impose an unjustified hard arrival deadline.

Re-run the same warm X/Y cases and retain <=5 mm final original-goal error,
no worse peak lean, <=5% fallback and no >10% strong-X path-RMS regression.
Temporary retreat required by balancing remains allowed; stopping or ending
far from the goal does not pass. If the existing reference already holds
accurately, focus the contrast on terminal value/recovery planning instead of
declaring it infeasible. Disturbance/offset estimation is a later branch if
the model can hold the correct target but Gazebo shows systematic unpredicted
drift. Runtime optimization and broader terrain trials follow functional
goal-recovery success.

**Evidence:** [saved own-model goal forecasts](docs/iterations/2026-11-25/mpc-balance/evidence/trajectory/predicted_goal_progress.json),
[target builder and nonlinear cost](scripts/run-mpc-comparison.py), and
[optimizer input cost](src/spherical_wheel/predictive_control.py).
[Kuntz and Rawlings's tracking-MPC treatment](https://sites.engineering.ucsb.edu/~jbraw/jbrweb-archives/tech-reports/twccc-2024-04.pdf)
supports steady-state state/input targets and corresponding regulation costs.
Its convergence assumptions are not established for our nonsmooth contact
model; it supplies design context, not a guarantee for this proposed robot test.

### I-31 authorized follow-up — implemented; five-anchor private audit completed

**Question/change.** The user authorized the proposal. Added optional physical
input-reference tracking and an added terminal quadratic on final state and
normalized input error, including cross terms for a command-change-aware tail.
Default solver behavior stays exact in six preserved constant/LTV comparison
calls. Added a bounded equilibrium finder and independent forward-hold audit;
these do not activate a new closed-loop controller.

**Fixed conditions and source.** Reused I-30 trajectory-MPC X/Y states, original
destination and estimated supports, not new trajectory executions. Each anchor
keeps its calibrated plane fixed while fitting all five bounded inputs and
making one nearby height/tilt/shaft-phase fit. Pose bounds are ±3 mm, ±0.08 rad
tilt and ±0.35 rad shaft phase. There is no live-state reset or estimator change.
The protocol and exact source are preserved before the source-backed audit.
The root pre-change harness and predictor agent's later snapshot are labeled
separately because the latter already includes optional objective hooks.

**Predeclared prerequisites.** Over 0.2 s, peak XY drift <=0.2 mm, attitude
change <=0.1°, body speed <=2 mm/s, angular speed <=0.02 rad/s and each internal
rate <=0.05 rad/s, plus valid support and physical input bounds. A separate
componentwise raw-acceleration threshold is recorded; every selected candidate
already fails the hold limits, independently of that extra condition.

**Observed negative result.** No verified target in five saved configurations:

| Anchor | Best-found peak XY drift | Peak attitude change | Peak body speed |
|---|---:|---:|---:|
| Y1.4 s | 1.513 mm | 0.608° | 18.18 mm/s |
| Y2 s | 1.394 mm | 0.552° | 16.72 mm/s |
| Y3 s | 1.384 mm | 0.561° | 16.49 mm/s |
| Y5 s | 1.224 mm | 0.501° | 14.50 mm/s |
| X2 s | 0.986 mm | 0.426° | 11.99 mm/s |

All five best-found candidates retain positive support and bounded commands;
all fail position, attitude and rate limits. Nearby solves do not improve the
declared hold-violation score. A separate properly scaled Y2 search converges
numerically but also fails: 0.995 mm /0.505° at 0.2 s and
8.114 mm /4.248° at 0.4 s. Preliminary Y2 fits using different residual scaling
and a root cross-check are retained, not substituted for the five-anchor audit.
These overlap the same saved Y2 state and are not new independent trajectories.

**What this establishes/limits.** Current best-found rest targets are not
verified operating points of the private prediction model under the declared
limits. Local nonsmooth least-squares fits, with mixed-unit acceleration
residuals and bounded nearby configurations, do not prove physical
unreachability. Zero-acceleration inverse dynamics requires about 3.17 N of
unactuated tangential base force at Y2; low-speed private contact behavior is a
specific concern, not an established sole cause of I-30's retreat.

**Decision.** The prerequisite fails. Do not derive a terminal value around an
unverified equilibrium, claim local equilibrium recovery, or launch the
equilibrium-based MPC performance candidate. Retain tested optional objective
support as preparation. A separately declared open-loop Gazebo/private-model
hold comparison investigates the contact-model concern; its explicit diagnostic
initialization is not a reset of an ongoing trajectory test.

**Verification/progress preservation.** The solver passes 61 tests in both
environments (same tests, not additive), including independent multistage KKT
and nonzero input/cross-tail checks. An overlapping selected integration suite
passes 89 checks before adding the equilibrium module tests. Seventeen focused
equilibrium checks pass in both environments. The final combined selected
integration suite passes **106 tests in 50.27 s** in the Gazebo environment:
61 solver +16 predictor +10 hardware-mode +2 nonlinear scorer +17 equilibrium.
This overlaps earlier suites, is not additive and is not the full repo suite.
Afterward, the scorer tests gained an optional-OSQP skip guard and both were
rerun successfully; runtime code was unchanged. Exact final-integration sources
precede that test-only adjustment. Preserve the initial overly tight analytic test fixture,
surface-velocity slice error and NumPy-bool serialization error with their
sources/logs. Corrected setup errors do not count as successful physics tests.
The bare `python` alias was unavailable; protocol creation was repeated using
the repository interpreter before physics.

**Evidence:** [I-31 presentation and evidence index](docs/iterations/2026-11-25/mpc-equilibrium/README.md),
[local declared protocol](results/mpc_equilibrium_20261125/protocol.json),
[five-anchor report](results/mpc_equilibrium_20261125/audit/report.json),
[equilibrium audit source](scripts/audit-mpc-equilibrium.py), and
[best-found holds figure](results/mpc_equilibrium_20261125/audit/goal_holds.png).

### I-31 cross-engine follow-up — completed open-loop hold diagnostics

**Purpose/conditions.** Check whether failed holds are exclusive to the private
prediction model before attributing I-30 retreat to that model. Two separate
pose/input pairs from saved Y2 s are initialized in a diagnostic Gazebo/DART
process on the same 10° side-slope/.04 m mesh. Hold fixed bounded torques for
0.4 s, no tracking controller or observer. The private model receives exactly
the returned Gazebo initial pose/twist/input, including its bridge's 1 µs idle
reset step; estimated private plane stays frozen. No ongoing performance run
is reset and no true contacts enter a controller.

| Pose/input pair | Engine | XY drift at 0.2 s | Rotation at 0.2 s | XY drift at 0.4 s |
|---|---|---:|---:|---:|
| Analytic pose, fitted torques | Gazebo/DART | 1.069 mm | 0.437° | 4.938 mm |
| Same pair | Private MuJoCo | 1.046 mm | 0.430° | 8.054 mm |
| Independent nearby fit | Gazebo/DART | 0.986 mm | 0.501° | 8.125 mm |
| Same pair | Private MuJoCo | 0.996 mm | 0.506° | 8.120 mm |

**Outcome/limits.** Both fixed-input targets fail in Gazebo as well as the
private model, with loaded shell-A contact maintained. This is not a
private-model-only failure for those pairs, does not identify the root cause
of I-30 retreat and does not establish physical goal unreachability. The
analytic-pose prediction differs materially by 0.4 s, so full-horizon accuracy
is not established by the close 0.2 s results.

**Failures/counts.** Two startup assertion failures are retained: first raw
pose matching, then zero velocity were too strict for the documented reset
microstep. Matching the actual returned state corrects the fixture. Two
completed holds initially logged zero normal loads because an exact-name
lookup missed `shell_A_smooth`; source, kinematics and logging failure are
retained. Prefix aggregation/raw force capture was corrected and both short
holds repeated. Repeated kinematics match exactly. Total **four completed
Gazebo holds, two distinct pose/input pairs**, plus two failed startup attempts;
these are not new closed-loop MPC trajectories.

**Decision.** Retain I-30 as the comparison controller, with its unresolved goal
error. Verify a physically consistent goal-holding operating state before
terminal design; the bounded nearby search did not exhaust configurations.
Do not infer that particle filtering or further weight tuning fixes this result.

**Evidence:** [sealed Gazebo/private report](results/mpc_equilibrium_20261125/independent_search/gazebo_hold/report.json),
[matching/count/failure audit](results/mpc_equilibrium_20261125/independent_search/gazebo_hold/comparison_audit.json),
[0.1x hold diagnostic video](results/mpc_equilibrium_20261125/videos/nearby_goal_hold_gazebo_vs_private.mp4).
The 4.05 s clip reconstructs 81 saved poses at 20 fps using MuJoCo visuals;
Gazebo/DART supplies the left-panel dynamics. It explicitly labels open-loop
fixed torques/no feedback, does not rerun physics and is not a controller
performance comparison. Video source, decoded final-frame check and hashes
are retained. A new I-31 evidence pack is separate from immutable earlier packs.

## I-32 — Feedback-on recovery after the demonstrated drift

**Date/status:** 2026-11-25; user-authorized, protocol declared before new
physics; implemented, run and analyzed. No controller tuning or algorithm
replacement was performed in this campaign.

**Question/correction.** The user correctly observed that the I-31 clip never
showed a return, used a uniform slope rather than Mars terrain, and showed a
tilted platform. It was a 0.4 s fixed-input hold with feedback off. An unstable
balance point can require feedback; open-loop drift does not establish that
the controller cannot recover or that a different algorithm is needed. The
nonzero stationary acceleration residuals remain useful model/target evidence,
but are separate from a feedback recovery test. Tuning remains a possible
explanation; no causal conclusion is presumed.

**Declared comparison.** Reproduce the I-31 independent nearby fitted pose on
the same Gazebo/DART 10° side-slope mesh (.04 m grid), with original XY as the
fixed goal. For 0–0.4 s apply the same bounded fixed torques; run the existing
native observer/locator once each 5 ms using actual state/input history. At
0.4 s switch only the applied commands to either existing hardware-mode
gain-scheduled LQR/feedforward or existing I-30 trajectory MPC. Preserve state,
observer, estimator and synchronizer throughout. Observe another eight seconds.
No true contacts feed either controller. The ordinary design anchor remains
pi/2, and nominal joints are initialized once from the actual initial state.

**Acceptance/interpretation.** Show goal error and absolute world-up platform
tilt alongside its reference tilt and relative attitude error. A settled return
requires the final full second within 5 mm of the retained goal, XY speed
<=5 mm/s and reference attitude error <=2°, without a physical failure or input
bound violation. Also report first radius entry, minimum/peak/final error,
support history and fallback. A brief goal crossing is not a settled return.
The current reference intentionally leans on a slope; a horizontal platform is
not silently substituted as its objective. Neither failure nor success in
these two fixtures resolves all estimator/controller/tuning questions or
qualifies rough terrain. MPC still runs offline at its unchanged settings.

**Evidence:** [declared protocol](results/feedback_recovery_20261125/protocol.json).

### I-32 completed matched recovery — native succeeds, current MPC does not settle

The shared fixed-input history and feedback-switch state/observer match exactly.
Both simulations complete 8.4 s without a fall and with bounded physical inputs.
Both enter the 5 mm radius briefly at 0.435 s; their later behavior differs:

| Metric | Native LQR/feedforward | Current trajectory MPC |
|---|---:|---:|
| Goal error at feedback switch | 8.125 mm | 8.125 mm |
| Peak goal error during feedback | 32.788 mm | 88.278 mm |
| Final goal error | 0.381 mm | 31.215 mm |
| Final XY speed | 0.185 mm/s | 17.798 mm/s |
| Final-second maximum goal error | 0.381 mm | 49.518 mm |
| Final-second maximum XY speed | 0.518 mm/s | 19.423 mm/s |
| First complete one-second settling window starts | 4.67 s | None observed |
| Final absolute platform tilt | 9.732° | 9.739° |
| Final reference tilt | 9.731° | 9.731° |
| Final-second return gate | Pass | Fail |

The unchanged native controller recovers this demonstrated drift; the I-31
fixed-input hold therefore did not establish failure of feedback recovery.
The current MPC restores reference attitude but does not settle position in
the declared eight-second feedback window. It ends moving toward the goal,
so do not claim it can never return. Neither result guarantees indefinite or
rough-terrain stability. The platform's requested lean is not world-horizontal.

MPC has zero LQR fallback. There are 400 actuated accepted plans and 402 solved
QP calls, including three initial seeds; including the terminal observation,
401 plans/403 QP calls. A finite bounded nonlinear candidate can be selected
without proving global nonlinear optimality. Median planning takes about
0.997 s per .02 s simulated interval; this remains offline.

### I-32 longer original trajectories — extra time is not a uniform fix

After native recovery was observed, a separate protocol was declared before
extending the two original I-30 native trajectories from four to eight seconds
of post-motion recovery (total 9.4 s). All first 1,081 samples through 5.4 s
match the earlier baselines bit for bit across nine exact state/input/observer/
reference fields. The native controller/configuration source is unchanged.

| Original path | Goal error at 5.4 s | Goal error at 9.4 s | Final XY speed | Final-second gate |
|---|---:|---:|---:|---|
| Y | 98.826 mm | 5.804 mm | 1.721 mm/s | Fail |
| X | 5.300 mm | 139.607 mm | 298.623 mm/s | Fail |

Y benefits substantially from more time but still fails the declared final
one-second position/speed gate. X reaches about 1.01 mm at 7.12 s, then drifts
away later. Both physically complete without falling. No original-path MPC
extension is run here; these do not establish a matched longer-time trajectory
comparison with MPC.

The late X audit finds exactly zero motion, drive-speed and rolling-feedforward
references during 5.4–9.4 s. Error crosses 10 mm at 8.660 s; belt A first reaches
99% of its bound at 8.780 s, followed by true contact switching at 8.785 s.
At 8.885 s, truth and estimate both select belt A; LQR requests about
0.853 m/s, clipped to 0.3 m/s. Later shell-supported intervals also receive
synchronizer commands. Before 8.5 s patch identities agree and contact-point
gap stays below about 18.7 µm; later there are 21 identity-mismatch samples and
substantial belt saturation. Observer-versus-instantaneous-force differences
include filtering/timing effects. Correlation does not separate gain,
synchronizer, actuation, support-transition and estimator causes; no ablation
or tuning fix is claimed.

**Counts/verification.** Four new Gazebo executions: two held-drift feedback
cases and two extended original-baseline cases, totaling 7,124 guarded
controller calls. They are distinct fixtures; reused earlier baselines are not
added as new runs. Three focused recovery-metric tests pass. The earlier
106-test integration record is not rerun or added to this count. CLI help and
compile checks pass. Production controller/estimator code and parameters are
unchanged. Exact launch sources, test sources and raw results are preserved.

**Decision/evidence.** Keep native LQR/feedforward as the working baseline and
do not promote the current MPC. A successful local recovery and a late original
trajectory failure must both remain visible. The next native investigation
should target the repeatable late X event rather than accepting a short
near-goal frame or assuming extra time always resolves it.
[Presentation record](docs/iterations/2026-11-25/feedback-recovery/README.md),
[matched report](results/feedback_recovery_20261125/analysis/report.json),
[extended baselines](results/feedback_recovery_20261125/extended_original_baselines/analysis.json),
[late X audit](results/feedback_recovery_20261125/extended_original_baselines/late_x_audit/report.json),
[native recovery video](results/feedback_recovery_20261125/videos/native_feedback_recovery.mp4),
[matched LQR/MPC video](results/feedback_recovery_20261125/videos/feedback_recovery_native_vs_mpc.mp4).
Both clips reconstruct saved Gazebo physics at 0.5x with the feedback switch,
goal error and absolute/reference tilt explicitly labeled; rendering adds no
physics. Earlier presentation packs remain immutable.

## I-33 — Frozen-model and planning checks narrow the MPC issue

**Date/status:** 2026-11-25; implemented/analyzed offline follow-up to I-32.
Protocols and source were saved before each calculation. No additional Gazebo
executions, controller changes, weight tuning or estimator replacement.

**Question/change.** Distinguish unexpected prediction error from the intent
and quality of the selected MPC plans. Reconstruct the six estimated planes at
0.4, 0.6, 1, 2, 4 and 7 s, verifying saved states and offsets, and replay actual
future inputs at 20 ms stages. Each stage's command agrees across its four
5 ms measured samples. Compare saved Gazebo motion at 20/100/400 ms:

| Horizon | Median XY mismatch | Maximum XY mismatch | Maximum rotation mismatch |
|---|---:|---:|---:|
| 20 ms | 0.00261 mm | 0.00292 mm | 0.000758° |
| 100 ms | 0.01272 mm | 0.06491 mm | 0.01743° |
| 400 ms | 0.11547 mm | 1.45397 mm | 0.63864° |

Selected optimized forecasts themselves retreat: initial 0.4 s plan predicts
8.125 to 117.973 mm goal distance, the 2 s plan 44.903 to 58.604 mm and the 4 s
plan 87.407 to 88.098 mm. At 7 s the forecast approaches. These checks do not
validate Jacobians, prove a sole failure cause, or guarantee tuning success;
they do not support assigning the failed recovery solely to unexpected drift.

**A discriminating cost check.** At the initial 0.4 s anchor, reconstruct the
same model, target, weights, bounds, previous input and nonlinear scorer. The
selected plan's cost and every predicted state reproduce exactly. Evaluate two
bounded 20 ms command sequences obtained from the saved future native run:

| Offline sequence | Nonlinear cost | Predicted terminal goal error | Predicted absolute tilt |
|---|---:|---:|---:|
| Saved selected MPC | 24,525.748 | 117.973 mm | 54.317° |
| First native 5 ms command per stage | 41,777.448 | 133.215 mm | 53.488° |
| Mean of four native 5 ms commands per stage | 125.012 | 21.723 mm | 10.282° |

The averaged sequence costs about 196 times less under the unchanged objective
and obeys the same bounds. Thus the one-SQP search missed a feasible candidate
at this anchor; it is premature to conclude the weights require the observed
retreat. Simple first-sample downsampling is poor, so not every native-derived
seed helps. Saved future commands are diagnostic hindsight, not a deployable
policy, a new controller or evidence of a successful online fix.

**Decision/limits.** If MPC research continues, investigate feasible
initialization/nonlinear-search quality before further cost changes or a new
estimator. This counterexample does not prove a practical seed will fix the
closed-loop recovery. Keep native LQR as the comparison baseline and separately
diagnose its late contact-transition failure. Six replay anchors and three
offline candidate evaluations reuse I-32 traces; they are not additional Gazebo
cases. No model/controller source is changed. A JSON-label clarification is
preserved separately from calculations; exact cost equality is checked.

**Evidence:** [replay protocol](results/feedback_recovery_20261125/model_replay_protocol.json),
[replay report](results/feedback_recovery_20261125/model_replay/report.json),
[candidate protocol](results/feedback_recovery_20261125/candidate_cost_protocol.json),
[candidate report](results/feedback_recovery_20261125/candidate_cost/report.json), and
[presentation index](docs/iterations/2026-11-25/feedback-recovery/README.md).

## I-34 — Proposed priority: diagnose why the MPC search drifts

**Date/status:** 2026-11-25; proposed after the user's I-32/I-33 review. No new
controller, seed, solver change, tuning experiment or simulation in this entry.
Earlier evidence/presentation packs remain frozen.

**Direction/evidence.** Prioritize the MPC drift diagnosis; keep LQR as a
benchmark and the late native X event as a regression case. Current MPC makes
one SQP refinement per seed, initially from held previous input/small shaft
pulses and later from the shifted prior plan. Solving that local QP does not
establish nonlinear optimality. I-33's bounded candidate with 196 times lower
cost under the unchanged objective makes search/initialization a concrete first
target. Accurate actual-input replay does not validate the Jacobians. Saved
future LQR commands remain an offline counterexample, not an online seed.

**Proposed next experiment.** Freeze the failed 0.4 s and 2 s states. Keep model,
weights, horizon and bounds fixed. Compare one/two/five refinements and a
practical LQR-feedback rollout seed using estimated state/private predictions;
retain that feasible rollout as a candidate cost benchmark. If refinement does
not reduce actual nonlinear cost as expected, check directional derivatives and
sparse/full Jacobians before changing weights. Then repeat matched closed-loop
recovery. Include a 20 ms held-LQR comparison: current native feedback is 5 ms
while MPC holds inputs for 20 ms. Preserve the original 5 ms LQR result.

**Decision rule/limits.** Measure nonlinear cost reduction, settling, excursion,
attitude, bounds, fallback and runtime; record failed trials. No guaranteed
superiority or deployable fix. Cost/terminal/horizon tuning follows diagnosis
of the search contribution; no estimator replacement is justified by the
counterexample alone.

**Evidence/context:** [candidate counterexample](results/feedback_recovery_20261125/candidate_cost/report.json),
[one-refinement implementation](scripts/run-mpc-comparison.py),
[MIT trajectory-optimization notes](https://underactuated.mit.edu/trajopt.html),
[linear MPC/LQR terminal-design example](https://www.mathworks.com/help/mpc/ug/using-terminal-penalty-to-provide-lqr-performance.html).
References supply design context, not guarantees for this nonsmooth robot.

### I-34 authorized follow-up — implementation and experiments completed

**Status/change.** The user authorized the proposed investigation and asked
whether MPC uses observer contacts and can predict future contact. Implemented
an optional practical native-feedback rollout seed, configurable refinement
budgets and a held-native cadence control. Model, costs, .4 s horizon, physical
bounds and contact estimator remain unchanged. Old defaults retain one
refinement with the new seed disabled. Exact pre-change, test and launch
sources are preserved; the original/default-new runtime output and full input
sequence at 0.4 s match bit for bit.

**Present contact versus future prediction.** Hardware-mode momentum observation
estimates net wrench; the locator infers point/patch/loads; filtered normal and
friction plus equivalent point/total load condition MPC's private plane. No
live Gazebo contacts enter control. Future private contacts are computed by
geometry/dynamics, not fixed to the observed patch label. The old first plan
already predicts wheel support loss at +298 ms and fork-arm ground contact at
+373 ms; saved predicted states reproduce exactly. At 2 s it predicts shell-A
support throughout. These are hypotheses under candidate inputs, not actual
falls after replanning. The single plane/friction stays fixed over .4 s;
unknown upcoming terrain or a general multi-surface/contact posterior is not
predicted. Native feedforward fields are shadows when MPC supplies direct
inputs. Contact prediction is present; quality of using it is a separate issue.

**Practical seed.** Clone current contact-free controller/mirror, observer,
locator/filter/synchronizer history and nominal joints. Simulate native policy
at 5 ms in private predicted states, mean each four inputs into a 20 ms
sequence, then independently replay/score the coarse inputs. The first already-
computed native command is passed explicitly to avoid duplicate observer
updates. Explicit original-function calls bypass the experiment's monkeypatch.
No saved future command/state or true terrain/contact is read; source state
is unchanged. Only hardware/least-squares mode is supported by this helper.

**Authoritative actual-runtime frozen matrix:**

| At 0.4 s | One refinement | Two | Five |
|---|---:|---:|---:|
| Original bank | 24,525.748 | 12,610.934 | 5,599.801 |
| Add predicted-native seed | 129.089 | 127.556 | 99.880 |

At 2 s, all variants are approximately 58.338715. The initial seed is useful
without new weights, but five refinements at one frozen anchor are not a
closed-loop result. The additional two-refinement live gate (>=10% improvement
at both anchors) fails: about 1.19% initially, negligible at 2 s. It is not run.
The primary one-refinement seeded case isolates initialization at the live level.

**Preserved diagnostic stages/failure.** Initial independent per-seed chains
and later global-best continuations differ in cache/refinement ordering from
the final runtime-entrypoint matrix; all are labeled and preserved. A too-strict
cold-QP comparison at 2 s fails on cost difference 4.96e-5 (~8.5e-7 relative)
against the old continuously warmed solver. Saved-plan replay cost/states remain
exact; the fresh solve is not relabeled bit-identical to old cache history.

**Derivative finding.** At the audited initial seeded plan, sparse interpolation
predicts directional cost derivative about -57.20 versus nonlinear -18.26.
Full-stage evaluation predicts -57.57456 versus -57.57461 and accepts cost
129.089 to 120.636; sparse tested line steps worsen cost in that separate audit.
At 2 s full-stage directional agreement is also close but cost improvement is
tiny. Directions differ between models; no universal Jacobian validation or
sole-cause proof. Full Jacobians are not adopted in a live controller here.

**Closed-loop result: initialization alone does not fix drift.**

| Case | Input update | Final goal error | Peak feedback error | Final XY speed | Final-second gate |
|---|---:|---:|---:|---:|---|
| Native LQR, reused I-32 | 5 ms | 0.381 mm | 32.788 mm | 0.185 mm/s | Pass |
| Unchanged native held longer, new | 20 ms | 49.026 mm | 83.109 mm | 9.213 mm/s | Fail |
| Original MPC, reused I-32 | 20 ms | 31.215 mm | 88.278 mm | 17.798 mm/s | Fail |
| Practical-seeded MPC, new | 20 ms | 28.982 mm | 88.643 mm | 18.224 mm/s | Fail |

All shared drift prefixes and switch state/observer match exactly. Both new
cases complete without falling and keep applied inputs bounded. The slower
native case holds unchanged 5 ms-designed gains; it is not a redesigned 50 Hz
LQR and does not establish the best achievable slower-native performance.
Seeding improves initial modeled cost but gives no settled return and no peak-
excursion improvement in the matched live case. Do not promote it as a fix.
Platform lean remains near its ~9.73° reference, not world-horizontal.

Seeded MPC has zero seed-generation failures and zero true native fallback.
It selects lower modeled cost than its native-policy candidate in all 401
plans. The 400 actuated accepted plans use 802 solved QP calls (401/804 including
the terminal observation); all 1,600 actuated feedback samples are SQP-refined
source-3 commands, not source-4-only proposals or native fallback. Median
planning time is about 2.02 s per .02 s simulated interval, still offline.

**Why a low local cost can still drift.** A read-only decomposition of saved
costs reconstructs totals exactly. At 2 s the selected plan costs 58.339 and
predicts 58.604 mm terminal distance; the practical native candidate costs
109.299 and predicts less retreat, 51.803 mm. Its XY-position cost improves
only 52.265 to 49.476 within .4 s, while tilt cost rises 0.829 to 44.486 and
angular-velocity cost 0.284 to 8.124. Input-effort costs are only 0.046/0.049.
This is a concrete short-window balance/tracking tradeoff, not proof that a
specific weight or horizon change will solve the closed-loop case. Neither
candidate reaches the goal within that horizon.

**Counts/verification/decision.** Two new Gazebo executions, 3,362 guarded calls;
two I-32 baselines are explicitly reused. Frozen-state optimizer calls, contact
replays and cost decomposition are private diagnostics, not additional Gazebo
cases. The selected integration suite passes 124 checks in 46.34 s. The 15 seed
checks also pass separately in both environments; counts overlap and are not
added. Clone isolation, bounds, timing, recursion bypass and forbidden-contact
reads are tested. Earlier packs remain immutable. No defaults, observer or
particle filter are replaced. Better initialization alone is rejected as the
settling fix. Next isolate horizon/terminal return value and attitude-tracking
priorities with derivative fidelity/cadence controlled; do not infer that MPC
lacks contact information or that a new estimator is needed.

**Evidence:** [I-34 presentation/evidence index](docs/iterations/2026-11-25/mpc-search/README.md),
[protocol](results/mpc_search_20261125/protocol.json),
[frozen runtime matrix](results/mpc_search_20261125/frozen_search/runtime_entrypoints/report.json),
[derivative audit](results/mpc_search_20261125/frozen_search/derivative_audit/report.json),
[contact audit](results/mpc_search_20261125/contact_prediction/summary.json),
[live comparison](results/mpc_search_20261125/analysis/report.json),
[saved cost breakdown](results/mpc_search_20261125/cost_breakdown/report.json), and
[comparison video](results/mpc_search_20261125/videos/mpc_original_vs_predicted_native_seed.mp4).
The clip reconstructs saved Gazebo physics; it is not a private-model-versus-
Gazebo comparison and adds no physics.

## I-35 — Longer-horizon frozen comparison

**Date/status:** 2026-11-25; authorized, run and analyzed. Six private MuJoCo
forecasts; zero new Gazebo trials. Neither longer horizon passes the declared
selection rules. No default, observer, weight or core controller change.

**Question/change.** Does .4 s lookahead miss the future benefit of corrective
lean and return? At the same I-32 failed-MPC states (.4 s and 2 s), compare
.4/.8/1.2 s horizons (20/40/60 stages), full-stage Jacobians, present-state
predicted-native seed and a fixed five-refinement budget. Keep weights/scales,
input/change costs, terminal multiplier 4, bounds, model, observer/locator,
friction, solver tolerances and .02 s command/stage period fixed. The terminal
penalty moves to each new horizon end. Extend the 2 s preceding warm sequence
with its last bounded input before the normal runtime shift. No saved future
commands seed this experiment. Pre-change and exact tested sources are frozen.

**Conditions/gates.** Reuse the I-32 slope fixture and observer histories;
private predictions use the observer-conditioned single local plane, not a Mars
forecast or real measurements. Exact simulation state remains an assumed
measurement. Before calculation, declare selection of the shortest longer
horizon passing both anchors. Require bounded inputs, wheel load >=5% weight,
no nonwheel load >.05 N and absolute tilt <=20° at 1 ms physics substeps. Allow
early overshoot but cap peak distance at 40 mm plus same-horizon native-relative
criteria. At 2 s require terminal distance <=90% initial, a closing final .2 s,
peak <=75 mm, genuine nonlinear improvement and changed first input. Compare
costs only within a horizon. If none passes, do not launch Gazebo.

| Saved state | Initial distance | H=.4 final | H=.8 final | H=1.2 final |
|---|---:|---:|---:|---:|
| .4 s | 8.125 mm | 13.954 mm | 43.951 mm | 81.028 mm |
| 2 s | 44.903 mm | 58.603 mm | 71.860 mm | 107.228 mm |

**Outcome/failures.** H=.8 retains wheel support but fails the early excursion
and later return/closing gates; its late terminal radial velocity is outward
301.51 mm/s. H=1.2 predicts 83.58°/98.98° peak tilt, wheel support loss and
loaded nonwheel contact. No new actual Gazebo fall is observed: these are
private-model predictions. All 42 QP calls solve and pass the existing normalized
KKT check, with zero seed-generation/execution failures; that is not nonlinear
convergence or motion-gate success. The early H=1.2 case retains the native
candidate with no nonlinear improvement. Raw costs across horizons are not
ranked. Plan wall times 14–65 s include concurrent-worker contention and remain
far above the .02 s simulated command period.

**Limitations/learning.** The fine 5 ms native-policy trajectory and its
independently replayed averaged 20 ms inputs diverge by 77–103 mm terminal XY
at longer horizons (3.77–5.25 mm at .4 s). Coarse inputs are correctly replayed
and scored; this shows a seed-quality limitation, not fine states mistakenly
used as coarse dynamics. Some long-horizon Hessian maxima reach 10^17–10^25;
relative negative eigenvalues at that scale do not prove physical nonconvexity.
No objective scaling or PSD repair is applied. Five refinements and solved local
QPs do not certify the best nonlinear plan. Per-refinement costs/solver records
are saved; intermediate accepted motion is unavailable and no duplicate solves
are run to add it. Every common state/target/native-input prefix and every
1 ms replay's 20 ms endpoints match bit for bit.

**Decision/next hypothesis.** Do not launch the conditional Gazebo test or
promote a horizon change. This implementation did not find a credible return;
it does not establish physical impossibility or that longer horizons cannot
help. Next proposal: construct each native seed stage from the state actually
reached under its held 20 ms input, with consistent cloned observer/controller
history, then compare seeds at fixed horizon/cadence/weights. This is not
implemented here. Numerical conditioning remains a separate unresolved issue.
No particle-filter or estimator replacement is justified by this result.

**Verification/evidence.** New diagnostic CLI/compile checks and six completed
forecasts; no new unit-test suite or Gazebo executions. Prior I-34's 124 selected
checks are not added to these counts. Exact worker hashes, all shared prefixes,
bit-exact replay endpoints, compact copies and archive contents are verified.
Unexecuted later instrumentation is labeled separately; the repo script equals
the tested worker source. Earlier presentation packs remain immutable.
[Presentation/evidence index](docs/iterations/2026-11-25/mpc-horizon/README.md),
[protocol](results/mpc_horizon_20261125/protocol.json),
[report/gates](results/mpc_horizon_20261125/frozen/report.json),
[trajectory plot](results/mpc_horizon_20261125/frozen/horizon_trajectories.png),
[QP conditioning](results/mpc_horizon_20261125/frozen/solver_conditioning.csv).

## I-36 — Proposed priority review after the negative horizon result

**Date/status:** 2026-11-25; recommendation in response to the user's doubt
about continuing this direction. Proposed only: no code, new calculations,
simulation, tuning or algorithm change. This does not overwrite I-35 or its
frozen presentation pack, and no follow-up experiment is represented as
user-approved or completed.

**Evidence/reassessment.** I-34's practical seed markedly lowers initial modeled
cost without settling the matched live recovery; I-35's longer horizons do not
produce qualifying return plans. Making the seed consistent with 20 ms held
inputs is a valid implementation hypothesis, but its large fine/coarse gap
does not establish that another seed fix is the highest-value next robot
improvement. The existing 5 ms native LQR/feedforward recovers one matched drift
case; holding its unchanged commands at 20 ms fails. This establishes cadence
sensitivity for that controller, not that 20 ms control is impossible or the
sole cause of MPC drift. At the later short-horizon anchor, the modeled objective
also penalizes corrective lean enough to prefer more position retreat among
the compared candidates. Search quality, cost preference and update timing
remain distinct limitations.

**Recommendation/next hypothesis.** Deprioritize direct-actuation MPC tuning as
the next practical improvement; retain it as research evidence. Use the
unchanged 5 ms native controller and observer as the working baseline and
investigate the actual late X recovery failure already recorded in I-32.
That path reaches about 1.01 mm error at 7.12 s, then exceeds 10 mm at 8.660 s;
belt saturation starts at 8.780 s and true contact switching at 8.785 s. Since
drift precedes those later events, neither is established as its initial cause.
First inspect the saved pre-onset state, reference-relative lean/rate, changing
configuration, separate LQR/feedforward/synchronizer commands, observer support
and applied inputs. Then choose one targeted matched ablation from the earliest
supported explanation, retaining the full preceding history. Do not repeat the
already-regressing I-17–I-21 reference/allocation changes without new evidence.

**Limits/decision boundary.** No broad robustness claim for LQR, no disproof of
MPC, no new particle-filter justification, and no claim that the observer is
universally correct. Judge any next candidate by the same original goal,
settling, attitude, command bounds and retained regression cases, rather than
modeled objective reduction alone. This recommendation changes the proposed
priority; the diagnostic/ablation itself has not run.

**Evidence:** [I-32 recovery and late-event record](docs/iterations/2026-11-25/feedback-recovery/README.md),
[I-34 search record](docs/iterations/2026-11-25/mpc-search/README.md),
[I-35 horizon record](docs/iterations/2026-11-25/mpc-horizon/README.md), and
[sample-time/horizon design context](https://www.mathworks.com/help/mpc/ug/choosing-sample-time-and-horizons.html).
External design guidance is context, not a diagnosis of this robot.

## I-37 — The video shows sustained return: correct the priority inference

**Date/status:** 2026-11-25; completed saved-data review prompted by the user's
I-34 video observation. No new physics, controller change or test execution.

**Question/evidence.** Is MPC slowly going back toward the goal in that video?
Yes. Hash-verified precision traces show original MPC peaking at 88.278 mm
(4.360 s), then decreasing monotonically to 31.215 mm at 8.4 s. Seeded MPC peaks
at 88.643 mm (4.295 s), then decreases monotonically to 28.982 mm. In the final
second their errors drop 49.518→31.215 mm and 47.939→28.982 mm. Every final-window
radial sample points inward; final radial velocities are −17.174/−17.989 mm/s.
These are recorded Gazebo trajectories, distinct from I-35 private forecasts.

**Correction/limits.** Both fail the declared settled-return criterion within
eight seconds of feedback, but show sustained return when the recording stops.
That deadline does not establish inability to recover. I-36's proposed pivot
gave that failure too much weight; its next-priority recommendation is qualified
by this review. No guarantee of eventual settling, no claim that the observed
improvement fixes overshoot, runtime or broad robustness. Native late-X failures
and I-35 rejected forecasts remain intact, rather than being overwritten.

**Revised proposal.** Before rejecting the seeded MPC on recovery grounds,
extend that unchanged I-34 live case to 16.4 s total (16 s feedback). Keep its
one refinement, stride-5 Jacobian, .4 s horizon, observer, weights, bounds,
reference and 20 ms command timing; do not substitute I-35 settings. Preserve
the exact prior sources and verify the common .0–8.4 s prefix. Retain the same
one-second settling gate and report overshoot, oscillation or renewed drift.
Prior baselines are reused, not new executions. This extension is proposed
only and has not been launched.

**Verification/evidence.** Independent read-only review agrees with the inward
trend and verifies both input hashes. A saved-data reduction reproduces the
table and plot; compact copies retain the exact audit source and hashes.
I-32/I-33 and I-34 presentation archives retain their original hashes. No
overlapping unit-test counts are added.
[Review and plot](docs/iterations/2026-11-25/mpc-return-review/README.md),
[raw audit](results/mpc_return_review_20261125/report.json),
[original video](results/mpc_search_20261125/videos/mpc_original_vs_predicted_native_seed.mp4).

## I-38 — Authorized unchanged MPC extension to test eventual settling

**Date/status:** 2026-11-25; declared and launched; completed follow-up below. The user
authorized I-37's proposed extension. One new Gazebo/DART run: .4 s matched
fixed-input drift then 16 s seeded MPC feedback (16.4 s total). Earlier
I-32/I-34 traces are reused baselines, not new executions.

**Question/change.** Does the observed inward return finish and settle with
more time? Only requested duration/output change. Retain exact I-34 seeded
sources/settings: one refinement, stride-5 Jacobian, .4 s horizon, .02 s input
hold, .005 s observation, observer/locator, weights, model, bounds, starting
pose, retained XY goal and slope-lean reference. Uniform10° slope mesh .04 m,
not Mars. No resets at feedback switch or old end; no controller/default change.

**Preflight/protocol.** All 52 source hashes, saved-case and gain-cache hashes,
and NumPy/SciPy/MuJoCo/OSQP versions match I-34. The unchanged runner snapshots
sources before execution. Verify 1,681 physical/observer/reference/input
samples and 401 plan decisions through8.4 s, excluding timing metadata. The
old terminal command becomes actuated only in the extension. Keep the original
final-full-second gate (<=5 mm, <=5 mm/s, <=2° reference-relative lean),
completion/no fall and bounded inputs. Record crossings/overshoot, first settled
window, late drift/fallback and runtime. No outcome is claimed while running.

**Evidence:** [running presentation record](docs/iterations/2026-11-25/mpc-extended-return/README.md),
[declared protocol](results/mpc_extended_return_20261125/protocol.json),
[launch command](results/mpc_extended_return_20261125/launch_command.txt),
[live log](results/mpc_extended_return_20261125/launch.log).

### I-38 completed — near-goal return, renewed retreat, no settled hold

**Outcome.** One new 16.4 s Gazebo/DART run completes without a fall. The old
1,681 samples through 8.4 s match bit for bit across every non-wall precision
and rollout field; all 401 plans match after excluding wall timings. Source,
settings, packages, terrain bytes, gain cache, initialization and activation
also match. This is a verified continuation of the recorded history with no
new estimator, gain, cost, seed algorithm or horizon setting.

| Event | Time | Goal distance |
|---|---:|---:|
| Old cutoff, moving inward | 8.400 s | 28.982 mm |
| Reenters 5 mm radius | 10.185 s | <=5 mm |
| Closest added-interval approach | 10.755 s | 2.936 mm |
| Last sample in this 5 mm visit | 11.400 s | <=5 mm |
| Largest subsequent retreat | 15.165 s | 20.492 mm |
| Final endpoint, moving inward again | 16.400 s | 19.100 mm |

**Acceptance/interpretation.** Position is near goal for 1.215 s, but speed
exceeds 5 mm/s during part of that visit. All position/speed/reference-lean
conditions hold together for at most .530 s; no full 1 s settled window occurs.
Final XY speed is 2.997 mm/s, inward radial speed 2.052 mm/s, absolute tilt 9.754°,
reference 9.731°, and relative lean error .0262°. Final 1 s max error 20.434 mm
exceeds 5 mm, so the unchanged final gate fails. The user's observed return
was real, and the extension establishes a later near-goal passage followed by
renewed separation and another inward trend. It does not establish eventual
convergence/divergence, a limit cycle, or a particular damping/gain/cost cause.

**Contact/commands.** In all 1,600 added-interval samples, truth and estimate
identify dominant shell A, with zero patch transitions/disagreements. Minimum
logged normal load 17.947 N, zero logged fork load, and no added-interval input
saturation. Actual inputs remain bounded throughout. This event is not
accompanied by a patch handover or command saturation; dominant identity alone
does not validate every contact estimate or general multi-contact inference.

**Counts/verification.** 3,281 guarded calls; 800 actuated accepted plans and
1,602 solved QPs (801/1,604 including terminal observation). All 3,200 actuated
feedback samples use SQP-refined source 3; zero true native fallback or
seed-generation/trial failures. Median actuated update 2.441 s per .02 s simulated
interval, still offline. One new simulation, not a new unit-test suite; prior
124 checks are not added. Independent review checks prefix/gate arithmetic.
Pre-execution tooling review corrected exclusion of wall-clock plane_update_s
from plan equality before analysis; physical times remain included. Primary
analysis is sealed before a separate saved-data event-detail stage; no extra
physics or duplicate optimization fills that stage.

**Media/preservation/decision.** The 32.85 s half-speed video shows the fixed
goal inset, old 8.4 s cutoff, error/speed/absolute/reference-tilt charts, and final
gate. It reconstructs saved Gazebo states in MuJoCo, adds no physics, and its
decoded final frame/hashes are verified. Exact launch, analysis and render
source stages plus compact reports are retained outside ignored results;
larger NPZ/video payloads are in a separate presentation bundle. Earlier packs
remain immutable. Keep this controller as an experimental reference: it can
return near the goal in this fixture but has not held it by 16.4 s. The more
concrete next diagnostic is the near-goal 10.2–11.4 s command/prediction response;
no new tuning, estimator replacement or further extension is performed here.

**Evidence:** [completed presentation record](docs/iterations/2026-11-25/mpc-extended-return/README.md),
[report](results/mpc_extended_return_20261125/analysis/report.json),
[return events](results/mpc_extended_return_20261125/analysis/return_events.json),
[prefix verification](results/mpc_extended_return_20261125/analysis/prefix_verification.json),
[full video](results/mpc_extended_return_20261125/videos/seeded_mpc_extended_return.mp4), and
[presentation bundle](results/mpc_extended_return_20261125/presentation-pack.zip).

## I-39 — Keep fast LQR; separate contact prediction from iLQR

**Date/status:** 2026-11-25; architecture review and proposed next experiment.
The user prefers LQR for now and asks about iLQR, predictive contacts and a
better development direction. No optimizer, controller mode, gain, observer,
configuration or source is changed; no simulation or numerical test runs.
Completed I-38 artifacts and presentation archives remain frozen.

**Clarification.** The working baseline is gain-scheduled LQR with contact-aware
feedforward and incoming-drive synchronization, using 5 ms feedback in the
retained Gazebo comparisons. iLQR iteratively rolls out a nonlinear model and
solves local time-varying LQR problems along a nominal trajectory. It is an
optimization method, not a contact estimator; used in a receding-horizon loop
it can be a nonlinear MPC solver. The existing SQP prototype already makes
trajectory linearizations and quadratic subproblems, so changing to iLQR does
not newly supply contact prediction or establish better settling. Riccati
structure may help computation, but model/derivative costs and measured runtime
still require validation. Input limits need an appropriate constrained method;
hard contact transitions require suitable derivatives/hybrid treatment or a
validated smooth contact model. Neither issue disappears with the algorithm's
name.

**Present versus future contact.** Current observation remains the anchor:
momentum wrench -> support locator -> filtered normal/friction/load estimates.
EstimatedPlanePredictor already predicts future unilateral/frictional contacts
from state, geometry and candidate actions on that fitted local plane. The
plane/friction are frozen over each prediction, not the robot's future patch
identity. Multiple robot contacts with that plane can appear in the model;
arbitrary unseen terrain planes and general contact-distribution uncertainty
are not inferred. The LQR's existing ContactSynchronizer has a .12 s
constant-velocity kinematic preview, explicitly distinct from dynamic contact
prediction. Future measurements must correct predictions; a predicted contact
must not be treated as an observed contact.

**Recommendation.** Retain the observer-driven 5 ms native LQR/FF as the live
baseline. First evaluate a short dynamic preview under that same fast feedback
policy in diagnostic/shadow mode. Reuse private predictor and controller-clone
components, retaining fine 5 ms actions/states instead of averaging them into
20 ms holds. Actual comparison trajectories must also use native LQR, with
matching goals, full observer/controller history and commands. Do not compare
a native-policy forecast against an actual MPC future and call the difference
model error. Start with one single-support case and one controlled shell/belt
transfer, using the existing late native-X event as a diagnostic anchor where
history can be reconstructed correctly. Score contact identity, timing,
location/effective lever arm, load transfer and motion against simulator truth
offline; truth remains excluded from control. These tests are proposed only.

**Decision boundary.** Forecast accuracy/runtime must be measured before using
it to change gain blending, feedforward or existing incoming-drive preparation.
Do not assume better preview fixes the original failure. I-38's renewed drift
occurred with matching shell-A identities and no handover/saturation; that
does not validate every estimate, but gives no evidence for replacing the
observer to fix that event. Prior I-17–I-21 allocation/reference regressions
remain counterexamples to repeating a broad feasibility/tilt-weight sweep.

If a planner is later justified, a slower layer can choose bounded motion/lean
references while fast LQR continues stabilizing. Its predictor must include
the fast LQR/observer loop within each planning stage, rather than replace it
with held direct torques. iLQR could be tested offline on a controlled nominal
trajectory, with input bounds and feedback-cadence comparisons, before hybrid
contact planning. Re-linearizing at an arbitrary non-equilibrium state alone
does not supply a dynamically consistent reference or stability certificate.
No online timing feasibility or hardware readiness is claimed.

**Evidence and design context:**
[native feedback](src/spherical_wheel/offroad_control.py),
[existing kinematic preview](src/spherical_wheel/synchronization.py),
[private dynamic predictor](src/spherical_wheel/predictive_model.py),
[fine policy rollout components](src/spherical_wheel/predictive_seed.py),
[I-38 observed return/retreat](docs/iterations/2026-11-25/mpc-extended-return/README.md),
[MIT trajectory optimization/iLQR notes](https://underactuated.mit.edu/trajopt.html),
[Control-Limited DDP](https://roboti.us/lab/papers/TassaICRA14.pdf),
[hybrid iLQR](https://arxiv.org/abs/2103.14584), and
[hybrid iLQR in MPC](https://arxiv.org/abs/2207.04591).
Primary references describe algorithms, not demonstrated improvements on this
wheel. Repository evidence was reviewed read-only; documentation links and
unchanged source/archive hashes are checked without rerunning physics.

## I-40 — Authorized LQR contact forecast and trajectory audit

**Date/status:** 2026-11-25; declared, implementation/validation in progress.
The user authorized I-39's diagnostic forecast experiment and requested more
tests/trajectories. Eight fresh native Gazebo/DART cases are declared before
campaign calculations. No result or controller improvement is claimed yet.

**Question/change.** Keep hardware-mode native LQR/FF/synchronization running
at 5 ms with existing gains, least-squares allocation and observer/locator.
Add a separate diagnostic that clones its current history and forecasts
200 ms at the same 5 ms feedback cadence, scoring 50/100/200 ms endpoints.
No 20 ms averaging, optimizer replacement, future-command replay as a seed,
or true-contact input. Reuse private estimated-plane prediction, with its
plane/friction frozen within each rollout and cloned filters updated from
predicted states. Preserve fit failures/flags rather than repairing from truth.

**Cases/protocol.** Flat hold/x/y-transfer, slope-x late regression/slope-y,
flat out-and-back/triangle and actual Mars figure-eight phase+.9. Standard
terrain-informed startup placement is declared, exact simulated state remains
an assumed measurement, and live contact reads are guarded. The late slope
paths retain their .8 s measured-origin reanchor; forecasting starts after .9 s
so the future origin reset is not leaked into a past forecast. Other anchors
start at .4 s. Up to12 uniform primary anchors plus8 separately labeled
truth-selected pre-transfer anchors per case, with unique-event counts and
censored endpoints. The actual future uses the same native policy/path.

**Comparison/decision.** Compare contact sets/identity/timing, load shares,
loaded points/effective lever and motion, retaining raw and15ms-dwell events.
Observer-persistence and constant-velocity motion/geometry are explicit
diagnostic baselines; the existing120ms preparation preview is an exposure
warning, not a force model. Score only common observables fairly. A named
transfer fixture does not guarantee an isolated transfer; that criterion must
be checked. Report trajectory completion, tracking and original settling/Mars
gates separately. This is diagnostic-only with no automatic command adoption.
Source/test/failure stages, exact fresh-run provenance and earlier archives
are preserved; counts distinguish unit tests, actual runs and private forecasts.

**Evidence:** [presentation record](docs/iterations/2026-11-25/lqr-contact-forecast/README.md),
[protocol](results/lqr_contact_forecast_20261125/protocol.json),
[exact cases](results/lqr_contact_forecast_20261125/campaign/jobs.json).

### I-40 completed — useful motion forecasts, contact preview remains diagnostic

**Implementation/validation.** Added diagnostic-only
`contact_prediction.forecast_native_contacts`, retaining 5 ms native commands,
states, private patch loads/points and current kinematic preview. The current
observer/controller history is cloned; solved private contacts never feed
that policy. Existing controller/model/seed files remain unchanged. Latest
45 selected checks pass in both .venv and the Gazebo environment: 14 new
forecast, 15 seed and 16 model checks. Stage/environment counts overlap, are
not additive and are not a full-repo suite. A pre-campaign review corrected
stale preview timing in the API's before-command branch; exact earlier and
later passing source stages remain preserved.

**Actual trajectory results.** Eight fresh Gazebo/DART runs complete without
falls, with 18,848 guarded calls and finite bounded applied commands. Four of
seven controlled cases pass the original final-second settling gate; the Mars
case fails its separate existing route gates. No forecast changes actuation.

| Case | Final original-path error | Quality gate |
|---|---:|---|
| Flat hold | .0001 mm | Pass |
| Flat x | 1.77 mm | Pass |
| Flat y transfer fixture | 10.18 mm | Fail |
| Slope x late regression | 99.45 mm | Fail |
| Slope y | 6.02 mm | Fail |
| Flat out-and-back | 3.21 mm | Pass |
| Flat triangle | 3.08 mm | Pass |
| Mars figure-eight +.9 | 190.19 mm | Existing route gate fails |

The Mars path runs 44 s on the actual yard DEM; controlled surfaces are flat
or uniform 10° slope. The intended flat-y fixture produces no stable family
transfer. The prescribed fallback is the first late-x shell-to-belt family
event at 9.260 s, forecast from 9.160 s. It has no other stable family switch
within ±100 ms, but raw forces chatter; do not call it a clean contact impulse.
No sustained multiple-patch loading above 5% weight is demonstrated.

**Forecast audit.** All 113 unique anchor forecasts compute, comprising 96
uniform primary and 17 truth-conditioned diagnostic anchors. All reconstructed
histories have exactly zero command/requested-command/wrench/normal/nominal/
reference gap, and source controller fingerprints remain unchanged. The 339
50/100/200 ms endpoints are nested, not independent extra cases. Future truth
only selects separately labeled diagnostic windows and evaluates predictions.

| Horizon | Uniform dynamic XY p95 | Constant-velocity XY p95 |
|---|---:|---:|
| 50 ms | 1.27 mm | 1.26 mm |
| 100 ms | 2.25 mm | 4.35 mm |
| 200 ms | 5.47 mm | 12.48 mm |

Uniform 200 ms dominant support accuracy is 92/96 dynamic versus 93/96 holding
the current observed label; dynamic maximum XY error is 29.63 mm. At selected
transfer anchors the endpoint support figures are 14/17 versus 5/17, but these
correlated truth-conditioned samples do not establish physical-event recall.
Corrected 120 ms incoming-warning proxy TP/FP/FN: uniform dynamic 3/0/3 versus
native kinematic 2/18/4; event-conditioned 7/0/16 versus 14/1/9. Preparation
warnings and contact-load forecasts have different meanings; compare these
only as stated proxies, not force-model accuracy or statistical superiority.

Twelve of 113 fitted models miss current-load tolerance (5 uniform/7 event);
all are bracketed, and all remain in accuracy results. The selected late-x
forecast has −1.966 N initial fit residual, predicts loss of wheel load near
+100 ms and dominant shell A at +200 ms when actual support is belt A. In
slope-y at anchor 4.260 s/+50 ms the equivalent point is within .465 mm and
total load within .048 N, but predicted dominant belt A/two loaded patches
disagrees with actual shell A alone. Resultant accuracy is not load-share
or contact-identity accuracy.

**Runtime/limits.** Forecast plus calibration costs 270 ms median, 364 ms p95,
416 ms maximum, with 0/113 meeting a 20 ms budget. Three workers ran concurrently;
history replay, predictor construction and serialization are excluded, so no
real-time claim follows. Measurements remain ideal exact simulation state;
unknown terrain/multi-surface inference and hardware validation are not added.

**Retained errors/corrections.** An unexecuted runner syntax error and a preview
metadata relative-path failure are retained with exact sources/logs. Primary
analysis initially consumed a generator while pooling warnings, incorrectly
zeroing FP/FN. Per-anchor records and endpoint/trajectory metrics were intact.
Original report/source/manifest are sealed; a separate arithmetic correction,
adversarial generator check and final consolidation supply authoritative
`analysis/final_report.json`. No physics/geometry/forecast rerun corrects this.
Independent review recomputes corrected warning counts and trajectory gates.

The fresh late-x case is not a bitwise I-32 reproduction: generated mesh
extent/tessellation is ±1.0 m/51×51 versus old ±.9 m/46×46, both .04 m cells.
Tiny initial numerical differences amplify; no sole cause is inferred. Its
99.45 mm endpoint is not a claimed improvement over old 139.61 mm. Every
forecast is evaluated against its own fresh matched native trajectory.

**Decision/preservation.** Retain live LQR/observer and keep this preview
diagnostic. Motion predictions add useful information, but load fitting,
contact warning reliability, outliers and runtime preclude command adoption
from this study. More elaborate iLQR/particle filtering is not implemented.
Two decoded/hash-verified videos preserve the trajectory outcomes and the
predeclared hard forecast example. Exact test/launch/analysis/render stages,
failures and corrected reports are archived; earlier packs remain unchanged.

**Evidence:** [completed presentation record](docs/iterations/2026-11-25/lqr-contact-forecast/README.md),
[authoritative report](results/lqr_contact_forecast_20261125/analysis/final_report.json),
[forecast summary figure](results/lqr_contact_forecast_20261125/analysis/summary_plot_stage/forecast_summary.png),
[trajectory video](results/lqr_contact_forecast_20261125/videos/native_lqr_trajectory_overview.mp4),
[forecast versus actual](results/lqr_contact_forecast_20261125/videos/contact_forecast_vs_gazebo.mp4),
[review](results/lqr_contact_forecast_20261125/review/final_review.md).

## I-41 — Figure-eight slope/departure and turning-overshoot hypotheses

**Date/status:** 2026-11-25; protocol declared before execution, completed below. The user
steered the interrupted general fit-improvement preflight toward two observed
events: first departure on a small slope and overshoot after direction changes.
The interrupted preflight changed no code and ran no experiment. These user
interpretations are hypotheses, not assumed causes.

**Scope/protocol.** Reuse the exact saved I-40 Mars figure-eight. Define first
local path distance >20 mm sustained >=100 ms after path start, with10/30mm
sensitivity and ±2s reference-time branch restriction. Report along-track lag,
signed normal error and boundary ambiguity. At reference component reversals
with full speed>=5mm/s, distinguish overshoot past a geometric extremum from
moving-reference tracking error; measure actual reversal lag and the full
trajectory rather than treating each component reversal as a stop.

The new dynamic forecast was not actuating this trace; inspect the active
120ms kinematic synchronizer separately, along with LQR/FF/support/heading/
clipping, gains, contacts, terrain and geometric drive authority. DEM tangent
normal is not an actual force-weighted collision normal. Dense private native
forecasts at<=35 declared event anchors are paired with saved-future-command
model replay, explicitly a hindsight diagnostic and not deployable prediction.

**Controlled comparison.** One new44s flat figure-eight retains the exact
reference/phase/yaw/native5mscontroller and matches the global500² DEM collision
grid, with nominal friction and constant saved initial DEM height. Appropriate
initial balancing changes with terrain; this is a whole-terrain comparison,
not isolated local-slope causality or a bit-identical startup. No new Mars run,
controller tuning, gain/observer replacement or automatic sync-off ablation.
No outcome claimed yet. Sources/protocol are preserved before calculations.

**Evidence:** [presentation record](docs/iterations/2026-11-25/figure8-event-audit/README.md),
[protocol](results/figure8_event_audit_20261125/protocol.json),
[reused Mars case](results/lqr_contact_forecast_20261125/campaign/runs/mars_figure8_p090/provenance.json).

### I-41 completed — separate terrain-transition departure from turning response

**Outcome.** No live controller, observer, gain, synchronizer or dynamic-model
change. One reused I-40 Mars execution, one new 44 s flat Gazebo/DART execution
(8,801 guarded calls, no fall), 35 native private forecasts and 35 explicit
offline hindsight-input model replays. The 105 paired nested endpoints are
not additional independent experiments. The dynamic forecast was shadow-only;
the existing 120 ms kinematic synchronizer remained active.

**First departure.** The declared local-path 20 mm/100 ms onset is 6.860 s,
confirmed at 6.960 s; the 30 mm onset is 6.910 s. The 10 mm threshold already
fires during startup at 1 s, and time-aligned 20 mm error starts at 2.690 s,
mostly lag. Around 6.7–6.9 s terrain steepens, shell/belt identities chatter
and belt A requests exceed its ±0.3 m/s simulated bound (about 0.421 m/s at
6.725 s and 0.835 m/s at 6.825 s). At 6.860 s actual belt A/estimated shell A
points differ only 0.183 mm, the gain step is 23.8%, the previous belt command
changes 0.3→0 m/s and shell A changes +0.061→−0.569 Nm. Current motion reflects
preceding inputs; the command at 6.860 s acts over the next 5 ms. Synchronizer
delta/weights are zero at that instant but active earlier, so its contribution
is not excluded.

**Geometry and comparison.** At 6.860 s local DEM grade is 12.60°, actual
platform tilt 8.60° versus reference 12.19° (full up-vector error 4.705°).
The estimated normal differs 0.038° from the projected contact facet. Logged
DEM closest tangent and the facet at equivalent contact XY are not physical
force-weighted contact normals. Flat moving RMS is 16.821 mm versus Mars
50.486 mm; first local 20 mm departure is 17.735 s versus 6.860 s; final error
is 0.115 mm versus 190.191 mm. Exact reference time/XY/V/A are byte-identical.
Flat removes all relief/obstacles and changes standard balanced initial pose
and history. Mars initial error is 15.809 mm, and 12.328 mm at path start;
flat starts essentially centered. This compares whole terrain conditions,
not isolated slope causality.

**Prediction.** All 35 source histories reproduce exactly and paired first
5 ms state, contact and input match. Five initial load fits miss tolerance.
At 6.660 s/+200 ms, normal forecast XY error is 15.128 mm and actual-future-
command replay error is 22.996 mm; at 6.860 s/+200 ms, errors are 10.543 and
11.832 mm. Future command-sequence divergence alone does not explain the gap.
Frozen plane, calibration and contact dynamics remain candidates, not proven
live causes. At 200 ms, onset patch agreement is A 1/5, B 4/5; turn anchors
26/30 for each. Overlapping selected anchors are not population accuracy.
No future states, contacts or terrain feed the model; future actual commands
are used only in explicitly hindsight mode B.

**Turns.** The first reference component reversal at 11.775 s follows the
early departure. Actual reversal lags 1.101 s with zero geometric overshoot
in the declared [−1,+2] s window. The first three Mars turns have zero;
the later three show 9.84/28.02/60.20 mm. Flat shows 8.03/15.62/23.44/28.96/
14.97/26.49 mm, so overshoot does not require terrain relief. Smooth component
reversals are not full reference stops. Differing preceding states prevent
assigning every Mars/flat difference to terrain.

**Development/validation.** The initial saved-data helper included one
out-of-window projection segment and missed exact zero-plateau reversals.
Failing checks, sources and stage 01 are preserved; corrected stage 02 passes
adversarial assertions. All Mars distances, onsets and turn times/lags are
unchanged; the 35-anchor file is byte-identical. No new Gazebo or forecast
rerun was needed. Saved-data helper/history checks are separate from earlier
I-40 integration checks; this is not a new full-repository suite. Exact
pre-change core sources and five old presentation bundles remain unchanged.
Annotated clips replay saved states without new physics.

A secondary exact-segment branch-ambiguity audit was declared after primary
selection; its 1 mm tolerance and temporal grouping do not alter anchors.
No selected departure/turn window has a competing branch under that definition;
remote contenders occur elsewhere near the crossing/start/end. Four geometry
assertions pass. The initial video preview's inset indexing error and exact
source/log are retained; corrected rendering changes no underlying physics.
Independent review corrected exact root-summary geometry aliases (facet angle
versus logged-tangent angle and evaluated versus float32 grade). The original
consolidation/source remain preserved; rounded findings and all experiments
are unchanged. The final summary derives these fields from sealed terrain data.

**Decision.** Keep LQR. The next proposed diagnostic should isolate patch/load
selection and gain/drive handoff around 6.7–6.9 s before weight tuning,
particle filtering or iLQR. Separately investigate flat-turn lag/damping.
No causal ablation or fix was implemented in I-41.

**Evidence:** [full presentation record](docs/iterations/2026-11-25/figure8-event-audit/README.md),
[consolidated report](docs/iterations/2026-11-25/figure8-event-audit/evidence/final_report.json),
[annotated video](results/figure8_event_audit_20261125/prediction/videos/mars_departure_and_direction_changes.mp4),
[shareable bundle](results/figure8_event_audit_20261125/presentation-pack.zip).

## I-42 — Isolate early support routing and incoming-drive preparation

**Date/status:** 2026-11-25; protocol declared before harness implementation
or physics; completed results below. User agreed to the I-41 handoff investigation and asked how to
improve the controller. Improvement remains a hypothesis, not a completed fix.

**Question/change.** Four fresh native-LQR Mars figure-eight runs at phase +0.9
compare baseline, feedback-support-share oracle, preparation mask, and both.
Only intervene during command holds in [4.0,10.0) s; this covers observed belt
preparation starting near 4.38 s and the 6.86 s departure. No simulator or
observer reset; histories carry consequences beyond 10 s. All runs last 44 s.

The oracle supplies current last-1-ms true fractions to feedback gain/holding
torque only, preserving inferred total load. Wrench, normal, estimated records,
geometry, rolling allocation and synchronization records stay native in the
same-state call. Insufficient-load fallback is explicit. This diagnostic has
instantaneous-versus-filtered timing limitations and is not deployable. The
preparation mask zeroes command and nominal-reference target weights while
private preview/plane updates continue. This tests both preparation effects,
not torque cancellation alone. LQR gains, estimator and defaults are unchanged.

**Comparison.** Exact pre-4 s command/history prefix and state at activation;
fresh baseline versus full I-40 trace; primary [6.4,7.4] s and recovery
[7.4,10] s plus full-route metrics. Retain path error, lean, clipping, command
steps, existing route gates and descriptive factorial interaction. Screen
preparation-off for at least 10% lower primary local-path RMS, completion,
peak lean no more than +0.5° and moving route RMS no more than +5%. These are
provisional screens, not adoption criteria; one run per arm is not robustness.

**Prior evidence/limits.** I-25 true-share and full-record interventions on a
different late phase −1 event all failed gates and worsened peak lean. Do not
assume better labels fix the issue or repeat a broad record replacement as an
identity-only test. The current 200 ms dynamic preview remains unused/offline.
Preserve 55 pre-change source origins and six old bundle hashes before work.
No new particle filter, iLQR, weights, actual robot or physical motor limits.

**Evidence:** [iteration record](docs/iterations/2026-11-25/handoff-factorial/README.md),
[declared protocol](results/handoff_factorial_20261125/protocol.json).

### I-42 completed — initial departure persists; preparation has later tradeoffs

**Execution and invariance.** Exactly four fresh 44 s Gazebo/DART cases, no
retries, maximum three concurrent isolated processes. All finish without a
fall; 35,204 guarded command evaluations include four unactuated terminal rows.
All arms have the same pre-4 s inputs and exact state/extended controller
history at activation. The fresh baseline reproduces all 17 common I-40
precision fields and cached I-41 path geometry bit-for-bit. References match.
LQR gain-bank settings, observer/contact-estimation code and global defaults
remain unchanged; the diagnostic changes load-based gain blends and/or
preparation command/reference weights exactly as declared.

**Primary negative result.** Every first sustained 20 mm local-path departure
remains 6.860 s. Primary local RMS is 38.661/38.850/38.548/38.792 mm for
baseline/feedback shares/preparation mask/both. Masking improves this by just
0.29%, failing the predeclared 10% early-improvement screen; its primary peak
local distance slightly worsens. First 10/30 mm threshold times are also
unchanged. Correcting the tested feedback load path and removing preparation
do not remedy this early event.

| Arm | Moving RMS mm | Peak timed error mm | Final error mm | Peak relative lean deg | Settled / all old gates |
| --- | ---: | ---: | ---: | ---: | --- |
| Baseline | 50.486 | 190.901 | 190.191 | 8.709 | No / Fail |
| Diagnostic true feedback shares | 41.635 | 85.335 | 8.804 | 5.323 | No / Fail: settling |
| Preparation masked, 4–10 s | 41.681 | 97.401 | 11.758 | 6.352 | Yes / Pass |
| Both | 45.355 | 101.198 | 26.032 | 8.600 | Yes / Fail: attitude |

**Later benefit and regression.** Masking lowers moving RMS 17.44% and passes
all original route gates in one deterministic run. This is distinct from the
failed new early-event screen. A separately declared saved-data check of all
six original turns reveals four worse overshoots, one improved and one
unchanged. First-turn lag falls from 1.101 to 0.832 s while geometric overshoot
increases from zero to 18.05 mm. All turns occur after native preparation
resumes at 10 s; differences carry earlier physical/controller history.
Do not promote the timed mask or assume old route gates ensure better turns.

**Scope checks.** Each intervention spans 1,200 held commands. True shares
are eligible 1,200/1,200 oracle-only and 1,199/1,200 combined, with one explicit
low-truth-load fallback; materially changed shares occur on only nine calls
per oracle arm. Inferred total load is preserved exactly, all seven same-state
invariants pass, and both mask levels/preparation increments are zero while
private preview/plane computations continue. No injection/mask outside window.
First oracle command/state differences occur at 6.725/6.730 s; first mask
command/state differences at 4.380/6.700 s. Feedback-only truth does not replace
geometry/rolling feedforward or remove instantaneous-versus-filtered timing
confounds, so negative results do not establish estimator irrelevance.

**Shared-demand audit.** A separately declared saved-data decomposition finds
that at 6.825 s all four correctly select belt A yet request 0.822–0.835 m/s
against 0.300 m/s. Baseline LQR contributes +0.8438 m/s: attitude +0.5455,
angular rate +0.2021, linear velocity +0.0747, position +0.0192 and joint rate
+0.0022. Rolling feedforward adds −0.0091, preparation zero. Actual/desired
tilt is 7.641/12.031°, full up-vector error 5.294°. Thus common loaded-drive
balance demand precedes the routing switch. At 6.860 the oracle avoids the
baseline's shell-routing error but retains the same departure. All 36 selected
snapshots have zero desired angular-rate components while desired lean changes.
Exact grouped terms reconstruct saved LQR after its float32 cast; largest
normalized component residual 2.30e−8. This identifies a candidate bottleneck,
not proof that changing references will fix it.

**Decision.** Keep defaults. Next candidate: shape an attainable smooth tilt
reference and its consistent desired tilt rate, keeping final estimated-ground
lean and existing heading behavior. Test reference consistency/loaded-drive
authority before adopting another estimator/controller or weakening balance
feedback. A measured-state preparation gate remains a separate candidate with
turning regressions to resolve. Neither candidate is implemented in I-42.

**Validation/corrections.** Twenty-one unique pure harness contracts pass in
two numerical environments; environment repetitions and repeated I-41 geometry
checks are not additive full-repository tests. Independent scope QA passes
all four cases. Initial analysis compared float32 bounds against double
provenance exactly (2.384e−8 gap); initial QA assumed float64 rollout fields.
Both failures and exact sources are retained; corrected checks use documented
metadata tolerance/exact float32 cast while precision clocks/histories stay
exact. No physics rerun. Initial video footer/source is retained; later
presentation refinement exposes early-screen failure alongside the original
route-gate pass. Launch-source hashes were unchanged at campaign completion;
renderer refinements happened later. All 55 pre-change origins and six older
bundles remain preserved. Raw sources, failed stages and video are retained.

**Evidence:** [full record](docs/iterations/2026-11-25/handoff-factorial/README.md),
[primary report](docs/iterations/2026-11-25/handoff-factorial/evidence/analysis/stage01/report.json),
[turn regression](docs/iterations/2026-11-25/handoff-factorial/evidence/analysis/turn_addendum/report.json),
[shared feedback demand](docs/iterations/2026-11-25/handoff-factorial/evidence/shared_path_audit/README.md),
[early-event video](results/handoff_factorial_20261125/videos/handoff_factorial_comparison.mp4),
[presentation bundle](results/handoff_factorial_20261125/presentation-pack.zip).

## I-43 — Bounded tilt target with a consistent desired tilt rate

**Date/status:** 2026-11-25; declared before implementation and tests; completed below. User
explicitly authorized the I-42 reference-generator proposal. No result or
default promotion is presumed. Preserve 83 source/test origins and seven old
bundles before edits; add an opt-in module/controller integration and tests.

**Question.** Does an attainable-reference candidate reduce the common early
loaded-belt demand and path departure without hiding raw-target balance error
or worsening direction changes? I-42's 0.83 m/s request against 0.30 m/s and
zero desired angular rate during changing lean motivate, but do not prove,
this hypothesis. Previous preparation masking passed old route gates yet
worsened four turns; those negative results stay relevant.

**Declared design.** Two-stage bounded tanh cascade in the 2D tilt chart,
time constant 0.08 s, chart rate <=60°/s, derived chart acceleration <=1500°/s².
Integrate previous held input with convex SSP-Heun substeps <=1 ms; then
register current observer-derived target. Initialize at 4 s with exact native
quaternion/zero rate, remain enabled through the route. Generate analytic
tilt rate and map it through the SO(3) left Jacobian/equilibrium rotation and
actual-body frame. Do not differentiate carried actual yaw or add its rate.
Preserve static ground lean and heading behavior. These caps are declared
numerical settings, not measured hardware or proven physical attainability.

**Comparison.** Three fresh 44 s same-phase +0.9 Mars runs: native, shaped
angle with zero rate, shaped angle with consistent rate. All synchronization,
contact estimation, rolling references, gain-bank settings and timed motion
program remain native. Default feature None retains legacy parameter output.
Advance generator once per command, clear on reset, preserve independent state
in native private clones. Test quaternion/frame/causality/bounds/convergence,
default saved-history behavior and integration before new physics.

**Fair metrics/screen.** Compare shaped-target and raw/native-target lean
separately, plus absolute tilt, nominal path, demand, clipping and all six
I-41 turns. Recompute raw-reference settling/route gates; a moved target must
not manufacture balance success. Primary window6.4–7.4 s, demand6.6–6.9 s;
require >=10% reduction in both primary local RMS and peak belt-A request,
raw peak lean <=baseline+0.5°, moving RMS <=1.05baseline, declared peak/final
tracking bounds and each turn no worse than baseline+max(2mm,5%). Candidate
is consistent-rate only; angle-only remains an ablation. No retuning.

**Conditional regression.** Only a full main-screen pass opens the historical
five-case suite: reuse the exact-source +0.9 pair and add eight fresh runs for
figure-eight +0.3/−1, triangle and out-and-back. At most 11 new executions;
overlapping reused pairs are not additional experiments. No automatic global
adoption, new estimator/controller family or hardware claim.

**Evidence:** [iteration record](docs/iterations/2026-11-25/tilt-reference/README.md),
[protocol](results/tilt_reference_20261125/protocol.json),
[ownership/source plan](results/tilt_reference_20261125/implementation_plan.json).

### I-43 completed — early demand improves; whole-route acceptance fails

**Implementation.** Added optional tilt_reference.py and the nested
OffroadParams.tilt_reference setting, None by default and omitted from legacy
as_dict output. Reset clears state; the raw accessor stays pure. One update
per command shapes the angle and analytically supplies the tilt rate in the
actual-body angular-velocity frame, without measured yaw differentiation.
The initial 4 s command returns the exact old quaternion and zero rate. The
only existing source modified is offroad_control.py; the module and two test
files are new. Gain bank, observer, locator, geometry, rolling allocation,
holding torque and synchronization code remain unchanged. No default switch.

**Tests/compatibility.** 109 unique selected tests pass in both environments:
32 new (20 math, 12 controller/clone/forecast) and 77 existing cases. These are
subsets; repetitions/independent overlapping stages are not additional unique
tests. Independent 100-case numerical audits verify rotation derivatives,
smooth-flow Jacobian and SSP integration, not robot trials. Full guarded
saved-measurement replay over 8,801 old observations reproduces commands,
observer wrench, normal, nominal history and references bit-for-bit. Fresh
Gazebo baseline reproduces all 17 common I-40 precision fields exactly.

**Experiments.** Three fresh 44 s +0.9 Mars figure-eight executions: native,
shaped angle with zero rate, shaped angle with consistent rate. All finish
without falling. Same nominal time/XY/V/A and comparable dynamic histories
through activation, including exact first active output. Auxiliary preactive
clock bookkeeping differs by design and is recorded in full fingerprints;
common/native plus canonical uninitialized state matches. No true contacts or
terrain enter command; solved truth is recorded afterward by evaluation only.

| Metric | Baseline | Angle only | Consistent rate |
| --- | ---: | ---: | ---: |
| Early local-path RMS mm | 38.661 | 25.889 | 29.893 |
| Peak belt-A request m/s, 6.6–6.9 s | 0.835 | 0.376 | 0.491 |
| Primary requested-clipping fraction | 9.0% | 3.0% | 4.5% |
| First sustained 20 mm departure s | 6.860 | 6.830 | 6.820 |
| Moving timed-position RMS mm | 50.486 | 61.863 | 52.180 |
| Final timed-position error mm | 190.191 | 63.058 | 35.605 |
| Full raw/native-reference peak lean error deg | 8.709 | 13.135 | 13.581 |
| Full used/shaped-reference peak lean error deg | 8.709 | 2.399 | 6.973 |
| Full absolute platform tilt max deg | 13.784 | 13.647 | 13.652 |

**Partial benefit.** Consistent-rate early RMS falls 22.7% and peak demand
41.2%, although 0.491 m/s still exceeds the 0.300 m/s simulation bound. First
event amplitude/duration decrease, but onset moves 40 ms earlier rather than
being prevented/delayed. Primary raw peak lean rises only 5.328→5.580°, within
the declared +0.5° tolerance. Full moving RMS rises 3.35%, within its declared
5% tolerance, while peak/final error improve. Angle-only does better on early
error/demand but worse on primary raw lean and route RMS; it remains an
ablation, not an outcome-selected substitute.

**Regressions and decision.** Full raw-reference error rises 8.709→13.581°,
failing the +0.5° condition. A lower shaped-target error would conceal that
tracking mismatch; it is not evidence of toppling, and all absolute tilts are
similar. Four of six fixed turn overshoots worsen: first 0→24.469 mm, third
0→33.272, fourth 9.836→35.306, sixth 60.204→94.125. First reversal lag improves
1.101→−0.035 s but spatial overshoot grows; fifth reversal is not sustained
within the fixed window despite smaller overshoot. The overall predeclared
screen fails. Keep native default; retain the optional prototype and negative
results. No post-outcome retuning, no conditional eight executions, exactly
three physical runs and zero retries. Further parameter/candidate work is
separate; no validated overall improvement is claimed.

**Raw QA.** Both enabled histories replay exactly over 8,801 inputs: one
initialization plus 8,000 advances each, substeps <=1 ms, independent frame
error <=3.34e−16 rad/s. Observed chart rate36.25/45.12°/s and acceleration
823.89/813.42°/s² remain within60/1500. The latter is a chart bound, not body
component acceleration or measured motor attainability. Raw/shaped summaries
and settling/gates are both preserved with correct Gazebo/DART metadata.

**Retained development.** Initial19-test math stage lacked the preactivation
backwards-clock case; review corrected it and expanded to20 without replacing
the first snapshot. Harness stages9/12/13 overlap; a ROS pytest-plugin failure
occurred before collection and was corrected by isolated plugin loading.
An independent raw-QA 3.5e−13 rad discrepancy came from full-precision mass
metadata versus actual rounded model XML; exact model-constant correction and
failed source stage are preserved, with no raw/control/simulation change.
Initial renderer compile/preview and later presentation refinements remain
separate. Runtime source lock126 references stayed unchanged through physics.
All 83 pre-change source/test origins and seven older bundles are retained.

**Evidence:** [full iteration record](docs/iterations/2026-11-25/tilt-reference/README.md),
[main report](docs/iterations/2026-11-25/tilt-reference/evidence/analysis/stage01/report.json),
[comparison video](results/tilt_reference_20261125/videos/tilt_reference_comparison.mp4),
[conditional-run decision](results/tilt_reference_20261125/regression_decision.json),
[presentation bundle](results/tilt_reference_20261125/presentation-pack.zip).

### I-44 proposed — remember observed terrain for contact preview (2026-11-26)

**Question.** Can depth observations captured before arrival improve future
contact geometry when the wheel later occludes the surface? This could help
on the first traversal if the terrain was visible ahead. The bump remains a
plausible contributor, not a complete proven explanation for drift/overshoot.

**Current architecture audit.** The I-40–I-43 native baseline uses observer-only
`hardware` mode, least-squares allocation, synchronization enabled and tilt
shaping disabled. It includes support inference/filtering, timed trajectory and
lean references, gain-scheduled LQR, rolling/static feedforward, 120 ms incoming
drive preparation, clipping and low-load handling. Its exact simulated body and
joint state is not a real localization pipeline. Existing cameras render the
scene; no robot depth/LiDAR ingestion, SLAM or persistent sensed terrain map
was found. The 200 ms dynamic forecast remains a shadow diagnostic. Generic
`OffroadParams()` still defaults to `terrain_aware`; this audit does not claim
all repository modes are observer-only.

**Proposed change/comparison.** Keep LQR unchanged. First use the saved native
Mars +0.9 trace with a finite-view, occlusion-aware ideal range adapter and
explicit perfect-pose registration. Compare observer-plane, current-frame-only
and accumulated-memory geometry under the same motion preview. Preserve
unknown regions, acquisition timestamps, causal pose history and uncertainty;
score coverage/abstentions as well as geometry and warning timing. Only the
sensor adapter may access scene geometry. Truth contacts remain evaluation-only.
Declare sensor settings and quantitative gates before any execution.

**Limitations/decision.** Map geometry cannot supply actual load shares,
friction or slip, and general multiple-surface contact remains unverified.
Mars visual heightmap/collision-mesh differences and robot self-occlusion must
be accounted for. A first ideal replay would establish utility of visibility
and memory only; pose drift/latency and a real sensor frontend come later.
No runtime/default changes, new simulations, sensor results or adoption. Keep
the failed I-43 whole-route screen and unresolved overshoot visible.

**Evidence:** [design and experiment proposal](docs/iterations/2026-11-26/terrain-map-preview/README.md),
[source audit](docs/iterations/2026-11-26/terrain-map-preview/source_audit.json),
[dated presentation notes](docs/iterations/2026-11-26/README.md).

**User clarification, 2026-11-26.** Replace the initial single-view starting
proposal with four virtual depth cameras around the platform: changing yaw means
one camera need not face the travel direction. Proposed azimuths are spaced
90 degrees apart; mounts, downward angles, overlapping fields of view and actual
ground coverage must be declared/checked before execution. Fuse all four views
in one causal terrain memory. Use exact simulated body pose and velocity for
control and capture-time registration at this stage; developing a state estimator
or SLAM is deferred. The latest-frame comparison now uses all four cameras.
This is a proposal revision only; no sensor implementation or new run occurred.

### I-44 implementation started — four-camera shadow replay registered

**Authorization and scope.** The user approved proceeding with four depth views
and exact simulated body state. Add separate range-adapter and observed-terrain
memory modules, focused tests and an offline campaign runner. Existing LQR,
observer, allocation, synchronization, defaults and old rollouts stay unchanged.

**Registered before campaign outcomes.** Four platform-mounted views at body
yaw offsets 0/90/180/270°, 45° downward, 64×48 pixels, 100° horizontal field of
view, 0.08–2 m Euclidean range, 20 Hz capture and 50 ms delivery latency. Use
capture-time exact pose. Static 10 mm observed cells and 40 mm local plane fits
require observed query cells, surrounding support and declared fit/ambiguity
checks. Query only delivered observations; current-view and memory arms use the
same four cameras and identical constant-velocity motion previews.

**Comparison.** Reuse the saved 44 s native Mars +0.9 figure-eight; no new
Gazebo execution or changed commands. Evaluate 119 unique anchors (109 uniform,
11 dense near 6.7–6.9 s, one overlap) at 50/100/200 ms. Separate geometric surface
coverage/error from future actual-contact candidate error, which also includes
motion-model error. The observer-plane comparator is a newly constructed
current tangent, not an exact reconstruction of the cached live synchronizer.
Collision-mesh face normals are geometric proxies, not saved force-weighted
contact normals. Unknown and rejected-fit queries remain in coverage totals.

**Gates and preservation.** Register integrity fixtures and all-query utility
criteria before replay. Source/old-bundle snapshots precede implementation;
failed setup/test/render stages must remain visible. A useful geometry result
would justify sensor work, not live-controller adoption or an overshoot claim.

**Evidence:** [registered protocol](results/terrain_memory_20261126/protocol.json),
[pre-execution clarifications](results/terrain_memory_20261126/protocol_clarifications.json),
[pre-change source/evidence manifest](results/terrain_memory_20261126/pre_change/manifest.json).

### I-44 completed — memory recovers terrain; contact forecasts still regress

**Implementation/checks.** Added only separate terrain_depth.py,
terrain_memory.py, the offline runner and three focused test files. Existing
LQR/observer/allocation/synchronization/defaults remain unchanged. Final selected
suite: 75 unique cases in each environment (51 new, 24 existing); repeated and
earlier overlapping stages are not extra tests. Exact final sources and both
logs are sealed. Initial71-case smoke lacks a dedicated complete pre-command
snapshot/log; that limitation is retained. A one-ulp sloped-pole failure was
reproduced/preserved and fixed minimally before contact-preview execution.

**Execution.** One reused44s Gazebo/DART Mars +0.9/yaw45° history, zero new
physics executions. 881 four-camera batches (3,524frames), 119 unique anchors
and357 anchor/horizon pairs evaluated by3 methods. The109 uniform and11 dense
anchors overlap once; report strata separately. No future frames/terrain truth
enter the mapper or preview. Shared CV agrees with independent MuJoCo
integration within2.22e−16; original inputs/commands remain unchanged by hash.

**Registered primary result.** Uniform terrain availability is observer327/327,
latest-four0/327, memory277/327 (84.71%). Latest frames contain terrain but cannot
see those underbody query locations. Memory covered heightRMSE0.683mm and
normalp904.496° satisfy all four declared terrain-utility gates. Dense6.7–6.9s
memory coverage is only3/33, with1 usable contact candidate. First usable
surface preview is6.78s/+200ms; this does not establish dependable anticipation
of the original first-deviation event. No camera/map parameter retuning.

**Post-registered matched supplement.** On277 identical surface pairs,
heightRMSE1.274→0.683mm and normalp909.450→4.496° improve, while their medians
worsen (0.0556→0.3025mm,0.907→1.414°). On271 identical future-contact pairs,
nearest-candidate-to-equivalent-point RMS worsens7.903→11.304mm; memory is
better53/worse218 pairs. The metric includes shared CV motion error and is
optimistic for ambiguous candidate sets; it does not measure load sharing or
force accuracy. Unloaded fallback points are excluded. Geometric family
warnings include false changes, retained explicitly in the raw report.

**Decision/limits.** Retain four-view memory as an offline geometric prior;
do not adopt it into live gains/torques/transitions. Passing the sensor-utility
screen is not controller acceptance. Resolve early visibility and the contact
geometry/motion mismatch before live comparison. Ideal depth, exact body state,
static2.5D fits and compiled-face normal proxies remain assumptions; no SLAM,
general multiple-surface validation, real-time or hardware claim. Drift and
overshoot are unchanged because this is shadow replay.

**Independent QA/preservation.** Full depth pose/latency/calibration/source-mesh
checks and64 direct-triangle ray samples pass. Map QA verifies causal prefixes,
snapshots, shared CV, reconstruction and raw reductions. QA scope/parser
failures are preserved and corrected separately; primary data is unchanged.
Both positive terrain results and adverse contact results appear in the video
and presentation record. Large478MB derived map snapshots stay local; the
bundle retains raw inputs needed to regenerate them.

**Evidence:** [completed record](docs/iterations/2026-11-26/terrain-map-preview/README.md),
[primary report](results/terrain_memory_20261126/campaign/analyze_stage01/report.json),
[matched supplement](results/terrain_memory_20261126/analysis/matched_pairs_stage01/report.json),
[video](results/terrain_memory_20261126/videos/terrain_memory_shadow.mp4),
[presentation bundle](results/terrain_memory_20261126/presentation-pack.zip).

### I-45 proposed — one decision test before more contact-preview development

**Question.** The user sees no driving improvement and asks whether continued
investigation is worthwhile. I-44 was unchanged-command replay, so its video
cannot measure a driving benefit; its separate contact-point forecast worsened.
Avoid expanding cameras/SLAM or changing live feedback on that evidence.

**Proposed comparison.** Reuse I-44's fixed119 anchors/three horizons. Freeze
each map and current observer tangent at the anchor. Evaluate both providers
with existing CV configuration and with actual saved future body/joint
configuration. Future pose is explicitly diagnostic hindsight; no future depth,
terrain, observer update or solved contact enters generation. Evaluate future
contact records afterward. Report all-slot coverage, early abstentions and
fixed commonly eligible errors; separate single contacts from multi-contact
equivalent averages. No source/settings change or new run is performed now.

**Decision rule.** If memory remains worse with actual configuration, park
current map-to-contact/control integration: motion extrapolation alone cannot
explain the disadvantage. If memory helps only with actual configuration,
investigate motion prediction next. Missing early observations remain a separate
coverage limit. Exact numerical protocol/matching/effect thresholds must precede
execution; this is a proposal, not a registered or completed experiment.

**Prior evidence/limit.** I-41 used future commands, not actual future states;
I-42 feedback-share truth did not move the first departure and documented large
balance demand/clipping. Earlier broader record oracles used another phase/
window and cannot rule out every full-information live experiment. A broad
new live oracle is deferred in favor of this narrower diagnostic. LQR/reference
feasibility and flat-ground turning remain the next driving questions if this
particular forecast has no useful advantage. Existing negative trials remain.

**Evidence:** [proposal and stopping branches](docs/iterations/2026-11-26/contact-preview-pose-isolation/README.md),
[I-44 matched results](results/terrain_memory_20261126/analysis/matched_pairs_stage01/report.json).
I-44 sealed source/data/archive are unchanged; living notes/log receive this
later proposal. No new tests or performance claim.

### I-45 registered — hindsight configuration test authorized

The user approved proceeding. Keep all I-44 settings and119 anchors/three
horizons; four conditions give1,428 rows over357 unique slots. Add separate
diagnostic module/runner/tests only. Depth, original commands and previous
results are reused; no new scene/physics/controller execution. Future qpos
(body and all three joint configurations) is explicitly hindsight, used with
zero extra horizon/velocity under the unchanged geometric preview. Map and
observer provider knowledge stay at the anchor. Generation precedes truth scoring.

Before outcomes, logger audit refines primary truth to "single resolved-contact
patch mean": total counts above0.05N equals1, one patch exceeds declared load
threshold, selected point finite, other-patch load<=0.05N. Weak contacts may
remain in the mean; do not call it an exact physical collision point. Preserve
legacy global averages separately and retain the last1ms contact/state convention.

Register fixed four-way complete/nonempty matching without dropping wrong
identities. Minimum30 pairs/50% eligible support; material RMS gain>=10% and
>=0.5mm, p95<=observer+0.5mm, matched identity loss/ambiguity gain<=5pp.
Oracle-map complete/nonempty uniform coverage>=70% and <=5pp loss vs CV-map,
not vs observer extrapolation. Insufficient or mixed results do not justify
integration. These are diagnostic branch rules, not a control success gate.

Preserve pre-change110 source files and14 older bundles; sealed I-44 runtime
and test hashes initially match. Require full CV reproduction and causal-prefix
checks before interpreting any oracle contrast. No outcome or final test count
is claimed at registration.

Evidence: [protocol](results/contact_preview_pose_20261126/protocol.json),
[registered record](docs/iterations/2026-11-26/contact-preview-pose-isolation/README.md),
[pre-change manifest](results/contact_preview_pose_20261126/pre_change/manifest.json).

### I-45 completed — motion matters; no material map-contact advantage

**Implementation/scope.** New contact_pose_diagnostic.py, runner and two test
files only; all previous runtime/scripts/tests/settings stay unchanged. Rebuild
anchor-limited memory from saved depth; generate observer/memory×CV/actual-qpos
before truth scoring. No scene, future depth/observer update, physics or
controller call. Stream the first old map prefix without duplicating 478 MB.

**Integrity.** 1,428 rows / 357 unique slots complete. All 714 CV rows reproduce
I-44 within 1e-10 with zero failures; all 119 delivery prefixes match, first
17,595-cell prefix exactly. Source/input hashes and map locks pass. Offline
runtime 45.56 s; no real-time or new-robot-trial claim.

**Registered primary outcome.** Among 327 uniform slots, 292 meet the resolved
patch-mean rule and 244 are complete/nonempty across all four conditions
(83.56% of eligible). No predicted identity filtering. RMS mm: observer CV
7.951, memory CV 11.492, observer actual-pose 7.476, memory actual-pose 7.722.
Actual configuration improves memory 32.8% but leaves it 0.245 mm / 3.28% worse
than observer. Both registered material RMS gains fail; other quality/support/
coverage checks pass. Motion contributes substantially; no unique residual
cause or uniform failure is claimed.

**Mixed metrics.** Actual-pose memory median 3.516 mm versus observer 0.209 mm
and patch membership 233/244 versus 244/244 are worse. Its p95 18.413 versus
19.402 mm and maximum 20.885 versus 34.231 mm are better. Legacy 271-pair CV
scores preserve 7.903/11.304 mm; actual-pose gives 7.393/7.602 mm. Keep these
global averages separate from primary resolved patch means, which may include
weak contacts and retain the last-1ms timing convention.

**Coverage/onset.** Both memory branches have 276/327 complete/nonempty uniform
previews. Dense actual-pose centre support rises 3→7/33, but all seven have
incomplete contact-foot regions: zero complete previews versus CV's one.
No dense four-way matched RMS exists; accurate future configuration does not
establish a fix for the first departure.

**Decision.** Apply registered branch: park current map-to-contact integration.
Keep native LQR/observer/feedforward/synchronization unchanged and memory
offline. No post-outcome tuning/new live oracle. Return to separately declared
reference/actuator feasibility and turning work for driving improvement. Terrain
sensing generally is not rejected; local geometry/smoothing, ideal rims,
averaged targets and timing remain possible residual contributors.

**Validation/development.** 61 unique selected tests pass in both environments
(28 new + 33 existing); repeated stages/environments are not added. Independent
raw QA verifies all rows, truth, poses, masks, statistics and gates. A 1e-16
quaternion fixture failed by 1.11e-16; source/log and corrected legacy-equivalence
check are retained without algorithm change. Unqualified-python preflight 127
was corrected before calculation. Presentation active-log manifest correction
is preserved; scientific/media outputs stay unchanged. Source/log/plots/QA/
video/bundle retain favorable and adverse findings.

**Evidence:** [completed record](docs/iterations/2026-11-26/contact-preview-pose-isolation/README.md),
[primary report](results/contact_preview_pose_20261126/campaign/stage01/report.json),
[independent QA](results/contact_preview_pose_20261126/review/final_report.json),
[diagnostic video](results/contact_preview_pose_20261126/videos/contact_pose_diagnostic.mp4),
[presentation bundle](results/contact_preview_pose_20261126/presentation-pack.zip).

### I-46 executed / analyzed — current filtering retained

**Question/current stack.** User requests the prior LQR/momentum-observer/
contact-feedforward baseline and asks whether misassignment from lag is
inevitable. That selected live baseline was never replaced by camera outputs.
It has active 120 ms CV incoming-drive preparation; newer dynamic/map contact
forecasts remain offline. Inferred records affect rolling references; inferred
shares blend LQR gains and holding torques. Native least-squares constraints
are not load-share weighted. Keep estimator complexity fixed for this audit.

**Finding/limit.** Observer interval-mean wrench is filtered at 100 rad/s,
nominal 10 ms time constant, then located against current configuration. A
time constant is not a fixed delay. Current geometry plus filtered history can
produce premature, late or chattering support labels. I-41 records both leading
belt-A assignment at 6.725 s and trailing shell-A assignment at 6.860 s. Separate
normal/load filters are not the direct support-label filter. Causal sensing
needs evidence after an unknown force change; large wrong-drive commands need
not be treated as inevitable.

**Proposed comparison.** Keep commands unchanged. Reconstruct already-available
interval-mean versus filtered momentum-observer wrench on saved 6.4–7.4 s
measurements; apply identical locator/current configuration. Compare routing,
geometry margins, gains/holding torques and command steps, with explicit
truth/interval timing and noise limits. Register exact criteria before any
execution. This is a diagnostic, not an adopted fast-routing controller.

**Decision constraints.** If useful, separately test transition detection versus
smoothed force magnitude and controller tolerance of ambiguity, accounting for
existing gain blending/preparation. Do not blindly add hysteresis, fixed time
advance or drive blending. I-42 true-share feedback avoided one switch without
preventing departure; demand was already 0.835 vs 0.300 m/s. I-25 interval-mean
truth shares regressed on another event. Better routing is not a promised cure
for saturated tracking. Keep feasibility/turning questions separate. This was
the original proposal; the approved execution follow-up is recorded below.

Evidence: [architecture and proposed audit](docs/iterations/2026-11-26/support-routing-timing/README.md).
Earlier sealed sources/reports/bundles remain unchanged.

**Approved execution follow-up (2026-11-26).** Implemented new shadow-only audit
module/runner and 24 focused tests, passing in both environments with final
sources unchanged. Registered protocol/clarifications before scientific
outcomes. Geometry-only Mars scan rejected all 2,091 candidates under the first
1 mm RMS / 3 mm planarity rule; an independently checked, explicitly revised
3 mm / 10 mm rule yielded 16 candidates. Selected center (-2.3,8.4), fitted
slope 4.100 degrees, RMS 2.776 mm, rock/lava distance 1.176 m. Slow 18 cm
out-and-back, phase 1.10 rad, yaw 45 degrees, 16 s, plus flat-plane companion.
Historical 44 s replay and separate stepping fixtures launched; native routes
require a passed hash-bound stepping comparison. Raw/effective support shares,
last-1-ms versus mean-5-ms truth, pure gain/holding sensitivity and local paired
wrench noise are distinguished. Registered timing/error/command/noise/coverage
gates precede outcomes. No faster controller or new estimator is actuated.
Full registration, negative site-selection result and limitations are in the
linked I-46 record.

**Completed result.** Both 16 s native cases completed with five raw 1 ms
samples per 5 ms interval. Both transfer directions and the exposure guard pass.
Seven common stable family transitions give 0 ms median absolute timing benefit.
Mars mismatch-equivalent duration improves 5→0 ms, flat stays 10→10 ms. The
following-hold diagnostic stays Mars 5→5 ms / flat 0→0 ms, so improved actuation
is not demonstrated. Maximum normalized requested steps are unchanged at
0.39916 and 0.001515. Static paired wrench-noise and false-switch gates pass;
timing and per-case absolute-benefit gates fail. Historical last-1-ms mismatch
improves 90→55 ms, but the first correcting cue at 6.725 s follows belt-A demand
at its bound at 6.700 s; the maximum requested jump remains 2.78247. Keep the
filter and native stack. No live candidate or tracking improvement is adopted.
The simple route supplies handoff regression evidence; later improvement should
revisit saturation and turning response.

**Validation and failures.** 54 distinct selected pytest cases pass in each
environment (24 new + 30 existing; not the full repo suite). Separate bridge
fixtures match full states/contacts exactly; 8,801 historical command/reference
rows reproduce bit-for-bit. Independent reconstruction of 32,000 raw route
substeps matches mean loads and wrenches. The first flat setup failed with zero
commands because a zero-coefficient plane feature still declares nonflat terrain.
A separate frozen flat-only driver copy corrects the constructor and uses a
fresh output directory; the original failure is intact. The generated flat cache
was reused, but no previous simulator state existed. Independent review caught
floating event ties before reduction; failed source/check and corrected review
are retained. The raw checker JSON-serialization failure is retained and fixed
only in a new stage. Source, protocol and plugin proofs remain bound. See the
I-46 record for reports, media, compact evidence and scope.

Evidence: [video](results/support_routing_timing_20261126/videos/support_routing_timing.mp4),
[canonical report](results/support_routing_timing_20261126/analysis/stage01/report.json),
[independent numerical QA](results/support_routing_timing_20261126/review/reduction_numeric_stage01/report.json),
[presentation bundle](results/support_routing_timing_20261126/presentation-pack.zip).

### I-47 — presentation study guide and saved-log investigation

**Request/scope.** Explain the current dynamics, LQR, rolling/holding feedforward,
momentum observer, support estimator and active geometric preview, and inspect
prior saturation, weak-authority, contact-timing and turning evidence. The user
requested a PDF for presentation preparation and no new simulation; cameras
and deployment are excluded from the talk scope.

**Work.** Produced a 33-page study guide with a five-minute outline, equations,
worked examples, source/method citations, attribution and practice questions.
Read-only audits recheck saved inputs/component sums, analytic drive geometry,
41 contact-event samples, seven transfers and six turn events. Source snapshot
preserves 145 origins and hashes 16 prior presentation bundles.

**Findings.** The final clip enforces input bounds; the unconstrained Riccati
cost and native reference allocation do not anticipate feasible total commands.
At6.825s, beltA request0.8347418461 equals LQR0.8438042577 minus rollingFF
0.0090624115, with88.604% of LQR from attitude/rate and applied0.3000m/s.
A single-shell configuration can lose a rolling direction; a separate31.650s
sample has condition number3961 and huge rate references. Neither mechanism
is automatically the other's cause. Lockstep physics does not advance during
Python computation; these logs do not establish compute-delay-induced staleness.
Wrench memory, geometry and solver/state conventions remain separate. First
Mars turn shows delayed reversal without geometric overshoot; flat also has
turning overshoot. Existing tradeoffs/negative trials remain unchanged.

**Decision/validation.** Keep current controller unchanged; propose consistent
reference/control feasibility and turn-response work without executing it here.
No new simulation, controller/observer evaluation, deployment claim or new
repository-test count. Draft math/layout failures and scientific wording
corrections are retained. Independent scientific/layout and final artifact
checks accompany the PDF.

Evidence: [study guide PDF](docs/iterations/2026-11-26/presentation-preparation/control-and-observer-study-guide.pdf),
[iteration record](docs/iterations/2026-11-26/presentation-preparation/README.md).

### I-48 — contact clock and radius-convention diagnostic

**Scope.** User requested clarification of soft effort cost versus hard output
limits and further investigation from existing contact records. No new physics
or controller/observer evaluation. Preserve prior source/narrative snapshots
and hashes of 17 earlier presentation archives.

**Method.** Register prior/current pose proxy × raw/radial-R individual-point
projection on I-46 saved1ms admitted smooth contacts with normal load>0.05N.
Exclude firstsubstep; verify clock/iteration/reset continuity. Hold point/tag/
load fixed. Compare the actual C++ radius-normalized band convention rather
than treating raw equivalent point/updated pose as identical to admission.

**Result.** Mars17,107 eligible records: postraw4 outside bands, priorraw5,
normalized0 for both poses. Flat15,999: raw10 for both, normalized0 for both.
Independent NumPy calculation agrees. Thus normalization accounts for these
particular individual-point comparison violations; pose shift alone does not.
This does not measure observer error, establish computation delay, change any
admission decision or demonstrate improved driving. Prior pose remains a proxy;
rejected/weak/rim/body records are outside this primary statistic.

**Secondary historical check.** On41 old6.700–6.900s equivalent-mean samples,
raw and radial-R projection both violate the dominant ideal band8times.
At6.860s normalization only changes−78.173→−74.168µm. Thus the new individual-
contact normalization result does not explain away the original discrepancy;
that older record lacks1ms individual/prior-pose data. The secondary protocol
preceded its arithmetic and does not alter primary criteria.

**Decision.** The previous logs CAN investigate observable filtering/geometry/
clock effects, while missing CPU/transport timing remains unmeasurable here.
Keep current control unchanged. Clipping already occurs inside controller and
plant; a constraint-aware design would need the total feedback/feedforward/
preparation command and consistent reference priorities, not only a larger R.

Evidence: [diagnostic record](docs/iterations/2026-11-26/contact-clock-disentangling/README.md).

### I-49 — belt-speed capacity sweep completed, fixed LQR gains

**Question.** User proposes more belt-speed capacity to relieve the saturated
balance-demand case before choosing motor capability. Test the assumption in
simulation without simultaneously retuning the controller.

**Registered conditions.** Three fresh44s originalMarsfigure8 cases, phase+.90,
yaw45°, operatingcaps bothbelts .30/.60/.90m/s. Original.30 model/config/design
plant, gainlibrary andordinaryK fixed; overrideonly physical/mirror operating
bounds afterdesignconstruction. Torque limits/path/observer/locator/algorithm
unchanged. Incomingpreparation usesnewoperatingcap, so this is not finalclip-only.
152 origins and17 oldpresentationarchives preserved beforework.

**Criteria/scope.** Full original timederrors/settling, raw/referencelean,
all6turnovershoot/lag, localdeparture, old/newcapheldsaturation, torqueclipping,
slip/load andexactbaseline/gainparity. Provisional benefit/nonregression rules
registered beforeoutcomes. Idealbeltservo; no motor-load or hardwarecapability
validation. No defaultchanges, retuning oroutcome-basedretries. Tooling/source
checks pass; completed outcomes follow.

**Observed result.** All three44s nativecases complete withoutfall. .30baseline
reproduces all17commonhistorical arrays bit-for-bit; bank/ordinaryK/initial/ref
identities fixed. Raising .30→.60/.90 changes movingRMS50.486→52.521mm (+4.03%),
peak190.901→110.009mm, final190.191→49.231mm and peakrelativelean8.709→6.053°.
Firstlocal20mm departure6.860→6.850s. Neither raisedarm settles or passesall
route/turn/provisional gates. Turn4overshoot9.836→17.412mm; its signedlagguard
fails even though absolutephase mismatch improves. Preserve all tradeoffs.

**Unexpected equivalence.** .60/.90 saved motion/wrench/forces/loads identical,
but appliedbeltA differs9holds(45ms), max.600vs.834742m/s. Sevenholds loadedat
bothendpoints; .60boundwasactive. Calculatedslipdifferswhilebodymotiondoesnot,
consistentwithsliding/limitedcontactresponse, notuniquelyproven by5mslogs.
More speed alone is insufficient; no motor chosen or default changed.

**Checks/record.** Nine distinct targetedpure/integrationcontracts, repeated
integrationenv notadditive. Independent raw/numeric/scopechecks pass. Wrong-key
QA adapter and renderer syntax drafts retained/corrected withoutphysicsreruns.
No existing runtime/config source edits, retuning or extra trajectory runs.

Evidence: [registration and scope](docs/iterations/2026-11-26/belt-capacity-sweep/README.md).

### I-50 — saved-log deviation and incoming-belt handoff investigation

2026-11-26. **Registered; analysis in progress, no new physics or runtime changes.**

Investigate I-49's large departure, the user's 18 s playback cue (recorded
32 s), and what incoming-drive synchronization actually guarantees. Preserve
all three native capacity histories. Distinguish timed/local/final-goal error,
command-origin arithmetic and physical-cause hypotheses. Audit actual blends,
loaded-patch cutoff, caps and kinematics without rerunning the controller.

Evidence and fixed windows: [I-50 registration](docs/iterations/2026-11-26/belt-handoff-investigation/README.md).

**Completed saved-data/source audit.** Playback 18 s is recorded 32 s. During
baseline shell/belt alternation, the belt-B → shell-B switch at 31.650 s
produces a nearly singular rolling matrix: axes 0.057775° apart, normalized
condition 3850.507. Exact private kinematics reconstruct saved references
+488.601/−1760.525 rad/s. Joint-speed LQR feedback produces requested shaft/
shell-B torques +111.389/−141.067 N m, clipped to +1.5/−0.6 N m. Preparation
is zero there. This is a reference/feedback failure during handoff, not proof
that all motion errors have this cause.

The raised arms provide another mechanism: at 32.170 s, estimated belt-B load
share switches 0→1, preparation weight 1→0 and belt command
−0.012112→−0.303361 m/s (LQR −0.319806 + rolling +0.016445). Physical slip
was low before the jump and rises at the next endpoint; at 32.190 s the
command changes −0.451307→0 during return to shell support. Existing preview
prepares incoming no-slip velocity, but loaded feedback does not guarantee
continuous handoff. Outgoing velocity is diagnostic, outside the target equation.

**Separate late drift remains unresolved.** The baseline peak at 43.985 s,
190.901 mm, is real local path departure. Its continuous late departure crosses
20 mm at 38.640 s and 50 mm at 40.625 s before late handoff. Evaluated and
estimated support both identify shell B, with accurate normal, well-conditioned
allocation and unsaturated commands at these samples. The raised peak at
39.630 s, 110.009 mm timed / 108.537 mm local, also has correct shell-B support
and unsaturated commands. Saved arithmetic does not uniquely identify the
physical cause of the entire late excursion. Earlier I-42 interventions prevent
blaming the early departure on support misassignment alone. Keep these event
histories distinct.

**Decision/checks.** No physics, controller or observer execution and no runtime/
default edits. Independent saved arithmetic, kinematic/source and interpretation
checks pass; a quarter-speed original-state replay shows support, preparation
and command overlays. Protect 156 prior origins and 18 archives. Audit setup/
syntax and renderer-footer failures are retained before corrected stages.
Next test reference feasibility and command continuity as separate candidates;
late unsaturated drift needs its own model/feedback audit. Do not adopt previously
failed uniform damping or claim a controller improvement from this investigation.

### I-51 — reference-feasibility and handoff-continuity isolation

2026-11-26. **Registered; implementation/execution pending.**

The user asks if I-49's late drift was intentionally allowed by allocation,
and requests the I-50 follow-up investigations. The active baseline was plain
least squares with no feasible motion-reference relaxation; original XY targets
remain. Investigate other allocation/feedback causes without mislabeling error
as a deliberate allowance.

Register four fresh original 44 s Mars figure-eight arms: baseline/reference
guard at 0.30 m/s, baseline/handoff guard at 0.60 m/s. Keep original design,
observer, targets and operating-cap interface. Reference guard uses existing
20 rad/s and 100 rad/s² assumptions only on joint-rate references; handoff guard
uses previous applied belt input and 3 m/s² only during observer-load crossings.
No combined candidate, outcome tuning or runtime/default edits. Preserve all
native and modified commands and the 159 captured origins / 19 prior archives.

Mechanistic contracts and whole-route/turn/lean comparisons are distinct; late
unsaturated drift gets a separate saved-data model/feedback audit. Full scope:
[I-51 registration](docs/iterations/2026-11-26/allocation-handoff-isolation/README.md).

**Additional registered diagnostic.** Saved late-window algebra establishes
unchanged XY/body targets, negligible LS residual and low contact slip, but
joint-rate tracking error supplies much of the drift velocity. World-Z versus
normal-relative gain phase differs 7.83° at 41 s. Register one separately
captured `phase_late030` case: identical original history before 38 s, then
only normal-relative phase scheduling through 44 s. Keep the locked four-arm
protocol untouched; require exact common-field parity before the switch.
Earlier global scheduling regressions preclude treating this as a known fix.
Maximum five native cases; no outcome tuning or automatic adoption.

**Completed outcomes.** All five fresh cases finish 44 s without falling.
The two primary baselines reproduce all 41 common I-49 precision fields
byte-for-byte. Independent native QA reconstructs the guards exactly and
confirms actual prior applied input, reference/design identities and truth guards.

The 0.30 reference guard changes moving RMS 50.486→44.213 mm, peak
190.901→97.186 mm, final 190.191→32.687 mm and peak relative lean
8.709→6.346°. It modifies 53 held samples / 265 ms from 6.700 s. In its own
history native references never reach the 20 rad/s box; the active intervention
is reference slew limiting at 100 rad/s². It is not an isolated replay of the
old 31.650 s singularity. Turn-four overshoot worsens by 11.468 mm and
settling/route/turn guards fail. Preserve signed versus absolute lag distinctions.

The 0.60 handoff guard improves moving RMS 52.521→42.056 mm and final error
49.231→15.575 mm, but peak relative lean worsens 6.053→11.683°. Its 496
modified holds / 2.480 s obey the 3 m/s² active-episode bound. Mechanistic
smoothness is not balance qualification; its turn/settling gates also fail.

The late phase diagnostic matches 47 common fields across all 7,600 samples
before 38 s and the physical/observer state at the switch. Only scheduling
changes then. Final error improves 190.191→53.993 mm; late 38–44 s RMS
84.652→56.707 mm. Moving RMS is essentially unchanged, 50.486→50.495 mm.
This establishes an effect of scheduling in this episode, not a general fix
or approval of a hardcoded time gate. Prior global scheduling failures remain.

**Hypothesis and decision.** Baseline targets were never deliberately relaxed;
late LS residual and slip were negligible. Continued joint-rate error and
platform rotation explain the velocity mismatch kinematically; soft LQR state
objectives and phase scheduling remain distinct from hard target tracking.
Keep defaults unchanged, preserve the useful reference-slew candidate, reject
adopting broad belt-transfer slew as balance-safe, and investigate consistent
contact-relative scheduling before a combined policy. No broader trajectory
runs after failed candidate screens. Full reports, movies and retained setup
failures are linked in the I-51 record above.

### I-52 — whole-run scheduling × joint-reference tracking across configurations

2026-11-26. **Registered; native outcomes pending.**

The user authorizes the I-51 next step. Register 12 fresh original 44 s Mars
figure-eight runs: phases −1.00/+0.30/+0.90, each baseline B, exact I-51
reference guard R, whole-run estimated-normal phase N, and both NR. All caps
remain 0.30 m/s; no time gating, new weights, belt-transfer guard, new estimator
or runtime/default edits. Preserve 163 prior origins and 20 older archives.

The phase flag changes lookup in a flat gain bank, not complete terrain-frame
LQR design. Ordinary local design/initial physical state identities are scoped
within each starting phase; initial scheduled K can differ by the phase factor.
Report every per-phase paired/factorial contrast, both native-target and
used-target joint-rate tracking, original route/turn/lean/settling checks and
all unfavorable outcomes. No pooled average or mechanistic bounds alone can
qualify a policy. Source consistency and historical-evidence audit accompany
the experiment. [Registration](docs/iterations/2026-11-26/contact-scheduling-joint-tracking/README.md).

**Historical correction before new outcomes.** The recovered old MuJoCo
normal-phase comparison improves cost by 3.7168% (2.7456547→2.6436046), with
zero falls in 14 runs per setting, but misses its 5% adoption margin. Previous
broad wording about global scheduling regressions is not supported by this
recovered comparison. A legacy Mars-falls comment has no recovered referenced
run folder and is not a quantified result. Preserve the old I-51 record/archive
unchanged and carry this correction forward. Exact old tuning JSON and the
distinct current explanatory source are captured in the I-52 source audit.

**Technical attempt retained.** Original `m100_N` exits 139 without usable
rollout/summary or saved command rows. Preserve its empty log, provenance,
sources and flat-model workspace; exact native fault site remains unknown.
Register one identical-source/parameter retry, run serially in a fresh output
root with Python fault tracing. Accept its physical outcome without retuning.
Keep this extra technical process attempt separate from the 12 policy cells,
and bind an explicit case-path resolver in a new analysis source stage.

**Native execution complete.** Eleven original processes and the one registered
identical-policy serial retry finish all 44 s / 8,801 observed calls without
falling. Keep thirteen process attempts (one unscorable native crash) distinct
from twelve complete policy cells. The original crash site remains unknown;
no claim of a controller fix or simulator root cause follows from retry success.
Outcome reduction and independent checks use the explicit preserved resolver.

**Analyzed outcome.** N lowers moving RMS at every start: −1.00
51.080→39.195 mm, +0.30 56.963→53.936 mm, +0.90 50.486→43.284 mm.
It is not a universal nonregression: at +0.30 peak error rises 154.280→192.143 mm
and relative lean rises 9.484→10.706°. At −1.00 N settles and passes original
route gates, but turn-two overshoot increases 9.532 mm and turn checks fail.

R improves RMS at +0.30/+0.90, but worsens −1.00 RMS 51.080→76.789 mm,
peak 116.256→306.794 mm and relative lean 7.399→13.196°. Only 55 held samples
are modified there; the speed ceiling never binds. Moving actual-minus-used
joint-rate vector RMS worsens 3.458→5.116 rad/s, so the regression is real
motion/balance behavior, not an easier-target scoring artifact.

NR at +0.90 improves final error 190.191→9.527 mm and passes original settling
and route gates. It still worsens four turn overshoots; turn-two overshoot
increases 23.463 mm. NR also settles at +0.30 but fails original route gates;
at −1.00 it is worse than N alone. Joint-reference/scheduling benefits are
configuration-dependent and not additive. All nine full candidate-per-phase
screens fail; no variant qualifies across three starts. Keep every signed and
absolute lag value, all 72 events, and both native/used reference errors.

**Checks/decision.** Both known +0.90 B/R cases reproduce all 47 I-51 common
precision fields exactly. Independent raw/source/phase/guard/held-gain checks
and independent metrics/screens match. Assertion totals are evidence QA, not
additional simulations or full repository tests. Keep runtime/config/defaults
unchanged. N is the lead experimental scheduling candidate; the fixed reference
slew guard is not safe to enable universally. Preserve NR +0.90 as a useful
recovery demonstration, not a general fix. Next investigate turn-specific
reference/feedback consistency and full contact geometry; no new weight scan,
time gate, controller or extra trajectory cases were added after outcomes.

### I-53 — teammate publication and visible acceleration-burst audit

2026-11-26. **Analyzed and validated; user requests repository push.**

The user questions the visual improvement and identifies the −1.00 R burst
near playback 10 s. Exact clock mapping is recorded 32 s. Saved-data analysis
of all four arms over 28–36 s confirms R reaches 1.025 m/s at 32.305 s. N
peaks at 0.263 m/s and has lower 50 ms acceleration RMS, but retains transients.

At 32.250 s, evaluated and inferred belt-B → shell-B transfer changes shell-B
request −0.092→−7.719 N m, clipped at −0.600. LQR gain-change contribution
−7.602 N m dominates error-change −0.011 N m; prior lean error is 13.17°.
Reference bounds hold, and the current guard modification actually softens
demand by +0.098 N m on the frozen state. This identifies an abrupt scheduled
feedback request acting on a bad entering state with saturation, not unique
physical causation or an actual acceleration-bound violation. No new Gazebo
campaign/control change; preserve the R regression and N's remaining twitches.

Prepare a teammate guide, study-PDF links, selected overview/slow-event media,
exact event subset, complete existing code/tests/iteration evidence and a
~65 MiB hash-bound native-run workspace overlay. Restoration refuses changed
files and unsafe targets; fresh-campaign preparation starts no physics and
does not claim the old retry history. Full original raw datasets/large ZIPs
remain local, with scope explicit. Unrelated firmware/ESP32 work stays outside
publication.

Existing suite: 579 passed / 6 skipped. Final new workspace suite: 21 passed
separately, not 17+21. Preserve earlier adversarial test failures/source versions,
source/data hashes and zero-write verification of 118 original files. Restore
test-only 0.36 formatting churn in the initially clean generated report.
Capture 168 older origins and 21 older archives. Controller defaults remain
unchanged. [Handoff and evidence](docs/iterations/2026-11-26/teammate-handoff/README.md).

### I-54 — acceleration, entering reference motion and sustained platform spin

2026-11-26. **Saved-data analysis completed; no live controller/default change.**

The user asks for causes before another fix and identifies spinning after
I-52's −1.00 R burst. Rotation is actual platform heading: world-vertical
angular velocity is already 3.739 rad/s before the 32.250 s support change,
peaks at 6.635 rad/s at 32.290 s, and the platform makes 2.748 further turns
through 44 s. Its rate remains 1.161 rad/s at 42 s with zero XY reference.
Frame/projection validity, relative joint spin and current/prior holds are
separated. Other arms have different entering histories, not matched states.

Exact cached-gain reconstruction matches all loaded rows in all four arms.
The shell-B request jump splits into −7.429 N m from support-share change,
−0.172 from phase change and −0.011 from changing error. Both solver endpoint
and estimated support change together. A frozen old/new gain blend cannot
meet all five existing command caps at this state; that is algebra, not a
tested controller. Earlier guard changes first alter B/R commands on the
same state at 10.930 s; the 55 modified holds and 13 clusters are retained.

The entering relative-error rise substantially includes desired-attitude
motion. Over 31.400–31.745 s, desired up moves 15.621° while actual absolute
tilt decreases 6.042→2.709°; relative error grows 1.715→9.848°. Local guard
activation follows at 31.745 s. Normal-derived target motion, zero angular
velocity reference, belt preparation/cap and flat-bank interpolation are
separate candidate contributors; the initial growth cannot be attributed
solely to that local clamp. At the later 13.166° relative error, absolute tilt
is 6.171°. This qualifies any impression that the platform fell 13° from vertical.

Using saved state and verified mirror/SDF inertias, 32.250–32.275 s changes
platform/carrier vertical angular momentum +0.022915 kg m²/s versus shells
−0.021530; total changes +0.001385. Near-vertical shell axes and known motor
reaction support a substantial exchange contribution. Net-small total
momentum change does not uniquely separate opposing individual contact/joint
impulses. Dedicated heading commands are zero; native angular-rate feedback
remains active and later reverses torque. The episode is driven/oscillatory,
with slower continuing rotation afterward, not an undamped constant-rate coast.

Preserve registered saved-array/kinematic calculations, compact reports,
plots and checks. Mechanics stage01's snapshot-axis alias is retained and
corrected separately; computed momentum values were unaffected. Whole-route
causal attribution remains open because 1 ms impulse histories and matched
interventions are absent. Propose an identical-prefix, short-window diagnostic
before a fix. No new trajectory or full repo-test run; evidence QA is not
additional robot trials. [Record and evidence](docs/iterations/2026-11-26/acceleration-spin-cause/README.md).

### I-55 — identical-prefix motor and attitude-reference interventions

2026-11-26. **Four short native diagnostics completed; no default adopted.**

User authorizes the proposed causal investigation. Register native batch,
native 1 ms-read, zero-only-free-shell-A torque [32.250,32.275), and held-only
attitude target [31.580,31.745), all with original R settings/path/limits and
32.500 s diagnostic horizon. Preserve the original 44 s job metadata separately
from the shortened actual execution. No branch restores hidden solver history;
each replays the full original initialization and prefix in a fresh process.

Both native cases reproduce all 56 original precision fields at all 6,501
samples exactly; their 14 sidecar fields and selected numerical histories also
match. The 1 ms recorder preserves five-step command holds, captures raw states
before/after and all contacts, and executes no extra controller/observer call.
Original runtime/config/plugin/gains remain unchanged. The two intervention
prefixes and direct hook scopes pass their registered gates.

At 32.250 s both motor-test states have world yaw rate 3.825019 rad/s.
After the first hold it is 4.315864 native versus 4.045030 with only A zeroed:
the yaw-rate increase falls 55.2%. The other four held inputs are identical
during that first hold, and A carries zero exported force throughout all
25 affected 1 ms steps. The paired first-hold momentum changes are almost
entirely opposite A/base exchange. By 32.275 rates are 5.992068 versus
4.829581 rad/s, including later normal feedback. After release rates nearly
reconverge and planar speed at 32.305 is slightly worse (1.039 vs1.025 m/s).
This confirms a local actuator contribution, not a useful disabling policy.

At the 31.745 s attitude endpoint before release acts, yaw rate decreases
3.132654→1.872042 rad/s, but absolute tilt rises 2.709→4.899° and error to
the same untreated desired up worsens 9.848→15.932°. Error to the easier held
target is only 0.495°. Current branch-native target error is 10.624°; keep
all three targets distinct. Native reference release generates another
transient, retained separately. Moving balance reference affects real motion,
but freezing it sacrifices required tracking and is not adopted.

Versioned DART/physics source binds force=impulse/dt and current-step
publication; there is no supported one-step force shift. Actual solver
geometry precedes position integration, so origin sensitivities remain.
Native 25 ms complete-system yaw closure passes at 5.34% residual; native
55 ms fails at 11.80%. Retain the failed budget and linear residuals without
sign/time/threshold adjustments. Causal actuator evidence remains valid;
the complete physical force accounting is not uniquely closed.

A post-primary descriptive contact follow-up confirms a physical geometric
swing: over 69 endpoints from 31.405–31.745 s, actual force-weighted radial/
equivalent-wrench directions change 17.826/17.825° versus 16.031° estimated.
Interval equivalent-normal differences are 2.137° RMS/5.070° max; this is not
proof of an exact estimator-delay cause. Two final 1 ms samples have zero
force but all full 5 ms intervals carry load. The first analysis's always-
loaded assertion failure is preserved; its successor retains missing normals
explicitly. Canonical causal endpoints/gates are unchanged. This supports
prioritizing feasible control through genuine contact changes while retaining
estimation limits.

Validation includes 14 pure fake-recorder contracts, nine separate pure math
fixtures, and 53 independent saved-data assertions. These are separate QA
scopes, not extra simulator trials or a full repository-suite run. Retain the
initial floating-boundary fixture failure/correction and the independent
review's first incomplete input declaration, followed by corrected registration
and unchanged numerical results. Earlier repeated check counts are not additive.
Preserve compact source/protocol/report/plots plus exact 1 ms data and exact
5 ms event subsets. Next target is feasible continuous balance/drive handoff
with momentum and total-command limits, not blanket torque removal/reference
holding. [Record and evidence](docs/iterations/2026-11-26/causal-replay/README.md).

## I-56 — bounded total-command handoff / 2026-11-26 / run, rejected

Question: after I-55 confirmed an unloaded-shell yaw contribution and the cost
of freezing balance targets, can a continuous feasible command handoff reduce
rotation without compromising the native LQR's balance and trajectory response?
Implement two new opt-in modules plus a result-owned adapter; original runtime,
observer, reference generator, preparation, gain bank and defaults are unchanged.
Use current simulated state and observer-derived contacts only. A private soft
contact model supplies a 20 ms differential input-response matrix. Stage one
minimizes transverse body angular-response discrepancy relative to the clipped
native command within total amplitude and actual previous input slew boxes;
stage two preserves that attained response while trading travel, yaw, unloaded
relative shell momentum, readiness/native proximity, change and effort.
Command slew assumptions are [60, 60, 60 N m/s, 6, 6 m/s²]; these do not bound
actual robot acceleration. The response equality is local, not a stability or
friction-feasibility proof, and no future contact onset is predicted.

Freeze source/protocol/settings before outcomes at base cbaf7d4. Compare against
native B, not rejected R: same 44 s Mars figure-eight, shaft −1 rad, yaw 45°,
caps [1.5,.6,.6 N m,.3,.3 m/s]. Three fresh serial jobs B, A (yaw/momentum
weights zero), H (full) all complete. B reproduces 56 historical precision
fields exactly. Registered yaw benefit and full-route nonregression gates
control whether +.3/+.9 pairs run; no automatic promotion or retuning.

The full candidate fails: peak XY error 116.26→136.16 mm, final 42.65→76.72 mm,
absolute tilt 13.72→18.96°, raw-reference lean 7.40→13.85°. Moving RMS changes
51.08→55.14 mm. The ablation's final error improves to 15.47 mm, but peak error
rises to 183.97 mm and raw-reference lean 13.43°. Neither favorable endpoint nor
bounded commands supports adoption. Stop conditional broader experiments and
retain all three histories. Canonical detailed metrics/media are in the record.

Preserve the initial six-component surface-velocity assignment setup error and
its translational-slice correction. One same-state I-55 unloaded-shell torque
sensitivity matches first-5-ms yaw response within 0.377%; loaded-contact response
is not validated by that single check. A new readiness fixture fails under the
original excessive soft change cost (belt reaches 0.20598 m/s in 120 ms toward 0.30).
Normalize the soft cost by input bounds while retaining the hard slew box; the
corrected fixture reaches ≥0.297. Preserve both exact source versions before
physics. Final focused tests are 21 model + 20 allocator = 41 unique; nine separate
read-only preflight contracts, not a full repository suite. Older synthetic
review counts/subsets/replays are nonadditive and apply to superseded source.

Decision: rejected for default use. Test local-model fidelity and quantify
the balance-response loss imposed by rate bounds before another controller
variant; do not blame these regressions on estimator delay without evidence.
[Record, evidence and reproduction](docs/iterations/2026-11-26/constrained-handoff/README.md).

## I-57 — loaded-contact response validation / 2026-11-26 / analyzed, retained

Question: does the frozen I-56 response model predict matched-state physical
input effects, or did its loaded-contact approximation mislead allocation?
Keep runtime/model/controller defaults unchanged at source revision 75c9549.
Select prior H states 4.855 s (early shell B) and 35.925 s (late belt B) before
new perturbations. Reconstruct the original observer-derived patch records and
both 5/20 ms response matrices exactly; seal all predictions before physics.

Register 30 fresh serial Gazebo processes, 15 per state: saved continuation;
held native, governed, midpoint and duplicate governed; five governed-centered
input columns at full/half feasible steps. Every process replays all original
applied inputs from initialization and byte-checks every 5 ms prefix state,
clock and actual previous command. No saved-pose branch reset or live observer/
controller update. Native/midpoint are explicitly amplitude-only diagnostic
counterfactuals, possibly outside the candidate's optional first-step slew box.
Keep original run(5) holds and log 0/5/10/15/20 ms endpoints/contacts. Compare
angular vectors in world/common initial frame, not different moving body frames.

Both continuation pilots pass exactly, including all four post-anchor states.
The early state's 15 branches complete and duplicate exactly. Every resolved
loaded response passes its coarse registered screen; observed native-to-governed
20 ms planar change [-.002860729,+.000430672] m/s matches modeled
[-.002823642,+.000421307] to 1.322% vector error. Removing the contact penalty
predicts the opposite planar direction (236.7% error). Yaw error is 0.091%;
transverse change .000203 rad/s is below the .002 resolution floor, compatible
with a predicted nullspace but not a directional-accuracy claim. Half/full
column responses are nearly linear; zero unloaded-belt responses remain
unresolved, not positive directional tests. No endpoint support-mode changes
occur in this early set. These local results do not identify the whole-route
drift cause. All 30 branches ultimately complete; every prefix, continuation
and duplicate control is exact. Late native-to-governed 20 ms errors are
1.351% planar, 1.204% transverse and 1.988% world yaw; the large transverse
change is real: predicted 3.741 versus observed 3.783 rad/s. Prior shaft torque
+1.197503 N m and the optional 60 N m/s rate bound force +0.897503, while the
clipped native request is -1.5. The first feasible command remains opposite
the native request; a complete reversal needs at least 44.96 ms at that rate.
This confirms a local physical cost of the assumed rate bound, not the cause
of the entire trajectory or proof of a new controller fix.

Final loaded scores: 118 pass, 165 unresolved, five fail. The five are small
late 20 ms couplings: shell-A/B torque planar full probes have 73.26% error
on ~0.55 mm/s observed effects; half probes cross the 0.2 mm/s resolution
floor; belt-B full yaw crosses the 0.002 rad/s floor. Preserve failures without
widening thresholds. All 25 predicted-small transverse checks are quiet;
full/half groups give 67 passes, 77 unresolved and no failed linearity checks.
These correlated scores are not independent experiments. No endpoint support
change is observed; this does not validate contact transfers or multicontact.

Separate post-primary exploratory analysis identifies the yaw-cost baseline
assumption. At the late held-command 20 ms endpoint, current+Gdelta predicts
-0.00584 rad/s versus actual governed +0.76183; omitted native future change
is +0.79683, paired response error only -0.02917. A frozen affine forecast
gives +0.73614 there. A further one-step audit reconstructs all 8,800 original
H intervals: affine RMS improves in every group, including planar
0.019260→0.008616 m/s and world yaw 0.096866→0.008064 rad/s, but large tails
and occasional worse component errors remain. Exact current state and actual
applied input are supplied every step; this is not a multistep forecast or
cost/controller validation. No model fitting, gate changes or new physics in
these exploratory analyses.

Preserve eight pure replay contracts and eight separate arithmetic checks plus
a fabricated end-to-end schema fixture; these are not a full repo suite or
extra physical trials. The first synthetic figure's clipped annotation is
retained; the later freeze changes only that label, with numerical rules fixed.
Raw 5 ms contact endpoints cannot certify within-hold contact continuity. Coarse
thresholds are engineering assumptions, not calibrated model confidence.
Portable standalone NumPy replay reproduces every loaded/free score and
linearity metric plus direct endpoint controls; full pre-anchor parity remains
separate recorded certification. Independent formulas agree with final results.
No runtime/default changes. Decision: preserve native LQR; next isolate balance
priority above optional smoothing while retaining amplitude caps, with auxiliary
yaw/momentum disabled first. Investigate the cost's velocity baseline separately.
Neither proposed controller change is executed in I-57.
[Experiment record](docs/iterations/2026-11-26/loaded-contact-validation/README.md).

## I-58 — balance priority before optional smoothing / 2026-11-26 / analyzed, experimental

Question: does removing I-56 A's hard command-rate feasibility box, while
retaining its exact soft objective and zero yaw/momentum weights, restore native
balance response without a whole-route regression? Add only an opt-in allocator
module/tests plus result-owned harness/analysis; existing runtime files, observer,
model, raw references, gain bank and defaults remain unchanged at base d19cacb.
P retains hard total amplitude caps [1.5, .6, .6 N m, .3, .3 m/s] and exact clipped-
native G20 body-x/y response; previous command enters soft smoothing. Old 60/6
rates are diagnostic only. Row-normalized equality and verified native fallback
are numerical protections; the equality is not a stability/friction guarantee.

Freeze 7dfe9c9 protocol/68 sources before new physics. Initial fresh serial B/A/P
are same 44 s Mars figure-eight, shaft -1 rad, yaw 45°. B/A must reproduce all shared historical
precision fields byte-for-byte. Primary benefit is total first+secondary modeled
priority restoration, max≤1e-8 and held RMS ≤10% old A with nontrivial old A loss.
Retain native B tracking/attitude/turn/settling nonregression, 5% late yaw/heading
tolerances and added full speed/50 ms 3D acceleration guards. Continuity RMS/p99/max
is fixed secondary information, not an alternative benefit chosen after results.
Only all-gate lead success permits four +.3/+.9 B/P runs; no default promotion.

Pre-execution verification: 31 new allocator tests; 72 combined passes include the
same 41 existing tests, not additional counts. Sixteen separate runner/adapter
contracts and eight saved-metric contracts pass. Initial ROS pytest-plugin
collection failure is retained, then autoload disabled. An unexecuted metric
draft's missing-short-window review issue is corrected before freezing. Thirty
independent synthetic comparisons agree with old A when its rate box is inactive.
Static same-state I-57 regression restores late shaft -1.5 versus A +.897503 and
modeled loss 3.7405 rad/s→numerical zero; this is not a new physical paired trial.

All seven fresh jobs complete 44 s. Lead B/A reproduce all 59 shared historical
precision fields byte-for-byte; additional B controls reproduce all 56 shared
fields. Initial P passes all 26 gates, then the four conditional runs execute
unchanged. Lead moving RMS improves 51.08→33.43 mm, peak 116.26→72.16 mm,
final 42.65→6.29 mm and late yaw peak 2.443→1.349 rad/s. However its 50 ms
3D acceleration peak increases 8.078→8.955 m/s², narrowly inside the fixed
8.985 allowance. It still rotates at the goal and fails actual settling.

Broader results fail the full screen. At +.30, moving RMS 56.96→54.37 mm
improves slightly but final error 20.82→52.62 mm and turn-six overshoot
+12.903 mm fail. At +.90, RMS 50.49→48.37 mm and final 190.19→63.85 mm
improve, but yaw peak 1.835→3.084 rad/s, speed peak .319→.502 m/s, and turn
three/four overshoot +65.644/+26.167 mm fail. All P amplitude/equality gates
pass, max priority discrepancy≤9.991e-16, zero fallback and reduced command-
step RMS at every start. None of the seven runs passes actual final settling.
Numerical priority restoration and favorable average tracking do not establish
robust closed-loop improvement. Keep native LQR selected; no retuning/adoption.

A separately registered post-primary goal-hold audit reads fixed 41–44 s for all
six B/P runs, without new physics. +.30 P moves outward after 41.54 s then nearly
stalls 52.6 mm away; +.90 P still moves away at 44 s (mean final-half-second radial
velocity -20.09 mm/s), despite its smaller endpoint error than B. Last-half-second
modeled allocator radial adjustments are tiny/below prior model resolution;
no native clipping/rate/label issue alone explains that interval. +.30 support
disagreement begins after the outward trend. This does not establish a unique
cause or clear earlier state-history effects. Next diagnostic target is native
goal-hold pose/velocity/drive references at actual P states and turn response.

Frozen-source pure NumPy replay and independent formulas reproduce every final
metric/gate; full-rate projections, both initial/final reports, fixed videos for
all three starts and scientific plots are retained. Completion and initial
extension eligibility remain distinct from final all-phase acceptance (false).
[Record](docs/iterations/2026-11-26/balance-priority/README.md).

## I-59 / 2026-11-26 / analyzed — retained startup-gain audit

Question: do phase-specific ordinary LQR settings retained at startup materially
explain why the three initial phases produce different trajectories? Source
inspection alone only establishes a possible path: current terrain feedback
overwrites its shaft row with the ordinary local design's row.

Freeze protocol, sources and 30 input-file hashes before numerical outcomes.
Use all six I-58 B/P traces, three starts -1/+0.3/+0.9 rad, 8,801 saved states
each: saved Gazebo/DART Mars figure-eight, yaw 45°, caps [1.5,.6,.6 N m,.3,.3 m/s],
source base 4ce407c with exact input/source hashes. Zero new Gazebo physics runs.
At identical stored state, observer contacts,
transformed error, references and history, swap only shaft row zero. Keep
scheduled rows 1–4, feedforward, preparation targets/weights, heading and fade
fixed. Trim-only and row+trim are separate secondary diagnostics. Propagate
changes through dependent preparation/fade stages, then original native clipping.
For P recordings this is a native-policy shadow, not a reoptimized final P input.
Full/moving/goal and three event windows, 1e-10 numerical closure, absolute
0.001 N m/0.0001 m/s and cap-normalized .001 resolutions are preregistered.

All rows/trims are constant within traces and same-phase B/P coefficients are
byte-exact. Across starts, largest shaft-row coefficient difference is
5.6831e-8 and equilibrium trim difference 5.0397e-13 N m. Maximum row-only
unfaded shaft-command effect across every state/anchor is 2.52744e-8 N m;
shaft synchronization is zero, fade ≤1 and clipping cannot enlarge this bound.
Full native reconstruction confirms only negligible effects under both fixed
resolutions; maximum native-clipped row-only effect is 1.93331e-9 N m and all
material counts are zero. Independent arithmetic agrees; zero intervention reproduces native
request/applied bytes. No trajectory benefit or stability claim follows.

This is a useful null result and corrects the earlier practical attribution:
the startup design selection exists in source, but its retained shaft row/trim
does not materially change commands at these recorded states. Do not change it
as the next fix on this evidence. Initial physical configuration, current
support scheduling and evolving state/reference history remain different;
the first divergence among those paths is the next proposed target. Native
LQR/observer/feedforward/preparation remain selected. Earlier mixed I-58 results
and all-phase rejection remain unchanged. Source-filename/Python-path setup
corrections and a source-line citation erratum are retained without runtime edits.

[Record, scientific plots and portable replay](docs/iterations/2026-11-26/startup-anchor-audit/README.md).

## I-60 / 2026-11-27 / analyzed — measured-velocity contact ranking, shadow only

Question: can the user's measured-velocity idea improve contact inference beyond
the momentum-observer/geometry locator? Add opt-in contact_hypotheses.py and
meaningful tests. Existing LQR, observer, feedforward, preview, references, gains,
limits and defaults remain unchanged. Source base 24f722e; zero new Gazebo
physics and no compensating commands.

Freeze protocol, old/new source and historical inputs before arithmetic, with
independent blind review. Use all 26,403 I-58 native B states: Gazebo/DART Mars
figure-eight, 44 s, shaft -1/+0.3/+0.9 rad, yaw 45°, caps
[1.5,.6,.6 N m,.3,.3 m/s]. Inputs are exact simulated q/qdot, prior held ideal
belt speeds, observer wrench and robot geometry. The observer intrinsic normal
bounds a 10° candidate cone; no desired/new command, terrain truth or future
state enters ranking. This is observer-conditioned inference, not an independent
replacement for the observer.

Seventeen normal seeds and six patch/rim kinds form 102 candidate slots. Valid
points satisfy geometry, cone and 1 N compression. Four family scores compare
geometry (G), geometry+wrench (WG), geometry+velocity (VG) and all terms (F).
Scales are 10°, .02 m/s and R*norm(force). Family minima use complete candidates.
Acceptance requires >=2 families, motion availability >=.02 m/s, margin >=.25,
residual <=.02 m/s and residual contrast >=.01 m/s. No smoothing or calibrated
probability. All three feature/rank outputs are sealed before truth is read.
28 selected core tests pass (22 new + six existing); seven separate evaluator
contracts pass. Earlier 19/27 core runs overlap and are not additive.

Primary [1,41) has 23,423 dominant-support eligible states: >=1 N total and
>=90% one family, not strict single physical contact. True-family candidate
coverage is 100%. Native/G/WG errors are 412 (1.759%); VG/F errors are 109
(.465%), 73.5% fewer. Per-start errors fall 51→11, 226→51 and 135→47; false
switches 17→13, 121→27 and 65→37. All 12 registered shadow gates pass. F accepts
6,902 states with 18 errors (.261%), versus 306 native errors on those same
states. Coverage is 86.60% of 7,970 ambiguous motion-eligible states, but 29.47%
of all primary eligible states. Stable ±10 ms truth subset improves 75→3 errors
in 21,983 states; last-1-ms versus filtered-5-ms clock qualifications remain.

Independent rank/evaluation/kinematic checks agree. VG/F raw labels are identical
at all 26,403 states, so the added moment score has no demonstrated label benefit.
Three primary accepted decisions differ; F accepts one additional error. Fixed
event inspection retains adverse cases: at -1 rad, 35.205 s, true/native support
is belt B, but a small velocity residual strongly favors wrong shell B. This is
model consistency, not proof of slip or a unique inverse.

A descriptive post-primary point check is registered before its arithmetic,
without changing scores/gates. Paired point RMS improves 3.134→2.672 mm, while
maximum discrepancy worsens 26.91→44.04 mm. Accepted maximum grows
23.76→29.84 mm; every phase's maximum worsens. Correct-family subset improves
both RMS and maximum, so enlarged extremes come from remaining wrong-family
choices. Radial-R proxies and dominant-patch means yield similar conclusions.
Independent source review corrects a terminology overstatement: logged counts
include only force >.05 N records. One resolved record does not prove one physical
point; weaker records still contribute to means. Erratum and original README
are preserved; frozen statistics/gates are unchanged. A plot-legend overlap is
corrected separately without changing data.

Decision: retain the useful shadow family-inference signal for further controlled
tests; no live adoption or tracking claim. Before compensation, investigate
bounded geometry changes and slip/ambiguous hypotheses. Existing drift, turning
and saturation issues remain open. Compact full-rate rank/evaluation/point
projections replay with NumPy; full banks/raw state/model assets remain local
and hash-bound.
[Record, plots and reproduction](docs/iterations/2026-11-27/velocity-contact-ranking/README.md).

## I-61 / 2026-11-27 / analyzed, rejected — joint-load consistency veto

Question: do the extra observer joint-load channels reject harmful I-60
overrides without losing useful velocity-based corrections? Source base 92fe995.
Keep all I-60 candidates, scores, accepted labels/points, native controller,
observer, references, gains, caps and defaults fixed. Add opt-in
contact_load_consistency.py and tests. No new Gazebo physics or live commands.

Register protocol, old/new source and 30 historical inputs before residual
arithmetic. Use three I-58 native B Mars figure-eight histories, all 26,403
states, 44 s, shaft -1/+0.3/+0.9 rad, yaw 45°, caps
[1.5,.6,.6 N m,.3,.3 m/s]. Predict J_point.T*F_observed at each frozen proposal;
rotate base angular rows into world, retain scalar joint rows. Primary veto is
all extra-joint mismatches <=.03 N m +.10*R*norm(F), R=.12 m. No force fit,
native-relative gate, rescoring, point cap or temporal smoothing. Force rows
repeat the supplied force, and shaft is largely redundant with net moment;
the main added identity cues are the two shell-joint rows.

All three feature/policy outputs are sealed before truth arrays. Compare native
N; I-60 accepted proposal else native A; and accepted+load-pass proposal else
native C. SAME-family point changes are included. Primary [1,41) has 23,423
dominant-support eligible states, >=1 N and >=90% one family, not strict single
contact. N/A/C errors are 412/124/153; per phase 51/14/16, 226/64/84 and
135/46/53. A's 124 is the guarded comparator, not raw I-60 F's 109.

The veto removes 46 of 6,902 proposals: 30 useful corrections lost, one harmful
override prevented, 15 label-neutral point fallbacks. Harmful overrides of
native-correct labels fall 6→5 (16.7% versus required 50%); useful retention is
264/294=89.796% (required 90%). Point RMS N/A/C is 3.134/2.757/2.807 mm;
maximum 26.91/29.84/29.84 mm. The old raw 44.04 mm I-60 outlier was already
rejected by I-60 acceptance and is not a benefit of this load veto.

Five of 13 fixed gates fail: harmful removal, useful retention, pooled C-vs-A,
+.3-rad C-vs-A and point maximum versus N. Conditional physical trial does not
run. Stable oracle-GEOMETRY compatibility passes 99.870/99.881/99.352%; this
uses observed total force and raw dominant-family mean point, not true full
generalized forces. Post-primary diagnosis finds ALL 30 lost useful corrections
outside ±10 ms stable truth, and all 30 also fail this oracle compatibility at
those frames. Shaft contributes 40/46 vetoes, shell A/B six each (overlap).
This points toward current-geometry/history/force-distribution limitations in
transfer, not proof that every observer channel or candidate point is wrong.

The -1-rad 35.205 s wrong shell-B proposal still passes (residuals about
[.173,-.000002,-.105] N m; tolerance.251). Correct shell-A at 7.875 s is vetoed
([.212,-.301,.272]; tolerance.266). Current fixed residual magnitude is not
selective enough; no post-result threshold tuning is performed. Native control
stays selected; I-60 remains a promising shadow signal, not a deployed estimator.

45 selected core tests pass (17 new + same 28 existing), with nine separate
evaluator contracts and seven replay-adapter contracts. Independent analytic
loads, all gate/policy flags and point metrics agree. Archive preserves an
unexecuted schema correction, fixture-alias failure/correction, and a final
reviewer receipt whose pre-truth wording was clarified (written 16 s after global
truth phase began, reviewer still outcome-blind; sources/check scripts already
frozen). Producer NumPy 1.26.4 gives whole-report exact replay; NumPy 1.21.5 gives
all arrays/window metrics/gates exact with five forensic norm scalars differing
<=3.56 e-15 mm. A validation-only whitelist reports those differences without
changing values or scientific gates; initial strict failure is retained.

Decision: reject this instantaneous hard veto; no integration or new physical
comparison. A time-consistent load/transfer model is the next proposed
diagnostic, rather than hiding failure with tuning. Compact full-rate replay,
plots and source/provenance are retained; large original arrays stay local.
[Record and reproduction](docs/iterations/2026-11-27/contact-load-consistency/README.md).

## I-62 — bounded tuning of the joint-load veto (2026-11-27)

Status: **implemented, replayed, analyzed, rejected for adoption**. The user
asked whether I-61's tolerance numbers could be tuned. Freeze I-60 proposals
and I-61 residuals; sweep shared a,b = 0,.005,...,.300 (3,721 pairs), with
explicit off/A control and original (.03,.10) benchmark. Direct comparison
defines zero tolerance without changing the runtime helper. No live controller
change or new physics. Use the same I-58 native Gazebo/DART Mars figure-eight
starts -1/+.3/+.9 rad, yaw 45°, caps [1.5,.6,.6 N m,.3,.3 m/s], 5 ms logs.

Register gates, source/input hashes and whole-start folds before arithmetic.
Primary [1,41), dominant support >=1 N and >=90% retains 23,423 states, all
with matched finite N/A/truth points. Before execution clarify the point mask
to actual A fallback rather than invalid declined raw proposals. Fit on two
starts, evaluate the third; qualification requires >=50% harmful removal,
>=90% useful retention, pooled errors <=A, per-phase error/harm limits,
RMS/p95 <=A, point maximum <=N, nonempty retention. Select fewest errors,
SSE, vetoes, then integer a,b; no qualified fit means failed selection and
off/A. All three episodes were examined previously: retrospective internal
assessment, no unseen-validation claim.

Every fold and the full-data fit has 0/3,721 qualified settings. Out-of-fold
fallback reproduces A's 124 errors; fitting fails. Best unrestricted pooled
witness (0,.15) has 123 errors, five harmful changes and 294 useful corrections,
RMS 2.752 mm, maximum 29.843 mm. This one-sample gain fails harm/point gates.
Best with >=50% harmful removal (0,.06) has 232 errors, three harmful changes,
183 useful corrections (62.24%). Best with point maximum <=N (.005,.075) has
221 errors, four harmful changes, 195 useful corrections. No grid pair meets
even harmful/useful gates together. N: 412 errors, maximum 26.908 mm;
A: 124/29.843 mm; old I-61 C: 153/29.843 mm.

An independent original-input grid evaluation checks all settings and folds.
A separate dominance audit shows rejecting the 29.843 mm m100 35.205 s point
forces at least 99 useful losses for any nonnegative shared affine tolerance
on these fixed inputs. Only six harmful changes exist, implying at least
217 errors; grid minimum 221 is tighter and separate. A supplementary 99-witness
exact-sign certificate confirms margins. It is explicitly post-result: first
execution preceded its own receipt, then identical source was sealed/replayed.
No broader estimator impossibility or unique physical cause is claimed.

Initial stage01 stopped at the I-61 benchmark: policies, families, distances
and counts exact, but batched/scalar SSE differed <=2.26e-10 mm² (RMS
<=5.33e-15 mm) due to NumPy indexing memory layout. Make candidate rows
contiguous, retain guard/gates/ranges/selection, seal corrected source before
canonical stage02. Benchmark differences are now zero; old C's 16/84/53 errors
are exact. Eleven synthetic contracts pass inclusive, including an 8,003-point
chunk-invariance check. Preserve the abandoned unexecuted guard-relaxation
draft and ROS plugin missing-lark collection failure; disable pytest autoload
for the final check. Independent arithmetic, portable replay and archive
receipts have separate scopes; no controller/full-suite test count is claimed.

Decision: keep native LQR, momentum observer, rolling/holding feedforward and
120 ms preparation selected; retain I-60 velocity ranking offline. Threshold
tuning alone cannot meet the demonstrated point/label tradeoff. Investigate
time-consistent transfer/load hypotheses before another hard veto. Preserve
failed attempts and descriptive near-misses for the presentation.
[Record, figure, protocol and reproduction](docs/iterations/2026-11-27/contact-load-tuning/README.md).

## I-63 — observable conditions for a dynamic load tolerance (2026-11-27)

Status: **implemented, replayed, analyzed; conditional stages stopped**. The
user proposed changing the tolerance with observable conditions. Keep the
native LQR, momentum observer, rolling/holding feedforward and 120 ms incoming
preparation unchanged. Use the three original I-58 Gazebo/DART Mars figure-eight
recordings, shaft -1/+.3/+.9 rad, yaw 45°, same command caps and 5 ms logs.
All episodes were previously inspected; assessment is retrospective.

Freeze one candidate-independent risk before arithmetic: u is the largest
observer interval-versus-filtered extra-joint change normalized by the old
.03 + .10 R norm(F) scale; s measures known shell/belt seam surface proximity
and adjacent backward rate using 12 mm/120 ms engineering scales. Combined
r=max(u,s), not a probability. No candidate/support identity, true contacts,
terrain, true slip or future samples enter inference. Before execution replace
axial distance with spherical arclength to avoid flagging the entire 4.35 mm
axial-depth cap as near a 12 mm seam window; retain the unexecuted design.

Root seals 334 input/source/model-asset files and eight scientific sources.
Generate all three 8,801-row feature artifacts privately without physics or
contact queries, then seal before truth evaluation. Primary [1,41) retains
23,423 dominant-support states and 6,902 A proposals; all have known risk.
Check exact dominance for tolerance a+bR norm(F)+c*r, all coefficients
nonnegative, before any fitting. Unknown risk would retain A for positive c;
c=0 preserves the old gate. The original 99 useful witnesses match case/time.

All six harmful overrides have r=.892–.959. Worst point m100 35.205 s has
u=.51934, s=r=.93760, so it also looks transition-risky. Useful-correction
median r=.97080 looks promising in aggregate, but the necessary 99-frame
subset has median .62673. Combined-risk dominance still forces all 99 useful
losses, versus six harmful opportunities: errors >=124+99-6=217 if point
maximum must return to N's 26.908 mm from A's 29.843 mm. The new risk
inequality minimum margin is .01574 N m; all 99 remain robust to the entire
registered +/-1e-4 numerical-risk envelope. The observer-only descriptive
bound forces 59 losses; it does not replace the primary rule.

The registered stop triggers. No 18-schedule fit, leave-one-start-out schedule
selection, new +.6-rad episode or live adoption runs. This excludes the fixed
combined-score nonnegative affine rule, not all dynamic thresholds or the
positive I-60 velocity idea. It does not prove a unique physical cause of
trajectory deviation. Next proposed diagnostic: matching candidate load
histories and observer filtering/timing rather than another instantaneous veto.

Twelve feature contracts and fifteen separate evaluator contracts pass;
neither is a new full controller suite. Independent loops/geometry/Fraction
audit matches counts, witness identities and decision; max score difference
2.71e-13. Producer replay and archive-isolated NumPy 1.21.5 replay against
1.26.4 regenerate 108 feature and 183 analysis arrays byte-exactly, scientific
JSON equal apart from version metadata. Keep original cached axes/5.3 MB
primitives; an unexecuted packing option is not used. Preserve 17 event windows,
protocol, sources, all phase/group statistics and certificates.

Additional I-62 provenance clarification: stage03 pytest redirected stdout
only; its empty log does not contain the ROS plugin error's original stderr.
The correction receipt records the failure; README now states that limitation.
[I-63 record, plots and reproduction](docs/iterations/2026-11-27/contact-condition-diagnostic/README.md).

## I-64 — matching predicted and observed load windows (2026-11-27)

Status: **implemented, replayed, analyzed; model improvement retained as a
diagnostic, contact veto rejected**. The user approved testing whether timing
consistency improves the contact-load check. Use the same three native I-58
Gazebo/DART Mars figure-eight logs, shaft -1/+.3/+.9 rad, yaw 45°, unchanged
caps/model. Native LQR, momentum observer, rolling/holding feedforward and
120 ms preparation stay selected. No new physics or threshold optimization.

Before numerical work seal 337 original input/model/source files, protocol
and eight scientific sources. Extract endpoint joint-force maps at original
I-61 F, native physical alternative and unprojected native-equivalent points.
Use raw interval force once, not an already filtered force. M0 is original
instantaneous H*filteredF; M1 is raw interval H*rawF; primary M2 uses the same
observer recurrence with one frozen-A predicted history and current branches;
M3 is native-equivalent history sensitivity only. Current candidates never
rewrite the past or feed back into the anchor. Seed observed joint load once,
exclude 200 ms warmup; invalid history/gaps reset. Full9 observer recurrence,
one-step scaled-raw identity and inherited/current decomposition are controls.
Endpoint geometry is an interval approximation, not true force/contact history.

All three 8,801-state inference packets sealed before labels. Primary [1,41)
retains 23,423 dominant-support rows and 6,902 accepted proposals, all scorable;
all 6 harmful/18 accepted-wrong/worst-point events retained. Episodes already
examined: retrospective diagnosis, not unseen validation. Truth and past50ms
family-correct masks are evaluation-only, never inferred history.

Q1 model-consistency PASS on identical 6,884 accepted correct-family frames:
joint-vector RMS .0582828→.0372067 N m (36.2% lower), p95 .135488→.0714135;
all three starting-phase RMS values improve. Shaft channel supplies much of
the gain; shell channels are not uniformly better in every phase. Q2 current
family discrimination FAIL: useful correct preferences 212→243/294, harmful
correct preferences 2→1/6, no ties/unknown/near-ties. At 35.205 s wrong F still
scores below native physical alternative under M0, M1 and M2; raw M1 rejects
F only because both residuals are high. No unique physical cause established.

Fixed unchanged .03+.10 R norm(filtered F) shadow gates: A has 124 errors,
6 harmful changes and 294 useful corrections. M0: 153/5/264; M1: 132/3/283
(maximum point discrepancy 26.908 mm); M2: 128/6/290; M3: 128/6/290.
M2 point maximum remains 29.843 mm. A post-result row-set check confirms all
30 old useful rejections are restored, with four different new rejections:
+0.3 rad at 14.415 s; +0.9 rad at 17.530, 17.770 and 24.955 s. This is a net
26 fewer false rejections, while losing the one old harmful rejection.
All four shadows fail unchanged I-62 gates; no posthoc M1 promotion,
no current-controller changes or physical trajectory gain claimed.

Nineteen feature and fourteen separate analysis contracts pass inclusive of
development stages. Gap reconstruction control corrected before historical
arithmetic; previous source retained. Producer NumPy 1.26.4 reproduces 228
feature and 594 analysis arrays exactly. Independent 77407 recurrence/history
implementation matches masks/counts/preferences/Q1/Q2/shadow metrics; score
differences <=8.89e-16. System 1.21.5 strict byte comparison fails 84 feature
float arrays; preserved local full-pointwise audit measures <=1.11e-16 N m
and <=1.26e-12 auxiliary norm-share difference. Scientific source unchanged.

Separate sealed verification v2 preserves the strict failure, reconstructs
fresh arrays, checks 36 exact per-frame decision arrays plus structures,
integer/bool/nonfinite patterns and scientific summaries within preregistered
units. Archive-isolated system replay passes while reporting nonexact float
bytes and no portable pointwise floating bounds; hashes cannot prove closeness
when different. No reported value is replaced. Compact archive keeps 9.97 MB
cached primitives, bindings, sources, reports and 11 event windows; large
derived tables and independent duplicate arrays remain local with hashes.

Decision: retain matching filter history for model-consistency diagnosis;
do not introduce its load veto into the velocity estimator. Remaining current
contact ambiguity, candidate geometry and within-interval load distribution
need separate investigation. Preserve the partial success for presentation.
[Record, plots and reproduction](docs/iterations/2026-11-27/contact-load-history/README.md).

## I-65 / 2026-11-27 / implemented, run, analyzed; rejected for integration

Question: does a shared empirical bias/covariance score of the full three-joint
residual distinguish frozen contact alternatives better than maximum absolute
error? This is a different calculation from prior scalar-threshold tuning.
Keep the native LQR, momentum observer, feedforward and 120ms preparation live.

Reuse the I-58 Gazebo/DART Mars figure-eight starts -1/+0.3/+0.9rad, yaw45deg,
44s,5ms, caps [1.5,.6,.6Nm,.3,.3m/s]. No new physics or controller change.
Seal13 source/input files before calibration. Same23423 primary states,
6902 accepted proposals, fixed point domain. Fit on two whole episodes, evaluate
the third; all runs previously inspected, so retrospective internal comparison.
Simulator family labels choose training rows only, not held-out scoring inputs.

One physical correct-family branch/time, F-first then native-family Nalt, with
half total weight per training episode. Same normalized residual scale
.03+.10R||filteredF|| for both candidates. Shared pooled mean/population covariance,
fixed .2 isotropic shrinkage + .05^2 identity floor; squared Cholesky distance.
Raw preceding-interval model M1 primary; separate matched-history M2 control,
never promoted post-result. Pattern distance<=.01 and relative score ties1e-9
abstain. Only clearly preferred Nalt vetoes an accepted proposal, falling back
to ORIGINAL native equivalent point, not Nalt physical point. No history feedback.

M1 relative maxabs→covariance: useful correct preferences244→253/294, harmful
1→2/6. Actual method gain relative to this control, but both minimum targets fail.
A124errors/H6/U294 → covariance163errors/H4/U253:41 useful losses to prevent2
harmful changes, net39 more errors. Phase errors14→14,64→101,46→48; +.3 has
no harmful examples (unscorable safety), accounts37/41 useful losses. All6902
are scorable;6602 are ambiguous same-family alternatives, while all300 useful/
harmful opportunities are known/distinguishable. No denominator exclusions.

Point RMS2.75693→2.76395mm, maximum29.84286mm unchanged above native26.90764mm.
M2 covariance also fails,171errors/H4/U245. All unchanged shadow gates checked;
M1 fails harmful/useful/pooled/+.3rate/RMS/max gates. Good7.875s kept, wrongF
at35.205s still preferred (132.65 vsNalt137.81; both above trainingp95=9.56).
No unique physical cause or improved physical tracking established.

Fourteen method and ten separate evaluator contracts pass, not fullrepo suite.
Initial test launch exited127 before Python because relative venv path crossed
one extra parent; preserved and corrected without science changes. Producer
NumPy1.26.4 replay reproduces129 arrays byte-exact; NumPy1.21.5 differs in float
bytes but keeps discrete decisions and bounded results, no value replacement.
Independent verification and publication evidence are archived with this record;
its initial review schema mismatch and corrected stage are preserved separately.

Decision: reject integration; retain velocity evidence and unchanged live baseline.
Next proposed investigation is frozen physical candidate geometry versus endpoint/
interval single-point load-sharing adequacy at these saved events, not another
cutoff search. Covariance cannot recover a missing hypothesis or unmodeled load
sharing; this experiment does not decide which assumption is responsible.
[Report, comparison, events, sources and clone replay](docs/iterations/2026-11-27/contact-covariance-ranking/README.md).

## I-66 / 2026-11-27 / implemented, run, analyzed; diagnostic only

Question: do I-65 load-ranking failures arise from candidate geometry, or does
assigning the interval force to one current family remain inadequate? Reuse
I-58 native Gazebo/DART Mars figure-eight logs: shaft -1/+0.3/+0.9 rad, yaw45°,
44s,5ms, caps [1.5,.6,.6 N m,.3,.3 m/s]. No new physics, controller, observer,
feedforward, preparation, score, threshold or calibration change.

Source/schema audit establishes last-reported1ms normal-load-weighted contact
centroids versus preceding5ms observer load. The intervening per-contact force
history and true generalized loads are absent. The sampled centroid is not an
exact interval oracle, and CPU latency is not measured. Counts above .05N do
not establish strict single contact. Older substep archives describe other runs.

Feature sources/338inputs (306 original model/asset paths) sealed before private
kinematics;24 analysis source/cache/dependency files sealed before residuals.
All26403 poses valid, no dynamics/collision/contact query. Maps match I-64 within
3.47e-17; I-61 centre/angular/load maps exact. Retain raw finite centroid with
separate material flags. Four fixed diagnostic routes: correct physical F-first/
Nalt C; sampled centroid T; full inferred single-family wrench W; normal-share
common-direction force proxy S. Same pose/raw force/target; no true-force fitting.

All41 lost useful frames known. C→T vector RMS .139264→.105682 N m (24.1%lower),
p95 .318014→.258432;37 improve/four worsen. Registered geometric criterion fails
because the sole -1rad focus frame at32.110s worsens .112722→.164060. Both C/T
have39/41 compatible under the old component bound (one pass gained, one lost).
W RMS .068932,41/41 compatible; S=T there. Controls are not deployed estimators.

Frozen I-65 counterfactual on294 useful +six harmful distinct-family cases only:
useful253→276/294 (27 newly correct, four newly wrong); correct harmful rejection
2→4/6. No ties/unknowns;27/41 lost preferences restored. At35.205s the correct
belt candidate is26.04mm from its sampled centroid; replacement cuts error norm
.42181→.20759 N m and score137.81→43.08 below wrong F132.65. At32.110s a2.79mm
point replacement worsens fit, while W error is .01621 N m. Remaining harmful
misranks31.095/31.175 have active-joint/free-couple discrepancies; lower W error
is not evidence that a wrench classifier would fix them. No tracking gain claimed.

A separately registered post-result ancestry audit bounds single-family C/T/W
error by raw observer loads on structurally inactive joints. Floor RMS is
.03904 N m (13.6%of T squared error) on41focus, .13518 N m (72.7%) on six harmful,
and .00213 N m on stable correct pairs. These are coordinate-energy bounds,
not causal shares. At35.205s shellB floor .20663 remains below the old .25106
allowance; geometry already fixes its rank. Do not attribute all remaining errors
to this floor or proven lag. All300 useful/harmful endpoints have one positive
sampled family load; S=T does not exclude multiple points or interval transfers.

Fifteen geometry/extraction and seventeen separate analysis tests pass, not the
full controller suite. A pre-data statistic-label correction and repeated17-test
stage are retained without inflating counts. Independent hinge loads agree within
4.44e-16 N m; eigen scores within1.82e-12; masks/cohorts/counterfactual codes exact.
Independent structural bounds match exactly. Producer NumPy1.26.4 reproduces177
arrays byte-for-byte. NumPy1.21.5 differs in floating bytes but preserves decisions
and bounded summaries; canonical values retained. Figure layout iterations are
preserved; final one-page overview and eight-page appendix contain14fixed anchors.

Decision: geometry matters but is an incomplete explanation; keep live baseline.
Next proposed: bank coverage versus selected-point quality at these events before
changing search, then matched substep logging to separate temporal transfer from
model/actuation error if needed. Neither follow-up runs here. Preserve progress
for presentation, including the failed primary criterion and remaining ambiguity.
[Report, plots, events and clone replay](docs/iterations/2026-11-27/contact-geometry-load-audit/README.md).

## I-67 / 2026-11-27 / implemented, run, analyzed; conditional bank capacity met

Question: are useful point alternatives missing from the original bank, or does
its fused objective select a different point? Preserve all102 slots, validity,
weights, I-65 calibration and comparison gates. Same I-58 native Gazebo/DART
Mars figure-eight logs: -1/+0.3/+0.9rad, yaw45deg,44s,5ms, existing caps. No new
physics, candidate, online selector, controller, observer or feedforward change.

Use705 fixed ROI states: all294 useful/six harmful opportunities (including all41
I-65 lost useful frames), plus14 I-66 event windows ±100ms. Freeze18 feature
source/inputs before coordinate reconstruction;10 source/cache inputs before
capacity arithmetic. Generator reads a blind allowlist, not truth/covariance.
Original F/Nalt coordinates byte-exact over all26403 states; ROI batch points
within2.22e-16m, exact finite patterns/masks,1e-12m declared tolerance.

Compare original selected, nearest usable, nearest pre-admission and minimum
frozen I-65 load-score points within truth family (evaluation only). Keep wrong
branch fixed; apply original ambiguity/tie rules to every candidate before ANY
reduction. Existing selected membership guarantees baseline nonregression by
inclusion; that invariant is not a new estimator improvement.

Primary target MET: all29 newly correct I-66 reference-point decisions have a
usable original-bank alternative (27 focus useful +two harmful), none unsupported.
Focus conditional capacity27/41 equals sampled reference27; nearest-geometry25.
All-useful ANY capacity280/294 versus253 original/276 reference; harmful5/6
versus2 original/4 reference. These oracle-assisted counts do not actuate or
supply a new achieved error rate. Fourteen focus cases fail both reference and
all usable correct-family options; one harmful case remains unresolved.

Focus median distance selected6.257mm→nearest usable0.813mm (p95 10.579→3.401).
Nearest pre-admission equals usable on every focus/harmful row, so nearest
projection is not lost to admission there. This does not show all excluded
points lack better load score or that bank expansion can never matter.
Minimum-load point distance median3.813mm on focus and19.495mm on harmful,
maximum33.532mm: low residual is not proof of accurate contact localization.

Original per-family fused argmins and all300 original full I-65 codes reproduce.
A separately registered component summary finds nearer-point extra fused cost
normal-prior dominated in26/41 focus rows, velocity in15/41 and allsix harmful;
wrench never the largest positive term. Every nearer point costs more under
the unchanged objective. At35.205s, closer4.696mm versus selected26.036mm has
cost changes (-.1375,-.00407,+5.5402), so velocity outweighs the other terms.
At13.470s normal prior outweighs a tiny velocity benefit. No bug, physical slip
cause or universal replacement weight is established.

Fourteen inclusive helper and twelve separate analysis contracts pass (26
focused tests, not full controller suite). Preserve the initial13-test helper
and pre-data offset/admission clarification without adding repeated counts.
Independent scalar minima/hinge/eigen review confirms all indices, masks,
cohort matrices and29/29 targets; max score difference1.82e-12. Its first review
mismatched nine geometry-only rows because of an out-of-scope load-minimum mask;
corrected review and original failed stage retained, no primary source change.
Producer NumPy1.26.4 replays177 arrays byte-exact; NumPy1.21.5 retains exact
indices/decisions and bounded values with nonexact float bytes. Final one-page
comparison and eight-page atlas retain14 anchors; initial layout retained.

Also authored docs/CHATGPT_CONTROL_CONTEXT.md, a standalone study/context upload
covering mechanism, dynamics, actual LQR/observer/feedforward/preview, inputs,
clipping, timing, live-vs-offline boundaries, I-60–I-67 and presentation FAQ.
It supplies equations and results without requiring the repository; links identify
optional evidence. Independent boundary review and local-link checks completed.

Decision: next target is within-family point selection and velocity/normal-model
assumptions, evaluated on the full cohorts before any integration. No automatic
bank expansion, pure load argmin, new weights or new physics. Interval force
routing remains limited by absent substep truth. Keep live baseline unchanged.
[Report, point clouds, fixed events and clone replay](docs/iterations/2026-11-27/contact-bank-audit/README.md).

## I-68 — 2026-11-27 — matched interval replay exposes hidden force routing

Status: completed instrumented diagnostic, not adopted control improvement.
Reproduce unchanged I-58 native m100_B on Gazebo8.10/DART6.19.5 Mars figure-eight,
shaft−1rad,yaw45°,44s,5ms control/1ms physics, original limits and gains.
Only physical run(5) becomes five run(1) calls; all solver contact data remains
evaluation-only. Freeze74sources/8inputs/two plugin binaries before physics.
Capture44,000solves/8,800holds; all59precisionfields, identical fieldsets and
8,801rows match old native history byte-exact. No new trajectory gain claimed.

Predeclare primary[1,41), broad[32,36), and fixed±100ms windows around7.875,
32.110,35.055,35.205. Integrate every reported point force at its pre-solve
Jacobian in native coordinates, then apply exact observer midpoint angular
rotation. Repeat post-pose sensitivity. Raw observer joint-vector RMS versus
integrated reference is.0000866Nm versus.116254Nm for final-solve-only;
broad.0001227 versus.100878. Both pose controls improve every fixed group.
These are consistency comparisons, not a calibrated sensor error bound.

32.110: three shellB positive-pressure samples then two beltB; raw shellB
load+.016193Nm agrees integrated+.016228, lastsolve0. This establishes interval
transfer here, not CPU latency. 35.205: pressure appears beltB-only, yet two
ADMITTED shellB rows carry26.53/12.72N tangent force with zero normalprojection.
They contribute−.206606Nm versus observer−.206627. Positive depths keep them
in native full-wrench sums, while positive-pressure family labels/centroids/
counts give zero weight. Earlier belt-only inference was pressure-only;
correction preserved. Do not read historical dominant-normal labels as all-force
routing or assert a physically valid zero-normal friction law / proven solverbug.

Separately sealed POSTHOC numerical screen detects1,119near-zero-normal force-
bearing rows in290/8,000primaryintervals; rejectedforcezero, maxFt38.04N.
Pattern also occurs in goodcontrol, so presencealone doesnotcausebadtracking.
Postsolve pressure-weighted slipRMS at35.205 remains.1392m/s versus.000281at
32.110 and.000329at7.875. Intrinsic-normal mismatch7.24°atlater event; these
support model-assumption investigation but do not establish replacementweights.

Fourteen recorder and13separate numerical contracts pass. Preserve initial
roundoff-exact testfailure, wrapper syntax error and lazyNPZ interrupted launch;
corrected5e-16offline tolerance and sealed eager adapter leave arithmetic and
native parity unchanged. Independent40-check traceaudit passes; actualmodel
Jacobian/force map within8.88e-16 and slip1.53e-16m/s overfourfixedintervals.
Three storage contracts; lossless14.13MB package restores36tracearrays and
both10fieldprecisionbindings byteexact onNumPy1.26.4/1.21.5. Cached producer
replay reproduces58scientificarrays byteexact and report values exactly.
Retain layoutstages and narrow earlier pressure interpretation transparently.
Preserve publication reseal failure/correction separately; final19.44MBarchive
omits redundantnativecommandJSON while14.13MBlosslessinputs remaincomplete.

Decision: retain nativeLQR/momentumobserver/feedforward/preparation. Audit the
installed contact/friction force-normal boundary and reference-label semantics
before another selector/weight/particlefilter. Onephase only; exactsolvercause,
otherstarts and improvedtracking remain unproved. Fullrawrolloutlocal; compact
package,5-pageplots/reports, source/protocol/review/failedstages tracked.
[Report, comparison, fixed events and clone replay](docs/iterations/2026-11-27/contact-interval-replay/README.md).

## I-69 — 2026-11-27 — installed solver explains the zero-normal forces

Status: source/controlled/native diagnostic completed; no controller or solver
adoption. Follow I-68's admitted zero-normal tangent-force records. Register
question/SPD3x3 witness before compile/run, then separately freeze74capture
sources/6inputs/5binaries before original native I58 m100_B reproduction.
Preserve LQR/observer/geometry/feedforward/preparation/gains/bounds/plugins.

Official DART6.19.5 source documents tangent boxes fixed once using intermediate
normal impulses; later pivots can change normals without resetting boxes.
Installed header/cached/tag match, installed/cached executable.text exact;
whole-library prefix relocation documented rather than claiming fullhashmatch.
Standalone installed-library SPD witness returns Dantzig success withx=(0,.7,0)
for both earlyflags: frozenboxvalid, finalnormal-dependent boundviolated. PGS
default fails convergence; no solver switching/tuning justified.

Private LD_PRELOAD observation delegates sameinstalled solver/applyImpulse calls,
logging fourfixed5ms intervals in a full44s Gazebo/DART Mars figure-eight,
shaft−1rad/yaw45°,5mscontrol/1msphysics. All59precisionarrays andALL36I68substep
arrays reproduce byte-exact. Newcapture160packets=20Dantzig(successall,PGS0)
+140appliedcontacts. Everyappliedlambda equals original-order parentx exactly;
normal/basis/lambda/dt reconstructforce within3.55e-15N. All140matchold/newexports,
25nonzeroforces unique collisionfamilytags, no signreversal needed.

35.205anchor shellB atactual35.203/35.205s: normalrow1 zero,tangentrows2/3
reference1,applied lambda=(0,.0201248655,-.0172934405)/(0,.0104874728,-.0072003873)
Ns. Both finalcomponent limitszero, originalcallsuccess; recreate26.5344/12.7213N
forces. This localizes those records to applied Dantzig approximation before
export/observer. Acrossselectedanchors12/140contacts violatefinaldependentboxes,
includingtwo goodcontrol rimAcontacts;32.110hasnone (temporaltransferinstead).
No claimapproximationexplainsentiredeviation orbadtrackingalone.

Four core andseven separate adapter contracts pass. Preserve initialsynthetic
exactfloat failure/test-only2e-13correction, prelaunchfallbackbuffer/frictionless-
basis corrections, independent+infinity serializerfailureandcorrection, layout
stage01 overlap/stage02fix. Independentdecoder agrees140contacts/12violationIDs,
59/36parity andall25nonzero same-sign exports. No runtime plugin/library edits.
Two storagecontracts;28,319bytesofpacket/contextNPZ plusmetadata preserveactive
mathematicalvalues, omituninitializedpadding andunknownfulltraceareas explicitly.
NumPy1.26.4replaysall220arrays/reportexact;1.21.5strictfailurepreserved, onlyderived
roundoff≤1.39e-17arrays/3.55e-15report, allinputs/discreteoutputs exact; separately
scopedboundedverificationpasses withoutreplacingvalues.

Decision: keepnativecontroller/observer asbaseline; validatecorrectedapplied
contactsolution(finalbounds+complementarity/convergence) on savedproblems before
controlledshell/belttransfer andtrajectory comparisons. Do not merelyclipreported
forces, whichwouldbreakmotion/wrenchconsistency. No correctedsolverimplemented.
[Report, explanation PDF and cached replay](docs/iterations/2026-11-27/contact-force-boundary/README.md).

## Evidence preservation and reproduction

Compact report/plot/test snapshots live outside ignored `results/`. The dated
manifest records their original locations and hashes, the original per-campaign
provenance, and a media inventory. Full MP4s and NPZ rollouts remain local. A
shareable presentation bundle includes these notes and three selected clips.
The first presentation bundle captured I-01–I-10; the I-11 follow-up above is
preserved separately with its own evidence manifest.
The completed I-12–I-14 configuration study has a separate
[presentation/evidence index](docs/iterations/2026-11-25/configuration-study/README.md)
and [manifest](docs/iterations/2026-11-25/configuration-study/manifest.json).

Original experiment sources included uncommitted edits. Source hashes identify
the recorded versions, but cannot recreate missing historical source files.
The snapshot's `source/` folder preserves the relevant current controller,
diagnostic, script, and test files at the end of this backfill. It is explicitly
a later source snapshot, not the exact tested source of every historical run.
Archived local reproduction scripts are retained as evidence; some derive paths
from their location and must be restored to their original manifest paths to run.
Current-source reruns are new experiments, not guaranteed bitwise reproductions.

The current focused runner and comparison tool are
[run-contact-diagnostics.py](scripts/run-contact-diagnostics.py) and
[compare-contact-allocation.py](scripts/compare-contact-allocation.py).
Run them inside the Gazebo environment described in [GAZEBO.md](GAZEBO.md):

```bash
MUJOCO_GL=egl python scripts/run-contact-diagnostics.py --variant baseline --output results/new_iteration/baseline
MUJOCO_GL=egl python scripts/run-contact-diagnostics.py --variant bounded --output results/new_iteration/bounded
MUJOCO_GL=egl python scripts/run-contact-diagnostics.py --variant bounded --effort-weight .005 --output results/new_iteration/stronger
MUJOCO_GL=egl python scripts/run-contact-diagnostics.py --variant bounded --adaptive-effort --output results/new_iteration/adaptive
python scripts/compare-contact-allocation.py --baseline results/new_iteration/baseline --candidate results/new_iteration/adaptive --output results/new_iteration/comparison
```

## Template for future iterations

```text
ID / date / status: proposed | implemented | run | analyzed | retained | rejected
Question or hypothesis:
Change and parameter values:
Baseline campaign / source revision or hashes:
Simulator, model, measurements, routes, initial conditions:
Experiment and acceptance criteria:
Observed outcome, including regressions/setup failures:
What this establishes and what remains uncertain:
Decision and next hypothesis:
Evidence: raw report, compact snapshot, plot, video, test log, reproduction command
```

Append results to a new entry or clearly dated follow-up. Keep earlier results
and rejected candidates visible when the preferred approach changes.
