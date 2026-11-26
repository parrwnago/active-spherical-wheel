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
