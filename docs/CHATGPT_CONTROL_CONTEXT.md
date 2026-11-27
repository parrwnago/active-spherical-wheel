# Active spherical wheel: standalone context for another ChatGPT

**Revision 2 — 27 November 2026, through I-67.** Live-source baseline: repository
commit `4d6086b`; I-67 changes no live control code. This is a study and discussion
document, not a claim that every retained experiment is part of the operating
controller. It is intended to remain useful without access to the repository.

## Instructions for the ChatGPT reading this file

Use this document as project context. Explain mechanisms and equations in plain
language before introducing jargon. Help the student understand and defend the
work, including unsuccessful iterations. Distinguish **implemented live
behavior**, **offline observations**, **causal hypotheses**, and **proposals**.
Do not assume the repository, videos, or raw logs accompany this upload. Links
below identify evidence in the repository; ask for the relevant file when its
contents matter and are not supplied here. Never invent a result or describe a
simulator-truth diagnostic as information available to the live estimator.

The current choice is **gain-scheduled LQR, momentum observer, geometry-based
support estimation, rolling/holding feedforward, and short incoming-drive
preparation**. All work is simulation-only. Exact simulated body state is
allowed; contact truth is reserved for evaluation in the selected experiments.
No MPC, iLQR, particle filter, camera, LiDAR, SLAM, or live dynamic-contact
forecast currently commands the selected robot. There is a limited **kinematic
preview**, explained below. The code also contains alternative modes and
rejected candidates, so finding a module does not establish that it is active.

## 1. Mechanism, state, and simulator

The robot is a balancing platform supported by a ball-like wheel. A fork
connects the upper platform to a common shaft/carrier. The carrier rotates
relative to the fork; two hemispherical shells rotate independently relative
to the carrier. Each pole has a belt module fixed to the carrier. As the
assembly rotates, support can transfer between shell A, belt A, shell B, belt
B, and equatorial rim contacts. The two shells are not ordinary parallel
fixed-axis wheels. Their current geometry changes the direction and lever arm
through which each drive affects motion.

The nominal outer radius is **0.12 m**. The shells have a 0.11 m inner radius,
an equatorial half-gap of 0.010 m, and polar openings derived from the supplied
geometry. Collision/contact patches approximate these clipped spherical
surfaces. Mass properties are explicitly **illustrative design-study values**,
not measured hardware or a complete native-CAD inertia identification. Gear,
bearing, electrical, and belt-motor load dynamics are not modeled.

The generalized position has ten numbers:

```text
q = [world position (3), orientation quaternion (4),
     shaft angle, shell-A angle, shell-B angle].
```

The generalized velocity has nine:

```text
v = [world linear velocity (3), body-frame angular velocity (3),
     shaft rate, shell-A rate, shell-B rate].
```

A unit quaternion uses four coordinates for three rotational degrees of
freedom. Therefore `qdot = N(q) v`; quaternion derivatives are not angular
velocities. The small configuration error is nine-dimensional, and adding
nine velocity errors gives the controller's **18-dimensional tangent error**.
Orientation error is a three-component relative rotation, not subtraction of
four quaternion components.

| Input | Meaning | Unit | Assumed absolute bound |
| --- | --- | --- | ---: |
| `u[0]` | Common-shaft torque | N m | 1.5 |
| `u[1]` | Shell-A torque | N m | 0.6 |
| `u[2]` | Shell-B torque | N m | 0.6 |
| `u[3]` | Belt-A surface speed | m/s | 0.30 |
| `u[4]` | Belt-B surface speed | m/s | 0.30 |

The belt channels prescribe ideal surface velocity at contact. They are not
torque commands to modeled belt motors. A shell joint-rate reference, meanwhile,
is a feedback target; it does not directly prescribe actual shell speed.
These distinctions matter when interpreting “effort,” acceleration, and motor
feasibility.

Recent driving evidence uses **Gazebo Harmonic with DART**, on a Marsyard2021
terrain derived from Leo Rover assets. A companion **MuJoCo** model supplies
mass/bias calculations, Jacobians, local LQR design, and private kinematics.
The measured-state mirror used by the selected controller has no live ground
contacts to query. This is not a claim that two engines have identical contact
dynamics. Physics advances in **1 ms** steps; commands are held for **5 ms**,
giving 200 Hz control.

Sources: [model configuration](../config/model.yaml),
[state/actuation interface](../src/spherical_wheel/sim.py),
[Gazebo interface](../src/spherical_wheel/gazebo/plant.py),
[Gazebo documentation](../GAZEBO.md).

## 2. Dynamics and the complete control loop

The model convention can be written

```text
M(q) vdot + c(q,v) = S tau + f_passive(q,v) + f_external.
```

`M` is the nine-by-nine mass matrix; `c` contains modeled velocity-dependent
and gravity bias forces. `S tau` applies the three internal motor torques.
Passive terms include modeled damping. Contact forces, including traction
created by prescribed belt motion, enter through `f_external`. A point force
has generalized load `J_point(q,p)^T F`; a contact couple adds its angular
Jacobian contribution. One cannot replace this geometry by a fixed wheel
radius for every configuration.

At each control time `t_k`, the selected loop does the following:

1. Read exact simulated `q_k, v_k`. Retain the input actually applied during
   the preceding interval, including clipping.
2. Compare momentum change over that preceding interval with modeled known
   forces. Filter the inferred external generalized load.
3. Infer a support normal, equivalent contact location, supporting patch
   identity, and approximate load shares from the wrench and current geometry.
4. Update normal/rim filtering and construct the current local support plane.
5. Read the timed XY position, velocity, and acceleration reference. Construct
   lean, rolling joint-rate/belt-speed references, holding feedforward, and
   support-dependent LQR feedback.
6. Blend incoming-drive preparation, apply enabled correction/fade logic, and
   clip the **total** five-input command to its bounds.
7. Hold that applied command for five 1 ms physics steps. Save diagnostics and
   advance nominal joint-reference bookkeeping.

The observer thus estimates a recent interval; it does not see the future.
The simulation pauses for controller calculations, so these recordings do not
establish a real computer's execution latency. Filtering, interval averaging,
current-geometry reconstruction, and actual computational latency are different
possible sources of timing error.

The selected source mode is named **`hardware`**, but this describes an
information boundary, not a hardware experiment. Generic `terrain_aware` mode
reads solver contacts; generic `momentum_observer` mode replaces the wrench
but still uses some solver support geometry. Neither should silently be called
the selected contact-truth-free configuration.

Source: [offroad control](../src/spherical_wheel/offroad_control.py).

## 3. What the LQR and feedforward actually do

LQR means **linear quadratic regulator**. Around a flat-ground equilibrium at
a particular support configuration, finite differences of the full 5 ms
held-command transition produce local matrices `A` and `B`. The design scales
state and input coordinates, reduces to the reachable subspace, and solves a
discrete algebraic Riccati equation. Growing discarded modes are rejected.
The default design uses the full 18-component tangent state; a separate
frozen-phase design function also exists and must not be confused with it.

The local objective has the familiar form

```text
sum_k [e_scaled(k)^T Q e_scaled(k) + du_scaled(k)^T R du_scaled(k)].
```

Balance, travel errors, and input use receive different normalized weights.
`R = diag(0.3, 0.3, 0.3, 1, 1)` in normalized coordinates. This is not a
calibrated electrical-energy cost: three inputs are torques and two are speeds.
The Riccati solution gives a linear feedback gain, rather than solving a new
constrained optimization at each sample.

For reference, the full flat-design state scales and diagonal weights are:

```text
state scales = [.15,.15,.005, .15,.15,.5, 1,1,1,
                1,1,.1, 1,1,1, 10,10,10]
Q diagonal  = [2,2,1, 30,30,.01, .001,.001,.001,
               2,2,.1, 4,4,.05, .01,.01,.01]
input scales = [1.5,.6,.6,.3,.3] in the mixed units listed above
```

The coordinate order is position, orientation tangent, joint position, linear
velocity, angular velocity, joint velocity (three entries per group). In the
scaled reachable coordinates, the discrete Riccati equation and gain are

```text
P = Ac^T P Ac - Ac^T P Bc (R + Bc^T P Bc)^-1 Bc^T P Ac + Qc
Kc = (R + Bc^T P Bc)^-1 Bc^T P Ac.
```

`Ac,Bc,Qc` are the reachable-subspace matrices, not the original full matrices.
The implementation maps this gain back through that subspace and physical
state/input scales. The numbers are design choices, not measured energy costs.
Runtime error modifications and gain interpolation still matter afterward.

A library stores local gains across shell/belt support phases. Runtime
interpolation blends nearby phase gains, then weights support families by
their **estimated** normal loads. The selected schedule measures phase against
world vertical, not the estimated terrain normal. A short inferred contact
dropout retains the previous loaded gain/reference. The shaft-command row
uses the ordinary startup design. However, I-59's identical-state audit found
swapping startup shaft rows changed requests by at most `2.53e-8 N m`; this
does **not** support that retained row as the explanation for phase sensitivity.

Runtime errors receive several deliberate transformations. XY position and
linear-velocity errors rotate into the current heading frame. Desired
orientation follows current yaw, making heading free in these trials, although
yaw-rate feedback remains. Height follows the measured terrain motion rather
than imposing a flat-height error. Shell material-angle errors are zeroed
because rotating an axisymmetric shell is not itself a balance error. The
selected rolling-consistent shaft error is `heading_x_error/R - pitch_error`.
All velocity feedback remains. These modifications are part of the actual
controller, beyond the textbook formula.

The pre-preparation request is

```text
u_request = u_equilibrium + u_rolling + u_holding - K_scheduled e.
```

**Rolling feedforward** uses inferred contact-point Jacobians to solve
horizontal no-slip velocity equations. Its five unknowns are three joint-rate
references and two belt surface speeds. Native least squares uses numerical
normalization `[5,5,5,0.3,0.3]`; those preferences are **not hard motor-speed
limits**, and native rows are not load-weighted. The three joint rates feed
the feedback reference; the two belt speeds feed directly into command
feedforward. The first three rolling torque-feedforward channels are zero.

**Holding feedforward** compensates modeled gravitational moments associated
with support offset and divides shell holding action using inferred loads.
The reference lean also accounts for desired acceleration and estimated
support displacement. Its 28-degree desired-lean ceiling is not a guarantee
that actual tilt stays below 28 degrees.

Finally, preparation and fade logic act before hard clipping. LQR's effort
penalty makes large commands expensive but **does not enforce an inequality**
such as `|u| <= bound`, anticipate clipped dynamics, or reserve balance
authority. The final actuator command is bounded; the internal request can
be much larger. Neither clipping nor a bound on a joint-rate reference
guarantees a bound on actual joint acceleration.

Sources: [LQR design](../src/spherical_wheel/control.py),
[gain library](../src/spherical_wheel/recovery_control.py),
[rolling allocation](../src/spherical_wheel/experiments.py),
[runtime integration](../src/spherical_wheel/offroad_control.py).

## 4. Momentum observer: information, frames, and timing

Define generalized momentum `p = M(q)v`. Then

```text
pdot = S tau + f_passive - c + Mdot v + f_external.
```

The observer subtracts the integral of the modeled known terms from measured
momentum change. It avoids directly differentiating measured velocity to
obtain acceleration. In the implementation, endpoint passive/bias terms use
trapezoidal approximations, and the `Mdot v` integral is approximated by
`(M_k-M_previous) * average(v_previous,v_k)`. Dividing the remaining momentum
change by the actual interval gives `interval_mean`, a nine-component
external-load estimate for the preceding interval.

The raw estimate is filtered as

```text
gamma_k = gamma_previous
          + (1 - exp(-100 * dt)) * (interval_mean_k - gamma_previous).
```

The cutoff is **100 rad/s**, not 100 Hz; its time constant is 10 ms. At 5 ms,
the new interval contributes about 0.3935 of the update. This filter and the
5 ms averaging introduce memory even with instantaneous computation.

Force rows 0–2 are world-frame forces. Moment rows 3–5 are converted from the
free-joint/body convention to world coordinates using midpoint orientation
before filtering; they describe moment about the wheel/free-joint centre.
Rows 6–8 are scalar generalized loads at the shaft and shell joints. Mixing
world moments with untransformed body-frame moments would be an error.
The selected live support locator consumes the six resultant force/moment
components. The extra three joint-load components are used in the recent
offline diagnostics; their proposed veto/scoring rules are not active control.

Inputs are pose, velocity, the previous **actually applied** command, model
mass/bias/passive terms, and time. No acceleration sensor, current reading,
solver contact force, or true contact point is required by this calculation.
The simulated direct-torque model converts known motor commands to modeled
torque. That is not experimental evidence that commanded torque always equals
real motor torque. A future robot would need suitable sensing/calibration;
there is no actual robot in this project yet.

Internal motor torques cancel in free-body resultant rows, but affect joint
rows and known momentum balance. Belt traction is external contact load, not
a fourth/fifth generalized motor-torque input. Any unmodeled friction, unknown
push, inertia error, or measurement error can also appear in the residual.
“Observer output” therefore means inferred total external load, not uniquely
ground contact or an exact contact decomposition.

Source: [MomentumObserver](../src/spherical_wheel/contact_sensing.py).

## 5. From wrench to support, and why a close point can be wrong

The first six observer components provide net force `F` and centre moment
`M`. For a single ideal point force, `M = r cross F`. Known spherical geometry
and compression constraints help choose a support normal and point near
`centre - R * normal`. Current shell/belt band geometry then assigns a family.
Special equatorial-rim handling can infer a two-rim distribution. Normal/load
filters temper changes, especially near the gap.

This is an **equivalent-support construction**. A moment parallel to `F`
cannot be explained by a pure point force; multiple contacts can contribute
the same net wrench. Estimated family loads are derived from that same wrench
and geometry, not extra independent load sensors. A single resultant cannot
generally recover arbitrary contact points, tangential forces, free couples,
and load shares simultaneously.

Point accuracy and identity accuracy are different. Near a shell–belt seam,
two nearby points can belong to different driven bodies. A previously audited
example had only 0.315 mm position discrepancy yet assigned belt A while shell
A carried the load. That changes speed references and gain selection even
though a point-distance plot looks excellent. It does not by itself establish
whether filtering, geometry, force distribution, or another modeling choice
caused the mistake.

The estimator also reports a friction/slip-related diagnostic. With inferred
contacts it is based on reconstructed material-point motion. It is not an
independent true-slip sensor. Estimated friction is reported rather than
hard-limiting commands: earlier friction-based torque reduction harmed
balance. Exact support identity, arbitrary multi-contact reconstruction, and
continuous transfer of control authority remain limitations.

Source: [SupportLocator and ContactEstimator](../src/spherical_wheel/contact_sensing.py).

## 6. Is there contact prediction? Yes, but only a narrow preview

The active synchronizer integrates current generalized velocity forward on a
private kinematic model for **120 ms**, against the current observer-derived
local plane. It checks whether a shell/belt patch may become geometrically
exposed, then prepares an incoming drive to reduce predicted ground-tangent
material velocity. This is more useful than simply equating shell angular
speed with belt linear speed: the matching condition depends on lever arm,
body motion, and orientation.

An unloaded shell uses a torque speed correction, with a numerical target
ceiling of 20 rad/s and blend capped at 0.35 because its reaction torque moves
the platform. Belt preparation requests bounded surface speed. At an estimated
5% normal-load share, preparation weight is zero and loaded feedback takes
over. This switch is not guaranteed command-continuous.

It predicts **geometric exposure under constant velocity**, not future force,
load sharing, slip, exact touchdown time, unknown bumps, or a dynamically
closed-loop trajectory. It has no future terrain map. Prior dynamic forecasts
and four-direction camera-memory work were exploratory and are parked.
“No prediction at all” and “we predict future contact dynamics” would both
misdescribe the selected system.

Source: [ContactSynchronizer](../src/spherical_wheel/synchronization.py).

## 7. Velocity consistency and the offline experiment series

The student's idea was to infer contact from disagreement between intended
travel and actual travel. The more controlled version compares **measured
joint/belt motion plus candidate geometry** with independently available body
velocity. A wrong effective radius/family can create disagreement, but so can
slip, saturation, transient dynamics, poor excitation, or model error. Desired
speed alone does not identify the cause. In this ideal belt model, previous
applied belt speed is known; it is not a simulated encoder/current measurement.

I-60 formed a frozen bank of **102 candidates**: 17 observer-normal seeds
within a 10-degree cone, across four spherical patch bands and two shell rims,
mapping to four family labels. Geometry, moment consistency, and measured
velocity supplied scores. Fixed excitation, residual, separation, and margin
rules allowed abstention. These scores are not calibrated probabilities, and
the observer-conditioned domain can omit a good alternative. This is not a
particle filter and it has not replaced the live locator.

### Common evidence and notation

I-60–I-67 reuse three I-58 **native** 44-second Gazebo/DART Mars figure-eight
recordings: shaft starts **-1.0, +0.3, +0.9 rad**, initial yaw 45 degrees,
unchanged input bounds. Each has 8,801 saved states. The primary `[1,41)`-second
evaluation mask has **23,423** states with at least 1 N total normal load and
one uniquely dominant family carrying at least 90%. Weaker contacts remain
possible; “dominant” does not mean strict single contact. Contact truth is
used for these offline labels and checks, not the selected live loop.

`N` denotes the original native equivalent-support result. `F` denotes the
frozen velocity/fused proposal. Offline policy `A` uses F's point/family when
accepted and otherwise N. A has **124** family errors, improves 294 native
errors, and introduces six harmful changes relative to N. There are 6,902
accepted proposals: 18 wrong, including 12 where N was also wrong. Acceptance
covers 86.60% of ambiguous motion-eligible rows but only 29.47% of all primary
rows; it is not near-universal replacement coverage. The 294
useful and six harmful opportunities are small, correlated row sets, not
300 independent trials.

`Nalt` is a **physical candidate from the frozen bank in N's family**. It is
not N's original equivalent point. Family correctness is also not proof that
the candidate point or force representation is correct.

| Iteration | Question and result | Decision |
| --- | --- | --- |
| [I-60: velocity ranking](iterations/2026-11-27/velocity-contact-ranking/README.md) | Raw wrong/unknown labels 412→109; all 12 shadow gates pass. Accepted F has 18/6,902 errors versus N's 306 on the same rows. Velocity-only and fused labels are identical. Raw point RMS 3.134→2.672 mm, but maximum 26.91→44.04 mm; A's maximum is 29.843 mm. | Useful offline identity evidence, unsafe to assume improved feedforward geometry. No adoption. |
| [I-61: joint-load veto](iterations/2026-11-27/contact-load-consistency/README.md) | Compare three extra joint-load components with `J^T F`; fixed bound `0.03 + 0.10 R norm(F)` N m. Removes one harmful override but loses 30 useful corrections: A's 124 errors become 153. Five of 13 gates fail. | Reject this veto. |
| [I-62: threshold tuning](iterations/2026-11-27/contact-load-tuning/README.md) | 3,721 shared nonnegative `(a,b)` settings in `a+bR norm(F)`, with whole-start holdouts. No qualified training/full setting. Best label-only count is 123, but retains bad point tails. Point-safe grid minimum is 221 errors. | Parameter adjustment insufficient for the combined goals. |
| [I-63: condition-dependent allowance](iterations/2026-11-27/contact-condition-diagnostic/README.md) | Add `c max(u,s)`, using observer innovation `u` and seam/rate risk `s`. An exact necessary-condition audit still forces 99 useful losses to remove the bad peak. | Stop before the conditional 18-schedule fit. Excludes this rule class, not all adaptive methods. |
| [I-64: matched load history](iterations/2026-11-27/contact-load-history/README.md) | Match raw interval forces and the observer's filter using frozen estimated A shadow-path history on recorded native motion. Correct-family RMS 0.05828→0.03721 N m on 6,884 paired accepted rows, improving every start: model-consistency Q1 passes. Discrimination Q2 fails; shadow policy has 128 errors and prevents none of six harmful changes. | Better time-consistent comparison, no adopted veto. |
| [I-65: covariance-aware relative score](iterations/2026-11-27/contact-covariance-ranking/README.md) | Fit shared three-vector bias/covariance on two whole episodes, score the third. Useful preferences improve 244→253/294 and harmful rejections 1→2/6 versus relative max-component scoring. Shadow result is 163 errors: loses 41 useful corrections to prevent two harmful changes. Point maximum stays 29.843 mm. | Different method, net regression; no adoption. |
| [I-66: point versus load representation](iterations/2026-11-27/contact-geometry-load-audit/README.md) | On those 41 lost useful corrections, sampled-point replacement lowers joint-vector RMS 0.13926→0.10568 N m, 24.1%. Thirty-seven improve, four worsen; the registered criterion fails because the -1-rad focus case worsens. Truth-assisted relative preferences become 276/294 useful, 4/6 harmful. | Geometry matters, but does not explain all disagreement. Diagnostic only. |
| [I-67: original-bank capacity](iterations/2026-11-27/contact-bank-audit/README.md) | Existing usable candidates reproduce all 29 newly correct I-66 reference-point decisions. On the 41 focus rows, selected median distance is 6.26 mm versus 0.81 mm for the nearest usable candidate. Known-correct-family any-point capacity is 27/41 focus, 280/294 useful and 5/6 harmful. | Selection opportunity within the existing bank; no deployable selector or live improvement. |

Every row is offline saved-log work, not a new driving improvement. Episodes
were inspected before these later analyses; whole-episode holdouts reduce
leakage but do not create prospective validation. No table row should be
presented as a completed hardware test or an adopted control upgrade.

### Why the later load comparisons differ

I-61's instantaneous filtered-load comparison mixes current geometry with
filtered force/history. I-64 compares raw interval load directly (M1), or
filters predicted load using its own frozen estimated A contact history (M2),
with one observed seed and 200 ms warmup. It does not re-seed from observed load
every sample or double-filter raw force. The M2 veto restores all 30 useful
corrections lost by M0, but loses four different useful corrections: net 26
fewer, not a uniform repair. The raw M1 absolute veto has 132 errors; that is
different from I-65's relative scoring experiment.

I-65 fits one shared residual mean/covariance for both branches, with equal
episode weights and at most one known-correct-family training branch per
timestamp. **Training uses simulator family labels** from the other episodes;
held-out inference does not. “Correct family” does not certify a correct point.
Fixed 0.2 trace shrinkage and a 0.05 normalized standard-deviation
floor regularize it. This models residual variation, not pure sensor noise.
The score changes both bias handling and covariance weighting; its result
does not isolate a benefit from correlations alone.
Ambiguous predicted patterns retain A. Most accepted pairs are same-family
duplicates, so only a small fraction provide meaningful identity competition.

The native-equivalent history control also deserves care: its assigned-body
point is materially invalid on 1,817/23,423 primary rows, although the virtual
Jacobian remains finite. This is a history-model diagnostic, not evidence of
a physical contact there. Physical F/Nalt comparisons retain separate validity.

I-62/I-63's **217-error bound** is a lower bound for the specified frozen
nonnegative affine rule: reject the required worst point and lose at least
99 useful changes, with at most six harmful changes available to repair,
giving `124+99-6`. It is not an achieved result, nor an impossibility theorem
for all contact estimation, nonlinear thresholds, or future data.

## 8. What I-66 can and cannot establish

The raw observer load averages the preceding **5 ms**, whereas saved truth
points/normal loads come from the **last reported 1 ms physics step**.
Those points are normal-load-weighted centroids, sometimes inside the sphere;
they are not guaranteed individual contact or wrench-equivalent points.
Individual force vectors, intervening per-contact histories, and true joint
generalized loads were not saved for these three native episodes. Older
substep recordings from different runs cannot fill the gap.
Saved total truth force/moment are float32 endpoint context, not an exact
5 ms reference; high-precision pose and centroid records do not restore the
missing per-substep force history.

I-66 keeps current recorded pose and raw observer force fixed. It compares:

- **C:** the correct-family physical candidate, F first or otherwise Nalt;
- **T:** replace only C's point with that family's sampled centroid;
- **W:** assign the full inferred force and centre moment to that family,
  allowing a free couple rather than requiring a point force;
- **S:** split observer force by sampled normal-load shares, assuming a common
  force direction, using each family's sampled centroid.

On the 41 focus rows, C→T improves pooled RMS but not every starting phase.
W lowers RMS further to 0.06893 N m and meets the unchanged component bound
on 41/41 rows; T and C meet it on 39/41. W uses additional information from
the **same observer**, so it is not independent ground truth. S equals T on
this focus cohort, not a validation of general multi-contact force routing.

At **35.205 s**, the correct physical Nalt belt point is **26.04 mm** from
the sampled centroid; replacing it fixes that relative decision. The native
equivalent point is a different point and must not be assigned this distance.
At **32.110 s**, correcting a 2.79 mm point displacement worsens agreement,
while W helps. Structurally inactive joint rows impose error that moving a
point cannot repair. Such a floor does not imply the old component bound must
fail or explain every remaining failure. These observations support both
geometry sensitivity and remaining load-model disagreement. They do not
uniquely prove observer lag,
slip, multi-contact, or a simulator fault. The truth-assisted counterfactual
also keeps a known-wrong branch unchanged; it is explanatory, not deployable.

## 9. Earlier controller trials and unresolved limits

MPC was explored in I-31–I-38 but did not deliver reliable settling sufficient
to replace LQR. MPC is only better when its model, objective, horizon,
constraints, and solver match the problem. iLQR and particle filtering were
discussed; neither is selected. Camera/terrain-memory and dynamic preview
experiments similarly did not establish improved driving.

Bounded rolling allocation, configuration-dependent effort, reference guards,
normal-relative gain scheduling, and total-command allocation were tested.
Some improved individual starts, but failed broader gates. For example,
I-56's bounded total-command candidate worsened peak tracking 116.26→136.16 mm
and final error 42.65→76.72 mm. I-58 smoothing improved the -1-rad lead case
but regressed other starts. Completion, numerical constraint satisfaction,
and adoption are separate outcomes.

The main unresolved issues are:

- **Configuration-dependent authority.** Useful drive directions can nearly
  coincide, or a lever can vanish. Matching a sideways request may require
  huge opposing rates. Both hemispheres need not be loaded; shaft rotation,
  shell rotation, and body attitude all matter. Tilt alone is not the cause.
- **Saturation and handoffs.** Large pre-existing errors combined with a
  support/gain change can produce a request jump, followed by clipping.
  Pre-spinning an unloaded shell also produces reaction torque and can affect
  yaw. No global stability certificate covers interpolation, switching,
  clipping, and nonlinear rough terrain.
- **Turning and phase sensitivity.** Timed reversals can overshoot or delay;
  initial phase changes later support and command history. A good -1-rad
  video does not establish robustness at +0.3/+0.9 rad.
- **Ambiguous inference.** A good equivalent point, correct family, plausible
  net wrench, and correct force history are different tests. Multi-contact
  remains incompletely observable from the present signals.

For an illustrative two-direction calculation, suppose effective drive
directions are `(1,0.01)` and `(1,-0.01)`. To obtain lateral velocity
`(0,0.10)` m/s, their equivalent speed coefficients must be `+5` and `-5`
m/s: large forward components cancel, leaving a small sideways component.
This is a toy explanation of poor authority, not a measured robot event.
An effort cost can accept lateral error instead; it cannot create a missing
physical direction.

Sources: [teammate handoff](TEAMMATE_HANDOFF.md),
[iteration ledger](../ITERATION_LOG.md),
[presentation notes](iterations/2026-11-27/README.md).

## 10. Questions to practice explaining

**Why can a slow requested path saturate a belt?** Because the total command
also balances the platform. At recorded 6.825 s, desired planar velocity is
about `(0.0324,0.0313)` m/s, but belt-A request is 0.83474 m/s and applied
speed 0.30000. LQR contributes +0.84380 and rolling feedforward -0.00906;
orientation/angular-rate terms account for 88.6% of the positive LQR request.
This particular saturation is not mainly rolling feedforward. It does not
establish the cause of every trajectory deviation.

**Isn't effort penalization already a hard constraint?** No. Finite quadratic
costs permit large requests for sufficiently large errors. Final clipping
protects command magnitude but changes the realized dynamics. A constrained
total-command allocation or reference governor could account for limits, but
must preserve balance and pass multi-phase tests. Prior versions regressed.
Higher motor speed is a design hypothesis, not a substitute for this analysis;
the ideal belt model omits real torque-speed and power limits.

**Must the robot reach every point at a specified time?** Not inherently.
Geometric path following can adjust progress to conditions. The selected
controller tracks a timed reference; switching to progress-based following
would be a new design with separate evaluation, not a setting already active.

**Why not determine contact from measured velocity alone?** Different
candidates can predict nearly identical motion; low motion gives little
information. Slip and actuator limits can mimic geometry error. Use genuine
measured rates and independent body motion, retain unknowns, and compare
competing hypotheses rather than equating mismatch with one cause.

**Why doesn't lower joint-load error prove the right contact?** Both candidate
models may be poor. At 35.205 s, I-65 covariance scores are about 132.65 for
wrong F and 137.81 for correct Nalt: a relative win still chooses wrong.
Both are far above the relevant training 95th percentile, about 9.56.
Torque-null directions can also make different points indistinguishable.
Full-wrench W relaxes the point-force assumption, so lower residual is not
independent confirmation of physical identity.

**Can we blame estimation lag?** Memory and timing mismatch are plausible.
The current logs cannot reconstruct exact preceding-5-ms contact-force
history, and they do not measure CPU latency. Use that uncertainty explicitly
instead of turning a plausible explanation into an established cause.

**What is the defensible project contribution?** Integration and evaluation
of this mechanism/model, momentum-based support construction, contact-aware
references/feedforward, scheduled feedback, drive preparation, and controlled
diagnostics. LQR, Riccati solvers, and generalized-momentum observers are
established methods. Rejected experiments are useful evidence about where
this integration fails, not new control-theory proofs.

## 11. I-67: does the original bank already contain a useful point?

**Completed offline, with original points and rules unchanged.** The audit
uses 705 saved rows: all 294 useful and six harmful opportunities, plus the
14 fixed I-66 event windows. It reconstructs the original 102 slots from saved
pose and normal estimates, retaining the original usable-candidate mask.
It does not add points, expand the 10-degree cone, relax compression gates,
refit covariance, or alter the controller.

The original selector minimizes a fused score combining normal-angle prior,
net-moment mismatch, and tangential velocity mismatch. I-67 asks a different,
evaluation-only question: **given the known correct family, does any existing
usable point produce a correct relative I-65 joint-load preference?** The
wrong-family branch and all comparison rules remain fixed. Truth identifies
the family and reference only for this audit.

All **29** newly correct I-66 sampled-point decisions can be reproduced by an
existing usable candidate: 27 rejected useful corrections and two additional
harmful rejections. No target repair requires enlarging the bank. On the 41
focus rows, median selected-point distance is **6.26 mm**, compared with
**0.81 mm** for the nearest usable point. The nearest pre-admission projection
is the same distance on every focus/harmful row: the tested admission rules
are not hiding a closer projection on those rows.

The known-family existence counts are 27/41 focus, 280/294 useful and 5/6 harmful.
These are **capacity**, not an achieved new estimator. Existing-point capacity
must retain the original success by set inclusion, so its nonregression is an
invariant, not evidence of generalization. Fourteen focus rows still fail the
frozen load comparison with every usable correct-family candidate, and the
sampled reference also fails on those same rows.

At 35.205 s, the original correct-family belt point is 26.04 mm from the sampled
centroid, while an already usable point is 4.70 mm away. The fused selector
nevertheless prefers its original point because the velocity-cost advantage
outweighs the other terms. This explains the computed choice; it does not
prove physical slip or an implementation bug. In the fixed 41 focus rows,
the normal prior supplies the largest extra penalty for the nearer point in
26 cases and velocity in 15; velocity dominates all six harmful cases. One
universal weight adjustment is not established.

Closest geometry and lowest load score are different objectives. On the six
harmful frames, minimum-load-score points have median centroid distance
19.50 mm even though nearest-geometry points have median distance 3.28 mm.
Ambiguous lever directions and model mismatch can favor a low-residual point
that is poorly localized. Choosing the nearest recorded point requires truth;
choosing the minimum load score alone is not a validated solution either.

The justified next target is point selection within the existing family bank,
with velocity and load-model assumptions examined explicitly. Candidate
expansion and new weights remain proposals. Exact temporal load routing still
needs matched substep force records in a later instrumented replay. None of
these diagnostic results changes the selected live LQR/observer/feedforward.

## 12. Sources, attribution, and how to continue a discussion

The repository's [study-guide PDF](iterations/2026-11-26/presentation-preparation/control-and-observer-study-guide.pdf)
and [teammate handoff](TEAMMATE_HANDOFF.md) provide further derivations and
evidence links. Primary references already checked in that guide are:

- Russ Tedrake, [*Underactuated Robotics*: Linear Quadratic Regulators](https://underactuated.mit.edu/lqr.html).
- A. De Luca and R. Mattone, [“Sensorless Robot Collision Detection and Hybrid Force/Motion Control,” ICRA 2005](https://www.diag.uniroma1.it/~labrob/pub/papers/ICRA05_CollDetect.pdf), DOI `10.1109/ROBOT.2005.1570247`.
- A. De Luca and R. Mattone, [“Actuator failure detection and isolation using generalized momenta,” ICRA 2003](https://doi.org/10.1109/ROBOT.2003.1241665).
- [MuJoCo computation documentation](https://mujoco.readthedocs.io/en/stable/computation/index.html) and [SciPy's discrete Riccati solver](https://docs.scipy.org/doc/scipy-1.15.3/reference/generated/scipy.linalg.solve_discrete_are.html).
- [Gazebo Harmonic documentation](https://gazebosim.org/docs/harmonic/comparison/) and [Leo Rover simulator/Mars assets](https://github.com/LeoRover/leo_simulator-ros2).

The momentum implementation is a project adaptation, not a claim of exact
equivalence to a cited paper. Project-specific controller/diagnostic code,
experiment orchestration, analysis, and drafting used **Codex/AI assistance**.
Students should identify actual individual contributions from their own work
records, understand the implementation, and cite third-party ideas/tools.
This document cannot assign personal authorship.

For further questions, start from the live pipeline in Sections 2–6. For a
numerical claim, identify the recording, clock, evaluation mask, policy, and
comparison denominator. If code or logs are missing, request them. Label a
suggested fix as a proposal until implemented and tested; do not silently
promote offline family-label improvements into better balance or tracking.
