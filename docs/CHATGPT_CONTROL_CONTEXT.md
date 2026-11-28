# Active spherical wheel: standalone context for another ChatGPT

**Revision 13 — 28 November 2026, through I-78.** The selected live controller
remains the original native N. I-78 starts from repository commit `9f7cfc8` and
tests managed path progress; no production/default promotion is made.
This is a study and discussion
document, not a claim that every retained experiment is part of the operating
controller. It is intended to remain useful without access to the repository.

The latest experiment, **I-78**, slows path progress around the unchanged LQR
and observer using current inferred geometry and nominal full-command pressure.
Desired position, velocity and acceleration remain coherent. Six fresh straight
runs complete; native arrays exactly replay the earlier baseline. Peak path
departures N→G are148.9→102.0,124.6→124.4,59.7→59.2 mm at−1.0/+0.3/+0.9 rad,
with0.985/0.600/0.005 s extra travel time. It fails two of82 fixed checks because
+0.3 peak speed and tilt increase. Keep G experimental. This is a reactive
policy, not a future contact predictor or a guarantee of command feasibility.
Section22 explains the test, retained failures and causal limits.

The preceding investigation, **I-77**, returns to the baseline. In a matched-prefix
straight −1 rad test, limiting only feedback joint-rate references during one
250 ms window reduces subsequent geometric peak error over 20.4–24 s from 148.9
to 22.0 mm. Removing only the worst 5 ms pulse does not help. This is a causal
diagnostic, not an adopted controller. The map has nearly parallel drive
directions even at the recorded pressure point. A separate figure-eight event
has actual rapid contact-normal changes, accurately reconstructed contact
wrench, and a rapidly moving normal-derived lean target with zero angular-rate
reference. Section21 gives the evidence and qualifications.

The preceding experiment, **I-76**, extends the same heading helper to straight,
90° arc and figure-eight routes, each at three starts: 18 corrected runs. All
complete. Heading RMS improves 8/9 pairs, but timed XY only3/9 and geometric
path distance4/9. The figure-eight is promising: every phase's geometric RMS
improves,39.87→15.54,27.91→22.22,32.73→17.21 mm; timed error improves 2/3.
Straight improves 1/3; arc 0/3. The screen fails42/207 control checks despite
686 consistency checks passing. Equal stopping totals 3 N /3 T conceal two lost
native successes and two different gains. A straight−1 T transient reaches
19.89° tilt/1.09 m/s/14.12 m/s². Keep T experimental; Section20 distinguishes
route-specific benefit from general improvement and retains setup/recovery history.

The preceding experiment, **I-75**, compares native N with trajectory-facing
heading T in six fresh S-path episodes. Both start aligned to the initial path
tangent, yaw argument 149.242843°, rather than the old 45°. All complete; all
212 setup checks pass, but 15/60 fixed control checks fail. Heading RMS N→T is
78.26→9.43°, 96.52→22.87°, 94.45→23.61°. Moving timed XY RMS nevertheless
worsens in all three phases: 19.66→33.82, 18.07→24.14, 20.08→27.01 mm. T ends
81.93/56.99/26.11 mm from goal and none passes original four-condition settling.
Pointing improves, with worse tracking. A +0.9 demand spike again comes through
rolling joint-speed references entering LQR, not a large direct heading torque.
Keep T experimental. This overlay retains zero LQR angular-rate references and
translation-only rolling allocation; full reference coordination is untested.
Section 19 gives the mechanism, exact rate distinction and limitations.

The preceding experiment, **I-74**, compares native N with native plus bounded
starting-heading hold H in six fresh S-path episodes. All complete and native
data reproduce I-73 exactly. H improves final stopping in two phases, but fails
12 of 66 fixed checks. At −1.0 rad moving XY RMS falls 47.31→24.38 mm and H
settles, ending 2.67° from its target; +0.3 settles near the goal but56.40° off
heading, and +0.9 ends53.46 mm and 91.42° off. A rare −1.0 unclipped request
spike is traced to enormous rolling joint-speed references entering LQR,
while the heading torque itself is small. Keep H experimental. Section18
separates the stopping benefit, heading authority limits and command regression.

The preceding experiment, **I-73**, runs nine fresh 45-second Gazebo/DART S-shaped
episodes: native N, guarded velocity A and geometry-plus-velocity C, at three
starting phases, with a ten-second final hold. Only rolling-reference routing
changes. All runs complete, but C fails 12 of 36 fixed control checks: tracking
improves at −1.0 rad, worsens at +0.3 rad and changes little at +0.9 rad. None
fully settles under the simultaneous position/speed/rotation/attitude criteria;
several return close to the goal but keep moving or rotating. Original heading
is free. Keep the original selected pipeline; neither A nor C is adopted.
Read Section 17 for the fresh motion evidence and its limited intervention scope.

The preceding experiment, I-72, chooses a physical point near the native observer's
equivalent point before applying the existing velocity/fused ranking. Its primary
accepted/fallback policy **C** makes **127** mistakes versus old policy A's
**124** and native N's **412**. On the same full primary rows, C improves point
RMS, p95 and maximum against N, but worsens RMS/p95 against A despite a smaller
maximum. It fails four of ten gates, including a small identity regression and
lower acceptance coverage, and is **not adopted**. Read Section 16 for the
spatial tradeoff. I-71's **STOP THIS HMM VERSION** decision remains in force;
I-72 does not restart HMM tuning. Its offline acceptance failure is not erased
by the later mixed driving comparison.
The I-68/I-69 force-boundary findings remain relevant qualifications on the
recorded pressure labels, but the user has **stopped the solver-investigation
direction**. Preserve that history without treating it as the next work item.

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
The temporal estimator remains an opt-in offline experiment. The geometry-first
selector is also opt-in: I-73 tests it only in rolling-reference generation,
while native support loads, gain routing, normal and support torque remain in
use. Neither is a particle filter or an adopted replacement for the support locator.

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

I-60–I-67 and I-70–I-72 reuse three I-58 **native** 44-second Gazebo/DART Mars figure-eight
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

`C` is introduced in I-72: accept that experiment's new geometry-first fused
family/physical point, otherwise keep N's family/equivalent point. It differs
from A, which retains the old I-60 proposal and acceptance. Neither policy is
the selected live controller's adopted replacement.

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
| [I-70: hybrid temporal modes](iterations/2026-11-27/temporal-contact-modes/README.md) | Implements a causal 21-mode rolling/slip, single/pair/unknown estimator. Primary mistakes A124→564; frame evidence305 and no-edge temporal control646. Matched point RMS improves but maximum worsens; six of sixteen gates fail. | Completed shadow experiment, not adopted. Section 14 separates identity errors, abstention, multi-contact and timing results. |
| [I-71: endpoint-pair stopping ablation](iterations/2026-11-27/endpoint-pair-ablation/README.md) | Keep nine singleton/unknown endpoint modes and preceding-interval pair edges, using frozen I-70 costs. Primary mistakes fall to425 but matching frame has303; every phase regresses. Four of eight continuation checks and seven of sixteen adoption gates fail. | Stop this HMM version without tuning. All471 true pair-label rows remain errors in the full loaded-set comparison; Section 15 gives the scope. |
| [I-72: geometry-first velocity selection](iterations/2026-11-27/geometry-first-contact/README.md) | Choose one physical candidate per family nearest native N's equivalent point, then reuse fixed I-60 ranking/acceptance. Primary C has127 errors versus A124; acceptance6848 versus 6902. Full-primary point RMS/p95/max improve against N, but RMS/p95 worsen against A despite a better maximum. Four of ten gates fail. | Mixed spatial benefit, configured selector rejected; no tuning or live adoption. Section 16 retains the complete comparisons. |
| [I-73: fresh S-path rolling-reference comparison](iterations/2026-11-27/contact-reference-control/README.md) | Nine new45s Gazebo/DART episodes, N/A/C ×three phases. Only rolling reference/feedforward records change. Allcomplete; C fails12/36controlchecks and noneof9fullysettles. −1.0tracking improves, +.3regresses, +.9littlechange. | No consistent motion gain; original pipeline retained. Section17 distinguishes positionreturn, motion and freeheading. |
| [I-74: bounded starting-heading hold](iterations/2026-11-27/heading-hold-control/README.md) | Six new matched N/H S episodes, exact native replay. H improves stopping in two phases; only −1.0 meets the additional heading tolerance. Twelve of 66 fixed checks fail, including command magnitude/rotation peaks and+.9tracking. | H remains experimental. Section18 explains configuration-dependent helper availability and the rolling-reference/LQR demand spike. |
| [I-75: trajectory-facing heading](iterations/2026-11-27/tangent-heading-control/README.md) | Six fresh aligned-start N/T episodes. Heading RMS improves in all three, but every moving XY RMS worsens; 15/60 fixed control checks fail. No T meets original four-condition settling. | T stays experimental. Section 19 distinguishes an improved heading objective from tracking, command feasibility and a fully coordinated yaw reference. |
| [I-76: heading across three routes](iterations/2026-11-27/tangent-heading-routes/README.md) | Eighteen corrected N/T runs all complete. Heading improves8/9, timedXY3/9, geometric path4/9; eight geometric RMS improvesall3, arcnone. Fixed screen fails42/207 checks. | T remains experimental. Section20 preserves a real eight path benefit, stopping/transient regressions, numerical endpoint correction, client recovery and exact replay. |

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

**Can we blame estimation lag?** Distinguish the recordings. The older three
episodes lack full substep force history; the identical-motion I-68 replay
adds it for the −1-rad phase. At 32.110 s that replay establishes contact
transfer inside the preceding 5 ms, rather than CPU delay. Filtering and
interval averaging still matter. I-70 tests an approximate mixed-interval
model, but its failed gates do not establish a general timing correction.

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

At I-67, the proposed next target was point selection within the existing bank,
with velocity and load-model assumptions examined explicitly. I-68 below now
provides matched substep records for one starting phase. Candidate expansion
and new weights remain proposals; no live control upgrade is adopted.

## 12. I-68: the observer interval and hidden force routing

One new **unchanged native LQR replay** reproduced I-58's −1-rad, yaw45° Mars
figure-eight over the full 44 seconds. Control and observer still update every
5 ms; an evaluation-only recorder copies state and all contact rows after each
1 ms physics solve. There is no intermediate command or observer call. All
59 old precision fields and all 8,801 control rows reproduce byte-for-byte.
This is stronger logging of identical motion, not improved tracking.

At time t, the observer's raw load describes the preceding `[t−5ms,t]`, while
the old contact summary described only the final 1 ms solve. For each solve,
map the reported force with the point Jacobian at its pre-solve pose. Average
native generalized loads over all five solves, then apply the observer's one
interval-midpoint rotation to its base angular rows. Repeating at post-solve
poses is a sensitivity control, not an accuracy-confidence interval.

On 8,000 primary rows, `1 <= t < 41 s`, joint-vector RMS discrepancy is
**0.0000866 N m** with this integrated reference versus **0.1163 N m** using
only the final solve. Post-pose mapping gives 0.00164 N m and still improves
on the final-solve comparison. All fixed windows pass both controls. The
observer is therefore closely consistent with this simulation's reported
force history. The reference does not export contact couples or independently
measure every solver constraint load; this is not a hardware accuracy claim.

Two events explain why that distinction matters:

- **32.110 s:** three shell-B positive-pressure samples precede two belt-B
  samples. The endpoint looks belt-only, but the observer's shell-B load
  `+0.016193 N m` matches the integrated shell contribution `+0.016228 N m`.
  This is temporal contact transfer within the observation interval. It does
  not establish CPU delay or simultaneous two-contact support.
- **35.205 s:** belt B has all the positive normal pressure, yet two admitted
  shell-B records carry tangential force of 26.53 N and 12.72 N with zero normal
  projection. Their integrated shell-B torque is `−0.206606 N m`, matching the
  observer's `−0.206627 N m`. Their positive depth retains them in native wrench
  sums, but normal-load weights exclude them from family loads/centroids/counts.
  Thus “belt-only” was a pressure label, not a statement that no shell force
  existed. The original observer did not invent this shell torque.

A separately sealed **posthoc** screen finds near-zero-normal, nonzero-tangent
records in 290 of the 8,000 primary intervals, including the good control.
The numerical detection rule is `abs(Fn) <= 1e-10*max(1,|F|)` with
`|Ft| > 1e-8 N`; it is not a learned physical uncertainty threshold. All selected
rows are admitted, and rejected forces are zero. Presence alone does not prove
poor tracking. Their zero-normal tangential behavior needs an installed solver/
contact-reporting audit before treating it as valid Coulomb friction or a proven
implementation bug. At I-68 the source inspection did not establish its internal cause; I-69 below
now connects the installed solver approximation to these specific records.

The late event also has actual-point rolling mismatch: pressure-weighted slip
RMS is 0.133 m/s before the solves and 0.139 m/s after. The post-solve values at
32.110 s and the good 7.875 s control are only 0.000281 and 0.000329 m/s. Thus
not every pre-solve contact velocity is persistent slip, but the later event
really limits the strict zero-slip model. Its intrinsic-normal discrepancy is
7.24 degrees; that does not identify a universally better normal-prior weight.
These slip diagnostics use positive-load solver contacts and omit zero-pressure
rows from their pressure weighting.

The old I-60–I-67 counts remain historical comparisons against **dominant
positive-normal-load labels**. They were not rerun or relabeled here; one phase
cannot establish corrected rates across three phases. A family that is “wrong”
against that pressure label can still be a body carrying reported tangential
force. The conditional bank-capacity results also remain offline existence
counts, not deployed improvements.

At I-68 the next target was the contact-force/normal boundary and evaluation
labels. I-69 below completed that diagnostic connection. The user subsequently
parked solver investigation; this paragraph is historical, not an instruction
to reopen it. LQR, momentum observer, geometry support locator, feedforward and
incoming preparation remain selected. No new selector or controller is deployed.

Fourteen recorder and thirteen numerical contracts pass; independent real-model
Jacobian checks agree near floating-point precision. The compact 14.13 MB input
package restores all 36 trace arrays and both ten-field precision bindings
byte-exactly on two NumPy versions. Its cached replay reproduces 58 analysis
arrays and scientific report values exactly. Full models/rollouts remain local.
The [I-68 record, comparison and four annotated events](iterations/2026-11-27/contact-interval-replay/README.md)
include the protocol, failed setup/analysis stages and portable reproduction.

## 13. I-69: source of the zero-normal tangential forces

The follow-up separates three levels of evidence: official implementation,
a controlled test of the installed library, and the actual original episode.
The contact solver first solves normal constraints, sets tangent impulse bounds
from those intermediate normal impulses, then continues solving the coupled
system. Later normal impulses can change while the earlier tangent allowances
remain fixed. This is a documented approximation in DART's ODE-style Dantzig
solver; a successful return does not guarantee the final dependent bounds.

For ideal pointwise friction, each tangent component should satisfy
`abs(lambda_t_i) <= mu_i*abs(lambda_n)`. A two-direction friction pyramid and
a circular cone differ, but both require zero tangential impulse at zero normal.
Here the installed solver can retain a sideways impulse even when its final
pressing impulse becomes zero.

A new standalone executable uses the existing installed library with one
registered positive-definite3x3 problem. Dantzig returns success with
`(normal,tangent1,tangent2)=(0,.7,0)`, satisfying its frozen absolute boxes but
violating the zero tangent allowance implied by the final normal. These are
normalized test units. Default PGS fails to converge on the same problem, so
switching to PGS is not an established correction.

Then a private diagnostic hook copies the original native solve inputs,
outputs and applied contact impulses while delegating the installed functions
unchanged. Replay the full −1-rad, yaw45° Mars figure-eight. All59 control
precision arrays and all 36 prior1ms trace arrays reproduce byte-for-byte.
Logging changes no motion, command, observer, plugin or solver method.

Across the 20 physics steps preceding the four fixed times, all 20 Dantzig calls
return success; PGS is unused. There are 140 applied contacts. Every applied
impulse matches its solver solution segment exactly; impulse/normal/basis/dt
reconstruct the reported force within 3.55e−15N. All140 contacts match exports,
with25 nonzero-force contacts unambiguously assigned to material families.

The previously confusing shellB records are directly tied to those solutions:

| Actual physics time | Applied `(normal,t1,t2)` impulse, N s | Tangent limits from final normal |
|---|---|---|
| 35.203s | `(0,.0201248655,−.0172934405)` | `(0,0)` |
| 35.205s | `(0,.0104874728,−.0072003873)` | `(0,0)` |

Their forces26.53N and 12.72N already exist in the applied contact solve and
match the old bridge records. Thus the specific zero-normal tangential-force
anomaly is localized to the installed friction approximation, and the observer
is responding to loads that the simulation really applies. This is stronger
than an unexplained force/normal pairing discrepancy.

Twelve of 140 selected applied contacts violate final-dependent tangent-component
bounds. Two occur in the good7.875s control; none occurs in the 32.110s interval,
where temporal transfer remains the explanation. Five occur at each later
anchor35.055/35.205s. These selected counts are not trajectory-wide frequencies,
and the good-control occurrence prevents a claim that this mechanism alone
causes poor tracking. The specific large trajectory deviation remains incompletely
explained.

At I-69 a possible follow-up was to validate a contact-solver/model correction
on saved problems before more driving. The user stopped that direction; no
corrected solver is implemented or adopted. Changing only exported force values
would leave actual simulated dynamics and the observer inconsistent, so the
finding remains an interpretation limit. The current work instead retained
the baseline and tested the separate shadow estimator described next.

The [I-69 record and explanation PDF](iterations/2026-11-27/contact-force-boundary/README.md)
include source/package provenance, controlled/native evidence, independent
checks and cached numerical replay. Full new rollouts remain local; the compact
package retains the solver problems and 20 selected contact contexts without
claiming to restore the entire simulation.

## 14. I-70: an implemented hybrid estimator, not an adopted replacement

This experiment runs only on the three saved native episodes, processing all
26,403 frames. It changes no robot command and generates no new physics. The
question is whether remembering plausible contact modes improves decisions
over time, rather than selecting one explanation independently every sample.
All inference outputs were sealed before evaluation labels were opened.

The estimator tracks **21 modes**. Four single-family support sets and six
two-family sets each have a rolling and a slipping version; an additional mode
means unknown. For example, “shell A rolling” and “shell A plus belt B slipping”
are different hypotheses. A pair permits two separate contact forces, each
carrying at least 10% of their combined normal load. Each point supplies force
only; no independent contact couple is fitted. It cannot represent three
families or arbitrary multiple points within one family. These modeled sets
must not be confused with proof of simultaneous physical contact.

Inputs are measured-state primitives already allowed in simulation: pose and
velocity, previous applied commands, the raw nine-component observer estimate
for the preceding interval, the filtered-wrench-derived normal hint, and known
robot geometry. Candidate point velocities include prescribed belt surface
motion. No solver contacts, true points/load shares, terrain map, reference
trajectory or future samples enter inference. Exact simulated body state is
still permitted; there is no independent new contact sensor.

The original 102-point bank is retained. Each family keeps at most two points:
one favored by geometry and one by geometry plus rolling-motion consistency,
with duplicates removed. At a complete single/pair point combination, a
nonnegative least-squares calculation fits separate forces inside assumed
friction pyramids to explain all nine observer components. Geometry and
tangential-motion terms then contribute to a profile score. Rolling uses a
0.02 m/s motion scale; slipping uses the broader 0.15 m/s scale. Other fixed
assumptions include a 10-degree normal scale, friction coefficient 0.7, force
scale 0.5 N and moment/joint scales 0.03 N m. These are engineering assumptions,
not parameters learned from this evaluation.

A **hidden Markov model (HMM)** carries past mode beliefs forward, preferring
some continuity while allowing switches. It uses only current/past data, with
no backward smoothing or future terrain prediction. The contribution of a
5 ms observation is reduced using an assumed 20 ms correlation time, since
the available cues share state and observer history. A special transition
calculation allows the preceding interval's force to come from both the
exiting and incoming family while scoring rolling/slip motion only at the
incoming endpoint. This addresses “old load in a recent interval, new support
now” explicitly; it remains an endpoint-geometry approximation to that history.
Before reporting a support set, rolling and slipping beliefs for that same set
are added together, giving eleven support choices including unknown. Splitting
belief between two motion hypotheses does not itself discard that support set.

For each mode, alternative complete-fit scores are combined without rewarding
a mode merely for having more candidates. Reported physical points/forces
come from one complete fit, rather than mixing independently chosen components.
Its point covariance describes only the small conditional shortlist. Neither
that covariance nor the normalized HMM numbers establish full-surface
uncertainty, unique forces or calibrated physical probabilities. This method
is **not a particle filter**: it does not maintain and resample a cloud of
continuous contact particles. It also continues to use the momentum observer.

### Completed comparison and decision

On the same **23,423 dominant-positive-pressure rows**, unknown and pair outputs
count as mistakes when the reference label is a dominant singleton. The first
numeric column below includes every such mistake, not only wrong families:

| Method | Total primary mistakes | Wrong singleton | Unknown | Pair output |
| --- | ---: | ---: | ---: | ---: |
| Existing accepted/fallback policy A | 124 | 124 | 0 | 0 |
| New frame evidence, without history | 305 | 62 | 204 | 39 |
| Temporal model without transition-specific evidence | 646 | 147 | 241 | 258 |
| Primary transition-aware HMM | **564** | **140** | **223** | **201** |

Thus remembering modes does not automatically improve the primary decision.
Transition-specific evidence helps relative to the no-edge temporal control,
but this HMM is worse than both A and its own frame-only control on that metric.
It fails **six of sixteen gates**: pooled primary error reduction, each of the
three starting-phase error comparisons, high-confidence wrong count, and
maximum point distance. Even at reported confidence at least 0.8, it makes
150 **nonunknown** mistakes: 53 wrong singletons and 97 false pairs, versus
A's 124 total. The registered covered-confidence metric excludes unknown;
156 confident unknown outputs are reported separately and remain primary
mistakes. **Do not adopt this version.**

Other metrics show limited benefits and must retain their denominators:

- On **23,894 loaded rows**, the full active-family-set comparison improves
  from A's 595 mistakes to 582: only **13 additional correct rows**. Set Brier
  score improves **0.049803→0.037896**. Brier measures probability-vector error
  against recorded labels; a lower score can coexist with worse dominant-mode
  decisions and does not prove calibration outside these episodes.
- On **22,999 exactly matched finite-singleton rows**, point RMS improves
  **2.54550→2.22954 mm**, while maximum distance worsens
  **26.90764→29.45375 mm**. HMM finite-singleton coverage is **98.19%** of primary
  rows. Wrong-family point errors are retained; missing unknown/pair points
  are reported through coverage, not assigned zero distance. These matched
  maxima are not A's maximum over every original primary row.
- All **71 defined pressure transitions** are acquired. HMM median absolute
  latency is 5 ms and p95 is **40 ms**, versus A's **20 ms** p95. HMM false support
  changes fall from frame evidence's 158 to 97, but A has only 30. Much of that
  frame-to-HMM reduction removes unknown transitions; wrong singleton toggles
  increase from 18 to 27. “Smoother” therefore needs a specified comparison.

A separately sealed **post-result diagnostic** explains the tradeoff without
changing parameters or gates. The HMM recognizes **453 of 471** loaded pair
rows, but relative to A it gains only **26** dominant-singleton decisions and
loses **466**. Those pair successes almost offset the singleton losses in the
full-set metric; they do not rescue the failed primary result.

The +0.3-rad episode contains **197 of 201** false pair outputs. Its longest
constant false-pair spell spans **15.725–15.765 s**, nine 5 ms samples counted
as **45 ms** of sampled disagreement. The HMM retains shell A+B while pressure
supports shell B; frame evidence already selects shell B at both ends. This
is consistent with temporal retention contributing at that event, without
isolating a dwell parameter or proving one cause for all mistakes.

The labels remain sampled positive normal pressure from the final 1 ms solve,
not every force-bearing body or first physical touch. I-68/I-69's qualification
is retained without relabeling old episodes or restarting solver work.
These are retrospective results on three already inspected starts, not unseen
terrain validation or evidence of improved balance/tracking. Independent
recurrence and force-QP checks support numerical correctness; they do not
convert failed scientific gates into acceptance.

After I-70, the decision was to keep the live baseline. An existing-log review
could first identify which measurement terms and temporal
preferences produce the wrong, unknown and pair outputs. Possible revisions
include simpler confidence-aware handoffs, better justified transition models,
or different contact/slip likelihoods. A particle filter could retain richer
point alternatives, but would still inherit measurement-model and observability
limits. None is a proven next improvement, and no new solver or hardware work
is implied by these options. The user then authorized only the bounded stopping
ablation in Section 15; those earlier options are not adopted or automatically
continued after its stopping decision.

The [I-70 record, results and reproduction](iterations/2026-11-27/temporal-contact-modes/README.md)
preserve the protocol, rejected candidate, source snapshots and evidence.
The opt-in implementations are [contact profile models](../src/spherical_wheel/shadow_contact_models.py)
and [causal temporal filtering](../src/spherical_wheel/temporal_contact_filter.py).

## 15. I-71: the endpoint-pair ablation and stopping decision

The user approved one bounded experiment to distinguish a simultaneous
two-family endpoint estimate from two families contributing during the previous
5 ms interval. It replays the same three saved episodes. There is no new
driving, force fitting, nonnegative least-squares solve, geometry expansion,
noise-scale search, controller change or solver investigation.

The restricted estimator has **nine endpoint modes**: four singleton families,
each rolling/slipping, plus unknown. It cannot emit a simultaneous endpoint pair.
The original two-family **transition-interval edges remain available** when
switching between singleton endpoints. Thus a recent interval may contain
exiting shell A and incoming belt A force even when the reported current
endpoint is only belt A. This does not make that force history directly measured;
the same endpoint-geometry approximation and likelihood assumptions remain.

All original node/edge costs and complete-fit candidate points are frozen.
Removing pair modes conditions the prior onto allowed endpoints. Each allowed
mode keeps its original diagonal holding probability exactly; remaining
off-diagonal jump probabilities are redistributed proportionally among allowed
destinations. Dwell-time parameters are not tuned. The smaller graph changes
expressivity, prior odds and jump destinations, so this is not an isolated
dwell-time experiment. New beliefs can choose different existing complete fits;
evaluation therefore copies fresh I-71 point outputs instead of reusing I-70's
selected HMM points.

The exact I-70 masks, metrics and sixteen adoption gates are reused. A separate
continuation decision requires the primary restricted edge-HMM to make strictly
fewer pooled dominant-singleton mistakes than its matching restricted frame
control, with no phase regression; missed transitions, reverted attempts and
false support changes must not increase, and absolute latency p95 must be at
most 50 ms. These eight checks are a stopping rule, not relaxed adoption gates.
All inference outputs are sealed before evaluation truth is opened.

On the unchanged **23,423 dominant-positive-pressure rows**:

| Restricted I-71 method | Total primary mistakes | Wrong singleton | Unknown | Pair output |
| --- | ---: | ---: | ---: | ---: |
| Matching frame evidence | 303 | 62 | 241 | 0 |
| No-edge temporal control | 492 | 166 | 326 | 0 |
| Primary transition-edge HMM | **425** | **155** | **270** | **0** |

For comparison, frozen N/F/A still have 412/109/124 mistakes, and I-70's primary
HMM has 564. Removing endpoint pairs reduces the HMM total by 139, mainly in
the +0.3-rad episode. It does not make history beneficial against its own frame
control:

| Starting phase | Restricted frame mistakes | Restricted primary HMM mistakes |
| --- | ---: | ---: |
| -1.0 rad | 56 | 83 |
| +0.3 rad | 110 | 160 |
| +0.9 rad | 137 | 182 |

The estimator fails **four of eight continuation checks**: pooled improvement
and all three phase nonregression comparisons. It also fails **seven of sixteen
adoption gates**. The recorded decision is **STOP THIS HMM VERSION**. There is
no post-result dwell/weight tuning or live adoption. This stops this candidate,
not the original momentum observer or gain-scheduled LQR.

All **471 true pair-label rows** remain in the full **23,894 loaded-row** domain.
The restricted model deliberately cannot match them; it has **896** full-set
mistakes, equal to 425 dominant-singleton mistakes plus those 471 pair misses,
versus A's 595. Set Brier is **0.063652**, worse than A's **0.049803**. In the
unchanged metric schema, `representable` and `unsupported_truth_count` refer to
the original eleven-support-set evaluation bank: singleton/pair categories are
representable, while three-or-more-family sets would be out of that bank.
The restricted model's 471 pair misses are reported separately, not removed
from the denominator or relabeled as evaluation-unsupported truth.

Unknown remains a primary mistake and never provides confident coverage. At
reported support confidence at least 0.8, the restricted HMM covers **97.78%**
of primary rows and makes **76 nonunknown mistakes**. Passing that particular
confidence check does not rescue its 270 unknown mistakes or establish calibrated
physical probabilities.

Partial point and timing results retain their comparison conditions:

- On **23,153 exactly matched finite-singleton timestamps**, current A-to-HMM
  point RMS improves **2.61048→2.32530 mm**, but maximum distance worsens
  **26.90764→29.45375 mm**. HMM point coverage is **98.847%**; A is available
  on every matched row. Wrong-family singleton errors remain included. I-70's
  point comparison used a different set of 22,999 timestamps, so its RMS is
  not a directly matched before/after result for I-71.
- All **71 defined pressure transitions** match, with zero misses and absolute
  latency p95 **40 ms**. The HMM has ten reverted attempts and 85 false support
  changes versus matching frame evidence's seventeen and 156. A still has
  thirty false changes and 20 ms p95. This is a limited comparison benefit,
  not evidence of improved physical tracking or an overall timing solution.

The result supports a narrow conclusion: removing endpoint pair states reduces
I-70's dominant-mode errors, but is insufficient. Matching frame
errors barely change, 305 to 303, and temporal inference still regresses in every
phase. The combined observation model and temporal behavior remain inadequate
for the registered goals. This experiment does not isolate force likelihood
from geometry and velocity terms, prove all temporal estimation methods fail,
or show that a particle filter would solve the ambiguity.

The [I-71 record and reproduction](iterations/2026-11-27/endpoint-pair-ablation/README.md)
retain the unchanged costs, restricted graph, refreshed point outputs, failed
gates and stopping decision. These are retrospective pressure-label comparisons,
not unseen terrain validation, complete force-routing truth or hardware evidence.
The live LQR/observer/support geometry/feedforward/short preparation pipeline
remains selected. Solver work stays parked; no hardware or camera additions are
part of this completed ablation.

## 16. I-72: geometry-first point selection, with a mixed spatial benefit

After stopping the HMM version, the user authorized one simpler selector. I-60's
measured-velocity evidence helped identify families, but freely selecting a point
to fit that evidence produced point-distance peaks. I-67 showed nearby usable
physical points already existed in the bank. I-72 asks whether geometry can
choose the representative point first, leaving velocity to compare families.

For each of the four families, choose the original eligible physical candidate
closest in squared 3D distance to the **native observer equivalent point**.
Equal-distance ties use the smallest original candidate ID. This anchor comes
from the existing momentum-observer/support-geometry result, not a true contact,
Nalt or terrain map. Native family identity is not provided to point selection
or ranking; it is used only by the separate native fallback. The equivalent point
can itself be biased or interior during multiple contacts, so nearest-to-anchor
does not mean nearest-to-truth.

Then call the exact frozen I-60 ranker on those representatives, using all old
normal, moment and measured-velocity terms, scales and acceptance rules. The
primary score is fused; VG, normal plus velocity, is secondary. A physical point
is selected as a complete candidate, rather than choosing score terms from
different points. There is no HMM, new force fit, NNLS solve, threshold search,
new candidate geometry or physics rollout. The small NumPy-only
[geometry-first module](../src/spherical_wheel/geometry_first_contact.py) is
retained as opt-in research code and is never imported by the live controller.

The primary policy **C** accepts the new fused family and its physical point
only when the unchanged I-60 rules accept it. Otherwise it retains **both** N's
original family and N's equivalent point. It does not move N's point onto another
body by relabeling it, substitute a physical Nalt point, or reuse an old HMM's
point output. A same-family accepted update still uses the newly selected
physical point. All three inference outputs are sealed before evaluation truth
is read.

On the same **23,423 dominant-positive-pressure primary rows**:

| Policy | Family mistakes |
| --- | ---: |
| Native N | 412 |
| Old raw fused F | 109 |
| Old accepted/fallback A | 124 |
| New geometry-first raw fused | 118 |
| New geometry-first raw VG, secondary | 117 |
| Primary C: new fused accepted, else N | **127** |
| New VG accepted, else N, secondary | 126 |

C's phase counts are **14/67/46** at starts -1.0/+0.3/+0.9 rad; A's are
**14/64/46**. C gains six correct decisions and loses nine relative to A.
It preserves **286 of 294** old useful corrections, losing eight, and repairs
**one of six** old harmful overrides and one of eighteen old accepted-wrong
rows. New accepted fused proposals contain sixteen wrong-family rows out of
6,848, versus N's 301 errors on exactly those rows. These subset counts do not
replace C's full-domain result or establish calibrated confidence.

New acceptance drops by **54**, from 6,902 to **6,848**. The ambiguity/motion
denominator stays fixed to the old I-60 eligibility rules: original at least two
valid families, measured excitation at least 0.02 m/s and primary truth. On those
**7,970 rows**, acceptance falls **86.5997%→85.9222%**; on all primary rows it
falls **29.4668%→29.2362%**. Both acceptance gate lines fail, but they share a
numerator and are explicitly redundant constraints, not independent evidence.
No new domain is selected after seeing which representatives are accepted.

The full-primary point result is the useful spatial finding. Every policy below
has a finite point on exactly the same **23,423 rows**, with wrong-family points
and native fallback rows retained:

| Policy | RMS, mm | p95, mm | Maximum, mm |
| --- | ---: | ---: | ---: |
| Native N | 3.136094 | 7.388477 | 26.907642 |
| Old A | 2.759063 | 5.797002 | 29.842859 |
| Primary C | **2.909967** | **6.342826** | **25.565903** |

C passes all registered point gates against N, including **100% coverage**.
Against A, however, it has worse RMS and p95 while improving the maximum. Thus
“better point estimation” must identify the statistic and comparison. These
distances use the raw sampled **dominant-family pressure centroid**, not I-60's
raw equivalent-point truth target; do not substitute the older approximately
3.134 mm native RMS from that other target. Missing C on a fixed N reference
row would fail comparison rather than shrink the point denominator.

At the fixed bad **35.205 s** event, C still assigns shell B while pressure
supports belt B. Its point distance falls from A's 29.84286 to **25.56590 mm**,
but N is **15.15505 mm**. The smaller peak does not repair identity. At the
repaired old harmful row, **+0.9 rad, 12.555 s**, raw new fused still chooses
shell A instead of pressure-labeled belt A despite a point just **0.22978 mm**
from the reference. The unchanged acceptance rule abstains, so C correctly
retains native belt A and its 0.54945 mm equivalent-point distance. A very close
point can still identify the wrong material surface.

All **471 mixed-pressure pair rows** remain in the full **23,894 loaded-row**
metric. This single-family selector cannot match them: C has **598** full-set
mistakes, equal to 127 primary mistakes plus those 471 pair misses, versus
A's 595. It is not a general multicontact estimator. The five fixed anchors
and every old harmful row are retained in the evidence. Descriptive timing
matches all 71 pressure transitions, with zero misses, p95 absolute latency
20 ms, 22 reverted attempts and thirty false switches; A has twenty reverted
attempts and thirty false switches. Timing is not a new acceptance gate.

The configured selector fails **four of ten gates**: pooled identity, the
+0.3-rad identity comparison and the two redundant acceptance-coverage checks.
The decision is to **reject this fixed selector without tuning or secondary
winner substitution**. The point findings are retained as progress; they do
not override the registered decision. Passing all ten would only have qualified
a later validation proposal, not live adoption.

There are **39 selected new component contract tests**: sixteen core, six
consumer, four input and thirteen evaluation tests. The thirteen include the
earlier twelve evaluation contracts plus the final consumer-alias checks;
stage counts and nested subchecks must not be added. Frozen I-70 helper tests
are reused evidence, not new tests or the full repository suite. Reported
isolated replay reproduces 237 arrays across all 26,403 saved frames; this is
numerical reproducibility, not additional physical trials. Independent checks
are documented separately in the iteration evidence.

Four separate independent-verifier contracts also pass, giving **43 new focused
contract functions** in total. Independent choices, masks, IDs, flags, physical
points and fallback match all 79 fields per case; the largest numeric difference
is 2.22e−16. Its initial reference used infinity for an unavailable distance
where the production API uses NaN. Both stages are retained; correcting that
diagnostic sentinel changes no selector, score, threshold or scientific output.

The [I-72 record and reproduction](iterations/2026-11-27/geometry-first-contact/README.md)
retain both the geometry gains and the failed identity/coverage conditions.
These are three previously inspected trajectories, with last-1ms pressure labels
sampled every 5 ms. There is no demonstrated new balance/tracking improvement,
unseen validation or independent sensing. The selected live LQR, momentum
observer, support geometry, feedforward and short preparation remain unchanged.
I-71's HMM stopping decision remains in force; solver investigation is parked,
and this result adds no camera, hardware or new controller direction.

## 17. I-73: fresh S-path motion, with mixed effects and incomplete settling

The user approved an S-shaped or earlier figure-eight route. I-73 deliberately
uses a **new S** with three seconds initial hold, 32 seconds of smooth quintic
progress, and ten seconds at the final fixed goal. Geometry is
`origin + R(113°) [1.2u, 0.14 sin(2πu)]` metres. Start origin is
`[−2.4, 10.0]` m; initial robot yaw is 45°. Requested initial ground-relative
phases are −1.0, +0.3 and +0.9 rad; the common placement routine adjusts the
literal shaft angle for the local normal. No per-method reference reanchor,
gain tuning, threshold tuning or reset occurs after placement.

The original LQR, momentum observer, native support-force/load/normal estimates,
gain routing, holding torque, 120 ms incoming preparation, fade and final caps
remain. N uses native inferred records. A uses the original I-60 accepted fused
physical point/family, otherwise exact native fallback. C uses the frozen I-72
geometry-first fused point/family, otherwise native fallback. The selected
record contains geometry only, **no invented force or load shares**. It replaces
only the argument to the stateless rolling least-squares calculation.

This intervention changes joint velocity references **and** rolling belt
feedforward. Joint references enter LQR feedback errors, so it is not a pure
additive feedforward test. Native load/gain routing remains a possible source
of limitation; this is not a full estimator replacement. At exactly zero
desired velocity, the original branch bypasses rolling selection. Final hold
therefore tests arrival and retained history, not new contact-selector commands.

All nine fresh Gazebo/DART runs complete 45 seconds with no declared physical
failure. The same flat gain-cache bytes, five-millisecond feedback, one-millisecond
physics, and `[1.5, .6, .6 N m, .3, .3 m/s]` caps are retained. The method sees
exact simulated q/v and previous actual held command, plus observer outputs and
known robot geometry. Terrain truth is used for offline site choice/common
balanced placement and evaluation, never read during the guarded command.

| Requested start phase | Native N moving timed XY RMS | Velocity A | Geometry + velocity C |
|---|---:|---:|---:|
| −1.0 rad | 47.31 mm | 44.39 mm | 45.38 mm |
| +0.3 rad | 22.85 mm | 27.05 mm | 27.53 mm |
| +0.9 rad | 23.37 mm | 23.53 mm | 23.47 mm |

C improves −1.0 tracking, regresses at +0.3, and changes little at +0.9.
At −1.0 its late `[29,45]` s world-vertical angular speed peak grows from
0.1515 to 0.9714 rad/s. At +0.3 its final position error grows 7.02→28.42 mm;
at +0.9 it is 44.12 versus 41.41 mm. No method has a uniform win.
Native −1.0 has the largest moving RMS on this S, unlike some earlier favorable
figure-eight results. Phase ranking depends on the tested task; this experiment
changes both route and site and cannot isolate which caused the ranking change.

The strict settling condition requires all of the following at **every actual
state sample in final `[43,45]` s**: distance from the fixed goal ≤20 mm,
XY speed ≤0.02 m/s, absolute physical world-vertical angular speed ≤0.05 rad/s,
and angle between actual/reference up vectors ≤2°. The whole ten-second hold
is also searched for a simultaneous sustained two-second entry and later exit.
None of the nine achieves either settling measure. All nine exceed the rotation
criterion. The selected controller has **free heading**, so residual yaw is not
proof of a fall or a bad lean reference.

Do not confuse this with failure to return. At −1.0, final position errors
N/A/C are **4.86/1.93/1.79 mm**, but all retain excess translational speed and
rotation. At +0.3, N and A meet the other three final-window criteria and fail
only rotation. Position, balance, yaw and velocity are separate observations.
World-vertical angular speed is `R(q) ω_body`; it is not an Euler yaw derivative.
Absolute body tilt and body-to-reference up-vector error are also different.

All 101 setup/initial-state/reference contracts pass. C has **1.040/1.525/0.100 s**
of direct reference intervention, meeting the predeclared minimum; +0.9 exposure
is especially sparse. This compares native/selected rolling references on the
same current state, not a complete native command after preparation/clipping.
Twelve of 36 primary C checks fail, including all three settling checks and late
rotation regressions, with additional +0.3 tracking/attitude regressions.

The site-selection history matters. An original proposal and two fixed 25-site
grids fail the initial 8° maximum-facet screen. Before physics, that map-only
screen is explicitly revised to 12°, selecting the already inspected least-steep
site: 11.288° corridor maximum, 75.69 mm relief and at least 1 m projected rock/
lava clearance. It is **moderate faceted Mars ground**, not flat or a passed
8° screen. All failed proposals remain in the record; no controller outcome
was used to pick the site or relax the screen.

Verification includes eighteen adapter/runner contract functions and exact native
moving/stationary command/history identity; four unique route contracts; limited
feature/ranker parity on 39 old states; and 22 final evaluation contracts
(inclusive of the earlier 21). A verifier source/input-map lookup is corrected
before the first outcome evaluation. Media previews initially fail because
native saved rollouts encode float32 while precision files retain float64;
all 36 cast checks match, and corrected videos render the full-precision
recorded poses. These setup stages, the fixed source/gate seals, all nine cases,
comparison plots and unsuccessful decisions are preserved. Counts are scoped,
not the full repository suite.

**Decision:** keep the original selected controller/observer/feedforward and
retain A/C as experimental tools. Do not promote secondary A after seeing the
outcomes or reverse I-72's failed estimator screen. One deterministic new S
per method/phase is not general multi-contact validation or a statistical
replication study. The experiment does not establish the complete physical
cause of every correction. It shows that this rolling-contact change alone
has not produced a robust gain across the tested configurations.

The [I-73 record, PDF, full-hold videos and reproduction](iterations/2026-11-27/contact-reference-control/README.md)
contain the evidence. Solver investigation, HMM tuning and camera work remain
outside the selected next step; no hardware result is claimed.

## 18. I-74: heading hold helps some stops but exposes important limits

The user explicitly approved baseline versus baseline plus bounded heading
hold on the same three S-path starts. This tests an **existing option**, not
a new controller family or estimator. Native N retains the selected defaults.
H sets `heading_reference='start'`, `heading_actuation='shells'`, heading gain
0.2 N m/rad, rate damping0.06 N m s/rad, helper motor-torque cap0.15 N m and
horizontal reaction budget0.05 N m. The actual initial yaw is latched once and
is the target throughout the entire45 s, including motion and final hold.
Heading error used by the helper is clamped to±0.5 rad. Raw pointing error
is not clamped in evaluation or on-screen displays.

All other selected components remain: original flat LQR/gain bytes, momentum
observer and geometry support locator, inferred contact loads/normal/gain
routing, rolling and holding feedforward,120 ms incoming preparation, fade,
and final caps `[1.5,.6,.6 N m,.3,.3 m/s]`. Exact I-73 S/site/timing are reused;
no new terrain search, simulator-force control input, gain design or contact
selector is added. Six fresh Gazebo/DART runs complete; all 95 setup checks pass.
Each native recording reproduces46 shared original numerical arrays byte-exactly.

| Requested phase | Moving XY RMS N→H mm | Final XY error N→H mm | H final absolute heading error |
|---|---:|---:|---:|
| −1.0 rad | 47.31→24.38 | 4.863→0.155 | 2.67° |
| +0.3 rad | 22.85→24.00 | 7.018→2.868 | 56.40° |
| +0.9 rad | 23.37→33.69 | 41.410→53.460 | 91.42° |

H−1.0 and H+0.3 pass the final two-second simultaneous position/speed/world-z
rotation/up-vector attitude criteria. Their first sustained two-second entries
start at 35.480 and 35.455 s. Only−1.0 also meets the added≤10° heading tolerance.
Thus **stopping near the goal and holding the correct heading are separate
achievements**. H+0.9 is nearly motionless at the wrong goal offset/heading.
The three native runs retain their original incomplete settling.

Heading-error RMS over 3–45 s, N→H, is 47.09→46.55°,116.24→34.09° and
104.04→82.12°. Late physical absolute rotation over 29–45 s falls
68.88→65.79°,202.21→65.52° and 356.28→37.71°. This integral uses
`∫|[R(q) ω_body]_z| dt` with actual timestamps. It is not an Euler-yaw derivative,
nor does it guarantee a smaller instantaneous rotation peak. Wrapped pointing
error also cannot reveal all extra full turns; unwrapped yaw is shown separately.

The fixed screen requires20% improvements in both heading metrics at every
phase and nonregression in tracking, lean/tilt, speed,50 ms acceleration,
rotation peaks, saturation fractions, normalized request excess and command
steps. Twelve of 66 checks fail. −1.0 does not achieve either20% heading benefit,
and worsens peak absolute tilt/late rotation speed/unclipped request excess.
+0.3 improves total rotation but regresses lean/rotation-speed/command-step
peaks. +0.9 improves heading metrics but worsens tracking. All actual applied
inputs respect their caps. The four/five-condition settling measures are
descriptive and cannot replace the predeclared all-phase comparison.

Why can the helper stop acting while heading remains wrong? It spins a
sufficiently unloaded shell so the motor's reaction has a world-vertical
component. Its authority changes with carrier phase and inferred support.
At both 35 and 45 s in H+0.9, neither estimated shell is ≤5% of total load, and
the shell-axis vertical lever magnitude is 0.0653, below the existing0.12 cutoff.
The helper therefore commands zero despite approximately −91.4° raw heading
error. This is a concrete limit of **this helper's policy**, not proof the robot
has no possible yaw strategy in that configuration. At +0.3 the helper stays
eligible and commands about 0.05074 N m to each shell at 45 s despite −56.4° error;
not every offset is caused by that eligibility switch. The full physical
reasons need a separate matched investigation.

A bounded posthoc command ledger finds the large −1.0 H demand at 16.670 s.
The recorded rolling joint-speed references are about −6555/−23638 rad/s for
shaft/shell A, versus actual+.322/−6.999 rad/s. LQR joint-velocity error terms
produce roughly−1498/−1893 N m. Total request is approximately
`[−1498.545,−1892.619,−61.216,0,0]`; final applied input is
`[−1.5,−.6,−.6,0,0]`. Heading's own contribution is only −0.070819 N m on shell B.
Rolling feedforward/preparation/fade deltas are zero in that row. Therefore
the enormous algebraic demand enters through **rolling joint-speed references
and LQR velocity feedback**, not direct heading torque. The huge reference is
one 5 ms row in the fixed context; ordinary references return at 16.675 s.

Clipping fraction alone hides that severity: −1.0 N/H clip on0.556%/0.733% of
held intervals, yet normalized request-excess RMS grows0.1929→34.8845. The
separate magnitude guard catches it. The giant values are **requests**, not
actual motor torques. Ledger/block reconstruction residuals are ≤2.274e−13.
Contact-Jacobian/authority conditioning was not logged; a near-singular rolling
map is a hypothesis, not a proved physical explanation from this audit.

Twelve focused runner/telemetry functions pass, including exact instrumented
versus original N/H commands, diagnostics and history. Twenty-three final
evaluation functions pass, inclusive of an earlier 21-stage execution. Existing
media/package contracts are reused; one new heading-target/wrap media check
is added. Source review corrects an evaluator field alias and preserves the
raw-versus-used-error media distinction before outcome/pose reads. These are
not physical failures or full-repository-suite counts. The posthoc algebraic
identities are checks of saved equations, not additional unit tests.

**Decision:** keep original N selected and H as an experimental option. The
improved stopping is real progress; it does not establish robust all-phase
heading regulation. There is no post-result parameter sweep, default promotion,
new estimator, solver change, camera or hardware result. All full-hold videos,
positive results, regressions, command ledgers and reproduction inputs are in
the [I-74 record](iterations/2026-11-27/heading-hold-control/README.md).

The user subsequently clarifies the intended heading objective: **face the
direction of desired trajectory travel**, rather than preserve starting yaw.
I-74 tested the latter; it is not evidence of tangent-heading performance.
The proposed moving target is `psi_d = atan2(vy_desired, vx_desired)`, using desired motion,
not the measured drift direction. Keep the last valid direction at a stop,
unwrap/filter/bound target changes and include a desired turn-rate reference.
The current helper damps absolute world-z rate; it does not yet track a
trajectory yaw-rate reference. A reversal needs a declared choice between
turning around and driving backward. Current S initial tangent is149.24°,
versus initial body yaw45°, so fair startup alignment is a real design choice.
At that clarification, implementation and testing remained proposed. The later
authorized I-75 below tests the S-path tangent with aligned startup and a
declared stop policy; it does not change the I-74 result.

For presentation terminology, **holonomic constraints** are configuration
relations such as `f(q,t)=0`. Nonholonomic rolling restrictions constrain
velocities and cannot in general be integrated into such position relations.
The mobile-robot shorthand **holonomic/omnidirectional base** concerns available
chassis velocities: independent planar translation and yaw rather than just
reaching arbitrary destinations over time. These are different description
levels. A path-facing heading target does not impose a new physical constraint
or require a holonomic base; a nonholonomic car also faces its path tangent.
The author's [Modern Robotics §2.4](https://modernrobotics.northwestern.edu/nu-gm-book-resource/2-4-configuration-and-velocity-constraints/)
and [§13.1](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-1-wheeled-mobile-robots/)
explain the distinction.

For this robot, say **designed for omnidirectional translation, with
configuration-dependent heading authority**. Current rolling allocation solves
XY contact equations and handles yaw separately. Neither moving sideways nor
the I-74 helper failure establishes a complete holonomic/nonholonomic
classification. A formal claim needs the full rolling/contact constraints and
coupled kinematic/dynamic authority, including balance and internal motion.
Do not equate a weak heading controller with a mechanical impossibility, or
count five actuator commands as proof of three independently controlled chassis
motions. The selected baseline, prior results and parked work remain unchanged.

## 19. I-75: face the path tangent, with worse position tracking

**Question and intervention.** The user wants the platform to point in the
planned direction of travel rather than hold initial yaw. Six new Gazebo/DART
runs compare native N and a result-owned T adapter at phases −1.0, +0.3 and
+0.9 rad. Same fixed moderate Mars site, S path, 5 ms control /1 ms physics and
45 s episode: 3 s hold, 32 s motion, 10 s final hold. Original gains/LQR,
momentum observer, support/load/normal/gain estimation, rolling/holding
feedforward, preparation and final `[1.5,.6,.6 N m,.3,.3 m/s]` caps remain.
No camera, new contact estimator, solver or dynamic contact forecast enters.

**Fair initialization.** Both N and T use yaw argument 149.242843°, the initial
geometric tangent. Paired q/v are exactly equal; all actual initial heading
errors are approximately .04292°. These native data are new matched baselines,
not numerical replays of the old 45° starts. Comparing old and new native
numbers as a controlled method effect would confound initial orientation.

**Heading reference at rest.** Desired heading is the analytic S-path tangent,
113° + atan2(2π*.14*cos(2πu),1.2), with quintic progress u through the motion.
It is defined even when speed is zero and retains the endpoint tangent in the
final hold. Its derivative supplies desired turn rate. Raw=command throughout:
no active reference governor or shaping lag. Sampled peaks .231862 rad/s and
.0692701 rad/s² pass the registered .35/.5 reference-design guards; these are
not actuator-feasibility guarantees. No progress reversal occurs on this S, so
a general turn-around versus backward-driving policy is not established.

**Measured rate.** With body-forward vector h=R[:,0] and
hdot=omega_world×h, exact Euler heading rate is
(hx*hdot_y−hy*hdot_x)/(hx²+hy²). Projection norm²≤1e−8 disables the helper with
an explicit invalid flag. Physical world-z angular speed is separately logged;
on a tilted body it is not substituted for the Euler derivative. T retains
helper gain .2, damping .06, cap .15 N m, horizontal reaction budget .05 N m,
load/lever eligibility and ±.5 rad used-error clamp. It damps rate error relative
to the planned turn, rather than damping only absolute world-z rotation.

**Known scope before results.** Native LQR angular-velocity references stay
exactly zero and rolling allocation handles translation only. T is an overlay,
not a complete coordinated planar-pose controller. Native rate feedback may
oppose intended turns. Full consistent angular/rolling/LQR reference changes
would be another intervention, especially with tilted world-up lean references.
A negative overlay result does not establish mechanical impossibility or a
holonomic/nonholonomic classification.

| Phase | Heading RMS N→T, degrees | Euler-rate-error RMS N→T, rad/s | Moving XY RMS N→T, mm | Final XY N→T, mm |
|---|---:|---:|---:|---:|
| −1.0 | 78.26→9.43 | .20181→.06997 | 19.66→33.82 | 3.11→81.93 |
| +0.3 | 96.52→22.87 | .11068→.11209 | 18.07→24.14 | 11.77→56.99 |
| +0.9 | 94.45→23.61 | .36201→.16983 | 20.08→27.01 | 41.91→26.11 |

Heading/rate metrics use 3–45 s including terminal state; moving XY uses timed
reference over [3,35), not nearest geometric path distance. All six complete,
finite and bounded. All 212 setup checks pass; **45/60 control checks pass**.
Every phase improves heading RMS by at least 20%; rate RMS passes 2/3. Every
moving XY RMS worsens. Additional failed margins include lean, position peaks,
acceleration, Euler-rate peak, clipping-excess magnitude and command steps.
No gates or parameters change after outcomes. Status **FAIL_TANGENT_SCREEN**.

Only +0.3 N passes the original four simultaneous final-2 s settling criteria;
no T does, and no case passes the five-condition version adding ≤10° heading.
Final T heading errors are 5.41/17.68/7.46°. −1.0 T is almost stationary about
81.93 mm off goal. Better pointing or little final motion does not establish
successful goal return. Actual helper activity is substantial in all T runs.

**Command spike, algebraic source.** A sealed bounded posthoc audit reconstructs
ledgers at each maximum normalized clipping excess and fixed ±25 ms context.
In +0.9 T at 15.515 s, shaft/shell-B rolling references −74.107/+206.791 rad/s
versus actual +3.510/−7.594 generate LQR joint-rate blocks −17.740/+17.401 N m.
Requested motors `[−18.290,.790,17.551]` clip to `[−1.5,.6,.6]` N m. Heading
adds only `[0,.127,0]` N m; rolling additive feedforward/preparation/fade are
zero at that row. Thus the large demand enters through rolling references and
LQR velocity error, not direct helper magnitude. This is an algebraic source,
not the complete physical cause or proof of a contact-Jacobian singularity.
All actual caps hold. +0.9 request-excess RMS rises .00670→.39975 despite
any-channel clipping occurring only .200%/.2556% of N/T held intervals.

All raw/feedback angular refs verify zero. Rate-block/helper motor-vector
opposition is descriptive and does not establish net physical yaw conflict.
At 35/45 s −1.0 T has no eligible free shell and lever ≈.0138, while +0.3 T still
has an eligible free shell with ≈.07 N m helper (35 s held; 45 s emitted but
unheld). Not every remaining offset can
be attributed to loss of helper availability. No new physical mapping or
causal intervention is performed in this audit.

**Verification and decision.** Five reference functions, twelve adapter/runner
functions (four identity subtests included) and final 28 evaluation functions
(inclusive of earlier 27) pass. Seven media functions include five reused and two new;
six restore functions are reused. These are scoped checks, not the repo suite.
Preserved reference-loader, venv-launcher and draft docstring failures precede
physics/outcome analysis and do not hide failed physical episodes. Original
83-source/17-input/48-output bindings stay unchanged. Posthoc algebraic
identities are not extra software tests. Two post-encode video decode failures
are preserved; separate sequential recovery verifies all 2,703 existing frames
and unchanged encoded bytes, with no rerender or physics. Exact decoder resource
cause remains unconfirmed. Sources, compact outcomes, PDF, full videos and
portable restoration are in the
[I-75 record](iterations/2026-11-27/tangent-heading-control/README.md).

Retain original N as selected; T remains experimental. Preserve both the
heading progress and tracking/stopping/command regressions for presentation.
A full coordinated heading reference remains proposed and untested. Do not
quietly promote the overlay or restart parked solver/HMM/PF work.

## 20. I-76: broader route comparison supports a limited figure-eight benefit

**Question.** The user believes facing planned travel may help more generally
and asks for more trajectories. Keep the I-75 helper settings and original
LQR/observer/contact feedforward unchanged; broaden geometry without tuning.
Nine matched route/phase pairs compare native N with trajectory-heading T.
The selected live controller stays N. T is an opt-in experiment, not a replacement
contact estimator or dynamic contact predictor.

**Inputs and design.** Fixed site anchor[-2.4,10]m, rotation113°, quintic progress,
3 s initial/10 s final hold,5 ms control/1 ms Gazebo/DART physics. Straight uses
[1.2u,0], arc[.6sin(pi*u/2),.6(1−cos(pi*u/2))], both32 s motion/45 s total.
Eight uses[.5sin(2piu),.25sin(4piu)],120 s motion/133 s total, with local origin
offset[.5,0]m. Both N/T start facing their geometric tangent: yaw113° for
straight/arc,158° for eight. Initial q/v/last input match exactly per pair.
These are fresh baselines, not old S recordings. Eight starts/ends at its
crossing with a regular tangent; it is not a stop/reversal at that crossing.

Raw and command heading coincide. Exact Euler-heading-rate feedback follows
the planned derivative, with original .2/.06 gains, .15Nm helper cap and .05Nm
horizontal reaction budget, free-shell/lever eligibility and used-error clamp.
Planned rate peaks0/.09204/.29677rad/s meet the same .35/.5 rate/acceleration
guards. Planned accumulated heading variation, sampled over4097uniformu, is
about0°/90°/540°; eight has zero net heading change. Its slower schedule does
not make all routes equally difficult. Native LQR angular refs stay zero and
rolling allocation translation-only, as in I-75. Full reference coordination
remains untested.

**Terrain boundaries are explicit.** Straight/arc pass the original moderate
12°facet/80mm relief/.15m corridor/≥1m feature-clearance screen. Eight fails
at its original placement111.37mm/12.385°, and after half-metre forward shift
96.72mm/12.385°. A finite preregistered six-placement map-only scan finds no
pass; no candidate is selected. Before reading controller outcomes, the domain
is revised to retain the shifted eight as explicitly rough stress outside the
moderate screen. Its slope/relief pass flags remain false; coverage/clearance
pass. Straight/arc must still pass. No failed geometry gate is relabeled as a
pass. The 686 later consistency checks concern the registered source/reference/
state boundary, not a new moderate-screen pass for eight. Comparisons isolate
method effects within each route/phase, not geometry alone across terrains.

**Criteria fixed before outcomes.** T heading RMS≤max(.8N,5°), Euler-rate RMS
≤max(.8N,.03rad/s), with relative benefits distinguished from absolute-floor
passes. Prior timed XY/lean/tilt/speed/50 ms acceleration/clip-frequency/clip-
magnitude/command-step guards remain. Add moving geometric RMS≤1.10N+1mm,
peak≤N+10mm, and require T four-condition stopping whenever N passes. All nine
pairs must pass. Geometric distance is exact distance to a fixed4097point
uniform-u polyline, not exact analytic distance. It can hide incorrect progress
at the crossing, so timed tracking remains a separate criterion.

| Route / phase | Heading RMS N→T, degrees | Moving timed XY RMS N→T, mm | Geometric RMS N→T, mm | Final goal N→T, mm |
|---|---:|---:|---:|---:|
| straight−1 |69.08→34.46|42.42→59.32|39.08→56.29|18.74→22.69|
| straight+.3 |76.25→17.89|35.81→34.67|32.57→29.23|1.26→52.38|
| straight+.9 |91.35→9.64|29.35→40.83|18.32→22.78|2.13→.54|
| arc−1 |26.07→13.98|16.91→24.87|10.91→12.60|1.96→8.67|
| arc+.3 |38.24→12.74|13.62→58.12|7.75→12.62|13.24→133.01|
| arc+.9 |83.91→18.50|24.10→25.28|13.54→21.14|7.28→40.68|
| eight−1 |96.02→76.96|46.36→28.66|39.87→15.54|32.88→42.13|
| eight+.3 |88.84→90.89|44.53→42.58|27.91→22.22|.301→28.32|
| eight+.9 |101.15→51.70|62.91→69.54|32.73→17.21|.060→12.74|

Heading/rate windows are3s through terminal, moving windows[3,35) or[3,123).
All 18 complete, finite and capped;686/686 consistency checks pass. **165/207
control checks pass;42 fail: FAIL_TANGENT_SCREEN**. Heading RMS improves 8/9,
reaches 20% benefit7/9; rateRMS improves7/9, reaches 20% benefit6/9. No absolute-floor
pass. TimedXY improves 3/9; geometric4/9. Eight−1 misses the20% heading threshold
narrowly at 19.846%reduction (.148°RMS excess), while its other22 checks pass.
Keep that failed gate; do not redefine success because it nearly passes.

**What is promising.** All three eight starts improve geometric distance, with
timed improvement2/3. This is motion evidence beyond better pointing. Straight
improves 1/3; arc 0/3, so there is no consistent general benefit under this overlay.
Eight+.9 has better geometric distance and final heading, but worse timed RMS
and final position error;
these measure different aspects and must not be combined into an invented win.

**Stopping and peaks.** N passes final 2 s four-condition settling at arc+.9,
eight+.3,eight+.9. T passes straight+.9,arc−1,eight+.9. Equal totals 3each conceal
two native successes lost and two different gains. Five-condition stopping,
which adds≤10° raw heading error throughout the same final 2 s, passes only
straight+.9 T and eight+.9 T; N none. Straight−1 T has 19.89° tilt,1.09 m/s speed,
14.12 m/s²50 ms acceleration and4.725 rad/s Euler-rate-error peaks. All actual
input caps hold. No T activity is insufficient, so the result is not a lack-of-
exposure inconclusive screen. Better average pointing is not final regulation.

**Different algebraic spike sources.** A bounded descriptive audit uses all 18
maximum normalized clipping events and±25ms contexts, declared during campaign
before outcome reads. All 144 ledger identities verify. In straight−1 N at
20.585s, rolling shaft/shell-B refs26.056/−91.896rad/s produce large joint-rate
LQR terms; motors request4.741/−.167/−7.190Nm and apply1.5/−.167/−.6. In
straight−1 T at20.975s, refs are ordinary .544/0/.183rad/s; attitude and body-
velocity feedback dominate shell-B request−7.272Nm, clipped−.6. Helper adds
only+.107Nm onA. Large rolling refs are not the source of every request; a
small direct helper term does not establish how the different closed-loop
states arose. No contact singularity or complete physical cause is proved.
Eight max normalized clipping excess N/T improves7.367/.327,3.242/3.175,
5.503/1.973; selected+.3T/+.9T events do contain dominant joint-rate blocks.

**Iteration history is part of the result.** The first dynamic attempt is
unscored: twelve processes finish, three hit a numerical reference exception,
three remaining ones are explicitly aborted with lost in-memory states disclosed.
The next accumulated clock122.99999999996062s is just below123s, but expanded
quintic cancellation yields progress1.0000000000000009. Strict geometry-domain
validation throws. A minimal clip to mathematical[0,1], preserving derivative
arithmetic and every controller parameter, fixes only that numeric domain.
Adversarial observed-clock/ULP checks pass. Stored terrain-fixture XY/V/A and
seven ground points match exactly; no new map query is performed.

All 18 corrected cases start fresh. After accidental client closure, six complete
corrected runs are recovered with their actual previously observed exit0 tool
stdout and independently verified artifact/provenance hashes; three interrupted
arc records/logs are preserved, twelve remaining jobs resume unchanged. No
first-attempt recording is pooled into corrected outcomes. Numerical replay then
finds all 12 old full recordings,3saved precision prefixes and3truth prefixes
exact by dtype/shape/arraybytes. The original partials were not in the first
120file seal; they receive explicit later bindings before replay. Earlier error
diagnosis read only time. This is correction reproducibility, not an adoption
result or new unit-test count; three unsaved aborts are excluded.

**Verification and decision.** Final scoped functions:10 reference,13 runner,
34 evaluation,9 media,7 package/restore plus one metadata-coverage regression,
with earlier stages/subtests included.
Original gains/plugins/production sources remain. All149 corrected output files
seal before canonical evaluation, which runs once. Allnine full side-by-side
videos retain actual goal holds, raw heading/rate/tilt and command metadata.
[Full I-76 record, PDF, videos and restoration](iterations/2026-11-27/tangent-heading-routes/README.md).
Keep native N selected and T experimental. The figure-eight spatial benefit
justifies a focused next hypothesis; it does not qualify the current overlay as
a general improvement or reject fully coordinated trajectory-facing control.
An archive coverage omission is also preserved: the initial suffix set omitted
JSONL recovery evidence needed by the final149-output seal. A separate metadata
amendment preserves the initial compact manifest/raw bundle, adds the exact
files with source/receipt provenance and tests missing/corrupt-JSONL behavior.
The coverage check also finds18 per-case model YAML snapshots missing from a
top-level scan; exact copies are validated against sealed prephysics provenance.
Final actual restoration must close over all149 sealed outputs and18 model
prerequisites, as well as registered result-owned tool inputs, not only the
selected inventory. Final actual-restore QA passes all167 essential
prerequisites and223 registered result-owned inputs, with all3,633 compact files
and7,746 raw members verified. No physics, video, metric or control source
changes result.

## 21. I-77: investigate the original baseline before adding complexity

The selected native N pipeline remains unchanged. I-77 first uses the nine
sealed I-76 native recordings and reconstructs three cases' rolling references
exactly over 44,603 saved states without integration. The rolling map stacks
unweighted per-contact XY constraints with scales [5,5,5,.3,.3]; those are
preferences, not hard speed limits. No bounded allocator deliberately allows
trajectory deviation in this selected baseline.

On straight −1 at 20.585 s, shaft/shell-B drive directions are 0.389° apart,
condition 571.82, and reference rates 26.06/−91.90 rad/s. Recorded pressure and
inferred support both identify shell B; equivalent points differ by 0.140 mm.
Substituting the pressure point still gives condition 383.97 and rates
17.73/−61.68 rad/s. Reference growth occurs while cross-track error is closing.
Later contact switching and the largest pulse must be distinguished from the
beginning of the response.

Three new 24 s diagnostic prefixes preserve original observer, normal, raw
allocation/cache, support feedforward, gains, preparation, fade and caps.
Native replay matches all 79 original precision fields through 24 s, except its
explicit shortened terminal held-command mask. Independent checks verify the
candidate prefixes and first-active states. A single-row hold at 20.585 s leaves
20.4–24 s geometric peak 150.7 mm versus native 148.9 mm. A ±5 rad/s feedback
reference cap during [20.400,20.650) s changes 43 rows and reduces peak to
21.98 mm and RMS from 108.24 to 16.86 mm. At 20.650 s before release acts,
error is slightly worse: 12.507 versus 11.995 mm. The benefit develops later.
This proves a selected reference-window contribution, not a general fix, motor
rating or full 45 s route success. Earlier guard/allocator regressions remain
relevant.

The late eight −1 has another pattern. Its small-position crossing near
99.175 s retains 29.7 mm/s velocity disagreement. Actual drive-rate disagreement
dominates late velocity error; body rotation partly cancels it and material
rolling residual is tiny in 98.5–100.8 s. That does not exonerate the preceding
96 s disturbance. The last jointly small position/velocity/attitude interval
ends at 96.160 s. Desired-up motion appears at 96.150 s; at 96.4 s its rate is
84.3°/s, preceding 82 mm/s velocity disagreement at 96.5 s. The rolling inverse
is well conditioned and no command clips during 96–99.2 s.

An unchanged 97.5 s native replay reproduces all 79 precision fields and captures
1,500 physical 1 ms samples over 96–97.5 s. Exported pressure-weighted solver normals change rapidly. Applying the native 100 rad/s wrench filter to actual interval
forces/moments, anchored to native filtered wrench at 96 s, matches native
intrinsic direction within 0.02345° maximum / 0.00974° RMS. The observer wrench
is not inventing those changes in this window. Equivalent contact radial
direction, surface normal, filtered normal and nearest-centre DEM tangent are
distinct observables. Their current-vs-history disagreement does not establish
wall-clock calculation delay.

Reconstructing the unchanged attitude function confirms nearly all fast target
motion comes from the normal: 152.16°/s peak versus acceleration contribution
below 0.0027°/s, with no lean cap. Baseline angular-rate reference remains zero.
The isolated attitude-reference follow-up is recorded in the
[full I-77 cause report](iterations/2026-11-27/baseline-deviation-cause/README.md).
It must compare the same untreated native target separately from its held
target; a small error to an easier target is not a balance improvement.

The final held-attitude diagnostic freezes only the raw attitude supplied to
feedback during [96.400,96.900) s, then releases it. All 79 fields before the
intervention and first-active inputs reproduce exactly; independent analysis
reconstructs all 20,601 feedback errors/LQR terms. The primary pre-release
96.900 s geometric error slightly worsens, 5.924→6.898 mm, and 97.500 s is also
slightly worse. Later geometric peak over 96.4–103 s falls 316.714→62.191 mm
and RMS 142.194→44.979 mm. Release itself introduces a 0.491 N m shaft-command
step. This confirms an early reference-response/return contribution; it does
not separate holding from release or prove that one filter or missing angular
reference is the sole cause.

Targets diverge with physical paths. At 103 s the treated robot is 7.940° from
the saved original-native attitude but only 0.020° from its current branch-local
untreated target. The former is not by itself a balance failure; the latter is
not a robustness certificate. Holding terrain attitude at a chosen event time
is a diagnostic, not an adopted controller. All five diagnostic physics runs
complete their requested shortened horizons; no full-route acceptance follows.

Keep the momentum observer and native baseline selected. Accurate force
estimation does not guarantee feasible or smooth feedback references. Neither
desired-vs-actual velocity mismatch nor one accurate contact-point sample is
sufficient to choose a new estimator. Current reports separate measured
physical contributions from unresolved mechanisms and preserve failed setups.

## 22. I-78: coherent path retiming is useful in one case, not a general fix

The student asked whether the advancing time reference truly caused departures.
I-77 shows causal reference-window effects, but it does not identify a sole
root cause. I-78 tests a remedy hypothesis without changing original LQR,
observer, lean, gains, inverse rolling map, feedforward, preparation or caps.

The original straight reference has a3s hold,32s minimum-jerk1.2m traversal,
then10s goal hold. N finishes45s. G introduces virtual time τ and rate λ∈[0,1]:
`p=g(τ)`, `v=g′(τ)λ`, `a=g″(τ)λ²+g′(τ)λdot`. It can pause without a positive
floor, but a five-second failure to advance while unfinished and away from goal
fails the experiment. G must complete plus10s actual goal hold within75s.

One private native clone evaluates full-speed command demand at current state
and virtual position, with history refreshed from the live controller. It never
integrates physics or applies commands. Rate target is the minimum of one,
inferred load-weighted planar authority/.15m/s,5rad/s over nominal largest joint
reference, and .85 over normalized nominal full native request. A two-stage
bounded cascade, .10s time constant and4s⁻¹ rate bound, smooths the clock. These
are design assumptions, not motor ratings or a certified balance reserve.
Exactly one live update commands each5ms hold. No truth contacts, camera,
heading helper, tilt shaper or future contact forecast is introduced.

Six fresh Gazebo/DART Mars straight cases at[−2.4,10]m,yaw113° and starts
−1.0/+0.3/+0.9rad use the same gains and[1.5,.6,.6Nm,.3,.3m/s]caps. All original
N precision arrays replay exactly. G reaches the reference endpoint at
35.985/35.600/35.005s and then completes10s goal holds. Primary full-path metrics:

| Shaft start | Peak path error N→G | Progress-weighted RMS N→G | Added time |
| --- | ---: | ---: | ---: |
| −1.0 | 148.9→102.0 mm | 50.0→34.0 mm | .985s |
| +0.3 | 124.6→124.4 mm | 42.9→39.7 mm | .600s |
| +0.9 | 59.7→59.2 mm | 21.3→21.2 mm | .005s |

All180 setup checks pass;80/82 performance checks pass. At+0.3 absolute tilt
9.397→9.970° exceeds the native+.5° margin, and speed.1819→.2284m/s exceeds
.2101m/s. No phase/gate/parameter retuning follows results. All six meet final
position/speed convergence; only G−1 meets full position/speed/rotation/attitude
settling. G−1 clipping time drops.680→.240s but largest joint reference is still
66.13rad/s versus91.90 native. This is not an enforced future-input constraint.

At+0.3, G's peak speed occurs19.730s while desired speed is only.03869m/s and
no same-row command clips. Tilt peaks20.070s during reacceleration, when desired
acceleration is.07785m/s² and a motor request−.6594Nm clips to−.6Nm. Changed
reference/contact/state histories are associations, not isolated physical causes.
Slowing desired motion can alter balancing motion and does not directly cap
actual motion. The governor has no prediction or invariance proof.

Two independent raw-data audits reproduce scalar metrics and all82 decisions;
one covers564 scalar leaves. Original registered analysis is preserved alongside
pre-array structural/arrival corrections, without numerical gate changes. A
later prose claim that G+.9 never retimed was wrong; saved reports were correct:
first changed reference19.815s, minimum rate.9862, actual lag4.631ms. Direct raw
checks confirm the canonical metrics. No physics rerun corrects the prose.

Keep N selected. One meaningful improvement, two nearly unchanged peaks and a
regression support continued diagnosis, not claims that the clock alone caused
the problem or more threshold tuning must cure it. Sources, plots, PDF, videos,
conditions and reproduction are indexed in the
[I-78 report](iterations/2026-11-28/path-progress-governor/README.md).

## 23. Sources, attribution, and how to continue a discussion

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
