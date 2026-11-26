# Predictive balance-control prototype — I-27–I-30

2026-11-25 · [Iteration log](../../../../ITERATION_LOG.md#i-27--implement-estimated-plane-dynamic-prediction-and-constrained-mpc)
· [Evidence/source manifest](evidence/manifest.json)

**MPC is implemented and tested as an experiment. It substantially improves
balance in the initial weak-direction fixture, but does not yet recover the
original goal. It is not promoted over the existing controller.** The default
remains gain-scheduled LQR with contact-aware feedforward.

## What was implemented

The predictor owns a private copy of the mechanical model. It receives measured
pose/velocity, actual applied input and observer/locator estimates of support
point, normal, load and friction. It predicts its own unilateral contact physics
against an estimated plane. It never reads live Gazebo contacts or terrain.
The plane's private normal offset can be calibrated within +/-3 mm to inferred
load; no supporting force is injected and no physical pose is reset.

The optimizer chooses **three motor torques and two ideal belt-speed commands**
over 20 stages of .02 s (.4 s horizon). Existing physical input limits are hard
constraints. Costs cover predicted position/velocity, roll/pitch/rates, small
rotor-rate penalties, normalized input demand and input changes. Yaw and cyclic
joint-position penalties are zero so configuration can evolve. Height and
normal velocity use estimated-plane coordinates. Tilt, actual rotor speed and
slew do not have hard safety guarantees. Input cost is not measured electrical
effort; belt motor/load dynamics are still absent from the simulation.

The observer updates once every .005 s using the actual preceding command.
MPC replans every .02 s and directly supplies the final bounded physical input.
The native LQR runs as a shadow and available fallback. Its synchronization,
heading and fade outputs are not applied after a successful MPC decision.
Logs explicitly retain native shadow components and the MPC command delta;
shadow LQR/feedforward values must not be mistaken for the applied MPC law.

See [predictive_model.py](../../../../src/spherical_wheel/predictive_model.py),
[predictive_control.py](../../../../src/spherical_wheel/predictive_control.py) and
[run-mpc-comparison.py](../../../../scripts/run-mpc-comparison.py).

## Matched test and acceptance criteria

Every run uses a 10° side slope and starting shaft phase pi/2. After identical
.8 s LQR/observer warm-up, both branches anchor the body-center path once to
the measured center. The request is .6 s / 16 mm, with .2 s ramps and .04 m/s
peak speed, followed by 4 s recovery. The original timed path and destination
then remain fixed. No observer reset, imposed shaft phase or upright constraint
is used when activating MPC.
Both branches explicitly select the `hardware` estimator mode; the name denotes
simulation-state measurement surrogates, not a physical robot trial.

World Y is the initially weak direction; world X is a strong-direction control.
Record actual support and directional authority rather than assuming they stay
fixed. The initial cases maintain shell-A support during the motion pulse, but
do **not** maintain the weak-authority band throughout demand. They test a
transient through difficult geometry, not a permanently locked singularity.

The protocol requires useful progress and final original-goal error <=5 mm,
without worse peak lean. Weak Y must also reduce lean integral and original-clock
path RMS. Strong X must avoid a >10% path-RMS regression. More than 5% active
actuated LQR fallback fails. Staying upright while moving away from the goal
does not pass. Actual input-limit occupancy and solver/model failures are
reported independently of trajectory error and survival.

These are small simulation comparisons, with exact state inputs and ideal belt
actuation. They establish neither hardware behavior nor broad route robustness.

## First iteration: frozen affine MPC

The first candidate linearizes one .02 s transition and repeats that affine
model over the horizon. Its weights and terminal multiplier are fixed before
closed-loop trials. Four Gazebo runs compare LQR and MPC in both directions.

| Case | Active path RMS, mm | Peak reference-relative lean, ° | Final goal error, mm | Final travel toward +16 mm goal |
| --- | ---: | ---: | ---: | ---: |
| Y LQR | 235.63 | 9.36 | 98.83 | -74.47 mm |
| Y affine MPC | 20.07 | 1.77 | 25.08 | -8.75 mm |
| X LQR | 10.37 | 1.39 | 5.30 | 15.52 mm |
| X affine MPC | 13.58 | .92 | 1.79 | 15.27 mm |

Y lean integral falls from 13.57 to .615 degree-seconds and every generated
plan solves, with zero fallback. Nevertheless, final net displacement is opposite the
requested direction, so destination recovery fails. X has a closer endpoint
and lower peak lean but a 31% path-RMS regression and 628/920 active actuated
fallback samples (68.26%). Neither case passes the declared protocol.
All four executions finish without a physical fall.

See the [initial comparison](evidence/initial/analysis/comparison.png) and
[audited initial report](evidence/initial/analysis/report.json).

Watch the preserved initial comparisons:

- [Y LQR versus affine MPC](../../../../results/mpc_balance_20261125/initial/videos/world_y_lqr_vs_mpc_prototype.mp4).
- [X LQR versus affine MPC and fallback](../../../../results/mpc_balance_20261125/initial/videos/world_x_lqr_vs_mpc_prototype.mp4).

## Validate the model where it is actually used

On ten recorded warm X/Y states with their recorded applied inputs, maximum
prediction errors are .0302 mm/.00774° at 20 ms and .374 mm/.1074° at 100 ms.
Those local checks pass the diagnostic screen. They do not validate aggressive
optimized .4 s plans.

Replaying each optimized plan through private nonlinear physics reveals that
distinction. At .4 s, affine/nonlinear disagreement p95 is about 13.85 mm/6.69°
for Y and 33.99 mm/18.79° for X. This supports a trajectory-dependent model
experiment, without proving that model approximation alone causes poor goal
recovery.

The strong-direction fallback audit finds 156/231 static load-fit rejections.
At four exact failed states, the private solver's normal-force curve is
monotone but discontinuous at contact admission: a 10 nm offset scan can cross
a 9.52 N force jump. Nearest-load residuals .4–1.72 N occur at micrometre
offsets, far from the 3 mm bound. This is not an ordinary root-fitting bug.
Their actual-input response predictions remain accurate at 20/100 ms, but
finite-difference derivatives can change drastically with epsilon and amplify
over the horizon. Dropping normal-height/velocity coordinates did not fix that
offline; this proposed model reduction was not adopted.

The [force-fit curves](evidence/initial/calibration_audit/load_fit_curves.png)
and [calibration report](evidence/initial/calibration_audit/report.json) retain
the exact failed states and limitations.

## Rejected engineering trial: relax only the load-fit gate

One additional X run accepts a bracketed nearest-load fit within 10% of estimated
load, away from the private plane-offset bound. It retains the strict .2% fit
diagnostic and the same affine controller costs/horizon. The initial X LQR run
is reused as baseline; this is one new execution, not a new pair.

Fallback falls to 28/920 samples (3.04%), but final goal error becomes 29.54 mm,
active path RMS 19.73 mm and peak lean 3.82°. This trial fails. Nominal model
prediction accuracy and increased MPC activation alone are insufficient.
The initial Y run already passed every strict fit, so no duplicate Y gate-only
run was needed.

See the [rejected gate-only comparison](evidence/calibration_gate/analysis/comparison.png).

## Second control iteration: trajectory model and nonlinear screening

Costs, original goals, physical limits and .4 s horizon remain unchanged.
Dynamics are relinearized along the nonlinear warm trajectory in one root-state
tangent chart. Jacobians at nodes 0/5/10/15/19 are interpolated; nonlinear
nominal drift is retained at all 20 stages. The approximation and potential
contact-boundary derivative errors remain explicit.

The first update considers bounded predicted shaft-torque seeds +/- .03 N m
for .12 s alongside the warm start. They never impose a live pose or phase.
One QP refinement per seed/warm sequence is screened by nonlinear line searches
at fractions 1/.5/.25. Select the lowest finite nonlinear cost among the bounded
sequences. Later updates shift the previous plan and refine again. Source
counts distinguish warm sequence, initial seed, SQP refinement and LQR fallback.
An accepted sequence is not necessarily a solved QP or a stability certificate.

| Refined case | Active path RMS, mm | Peak lean, ° | Final goal error, mm | Final travel toward +16 mm |
| --- | ---: | ---: | ---: | ---: |
| Y trajectory MPC | 19.63 | 1.72 | 24.97 | -8.79 mm |
| X trajectory MPC | 9.18 | .84 | 8.16 | 8.87 mm |

Y accepts every generated sequence without LQR fallback and further improves
balance slightly. It still fails the original-goal criterion. Its median
update costs about 1.64 s of wall time per .02 s simulated replanning period.
X also uses zero fallback and improves path RMS and peak lean, but still ends
8.16 mm from its original goal. It fails the same <=5 mm criterion. Each case
has 230 actuated accepted sequences and 232 solved QP calls (three initial
seeds); counts including the unactuated terminal observation are 231/233.
Most applied sequences are SQP-refined, with the remainder shifted warm starts.
No initial seed sequence alone is applied.

See the [refined comparison](evidence/trajectory/analysis/comparison.png) and
[final report](evidence/trajectory/analysis/report.json) for all criteria,
reused-baseline labels, prediction consistency and source counts.

Watch the refined comparisons:

- [Y LQR versus trajectory MPC](../../../../results/mpc_balance_20261125/trajectory/videos/world_y_lqr_vs_trajectory_mpc.mp4).
- [X LQR versus trajectory MPC](../../../../results/mpc_balance_20261125/trajectory/videos/world_x_lqr_vs_trajectory_mpc.mp4).

For its chosen plans, .4 s private affine/nonlinear disagreement p95 is now
.247 mm/.079° for Y and .537 mm/.197° for X. This is improved internal plan
consistency, not full-horizon validation against Gazebo. X median update time
is about 2.02 s. The candidate remains an offline prototype.

Saved Y [nonlinear plan forecasts](evidence/trajectory/predicted_goal_progress.json)
themselves show retreat from the goal during
recovery while reducing aggregate cost. The remaining issue therefore includes
finite-horizon objective/terminal tracking design and feasible recovery
planning, rather than solely unexpected plant drift. This does not establish
physical impossibility of reaching the goal or validate a particular cost fix.

The initial affine candidate also costs about .36/.43 s per successful Y/X
update. These are offline prototypes, not real-time implementations. X's
initial .009 s all-update median must not be presented as planning speed,
because fast calibration failures dominate that statistic.

Seven new Gazebo executions are preserved: two initial baselines, two initial
affine MPCs, one gate-only X trial and two trajectory MPCs. Later comparisons
reuse the initial baselines; rendered comparisons do not add physics runs.
Both final candidates fail destination recovery, so no broader Mars sweep or
default promotion follows. The balancing improvement and failed goal test
remain visible together.

## Tests, records and limits

The initial combined predictor/solver test run passes 41 tests; the selected
Gazebo-environment integration passes 51. After LTV/interpolation and the
external solver stationarity gate, 69 selected integration tests pass. These
suites overlap and are not additive or the full repository suite. The same
predictor tests were run in two Python environments; that does not double
their unique count.

Independent tests include Riccati/scalar optima, constrained drift, varying
authority and affine anchors, cart-pole recovery with goal progress, private
model immutability, correct torque/belt semantics, common-chart derivatives
and exact nominal replay. A mock stale-Hessian backend verifies external KKT
rejection. None of 304 accepted initial plans would fail that later numerical
gate; passing it still does not certify model accuracy.

An initial optimizer-test fixture expected a negative command while a large
previous positive input and change cost correctly favored the positive bound.
That failed fixture and correction are preserved, with no optimization-math
change. Initial sources, audits, the rejected gate-only trial and refined
sources/results remain separate. Exact source hashes and raw-data inventories
are in the [manifest](evidence/manifest.json). Large rollouts and full plan JSONs
remain local; compact metrics, tests, plots, source and video metadata/stills
are preserved outside ignored results. Archived Python copies use `.py.txt`.
Seven exact launch snapshots are distinct from later CLI/error-path/diagnostic
clarifications, which do not change physical control math. The final CLI rejects
incompatible plan periods and missing optional solver dependencies before a run.

Videos replay saved Gazebo/DART physics states through MuJoCo rendering; making
them performs no new physics. The controller's private MuJoCo prediction role
is separate from that rendering role and from the physical Gazebo plant.

The local [presentation pack](../../../../results/mpc_balance_20261125/presentation-pack.zip)
contains these notes, iteration entries, compact source/evidence and all four
selected clips. Earlier packs remain frozen at their recorded stages.

## Reproduction

Use the Gazebo environment in [GAZEBO.md](../../../../GAZEBO.md), install the
optional solver there with `python -m pip install -e '.[mpc]'`, and use fresh
output directories. Completed run folders are never overwritten.

```bash
python scripts/run-mpc-comparison.py --output results/new_mpc/initial --variant baseline --motion world_y
python scripts/run-mpc-comparison.py --output results/new_mpc/initial --variant baseline --motion world_x
python scripts/run-mpc-comparison.py --output results/new_mpc/initial --variant mpc --motion world_y
python scripts/run-mpc-comparison.py --output results/new_mpc/initial --variant mpc --motion world_x
python scripts/run-mpc-comparison.py --output results/new_mpc/calibration_gate --variant mpc --motion world_x --load-residual-fraction .10
python scripts/run-mpc-comparison.py --output results/new_mpc/trajectory --variant mpc --motion world_y --model-mode trajectory --jacobian-stride 5 --load-residual-fraction .10
python scripts/run-mpc-comparison.py --output results/new_mpc/trajectory --variant mpc --motion world_x --model-mode trajectory --jacobian-stride 5 --load-residual-fraction .10
```

Current-source reruns are new experiments; archived stage-specific source is
identified separately. Reused baselines are explicitly labeled in later
comparisons. No broader Mars qualification follows a failed small-case gate.
All improvements and regressions remain in the research record.
