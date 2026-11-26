# I-55 — matched-prefix causal diagnostics

2026-11-26. **Four short Gazebo diagnostics; no controller default adopted.**

The question is why the I-52 −1.00-rad R case accelerates and spins, before
choosing a control change. I-54 identified a moving balance-attitude target,
an abrupt support-dependent feedback request and shell/platform momentum
exchange. This follow-up changes one action at a time after an identical
native prefix, rather than treating the different B/R/N/NR histories as
matched-state interventions.

## Conditions and registered cases

Every case uses the original R controller from I-52: exact I-51 joint-reference
guard, ordinary flat gain bank, world-vertical gain phase, momentum observer
and geometry-derived contacts, incoming-drive preparation, initial common-shaft
phase −1.00 rad and initial yaw 45°. Body/joint states are exact simulation
measurements. The terrain is the same Mars yard and the reference is the same
40 s figure-eight. Commands retain caps [1.5, 0.6, 0.6 N m, 0.30, 0.30 m/s].

All cases stop at recorded **32.500 s**. This is a deliberately truncated
prefix of the old 44 s job, not a complete route/settling trial. Inherited
parent-runner completion fields mean completion of this short requested
horizon. No full-route acceptance claim is made.
The original overview's playback 10.000 s corresponds to recorded 32.000 s;
the finer events here come from the new 1 ms logs rather than extra frames
in the old video.

| Case | Direct intervention |
| --- | --- |
| `native_batch` | None; original five physics steps in one bridge call |
| `native_substeps` | None; read state and all contacts after each 1 ms step over 31.400–32.500 s |
| `zero_free_shellA` | Same recording; set only shell-A requested torque to zero during 32.250–32.275 s, then apply the existing command box |
| `hold_attitude` | Same recording; hold only the balance-attitude quaternion passed to the original feedback during 31.580–31.745 s, using its first native value |

The attitude intervention holds the desired up direction; heading remains
free. It does not freeze the observer, ground-normal estimate, gains,
feedforward, XY target or velocity-reference logic. Their later values may
change through feedback once the physical states diverge. At the end of each
intervention the native rule resumes; release effects are distinct from the
primary endpoint before release has acted on the plant.

The original runner and all live runtime/config/plugin/gain files remain
unchanged. A results-owned wrapper calls the original command and feedback
once. No branch restores a pose or resets hidden solver/controller history;
every branch replays the whole prefix from the original initialization.
The observer receives the actual previous physical input in the normal way.

## Replay and measurement gates

Both native cases reproduce **all 56 shared precision fields over 6,501
control samples exactly**, through 32.500 s, against the original I-52 R
trace. Their 14 sidecar fields, including selected numerical controller-history
fingerprints, also match exactly. This establishes that the additional reads
and five separate `run(1)` calls did not change the observed native evolution.

The 1 ms recorder keeps 1,100 consecutive samples over 220 unchanged 5 ms
holds, with raw bridge state before/after every solve, every exported contact
row (including rejected/body records), actual held input, friction, disturbance,
world clock and controller clock. It makes no controller/observer/mirror
update inside a hold. Four fresh processes run serially. These are four
simulator diagnostics, not thousands of independent tests.

## Removing the unloaded shell torque confirms a yaw contribution

The shell-A branch matches the original state, observer, gain and selected
controller history through the intervention start. All earlier outputs match.
Only the registered shell-A request component is overridden. Shell A has
**zero exported contact force throughout all 25 affected 1 ms steps**, in
both native and treated traces.

At the start, both have world-vertical platform angular velocity
**3.825019 rad/s**. Over the first 5 ms the other four inputs are still held
at exactly the same values, so this is a clean single-input comparison:

| Recorded time | Native world-vertical rate | Shell-A-zero rate |
| --- | ---: | ---: |
| 32.250 s | 3.825019 rad/s | 3.825019 rad/s |
| 32.251 s | 3.923975 rad/s | 3.869608 rad/s |
| 32.255 s | 4.315864 rad/s | 4.045030 rad/s |
| 32.275 s | 5.992068 rad/s | 4.829581 rad/s |
| 32.305 s | 5.957415 rad/s | 5.813504 rad/s |
| 32.500 s | 2.170991 rad/s | 2.188376 rad/s |

The first-hold increase is 0.490845 versus 0.220012 rad/s: removing shell-A
actuation reduces that increase by **55.2%**. This establishes a local causal
contribution of that motor action to the yaw surge. Its axis is nearly
vertical and its opposite motor reaction has the observed yaw direction.
The ground and other bodies can mediate the response; the comparison does
not identify an independent unlimited yaw actuator.

Over that same first hold, treatment minus native angular-momentum changes
are −0.002846 kg m²/s for fork/carrier, +0.002840 for shell A, +0.000003
for shell B and approximately −0.000003 for the whole assembly. The paired
effect is almost entirely an opposing shell-A/platform exchange under the
verified inertias. This strengthens the reaction explanation beyond merely
observing opposite rotations in one trace.

After the first hold, the unchanged feedback reacts to different states;
other commands may differ without being directly overridden. The 25 ms
result therefore includes the closed-loop response. At 32.275 s the original
shell-A rule resumes. Yaw rates later nearly reconverge, and at 32.305 s
planar speed is slightly worse, **1.039 versus 1.025 m/s**. This diagnostic is
not a controller improvement or an argument for disabling the unloaded shell.

## Holding the balance target reduces spin but worsens target tracking

The attitude branch also reproduces its complete native prefix through
31.580 s, including measured state and selected history. The hook changes
only the quaternion supplied to native feedback. It does not change a gain
or normal-source setting.

At **31.745 s**, before the first released command acts:

| Metric | Native R replay | Held-attitude branch |
| --- | ---: | ---: |
| Platform world-vertical angular velocity | 3.133 rad/s | 1.872 rad/s |
| Absolute tilt from vertical | 2.709° | 4.899° |
| Error to the same fixed untreated desired-up direction | 9.848° | **15.932°** |
| Error to the held attitude target | Different target | **0.495°** |

The actual up directions differ by 6.129°. Therefore the moving attitude
target changes real robot motion and feedback demand, beyond its arithmetic
effect on the error metric. However, the small 0.495° held-target error would
be a misleading improvement claim: the branch is further from the original
desired upright direction. Its current normal-derived target also differs
from the untreated target once states diverge; its error to that current
branch-native target is 10.624°. All three references remain distinct.

The logged row at 31.745 s already contains the resumed native target and a
saturated command [1.5, −0.6, −0.6 N m, 0, 0.265 m/s]. That command acts
on the following interval, so it cannot explain the primary endpoint state.
Resuming the moving target creates another yaw transient afterward. Those
release effects are retained separately, not used to redefine the primary
comparison or hide an adverse outcome. Holding and abruptly releasing the
attitude target is a diagnostic intervention, not a proposed controller.

### The rapid target swing accompanies a real contact-geometry change

A descriptive follow-up uses the newly recorded contacts at 69 fixed control
endpoints from 31.405–31.745 s. The force-weighted contact-point radial
direction changes **17.826°**; the equivalent direction reconstructed from
the actual signed force/moment changes **17.825°**. The estimated normal
changes **16.031°**. Thus a large physical contact-geometry swing is present;
this episode cannot be explained solely as a fictitious estimated-normal jump.

The preceding 5 ms true-wrench equivalent direction differs from the estimated
normal by 2.137° RMS and at most 5.070°. This comparison does not prove the
estimator is perfect or uniquely attribute its differences to delay. It
compares a physical interval wrench with a filtered model-based estimate.
Wheel radial directions, solver normals and a unique terrain plane are
different observables, especially during multi-contact.

Two last-1-ms endpoints carry zero force, while every full 5 ms interval
carries some force. Those missing endpoint normals are retained explicitly;
they are not interpolated or relabeled as estimator failures. Only shell B
carries force in this fixed window. The pre/post wheel-center sensitivity
of the intrinsic reconstruction is at most about 0.206°. This follow-up is
saved-data analysis after the registered experiments, not another intervention
or a change to their primary endpoints or gates.

## Scope of the source and force accounting

Each recorded contact force is retained with the same exported point and
world frame. Versioned DART source computes contact force from constraint
impulse divided by the solver timestep, and the Gazebo physics adapter
preserves the force. Thus `F × actual dt` can reconstruct each solve's contact
impulse, conditional on the export's ordering/completeness. The source review
and linear/angular accounting retain those qualifications.
[Versioned DART implementation](https://github.com/dartsim/dart/blob/v6.19.5/dart/constraint/ContactConstraint.cpp#L645),
[Gazebo DART adapter](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.5.0/dartsim/src/SimulationFeatures.cc#L239).

The local physics system advances DART, updates link/joint state and replaces
contact data before the bridge reads them in PostUpdate. The source does not
support shifting the forces by one step to improve a budget fit. The primary
moment center is the midpoint system COM; pre/post COM sensitivities retain
the exact same exported contact points. Mirror/SDF inertias and angular-velocity
frames are verified; calculations use spin plus orbital angular momentum.

Per-body budgets retain the moving-COM transport term and known shell and
common-shaft motor couples. Internal couples cancel in the complete-system
budget. Any remaining per-body term must retain joint-constraint, passive,
transport/discretization and modeling qualifications; it is not an isolated
unmeasured contact torque.

The registered complete-system yaw accounting has mixed closure:

| Native interval | State-derived ΔH | Contact impulse, midpoint COM | Residual | Registered check |
| --- | ---: | ---: | ---: | --- |
| 32.250–32.275 s | +0.001385 N m s | +0.001312 N m s | −0.000074 N m s, 5.34% | Pass |
| 32.250–32.305 s | +0.002798 N m s | +0.002468 N m s | −0.000330 N m s, 11.80% | **Fail** |

The threshold is `max(0.0001 N m s, 10% of absolute ΔH)`, declared before
branch analysis. Prior/post COM sensitivities do not rescue the longer
window. Linear-momentum accounting also retains nonzero residuals. These
failures limit claims of a completely closed force/impulse budget; they do
not erase the matched-input causal effect measured directly in angular
velocity. No force signs, time shifts or thresholds were changed to pass.

## Status and interpretation

These are causal diagnostics of a specific configuration and time window.
They distinguish an individual motor contribution from cross-run correlation,
but they do not certify whole-route performance, arbitrary multi-contact
behavior or a universally safe controller change. Earlier unsuccessful trials
and evidence remain unchanged.

The next control-design target is a feasible, continuous balance/drive handoff
that accounts for shell momentum and total applied-command limits while
tracking the necessary upright target. Simply removing unloaded-shell torque
or freezing the target sacrifices another part of the response. The earlier
[I-43 shaped-attitude/rate trial](../../2026-11-25/tilt-reference/README.md)
already had mixed results and was not adopted; this investigation does not
turn reference smoothing into a demonstrated general fix.

## Evidence, validation and reproduction

- [Registered four-case protocol](evidence/protocol.json)
  · [native replay/parity gate](evidence/native_parity_gate.json).
- [Canonical four-case report](evidence/analysis/stage01/report.json)
  · [compact presentation summary](evidence/analysis/presentation_stage02_three_targets/summary.json).
- [Shell-A causal response](evidence/analysis/presentation_stage02_three_targets/shell_A_causal_response.pdf)
  · [image](evidence/analysis/presentation_stage02_three_targets/shell_A_causal_response.png).
- [Attitude response and three reference definitions](evidence/analysis/presentation_stage02_three_targets/attitude_causal_response.pdf)
  · [image](evidence/analysis/presentation_stage02_three_targets/attitude_causal_response.png).
- [Portable event data and example](data/README.md)
  · [data hashes/derivations](data/manifest.json).
- [14 fake-recorder contracts](evidence/instrumentation_checks_stage02/report.json)
  · [nine pure math checks](evidence/analysis/checks_stage02_corrected/report.json)
  · [53 independent saved-data assertions](evidence/review/independent_numeric_stage02/report.json).
- [Force export source proof](evidence/review/budget_source_review_stage01/report.json)
  · [physics/publication timing proof](evidence/review/source_timing_stage01/report.json)
  · [contact-geometry follow-up](evidence/review/normal_geometry_stage02/report.json)
  · [publication checks](evidence/publication_review/report.json)
  · [staged scope before its metadata entry](evidence/publication_review/staged_scope_before_metadata.json)
  · [portable evidence inventory](evidence/manifest.json).

These are separate QA scopes, not an additive full-repository test count.
No extra full repository-suite run was needed for unchanged runtime sources.
The initial math fixture failed an inclusive floating-boundary expectation;
the corrected stage uses only a 2e−15 N m s roundoff allowance, without changing
the physical budget tolerance. The independent review's first input declaration
omitted two attitude precision files; its corrected registration and rerun
reproduce all numerical values. Both stages remain visible. A later release
readout's console JSON serialization failed after its correct report was saved;
the fresh stdout correction changes no result. Earlier plots and corrected
three-target presentation plots are retained separately.
The first contact-normal follow-up assumed every endpoint carried force and
stopped on an assertion. Its preserved successor reports the two undefined
endpoint normals and all 5 ms interval observations explicitly.

The actual commands, after activating the existing Gazebo environment, were:

```bash
python results/causal_replay_20261126/run_diagnostic.py --case native_batch --run
python results/causal_replay_20261126/run_diagnostic.py --case native_substeps --run
python results/causal_replay_20261126/run_diagnostic.py --case zero_free_shellA --run
python results/causal_replay_20261126/run_diagnostic.py --case hold_attitude --run
```

Execution used `MUJOCO_GL=egl`, `OPENBLAS_NUM_THREADS=1`, `OMP_NUM_THREADS=1`
and the established Gazebo environment, with external ROS path variables
removed. The [executed wrapper](evidence/run_diagnostic.py) and
[recorder](evidence/substep_trace.py) are archived byte-exact. They refuse an
existing case output. The prefix hash/reference is the original full I-52
`m100_R/precision.npz`; original I-52 inputs and frozen dependencies remain
requirements. Full prefix logs/workspaces are local, while compact 1 ms traces
and 5 ms event subsets are committed here. This archive is not a new generic
fresh-clone launcher: restore sources to their recorded original paths and
register any new baseline dataset explicitly. Do not edit old protocol hashes
or execute archived model XML expecting its omitted relative mesh assets.
The [existing workspace guide](../../../../research/README.md) explains the
native I-52 prerequisites and their restoration scope.
