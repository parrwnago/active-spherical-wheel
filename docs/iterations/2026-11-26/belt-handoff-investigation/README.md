# I-50 — saved-log deviation and shell/belt handoff investigation

2026-11-26. **Saved-data/source investigation completed; no new physics or runtime changes.**

The user asks what caused the large trajectory deviation in I-49, whether the
twitch near 18 seconds of the comparison video is caused by shell-to-belt
transfer, and whether incoming belt speed is synchronized on unknown terrain.
The three original 44 s Mars figure-eight capacity arms remain the evidence;
this investigation does not introduce another controller or retune them.
The simulator is Gazebo/DART on Mars2021 DEM, phase +0.90 rad, yaw 45°,
origin [−1.6, 9.6] m, radius 0.45 m and belt caps 0.30/0.60/0.90 m/s.
The selected loop is gain-scheduled LQR, momentum observer, support locator,
rolling/holding feedforward and incoming kinematic preparation; body state
comes from simulation and solved contacts are evaluation-only. The complete
[I-49 conditions](../belt-capacity-sweep/README.md) remain unchanged.

The video has a 2 s introduction and 2× playback. Frame 360 at playback 18 s
corresponds to recorded 32 s. Inspect playback 17–19 s / recorded 30–34 s, with
31.8–32.1 s for detailed chronology. Also inspect recorded 17–19 s to resolve
the alternative meaning of the user's timestamp, the original 6.4–7.4 s
departure, and the late 39–44 s goal departure. Full-trace reductions identify
the largest timed and local geometric errors without selecting a favorable arm.

Keep timed position error, local geometric path departure, progress lag and
final goal distance distinct. Reconstruct the logged command terms and gain/
error changes without rerunning the controller. A command decomposition is
an arithmetic identity, not an intervention proving unique physical causality.
Inspect truth contacts only for evaluation; the selected controller uses the
momentum observer and inferred support records. Check preparation weights,
targets recoverable from blends, command caps and the loaded-patch cutoff.
State explicitly which future-contact details were not recorded.

Source, prior archive and input hashes were preserved before this audit in
[local registration](../../../../results/belt_handoff_event_20261126/scope.json)
and its pre-change manifest. Current runtime/configuration files, old reports,
old media and old archives are protected. Slow replays will render saved
Gazebo states, not integrate new physics.

## What the saved records establish

The handoff interval contains two identifiable command-generation failures.
They do not uniquely explain the later, largest position excursion.

### Nearly parallel rolling directions generate infeasible references

At recorded **31.650 s**, the 0.30 m/s arm changes from belt B back to shell B;
both estimated and evaluated support identify shell B at that sample. There
have already been shell/belt switches since 31.030 s. The two world-XY rolling
directions available through the common shaft and shell B are only
**0.057775° apart**, with normalized allocation condition number **3850.507**.
Independent kinematics using the saved exact state and inferred contact point
reproduce the recorded allocation to within 4.7e-10 rad/s.

| Quantity at 31.650 s | Common shaft | Shell B |
| --- | ---: | ---: |
| Allocated joint-rate reference | +488.601 rad/s | −1760.525 rad/s |
| Actual joint rate | −0.0786 rad/s | −1.690 rad/s |
| Joint-velocity LQR contribution | +111.695 N m | −141.539 N m |
| Full requested torque | +111.389 N m | −141.067 N m |
| Applied torque after clipping | +1.500 N m | −0.600 N m |

The rolling reference is produced by the retained unconstrained least-squares
allocation. Its numerical scaling expresses preferences, not speed or torque
limits. Feedback then acts on the huge joint-speed error. Incoming preparation
weights and command deltas are zero at this sample; it is not the preparation
target that produces these torques. At 31.655 s the requested shaft/shell-B
torques reverse by −120.139/+150.690 N m. The gain/error decomposition assigns
almost all that jump to changing error/reference, rather than a gain jump.

This is a demonstrated controller/reference interface failure during
alternating support. It is not evidence that a motor should supply 141 N m,
nor proof that this one instant causes every later path error. The raised-cap
arms follow different histories and have well-conditioned allocation at
31.650 s: condition 1.163, drive angle 85.498°, references +0.498/−0.344 rad/s.
They still wobble and drift, so the singularity is not a universal explanation.

### Prepared and loaded commands are not continuous

The 0.60 and 0.90 m/s arms provide a separate example. Their saved body motion
is identical; retaining both records does not make two independent outcomes.

| Recorded time | Evaluated support | Estimated support | Belt-B command / action |
| --- | --- | --- | --- |
| 32.165 s | Belt B | Shell B | Preparation weight 1; −0.012112 m/s |
| 32.170 s | Belt B | Belt B | Preparation weight 0; −0.303361 m/s |
| 32.175 s | Belt B | Belt B | After the previous 5 ms hold, logged slip is 0.131466 m/s |
| 32.185 s | Shell B | Belt B | Belt request −0.451307 m/s |
| 32.190 s | Shell B | Shell B | Belt request returns to zero |

At 32.170 s, the belt request is exactly the new LQR term −0.319806 plus
rolling feedforward +0.016445 m/s. It is within the raised belt bounds. The
incoming belt had low evaluator-reported slip before this change: about
0.0000935 m/s at 32.165 s and 0.0000617 m/s at 32.170 s. Slip rises at the next
saved endpoint after the command jump. Current-contact endpoint diagnostics,
virtual-point residuals and future-point predictions are different quantities;
they must not be treated as interchangeable speed measurements.

Thus the records identify a **preparation-to-loaded-command discontinuity**
and short support-label disagreement/chatter. They do not establish that all
of the twitch is an impact, a computational delay or a failure to prepare the
belt. At 5 ms sampling, within-hold contacts and all 1 ms force solves are not
fully recorded. Lockstep Python computation does not itself advance physics.

## What synchronization means in the current controller

The existing preview extrapolates current generalized velocity for up to
**120 ms** over the observer-derived local tangent plane. It checks which
spherical patch is approaching exposure, prepares an unloaded patch if its
predicted plane gap is at most 20 mm, and blends the bounded preparation
command. It is a kinematic preview, not a forecast of contact force onset on
unknown terrain. Preparation fades to zero when that patch's estimated load
share reaches **5%**. In these runs the estimator often switches between
single-patch assignments, so a smooth formula can still receive a step input.

For a belt, it chooses a surface speed minimizing the predicted world
tangential contact velocity:

`v_contact = P (J(q) qdot) + a_belt(q) v_belt`.

Here `P` projects onto the estimated ground tangent plane and `a_belt` is the
belt's world tangent direction. Base motion, platform/common-shaft motion and
geometry are included. Belt speed in m/s must not be equated directly to shell
angular speed in rad/s; even a radius conversion must include the effective
contact lever and direction.

The code calculates an outgoing load-weighted surface velocity for diagnostics,
but **does not use that velocity in its target equations**. Its target is
incoming no-slip preparation, not explicit matching of outgoing measured slip.
That is not inherently a bug: copying outgoing slip could retain undesirable
sliding. More directly relevant here, the later loaded LQR/feedforward command
is not constrained to start at the prepared speed or change continuously.
The current implementation therefore cannot guarantee a smooth speed handoff.

The simulation prescribes belt surface speed. These records establish command
and contact behavior in that model, not independent encoder tracking of an
accelerating belt. Full per-preview future point, gap, chosen horizon and
residual dictionaries were not recorded in I-49. Active scalar targets can be
recovered from logged blend weights/deltas; their complete future geometry
cannot be recovered from those scalars alone.

Source: [allocation](../../../../src/spherical_wheel/experiments.py#L30),
[gain/command integration](../../../../src/spherical_wheel/offroad_control.py#L684),
[preview](../../../../src/spherical_wheel/synchronization.py#L154).
Line numbers refer to the preserved current source; exact file hashes are in
the pre-change and per-audit manifests.

## The large trajectory deviations are not one event

| Event | 0.30 m/s arm | 0.60 / 0.90 m/s arms |
| --- | ---: | ---: |
| First sustained 20 mm local departure | 6.860 s | 6.850 s |
| Largest timed error | 190.901 mm at 43.985 s | 110.009 mm at 39.630 s |
| Local geometric departure at that peak | 190.901 mm | 108.537 mm |

The late peak is a real geometric departure, not merely being late on the
timed figure-eight. The continuous late departure containing the baseline
peak crosses 20 mm at 38.640 s, 50 mm at 40.625 s, 100 mm at 42.110 s and
150 mm at 43.725 s. At the 50 mm crossing, evaluated and estimated support
both identify shell B, the estimated normal is essentially the logged DEM
normal, preparation weights are zero and requested torques
`[0.246, 0.00053, 0.0284] N m` are unsaturated. At 41 s the desired velocity
has become zero, but measured body velocity is approximately
`[−0.003, +0.061] m/s`; the position error is already 78.54 mm.

Consequently, late handoff/saturation cannot be the onset of that late drift.
The retained local balance/tracking loop fails to arrest an existing drift
even under correctly identified support and without clipping at those samples.
The allocation is also well conditioned at these late samples (normalized
condition numbers about 2–4, rather than 3850). The raised-arm 39.630 s peak
likewise has correctly identified shell-B support, unsaturated commands and
condition number 1.91.
Logged feedback groups show interacting position, attitude, velocity and
joint-rate corrections; arithmetic cancellation alone is not a proven bug or
a counterfactual explanation. A unique physical cause of the entire late
excursion remains unresolved by these histories. It would be misleading to
attribute it all to the 31.650 s singularity, observer latency or belt speed.

The early 6.8 s event likewise has its own chronology: belt-A requests clip
before the first sustained departure, and short support disagreement occurs
around 6.860 s. Earlier I-42 interventions did not remove that departure by
changing feedback shares alone; a coincident misassignment is not sufficient
evidence that it is the sole cause.

## Decision and next experiment

Keep the current baseline and all unsuccessful candidates preserved. Do not
adopt a faster belt, a faster contact filter or a new default from this audit.
The next controller experiments should separately address (1) infeasible
joint-rate references and their effect on the total bounded feedback command,
and (2) continuity from incoming preparation into loaded balance control.
Previously rejected uniform damping/bounded variants are not established fixes.
First use the identified event windows and controlled transfers, then require
the unchanged full-route and turn regression comparisons. Investigate the
late unsaturated drift as a separate balance/tracking-model question.

## Evidence and verification

[Quarter-speed handoff replay](../../../../results/belt_handoff_event_20261126/video_audit/belt_handoff_slow.mp4)
shows recorded 30.50–33.50 s with original playback clocks, evaluated versus
inferred support, preparation weights and requested/applied belt commands.
It renders original saved Gazebo states at 10 ms pose spacing; plots retain
all 5 ms samples. It creates no new physics or controller calls.

[Motion/support plot](../../../../results/belt_handoff_event_20261126/video_audit/event_motion.pdf)
· [Allocation and torque spike](../../../../results/belt_handoff_event_20261126/deviation_audit/plots_stage01/baseline_rate_and_torque_spike.pdf)
· [Late drift context](../../../../results/belt_handoff_event_20261126/deviation_audit/plots_stage01/late_drift_context.pdf)
· [Complete deviation audit](../../../../results/belt_handoff_event_20261126/deviation_audit/README.md)
· [Complete synchronization audit](../../../../results/belt_handoff_event_20261126/sync_audit/README.md)
· [Independent arithmetic checks](../../../../results/belt_handoff_event_20261126/review/independent_saved_data.json)
· [Protection and clock checks](../../../../results/belt_handoff_event_20261126/review/protection_and_clock.json)
· [Kinematic allocation reconstruction](../../../../results/belt_handoff_event_20261126/sync_audit/stage05_rolling_geometry/report.json).

Independent NumPy reconstruction checks requested command terms, the exact
LQR identity, final clipping and the 5% preparation cutoff across all three
saved traces. Separate interpretation review confirms the two command-origin
mechanisms and the limitation on late-drift causality. Media QA checks the
encoding, clocks and decoded overlays. These are saved-evidence checks, not
additional simulator trials or a rerun of the repository test suite.

Failed audit setup stages are retained: a mock-guard setup error before row
processing, a syntax error before the rolling-geometry audit, and an initial
slow-replay footer clipping corrected in a new render-source stage. A root
exploratory read used the wrong NPZ key and stopped before reductions. Earlier
source/log snapshots are preserved; no failed native run was replaced.
The compact deviation-summary script also had a syntax-only failure retained
before its corrected execution. No scientific reduction was overwritten.

[Compact evidence inventory](evidence/manifest.json)
· [Shareable source/data bundle](../../../../results/belt_handoff_event_20261126/presentation-pack.zip).
Old I-49 artifacts and all 18 prior presentation archives remain intact.

To repeat the saved-data audits, restore the workspace layout recorded in the
bundle and choose fresh output directories:

```bash
.venv/bin/python results/belt_handoff_event_20261126/repeat_saved_audit.py --mode deviation --output results/i50_repeat_deviation
.venv/bin/python results/belt_handoff_event_20261126/repeat_saved_audit.py --mode rolling --output results/i50_repeat_rolling
```

The wrapper preserves the original input roots and sources and redirects only
the output. It refuses existing output directories. The arithmetic audit uses NumPy; the
private-pose Jacobian reconstruction additionally requires the original mirror
XML/meshes and MuJoCo. It does not call a physics integrator or query contacts.
The native 44 s runs are not rerun by these commands.
