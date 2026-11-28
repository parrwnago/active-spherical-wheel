# I-75 — heading follows the desired S-path tangent

Status: **completed and analyzed; experimental, not adopted**, 27 November 2026.
Six fresh Gazebo/DART episodes compare the unchanged native controller N with
trajectory-facing heading T. T points along the desired path much better, but
moving position error increases at every start. All runs complete 45 s without
a declared physical failure; 15 of 60 fixed control checks fail despite all
212 setup checks passing. Completion is separate from acceptance.

[Motion/heading comparison](evidence/publication/plots_stage01/motion_comparison.png) ·
[Goal hold](evidence/publication/plots_stage01/goal_hold.png) ·
[Two-page PDF](evidence/publication/plots_stage01/tangent_heading_comparison.pdf) ·
[Canonical numerical report](evidence/evaluation/completed_stage01/science.json)

| Start phase | Heading RMS N→T, degrees | Moving XY RMS N→T, mm | Final XY error N→T, mm |
|---|---:|---:|---:|
| −1.0 rad | 78.26→9.43 | 19.66→33.82 | 3.11→81.93 |
| +0.3 rad | 96.52→22.87 | 18.07→24.14 | 11.77→56.99 |
| +0.9 rad | 94.45→23.61 | 20.08→27.01 | 41.91→26.11 |

Heading RMS uses raw desired geometric tangent versus actual Euler heading over
3–45 s. Moving XY RMS uses the common **timed** reference over 3–35 s; this is
not nearest-path distance. Final errors are at 45 s. Better pointing is real,
but it does not mean better path tracking or successful return to the goal.

## Comparison conditions and exact intervention

Use the prior fixed S route on the moderate faceted Mars site: origin
`[-2.4,10]` m, rotation 113°, local `[1.2u,.14 sin(2πu)]`, quintic progress.
Each episode has 3 s initial hold, 32 s motion and 10 s final hold, with
5 ms control / 1 ms physics. No new site selection or terrain input enters
control. Requested shaft phases are −1.0, +0.3 and +0.9 rad.

**Both N and T now start with yaw argument 149.242843°**, the initial geometric
path tangent. This removes the approximately 104° startup mismatch of the old
45° starts. These are new paired native baselines; I-73/I-74 native numbers
are not matched comparisons for this experiment. Actual initial heading error
is approximately .04292° in all six episodes, and paired initial q/v agree exactly.

Original LQR/gain bytes, momentum observer, geometry support locator, inferred
contact/normal/load/gain routing, rolling/holding feedforward, incoming-drive
preparation and final input caps remain unchanged. N has heading helper off.
T is a result-owned adapter of the existing unloaded-shell helper, with gain
.2 N m/rad, damping .06 N m s/rad, helper cap .15 N m and horizontal reaction
budget .05 N m. Original load/lever eligibility and ±.5 rad used-error clamp
remain. Final total caps stay `[1.5,.6,.6 N m,.3,.3 m/s]`; the reaction budget
bounds the added helper, not every balance/rolling correction.

T changes the desired heading to the **planned geometric tangent**, not the
measured drift direction. Its desired rate is the derivative of that target.
For body-forward axis `h = R[:,0]` and `hdot = omega_world × h`, measured Euler
heading rate is `(hx*hdot_y − hy*hdot_x)/(hx²+hy²)`. If the denominator is at
most 1e−8, the helper is skipped and invalidity is logged. Physical world-z
angular velocity is logged separately; it is not substituted for Euler rate.

The analytic target exists at rest, is continuous through both holds, and
retains the planned endpoint tangent after motion. Raw and commanded headings
are identical: no active governor, smoothing lag or hidden easier target.
The 9,001-sample check gives peak rate .231862 rad/s and acceleration
.0692701 rad/s², below preregistered .35/.5 guards. Conservative analytic
bounds also pass. These are reference-design guards, not proof of actuator
feasibility. The S route contains no reversal of progress; this experiment
does not establish a general turn-around versus backward-driving policy.

**Predeclared scope limitation:** native LQR body-angular-velocity references
remain exactly zero, and the original rolling allocation handles translation
only. This is a heading-helper overlay. It does not coordinate a complete
planar-pose reference through LQR and rolling kinematics. Existing rate feedback
can oppose intended turns; that risk was recorded before outcomes. Neither a
failed helper screen nor sideways travel proves a mechanical holonomic or
nonholonomic classification.

## Results and acceptance

Before physics, every T phase was required to improve raw heading-error RMS
and exact Euler heading-rate tracking RMS by at least 20%, with fixed position,
lean/tilt, speed/50 ms acceleration, clipping and command-step margins. Absolute
rotation was descriptive because turning is intended. Settling was separately
descriptive; it cannot replace failed tracking or command gates.

All three heading RMS benefits pass. Rate-error RMS N→T is
.20181→.06997, .11068→.11209 and .36201→.16983 rad/s: two of three pass.
All T helper activity requirements pass, with 26.215/44.005/43.200 s of
pre-fade/clipping nonzero helper activity. This establishes execution, not a
complete same-state applied-command counterfactual.

The fixed screen is **FAIL_TANGENT_SCREEN**, with 45/60 control checks passing:

- **−1.0:** moving RMS, full peak and final position errors exceed their margins;
  raw-reference lean RMS also regresses. T becomes almost stationary about
  81.93 mm from the goal, despite final heading error only 5.41°.
- **+0.3:** heading-rate RMS benefit fails; moving RMS, peak/final position,
  acceleration peak and command-step margins fail. T ends 56.99 mm from goal
  with 17.68° heading error.
- **+0.9:** moving RMS, acceleration peak, Euler-rate-error peak, normalized
  request-excess RMS and command-step margins fail. Final position does improve
  to 26.11 mm and heading error to 7.46°, but the command transient remains.

Only +0.3 N passes the original four simultaneous final-2 s criteria: goal
error ≤20 mm, XY speed ≤.02 m/s, absolute physical world-z angular speed
≤.05 rad/s and raw-reference up-vector error ≤2°. No T passes all four;
no case passes the five-condition version adding ≤10° raw tangent error.
Do not call a nearly stationary offset a successful goal return.

Any-channel clipping fractions N/T are .100%/0%, 0%/.0889%, and
.200%/.2556%. The +0.9 normalized request-excess RMS nevertheless rises
.00670→.39975, and peak normalized applied-command step rises 1.184→3.014.
Rare large requests matter even when clipping frequency is small. Actual
commands stay inside the registered final caps in every case.

## What the bounded posthoc audit establishes

The separately sealed [audit](evidence/evaluation/spike_audit_stage01/report.json)
selects each case's largest normalized clipping excess and fixed ±25 ms context.
An earliest zero tie is explicitly marked as no clipping, not called a spike.
It reconstructs full-record command ledgers and LQR error blocks without new
physics, inference, thresholds or acceptance changes.

At **15.515 s in +0.9 T**, rolling references for shaft/shell B are
**−74.107/+206.791 rad/s**, versus actual +3.510/−7.594 rad/s. The associated
LQR joint-rate blocks contribute approximately −17.740/+17.401 N m on those
motors. Requested motor torques are `[−18.290,.790,17.551]` N m; actually
applied torques are `[−1.5,.6,.6]` N m. The direct heading helper contributes
only `[0,.127,0]` N m. Rolling additive feedforward, preparation delta and fade
delta are zero at this row.

This localizes the large demand to **rolling joint-speed references entering
LQR velocity feedback**, rather than a large direct heading torque. It does
not establish the full physical reason that those references become large.
No new contact Jacobian or conditioning analysis is computed in this audit,
and no singularity, contact-timing failure or whole-path causal claim follows.
The derived rolling shaft-position error block is labeled separately from
measured shaft angle.

All six raw and feedback LQR body-angular references are verified exactly zero.
A moving-window audit finds the angular-rate feedback motor vector sometimes
opposes the helper motor vector. These are command-coordinate alignments,
not inferred net physical yaw torque; they do not prove yaw conflict causes
the regressions. A coordinated reference intervention would be a new experiment.

At 35 and 45 s, −1.0 T has no eligible unloaded shell and vertical lever about
.0138; its helper is zero while the position offset stays about 81.94→81.93 mm.
In +0.3 T, a free shell remains eligible: the 35 s held helper is about
.0704 N m and the 45 s emitted, unheld helper about .0686 N m, yet position
error is 53.89→56.99 mm. The residual offsets therefore cannot all
be explained by loss of helper eligibility.

**Decision:** retain original native N as the selected controller; preserve T
as experimental. Pointing improves substantially but every moving XY RMS
worsens, with failed stopping/command guards. No post-result tuning or automatic
promotion. This does not reject trajectory-facing heading as a concept; it
limits the present overlay and leaves full reference coordination untested.
No camera, solver, HMM, particle filter, hardware evidence or new dynamic-contact
predictor is introduced.

## Verification and preserved setup history

Five unique reference contracts pass. Twelve focused adapter/runner functions
pass: eight adapter, three protocol and one real-controller identity function
with four included subtests. Native identity includes tilted/angular-rate states;
T equivalence to the original constant-target helper is scoped to level/pure-yaw,
zero-reference-rate cases. Twenty-eight final evaluation functions pass, inclusive
of the earlier 27. Seven media functions include five reused and two new checks;
six package/restore functions are reused. These scoped counts are not the full
repository suite, and subtests or algebraic identities are not added to them.

Preserved failures are setup/development history: a reference loader protocol-key
mismatch before arithmetic, a test launcher resolving the venv symlink into
system Python and failing imports, and a draft evaluator docstring syntax typo.
The loader/launcher/draft corrections are retained with source/log snapshots.
No failed physical episode or changed control law is hidden by those fixes.

Two video processes also fail during post-encode random-seek decoder
initialization, after all frames are encoded. Exact resource cause is
unconfirmed; concurrent decoder pressure is a hypothesis. Their failure
receipts, source and small artifacts are preserved in
[media setup history](evidence/publication/video_tools/setup_failures.json).
A distinct sequential single-thread
[recovery](evidence/publication/video_decode_recovery_stage01/report.json)
decodes all 2,703 frames and verifies original encoded bytes unchanged.
No rendering, physics or control is rerun. Final source/input, frame-count and
visual QA passes; terminal 45 s helper output is explicitly logged but unheld.

Before physics, protocol
`d579e1a21890b2da48dbd5c664ddbb692a1bb5c67f112262fbdf2b95500671ee`
binds 83 control/runtime/runner sources and 17 inputs, including original gains,
plugins and portable fixture provenance. Output seal
`8bacbf437eadc69eb4210f1c8f0d3d3f290ce437795da343b8f3a6aa5b2d892d`
binds all 48 case artifacts before outcome analysis. These bindings remain
unchanged. Pure evaluation sources/tests and their input seal precede reads of
the new case arrays; canonical evaluation runs once. The posthoc audit has its
own later seal and does not change the canonical result.

## Videos and reproduction

Each full video compares **N / T** side by side, covering saved Gazebo time
0–45 s including the full final hold, from one static camera covering all six
paths. Recorded full-precision poses are rendered without new physics. At
20 fps, 901 inclusive endpoint frames encode as 45.05 s. The HUD distinguishes
raw/command heading, actual Euler heading rate, desired rate, physical world-z
rate, body tilt/reference lean/up-vector error, helper activity and XY error.

- [−1.0 rad comparison](../../../../results/tangent_heading_control_20261127/publication/videos_stage01/m100/m100_N_T.mp4)
- [+0.3 rad comparison](../../../../results/tangent_heading_control_20261127/publication/videos_stage01/p030/p030_N_T.mp4)
- [+0.9 rad comparison](../../../../results/tangent_heading_control_20261127/publication/videos_stage01/p090/p090_N_T.mp4)
- [Decoded first/middle/final frames](evidence/publication/video_quality_stage01/decoded_contact_sheet.jpg)
- [Media quality report](evidence/publication/video_quality_stage01/report.json)

Local work: `results/tangent_heading_control_20261127/`. Compact tracked evidence
is under this page's `evidence/`. Large precision files, rollouts, full-rate
signals and videos are in the companion
[lossless raw/media bundle](../../../../results/tangent_heading_control_20261127/publication/i75-tangent-heading-control-raw-media.tar.gz).
Its [inventory and hashes](evidence/publication/local_raw_bundle_inventory.json)
identify every member. Share the bundle with the repository. Plot-only 50 ms
projections do not reproduce full-rate acceptance metrics. Historical I-74
protocol/fixture provenance is included; old native raw replays are not required
because I-75 uses a newly aligned initial yaw for both policies.

From a clean checkout, after obtaining the bundle, restore the result-owned
files and optionally the frozen original 52 KB gain cache:

```bash
.venv/bin/python docs/iterations/2026-11-27/tangent-heading-control/evidence/publication/package_tools/restore_record.py --evidence docs/iterations/2026-11-27/tangent-heading-control/evidence --repo-root . --raw-bundle /path/to/i75-tangent-heading-control-raw-media.tar.gz --install-gain-cache
```

Restore refuses conflicting bytes and installs no live source or runtime plugin.
The two runtime `.so` members are verification payloads. See
[dependency bindings](evidence/publication/runtime_dependency_bindings.json)
and [Gazebo setup](../../../../GAZEBO.md). `--check-runtime` verifies the frozen
requirements; changing those requirements creates a newly identified experiment.

Re-evaluate saved full-rate arrays into a **new** directory:

```bash
.venv/bin/python results/tangent_heading_control_20261127/evaluation/evaluate.py --protocol results/tangent_heading_control_20261127/protocol/protocol.json --runs-root results/tangent_heading_control_20261127/campaign/runs --all-runs-seal results/tangent_heading_control_20261127/campaign/outputs_sealed.json --output results/tangent_heading_control_20261127/evaluation/reproduced_stage01
```

For fresh physics in a separate restored checkout, keep the historical campaign
under a distinct name first. Use this only if `campaign_original_i75` does not
already exist:

```bash
mv results/tangent_heading_control_20261127/campaign results/tangent_heading_control_20261127/campaign_original_i75
.venv/bin/python results/tangent_heading_control_20261127/campaign_tools/launch.py
```

The launcher uses the isolated Gazebo micromamba environment with sanitized
Python/library paths, BLAS/OpenMP threads fixed to one, one fresh process per
case and three cases at a time. Existing case directories are refused. New
physics is a new episode, not guaranteed cross-machine bit identity.

Final [archive/restore QA](archive_verification/receipt.json) verifies all
503 bound compact files (35.92 MB, plus the manifest), all 35 members of the
114.89 MB raw/media bundle, and exact 50 ms plot-only subsets retaining the
terminal state. An actual isolated restore recreates 503 compact files,
33 result-owned raw/video files and the frozen gain cache. Unrelated sentinel
bytes remain unchanged; no live source or plugin is installed. Root independently
rechecks every compact file hash and size. The immutable compact manifest SHA is
`07d0a24392cda7fd289f1bef12a75d9b3fef8d9c78137728d3cd960265ca5fe2`;
bundle SHA is
`a177600220f0d12d24b4ef38361d25341e284abfe5f583698f33bec8f01e5ccd`.
Final QA receipts sit beside the evidence tree rather than modifying it.
