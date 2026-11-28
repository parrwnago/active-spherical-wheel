# I-76 — trajectory-facing heading across three routes

Status: **completed and analyzed; experimental, not adopted**, 27 November 2026.
All eighteen corrected Gazebo/DART episodes complete their full schedules.
Heading RMS improves in eight of nine comparisons, but timed position RMS in
three and geometric path RMS in four. The figure-eight shows real path-following
benefits; straight and arc do not show a consistent gain. The fixed screen is
**FAIL_TANGENT_SCREEN**: 165/207 control checks pass, with 42 failures. All
686 source/reference/state consistency checks pass. The eight remains outside
the original moderate-terrain screen under its explicit stress classification.
Completion and consistency are separate from acceptance.

[Planned routes](evidence/publication/design_stage01/route_design.png) ·
[Six-page results PDF](evidence/publication/plots_stage01/tangent_routes_comparison.pdf) ·
[Canonical report](evidence/evaluation/completed_stage01/science.json) ·
[Command audit](evidence/evaluation/command_audit_stage01/report.json)

## All nine matched comparisons

N is the unchanged native LQR/observer/contact-feedforward baseline; T adds the
same bounded trajectory-facing heading overlay tested in I-75. Arrows are N→T.

| Route / shaft phase, rad | Heading RMS, degrees | Moving timed XY RMS, mm | Geometric path RMS, mm | Final goal error, mm |
|---|---:|---:|---:|---:|
| straight −1.0 | 69.08→34.46 | 42.42→59.32 | 39.08→56.29 | 18.74→22.69 |
| straight +0.3 | 76.25→17.89 | 35.81→34.67 | 32.57→29.23 | 1.26→52.38 |
| straight +0.9 | 91.35→9.64 | 29.35→40.83 | 18.32→22.78 | 2.13→0.54 |
| arc −1.0 | 26.07→13.98 | 16.91→24.87 | 10.91→12.60 | 1.96→8.67 |
| arc +0.3 | 38.24→12.74 | 13.62→58.12 | 7.75→12.62 | 13.24→133.01 |
| arc +0.9 | 83.91→18.50 | 24.10→25.28 | 13.54→21.14 | 7.28→40.68 |
| eight −1.0 | 96.02→76.96 | 46.36→28.66 | 39.87→15.54 | 32.88→42.13 |
| eight +0.3 | 88.84→90.89 | 44.53→42.58 | 27.91→22.22 | 0.30→28.32 |
| eight +0.9 | 101.15→51.70 | 62.91→69.54 | 32.73→17.21 | 0.06→12.74 |

Heading RMS covers 3 s through the actual terminal sample. Moving errors cover
[3,35) s for straight/arc and [3,123) s for eight. Geometric distance is the
exact point-to-segment minimum on the full fixed 4,097-point uniform-parameter
polyline, not exact analytic-curve distance. At the eight's crossing it can hide
incorrect progress; timed error remains independent. Goals and the entire
planned path are retained throughout the comparison.

The figure-eight's geometric RMS decreases in **all three** starts:
39.87→15.54, 27.91→22.22 and 32.73→17.21 mm. Timed RMS improves in two;
+0.9 increases 62.91→69.54 mm despite better geometric path distance.
Straight improves timed/geometric RMS only at +0.3; arc improves neither metric
at any start. This supports investigating the idea further without claiming
that the current implementation is better in general.

Seven of nine cases achieve the registered 20% heading-RMS benefit, and six
achieve the Euler-rate-RMS benefit. No pass relies on the absolute floors.
Eight −1 improves heading RMS by 19.846%, missing the fixed 20% margin by
.148° RMS; its other 22 checks pass. Keep that recorded failure rather than
changing the threshold after results. No pair passes every check.

## Stopping and transient regressions

Original four-condition settling requires the final 2 s to satisfy goal error
≤20 mm, XY speed ≤.02 m/s, absolute physical world-z angular speed ≤.05 rad/s
and raw-reference up-vector error ≤2°, simultaneously. N passes arc +0.9,
eight +0.3 and eight +0.9. T passes straight +0.9, arc −1 and eight +0.9.
Both have three successes, but T loses two native successes and gains different
ones. Requiring raw heading error ≤10° throughout the same final 2 s leaves no
N successes and two T successes:
straight +0.9 and eight +0.9.

Final T heading errors are 57.60/27.15/4.27° on straight,
19.02/5.01/44.57° on arc and 61.09/105.74/3.39° on eight. Better average
pointing does not guarantee correct final heading or a successful stop.

The previously favorable −1 start is not uniformly best on the new routes.
The straight −1 T transient is substantial: peak tilt **19.89°**, XY speed
**1.09 m/s** and 50 ms 3D acceleration **14.12 m/s²**, with Euler-rate error
peak 4.725 rad/s. Its moving error worsens and the full video retains the event.
All actual commands remain inside their final caps. Failures split 18 on
straight, 14 on arc and 10 on eight; all nine T helper-activity requirements pass.

## Same control law, different planned routes

Original gain-scheduled LQR, momentum observer, geometry support estimator,
inferred load/normal/gain routing, rolling/holding feedforward, incoming-drive
preparation and final `[1.5,.6,.6 N m,.3,.3 m/s]` bounds remain unchanged.
Body state is exact simulated state; live contact inputs are inferred.
No solved-contact/terrain query enters command generation. No camera, new
estimator, particle filter, solver change or dynamic contact forecast is added.
The heading target is a planned trajectory reference, not contact prediction.

T uses helper gain .2 N m/rad, damping .06 N m s/rad, cap .15 N m,
horizontal reaction budget .05 N m and existing free-shell/lever eligibility
and ±.5 rad used-error clamp. Its angle/rate reference is the planned geometric
tangent; measured rate is the exact Euler-heading derivative, with physical
world-z angular velocity separately logged. Raw and command targets coincide;
there is no active reference governor or hidden easier objective.

The LQR body-angular-rate reference remains zero and rolling allocation handles
translation only. This is the same minimal overlay as I-75, not a coordinated
planar-pose controller. The experiment does not reject trajectory-facing control
as a concept or establish mechanical holonomy. Retain original N selected and
T experimental; there is no post-result tuning or default promotion.

All routes use a fixed site anchor `[-2.4,10]` m, rotation 113°, quintic progress,
3 s initial and 10 s final holds. Local shapes and motion durations are:

- Straight: `[1.2u,0]`, 32 s motion / 45 s total, initial yaw argument 113°.
- Arc: `[.6sin(pi*u/2),.6(1−cos(pi*u/2))]`, 32 / 45 s, initial yaw 113°.
- Eight: `[.5sin(2pi*u),.25sin(4pi*u)]`, 120 / 133 s, initial yaw 158°;
  its planned origin is shifted local `[.5,0]` m to
  `[-2.595365564244637,10.46025242672622]`.

Each N/T pair shares its exact initial q/v and last input, facing the initial
geometric tangent. The eight starts/ends at its crossing with a regular tangent.
Its longer schedule keeps the same .35 rad/s/.5 rad/s² reference guards; sampled
rate peaks are 0/.09204/.29677 rad/s and speed peaks .07031/.05522/.06942 m/s.
Planned total heading variation from 4,097 uniform-parameter tangent samples
is approximately 0°/90°/540°; the eight returns
to its initial heading. Shapes, distances, durations and initial terrain differ.
Only within-route/phase pairs isolate the heading intervention; this is not an
equal-difficulty or exhaustive generality study.

## Terrain setup history and honest stress classification

The original moderate fixture screen stays 12° maximum sampled corridor facet,
80 mm total height relief, .15 m corridor and ≥1 m projected feature clearance.
Straight passes at 48.08 mm /11.288°, arc at 66.26 mm /10.668°.
The initial eight fails at 111.37 mm /12.385°; a half-metre forward shift still
fails at 96.72 mm /12.385°. A preregistered finite six-placement map-only scan
also finds no pass; selection is null and every candidate remains recorded.

Before any controller outcome was read, the experiment domain was explicitly
revised: retain that shifted eight as **rough stress outside the moderate screen**.
Its slope/relief gates remain failed (pass flags false); DEM coverage and feature clearance
pass. The strict runner allows only this declared eight exception and still
requires straight/arc to pass. This does not convert a failed moderate screen
into a pass. No scan winner, controller gain or numeric performance gate is
chosen after outcomes. Final videos label the eight's stress classification.

## Numerical correction, failed attempt and client recovery

The first dynamic attempt is unscored and preserved under `failed_attempt01/`:
12 processes finish, three eight cases hit a reference exception, and three
remaining cases are explicitly aborted after that setup error. The next clock
122.99999999996062 s lies just below the 123 s endpoint, but cancellation in
expanded quintic progress produces `1.0000000000000009`. Strict geometry-domain
validation then throws. This is numerical reference setup failure, not a physical
controller-failure verdict. Aborted in-memory state arrays were not saved;
that limitation and the targeted termination are disclosed.

The minimal correction clips only computed progress to its declared `[0,1]`
range, preserving derivative arithmetic, geometry, timing and control settings.
Adversarial observed-clock/ULP contracts pass. Stored fixture XY, velocity,
acceleration and seven ground points match exactly at every sampled point;
no new terrain query is made. All eighteen cases are restarted under one
corrected protocol; old-attempt recordings are not pooled into this comparison.

After an accidental VS Code closure, six complete **corrected** straight runs
are recovered. Their actual exit0 records had been observed in prior launcher
tool stdout and are preserved verbatim, with independent artifact/provenance/
completion checks. Three interrupted arc logs/source records are retained with
unsaved in-memory states disclosed; twelve cases resume unchanged. The final
seal contains observed statuses for all eighteen, not invented recovery exits.

The [numeric replay](evidence/evaluation/retry_numeric_replay_stage01/report.json)
compares every stored array field: all twelve old complete recordings and three
saved precision prefixes match exactly, as do three truth prefixes. The three
unsaved aborts are excluded. Original partial files were not listed in the first
launcher's 120-file seal; they receive explicit later bindings before replay
comparison. Earlier diagnosis read only their time arrays. This is correction
reproducibility, not an additional software-test count or adoption criterion.
Logical relocation metadata preserves how earlier source paths should be
restored in a separate fixture; current files are not called the old tested source.

## Command audit: different events have different sources

The descriptive [audit](evidence/evaluation/command_audit_stage01/report.json)
was defined during collection before outcome access. It examines every case's
maximum normalized held clipping excess and fixed ±25 ms context, with zero
ties explicitly called no clipping. All 144 full-record ledger identities verify;
these are algebraic checks, not additional unit tests.

In straight −1 N at 20.585 s, rolling references shaft +26.056 /shell B
−91.896 rad/s generate large joint-rate LQR terms. Requested motors
`[4.741,−.167,−7.190]` N m clip to `[1.5,−.167,−.6]`.
In straight −1 T at 20.975 s, reference rates are ordinary (.544/0/.183 rad/s),
but attitude, linear-velocity and angular-velocity feedback dominate the shell B
request −7.272 N m, clipped to −.6. The direct helper is only +.107 N m on A.
The two events therefore cannot both be attributed to huge rolling references.
Small direct helper torque does not establish why the different closed-loop
states arose, and the audit does not prove a contact singularity or full cause.

Eight maximum normalized clipping excess N/T is 7.367/.327, 3.242/3.175 and
5.503/1.973. The +0.3/+0.9 T selected events do contain dominant joint-rate
reference terms. Preserve the distinct outcomes and all capped actual inputs.

## Verification and reproduction

Final focused function counts are 10 reference, 13 adapter/runner, 34 evaluation,
9 media, 7 package/restore and one additional metadata-coverage contract;
earlier stages and included subtests are not
added. These are scoped checks, not the repository-wide suite. Final protocol
`53106cdb758d76436e04b747c9483c4081abcc7ec1f2a163ace90b58b82085c2`
binds 86 source and 52 input files. All original gain-cache/model/plugin bytes
remain. Output seal
`e5e65236947d7b1945c90b3a74a133f055495851d1938b42dacd0ac26602fc2a`
binds 149 files before scoring, including recovery evidence. Evaluation runs
once with source/input seals and fixed criteria. Plots render saved signals and
full planned paths; videos render saved full-precision Gazebo poses without
new dynamics. First-attempt, setup and client-interruption histories remain.

Each full video compares N left / T right at 10 fps and includes the full actual
goal hold: 451 frames for 45 s cases, 1,331 for 133 s eight cases. Inclusive
endpoint encoding lasts 45.1/133.1 s. One static camera covers all eighteen paths;
body tilt, reference lean/up error, Euler versus world-z rates and raw heading
error are shown. Terminal commands are explicitly emitted but unheld.

- straight: [−1.0 rad](../../../../results/tangent_heading_routes_20261127/publication/videos_stage01/straight/m100/straight_m100_N_T.mp4) · [+0.3 rad](../../../../results/tangent_heading_routes_20261127/publication/videos_stage01/straight/p030/straight_p030_N_T.mp4) · [+0.9 rad](../../../../results/tangent_heading_routes_20261127/publication/videos_stage01/straight/p090/straight_p090_N_T.mp4)
- arc: [−1.0 rad](../../../../results/tangent_heading_routes_20261127/publication/videos_stage01/arc/m100/arc_m100_N_T.mp4) · [+0.3 rad](../../../../results/tangent_heading_routes_20261127/publication/videos_stage01/arc/p030/arc_p030_N_T.mp4) · [+0.9 rad](../../../../results/tangent_heading_routes_20261127/publication/videos_stage01/arc/p090/arc_p090_N_T.mp4)
- eight: [−1.0 rad](../../../../results/tangent_heading_routes_20261127/publication/videos_stage01/eight/m100/eight_m100_N_T.mp4) · [+0.3 rad](../../../../results/tangent_heading_routes_20261127/publication/videos_stage01/eight/p030/eight_p030_N_T.mp4) · [+0.9 rad](../../../../results/tangent_heading_routes_20261127/publication/videos_stage01/eight/p090/eight_p090_N_T.mp4)

Videos and full-rate arrays remain local or in the companion
[lossless raw/media bundle](../../../../results/tangent_heading_routes_20261127/publication/i76-tangent-heading-routes-raw-media.tar.gz).
[Inventory and hashes](evidence/publication/local_raw_bundle_inventory.json)
identify all members. Share that bundle with the repository; plot-only subsets
are not sufficient to recalculate acceptance metrics.

Restore into a clean checkout after obtaining the bundle:

```bash
.venv/bin/python docs/iterations/2026-11-27/tangent-heading-routes/evidence/publication/package_tools/restore_record.py --evidence docs/iterations/2026-11-27/tangent-heading-routes/evidence --repo-root . --raw-bundle /path/to/i76-tangent-heading-routes-raw-media.tar.gz --install-gain-cache
```

The restore refuses conflicting bytes and installs no live source or runtime
plugin. See [Gazebo setup](../../../../GAZEBO.md) and the archived dependency
bindings. Frozen runtime binaries are verification payloads; `--check-runtime`
checks the historical environment. Different source/binary requirements define
a new experiment.

Re-evaluate full-rate saved arrays into a new directory:

```bash
.venv/bin/python results/tangent_heading_routes_20261127/evaluation/evaluate.py --protocol results/tangent_heading_routes_20261127/protocol/protocol.json --runs-root results/tangent_heading_routes_20261127/campaign/runs --all-runs-seal results/tangent_heading_routes_20261127/campaign/outputs_sealed.json --output results/tangent_heading_routes_20261127/evaluation/reproduced_stage01
```

For fresh physics in a separate restored checkout, preserve the historical
campaign first; use this only if `campaign_original_i76` does not already exist:

```bash
mv results/tangent_heading_routes_20261127/campaign results/tangent_heading_routes_20261127/campaign_original_i76
.venv/bin/python results/tangent_heading_routes_20261127/campaign_tools/launch.py
```

The original launcher runs all eighteen fresh with three isolated Gazebo
processes, sanitized Python/library paths and BLAS/OpenMP threads fixed to one.
Do not use the historical recovery wrapper for a fresh campaign; it is evidence
of the specific interruption. Fresh physics is not guaranteed cross-machine
bit identity. An initial packaging coverage omission is retained explicitly: `.jsonl` was
missing from the small-file suffix set, so recovered launcher-status evidence
would not be restored. A separate metadata amendment preserves the original
compact manifest and raw bundle, copies the missing JSONL exactly and records
the correction source/receipt before rebuilding the final compact manifest.
The same coverage review finds 18 per-case model YAML snapshots omitted by
the top-level scan. They are added exactly and checked against sealed prephysics
provenance and the protocol model hash. One regression contract covers missing
or corrupt JSONL and YAML. Final [archive and actual-restore QA](archive_verification/receipt.json) passes
for all 3,633 bound compact files (129,468,477 bytes, plus the manifest), all
7,746 members of the 1,180,265,241-byte companion bundle and all eighteen exact
50 ms plot-only subsets with real terminal samples retained. An actual isolated
restore reproduces 7,744 result-owned raw files, the compact files and frozen
gain cache. An unrelated sentinel stays unchanged; live source and runtime
plugins are not installed.

The separate [restored prerequisite closure](archive_verification/closure/receipt.json)
verifies all 149 sealed outputs plus 18 provenance-bound model YAMLs (167 unique
essential prerequisites), both amended JSONLs and all 223 registered result-owned
producer inputs. The initial incomplete compact manifest and amendment sources/
receipts remain preserved inside evidence; the original builder and raw tar are
unchanged. Root independently verifies every final compact hash/size and field set.
Final QA receipts sit beside the immutable evidence tree.

Final compact manifest SHA:
`76ecf09b6a9a7fe46c5ddc9bc5d190ef6c522f56ecb94ceb7fea8bab80a4a638`.
Raw bundle SHA:
`259932a1f79d03552535276a30c8e45ac5268d7d40870baf0def4d6cdfdd2643`.
