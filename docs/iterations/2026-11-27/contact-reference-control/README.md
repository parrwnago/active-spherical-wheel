# I-73 — fresh S-path rolling-reference comparison

Status: **completed and analyzed; candidate not adopted**, 27 November 2026.
All nine fresh Gazebo/DART runs complete 45 s without a declared physical
failure. Geometry-plus-velocity C improves the −1.0 rad tracking result,
regresses at +0.3 rad and changes little at +0.9 rad. It fails 12 of 36 fixed
control-screen checks. None of the nine runs meets the simultaneous final-2 s
settling definition; this does **not** mean none returns near the goal.

[Motion comparison](evidence/publication/plots_stage01/motion_comparison.png) ·
[Goal-hold comparison](evidence/publication/plots_stage01/goal_hold.png) ·
[Two-page comparison PDF](evidence/publication/plots_stage01/control_comparison.pdf) ·
[Full numerical report](evidence/evaluation/completed_stage01/science.json)

| Start phase | Native N RMS | Velocity A RMS | Geometry + velocity C RMS |
|---|---:|---:|---:|
| −1.0 rad | 47.31 mm | 44.39 mm | 45.38 mm |
| +0.3 rad | 22.85 mm | 27.05 mm | 27.53 mm |
| +0.9 rad | 23.37 mm | 23.53 mm | 23.47 mm |

These are **timed XY RMS errors during motion**, not geometric path distance.
The same timed reference is used everywhere. At −1.0 rad, C's full-run peak
XY error decreases from 179.59 to 151.08 mm, but its late world-vertical angular
speed peak increases from 0.1515 to 0.9714 rad/s. At +0.3 rad, C's final error
increases from 7.02 to 28.42 mm; at +0.9 rad it is 44.12 versus 41.41 mm.
There is no consistent improvement across configurations.
Native −1.0 is also the worst moving-RMS phase on this S, unlike its favorable
ranking in some earlier figure-eight trials. Starting-phase advantage is not
universal. Because both route and site differ, this comparison cannot attribute
that ranking change to either factor alone.

The user approved a fresh S-shaped or previous figure-eight route. This test
uses a fixed S followed by a 10 s goal hold, comparing native observer contact
records (N), fixed guarded velocity selection (A), and fixed geometry-first
velocity selection (C), at shaft phases −1.0, +0.3, +0.9 rad. The robot starts at
45° yaw. These are new Gazebo/DART episodes, separate from I-72's saved-log study.

Only the stateless rolling-reference/feedforward call receives selected physical
candidate points. Native observer force, contact load shares, filtered normal,
gain selection, support torque and incoming-drive preparation stay native.
Joint velocity references enter LQR feedback, so this is more than additive
feedforward. A selected point carries no manufactured force/load fields.
Contact truth is collected after control, for evaluation only.

All methods keep the original flat gain bank, 5 ms feedback / 1 ms physics,
least-squares rolling allocation and `[1.5, .6, .6 N m, .3, .3 m/s]` bounds.
The reference uses 3 s initial hold, 32 s quintic progress and 10 s final hold.
No reference reanchoring, controller tuning, new gain design or threshold tuning
is included. In the final zero-velocity hold the rolling-selector hook is
bypassed; settling measures arrival and retained history, not direct new hold
commands from the contact selector.

The initial phase is the requested ground-relative configuration; the common
placement routine adjusts the literal shaft joint angle for the local normal.
All same-phase initial physical states and prior commands match byte-exactly.
The offline placement uses the known terrain to establish a balanced starting
fixture; online control gets exact simulated body/joint state and no DEM or
solved contacts. The selected heading is free: there is no commanded yaw hold.
Residual rotation is therefore a separate motion limitation, not proof of a
fall or a failure to achieve the intended lean.

The first proposed site was rejected by map-only screens before physics:
the 0.15 m corridor reached 14.09° facet slope and 109.3 mm relief, beyond the
original 8° / 80 mm screens. Two registered finite grids of 25 sites each also
failed the 8° facet screen. All failed screens are retained. Before any physics,
the facet screen was explicitly revised to 12° and the already inspected
least-steep site from the first grid selected: origin (−2.4, 10.0) m, maximum
11.288°, relief 75.69 mm and projected feature clearance at least 1 m. Other
screens, shape, timing and methods stay fixed. This is moderate faceted Mars
ground; it did not pass the original 8° screen. Terrain knowledge selects the
offline fixture and never enters the controller.

The registered primary comparison is C versus N at all three phases. C must
complete with finite state and bounded applied input, pass the fixed I-58-style
tracking/attitude/yaw/speed/50 ms acceleration nonregression margins, and satisfy
all four conditions throughout the final 2 s: goal distance ≤20 mm, XY speed
≤0.02 m/s, absolute world vertical angular speed ≤0.05 rad/s, raw-reference
up-vector error ≤2°. Report first sustained 2 s entry and later exits across
the whole hold separately. There must be a nontrivial native settling failure
and at least 0.10 s reference intervention in each C phase, otherwise an
otherwise-passing result is inconclusive. A remains a fixed secondary comparator;
it cannot replace C after outcomes. Passing this screen would not automatically
adopt the selector or reverse I-72's offline rejection.

Intervention exposure is measured at the reference/feedforward boundary against
the native rolling calculation on the same current state. It does not establish
an applied-command change after preparation/clipping. Actual preparation and
clipping are reported separately, with N as the independent closed-loop baseline.

## What the goal hold shows

Each entry below is the **maximum over the final physical 2 s**, including the
actual terminal state. Maxima need not occur at the same instant; acceptance
requires every condition at every sample. The final emitted but unheld command
is excluded from command/exposure statistics.

| Case | Goal distance mm | XY speed mm/s | Absolute world vertical angular speed rad/s | Raw-reference attitude error ° |
|---|---:|---:|---:|---:|
| Criterion | ≤20 | ≤20 | ≤0.05 | ≤2 |
| −1.0 N | 5.847 | 45.905 | 0.1515 | 0.8668 |
| −1.0 A | 2.568 | 46.641 | 0.2353 | 0.7175 |
| −1.0 C | 2.980 | 46.172 | 0.0979 | 0.8441 |
| +0.3 N | 7.018 | 13.726 | 0.2129 | 0.2766 |
| +0.3 A | 5.569 | 3.997 | 0.2696 | 0.0278 |
| +0.3 C | 28.418 | 39.090 | 0.4698 | 0.6426 |
| +0.9 N | 42.855 | 124.701 | 0.6436 | 2.5474 |
| +0.9 A | 42.944 | 106.133 | 0.3642 | 1.9305 |
| +0.9 C | 44.118 | 87.469 | 0.6993 | 1.4555 |

All nine fail the rotation criterion. Native/velocity +0.3 meet the other three
criteria; all −1.0 cases return close to goal but also fail the speed criterion.
No case has a fully observed simultaneous sustained 2 s settled interval anywhere
in the 10 s hold. Near-zero endpoint position alone is insufficient evidence of
settling. The world-vertical angular speed is `R(q) ω_body`, not the derivative
of an Euler yaw angle. Raw-reference attitude error compares body/reference up
vectors; absolute body tilt is a distinct quantity shown in the videos.

## Decision and limits

All setup/reference/initial-state contracts pass. All three C phases have at
least the predeclared reference intervention exposure: 1.040, 1.525 and 0.100 s.
Thus this is an executed intervention, with especially sparse exposure at +0.9.
Primary C fails settling at every phase; −1.0 additionally regresses on late
rotation peak/travel, +0.3 on tracking, final position, reference-lean peak and
late rotation, and +0.9 on late rotation peak. Physical completion, finite state,
held applied-input limits and actual previous-input history pass.

Retain the original selected LQR/observer/feedforward/preparation pipeline.
Keep A and C as experimental tools, with no post-result threshold tuning or
secondary-winner substitution. I-72's failed estimator screen remains intact.
This experiment tests **rolling-reference routing**, not a full replacement of
the native support-force/gain estimator. It cannot identify the complete cause
of every deviation, prove the contact label is correct, or establish general
multi-contact performance. A is descriptive, not promoted after seeing results.
One deterministic S episode per method/phase is not a statistical replication
study. The earlier solver investigation remains parked.

## Verification and preserved setup history

- Eighteen adapter/runner contract functions pass: 13 adapter, 4 runner,
  1 complete native-controller identity function with 2 included subtests.
  Moving/stationary wrapped N output, diagnostics and internal history are
  byte-exact. These are not full repository-suite results.
- Four unique route contracts pass at both origins; repeating them is not
  eight tests. Limited truth-free feature/ranker parity at 39 old states matches
  masks, IDs and accepted labels exactly and numeric fields to 1e−12 abs+relative.
- Twenty-two final evaluation contracts pass, inclusive of the earlier 21.
  Source review caught a verifier lookup that incorrectly expected the metric
  fragment in the source-hash map; it is an input binding. Preserve stage 01,
  then stage 02 fixes the lookup and tests the actual frozen protocol schema.
  This was corrected before the first outcome evaluation; no physics changed.
- Four final media contracts pass, inclusive of the first three. Three initial
  preview invocations failed before drawing because native `save_run` stores
  float32 while the precision file retains float64. All 36 cast comparisons
  match exactly. Preserve the failed sources/logs and render from the original
  full-precision saved states after checking that encoding relationship.
- All 86 pre-physics source bindings, 14 input bindings, both plugins, original
  gain bytes and 72 sealed case artifacts verify. Pure evaluation source was
  sealed and contract-tested before reading the new arrays. Control/gates were
  frozen before physics. The canonical evaluation ran once without a setup failure.

Physics protocol SHA256:
`30f37fabc23b76dab3caf9eca50fa9221c2cfefcae29385978303c57e550fbdf`.
All-nine output seal SHA256:
`54da1644bfd407e955e7f08a064315eb24e4a2405c51d139c25045ad885474ba`.
Source snapshots distinguish the original rejected sites and later corrections.

Final [archive/restore verification](archive_verification/receipt.json) passes:
501 compact files (28.23 MB including manifest), all 32 lossless bundle members
(155.01 MB), exact 50 ms plot projections with terminal states, and an actual
isolated restore of the archived compact record, 30 raw files and frozen gain
cache. The unrelated sentinel remains unchanged and no live source/plugin is
installed. Six focused package/restore contracts pass separately. The final
audit is stored beside the immutable evidence tree, whose manifest remains
`db4411fa096a99a27fa4f327e27a1264f78f65eeac5f3f3d2d760e994d9e8b29`.

## Videos and reproduction

Each video compares **N / A / C** side by side, covers saved Gazebo time 0–45 s
including the full goal hold, and uses one static camera covering all nine paths.
It renders the recorded full-precision poses, with no new physics. At 20 fps,
901 inclusive endpoint frames encode as 45.05 s. Body tilt, reference lean,
raw-reference up-vector error, world rotation and position error are visible.

- [−1.0 rad comparison](../../../../results/contact_reference_control_20261127/publication/videos_stage01/m100/m100_N_A_C.mp4)
- [+0.3 rad comparison](../../../../results/contact_reference_control_20261127/publication/videos_stage01/p030/p030_N_A_C.mp4)
- [+0.9 rad comparison](../../../../results/contact_reference_control_20261127/publication/videos_stage01/p090/p090_N_A_C.mp4)
- [Decoded first/middle/final frames for every phase](evidence/publication/video_quality_stage01/decoded_contact_sheet.jpg)
- [Media quality report](evidence/publication/video_quality_stage01/report.json)

Local work: `results/contact_reference_control_20261127/`. Compact tracked
evidence is under this page's `evidence/`. Large precision files, rollouts,
full-rate signals and videos stay in the companion
[lossless raw/media bundle](../../../../results/contact_reference_control_20261127/publication/i73-contact-reference-control-raw-media.tar.gz).
Its [inventory and hashes](evidence/publication/local_raw_bundle_inventory.json)
identify every member. Share this bundle **with the repository**; the bundle
contains the large payload while the repository supplies the compact evidence
and tools. The tracked 50 ms projections are for plotting only and cannot
reproduce acceptance metrics.

From a clean checkout, after obtaining that bundle, restore only this experiment's
result-owned files and optionally its original 52 KB gain-cache bytes:

```bash
.venv/bin/python docs/iterations/2026-11-27/contact-reference-control/evidence/publication/package_tools/restore_record.py --evidence docs/iterations/2026-11-27/contact-reference-control/evidence --repo-root . --raw-bundle /path/to/i73-contact-reference-control-raw-media.tar.gz --install-gain-cache
```

The restore refuses conflicting bytes and never overwrites live code or runtime
plugins. It verifies the compact record and raw bundle; two runtime `.so` members
are inventory/verification payloads, not automatically installed. Original tested
runtime/source/asset requirements are in
[dependency bindings](evidence/publication/runtime_dependency_bindings.json).
Adding `--check-runtime` verifies them. See [GAZEBO.md](../../../../GAZEBO.md) for
the Gazebo environment/build setup. A different compatible binary/environment
requires a newly identified experiment; do not weaken this historical protocol's
hash checks to describe it as the original trial.

Re-evaluate the exact saved full-rate arrays into a **new** output directory:

```bash
.venv/bin/python results/contact_reference_control_20261127/evaluation/evaluate.py --protocol results/contact_reference_control_20261127/protocol/protocol.json --runs-root results/contact_reference_control_20261127/campaign/runs --all-runs-seal results/contact_reference_control_20261127/campaign/outputs_sealed.json --output results/contact_reference_control_20261127/evaluation/reproduced_stage01
```

For fresh physics in a separate restored checkout, first keep the restored
historical campaign under a distinct name, then use the frozen isolated-process
launcher. Do this only where `campaign_original_i73` does not already exist:

```bash
mv results/contact_reference_control_20261127/campaign results/contact_reference_control_20261127/campaign_original_i73
.venv/bin/python results/contact_reference_control_20261127/campaign_tools/launch.py
```

The launcher uses `build/tools/bin/micromamba run -p build/gazebo-env`, sanitizes
inherited library/Python paths, fixes BLAS/OpenMP thread counts and starts one
fresh Gazebo process per case, three at a time. Case tools refuse existing case
directories. A fresh-physics reproduction is a new episode; preserving the old
source/gain/asset/binary bindings does not establish cross-machine bit identity.
