# I-74 — native baseline with bounded starting-heading hold

Status: **completed and analyzed; experimental, not adopted**, 27 November 2026.
User approved N versus H on the same three I-73 S-path starts. All six new
Gazebo/DART episodes complete 45 s without declared physical failure. Heading
hold improves stopping in two starts, but fails the fixed all-phase screen:
12 of 66 checks fail despite all 95 setup checks passing.

[Motion/heading comparison](evidence/publication/plots_stage01/motion_comparison.png) ·
[Goal hold](evidence/publication/plots_stage01/goal_hold.png) ·
[Two-page PDF](evidence/publication/plots_stage01/heading_hold_comparison.pdf) ·
[Canonical numerical report](evidence/evaluation/completed_stage01/science.json)

| Start phase | N moving XY RMS mm | H moving XY RMS mm | N final error mm | H final error mm |
|---|---:|---:|---:|---:|
| −1.0 rad | 47.31 | 24.38 | 4.863 | 0.155 |
| +0.3 rad | 22.85 | 24.00 | 7.018 | 2.868 |
| +0.9 rad | 23.37 | 33.69 | 41.410 | 53.460 |

N is the unchanged native baseline; H adds only the existing bounded heading
helper. Moving error is against the common **timed** XY reference during 3–35 s.
The −1.0 H peak XY error falls 179.59→50.05 mm. The +0.9 tracking regression is
real and remains in the result; it is not replaced by the better two cases.

H keeps the original LQR, momentum observer, support estimation and rolling/
holding feedforward, and enables the existing unloaded-shell heading helper.
The actual starting heading is its fixed target for the whole episode. Settings:
`heading_reference='start'`, `heading_actuation='shells'`, gain .2 N m/rad,
damping .06 N m s/rad, helper torque cap .15 N m and horizontal reaction budget
.05 N m. Existing load/lever eligibility and ±.5 rad error clamp remain.
The selected native baseline N retains its original configuration.

The torque budget bounds the added helper's horizontal reaction, not total
LQR requests or body lean. The original final caps still apply to total commands.
The body can have angular-rate feedback while heading angle is unregulated:
N already has the former, H adds the explicit starting-heading objective.

Six fresh Gazebo/DART episodes: N/H at requested starting phases −1.0, +0.3,
+0.9 rad, yaw 45°, exact I-73 fixed S and moderate faceted Mars site. Each has
3 s initial hold, 32 s motion, 10 s final hold, 5 ms control /1 ms physics and
unchanged `[1.5,.6,.6 N m,.3,.3 m/s]` final caps. No map enters live control.
No new site selection, gain-bank design or contact-selector intervention.

The fixed comparison asks for at least 20% lower wrapped heading-error RMS
during 3–45 s and at least 20% lower integral of absolute physical world-vertical
angular speed during 29–45 s in every phase, together with fixed tracking,
attitude, speed, acceleration and saturation nonregression guards. Settling is
reported separately. This is a bounded experiment, not automatic adoption or
a guarantee that yaw is independently controllable in every configuration.

## Rotation and actual heading

| Phase | Heading-error RMS N→H, degrees | Late physical rotation N→H, degrees | Final heading error N→H, degrees |
|---|---:|---:|---:|
| −1.0 | 47.09→46.55 | 68.88→65.79 | 3.76→2.67 |
| +0.3 | 116.24→34.09 | 202.21→65.52 | 17.30→56.40 |
| +0.9 | 104.04→82.12 | 356.28→37.71 | 158.58→91.42 |

Heading RMS uses actual wrapped `yaw − latched starting yaw` over 3–45 s.
The rotation integral is `∫|[R(q) ω_body]_z| dt` over 29–45 s, using physical
states and trapezoid integration, not an Euler-yaw derivative. Euler heading
travel is retained as a separate metric. A robot can stop rotating while still
pointing away from its requested heading. Raw error must also be distinguished
from the ±.5 rad clipped error used by the helper; plots/videos show raw error.

H−1.0 and H+0.3 pass the original **four simultaneous** final-2 s criteria:
goal distance ≤20 mm, XY speed ≤.02 m/s, |world vertical angular speed| ≤.05 rad/s,
raw-reference up-vector error ≤2°. Their first sustained 2 s entries start at
35.480 and 35.455 s. Only H−1.0 also satisfies the added ≤10° heading criterion.
All three N runs fail the four-criterion final settling condition, as in I-73;
H+0.9 also fails. Settling is descriptive here and does not replace the fixed
heading/tracking/balance/command gates.

## Why this is not adopted

Every H phase has substantial pre-fade/clipping helper activity:
43.110/42.120/29.215 s. It is an executed intervention. This activity is not a
complete same-state applied-command counterfactual. All three N recordings
reproduce **46 shared numerical arrays each** from I-73 byte-exactly, including
21 required core fields. Thus baseline changes do not explain the differences.

The fixed screen fails in these ways:

- **−1.0:** neither heading benefit reaches 20%, absolute tilt peak increases to
  13.06° beyond the 11.31° margin, and late rotation-speed peak increases.
  Normalized unclipped command excess RMS is 34.8845 versus N 0.192865. Although
  actual inputs remain capped, a rare very large request fails the command guard.
- **+0.3:** heading RMS and total late rotation improve substantially, but raw
  reference-lean peak narrowly exceeds the margin; full/late rotation-speed
  peaks and the normalized command-step peak also regress. A smaller total
  amount of rotation does not guarantee a smaller peak.
- **+0.9:** heading metrics improve, but moving RMS, full peak and final XY
  errors exceed the allowed tracking margins. It ends 53.46 mm from the goal
  and 91.42° from its starting heading.

Saturation-frequency margins pass, but they do not capture request magnitude.
At −1.0, any-channel clipping fraction is 0.733% versus N 0.556%; the normalized
excess RMS nevertheless increases dramatically. The separate demand-magnitude
guard catches this. No simulated actuator exceeds its final input bound.

The [posthoc fixed-time telemetry](evidence/publication/hold_telemetry_stage01/report.json)
explains one concrete availability limit, without claiming the whole trajectory's
cause. At 35 and 45 s in H+0.9, the helper is ineligible: neither estimated shell
load satisfies the ≤5% unloaded condition and the shell-axis vertical lever has
magnitude .0653, below the existing .12 cutoff. The heading term is zero despite
approximately −91.4° raw error. This shows the **existing helper's** dead zone;
it does not prove the mechanism has no other yaw-control strategy. At H+0.3
45 s the helper remains eligible and commands about .05074 N m to each shell
despite −56.4° error, so not every heading offset is an inactivity problem.

## The large request at 16.670 s

A separately sealed [bounded posthoc audit](evidence/evaluation/spike_audit_stage01/report.json)
inspects each case's largest normalized clipping excess and fixed ±25 ms context.
It does not change the canonical gates or rerun physics. The +0.3 N case has
no clipping, so its selected earliest zero tie is not called a real spike.

In −1.0 H at 16.670 s, the original rolling calculation supplies shaft/shell-A
references of approximately **−6,555/−23,638 rad/s**, versus actual measured
rates +.322/−6.999 rad/s. LQR joint-velocity-error terms produce approximately
−1498.3/−1892.7 N m. The total five-command request is approximately
`[−1498.545, −1892.619, −61.216, 0, 0]`, while actual applied input is
`[−1.5, −.6, −.6, 0, 0]` in N m /m/s.

The heading helper contributes only `[0,0,−.070819,0,0]` N m at this row.
Rolling feedforward, preparation delta and fade delta are zero. The enormous
request therefore enters through **rolling joint-speed references and LQR
velocity feedback**, not through a large direct heading torque. It occupies
one 5 ms row in the fixed context; ordinary references resume at 16.675 s.
Algebraic ledgers/decomposition identities agree within 2.274e−13. The derived
rolling shaft-error block remains explicitly separate from measured shaft angle.

This localizes a command source, not the full physical cause. No contact
Jacobian/condition metric was logged, so a near-singular contact map is a
hypothesis rather than an established explanation here. The actual capped
inputs and resulting physical motion must not be described as those enormous
unclipped torques. The audit's 42 algebraic identities are not additional
software tests and do not override the failed heading-control screen.

Retain native N as the selected baseline and H as an experimental option.
Do not declare successful general heading regulation from the two improved
stops, change gates after results, or tune a different setting into this report.
One fixed setting on one S route is not an exhaustive heading-controller study.
The LQR, observer, support estimation and contact feedforward remain unchanged;
this result does not qualify a new estimator or revive parked solver/HMM work.

Original N full-precision common arrays reproduce I-73 N. All six outputs
sealed before outcome analysis. Sources, fixed settings, gates, assets/gain/plugin
hashes and route froze before physics; pure evaluation code sealed and passed
focused checks before reading new arrays. Regressions and source-review corrections
are retained in this record.

## Verification and reproduction

Twelve scoped runner/telemetry contract functions pass: eight real-helper
telemetry, three parameter guards and one full native N/H command/info/history
identity function with four included subtests. Twenty-three final evaluation
functions pass, inclusive of the earlier 21. These are scoped software checks,
not the full repository suite. Four orientation and six package contracts from
I-73 are reused; one new media heading-wrap/target contract is added. Source
review preserves corrections to an evaluator field alias and the distinction
between raw and used/clipped heading error before outcome/media reads. No
controller or physics source correction and no failed physical setup occurred.

Before physics, protocol
`aa3d290f3278f515ffdc47042ae8725c69166f28cecf819d26f2541eac4e82a8`
binds 80 control/runtime/runner sources and 22 inputs, including portable byte-exact
I-73 native precision inputs, original gain cache and fixture provenance.
After all six process outputs, seal
`40e9b4ccbfe7d5bd268828e60a3478c1175180bd38416e7f2270ef94f6054424`
binds 48 artifacts. All source/input/gain/plugin/output bindings verify unchanged;
the canonical evaluation runs once after its own source/input seal.

Compact tracked evidence and the full raw/media companion are verified below.
The raw bundle includes the three original native precision inputs needed by
the exact replay gate, so evaluation does not depend on the ignored I-73 tree.

## Videos and reproduction

Each video compares **N / H** side by side, covers saved Gazebo time 0–45 s
including the full goal hold, and uses one static camera covering all six paths.
It renders the recorded full-precision poses, with no new physics. At 20 fps,
901 inclusive endpoint frames encode as 45.05 s. Body tilt, reference lean,
raw-reference up-vector error, world rotation and position error are visible.

- [−1.0 rad comparison](../../../../results/heading_hold_control_20261127/publication/videos_stage01/m100/m100_N_H.mp4)
- [+0.3 rad comparison](../../../../results/heading_hold_control_20261127/publication/videos_stage01/p030/p030_N_H.mp4)
- [+0.9 rad comparison](../../../../results/heading_hold_control_20261127/publication/videos_stage01/p090/p090_N_H.mp4)
- [Decoded first/middle/final frames for every phase](evidence/publication/video_quality_stage01/decoded_contact_sheet.jpg)
- [Media quality report](evidence/publication/video_quality_stage01/report.json)

Local work: `results/heading_hold_control_20261127/`. Compact tracked
evidence is under this page's `evidence/`. Large precision files, rollouts,
full-rate signals and videos stay in the companion
[lossless raw/media bundle](../../../../results/heading_hold_control_20261127/publication/i74-heading-hold-control-raw-media.tar.gz).
Its [inventory and hashes](evidence/publication/local_raw_bundle_inventory.json)
identify every member. Share this bundle **with the repository**; the bundle
contains the large payload while the repository supplies the compact evidence
and tools. The tracked 50 ms projections are for plotting only and cannot
reproduce acceptance metrics.

From a clean checkout, after obtaining that bundle, restore only this experiment's
result-owned files and optionally its original 52 KB gain-cache bytes:

```bash
.venv/bin/python docs/iterations/2026-11-27/heading-hold-control/evidence/publication/package_tools/restore_record.py --evidence docs/iterations/2026-11-27/heading-hold-control/evidence --repo-root . --raw-bundle /path/to/i74-heading-hold-control-raw-media.tar.gz --install-gain-cache
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
.venv/bin/python results/heading_hold_control_20261127/evaluation/evaluate.py --protocol results/heading_hold_control_20261127/protocol/protocol.json --runs-root results/heading_hold_control_20261127/campaign/runs --all-runs-seal results/heading_hold_control_20261127/campaign/outputs_sealed.json --output results/heading_hold_control_20261127/evaluation/reproduced_stage01
```

For fresh physics in a separate restored checkout, first keep the restored
historical campaign under a distinct name, then use the frozen isolated-process
launcher. Do this only where `campaign_original_i74` does not already exist:

```bash
mv results/heading_hold_control_20261127/campaign results/heading_hold_control_20261127/campaign_original_i74
.venv/bin/python results/heading_hold_control_20261127/campaign_tools/launch.py
```

The launcher uses `build/tools/bin/micromamba run -p build/gazebo-env`, sanitizes
inherited library/Python paths, fixes BLAS/OpenMP thread counts and starts one
fresh Gazebo process per case, three at a time. Case tools refuse existing case
directories. A fresh-physics reproduction is a new episode; preserving the old
source/gain/asset/binary bindings does not establish cross-machine bit identity.

Final [archive/restore QA](archive_verification/receipt.json) verifies 404 bound
compact files (22.69 MB including manifest), all 32 members of the 135.73 MB
raw/media tar, exact plot-only subsets, and an actual isolated restore of 404
compact files, 30 raw files and the frozen gain cache. An unrelated sentinel is
unchanged; no live source/plugin is installed. The final compact manifest is
`d86d79b1c7794a83a5e6804f80cf0b808e84278a4ad37e85acf74c3c791c5a91`.

An archive-only first curation mistakenly redirected stdout into its own
scanned source folder: the packet captured an empty active log which changed
after completion. The first packet/bundle and curator source/log hashes are
preserved locally under `publication/archive_failed_stage01/`, with the
[setup-failure record](evidence/publication/package_tools/packaging_setup_failure_stage01.json).
The corrected second curation avoids self-scanned output and passes full QA.
No experiment, metric, video or control source was rerun or changed by this
packaging correction. The final QA is beside the immutable evidence tree.
