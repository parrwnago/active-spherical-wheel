# I-82 — the higher belt limit improves the current figure-eight

2026-11-28. Status: **two runs completed and analyzed; fixed capacity screen
passed.** The 0.60 m/s operating limit is a promising experimental variant for
this case. The original 0.30 reference and production defaults remain preserved.

## Watch and share

- [Full 133 s comparison at 1×](../../../../results/current_belt_capacity_20261128/publication/video_stage01/eight_m100_B030_B060.mp4)
- [98–104 s slope-event excerpt](../../../../results/current_belt_capacity_20261128/publication/video_stage01/event_98_104_B030_B060.mp4)
- [Visual evidence ZIP: both videos, plots, PDF and audits](../../../../results/current_belt_capacity_20261128/publication/visual_bundle_stage01/I82-visual-comparison.zip)
- [Two-page comparison PDF](evidence/evaluation/completed_stage01/capacity_comparison.pdf)
- [Whole-route plot](evidence/evaluation/completed_stage01/full_route_comparison.png) · [Downhill-event plot](evidence/evaluation/completed_stage01/downhill_episode_comparison.png)
- [101.5 s frame](evidence/publication/video_stage01/pose_101.50s.jpg) · [133 s final frame](evidence/publication/video_stage01/pose_133.00s.jpg)

Left is 0.30 m/s; right is 0.60 m/s. Both use the same LQR gains, momentum
observer, inferred contacts, feedforward and preparation algorithm. The video
renders **recorded Gazebo/DART poses** with MuJoCo visuals only. It runs no
controller or physics. One static world-up camera, a common reference and the
same physical clock show the complete motion and final ten-second goal hold.
All 1,331 full-video frames and 61 excerpt frames were sequentially decoded;
encoded representative frames and model geometry were inspected.

## Result

| Metric | 0.30 m/s reference | 0.60 m/s variant |
| --- | ---: | ---: |
| Moving timed XY RMS | 46.36 mm | 23.36 mm |
| Full-run timed XY peak | 336.41 mm | 72.17 mm |
| Moving geometric path RMS | 39.87 mm | 15.20 mm |
| Final goal distance | 32.88 mm | 0.88 mm |
| Peak own-target up error | 9.46° | 2.88° |
| Peak absolute body tilt | 12.23° | 11.87° |
| Peak XY speed | 0.672 m/s | 0.257 m/s |

Both processes returned zero; both physical episodes reached 133 s with 26,601
recorded control rows and no physical failure. **All 111 setup checks and all
30 predeclared performance checks pass.** Passing this comparative screen is
separate from physical completion, complete settling and default adoption.

The 0.60 run returns very close and almost stops translating. Over the final
two seconds its maximum goal distance is 0.970 mm and XY speed 0.290 mm/s;
its up error is also below the fixed threshold. It **still rotates at up to
6.095°/s**, above the 0.05 rad/s settling limit. Thus neither arm satisfies all
four final settling conditions: 0.30 fails position and rotation; 0.60 fails
rotation only. Do not describe the new result as fully settled.

The screen requires at least 10% moving timed RMS and 20% full timed-peak
improvement, alongside fixed tracking, attitude, speed, rotation, acceleration,
command-step and motor-clipping nonregression checks. Native settling successes
must be preserved if present; this native arm had none. Actual operating-cap
and common original-0.30-cap command statistics are both retained, so simply
enlarging a clipping denominator cannot pass the comparison.
[Frozen definitions](evidence/evaluation/definitions.json) and the
[complete numerical report](evidence/evaluation/completed_stage01/report.json)
give exact conditions and all decisions.

## What changed and what did not

The user requested revisiting the larger limit on the exact current figure-eight
and requested video. [I-49](../../2026-11-26/belt-capacity-sweep/README.md) tested
a different 44 s/+0.90 rad/yaw45 case and had mixed results. Preserve that result;
it does not answer this 133 s/−1 rad/yaw158 case. [I-81](../slope-response-trace/README.md)
deliberately retained the 0.30 baseline for its unchanged physical diagnostic.

Starting from `907e55a4c984fea485f3ed20bc4f67e5e358cf7f`, I-82 runs original
I-76 native `eight_m100_N` on its rough-stress Mars terrain. There is no heading
helper, path retiming, new estimator or new force controller. The body-state
measurement boundary remains exact simulated pose/velocity; contact control
information is inferred. Physical solver contacts are recorded for evaluation.

Both arms construct the **original 0.30 design/configuration and gain bank**.
After design construction, both physical and measured-state mirror operating
belt bounds are set to 0.30 or 0.60 m/s. Torque bounds remain
`[1.5, 0.6, 0.6] N m`; the numerical gains and ordinary LQR design are identical.
Incoming-drive preparation reads the operating cap too. This tests the capacity
interface throughout the existing loop, rather than changing only the last clip.
The modeled belts prescribe surface speed: raising this limit does not establish
a finite motor force, torque, power or loaded speed envelope.

The fresh 0.30 run reproduces **all 79 original I-76 precision arrays exactly
through 133 s**, before the larger-limit arm is launched. The 0.60 arm's first
changed actual command and first nonbounds diagnostic occur at **80.880 s**.
Its physical/observer history matches through that first command boundary.
Therefore the later event is a comparison of two capacity histories, not an
isolated intervention at the same 100.9 s state.

## What the downhill recording adds

Each episode records 1,300 physical 1 ms samples across 260 held 5 ms command
intervals over 100.5–101.8 s, without adding an observer update or feeding solver
truth into control. At the interval ending 100.900 s:

| Whole-COM downhill force | 0.30 | 0.60 |
| --- | ---: | ---: |
| Ground contact | +10.598 N | −3.278 N |
| Gravity | +3.234 N | +3.265 N |
| Sampled net | +13.833 N | −0.012 N |

The higher-capacity history avoids the large downhill pulse at that clock time.
Its robot has already arrived there with a different motion/contact history.
These numbers support the observed improvement but do not identify which single
earlier saturation caused it, or prove that increasing one isolated command at
100.9 s would fix the baseline. [Force-window summary](evidence/evaluation/force_stage01/summary.md)
and [full descriptive report](evidence/evaluation/force_stage01/report.json)
retain all/admitted/native force variants and COM momentum closure. This
supplement was registered after primary motion-array reads and before its own
trace reduction; it does not add or change acceptance gates.

## Integrity, corrections and retained unsuccessful setup

The prephysics protocol binds 93 sources and 59 inputs; its SHA256 is
`ce39ad0f0cd6c3748e764b72d413b45ad20353c1deb641bdbd722c35402dc400`.
The closed-output seal binds 456 files. Sources, gain cache, initial conditions,
commands, truth guards and exact baseline replay were independently checked.
An independent reduction agrees with 124 scalar values and all 30 gate
values/limits/names/decisions, maximum numeric difference 1.74e−12.

Retain these corrections rather than overwriting the failed stages:

- An initial system-`python` compile returned 127; the explicit virtual-environment
  interpreter corrected the setup. Eight focused runner contracts pass.
- The scorer omitted an analysis-only I-76 geometric helper from the physical
  maps. Its exact bytes were separately registered at 08:18:10 UTC before either
  new saved precision/outcome existed; a named wrapper adds that binding.
- An audit using **old I-76 arrays only** exposed signed-max world-z spin reporting
  where physical peak magnitude required absolute-max. The original core remains
  archived; one line changes in `core_stage02.py`, with a negative-rate contract.
  Correction registration at 08:23:21 UTC occurred after B030 precision existed,
  before B060 precision existed, and **before any I-82 scoring array reads**. Numerical
  thresholds/definitions and physics do not change. Final eight metric contracts
  include the original seven; corrected old-metric parity is 14/14 exact.
  Retained records also include the intermediate parity and Path-metadata errors.
- The inherited decoder assumed 1600×960 while the new captioned canvas is
  1600×1010. New `render_stage02.py` corrects dimensions before reading/rendering
  arrays. Five final presentation contracts include the earlier four.
- Raw packaging separately closes four registered analysis/media dependencies
  outside the 93/59 physical maps at their original paths. Original packaging
  helper and contract history remain preserved. The first archive attempt stops
  before tar construction on relative/absolute CLI-path formatting. Preserve its
  failure receipt and use a separately named stage03 helper with resolved paths
  and a new output stage; no science or media changes. Six final packaging
  contracts include the earlier five and original four.

These focused contracts, packaging checks, independent reductions and physical
runs have different scopes; none is a full repository test-suite count. No
additional physics was run to repair analysis or presentation setup.

## Preserve and reproduce

[Compact manifest](evidence/manifest.json) binds exact reports, plots, receipts,
selected frames and the [source/structured-diagnostic archive](evidence/source-and-diagnostics.tar.gz).
It preserves 152 files totaling 9,304,438 bytes plus the manifest; the compact
archive has 665 verified members.
Large arrays, full movies and the raw bundle remain local under ignored
`results/current_belt_capacity_20261128/`. The visual ZIP is 7,897,618 bytes with
33 members, each CRC/SHA checked; SHA256
`18b79c3893ff9022c2b30a793d06326bcaf34c995f220674485cf9d7b076302e`.
The [full raw archive verification](evidence/publication/full_evidence_stage02/archive_verification.json)
and its adjacent inventory/restoration metadata bind both raw episodes, original
prerequisites, registered supplemental dependencies, exact sources, all failed
stages and the media. The [raw bundle](../../../../results/current_belt_capacity_20261128/publication/full_evidence_stage02/i82_full_evidence.tar.gz)
is 181,641,179 bytes with 971 stream-verified regular members; SHA256
`dcb4a7951b13db65c9779f7c58209d5f48a000c9a5c1b77287f29460da5df092`.
The omission inventory records 1,195 excluded paths. Follow the exact
[restoration instructions](evidence/publication/full_evidence_stage02/REPRODUCE.md).
Restore archived paths in an isolated checkout; do not
overwrite a teammate's active tree. Follow [GAZEBO.md](../../../../GAZEBO.md)
for the Gazebo/DART environment.

Recorded launch commands, environment and observed exits are preserved in
[campaign metadata](evidence/campaign/registered_launch.json). A new physical
comparison uses a new empty campaign and registered paths; never overwrite the
original evidence. The runner CLI for each arm is:

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH -u LD_PRELOAD \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  build/tools/bin/micromamba run -p build/gazebo-env python \
  results/current_belt_capacity_20261128/runner/run_case.py \
  --protocol results/current_belt_capacity_20261128/protocol/frozen_stage01/protocol.json \
  --arm B030 --output results/new_capacity_comparison/B030 --run
```

Require original full baseline parity before running B060. Repeat the command
with `--arm B060 --output results/new_capacity_comparison/B060` and add
`--native-baseline results/new_capacity_comparison/B030`.
For analysis/media-only reproduction, restore the exact archived source/input
paths and use new stages with `evaluation/evaluate_stage03.py` and
`publication/render_stage02.py`; recorded registrations and
[media CLI](evidence/publication/README.md) identify their arguments. Sources
inside the compact tar preserve their original repository-relative locations;
the raw bundle adds external input arrays/assets needed to execute them.

**Decision:** retain 0.60 as the leading capacity variant for this episode,
keep the original architecture, and preserve the 0.30 reference. A later
cross-phase/cross-route validation should determine generality before default
promotion. Residual rotation remains an explicit issue. This experiment
establishes a substantial capacity-related improvement in one matched case;
it does not settle the full contact/controller diagnosis.
