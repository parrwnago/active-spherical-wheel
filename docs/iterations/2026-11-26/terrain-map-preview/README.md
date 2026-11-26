# I-44 — remembered terrain for contact preview

Date: 2026-11-26 (America/New_York). **Status: implemented and analyzed; retained offline.**
The original design below is retained, followed by implementation and experiment
records. New diagnostic code adds four-view ideal depth capture and terrain
memory. Existing controller source/configuration/defaults and old experiments
remain unchanged. This is offline sensor replay, not a new Gazebo physics run,
an integrated Gazebo camera plugin, SLAM or hardware evidence.

**Design clarification, same date.** The initial proposal suggested one virtual
depth sensor for a controlled traversal. The user specified four cameras around
the platform because yaw and travel direction vary, and authorized exact
simulated body state for this stage. The current proposal therefore uses four
virtual depth views; implementing a body-state estimator or SLAM is deferred.
This supersedes the single-view starting proposal without changing runtime code.

## Question and motivation

Can terrain observed before arrival improve contact-geometry forecasts once
the wheel occludes the surface? A camera need not see directly underneath the
wheel at the current instant if a sufficiently well-registered earlier
observation remains available. This can help on the first traversal, provided
the ground was seen before arrival; it does not require a completed first lap.

The Mars bump is a plausible place to benefit from geometric preview. It is
not established that missing terrain explains the entire tracking deviation.
I-41–I-43 also exposed command saturation and direction-change regressions.
The dynamic forecast was diagnostic, so its prediction error did not directly
cause the live controller's commands. Terrain memory alone is not an overshoot
fix, a contact-force measurement or evidence that controller tuning is complete.

## Current baseline and proposed additions

“Current” here means the selected I-40–I-43 native baseline:
`mode='hardware'`, `rolling_allocation='least_squares'`, `synchronize=True`,
`tilt_reference=None`. The generic `OffroadParams()` default remains
`mode='terrain_aware'`; not every repository mode has this measurement boundary.

| Component | Status and role |
| --- | --- |
| Body/joint state and prior applied inputs | Active exact simulator measurements in a contact-free `StateMirror`; no implemented IMU/encoder/vision fusion. |
| Momentum observer | Active model-based estimate of net external force and moment. A push and ground reaction are not separately observable from this wrench alone. |
| Support locator and filters | Active shell/belt/rim support inference from wrench and geometry; filtered normal/load and inferred rolling-slip/friction behavior. |
| Path and attitude references | Active timed position, velocity and acceleration; desired lean uses acceleration and the estimated ground normal. |
| Rolling allocation and feedforward | Active least-squares conversion to drive references; rolling feedforward and static holding torques. The experimental bounded allocator is not selected. |
| Gain-scheduled LQR | Active balance/tracking feedback scheduled using configuration and inferred support/load. |
| Incoming-drive synchronization | Active 120 ms constant-velocity geometric preview to prepare unloaded drives. |
| Input bounds and low-load handling | Active command clipping, torque fading and damping under the existing idealized actuator model. |
| Native dynamic contact forecast | Existing 200 ms private-model, frozen-plane diagnostic; does not command the live robot. |
| Four depth cameras and terrain memory | Added as offline four-view capture/mapping diagnostics in this iteration; live LQR receives none of their outputs yet. The earlier camera declarations remain visualization-only. |
| Body-state input and sensor registration | Use exact simulated pose/velocity and capture-time camera transforms for this stage. IMU/encoder/vision or LiDAR state estimation and SLAM are deferred. |

MPC, iLQR and particle filtering are not active in this baseline. The I-43 tilt
shaper remains optional and disabled. There is no added active heading hold in
this configuration. Hardware will also require a calibrated actuator/input
model or torque estimates; a command is not automatically the achieved torque.

## Proposed architecture

```text
Four depth views + capture-time simulated pose + sensor calibration
              |
              v
Local terrain memory: observed/unknown, age, uncertainty
              |
              v
Robot geometry + configuration/motion preview -> possible future contacts

Current motion + known inputs -> momentum observer -> current support estimate

Initial experiment: log both branches; preserve live LQR commands.
Later, if validated: use preview to prepare feasible references/drive transitions.
```

Mapping and localization are complementary. A short-lived local terrain map
does not require globally consistent SLAM initially, but it does require the
relative pose between observation and arrival. For this stage use exact
simulated body pose and velocity for control and camera registration; this is
an explicit state oracle, not an implemented body-state estimator or SLAM.
Associate frames with their acquisition-time pose and account for processing
latency. In later robustness studies, propagate depth,
extrinsic and relative-pose uncertainty. Repeated depth pixels share pose error;
averaging them must not erase that common uncertainty. Keep observation
keyframes so later available pose corrections can update the local map.

Preserve unseen regions as unknown. During diagnostic replay, an empty startup
map or unknown region triggers abstention while native observer/LQR control
continues unchanged. A later validated closed-loop policy may choose conservative
references. Previously observed ground may remain useful during occlusion;
never-observed ground cannot acquire
measured status by interpolation. Older measurements need uncertainty/age
handling and checks for changed terrain.

Use mapped surfaces and known wheel geometry/configuration to predict possible
shell/belt/rim points, normals, clearances and conditional arrival times. Retain
multiple candidates near boundaries. The observer continues checking current
loaded support; a future mapped normal must not be substituted into current
holding torque or feedback before the robot reaches it. Geometry does not
determine actual load shares, friction, slip, deformation or sinkage. In
particular, terrain vision may not resolve a submillimetre shell/belt ambiguity.
A 2.5D height map is a reasonable first bump representation; floor-plus-step
and general multiple-surface contact need a representation retaining those
surfaces and separate validation.

Use four virtual depth cameras around the upper platform, with proposed
azimuths spaced 90 degrees apart and views directed outward/downward toward
the surrounding ground. This addresses travel directions that move outside a
single camera's view as platform yaw changes. Declare the mounting positions,
downward angles and fields of view before execution; choose overlap and measure
coverage rather than assuming four cameras guarantee complete ground visibility.
Minimum range, self-occlusion and platform tilt can still leave blind regions.
Accumulate observations from all four cameras in one terrain memory using
capture-time simulated pose and each camera's extrinsics. Evaluate coverage on
the yaw-changing figure-eight as well as a controlled traversal. This is a
simulation layout proposal, not a hardware purchasing decision.

## Smallest proposed experiment

1. Replay the saved native Mars figure-eight +0.9 trace without changing LQR.
   Declare all four mounts, per-camera intrinsics/extrinsics and timestamps,
   resolution, field of view/overlap, range, frame rate, latency, map resolution
   and scoring rules before examining outcomes. Begin with four-view ideal
   first-hit range rays and exact simulated pose/velocity, including capture-time
   registration, to isolate visibility/memory utility. Label this an ideal sensor adapter and
   registration oracle, not an implemented camera or SLAM system.
2. Only the sensor adapter accesses scene geometry. It emits visible depth
   pixels with timestamps/intrinsics/extrinsics. The mapper cannot query DEM
   height, triangle normals, future depth, true contacts or a complete mesh.
   Include robot self-occlusion and rocks/lava. Mars currently uses a DEM mesh
   for collision and a heightmap for visualization; rendered depth needs a
   visual/collision parity audit before it can serve as a physical-surface test.
3. At identical fixed uniform anchors and the predeclared 6.7–6.9 s episode,
   compare observer-plane, latest available four-camera batch only and
   accumulated-memory geometric previews with the same constant-velocity motion
   assumption and 50/100/200 ms
   horizons. This first comparison isolates terrain information from changes in
   the motion predictor. Both sensing variants use the same four views; memory
   additionally retains their past batches. Do not fit on future executed motion.
4. Report all-query coverage and abstentions, first-sight lead, covered-query
   point/normal error, candidate ambiguity and incoming-patch warning timing.
   Warning timing is a geometric proxy, not a predicted force-transfer result.
   Use Gazebo contacts only in evaluation; do not compare a predicted geometric
   normal directly with a force-weighted normal without naming the distinction.
   Include a controlled occlusion fixture, and preserve unsuccessful results.
5. Only process observations/pose corrections available at each prediction
   anchor. A full-run SLAM reconstruction with future loop closures cannot be
   replayed as though it had been available earlier. Live command outputs must
   remain identical. A successful ideal replay justifies a depth frontend and
   later correlated pose-drift/noise/latency study, not controller adoption.
   Implementing SLAM or a body-state estimator is not required for this stage.

At the original proposal stage, acceptance thresholds/settings were undeclared.
They were subsequently frozen in the registered protocol below before execution.
Later closed-loop work must recheck balance, limits,
route tracking and all direction-change cases; map accuracy alone cannot pass
the controller acceptance gates.

## Evidence and decision

The initial [source audit](source_audit.json) records pre-implementation code
hashes, inspected entry points, and unchanged I-43 controller/module and archive.
At that proposal stage no sensor adapter or executable reproduction command
existed; current implementation/commands are recorded below. Prior evidence
remains in [I-40](../../2026-11-25/lqr-contact-forecast/README.md),
[I-41](../../2026-11-25/figure8-event-audit/README.md),
[I-42](../../2026-11-25/handoff-factorial/README.md) and
[I-43](../../2026-11-25/tilt-reference/README.md).

Decision: retain LQR and the momentum observer. Prefer testing remembered
terrain as a geometric prior before replacing the estimator/controller family.
Overshoot remains a separate unresolved control/reference question. The original
proposal established no sensing benefit; experimental evidence belongs to the
subsequent replay record below.

Primary research context, checked 2026-11-26:

- Fankhauser, Bloesch and Hutter, [Probabilistic Terrain Mapping for Mobile
  Robots with Uncertain Localization](https://doi.org/10.3929/ethz-b-000272110),
  2018: local elevation mapping with range and localization uncertainty. This
  supports the mapping approach, not its performance on this wheel.
- Shan et al., [LVI-SAM](https://arxiv.org/abs/2104.10831), 2021: an example
  of visual/LiDAR/inertial pose estimation and map building; no dependency or
  implementation choice is made here.

## Implementation and registered replay

Added [terrain_depth.py](../../../../src/spherical_wheel/terrain_depth.py),
[terrain_memory.py](../../../../src/spherical_wheel/terrain_memory.py) and
[run-terrain-memory.py](../../../../scripts/run-terrain-memory.py). No existing
control path imports or invokes these diagnostics. The ray scene uses cropped
collision DEM/rock/lava triangles and a declared visible robot model. Known
robot first hits are invalid depth, never transparent. Depth frames expose no
scene, triangle IDs, surface normals or solved contacts to the mapper.

The fixed rig has four body-relative yaw offsets 0/90/180/270°, outward 45° down,
64×48 pixels, 100° horizontal field of view, square pixels, 0.08–2 m Euclidean
range, 20 Hz capture and 50 ms delivery latency. Optical axial depth is stored
as float32; ray calculations/poses are float64. The archive measures encoding
error separately. MuJoCo compiles mesh coordinates to float32; this is an ideal
range adapter with declared numerical limits, not a validated physical sensor.

The [registered protocol](evidence/registered_protocol/protocol.json) predates
campaign results. Before each execution, source snapshots identify its exact
code. Input hashes identify the unchanged I-40 native Mars +0.9 trace. The
latest-view comparator receives the same four cameras as memory; only retained
history differs. All three arms use the same current state and CV prediction.
Future physical contacts and private-scene geometric rays enter evaluation only.

Local output family: `results/terrain_memory_20261126/`. Fresh output stage
names are mandatory; the runner refuses to replace an existing stage. From
the repository root, using the declared protocol and saved native trace:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/run-terrain-memory.py --output results/terrain_memory_20261126 --stage depth --tag stage01
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python scripts/run-terrain-memory.py --output results/terrain_memory_20261126 --stage analyze --tag stage01 --depth-tag stage01
```

For a rerun choose unused tags. Scene construction also needs the fetched Mars
assets and the saved collision DEM identified by the protocol/provenance.
Source copies are evidence, not standalone installed packages; restore their
original paths in an isolated checkout to reproduce a historical source stage.

### Checks and retained development

The final selected suite passes **75 unique tests in each environment**: 51 new
cases (18 range/calibration, 17 memory/geometry, 16 campaign/integration) and 24
existing hardware-mode/contact-forecast cases. Environment repeats and earlier
overlapping development runs are not additional unique tests or robot trials.
The final sealed stage preserves pre-command sources and both logs. It follows
an initial 71-case smoke run whose complete pre-command source/log capture was
not sealed; that limitation is retained rather than presenting it as final QA.

An independent sloped-pole adversary exposed a one-ulp band-coordinate issue:
a valid belt pole could lose its candidate when its axial coordinate rounded
just above the radius. The failing pre-fix stage, minimal axis normalization/
zero-radius-pole correction and 17-case passing stage are all preserved. No
sensor/map settings were tuned on trajectory outcomes. A cross-module fixture
verifies actual rays → depth frames → memory: earlier observed flat ground stays
available after current robot occlusion, and yaw permutes which camera sees a
fixed world direction. No dynamics is integrated in those fixtures.

The adapter also passed a concave first-hit fixture and actual Mars/CAD scene
construction/triangle checks. These support geometry/implementation correctness;
they do not establish a contact-force or closed-loop tracking improvement.

## Completed replay and decision

One reused 44 s Gazebo/DART Mars figure-eight, initial phase +0.9 and yaw 45°.
There are **zero new physics executions**. The adapter produced 881 capture
batches / 3,524 depth frames. The three methods evaluated 119 unique anchors at
three horizons: 357 unique anchor/horizon pairs, 1,071 method rows. The uniform
stratum has 327 pairs and the dense bump stratum 33; their three pairs at 6.8 s
overlap and must not be added as independent observations.

### Remembered terrain is useful, with an early blind region

| Surface metric | Observer tangent | Latest four frames | Four-view memory |
| --- | ---: | ---: | ---: |
| Uniform available / queried | 327/327 | 0/327 | 277/327 |
| Uniform availability | 100% | 0% | 84.71% |
| Dense 6.7–6.9 s available / queried | 33/33 | 0/33 | 3/33 |
| Dense usable contact-candidate queries | 33/33 | 0/33 | 1/33 |

The latest frames contain substantial terrain: per-camera valid-pixel fractions
are 90.98%, 84.85%, 92.68% and 78.64%. Their zero query coverage means that the
specific CV-predicted underbody locations are unseen in the latest views, not
that the cameras produce empty images. Memory retains ground seen from earlier
positions/orientations and recovers most later queries. The first supported
memory query is at anchor 6.78 s, horizon 200 ms, using a cell first observed at
0 s. Only the 200 ms queries at anchors 6.78, 6.80 and 6.82 s are surface-supported
in the dense window; just the last has a usable contact candidate. This does
not establish dependable preview of the original first-deviation event.

All four registered **uniform terrain-utility** conditions pass: coverage
>=70%, gain over latest views >=10 percentage points, covered height RMSE
<=3 mm and covered normal p90 <=10°. Memory's actual values are 84.71%,
+84.71 percentage points, 0.683 mm and 4.496°. These gates concern geometric
information only; they are not live-control or contact-force acceptance gates.

### Matched comparisons expose the remaining contact problem

Because latest-frame coverage is zero, there is no three-way commonly covered
set. A separately labeled **post-registered descriptive supplement** compares
observer and memory at identical covered query pairs. It changes no settings,
primary results or gates.

| Same 277 uniform surface pairs | Observer tangent | Memory |
| --- | ---: | ---: |
| Height RMSE mm | 1.274 | 0.683 |
| Height median absolute error mm | 0.0556 | 0.3025 |
| Normal p90 deg | 9.450 | 4.496 |
| Normal median error deg | 0.907 | 1.414 |

Memory reduces larger surface errors, but its local fits introduce smoothing
error: the medians worsen. Normal scores are geometric face-normal proxies at
the common CV location, not force-weighted contact normals.

For **271 matched future-contact pairs**, nearest geometric-candidate error to
the saved load-weighted equivalent contact worsens from **7.903 to 11.304 mm
RMS**. Memory is better on 53 pairs and worse on 218. This nearest-set metric is
optimistic for ambiguous sets and includes shared motion-prediction error; it
does not establish force or load-share accuracy. Unloaded finite fallback
points are excluded. The shared CV position prediction itself has 7.902 mm RMS
and 37.396 mm maximum error across the 357 unique pairs.

Geometric family-change warnings also need caution: uniform observer/memory
have 29/30 warning slots with 15/19 endpoint false-change warnings, respectively.
These are candidate-set warnings relative to actual anchor support, scored
against actual future dominant family; they are not confirmed force-transfer
timings. Ambiguity and complete/partial query counts remain in the raw report.

**Decision:** retain four-camera memory as an offline geometric prior and keep
native LQR/observer/feedforward unchanged. The terrain-utility screen passes;
the resulting contact-point forecasts do not improve. Do not connect these
outputs to live gains, support torques or transition commands yet. Next work
should isolate early visibility and the contact-geometry/motion mismatch before
another closed-loop controller experiment. No improvement to drift or overshoot
is claimed from replaying unchanged motion.

### Runtime, independent QA and presentation

All 881 four-camera batches took 13.89 s offline, median capture 13.32 ms;
maximum axial encoding error was 5.96e-8 m. Analysis took 383.24 s before
serialization, including full static-map accumulation/copies. Neither number
establishes a real-time sensor/control pipeline.

Independent depth QA checks every capture transform and all four source/cropped
mesh groups; 64 fixed valid rays match direct source-triangle intersections to
within 0.10 micrometres. A first QA scope check stopped when unrelated mapper/
test files changed after depth capture; the corrected stage verifies all
depth-producing sources and lists those later differences. Independent map QA
checks delivery prefixes, snapshots, shared CV, first-prefix reconstruction,
fixed latest-view queries and raw summaries. Its initial parser expected an
optional observer-plane timestamp field; that failed parser stage is retained
and corrected without changing primary outputs.

- [Primary raw report](../../../../results/terrain_memory_20261126/campaign/analyze_stage01/report.json)
- [Post-registered matched supplement](../../../../results/terrain_memory_20261126/analysis/matched_pairs_stage01/report.json)
- [Scientific figure, PNG](../../../../results/terrain_memory_20261126/analysis/figures_stage01/terrain_memory_summary.png),
  [PDF](../../../../results/terrain_memory_20261126/analysis/figures_stage01/terrain_memory_summary.pdf)
- [Video](../../../../results/terrain_memory_20261126/videos/terrain_memory_shadow.mp4):
  unchanged actual saved robot, four ideal depth views, observed-only memory and
  frozen-anchor previews; early 6.4–10 s at half speed and late 30–36 s at normal
  speed, followed by the utility, early-blindness and paired-contact regression.
- [Presentation bundle](../../../../results/terrain_memory_20261126/presentation-pack.zip)

Compact completed evidence lives under `evidence/completed_stage01/`. Full raw
depth, forecasts and the large derived map snapshots remain local. The bundle
includes raw depth, exact saved-state inputs, scene inputs, sources, reports,
failure stages, figures and video; it omits the 478 MB derived full-map snapshot,
which can be regenerated. Original controller/test/config sources and earlier
artifacts are checked against their pre-change hashes when sealing the bundle.
