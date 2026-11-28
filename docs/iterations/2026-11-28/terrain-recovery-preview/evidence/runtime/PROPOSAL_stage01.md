# I-84 proposed: future-only observed-map incoming preparation

Status: source audit and interface proposal only. No implementation, test,
saved-array read, map capture, controller trial or physics execution yet.

## Question and scope

At the same 0.60 m/s operating belt capacity as I-82 B060, can previously
delivered four-camera depth observations improve preparation of an incoming
unloaded shell/belt when the terrain normal ahead differs from the current
observer plane? This is a narrow anticipation experiment, not a claim that
map-assisted preparation explains or cures the large trajectory departure.
Root first quantifies recovery in existing records and decides whether this
mechanism is worth a live comparison.

Retain the original timed 133 s eight-shaped path, phase -1 rad, yaw 158 deg,
LQR library and ordinary design, momentum observer/support locator/current
normal filter, least-squares rolling allocation, support feedforward, fade,
heading disabled, torque bounds and existing synchronizer gains/blends.
Both arms use physical and mirror operating bounds [1.5,.6,.6,.6,.6] after
the original 0.30 m/s design is constructed. No retiming or terrain lookahead
is inserted into loaded-drive holding feedback.

## Reused source interfaces

- `src/spherical_wheel/terrain_depth.py`: `four_camera_rig`,
  `TerrainRayScene.from_mars`, `FourCameraDepthAdapter.render_batch`, immutable
  `DepthFrame`. Only the sensor adapter sees collision scene geometry. Returned
  frames contain depth, calibration, capture pose and times, never hit IDs,
  surface normals, load, friction or future robot poses.
- `src/spherical_wheel/terrain_memory.py`: `MemorySettings`, `TerrainMemory`.
  `ingest(frames, available_at_s)` rejects frames not yet delivered. Query
  uses only observed cells and local geometric-fit gates. This is a static
  2.5D geometric prior; its confidence is not a calibrated probability.
- `src/spherical_wheel/synchronization.py`: `ContactSynchronizer.preview`,
  `_support`, `_velocity`, `_normal_loads_records`. Existing preview uses a
  private kinematic `MjData`, measured current velocity and 0.12 s CV poses.
- `src/spherical_wheel/offroad_control.py`: `OffroadController.synchronizer`
  is replaceable per instance. The command already separates estimated
  current records/plane from synchronization; no production change is needed.
- Original I-82 `results/current_belt_capacity_20261128/runner/run_case.py`
  and unchanged I-76 runner supply the matched physical/mirror capacity hook,
  one observer/preview per row, fixed-design and truth-access guards.
- `scripts/run-terrain-memory.py` supplies the prior frame-capture/latency
  convention; its saved-trace runner is not reused as a live controller.

## Minimum proposed module/API

New `runtime/map_preparation.py` imports only `math`, `numpy`,
`spherical_wheel.synchronization.ContactSynchronizer` and observed-memory
types. It receives a provider exposing `query(xy, at_time_s)`; it has no scene,
DEM, terrain evaluator, contact truth, physical plant or saved future-state API.

`MapPreparationSynchronizer(native_synchronizer, provider)` copies the native
constructor settings, reuses its measurement mirror, and subclasses only
`_support(patch, data)`, with an anchor context around the unchanged parent
`preview(primary_command, records, ground)` call. A finite anchor time/current
qpos is latched once. All map queries use the anchor time, even though their
XY locations belong to CV poses inside the existing 0.12 s horizon.

For the live plant/current private qpos, `_support` returns the exact original
result and performs zero map queries. For a different private future qpos:

1. Query the observed local plane at the predicted patch-body centre XY.
2. Use its unit normal to form the virtual spherical support point.
3. Query that point's XY too. Require both supported fits and <=3 mm
   disagreement between the centre plane's predicted foot height and the
   separately observed foot fit, as in the old contact-preview consistency
   check. Otherwise return the exact original current-plane `_support` result.
4. Compute the original signed band margin and clearance using the accepted
   mapped normal/plane. No support identity or normal load is asserted.

Only future activation, support gap and target tangent/projection change.
Current load fractions, current point velocities and outgoing velocity remain
the original observer-based values. The unchanged parent preview retains the
>=5% load exclusion, .35 shell blend ceiling, 20 rad/s shell target ceiling,
.025 N m s shell speed gain, .004 m lever regularization, .012 m transition
margin, .02 m support-gap test and physical torque/belt bounds. No new effort
gain or normal-blending parameter is introduced.

The parent preview is called exactly once per command. The subclass uses the
parent's current-vs-future private qpos distinction, not inferred execution
delay. A future pose that is numerically identical to the current pose falls
back to native support; this is conservative and avoids misclassifying a
stationary preview as new future terrain. Current holding plane, observer,
estimator, gain selection and support torque never receive the future normal.

New `runtime/causal_depth_memory.py` imports `collections.deque`, `numpy`,
the existing four-camera adapter/rig and terrain-memory types. It owns only
capture scheduling and a delivery queue. Capture is 20 Hz at the actual current
measured qpos; simulated delivery is capture+0.050 s. At time t, dequeue only
frames delivered by t before querying the map. A fixed optical scene comes
from the exact world collision DEM and feature meshes, cropped using declared
route/domain bounds plus 2.30 m, never future executed positions. Scene data
stay inside the adapter; the controller receives the observed-memory facade.

Reuse exact simulated capture-time pose for registration, explicitly assuming
perfect localization. Preserve the old four cameras: azimuths 0/90/180/270 deg,
64x48 each, 100 deg horizontal FOV, 45 deg downward pitch, .08-2 m range,
body mounts from `four_camera_rig`. Memory settings remain 10 mm cells,
40 mm normal radius, six supporting cells, 3 mm RMS/3 mm XY spread gates,
60 deg slope ceiling, 20 mm within-cell height-spread gate and no age expiry
on the declared static exact-pose map. Do not use an evaluator fallback.

## Contracts before captured-array audit

1. Unsupported or missing provider: exact native result, commands and weights;
   no current-state mutation, added observer update or preview call.
2. A mapped slope changes a constructed future preparation target while all
   current margins/actual velocities/load fractions/outgoing velocity match
   native; a >=5% loaded drive retains zero preparation weight.
3. Every provider query is at the frozen anchor clock and a different private
   future pose; no current normal/record is replaced and no future pose read.
4. Unsupported centre/foot, nonlocal foot disagreement or invalid geometry
   returns native support exactly; clear rejection reasons are logged.
5. Queue releases only captured/delivered frames, once, in time order; all
   four views use their own exact capture transform and fixed 50 ms latency.
6. Returned targets/blended commands respect inherited mixed-unit bounds and
   shell blend limits, including near-zero lever and stationary CV fixtures.
7. A read-only static scene fixture verifies self-first-hit masking and the
   mapper sees no geometry/hit IDs. Reuse existing depth/memory contracts.

Use generated small static kinematic fixtures, never launch Gazebo or load a
recorded rollout for these contracts. Preserve pre-test source snapshots/logs.

## Required causal visibility/action audit before any live arm

Root must register the exact source/import/input/metric hashes and audit
settings before reading I-82 B060 arrays. Traverse the saved B060 history in
time order. Only current/captured qpos and delivered depth enter the mapper.
At each frozen audit anchor, provide current measured q/v, original applied
history and observer-estimated records/plane. Current and map preparations
are kinematic/shadow only; do not execute a controller or update an observer.
Future executed poses/contact forces may enter a separate evaluator afterward
and must never gate online map acceptance.

Audit full-route availability plus event/recovery strata selected by I-83;
report observed centre coverage, complete centre+foot coverage, fit rejection
reasons, which future drive would be prepared, nonzero blend opportunities,
actual requested preparation-command changes, and command bounds. Count all
anchors in denominators, including abstentions. Distinguish map geometric
coverage from CV configuration prediction and actual load-transfer correctness.
If no meaningful nonzero opportunities exist in the targeted terrain event,
stop after the audit rather than launch a zero-action live comparison.
Root defines the numerical utility threshold and live launch decision before
these outcomes; the implementer does not tune after seeing them.

If the audit justifies it, freeze one full 133 s B060-native vs map-preparation
pair with both operating caps .60, untouched current holding feedback and no
parameter search. Require native replay parity and identical prefix through
the first actual map-caused command difference. Compare event recovery, full
tracking/balance, saturation, loaded-transfer speed mismatch and completion;
the candidate is not adopted merely for finishing. New evaluation logging
must remain read-only and source-bound.

## Runtime and known limitations

I-44's recorded offline 881 four-camera batches took 13.89 s, median capture
13.32 ms; its larger analysis took 383.24 s with map copies. These are prior
measurements on another trace, not a real-time guarantee or a runtime forecast
for this case. Avoid deep-copying the whole map per 5 ms control call, cache
repeated per-preview geometry queries locally, and report measured wall time
separately from the simulated 50 ms delivery latency.

I-44/I-45 previously found useful surface memory but no aggregate contact
forecast advantage; early visibility/foot coverage was weak. CV body/joint pose
prediction can still be wrong, terrain can be nonplanar, and geometry supplies
no load shares/friction/slip. Preparation cannot hold the robot against a
slope or repair a bad current support assignment. Its unloaded-shell torque
still reacts on the carrier. A null audit or worse live recovery is a valid
recorded result, not a reason for unregistered tuning.
