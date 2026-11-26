# I-41: figure-eight slope departure and turning response

2026-11-25. **Investigation completed; no controller fix claimed.** The user's
two video observations identify different effects. The early sustained path
departure is associated with changing terrain, shell/belt handoff and clipped
commands. The first turn is mainly a delayed reversal, while later turns show
genuine overshoot. Flat ground also produces overshoot.

Native 5 ms gain-scheduled LQR, contact-aware feedforward, observer and existing
120 ms kinematic synchronizer remain unchanged. The newer 200 ms dynamic
forecast was a shadow diagnostic throughout I-40 and this investigation: it
could reveal model mismatch but could not cause the recorded robot to drift.

[Annotated comparison video](../../../../results/figure8_event_audit_20261125/prediction/videos/mars_departure_and_direction_changes.mp4)
· [Shareable evidence bundle](../../../../results/figure8_event_audit_20261125/presentation-pack.zip)
· [Consolidated report](evidence/final_report.json)
· [Declared protocol](evidence/protocol.json)
· [Original I-40 record](../lqr-contact-forecast/README.md).

The 21.15 s clip replays the departure, first reversal and later sixth reversal
at half speed, followed by a labeled model diagnostic. All 423 frames decode
successfully. [Video provenance and corrections](evidence/prediction/videos/README.md)
· [Video QA](evidence/prediction/videos/qa.json).

## First departure: terrain matters, but one cause is not isolated

The declared primary event is the first distance from the local reference path
above **20 mm for at least 100 ms** after motion starts. It begins at **6.860 s**,
with confirmation at 6.960 s. Projection is restricted to reference time ±2 s
to avoid selecting the other figure-eight branch; this event does not hit that
window boundary and is lateral rather than just delayed progress.

This is not the first error of any size. A 10 mm threshold already fires during
startup at 1.000 s; 30 mm first fires at 6.910 s. Time-aligned tracking error
exceeds 20 mm at 2.690 s, largely because the robot is behind the moving target.
Those definitions are retained rather than selecting a favorable threshold.

The changing terrain and handoff overlap the primary departure:

| Time | Evidence from the saved Mars execution |
| --- | --- |
| 6.360 s | Desired-XY DEM grade is 12.60°; the geometric facet beneath the equivalent contact has grade 9.40°. |
| 6.725 s | The estimate selects belt A while shell A carries the load; belt A requests about 0.421 m/s against its ±0.300 m/s simulated bound. |
| 6.825 s | Belt A request reaches about 0.835 m/s and is clipped to 0.300 m/s. |
| 6.860 s | Actual load is on belt A, estimate selects shell A, and equivalent points differ only 0.183 mm. The gain matrix changes 23.8% from the prior sample. Belt command switches from 0.300 m/s to zero and shell A receives −0.569 Nm. |
| 6.860 s | Actual and desired local DEM grades are 12.60°. Platform tilt is 8.60°, reference tilt 12.19°; full up-vector error is 4.705°. |

Commands stamped at time t apply over the next 5 ms, so current motion reflects
earlier commands. These simultaneous symptoms do not prove which initiated
the deviation. The synchronizer prepared belt A before the event, but its
weights and command delta are all zero at the 6.860 s sample; an earlier effect
on state is still possible. No synchronizer-off or true-contact control
ablation has been run here.

One new 44 s **Gazebo Harmonic/DART flat figure-eight** uses byte-identical
reference time, XY and velocity, phase 0.9 rad, yaw 45°, gains, control cadence,
nominal friction 0.7 and the same global collision-mesh grid/triangulation.

| Metric | Original Mars | New flat |
| --- | ---: | ---: |
| First sustained local path distance >20 mm | 6.860 s | 17.735 s |
| Moving time-aligned position RMS | 50.486 mm | 16.821 mm |
| Peak time-aligned position error | 190.901 mm | 35.450 mm |
| Final time-aligned position error at 44 s | 190.191 mm | 0.115 mm |

Both runs complete without falling. Flat removes all relief, rocks and lava,
and uses the standard balanced flat initialization, so initial state and
observer history differ. Mars starts 15.809 mm from its reference and remains
12.328 mm away when the path starts at 1 s; flat starts essentially centered.
The comparison supports terrain relevance; it does not isolate
that particular small slope as the cause or establish repeatability.

## Prediction becomes less accurate around the transition

The declared event anchors are the primary departure and six smooth reference
velocity-component reversals, each at offsets −200/−100/0/+100/+200 ms, snapped
to 5 ms. All **35 private forecasts** succeed, each paired with a separate
diagnostic using the same initial model and saved future applied commands:

- **A:** cloned native LQR/observer evolves on predicted states every 5 ms.
- **B:** offline hindsight model replay uses the actual saved future commands.
  This is diagnostic information, unavailable to a deployable forecast.

Neither receives future actual states, true contacts or true terrain. Both
retain the same current observer-fitted plane/friction. All 35 reconstructed
controller histories match exactly; A/B first applied input and first 5 ms
state/contact also match exactly. Five initial load fits miss tolerance and
are retained, not filtered out.

| Forecast anchor | Horizon | A position prediction error | B prediction error with actual future commands |
| --- | ---: | ---: | ---: |
| 6.660 s | 200 ms | 15.128 mm | 22.996 mm |
| 6.760 s | 100 ms, reaching departure | 5.626 mm | 5.758 mm |
| 6.760 s | 200 ms | 12.257 mm | 19.321 mm |
| 6.860 s | 200 ms | 10.543 mm | 11.832 mm |
| 7.060 s | 200 ms | 1.473 mm | 2.325 mm |

These are prediction-versus-Gazebo errors, distinct from path-tracking errors.
Actual future commands do not remove the mismatch; an incorrect predicted
feedback sequence alone is insufficient to explain it. The frozen local
representation is a plausible contributor. Initial calibration and contact
dynamics also remain candidates: the first two onset anchors have poor load
fits, while the 6.860 s fit passes and still develops substantial future error.
This experiment does not prove that predicting terrain would fix live tracking.

At 200 ms, onset dominant-patch agreement is A 1/5 and B 4/5; turn-anchor
agreement is 26/30 for each. These are overlapping, event-selected forecasts
from one Mars run, not independent trials or population accuracy. The 105
paired nested endpoints are not 105 additional simulations.

## Direction changes: distinguish lag from overshoot

For the first six reference component reversals, geometric overshoot is the
signed distance past that component's reference extremum within [−1,+2] s.
Actual reversal requires the new velocity sign to last at least 100 ms.
A component reversal is not a stop: the other reference component is moving.

| Reference time / component | Mars reversal lag | Mars geometric overshoot | Flat geometric overshoot |
| --- | ---: | ---: | ---: |
| 11.775 s / y | +1.101 s | 0 mm | 8.03 mm |
| 15.377 s / x | +0.018 s | 0 mm | 15.62 mm |
| 18.301 s / y | −0.469 s | 0 mm | 23.44 mm |
| 23.699 s / y | −0.444 s | 9.84 mm | 28.96 mm |
| 26.623 s / x | −0.518 s | 28.02 mm | 14.97 mm |
| 30.225 s / y | +0.569 s | 60.20 mm | 26.49 mm |

Positive lag means reversal after the reference; negative means before it.
At the first Mars turn the robot turns late but never goes past the reference
extremum in this window. Later genuine overshoot supports the user's broader
observation. Flat overshoot shows that relief is not required; different
preceding states prevent attributing every Mars/flat difference to terrain.

## Decision and limitations

Keep live LQR. The next proposed experiment is a narrow, separately declared
diagnostic of transient patch/load selection and gain/drive handoff around
6.7–6.9 s, before changing weights or introducing particle filtering/iLQR.
A simulation-only true-patch comparison could help separate classification
from control/model effects; any such truth must be labeled diagnostic and
excluded from a proposed hardware controller. Turning lag/damping deserves a
separate flat-ground investigation. Neither follow-up has run in I-41.

The logged ground_normal is the piecewise-planar DEM closest-point tangent
near the wheel center, not a physical force-weighted collision normal. At the
departure the estimated normal is only 0.038° from the facet projected beneath
the equivalent contact. Large differences elsewhere between the estimate and
the logged DEM tangent are not sufficient evidence of observer failure.
An equivalent contact may average several physical contacts and fall over a
different facet; rocks/lava are excluded from these DEM evaluations. No general
multi-contact or hardware claim is made. Assumed exact simulation state and
known previous commands stand in for future robot measurements.

The horizontal authority calculation is an estimated-load-weighted contact
Jacobian with declared rate scales [5,5,5 rad/s, 0.3,0.3 m/s]. Its weak/strong
values at departure are 0.01655/0.59485 m/s; these characterize geometry under
assumed scales, not measured motor limits or a stability guarantee.

## Reproduction and preservation

[Event/command report](evidence/events/full_audit/report.json)
· [Mars/flat timeline](evidence/events/full_audit/mars_flat_comparison.png)
· [Signed commands](evidence/events/full_audit/mars/first_departure_signed_commands.png)
· [Terrain definitions and provenance](evidence/terrain/README.md)
· [Departure geometry](evidence/terrain/facet_addendum/first_departure_geometry.png)
· [A/B model report](evidence/prediction/reduction_stage01/report.json)
· [A/B prediction plot](evidence/prediction/reduction_stage01/onset_AB_xy_error.png).

The first saved-data event helper allowed one segment outside the declared
projection window and missed a reversal through an exactly zero velocity
plateau. Adversarial checks exposed both; initial sources/results and failing
logs are retained. Corrected stage 02 passes those checks. Every Mars local
distance, event time and reversal lag is unchanged; the 35-anchor file is
byte-identical. No simulation or forecast rerun was needed for this correction.

A separately declared secondary exact-segment audit checks ambiguity using
a 1 mm near-optimal distance tolerance and explicit temporal grouping. Neither
the departure nor any of the six turn windows has a competing local or remote
branch under that definition. Remote contenders exist elsewhere near the
figure-eight crossing/start/end; those are not treated as independent events.
The diagnostic criteria were declared after primary selection and did not
change the anchors. [Ambiguity audit](evidence/events/branch_ambiguity_stage/report.json).

The first video preview exposed an inset boundary-indexing error. The failed
preview, renderer source and log are preserved separately; final rendering
uses the corrected indices. This was a rendering issue, with no simulation,
forecast or measurement changes.
The initial endcard used a pre-onset anchor with a poor load fit; that version
is retained. The final endcard displays the passing-fit 6.860 s anchor and
states explicitly that all five poor fits remain in the analysis.

Independent review also corrected two exact geometry fields in the root
consolidation: the facet-angle field had copied the logged-tangent angle, and
grade used a float32 log instead of the evaluated DEM. The original report and
source are preserved; corrected fields are read directly from sealed terrain
context. Both angles round to 0.038° and the grade to 12.60°, so no stated
finding or experiment changes.

Validation consists of saved-data/adversarial helper checks, one new flat
Gazebo execution (8,801 guarded controller calls), exact history/first-step
checks for 35 A forecasts plus 35 B hindsight replays, and source/input/archive
hash checks. Earlier I-40 selected integration checks are not counted as a new
or full-repository I-41 test suite. Rendering replays recorded states; it does
not execute Gazebo again.

Exact commands, arguments, environment versions and source snapshots are in
the flat construction/provenance and forecast provenance. Restore archived
.py.txt files to their recorded origin to reproduce; reruns require fresh
output directories. consolidate.py only combines saved reports and asserts
source/input invariants. Nine core pre-change sources and all five older
presentation bundles remain hash-identical. The preceding interrupted general
load-fit-improvement preflight was read-only and did not implement a change.

[Compact evidence manifest](evidence/manifest.json)
· [Presentation payload manifest](../../../../results/figure8_event_audit_20261125/presentation_manifest.json).
The repository-side [archive verification](presentation_verification.json)
is created after bundling and stays outside the ZIP, since it records the
final ZIP's own hash. The payload manifest is included in the ZIP.
