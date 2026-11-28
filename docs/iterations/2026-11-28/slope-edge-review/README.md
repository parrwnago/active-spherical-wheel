# I-80 — the visible descent is real; its full cause is not isolated

2026-11-28. Completed video, saved-state and collision-mesh review. No new
simulation, controller/estimator change, tuning or acceptance trial.

The user identifies the **left/native panel of I-76 eight −1 rad at 101.500 s**
and suggests the robot rolls down a slope rather than visibly executing a
wrong velocity. We decoded the exact original video and reviewed 99–103 s of
the original native recording. This is a different episode from I-78's straight
retiming comparison; it does not revise those scores.

**Best current description: the robot rolls downhill while losing path tracking,
with rapid rotation and saturated correction.** The descent is directly
supported by the mesh and motion. It is not, by itself, evidence of a false
contact or an incorrect command. Neither is gravity established as the sole
cause: command routing changes rapidly in the same episode, and forces/work
have not been separated causally.

[Motion/contact PDF](evidence/support_plot_stage01/support_motion.pdf) ·
[motion/contact figure](evidence/support_plot_stage01/support_motion.png) ·
[terrain profile](evidence/terrain_stage02/terrain_profile.png) ·
[exact decoded user frame](evidence/visual/frames_stage01/frame_101.50s.png) ·
[original full comparison video](../../../../results/tangent_heading_routes_20261127/publication/videos_stage01/eight/m100/eight_m100_N_T.mp4).

## What is verified at this frame

| Quantity at 101.500 s | Recorded result |
| --- | ---: |
| Actual contact-location mesh slope | 8.644° |
| Wheel-centre downward velocity | 73.419 mm/s |
| Terrain-height decrease along current XY motion | 73.420 mm/s |
| Actual XY speed / desired XY speed | 0.5032 / approximately 0.0219 m/s |
| Positive pressure on shell A | 17.407 N |
| World-z rotation | −199.9°/s |
| Timed position error shown in video | 201.677 mm |

The near equality of vertical speed and terrain-height rate supports motion
following the slope. Current XY velocity is 16.313° from the local downhill
direction. Over 100–102.5 s the wheel centre drops 33.919 mm and the mesh
directly below its XY position drops 32.495 mm. Model XML confirms that the
free-joint origin is the spherical wheel centre, rather than the upper platform.
Centre height/acceleration is nevertheless not whole-system COM motion.

All **801 saved samples in 99–103 s** have positive wheel pressure, minimum
10.282 N. There is no demonstrated sustained airborne fall. These are the last
completed 1 ms solver samples saved every 5 ms, so brief unloading between
samples is not excluded. Actual pressure-mean points agree closely with the
retained collision mesh; a mean can still represent several contacts.

The retained mesh is a continuous triangulated height field. There is a real
nearby grade break, not a vertical cliff. The dominant pressure-mean facet
changes around **101.550 s**, from 8.644° to 4.813° slope (7.229° normal-direction
change). This is after the visible acceleration is already developing. The
first clipped-command marker at 100.835 s has no simultaneous
pressure-mean terrain-facet change; its pressure is on belt A. A slope can
contribute continuously without a new terrain edge at the onset.
This selected marker is not an independently detected first instability time.

## What the video alone misses

Timed error is already 44.819 mm at 100.5 s and 77.441 mm at 101.0 s, before
reaching 201.677 mm in the user frame. The earlier I-77 history also records
residual velocity and reference disturbances before this interval. The visible
edge cannot be assigned as the start of every part of the departure.

There is a rapid allocation of feedback between drives around the frame:

| Time, s | Last pressure family | Inferred dominant family | Requested → applied command |
| --- | --- | --- | --- |
| 101.495 | shell A | belt A | belt A −1.966 → −0.300 m/s |
| 101.500 | shell A | belt A | belt A −1.923 → −0.300 m/s |
| 101.505 | shell A | shell A | shell A −5.020 → −0.600 N m |
| 101.510 | shell A | shell A | shell A −4.516 → −0.600 N m |
| 101.515 | shell A | belt A | belt A −1.775 → −0.300 m/s |

The gain rows change with inferred support. At 101.500 s the equivalent-point
disagreement is only 0.496 mm. Small point error does not guarantee stable drive
assignment. Over 99–103 s the dominant families differ at 52/801 samples;
**this is not an observer-error count**. Instantaneous positive-pressure samples
and filtered completed-interval wrench have different timing and meanings.
There are also real pressure-family changes in the wider interval.

Each displayed command is calculated after the displayed state and acts next.
Clipping or a large corrective request can be a reaction to motion. This
accounting does not establish whether feedback or routing initiates, amplifies
or mitigates the descent. The acceleration, slip, spin and caps are correlated;
the analysis has no independent gravitational/actuator work decomposition.

## Decision and next diagnostic

Keep original N selected. Describe this observation as a loss of path tracking
during supported downhill motion, without claiming a wrong preview, false
contact, inevitable gravity-only drift or a proven controller bug. The video
shows the physical motion; it does not identify what caused each force.

The narrow proposed next check is the unchanged-native instrumented replay
from I-79, placing its fine contact/interval-wrench/preview ledger around the
**100.75–101.65 s acceleration and drive-transfer interval**. This would align
physical load transfer with inferred routing and preparation. A separate
matched-prefix intervention is needed to establish whether one component makes
the departure worse. Neither replay nor intervention is run in this review.

## Evidence, setup history and reproduction

[Support report](evidence/support_stage01/report.json) ·
[routing ledger](evidence/routing_stage01/report.json) ·
[mesh/centre report](evidence/terrain_stage02/report.json) ·
[actual pressure-facet follow-up](evidence/terrain_stage03/report.json) ·
[compact evidence manifest](evidence/manifest.json).

Source/input registrations precede each new extraction; their timing is
retrospective to the original recorded run, not a blinded new trial. Exact
scripts and output manifests are retained. Eight original video frames are
decoded at 10 fps without rendering new poses. Original I-76 raw arrays/mesh
remain separately identified local prerequisites, with hashes in the reports.
No previous trial/report/archive is changed.

The first terrain-analysis attempt repeatedly converted all 250,000 vertices
for every query and was explicitly terminated, observed exit143. Its original
source, registration and setup failure remain preserved. Stage02 caches the
immutable vertex array; query arithmetic/windows do not change. System ffprobe
was unavailable; installed imageio/packaged FFmpeg decodes the video instead.
These are analysis/tool setup corrections, not simulated robot failures.

Restore the captured result-owned sources at their recorded paths to repeat
the read-only reductions. Use new output stages and retain original inputs;
the scripts refuse to overwrite closed stages. No tests or physics are rerun
for this evidence review; validation checks hashes, links and extracted values.
