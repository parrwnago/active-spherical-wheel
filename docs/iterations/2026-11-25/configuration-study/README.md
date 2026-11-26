# Configuration and continuity study — I-12 to I-14

**Question:** Before introducing particle filtering, can configuration-dependent,
continuously varying allocation weights address the difficult corrections?

**Work completed:** 17,739 offline kinematic configurations, 141,912 requested
travel directions, four boundary scans, saved-event continuity analysis, and
40 short Gazebo/DART comparisons of existing controller settings. Controller
defaults and code remain unchanged. No new weight law or particle filter was
implemented.

[Research history](../../../../ITERATION_LOG.md) · [Snapshot manifest](manifest.json)

## 1. Configuration and direction both matter

Show [the same-attitude directional example](figures/directional_example.png).
At -20° platform roll and fixed shell-B support, changing shaft phase changes
the motion directions from independent to aligned. At the singular phase,
forward motion remains feasible while sideways motion loses the requested
60 mm/s. A scalar tilt signal cannot express this direction dependence.

The grid covers roll/pitch -20° to +20°, shaft phase -180° to +180°, zero yaw
and shell material angles, flat plus two 10° slope normals, and eight horizontal
travel directions. Normalization and assumed speed bounds match the previous
allocator study. Acceleration limits are not applied in this instantaneous map.

- [Authority map](figures/authority_map.png)
- [Bounded velocity-feasibility map](figures/feasibility_map.png)
- [Selected support map](figures/support_map.png)
- [Numerical summary](map_summary.json) and [coverage/symmetry audit](map_audit.json)

Synthetic contact records are used only for offline analysis. These are not
dynamic balance/friction tests or verified motor capabilities. Rim fallback
candidates have a plane-height inconsistency up to 1.666 mm and are excluded
from patch-interior interpretation. Grid fractions are not motion-failure
probabilities.

## 2. A smooth pose can still cross a discontinuous contact model

Show [the boundary scans](figures/boundary_continuity.png). One boundary has a
roughly 30.16 mm/s feasibility jump across a ±0.0001° phase perturbation. The
available drive channels change at the shell/belt selection boundary.

Then show [the saved-event continuity audit](figures/continuity.png). Median
contact-map changes are approximately 128–140 times larger at patch changes.
The active coefficient can change rapidly within a patch too. Bounded reference
steps do not bound LQR corrections, which reach 5.79 actuator limits per step
in the adaptive event window.

All six estimated changes in the inspected baseline window match actual
dominant-patch changes. It is inaccurate to present this solely as observer
chatter. These measurements support inspecting the handover, without proving
one cause of the excursion. See [continuity evidence](continuity_summary.json).

## 3. Controlled Gazebo cases retain the difficult outcomes

Flat ground and a 10° side slope; phases 0.60 and π/2; nominal compensated fork
pitch offsets 0°/+3°; hold or x/y travel with the same smooth 0.06 m/s ramp.
Eight common holds and 16 least-squares/adaptive moving pairs give 40 trials.

All finish four seconds, with 32,040 controller calls guarded against solver
contact reads. All 16 adaptive runs obey reference speed/acceleration bounds.
This is a short mechanism comparison, not route qualification or a new policy
success. Actual joint speed can exceed the reference cap.

At side-slope phase π/2 and zero nominal fork offset:

| Requested direction | Least-squares RMS | Existing adaptive RMS | Estimated patch changes |
| --- | ---: | ---: | ---: |
| World x | 16.26 mm | 16.36 mm | 0 / 0 |
| World y | 275.07 mm | 278.52 mm | 66 / 74 |

Show [directional traces](figures/controlled_directions.png) and
[all matched comparisons](figures/controlled_pairs.png). At another phase the
difficult direction changes, so do not generalize world y as always difficult.
The LS/adaptive comparison changes several allocator features together; it
does not isolate the scalar scheduling law.

The illustrated baseline y case turns approximately 87.8° in heading, versus
4.5° for x. Heading is free; do not interpret either trajectory as a fixed
Jacobian experiment or attribute all of the error to the scalar weight.

Videos, each replaying four seconds at 0.5×:

- [Same baseline, x versus y request](../../../../results/configuration_study_20261125/videos/baseline_world_x_vs_world_y.mp4)
- [Same y request, baseline versus existing adaptive](../../../../results/configuration_study_20261125/videos/world_y_baseline_vs_adaptive.mp4)

Both videos replay saved Gazebo/DART states using MuJoCo rendering. Their labels
distinguish different reference directions from different algorithms and avoid
claiming an improvement from a newly implemented policy.

## 4. Preserve the initialization limitation

The compensated +3° fork rotation preserves the initial physical carrier/contact
geometry, but the static observer reset shifts estimated contact about 6.45 mm
and changes the lean reference. Initial reference-relative errors are below
1.3e-6°, not 3°. This did not isolate a large balance-error priority condition.

The first 0.3 s is reported as a cold-observer window. Side-slope placement also
starts the body center about 20.83 mm from reference XY; that offset is shared
inside every matched allocator pair. Preserve these features when interpreting
short-run tracking and hold behavior.

The first map smoke run also had a plotting API error. It was fixed, but its
original failed log was overwritten; [the issue record](smoke_plot_issue.json)
is explicitly retrospective. Later source versions and successful checks are
preserved instead of presenting the pipeline as error-free.

## Decision and next experiment

Retain current defaults and defer particle filtering. The next proposed single
A/B change is smooth shell/belt feedback transfer with the current contact
estimate and allocator held fixed. Check attitude recovery carefully: reducing
a feedback jump can also weaken a needed response. Direction-specific travel
relaxation and warm-start balance-priority tests remain separate candidates.

This study establishes where to investigate and how to compare; it does not
establish that smoother weights or feedback blending will improve the robot.

**Subsequent priority correction (I-16).** The user pointed back to the original
common-shaft/hemisphere issue at one fixed contact. The handover-focused proposal
above addresses a separate discontinuity. The next allocator experiment should
first use the valid fixed-shell-B sweep to assess smooth configuration/direction
weighting and motion-reference feasibility, then test handovers separately.
The existing same-contact map already demonstrates loss of one motion direction
without a contact change. See
[I-16 in the research log](../../../../ITERATION_LOG.md#i-16--correct-the-next-test-priority-common-shaft-and-hemisphere).
No new controller or simulation was run for this correction.

## Reproduce and preserve

From the repo with its existing environment:

```bash
OPENBLAS_NUM_THREADS=1 .venv/bin/python scripts/map-rolling-configurations.py --output results/new_configuration/map
```

Inside the Gazebo environment from [GAZEBO.md](../../../../GAZEBO.md):

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 python scripts/run-configuration-cases.py --output results/new_configuration/controlled --workers 3
```

Compact reports, figures, source snapshots, and hashes are preserved here.
Raw mapping CSVs and NPZ rollouts remain under
`results/configuration_study_20261125/`. Archived analysis scripts retain their
original location assumptions. The numerical map and later plot-only source
versions are identified separately. There was no full-repo pytest rerun: code
checks here are map assertions, source/input audits, initialization invariance,
reference-bound checks, and guarded Gazebo execution.

The local `results/configuration_study_20261125/configuration-study-pack.zip`
contains this presentation/evidence index, snapshots, raw mapping tables, and
the two selected clips. Earlier I-01–I-10 clips remain in their first bundle.
This archive captures the study and I-15 proposal before the I-16 priority
correction; the current log preserves that subsequent discussion.
