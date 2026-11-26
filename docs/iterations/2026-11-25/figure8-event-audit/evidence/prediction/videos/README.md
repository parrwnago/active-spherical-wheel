# I-41 saved-state event presentation

[Watch the 21.15-second clip](mars_departure_and_direction_changes.mp4).
The three chapters show matched program-time windows on the actual Mars DEM
and the actual flat counterfactual: first sustained departure at 6.860 s,
first reference Y-component reversal at 11.775143 s, and turn06 at 30.224857 s.
Each window spans event −1 to +2 s and plays at 0.5×. Rendering selects the
nearest saved 5 ms state; the two sides use exactly matching time/reference arrays.
The common chart scales and fixed XY inset bounds support direct comparison.

These are recorded Gazebo/DART states displayed with the MuJoCo renderer, not
new simulations. Both use the unchanged native 5 ms LQR, observer and active
120 ms kinematic synchronizer. Terrain relief, rocks/lava and the appropriate
balanced initial pose differ. The comparison does not isolate slope alone.
DEM slope is geometric terrain information, not a measured collision normal.

The first reversal shows lag with zero Mars geometric component overshoot;
the later turn shows 60.20 mm Mars versus 26.49 mm flat geometric overshoot.
The final card is a separate model diagnostic. A uses native feedback on private
predicted states; B replays saved **future actual commands** only as offline
hindsight. Neither dynamic forecast commanded the actual robot. The displayed
6.860 s anchor passes its initial load fit; all five poor fits remain in the
35-anchor result set. Predictive mismatch also remains with actual inputs.

Exact render source is preserved in `source_snapshot/`, with input hashes in
[video_metadata.json](video_metadata.json) and [video_provenance.json](video_provenance.json).
[qa.json](qa.json) records all-frame decode, source/input/still hashes and
matched-window checks. [Independent prediction QA](../saved_data_QA/report.json)
checks saved forecast paths, bounds, hindsight inputs, initial load-fit labels
and all summary groups without executing a model.

Reproduction, into a fresh output stage because the tool protects existing files:

```sh
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -u results/figure8_event_audit_20261125/prediction/videos/render_figure8_events.py
.venv/bin/python -u results/figure8_event_audit_20261125/prediction/videos/verify_video.py
```

Development evidence is retained. `development/stage01_preview_boundary_failure/`
contains the initial preview error: a continuous window endpoint selected the
nearest saved state outside the strictly filtered inset. The final source uses
the same nearest-state endpoints for rendering and inset extraction.
`development/stage02_initial_endcard/` preserves the first complete presentation;
its endcard used the 6.760 s poor-fit anchor without an explicit load-fit caveat.
The final presentation uses the well-fitted onset anchor and discloses all five
poor fits. These are rendering/annotation corrections, with unchanged numerical
forecasts, actual traces and event selection. The successful corrected preview
predates the final endcard wording; its exact source remains separately preserved.
