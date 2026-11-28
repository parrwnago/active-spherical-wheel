# I-78 saved-state comparison videos

These are display reconstructions of the six closed Gazebo/DART experiments.
MuJoCo renders saved state; it does not rerun physics, the observer or control.
The original LQR, momentum observer, contact/feedforward stack and actuator caps
are retained. The G candidate changes the rate of progress along the same path.
Controller acceptance is evaluated separately from these videos.

Each N/G pair shares physical elapsed seconds at 1x, 10 frames/s. Reference
progress is never normalized to make the panels line up. Each panel freezes its
last recorded pose, reference and telemetry after its real endpoint, displays a
red `RUN ENDED` banner, and retains its last recorded timestamp. The final frame
may extend less than 0.1 s beyond a fractional endpoint; that interval is marked
ended, not extrapolated. The final emitted command is unheld.

The camera is fixed across all three comparisons. Its bounds cover all six
recorded paths and the full shared geometric path. The display model uses the
captured configuration; named robot geometry and inertial parameters are checked
against all six raw mirror XML models. The raw physical SDF and mirror XML are
bound by the rendering input receipt. Terrain is the original Mars DEM and mesh
display reconstruction; the visuals do not establish collision or contact truth.

The renderer verifies the complete six-output seal and actual process exits,
binds source/model/input hashes, and writes `registered_before_arrays_and_render`
before loading arrays or rendering. It verifies the bindings again afterward.
Physics integration entry points are forbidden during scene compilation/rendering.
Every completed MP4 is decoded sequentially to verify dimensions, frame count,
frame rate and duration. Representative pose and decoded frames are preserved.

Reproduce locally from the retained closed campaign, choosing a fresh stage:

```sh
.venv/bin/python results/path_progress_governor_20261128/publication/render.py \
  --phase m100 \
  --stage results/path_progress_governor_20261128/publication/video_m100_NEW \
  --input-seal results/path_progress_governor_20261128/campaign/outputs_sealed.json
```

Use `p030` and `p090` for the other starts. `--preview` writes selected saved-state
frames only. Source snapshots and three scoped pure presentation contracts are
in `contracts_stage01` and `contracts_stage02`; stage02 adapts only the helper's
seal reader to the actual final seal schema and protocol location. These are the
same three contracts, not six independent tests. A sealed-data preview is in
`preview_stage01`. All frozen experiment sources remain unchanged.

`assemble_bundle.py` packages the videos, representative frames, renderer source,
model/config snapshots, output seal and compact receipts. The shareable visual
bundle does not duplicate large rollouts; replaying the render requires the
closed campaign files at their registered local paths.
