# I-40 videos: trajectories and a private contact forecast

Both clips render saved states only. No new Gazebo execution, integration,
controller command or forecast is performed during rendering.

- `native_lqr_trajectory_overview.mp4`: 35.75 s, 1280×910, 20 fps. Three chapters
  show the same unchanged native 5 ms LQR/feedforward policy on flat X at 1×,
  flat out-and-back at 1×, and the actual ERC2021 Mars-yard figure-eight at 2×.
  Mars uses `MarsYard`'s DEM and the existing cropped rock-visual renderer,
  rather than a flat stand-in. Completion is not declared gate success: the
  final Mars state visibly retains about 190.19 mm error and 8.71° reference-
  relative attitude error.
- `contact_forecast_vs_gazebo.mp4`: 10.05 s, 1920×926, 20 fps. Left is recorded
  Gazebo/DART future; right is a stored PRIVATE estimated-plane forecast using
  the same 5 ms native feedback policy on predicted states. The 200 ms interval
  is played at 0.025×, with one-second endpoint holds and 50/100/200 ms markers.
  This is a model-versus-recording comparison, not two live controllers.

The intended `flat_y_transfer` case has no stable shell/belt family transfer.
The forecast example follows the root-approved, predeclared fallback: first
late-X stable family event at 9.260 s, forecast anchor 9.160 s. It is explicitly
labeled a stress example. Raw loads chatter; event classification uses the
separate 15 ms dwell rule. The example was not selected for forecast quality.
Truth conditions the evaluation window only and is never supplied to the
forecast or controller.

The private panel exposes its initial normal-load fit residual of −1.966 N,
outside tolerance. Both force plots use the same 85 N vertical scale. The
private reference-tilt display is the known anchor reference, because future
cloned-policy reference quaternions were not stored; the actual panel shows
its logged reference. Private force samples come from the final physics solver
step, 1 ms before the endpoint's integrated state, as documented by the API.

`overview_video_metadata.json` and `forecast_video_metadata.json` retain exact
input/source/media hashes, playback timing, chapter or example selection and
decoded-final-frame verification. Render source is preserved under
`source_snapshot/`. Final stills include the decoded last frame of each MP4.

Development evidence remains under `development/`: the first successful
overview endpoint previews, and a forecast preview whose frames rendered but
post-render metadata failed because a relative selection path was compared
against an absolute repository path. The failure log, exact source and all
four preview frames were preserved. Resolving the selection path corrected
that bookkeeping error; no numerical forecast or simulation was rerun.
Corrected preview files use distinct names so final renders do not overwrite
them. Generated viewer assets are under `_render_scenes/` and may be inventoried
instead of copied into compact evidence.

Original render commands, from the repo root with a fresh output stage:

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -u results/lqr_contact_forecast_20261125/videos/render_contact_forecasts.py --mode overview
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -u results/lqr_contact_forecast_20261125/videos/render_contact_forecasts.py --mode forecast --selection results/lqr_contact_forecast_20261125/videos/forecast_selection.json
```

The script rejects existing MP4/metadata targets. Reproduce in a fresh copy of
the campaign folder or choose a new rendering-stage destination; retain the
raw campaign at its recorded paths. `--preview` produces endpoint/milestone
stills before final encoding.
