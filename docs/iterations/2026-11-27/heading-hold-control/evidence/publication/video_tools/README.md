# I-74 media tools

Prepared before opening any new I-74 result arrays. Renderer derives from the
corrected I-73 saved-pose renderer; `source_i73/` and `adapted_from.json` retain
that provenance. It never steps a simulator or executes a controller.

Expected root release: `campaign/outputs_sealed.json`, with six N/H cases and
base-relative file hashes. Each rendering stage binds source, route, protocol,
DEM/feature assets and all six data files before opening the arrays. Root seal
and per-case artifact manifests must pass first.

The renderer uses exact recorded time/qpos/qvel/reference from `precision.npz`.
Native float32 rollouts must equal the original-dtype cast of those precise
fields. The actual latched heading target and recorded wrapped error are also
verified. The display shows wrapped target error, unwrapped yaw, physical
world-vertical angular speed, heading-torque norm and separate body/reference
tilt. Initial phase is the requested ground-relative configuration, while the
shaft angle in the HUD is the actual recorded joint angle.

After an explicit all-six root release:

```bash
.venv/bin/python results/heading_hold_control_20261127/publication/video_tools/render.py --phase m100 --stage results/heading_hold_control_20261127/publication/video_preview_stage01/m100 --input-seal results/heading_hold_control_20261127/campaign/outputs_sealed.json --preview
```

Repeat for p030 and p090. Root visual review precedes full rendering into fresh
`videos_stage01/<phase>` stages without `--preview`. Each video retains all
0–45 s of saved time, with 901 inclusive frames at 20 fps. Camera is world-fixed
and shared across all six actual paths plus the full fixed S reference.

Five media contract functions pass: four reused I-73 orientation functions and
one new heading-target/wrap function. These are not a full repository test suite.
