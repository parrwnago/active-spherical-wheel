# I56 fixed saved-state comparison

The plan fixes B/native, A/bounds+continuity and H/full at the −1.00 rad start before their outcomes are read. The overview always spans 0–44 s at 4×. A case that ends early holds only its final saved pose, with a prominent ended label; traces stop at the final observation. The fixed 28–36 s slow window is eligible only when all three cases cover it.

The renderer builds a display-only Mars scene and uses static kinematics on actual saved Gazebo poses. It guards dynamics, collision solving and integration entry points. Old helpers are copied unchanged as reference-only provenance; their original path globals are never rebound or executed.

After the parent seals all three cases and authorizes rendering:

```sh
MUJOCO_GL=egl .venv/bin/python results/constrained_handoff_20261126/video_tools/render_comparison.py --campaign-ready --mode preview --stage preview_stage01
MUJOCO_GL=egl .venv/bin/python results/constrained_handoff_20261126/video_tools/render_comparison.py --campaign-ready --mode render --stage render_stage01
```

Inspect the preview images before encoding. Each attempt uses a fresh stage; preserve failed stages and source snapshots. Metadata, input hashes, exact saved-state/frame clocks and decoded QA frames accompany each video. The full 5 ms logs and registered analysis determine scientific outcomes; video completion alone does not establish acceptance.
