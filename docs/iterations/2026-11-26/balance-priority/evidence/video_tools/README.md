# I58 fixed saved-state comparison

The plan fixes B/native, A/old hard-rate allocator and P/balance priority with soft change at the −1.00 rad start before I58 outcomes are read. The overview spans 0–44 s at 4×. Early-ended cases retain only their last saved pose with a prominent ended label, and their traces stop. The fixed 28–36 s slow window requires observed coverage from all three cases.

The display-only Mars scene draws actual saved Gazebo poses with static kinematics. Dynamics, collision solving and integration APIs are guarded. Absolute tilt, raw-reference up-axis error and physical world-Z angular velocity have distinct labels. Original I56 helper sources are preserved in `original_I56_reference_only/`; they are not edited or executed with rebound paths.

Render only after the root seals all three cases and authorizes outcome reads:

```sh
MUJOCO_GL=egl .venv/bin/python results/balance_priority_20261126/video_tools/render_comparison.py --campaign-ready --mode preview --stage preview_stage01
MUJOCO_GL=egl .venv/bin/python results/balance_priority_20261126/video_tools/render_comparison.py --campaign-ready --mode render --stage render_stage01
.venv/bin/python results/balance_priority_20261126/video_tools/audit_media.py
```

Inspect preview images before encoding. Each attempt needs a fresh stage; retain failed attempts and source snapshots. Hashes, exact frame/state clocks, decoded contact sheets and metadata accompany the clips. Video completion does not determine candidate acceptance. This tool does not run a controller, observer, model forecast or physics simulation.
