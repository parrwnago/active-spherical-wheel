The three phase pages show every N/A/C method over the entire 45-second
recording, including the 10-second goal hold. A single static camera is selected
from all nine actual and reference paths. Failed or truncated runs retain a
visibly marked final pose; their trajectories are not cropped away.

The robot poses come from the recorded Gazebo/DART runs. MuJoCo renders these
poses over a visual height field sampled from the original Mars DEM and the
original rock/lava meshes. It never integrates physics, evaluates contact, or
reruns the controller. The display's terrain interpolation is not evidence of a
new contact solution.

The HUD distinguishes absolute tilt from reference lean and raw-reference up
error, yaw from physical
world-z angular velocity, initial phase from current shaft angle, and requested
display time from the recorded pose time. Native (N), Velocity (A), and Geometry
+ velocity (C) are presentation labels for the three registered contact-routing
policies; only the rolling-reference contact records differ. The header's start
phase is the requested ground-relative initial configuration. The setup adjusts
the initial shaft joint for terrain normal, so that phase is distinct from the
actual current shaft joint angle shown in the HUD.

After all nine outputs have been sealed and explicitly released:

```bash
.venv/bin/python results/contact_reference_control_20261127/publication/video_tools/render.py \
  --phase m100 \
  --stage results/contact_reference_control_20261127/publication/video_preview_stage01/m100 \
  --input-seal /absolute/path/to/the/all-nine-output-seal.json --preview
```

Use a fresh stage and omit `--preview` to encode the full video. Repeat for
`p030` and `p090`. Each invocation snapshots source and input hashes before
reading the recorded arrays, verifies the original run manifests, and checks
that sources and inputs remain unchanged. Render receipts preserve camera,
time-to-state indexing, encoded-frame checks, and output hashes.

The first preview stage is retained with its pre-render loader failure: it
required the float32 display rollout and float64 precision arrays to be exactly
equal before considering their saved encodings. The corrected loader verifies
that every saved rollout time/state/reference value equals the full-precision
recording cast to the rollout's original dtype, then renders the unmodified
full-precision recorded pose and clock. It changes no physics or metric data.

The complete frozen S reference determines the planned trace and camera extent
even if a run ends early. Only its marker advances after a truncated recording;
the actual last pose stays frozen and visibly marked. Before reading any NPZ,
the renderer verifies the root output seal's protocol SHA, all nine process
identities/return flags, every sealed file hash/size, and the executed route's
source binding.

`contracts_stage01` preserves the initial three synthetic orientation/indexing
contracts. The final four contracts in `contracts_stage02` include those three;
they are four unique graphics checks, separate from scientific acceptance.
