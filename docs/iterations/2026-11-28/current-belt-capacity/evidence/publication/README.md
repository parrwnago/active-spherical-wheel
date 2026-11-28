# I-82 recorded-state capacity comparison media

The two panels show the same native LQR, observer, feedforward and preparation
with operating belt-speed limits 0.30 and 0.60 m/s. Model configuration, ordinary
design and gain bank remain the original 0.30 design. These movies do not run a
controller, observer or simulation: Gazebo/DART supplies recorded exact poses,
and MuJoCo is used only for visuals.

The full movie covers 0–133 s at 1×, 10 fps, including the final 10 s goal hold. A
second movie contains the exact same frames over 98–104 s, not a new simulation,
time rescaling or separately favorable camera. One static world-up camera covers
both full actual paths and the planned path. Ended runs freeze visibly at their
actual endpoint, and no final unheld command is shown as applied motion.

`render_stage02.py` is the final renderer. The first `render.py` and four-contract
stage are retained. Before reading arrays or rendering, source review found
the inherited I76 sequential decoder assumed a 1600×960 canvas while the new
captions use 1600×1010. Separately named stage02 supplies the dimension-aware
decoder; five pure presentation contracts pass, including the earlier four.
There was no physics or numerical experiment change from this correction.

The final output seal and actual process-status records are required before
any case array is loaded. Renderer source, inputs, models, terrain assets and
captured configuration are hashed before array reading. It verifies both raw
robot subtrees against the display model and the float32 rollout against exact
recorded poses. Physical stepping functions are prohibited during rendering.
Every encoded frame is decoded sequentially after writing and hashes rechecked.

CLI after the cases are closed:

```sh
MUJOCO_GL=egl .venv/bin/python results/current_belt_capacity_20261128/publication/render_stage02.py \
  --base results/current_belt_capacity_20261128 \
  --protocol results/current_belt_capacity_20261128/protocol/frozen_stage01/protocol.json \
  --input-seal results/current_belt_capacity_20261128/campaign/outputs_sealed.json \
  --case030 results/current_belt_capacity_20261128/campaign/runs/B030 \
  --case060 results/current_belt_capacity_20261128/campaign/runs/B060 \
  --stage results/current_belt_capacity_20261128/publication/new_video_stage
```

Add `--preview` and use another empty stage to inspect representative frames
before a full encode. The final movies are large local outputs linked from the
iteration report; compact source, metadata and selected frames are preserved.

The completed visual ZIP places the full movie and 98–104 s clip under `video/`,
and the comparison PDF, full-route and downhill plots under `evaluation/`.
Independent pair, substep and scalar/gate audits are under `verification/`.
`manifest.json` records the original source path, exact bytes and SHA-256 for
every bundled file. The ZIP is checked by CRC and each member's SHA-256.

This single matched test passed the fixed capacity screen: moving timed RMS
fell from 46.36 to 23.36 mm, and peak timed error from 336.41 to 72.17 mm.
Both full 133 s runs completed. The 0.60 m/s case ended 0.88 mm from the goal;
it still failed the four-condition settling test because of slow yaw rotation.
The result does not change controller defaults or establish general performance.
The first changed actual command was at 80.880 s, so the later downhill episodes
have different motion histories. The force summary is descriptive for those
two histories and does not isolate a same-state braking intervention.
