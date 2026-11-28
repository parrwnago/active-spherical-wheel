# I-75 saved-state media

Prepared without reading fresh campaign arrays. The renderer accepts only the
root-sealed six-run N/T campaign and verifies every sealed file, the executed
route/generator/protocol and the fixed heading settings before loading arrays.

The shared desired heading is the analytic geometric path tangent. Raw and
command streams are separately displayed and verified equal for this fixed C2
profile. N leaves its explicit heading helper off; T uses the planned heading
and Euler-heading-rate target. The native initial-yaw latch is retained as
provenance, not confused with the moving path-facing target.

Displays distinguish body tilt, raw-reference up error, actual Euler heading,
raw path-heading error, Euler heading rate/path rate, physical world-z angular
velocity and helper motor-torque norm. In particular, Euler heading rate is
not relabeled as world-z rotation. Seven focused synthetic media functions pass:
five retained I-74 functions and two new moving-target/rate-frame functions.
The counts are inclusive and not a full repository suite.

Only saved full-precision poses are rendered. Native float32 rollout encodings
must exactly match precision arrays cast to their original dtype. One fixed
world-up camera covers all six actual paths and the complete desired path.
The full 0–45 s window includes both holds; prematurely ended poses are visibly
frozen and flagged. The reference marker continues along the fixed full path.

After root release, first register and render the fixed previews:

```bash
.venv/bin/python results/tangent_heading_control_20261127/publication/video_tools/render.py --phase m100 --stage results/tangent_heading_control_20261127/publication/video_preview_stage01/m100 --input-seal results/tangent_heading_control_20261127/campaign/outputs_sealed.json --preview
```

After preview approval, omit `--preview` and use a new output stage under
`publication/videos_stage01/`. Each video is 1600×960, 20 fps, 901 inclusive
endpoint frames covering saved time0–45 s (45.05 s encoded). Decode and review
first/middle/final frames for all three phases. The media never integrates
physics or reruns the controller. Exact tested I-74 ancestors are retained in
`source_i74/` as historical source snapshots, not scripts to run in place.
