# I-76 full-route saved-state comparisons

Nine comparisons cover every straight/arc/eight route and all three starting
phases. Each shows native baseline N beside unchanged tangent-heading helper T.
The planned0–45s straight/arc and0–133s eight windows include both planned holds.
At10fps these contain451 and1331 inclusive endpoint frames. One static camera
covers the union of all18 recorded paths and all three complete references.
Straight/arc passed the moderate geometric screen. Eight is an explicitly
declared rough stress test that failed that screen; its header/metadata retain
this distinction. The fixed control acceptance gates remain unchanged.

The renderer verifies the root output seal, executed route/reference hashes,
run manifests, raw/command target equality, actual initial alignment and exact
Euler heading-rate telemetry before displaying full-precision saved poses. It
does not execute physics or control. Prematurely ended poses are frozen and
marked while the planned reference continues. Raw path heading, bounded command,
actual yaw, raw error, Euler heading rate and physical world-z rate remain
distinct. Completed episodes' terminal frames explicitly mark logged helper output UNHELD.
Each video's metadata separately records actual available episode windows and
completion. Frames after an early endpoint are explicitly unavailable simulation,
with only the last recorded pose/telemetry frozen, a red marker and capped pose
time. They do not show continued actual simulation or actual goal holding.

I-75 ancestors are retained byte-exact in source_i75/. Its two post-encode
random-seek failures informed the new sequential complete-stream decoder with
one FFmpeg input/output thread from the outset. No random seeking is used.

After root releases all18 sealed outputs, first render fixed previews:

```bash
PYTHONPATH=src .venv/bin/python results/tangent_heading_routes_20261127/publication/video_tools/render.py --route straight --phase m100 --stage results/tangent_heading_routes_20261127/publication/video_preview_stage01/straight/m100 --input-seal results/tangent_heading_routes_20261127/campaign/outputs_sealed.json --preview
```

After preview approval, omit --preview and use a new stage under
publication/videos_stage01/{route}/{phase}. All27 first/middle/final decoded
views require QA. Nine focused media functions comprise seven retained I-75
functions, a moving-tangent crossing-of-pi contract and an early-end clock/hold contract; this is
not the full repository suite.
