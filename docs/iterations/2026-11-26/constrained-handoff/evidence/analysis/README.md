# I-56 saved-data analysis

`reduce_handoff.py` reads the original sealed I-56 B/A/H artifacts and registered six reference-turn definitions. It calls no controller, observer, MuJoCo model, or physics. It writes a new stage and refuses to overwrite one.

```sh
.venv/bin/python results/constrained_handoff_20261126/analysis/reduce_handoff.py \
  --output results/constrained_handoff_20261126/analysis/NEW_STAGE
```

The primary report is `stage01/report.json`, with compact `cases.csv` and `turns.csv`. Its `registered_before_arithmetic.json` binds source and input hashes before this numeric reduction. It is an analysis registration, not a pre-physics protocol. The physics and acceptance rules were fixed separately in the I-56 protocol. The reducer was prepared before the root agent relayed an initial outcome summary; it was frozen and checked before its first saved-array reduction. The separate `checks_stage01/report.json` records synthetic validation and a corrected pytest plugin setup error.

Raw-reference lean measures the angle between actual body up and the native observer-derived body-up target; it is distinct from absolute tilt relative to gravity. Raw-reference peak/RMS includes all states from 0 through 44 s, matching the previous I-52 convention. XY moving RMS preserves the old exact `[1,41]` mask; because of accumulated floating point time, this trace contains 8000 samples in that legacy mask. Motion windows use a declared 1 ns endpoint tolerance, including the full 28–44 s late-yaw interval. All timestamps are preserved. Missing later phase runs remain explicitly unavailable.

Hard command and slew checks include all emitted requests, including the initial emission against the actual previous input and the final unheld emission. Time exposure excludes the terminal emission. Native infeasible demand is logged separately from final governed requests and active optimization constraints. Hard command bounds do not imply bounded measured acceleration or successful balance. Five and fifty millisecond acceleration figures are backward differences of measured endpoint velocities, not instantaneous contact-solver accelerations. World-Z angular rate is transformed from measured body angular velocity; heading travel uses unwrapped horizontal body-X angle and never bridges an invalid projection.

`stage01/portable_inputs/` contains lossless selected precision fields at **every original 5 ms timestamp**, the full known reference program, exact event definitions, metadata/protocols, and only the governor diagnostics needed by these metrics. `portable_inputs.json` binds each projection to its original precision SHA256 and its projected-file SHA256. Full original precision, rollout, and diagnostic artifacts remain in the local campaign run directories. Each projection has roughly 3 MB rather than 13 MB for the full precision capture. Total portable metric inputs for B/A/H are about 12 MB.

The stage is portable. Keep `portable_inputs.json`, `portable_inputs/`, `source_snapshot/`, `report.json`, and `artifact_manifest.json` together. They do not require restoration of old result folders or the runtime model. With NumPy installed:

```sh
python PATH/TO/replay_projection.py --data-source PATH/TO/STAGE \
  --output PATH/TO/NEW_REPLAY_REPORT.json
```

`replay_projection.py` verifies sealed hashes, executes only copied pure saved-array metric functions, and independently recomputes all case metrics and comparison gates. `replay_stage01.json` records exact reproduction of stage01, including unavailable later-phase cells. Python bytecode caches are not evidence artifacts and should be excluded from archives.

`plot_handoff.py` renders the requested scientific comparison from these exact portable projections:

```sh
.venv/bin/python results/constrained_handoff_20261126/analysis/plot_handoff.py \
  --data-source results/constrained_handoff_20261126/analysis/stage01 \
  --output results/constrained_handoff_20261126/analysis/NEW_PLOT_STAGE
```

The initial PNG/PDF is under `plots_stage01/`. Plot registrations and artifact manifests are kept separately from metric registration. Dense plotted lines connect every saved point and are rasterized in the PDF for size.

The registered −1.0 rad screen failed. The broader +0.3/+0.9 rad jobs were therefore not run. The bounded no-yaw/no-momentum arm is the predeclared diagnostic ablation, not an alternative candidate selected from these results. Defaults remain unchanged; no retuning or automatic adoption is implied by this analysis.
