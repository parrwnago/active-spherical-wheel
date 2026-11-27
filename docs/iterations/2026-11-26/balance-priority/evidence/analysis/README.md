# I-58 saved-array analysis

`reduce.py` reads sealed I-58 outputs and applies the unchanged root-owned `metrics.py` plus immutable I-56/I-49/I-41 pure metric functions. It calls no controller, estimator, dynamics model or physics. Missing future jobs and failed/partial experiments remain explicit. All acceptance choices come from the frozen protocol.

`source_stage01/registered_before_outcomes.json` preserves the exact analysis sources, protocol and legacy metric dependencies before this agent read any new I-58 outcome. New physics had already been started independently by the root agent under its frozen protocol. The eight unit checks and the separate synthetic end-to-end fixture validate metric behavior; they are not physical experiment outcomes.

Run a new, non-overwriting reduction stage:

```sh
.venv/bin/python results/balance_priority_20261126/analysis/reduce.py \
  --output results/balance_priority_20261126/analysis/NEW_STAGE
```

Each numeric stage writes `registered_before_arithmetic.json`, source snapshots, all input hashes, `report.json`, `cases.csv`, `turns.csv`, derived signals and geometry. Initial P versus B includes mandatory reproduced A and the predeclared priority-restoration benefit condition. Later phases compare P versus their matching B. Missing later phases cannot be described as passing. Completing 44 s is distinct from passing every registered condition.

The priority quantity is the **sum** of first-stage and secondary transverse-response differences: `G20[3:5] @ (final_applied - clipped_native_total)`. Testing only secondary equality would miss the balance authority lost in the original hard rate box. Priority maximum covers every emitted command; priority RMS and command-continuity exposure exclude the terminal unheld emission. Continuity includes the first command relative to the robot's actual previous applied input. It is a secondary diagnostic, not a selectable benefit gate. Old 60/6 rate thresholds are hard only for A; P and B comparisons against them are diagnostics. Input amplitude caps remain hard.

The legacy moving-XY mask is exactly `[1,41]` on saved floating-point timestamps. Attitude peak/RMS includes every measured state from 0 through the observed endpoint and uses the raw native observer-derived target. Absolute tilt from gravity is reported separately. World-Z angular rate uses the endpoint quaternion to transform local angular qvel. Acceleration values are 5/50 ms interval-average backward velocity differences, not instantaneous solver accelerations. All six corrected fixed reference turns are retained; old event outcomes and retry histories are not imported.

`portable_inputs/` holds only required precision fields, at **every original 5 ms timestamp**, together with the full reference program, event definitions, real unmodified provenance/summary/case identity and compressed full governor diagnostic histories. The projection is checked byte-for-byte against its original precision fields. `portable_inputs.json` binds each selected-field projection to the original full precision SHA256 and individual projected-file hashes. Full original captures remain local in the campaign directories.

For portable reproduction, keep these together:

- `portable_inputs.json` and `portable_inputs/`
- `source_snapshot/`
- `report.json` and `artifact_manifest.json`

With NumPy installed, replay from any filesystem location:

```sh
python PATH/TO/replay.py --data-source PATH/TO/STAGE --output NEW_REPLAY.json
```

Replay verifies sealed hashes and independently recomputes every case metric and gate from the projected inputs and copied pure source. It does not need the original repository, prior result folders, controller packages or simulator. The metric adapter copies only a native/governor label and metric settings for dispatch into immutable I-56 math; portable metadata keeps the actual I-58 variant, settings and provenance unchanged.

Scientific PNG/PDF plots are generated separately from the portable stage:

```sh
.venv/bin/python results/balance_priority_20261126/analysis/plot.py \
  --data-source results/balance_priority_20261126/analysis/STAGE \
  --output results/balance_priority_20261126/analysis/NEW_PLOT_STAGE
```

Plots connect all saved points. PDF lines are rasterized solely to keep artifact size practical. Each available B/P phase gets a comparison page and a separate motion/command-bound diagnostic page. A is included at the initial phase. Plot rendering never changes metric values or acceptance decisions.
