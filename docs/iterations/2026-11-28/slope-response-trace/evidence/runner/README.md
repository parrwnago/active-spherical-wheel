# I-81 unchanged native replay instrumentation

This result-owned harness reuses the original I-76 `eight_m100_N` initialization,
controller, 5 ms feedback, gains, limits, geometry and Gazebo/DART physics. It
executes the complete original prefix from reset and stops at 102.0 s. The
original nominal route is 133 s; this is not full-route completion or a controller
acceptance trial. No saved-state reset approximates hidden solver history.

The only physical stepping change is `run(5)` to five `run(1)` calls during
[100.50,101.80) s, using the unchanged I-55 `SubstepTrace`. All native raw contact
records and bridge states are copied after each 1 ms solve. These exported force
samples are not an exact solver-impulse oracle. Exactly 1,300 substeps over260
completed holds are required. The previous actual command stays held throughout
each five-step interval.

One wrapper calls original native command exactly once, then exports existing
controller values at every 5 ms sample in [100.50,101.80] s. It never recomputes
preview, observer, support location or model kinematics. Native preview is counted
once per command. Exact byte-level state/history/output guards bracket export.
The original I-76 outer truth/terrain guards remain active during this capture.

`preview_rows.json` contains261 independent JSON rows. Each has:

- `time_s`, original precision `source_row_index`, current `qpos`/`qvel`;
- previous actual input, newly returned applied input and unclipped request;
- original path position, velocity and acceleration inputs;
- full `sync` commands/weights/targets, including each selected future horizon,
  patch exposure/margins/gap, target speed/torque and predicted tangential velocity;
- current estimated `plane`, synchronizer plane, full returned native `info`;
- observer filtered generalized wrench, interval mean and final estimator normal.

Optional nonfinite diagnostics become JSON null. The original NPZ fields retain
their exact bytes. Preview candidate exposure is geometric and is not relabeled
as guaranteed force onset. Focus anchors100.75–101.65s have physical outcome
coverage for horizons up to120ms within the captured window.

The newly emitted command acts after the row's state; the observer update is
about the preceding completed interval. All79 original precision arrays through
102s must match the historical recording exactly, except that the diagnostic's
final command-held flag is false. Expected20401 native command/preview calls.

The root-owned frozen protocol requires `iteration: I-81`, `runner_settings`
equal to `settings.json`, and `source_sha256`/`input_sha256` path-to-hash maps.
Other root metadata is unrestricted. Original I-76 source/input/seal bindings are
checked independently, and new canonical runner/helper sources must be bound.

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH -u LD_PRELOAD \
  OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  build/tools/bin/micromamba run -p build/gazebo-env python \
  results/slope_response_trace_20261128/runner/run_trace.py \
  --protocol results/slope_response_trace_20261128/protocol/protocol.json \
  --preflight-only
```

Replace `--preflight-only` with `--run --output NEW_DIRECTORY` only for an
authorized new physics prefix. Closed output directories are never overwritten.
The root launcher must observe the actual process return code and preserve logs.

Six focused pure instrumentation contracts pass in `contracts_stage01`.
An earlier unsealed invocation failed before collecting tests because an
unrelated auto-loaded ROS pytest plugin required unavailable `lark`; its source
and receipt remain in `contracts_stage00_setup_failure`. Disabling third-party
plugin autoload corrects test setup only. Repeated tests/stages are not added
to the six-contract count; no full-repository test suite is claimed.
