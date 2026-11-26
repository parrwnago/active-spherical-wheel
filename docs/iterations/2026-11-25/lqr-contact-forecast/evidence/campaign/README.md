# I-40 campaign: unchanged native LQR and private fine-feedback forecasts

Eight fresh Gazebo/DART cases completed at their requested durations, with no
physical failure. There were exactly eight Gazebo executions: the first
`flat_hold` case and a second launch batch containing the remaining seven.
No successful run or forecast was repeated.

The actual controller is unchanged hardware-mode LQR/feedforward/synchronization
with least-squares rolling allocation, `track_contact_point=False`, and 5 ms
feedback. Each case preserves its exact launch source snapshot, provenance,
summary, terrain, full-precision measured-state/input history and evaluator-only
contact points/loads. Contact truth is read only by the evaluator after guarded
control. Forecasts are generated afterward from each fresh case's own native
history and pure planned reference; the actual future is not provided to them.

All 113 forecasts succeeded: 96 uniform anchors and 17 separately labeled
truth-conditioned evaluation anchors. Every reconstructed history matches its
recorded native commands, observer wrench, estimated normal, nominal joints and
reference exactly (reported maximum gap zero). The forecast uses the same 5 ms
native feedback in a private fitted-plane model for 200 ms, retaining all fine
commands and states. No 20 ms averaging occurs.

`simulate_all_summary.json` is the eight-case actual-run overview.
`forecast_all_summary.json` is the eight-case forecast overview.
`simulate_summary.json` and `forecast_summary.json` record the second, seven-case
launch batches; the first hold-case logs and immutable per-case files remain.
`artifact_manifest.json` seals all per-case actual/forecast artifacts and source
snapshots. The 864 source snapshot references and generated input/output hashes
were verified at campaign completion. These are evidence checks, not unit tests.

The nominal `flat_y_transfer` case did not produce a stable shell/belt-family
event. That coverage outcome is retained, rather than relabeling motion as a
demonstrated transfer. The declared first stable events in `slope_y`,
`slope_x_late`, and `mars_figure8_p090` do provide isolated family transitions
under the declared ±100 ms criterion. The late X event has only one usable
pre-event anchor: its −50 ms anchor would leave insufficient actual future for
the complete 200 ms evaluation. No later event was substituted to improve the
candidate's isolation.

`old_slope_x_repeatability.json` distinguishes the nominal historical regression
from an exact reproduction. The new slope mesh spans ±1.0 m (51×51 vertices),
whereas the old I-32 mesh spans ±0.9 m (46×46), both at about 0.04 m. Initial
state differs at roundoff and later motion differs. This does not establish the
mesh as the sole cause. Forecasts are evaluated against their own fresh actual
traces; no historical prefix identity is claimed.

The sole runner preflight failure was a Python syntax error before any Gazebo
or forecast calculation. Its exact unexecuted source/status is retained under
`runner_development/stage01_unexecuted/`. The corrected runner was compiled and
CLI/path endpoint checks passed before the first actual run. Core-module test
stages are documented separately by their owner.

Reproduction uses a fresh output family with the declared `protocol.json` and
`campaign/jobs.json`, and the same numerical packages as the actual Gazebo run:

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH MAMBA_ROOT_PREFIX="$PWD/build/mamba" build/tools/bin/micromamba run -p "$PWD/build/gazebo-env" env MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -u scripts/run-lqr-contact-forecast.py --output results/new_lqr_forecast --stage simulate --workers 3
env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH MAMBA_ROOT_PREFIX="$PWD/build/mamba" build/tools/bin/micromamba run -p "$PWD/build/gazebo-env" env MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -u scripts/run-lqr-contact-forecast.py --output results/new_lqr_forecast --stage forecast --workers 3
```

The runner refuses to overwrite existing case/forecast stages. Forecast
calibration uses estimated support only, with the fixed 3 mm/20-iteration fit;
bad-fit diagnostics remain available and no truth-driven repair or tuning is
applied. Completion is separate from tracking or forecast-accuracy acceptance.
