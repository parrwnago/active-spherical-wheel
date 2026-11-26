# I35 frozen horizon comparison

Six private MuJoCo forecasts completed from the saved I32 failed-MPC states at 0.4 s and 2 s. **Neither longer horizon passed the protocol, so no new Gazebo trial was run.** This is an offline model experiment, not a video or a closed-loop recovery result.

The actual `TrajectoryPredictiveExperiment` runtime used full-stage Jacobians, a present-state native policy rollout seed, horizons 0.4/0.8/1.2 s (20/40/60 stages), and the same five-refinement budget. The 20 ms stage period, weights, terminal multiplier 4, limits, model, observer, contact locator and solver tolerances stayed fixed. At 2 s the preceding old 20-stage sequence was extended by repeating its last bounded input before the normal one-stage runtime shift. Native rollout candidates covered each complete horizon independently.

| Frozen anchor | Horizon | Final goal distance | Peak tilt at 1 ms | Runtime wall time | Gate outcome |
| --- | --- | --- | --- | --- | --- |
| 0.4 s, initial 8.12 mm | 0.4 s | 13.95 mm | 14.14° | 22.92 s | Early comparison passes |
| 0.4 s | 0.8 s | 43.95 mm | 14.38° | 47.59 s | Fails 40 mm peak-distance rule |
| 0.4 s | 1.2 s | 81.03 mm | 83.58° | 64.73 s | Tips, loses support, loads nonwheel geoms |
| 2 s, initial 44.90 mm | 0.4 s | 58.60 mm | 9.90° | 14.34 s | Continues drifting away |
| 2 s | 0.8 s | 71.86 mm | 12.13° | 28.20 s | Continues drifting away |
| 2 s | 1.2 s | 107.23 mm | 98.98° | 37.08 s | Tips, loses support, loads nonwheel geoms |

The 0.8 s late plan ends moving radially away at 301.51 mm/s. All 42 QP calls returned solved and passed the existing normalized KKT test, but this does not establish convergence of the nonlinear optimization. Long horizons showed extreme Hessian scales, and the native fine 5 ms rollout versus averaged 20 ms coarse rollout diverged by 77–103 mm in terminal XY position. No objective scaling or PSD repair was applied. The 1.2 s early case retained the original native candidate without nonlinear improvement.

Shared current q/v, target first 20 stages, native fine first 80 commands and coarse first 20 commands were bit-identical across horizons. Every selected contact replay, sampled at each 1 ms physics step, reproduced the stored 20 ms state endpoints bit-for-bit. Contacts and support loads refer to the private estimated-plane model, using the simulator's last-step force convention. The unchanged model mass is 1.86 kg; the 5% weight threshold is 0.91233 N.

Raw objective sums are not ranked across horizons. Within each horizon, selected cost is compared to the independently rescored native candidate. Per-refinement exact nonlinear costs and QP diagnostics were retained; intermediate accepted motion/contact trajectories were not saved. Final selected and same-horizon native trajectories are available.

Files:

- `report.json`: all six scalar rows, explicit gates, prefix checks, nonlinear cost progress, fine/coarse terminal divergence, limitations.
- `horizon_trajectories.png`: selected and same-horizon native coarse goal distance, world-up tilt and XY speed, sampled at 1 ms.
- `nonlinear_cost_progress.png`: relative improvement from each horizon's own best initial candidate.
- `metrics.csv`, `gate_summary.csv`, `solver_conditioning.csv`: concise tables and all 42 QP conditioning records.
- `worker_04/`, `worker_2/`: original plans, trajectories, exact trial source snapshots, provenance and reports. Numerical results have not been replaced by later trials.
- `pre_1ms_instrumentation/`: declared script source before adding the 1 ms contact replay; no numerical calls ran with this source.
- `unrun_trace_instrumentation/`: later proposed per-candidate motion instrumentation, never executed. The repo script was restored to the exact tested worker snapshot.
- `postprocess_source/`: exact merge/gate/plot code captured before read-only postprocessing.

Original numerical commands (do not overwrite existing evidence; change the output paths for reproduction):

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -u scripts/diagnose-mpc-horizon.py --output results/mpc_horizon_20261125/frozen/worker_04 --anchors .4
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 .venv/bin/python -u scripts/diagnose-mpc-horizon.py --output results/mpc_horizon_20261125/frozen/worker_2 --anchors 2
```

These workers ran concurrently, so their wall times include shared CPU contention. The frozen results show that this implementation and budget did not produce a convincing corrective return plan; they do not prove that longer lookahead cannot help or that recovery is physically impossible.
