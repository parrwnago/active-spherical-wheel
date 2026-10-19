# Matched 81-case yaw regression comparison

This compares transition-aware feedback at frozen commit `3bfd7998ed72146521894f445c2907129ed9802c` with the shell-material-angle correction. Both modes retain incoming-drive synchronization. The candidate zeros only the two shell material-position errors; shaft phase and every velocity-feedback term remain active. It adds no heading constraint, external torque, damping force, or physical-parameter change.

[Full paired metrics](comparison.json) · [comparison plot](comparison.png) · [benchmark](reproduce.py) · [summary generator](summarize.py)

Each mode runs the same 81 cases for 10 simulation seconds: shell, polar, and transition starts; 0.5, 1, and 2 N per push; same-Y, alternating-Y, and cyclic +X/+Y/−X/−Y patterns; and three pulse-duration/spacing profiles. There are eight scheduled pulses per run. The transition scenario retains its original 60 mm forward / 5 mm lateral reference curve over 10 s.

| Profile | Pulse duration | Spacing | Last pulse ends |
| --- | --- | --- | --- |
| Light | 0.10 s | 0.40 s | 3.90 s |
| Medium | 0.20 s | 0.25 s | 2.95 s |
| Overlap | 0.30 s | 0.15 s | 2.35 s |

Overlapping forces add; a 2 N same-direction overlap train reaches 4 N net force. The benchmark records the actual applied force separately from actuator commands.

## Results

Both modes survive **79 of 81** cases. There are **78 common survivors**. The yaw and position comparisons below use those common survivors, so falling endpoints do not distort the steady-response comparison.

| Metric at 10 s | Frozen baseline | Free shell material angles |
| --- | ---: | ---: |
| Median absolute world-vertical yaw rate | 0.03769 rad/s | 0.01089 rad/s |
| 90th percentile absolute yaw rate | 0.59523 rad/s | 0.16253 rad/s |
| Largest final absolute yaw rate | 4.71954 rad/s | 1.11467 rad/s |
| Cases ending above 1 rad/s yaw | 5 | 1 |
| Demonstrated yaw settling before the endpoint | 50 | 63 |
| Median final XY position error | 3.221 mm | 2.284 mm |
| 90th percentile final XY error | 86.50 mm | 47.31 mm |
| Largest final XY error | 199.22 mm | 205.16 mm |

“Yaw” is actual angular velocity projected onto the world vertical, calculated from the platform quaternion and local angular velocity. It is not camera rotation or a commanded heading. Yaw settling means the absolute rate remains below 0.1 rad/s for every remaining sample after the last scheduled push, with at least 0.5 s of observed quiet tail. A null settling value means this was not demonstrated within the 10 s run.

The equal total survival count hides a tradeoff:

- **Regression:** polar medium 2 N same-Y survives in the baseline but falls at 2.980 s with the candidate.
- **Improvement:** transition overlap 2 N same-Y falls at 2.295 s in the baseline but survives with the candidate. The candidate still reaches 33.72° peak tilt and ends at 1.06° tilt and 97.53 mm position error; this is a demanding recovery, not precise tracking.
- **Shared failure:** polar overlap 2 N same-Y falls at 2.060 s in both modes.

Residual yaw is not eliminated in every strong case. The largest candidate final rate is 1.11467 rad/s in shell overlap 2 N same-Y. Fourteen other common survivors still exceed 0.1 rad/s at the endpoint. These exceptions and the largest XY regressions remain in the paired JSON rather than being removed from the comparison.

## Physical and protocol checks

The six generated physical configurations match. All 81 paired initial states match, as do the actual push histories and world-space XY reference histories up to the first fall in either run. All recorded unexpected world forces and uncommanded generalized forces are zero. The existing torque and belt-speed limits remain unchanged.

Both runs explicitly use `OPENBLAS_NUM_THREADS=1` and `OMP_NUM_THREADS=1`, also enforced by the reproduction script before NumPy imports. Source hashes captured at worker startup and thread caps appear in each run's metadata. Earlier partial uncapped runs were discarded and are not part of these results.

## Reproduce

Run from the repository directory with the supplied Python environment. Use a checkout of baseline commit `3bfd799` for the baseline package, without changing this checkout's candidate source:

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=/path/to/baseline/src \
  .venv/bin/python results/yaw_control/stress/reproduce.py \
  --output results/yaw_control/stress/baseline_3bfd799 \
  --controller-mode transition_aware --duration 10 --workers 3

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=src \
  .venv/bin/python results/yaw_control/stress/reproduce.py \
  --output results/yaw_control/stress/phasefree \
  --controller-mode transition_aware --duration 10 --workers 3

OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  .venv/bin/python results/yaw_control/stress/summarize.py
```

Each mode's directory contains generated models, invocation metadata, per-case JSON metrics, and compressed state/input/force trajectories. The baseline commit identifies the frozen implementation; source hashes in the recorded runs provide an additional check.

These are sampled 10 s simulations of the declared approximate model. Longer post-push observations are recorded separately under `../longtail/`. No global recovery guarantee, zero position bias, absolute-heading controller, or hardware capability is established by this comparison.
