# Polar tracking with a lateral disturbance

This is the saved output of `test_polar_tracking_crosses_from_belt_to_shell_and_recovers_push` in `tests/test_lab.py`. The saved run records the earlier 0.2.0 `SimulationLab` implementation. The current regression is also run against the revised browser controller; these historical numbers are retained unchanged.

Starting at polar shaft phase π/2, it follows a 40 mm forward curve with a 20 mm lateral excursion over 10 s. At 2 s, the fork receives a world lateral force of 0.5 N for 0.1 s. The pulse occupies exactly 20 controller intervals and supplies 0.05 N s of impulse.

The run completed without falling. Final absolute tilt was 0.0189609°; final planar tracking error was 4.39376 mm. The support moved from polar belt A to shell A as the shaft phase increased to 1.9217034 rad. Both sides of this cap belong to the same positive-x hemisphere, so this path does not contact shell B.

`polar_tracking_check.json` records the measured outcome and 101 actual contact/gain/shaft-phase observations. The test also checked a free root joint, zero equality constraints, zero applied generalized forces, and no world forces except the specified finite lateral push on the fork. At the end, the external push force was zero.

`rollout.npz` and `run.csv` preserve all sampled states, applied inputs, requested controller commands, reference coordinates, and forces. The NPZ model path is relative to this directory; the complete generated model is in `model/`. `configuration.yaml`, `events.json`, and `summary.json` preserve the experiment settings.

To rerun the regression from the repository root:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q -s tests/test_lab.py -k polar_tracking
```

The test prints the path of its newly saved portable artifacts. Physical properties remain provisional and the belts use ideal speed servos; this demonstrates one local trajectory and disturbance case.
