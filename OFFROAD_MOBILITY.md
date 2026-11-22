# Off-road mobility evaluation

This experiment measures the **unchanged current contact-aware controller**, paired with the flat-ground baseline, on identical terrain and starting states. Run from `feature/offroad-mobility-evaluation`. It adds trajectories and measurement, not new balancing authority.

## Questions and scenarios

| Case | What it tests |
|---|---|
| flat_circle | Full 0.45 m radius circle on level ground: control experiment |
| hills_circle_ccw / hills_circle_cw | Same circle in both directions over two-dimensional hills |
| rough_figure8 | Changing curvature, diagonal travel, crossing the centre and reversing turn direction |
| rough_side_to_side | Five lateral legs, repeated reversals, returning to the start |
| bank_circle | A circle over a 6 degree bank plus hills: uphill, downhill and across slope in one run |
| bumps_square | Stop-and-go corners over mounds and hollows; every corner has zero reference speed and acceleration |
| slick_figure8 | Changing direction while crossing a low-friction strip (mu 0.22) on hills |
| hills_eight_directions | Eight compass directions, each followed by a return to the centre |
| pushed_circle | Hills circle with perpendicular 0.5 N, 0.1 s pushes at 35% and 65% of the trajectory |

The ground spans the whole two-dimensional arena; no flat entry corridor masks lateral tests. The square has stops at corners, rather than instantaneous turns. Circles and figure-eights accelerate and decelerate smoothly, with analytic velocity and acceleration. Every run has one second before motion and three seconds after motion. Starts on slopes use the same initially balanced state for both controllers. The current ground-normal estimator is initialized from the simulator's terrain normal, as in the existing runner.

## Run the collection

The full sequential collection is available as `powershell -NoProfile -ExecutionPolicy Bypass -File scripts/collect-offroad-mobility.ps1 -Workers 3`. It runs the showcase, robustness, speed and half-step stages: **346 simulations and 173 paired MP4s**. Stage status and stdout/stderr logs are in `results/offroad/mobility_collection/`. It stops on infrastructure errors; an ordinary controller failure is a recorded result and does not stop collection. Keep the computer awake until collection finishes.

PowerShell from the repository root:

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
.\.venv\Scripts\python.exe -m spherical_wheel.mobility --suite showcase --workers 3 --output results/offroad/mobility_showcase
```

This runs 10 cases x 2 controllers = **20 simulations**, and writes **10 side-by-side MP4s** at real-time playback speed. Both controllers remain visible, including early failures. Cyan is the actual trail; orange is the entire planned route and moving reference point. The fixed camera preserves direction and scale. Each pair also contains per-controller `rollout.npz`, `summary.json`, and `plots.png`. Maps use equal x/y scale. `REPORT.md`, `results.json`, and `progress.json` update as pairs finish.

For repeatability and failure discovery:

```powershell
# 10 cases x 3 seeds x 4 wheel starting phases x 2 controllers = 240 simulations.
# Every pair gets a replay, including failures.
.\.venv\Scripts\python.exe -m spherical_wheel.mobility --suite robustness --workers 3 --output results/offroad/mobility_robustness

# 0.5x, 1x, 1.5x, 2x nominal speed; acceleration scales with speed squared.
# 10 cases x 4 speeds x 2 controllers = 80 simulations.
.\.venv\Scripts\python.exe -m spherical_wheel.mobility --suite speed --workers 3 --output results/offroad/mobility_speed

# Numerical sensitivity: repeat key cases at half the physics timestep.
.\.venv\Scripts\python.exe -m spherical_wheel.mobility --suite showcase --cases hills_circle_ccw rough_side_to_side rough_figure8 --timestep 0.0005 --workers 3 --output results/offroad/mobility_half_step
```

Use `--cases` to restrict a run. `--replays representative` renders the first seed/phase plus every pair where the current controller misses acceptance; `--replays none` skips rendering. Default `all` renders every pair. Full robustness is 120 videos and may take hours. All artifacts remain local; nothing is uploaded.

Reissuing an identical command resumes completed pairs and saved simulations. A manifest records source hashes, model parameters, seeds, phases and thresholds. A changed source/configuration requires a fresh output directory, preventing accidental mixing of experiments. Infrastructure errors are recorded separately from controller failures and make the command exit unsuccessfully. `finished` in progress means collection ended; check its `errors` and the report before interpreting results.

## Acceptance criteria fixed before running

These are proposed engineering targets for this 0.24 m diameter wheel, not certified limits:

- Complete the requested duration without falling or diverging more than 1 m from the reference.
- Moving RMS position error <= 50 mm, 95th percentile <= 100 mm, maximum <= 200 mm.
- Final target error and actual start-to-end closure <= 50 mm.
- Maximum attitude error relative to the balancing reference <= 8 degrees.
- Motor command saturation in <= 10% of control intervals.
- Settle during the last second: attitude error < 1 degree, horizontal speed < 0.03 m/s, angular speed < 0.08 rad/s, using the existing runner's definition.
- Circles must actually traverse 0.95–1.05 laps in the requested direction. Returning to the start without moving cannot pass.

All gates must pass. Completion alone is not a tracking pass. Failed early runs retain their errors and cannot pass by having a short, quiet log. Report tracking separately in eight world-direction bins to reveal weak directions. Also inspect slip distance, contact-loss time, impact load, shell/belt load shares and handovers, torque, joint speed and work. Work measures only the three rotary drives; it is not total electrical energy.

Heading excursion and staying within 10 degrees of the initial heading are reported separately. The current controller does **not** hold yaw by default. Traversing a circle is a translation task; spinning the platform through 360 degrees or translating with fixed orientation is a different task. This evaluation must not imply those capabilities from a successful path.

## How to make a convincing claim

1. Show both clockwise and counterclockwise circles, lateral reversals and the figure-eight in full-length videos, with matching reference/actual paths and numeric errors.
2. Publish every result, including failed starts, baseline comparisons and low-friction failures. Use all seeds/phases, not the best replay. Quote completion and acceptance counts separately, and median and worst errors.
3. Use the speed sweep to find the tested operating envelope. A failure at higher speed is useful evidence of the boundary. Run additional seeds and phases near the boundary before calling it reliable.
4. Repeat key cases at half the physics timestep. Differences at shell/rim transitions reveal numerical sensitivity; do not hide them by picking the prettier replay.
5. Keep terrain within the contact model. Each arena is audited on a 101x101 grid; all sampled concave radii must exceed 1.15 wheel radii. This is a numerical audit, not an analytic proof of every point. Sharp rocks, steps, narrow ruts, loose soil and deformable mud need different collision/soil models.
6. Before claiming hardware performance, reproduce the motion with measured masses, motor torque-speed limits, belt dynamics, real state estimation, noisy/delayed contact sensing and physical terrain tests. Add payload/COM, friction and sensor-delay sweeps as subsequent model extensions; the present suite does not simulate those effects.

The defensible result is: **the declared simulated wheel followed these multidirectional paths on these smooth terrains at these speeds, with these errors and failure rates**. Finite simulation trials cannot prove unrestricted off-road motion.

## Original local LQR comparison

Run `.\.venv\Scripts\python.exe scripts/compare-local-lqr-offroad.py` to compare the original interactive lab's `local_lqr` mode with `terrain_aware` on hills circles in both directions, the rough-ground figure-eight, and the banked circle. Results and four paired MP4s go to `results/offroad/local_lqr_comparison/`; `REPORT.md` updates after each completed pair.

This runner retains the original local LQR command law, acceleration reference, raw full-state feedback, shell/belt nearest-phase gain selection, rolling feedforward, and actuator clipping. The lab's x-travel gain-grid planner is called for both x extrema to cover closed paths. It adds no off-road lean/holding torque, heading-frame correction, joint-error wrapping or incoming-drive synchronization. Force sensing is present for logging only. Both controllers start in the same initially balanced state on the terrain. Four tests in `tests/test_local_lqr_comparison.py` verify the adapter's commands and references against the original `SimulationLab._advance_one` on flat and sloped ground.

An early failure's RMS error covers only its recorded prefix, and is not an equal-duration measure against a completed run. The video freezes that failed controller at its final state and marks it stopped while the other controller continues. These are one-seed, one-start comparisons, not a statistical robustness study of local LQR.

## Validation commands

```powershell
$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'
.\.venv\Scripts\python.exe -m pytest tests/test_mobility.py tests/test_offroad_ground.py tests/test_offroad_control.py -q
```

Trajectory tests check closed endpoints, exact derivatives against finite differences, continuity at joins and both circle directions. Terrain tests check all suite seeds. A stationary-wheel regression checks that ending at the requested start point cannot masquerade as completing a circle. Existing off-road tests cover contact mechanics and controller behavior.
