# Active spherical wheel simulation

This is a working MuJoCo design study for the supplied active spherical wheel. It combines the native CAD appearance with a declared approximate physical model, a simple fork and upper platform, and a local feedback controller. The project runs directly in Python; ROS is not required.

The simulation is intended to answer whether the interpreted mechanism can balance and follow slow local trajectories under its stated assumptions. The result files record what each run actually achieved. They do not establish hardware performance or global stability through every shell/belt contact transition.

## Interactive browser lab on Linux or Ubuntu under WSL2

From the project directory, run:

```bash
bash scripts/run-lab.sh
```

The launcher prepares `.venv` if needed, starts the MuJoCo simulation, and opens the interface in your browser. Use Python 3.10–3.12. With an existing environment, you can also start it manually:

```bash
.venv/bin/python -m spherical_wheel lab --port 8765
```

Open **http://127.0.0.1:8765** in Chrome or Firefox on the Linux computer running that command. Keep the terminal open; press Ctrl+C to stop. No additional CAD export is needed to use the lab.

For **Ubuntu under WSL2**, run the command in the Ubuntu terminal, then open **http://localhost:8765** in your Windows browser. Windows can access the WSL server through [localhost forwarding](https://learn.microsoft.com/en-us/windows/wsl/networking#accessing-linux-networking-apps-from-windows-localhost). This was checked on Ubuntu 22.04 / WSL2 with both the default WSLg graphics backend and explicit EGL rendering; a Windows HTTP client fetched the actual state and live JPEG successfully. If the renderer reports an OpenGL error, stop the server and retry with `MUJOCO_GL=egl bash scripts/run-lab.sh`.

The interface runs live MuJoCo physics with manual force pulses, initial tilt, belt/controller switches, and forward, lateral, and curved trajectories. Start with **Polar belt**, **Roll 2°**, **Pitch 0°**, **Reset**, then **Run**. Try a **0.5 N** push for **0.10 s** using A/D, the direction buttons, or a drag in the view. Push controls remain available while a trajectory runs.

The lab now defaults to **Transition aware** feedback with **Prepare incoming drives** enabled. It follows the actual shell/belt support and current heading, and uses a short kinematic preview to prepare an incoming drive's ground-tangent surface speed before load transfer. **Original local LQR** remains available for comparison. The 0.3.0 upgrade reduced falls from 20 to 2 in 81 matched repeated-push cases. Version 0.3.1 removes shell material-angle feedback that could sustain a delayed spin, and adds heading and vertical spin readings. See [YAW_BEHAVIOR.md](YAW_BEHAVIOR.md) for the physical yaw mechanisms and measured comparisons; settling and position bias remain limitations. The physical model and actuator bounds are unchanged; no MPC is implemented. See [CONTROL_UPGRADE.md](CONTROL_UPGRADE.md) for measured results and comparison instructions.

At polar contact the driven transverse belt can move the support point beneath the upper body's center of mass. Independent ±2° roll and ±0.5 N push checks recovered with active belts; the same cases fell when belt commands were held at zero. That comparison retains belt friction and uses ideal speed servos. See the [interaction guide](INTERACTIVE_LAB.md), [polar belt evidence](results/polar_belt_check/README.md), and [browser screenshot](results/lab_ui/desktop.png).

## Uneven ground

The wheel can also be run on smooth uneven terrain: grades, side slopes, rolling ground, ripples, mounds and a low-friction strip. The ground under the wheel is the tangent plane of an analytic height field, re-posed every physics step, so the same clipped spherical contact and friction solve apply. A controller mode that reads the ground reaction senses the slope from the contact force and moment, leans so the centre of mass is over the contact point, and feeds the static holding torque forward. The flat-floor controller is kept as the baseline for every comparison. On eleven terrain scenarios run from four starts each, the contact-aware mode finished 44 of 44 runs and the flat-floor controller 38 of 44.

```bash
.venv/bin/python -m spherical_wheel offroad limits      # static slope figures of the declared model
.venv/bin/python -m spherical_wheel offroad run --scenario course
.venv/bin/python -m spherical_wheel offroad replay --scenario course   # MP4 of both controllers
.venv/bin/python -m spherical_wheel offroad suite       # every scenario, both controllers
```

On Windows use `.venv\Scripts\python.exe` in place of `.venv/bin/python`. See [OFFROAD.md](OFFROAD.md) for the contact model and its checks, what the controller does with the contact force, measured results, and where it stops working.

## Gazebo and the ERC Mars yard

The same contact-aware controller, with the same gains, also runs in Gazebo Harmonic (gz-sim 8, DART physics). A bridge plugin steps Gazebo in lockstep with the Python controller and reproduces the MuJoCo contact rules (clipped shell and belt patches, ideal belt surface speed). The setup needs no `sudo`:

```bash
conda env create -f environment/gazebo.yml && conda activate asw-gz
bash scripts/setup-gazebo.sh
python -m spherical_wheel.gazebo parity --mujoco-reference   # off-road scenarios in both simulators
python -m spherical_wheel.gazebo mars                         # routes over Leo Rover's ERC 2021 Mars yard
```

See [GAZEBO.md](GAZEBO.md) for how the port works, the checks that it matches MuJoCo, the Gazebo issues it corrects, and the Mars yard results.

## Run it

For full circles, figure-eights, lateral reversals and eight-direction motion over uneven terrain, see [OFFROAD_MOBILITY.md](OFFROAD_MOBILITY.md). The evaluation compares the unchanged controllers and saves paired MP4 replays, raw logs, acceptance gates and tracking metrics.

Use Python 3.10–3.12 from the repository directory for the pinned environment. The supplied environment was verified with Python 3.10:

```bash
bash scripts/setup.sh
.venv/bin/python -m spherical_wheel generate
.venv/bin/python -m spherical_wheel validate --output results/validation
.venv/bin/python -m spherical_wheel analyze --output results/analysis
.venv/bin/python -m spherical_wheel balance --duration 8 --output results/balance
.venv/bin/python -m spherical_wheel track --duration 10 --tilt 0 0 --output results/tracking
.venv/bin/python -m spherical_wheel balance --phase 1.5707963267948966 \
  --duration 8 --output results/polar_balance
.venv/bin/python -m spherical_wheel balance --duration 8 --tilt 0 0 \
  --disturbance 0.5 --output results/disturbance
.venv/bin/python -m spherical_wheel transition --duration 10 --output results/transition
.venv/bin/python -m spherical_wheel sensitivity --duration 5 --output results/sensitivity
```

The setup script creates a local `.venv`, installs the pinned requirements and installs this package. If `ensurepip` is unavailable, it uses the host Python's pip to bootstrap pip into the local environment without changing system packages. An already prepared `.venv` can be used directly.

Create an animation from a saved run, or open the interactive viewer:

```bash
MUJOCO_GL=egl .venv/bin/python -m spherical_wheel render \
  --log results/balance/rollout.npz --output results/balance/preview.mp4
.venv/bin/python -m spherical_wheel viewer
```

`MUJOCO_GL=egl` selects headless rendering on Linux. The interactive viewer requires a graphical display. Run `.venv/bin/python -m spherical_wheel --help` and the relevant subcommand's `--help` for the current options.

The viewer computes a fresh equilibrium and full-state local controller for the loaded model, avoiding reuse of gains from a different configuration. Its initialization was checked headlessly; an interactive desktop session was not available for GUI testing.

The current automated suite has 154 passing tests, 98 of them for the uneven-ground work. Run it with:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q
```

## Demonstrated results

The saved demonstration runs below use the earlier local feedback implementation and the declared provisional masses, friction and actuator bounds. Results for the interactive controller upgrade are tracked separately in [CONTROL_UPGRADE.md](CONTROL_UPGRADE.md).

| Case | Demonstrated outcome |
| --- | --- |
| Balance after 2° roll and 2° pitch offsets | Completed 8 s; final reference-relative tilt 0.00104° |
| 4 cm forward curve with 2 cm lateral excursion | Completed 10 s; position RMS error 1.186 mm |
| Shell-to-belt transition over a 6 cm forward curve with 5 mm lateral excursion | Completed 10 s; position RMS error 1.447 mm |

These are individual demonstrated cases, and each completed without fork/platform floor contact. They do not establish a general stability region or hardware performance. See [RESULTS.md](RESULTS.md) for the evidence, additional polar/disturbance/sensitivity checks, unsuccessful comparisons, and links to plots and animations.

## Mechanism and coordinates

At zero shaft phase, the fork-frame origin is at the spherical wheel center. The x axis is the nominal forward direction, y is the common shaft axis, and z points upward. All lengths are meters, masses kilograms, angles radians, torques newton meters, and belt speeds meters per second.

```text
world
  └─ free joint → fork + upper platform
       └─ shaft revolute about fork y → carrier + fixed polar modules
            ├─ shell_A_joint about carrier x → positive-x shell
            └─ shell_B_joint about carrier x → negative-x shell
```

The user's clarification establishes that the motor stators, internal support structure, and belt-module frames stay with the carrier. Each shell rotates independently relative to that carrier. The common shaft drive rotates the carrier relative to the balancing fork. The free root has all six degrees of freedom; no artificial upright constraint supports the platform.

The five command channels are:

| Channel | Command | Provisional bound | Reaction/contact path |
| --- | --- | --- | --- |
| `shaft` | Output joint torque | ±1.50 N m | Carrier ↔ fork |
| `shell_A` | Output joint torque | ±0.60 N m | Shell A ↔ carrier |
| `shell_B` | Output joint torque | ±0.60 N m | Shell B ↔ carrier |
| `belt_A` | Ideal surface speed along carrier y | ±0.30 m/s | Friction at positive-x polar cap |
| `belt_B` | Ideal surface speed along carrier y | ±0.30 m/s | Friction at negative-x polar cap |

The belt channels prescribe contact-surface velocity. They transfer contact forces through the carrier but omit belt rotor inertia, motor torque/current limits, load-dependent speed response, and moving belt mass. The three torque channels are equivalent transmission-output torques. Gear ratios and motor torque-speed curves are not inferred from the pictures or CAD.

The initial common-shaft phase is 0.30 rad so nominal ground contact lies away from the observed equatorial opening. This is an initial configuration, not a locked joint. Material phases and contact regions change during motion.

## CAD and collision geometry

The supplied native SolidWorks package is the source of the shell and carrier display meshes. Their extraction and coordinate provenance are retained under `assets/cad/`. The native files are kept separate from generated simulation assets.

The decoded saved CAD geometry supports these dimensions:

| Geometry | Value |
| --- | --- |
| Outer spherical shell radius | 0.120 m |
| Inner spherical shell radius | 0.110 m |
| Polar aperture radius at the outer surface | 0.032 m |
| Corresponding outer polar cap angle | Approximately 15.466° |
| Equatorial shell termination | x = ±0.010 m in the carrier frame |

Those values come from native saved geometry, rather than invented nominal dimensions. Native assembly mesh placement and the final physical belt surface still require engineering confirmation. The common-shaft azimuth follows the handoff's perpendicular-axis interpretation; the unfinished external shaft assembly is not completely defined in the native CAD.

Native CAD meshes provide appearance only. The raw generated MJCF ground collisions use separate convex angular wedges bounded by the measured inner/outer shell radii, equatorial cut, and polar aperture. Each shell has 48 wedges and 24 meridian samples by default. Their individual convex hulls preserve the openings. Finite facets approximate the spherical surface and can affect contact derivatives and contact transitions.

The default simulation mode, `contact.mode: analytic_clipped_floor`, removes material-phase facet artifacts during contact inside a spherical patch. A runtime collision filter admits an exact sphere–plane contact only when the ideal floor support point lies inside the corresponding shell or belt cap, evaluated in that body's rotating frame. At an opening or outside a cap, the analytical contact is excluded and the finite rim/edge geometry remains active. For example, the equatorial gap admits no smooth shell contact, and a polar support point admits its belt cap rather than a shell. This represents a clipped analytical surface; an overlapping full sphere is never enabled across an opening. Set `contact.mode: faceted` to run the finite-mesh comparison.

The contact filter supports the declared fixed, horizontal floor only. It is installed after MuJoCo model compilation. Reserved incompatible collision masks keep its proxies inactive in a raw MJCF load while retaining them in MuJoCo's compiled collision tree. Regenerate the raw MJCF from the configuration when exporting: saving an already activated runtime model does not serialize its Python collision callback. Keep the selected contact mode with analysis/run results when comparing outcomes. Use the finite geometry for more general collision environments, and validate any new analytical boundary handling separately.

`shell_A`, `shell_B`, `belt_A`, and `belt_B` remain distinct contact contributors. The belt contact shapes are ideal spherical caps that fill the exposed polar openings. Their complete moving-belt route and measured protrusion are not available. The native internal rollers and frames are visual geometry; they do not silently replace those declared contact caps.

Internal geometry does not collide with other internal geometry. Shells, belts, fork and platform do collide with the floor. Fork/platform floor contact is recorded as a balance failure. The analytical collision wedges are in visual group 3 so normal renders show the native CAD shells; enable that group to inspect the collision approximation.

## Added fork and mass properties

The provisional fork uses two 22 × 18 × 185 mm arms centered at y = ±145 mm, joined by a bridge above the wheel. The arms clear the 120 mm spherical envelope. A 180 × 240 × 20 mm platform is centered 200 mm above the wheel center. These dimensions fill the unfinished design and remain editable in `config/model.yaml`.

All physical masses are assumptions from the handoff, because the native custom CAD parts do not supply usable material/mass data:

| Component | Assumed mass |
| --- | --- |
| Fork | 0.20 kg |
| Upper platform | 1.00 kg |
| Central carrier equivalent | 0.20 kg |
| Each shell equivalent | 0.15 kg |
| Each carrier-fixed polar module equivalent | 0.08 kg |
| Complete model | 1.86 kg |

The fork/platform are one rigid body with a combined center of mass about 0.19109 m above the sphere center for the default dimensions. Its inertia combines the individual boxes with the parallel-axis theorem. Carrier inertia combines a central box and two equivalent solid cylinders for the fixed polar modules. Shell inertias use a uniform spherical-shell volume with the axial equatorial cut and a conical polar aperture. These are explicit positive body tensors; collision meshes do not add mass a second time.

`generated/mass_properties.json` records the resulting masses, centers of mass and full inertia tensors. `generated/model_config.json` preserves the configuration used to generate that model.

## Simulation and controller

MuJoCo 3.16.0 supplies rigid-body dynamics, gravity, unilateral contact, finite friction, and moving-surface contact. The default physics interval is 1 ms and the controller holds its command for five physics steps. The model does not apply hidden world-frame stabilizing forces. Explicit disturbance inputs used in tests are separate from the five actuator commands.

The control code first searches for an upright nearby static equilibrium using the actual contact geometry. It differentiates the complete held-command stepping map, including the belt speed updates, and constructs the full 18-state local model: nine tangent position coordinates and nine velocities. Analysis and standalone balance/tracking experiments use `full_state_lqr`, which solves a discrete LQR on that model's reachable subspace, with command scaling that accounts for the different torque and speed units. Diagnostics retain equilibrium residuals, full-model closed-loop eigenvalues, reachable rank and discarded modes. State feedback uses simulator ground truth, so this stage does not validate an IMU estimator or real sensor delays.

The default design retains material phases and yaw in the full local analysis; neutral cyclic modes do not imply convergence of every internal coordinate. The earlier `frozen_phase_feedback` controller is retained as a comparison and is not the default full-state design. A locally controllable linear model alone does not prove successful nonlinear balance or tracking. Inspect the run metrics, saturation, contact contributors and floor-contact failures alongside the rendered motion. The demonstrated shell–belt transition covers one path; other phases, transitions and disturbances require further checks.

The interactive lab's `transition_aware` controller builds on those local designs with contact-weighted phase interpolation, heading-aware errors, a wrapped shaft-angle error, shell speed feedback without an arbitrary shell material-angle target, and incoming-drive preparation. Its reference heading follows unregulated yaw; it does not provide absolute-heading control. The previous interactive behavior is retained as `local_lqr`. The [upgrade notes](CONTROL_UPGRADE.md) distinguish feedback scheduling, kinematic preparation, and the limits of sampled recovery evidence.

## Files and next refinements

| Path | Purpose |
| --- | --- |
| `config/model.yaml` | Physical assumptions, dimensions, contact settings and actuator bounds |
| `src/spherical_wheel/model.py` | Parameterized MJCF/URDF generation and mass properties |
| `src/spherical_wheel/sim.py` | Nonlinear plant, input bounds, belt motion and contact reporting |
| `src/spherical_wheel/contact.py` | Optional clipped analytical sphere–floor contacts and boundary fallback |
| `src/spherical_wheel/control.py` | Equilibrium, finite differences and local controller |
| `src/spherical_wheel/recovery_control.py` | Contact-weighted feedback, heading-aware errors and fingerprinted gain cache |
| `src/spherical_wheel/synchronization.py` | Kinematic preview and bounded incoming-drive preparation |
| `src/spherical_wheel/validation.py` | Mechanism and physics diagnostics |
| `src/spherical_wheel/terrain.py` | Analytic uneven ground: height, slope, curvature and friction map |
| `src/spherical_wheel/offroad.py` | The plant on uneven ground: tangent-plane contact under the wheel |
| `src/spherical_wheel/gazebo/`, `gazebo/` | Gazebo reconstruction: SDF model, worlds, bridge and physics plugins, Mars yard (see GAZEBO.md) |
| `src/spherical_wheel/contact_sensing.py` | Ground-reaction measurement, normal from force and moment, friction estimate |
| `src/spherical_wheel/offroad_control.py` | Balance and tracking feedback that uses the ground reaction |
| `src/spherical_wheel/offroad_experiments.py`, `offroad_suite.py` | Off-road runs, metrics, scenario suites, tuning and slope limits |
| `src/spherical_wheel/offroad_analysis.py`, `offroad_geometry.py`, `offroad_view.py` | Figures, the wheel's support and leverage map, rendered stills |
| `generated/model.xml` | Generated primary MuJoCo model |
| `generated/model.urdf` | Companion geometry/inertia/three-revolute topology |
| `generated/model_preview.png` | Inspected nominal model view |
| `results/` | Saved analysis and simulation runs |

The URDF does not encode the ideal moving belt surface law. Use the MJCF/Python plant for the five-channel simulation.

The main fidelity improvements are measured component mass properties, final fork/upper payload properties, a confirmed exposed belt surface and drive route, actual motor/gear limits, and experimentally identified friction. A hardware controller also needs sensor estimation, latency and rate limits, actuator dynamics, and operating-point/transition validation. The current configuration is a reproducible starting point for that work.
