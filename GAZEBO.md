# Gazebo reconstruction and the ERC Mars yard

This adds a Gazebo Harmonic simulation of the wheel and runs the **unchanged contact-aware off-road controller** in it: the same Python code, the same 80-point gain library, the same runner and starting states as the MuJoCo study in [OFFROAD.md](OFFROAD.md). It then drives the wheel over Leo Rover's ERC 2021 Mars yard. Everything is still simulation of the same provisional model (assumed masses and friction, ideal belt servos, ground-truth state and contact force).

**Headline.**

* **The port behaves like MuJoCo.** In free space the two simulators agree to 5e-7 in position and 7e-8 in velocity after 1 s of random torques. Standing still, the measured contact load is the weight to 0.2%. Driving a polar belt moves the wheel in the same direction at a speed within 10%. On the eleven off-road scenarios all 22 runs finish in both simulators; the contact-aware controller's largest tracking error agrees within a few millimetres on most of them, and the flat-floor controller fails on grades in the same way.
* **On the Mars yard** the contact-aware controller finished 37 of 44 runs (11 routes from 4 starts): every start on the plain, a circle, a figure-eight, a 15 m patrol loop, a 12 deg climb and both directions over a 0.72 m crater rim, with median final errors of 7-35 mm. It fell on an 18 deg descent whose facets reach 27 deg (3 of 4 starts) and on a 6 cm lava edge (4 of 4). The flat-floor controller finished 3 of 11 routes, all nearly level, 0.4-0.9 m off the path. Only 2 of 44 runs pass every pre-declared gate: tracking mostly does, but attitude error exceeds 8 deg on this rough DEM and the freely turning platform fails the settling test.

## Run it

The Gazebo environment needs no `sudo`: it is a conda environment with conda-forge's Gazebo Harmonic (the release paired with ROS 2 Jazzy).

```bash
conda env create -f environment/gazebo.yml
conda activate asw-gz
bash scripts/setup-gazebo.sh          # pip packages, both plugins (build/gazebo), the Mars yard
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests/test_gazebo.py

python -m spherical_wheel.gazebo parity --workers 5 --mujoco-reference   # 11 scenarios x 2 controllers, both simulators
python -m spherical_wheel.gazebo mars --workers 5                        # 11 Mars routes x 2 controllers
python -m spherical_wheel.gazebo mars --controllers terrain_aware_heading_hold
python -m spherical_wheel.gazebo mars --controllers terrain_aware terrain_aware_fall_line   # fall-line heading steering (Follow-up below)
MUJOCO_GL=egl python -m spherical_wheel.gazebo mars-lqr                 # original local LQR vs contact-aware, side-by-side MP4s
MUJOCO_GL=egl python -m spherical_wheel.gazebo mars-observer            # contact-aware with and without force sensing, side-by-side MP4s
python -m spherical_wheel.gazebo view --route crater_rim_climb           # real-time pace; then `gz sim -g` in another terminal
```

Results go to `results/gazebo/{parity,mars}/` (each run: `summary.json`, `rollout.npz`, `plots.png`; each set: `REPORT.md`). A run takes about as long as it simulates, so the parity suite takes about 3 minutes with five workers and the Mars set about 10. The GUI path (`view` plus `gz sim -g`) has not been tried here; the batch commands run headless.

To build against ROS 2 Jazzy's own Gazebo instead (`sudo apt install ros-jazzy-ros-gz`), point `CMAKE_PREFIX_PATH` at `/opt/ros/jazzy/opt/gz_*`. The ROS vendor packages may carry a different DART; the deactivation switch below is a no-op on versions without it.

## How the reconstruction works

```text
Python, per 5 ms control interval                 Gazebo server (same process), 5 x 1 ms DART steps
------------------------------------              ------------------------------------------------
OffroadController.command()  (unchanged)          SphericalWheelBridge plugin
  reads plant.data   <- kinematic mirror  <-----    fork pose/twist, joint angles/speeds
  reads contacts     <- contact records   <-----    every contact: position, normal, force, depth
  5 commands -------------------------------->      joint torques; belt surface speed; push wrench
                                                   AswExactPhysics (gz-sim physics, exact state)
```

**Model.** [`model_sdf.py`](src/spherical_wheel/gazebo/model_sdf.py) writes the SDF model from `config/model.yaml` through the same `generate_model` and `mass_properties` as the MuJoCo model, so masses, centres of mass, inertia tensors, joint axes, damping (1e-4 N m s) and torque bounds are the same numbers. Every link origin is at the wheel centre and every joint angle is zero in the generated pose, so Gazebo joint angles are MuJoCo's `qpos[7:10]`. The native CAD meshes are the visuals.

**Contacts.** MuJoCo's default contact (`analytic_clipped_floor`) uses four concentric spheres, one per shell and belt, and admits a contact on one of them only where the support point lies on that surface's own patch; outside every patch the faceted rim geometry carries the wheel. Gazebo has no collision filter that can change per contact, so the bridge does it in the contact-surface hook: a contact on the wrong patch gets a target separation speed of -1000 m/s along its normal, which makes DART's solver give it zero force and zero friction. The 48-sided equatorial rims are capsules through the same rim vertices. A belt contact gets a target tangential velocity equal to the commanded belt speed along the carrier's y axis, which is the ideal belt servo of the MuJoCo model. Every admitted contact uses the ground friction coefficient of the current control interval (MuJoCo: the floor's friction under the wheel).

**Contact-solver limitation found in I-69.** The installed Dantzig solver can
retain tangential impulses after a contact's final normal impulse becomes zero,
because its friction bounds were set from an intermediate normal solution.
Direct native capture reproduces the specific shell forces and preserves all
original motion/command arrays. Positive-normal-load labels therefore do not
capture every force-bearing body in those samples. Default PGS failed a controlled
probe; no solver replacement is selected. See the [source, native evidence and
next validation step](docs/iterations/2026-11-27/contact-force-boundary/README.md).


**Same controller.** The controller reads `plant.model`/`plant.data` for frames and Jacobians, and the solved contacts for force sensing, the rolling feedforward and the incoming-drive preview. [`GazeboPlant`](src/spherical_wheel/gazebo/plant.py) keeps MuJoCo's model and data only as a *kinematic mirror* of Gazebo's state (written and `mj_forward`-ed after every interval, never integrated), and supplies Gazebo's contacts as records. `measure_contact`, `rolling_feedforward` and the synchronizer use the records whenever a plant provides them; their MuJoCo code paths are untouched. Fed with records built from MuJoCo's own contacts, the records paths reproduce the MuJoCo paths **bit for bit** over 3,600 control steps on three terrains (`tests/test_gazebo.py`). The original branch and this branch produce bit-identical MuJoCo runs (hills and the 9,101-step course). The gain library is the MuJoCo design, loaded from the same fingerprinted cache.

**Terrain.** The analytic test terrains go to Gazebo as triangle meshes sampled on a 1 cm grid (vertices exact to 5e-8 m). The plant checks at start-up that Gazebo collides with the surface the terrain model describes, by pressing the wheel 0.5 mm into it at several points; a wrong ground raises an error.

### Gazebo and DART behaviour that had to be corrected

These are properties of the stock software, not of the wheel, and each one changed results before it was fixed.

| Problem | Effect on this robot | Fix |
| --- | --- | --- |
| gz-sim copies a link's pose and velocity to the entity database only when the pose moved by more than 1e-6 | A wheel balancing near rest read a frozen velocity (observed: constant vz = -g dt) | `gazebo/physics`: gz-sim 8.10's physics system with exact per-step link state ([README](gazebo/physics/README.md)) |
| DART 6.19 puts a skeleton to sleep after 0.5 s below 1 cm/s | Gravity and contacts stopped: the standing wheel froze and every contact read zero force | Same system switches DART's automatic deactivation off |
| A link velocity command component stays after use and is re-applied as zero | The base was held at zero velocity on every later step | The bridge removes it after a reset |
| DART corrects penetration to exactly zero | The next collision check misses the contact; the wheel free-falls a step (micro-hop) | 50 µm contact error allowance (MuJoCo's soft contact rests at 33.5 µm) |
| Gazebo's height-map loaders place a generated map wrongly (flat at the highest sample, or offset) | The ground was not the terrain | Triangle meshes for the test terrains and the Mars DEM |
| Gazebo's mesh manager caches meshes by path for the whole process | A worker's later scenarios ran on its first scenario's ground | Mesh file names carry a hash of the terrain; the ground check catches any repeat |
| gz-sim's CMake target links GUI, rendering and FFmpeg | The plugin failed to load (conda libsystemd needs a newer glibc than Ubuntu 24.04's) | Plugins link with `--as-needed` |

Two settings were chosen by measurement: the ODE collision detector (rolling vertical-velocity noise near 1e-7 m/s against 1e-4 to 1e-2 m/s with Bullet, and every per-contact decision matched), and resets applied during a 1 µs physics step so that the starting state and its contact forces are those requested.

## Gazebo against MuJoCo

`python -m spherical_wheel.gazebo parity --mujoco-reference`. Same controller, gains, scenarios and starting state; MuJoCo run on the same machine. Tracking errors in mm, Gazebo / MuJoCo:

| Scenario | Contact-aware: max error | final error | Flat baseline: max error | final error | All 4 runs finished |
| --- | --- | --- | --- | --- | --- |
| flat forward | 21.9 / 21.2 | 1.4 / 1.1 | 21.1 / 21.1 | 0.8 / 1.1 | yes |
| ramp forward | 31.2 / 33.4 | 1.8 / 0.5 | 639.8 / 675.6 | 630.7 / 675.6 | yes |
| ramp lateral | 41.4 / 30.3 | 35.6 / 2.1 | 714.3 / 714.3 | 713.7 / 710.5 | yes |
| cross slope | 24.6 / 21.7 | 1.1 / 14.0 | 853.9 / 820.5 | 849.7 / 820.5 | yes |
| hills | 62.9 / 61.1 | 3.4 / 1.5 | 714.8 / 709.0 | 681.8 / 687.0 | yes |
| washboard | 71.3 / 123.4 | 2.6 / 0.4 | 488.0 / 460.1 | 29.6 / 63.9 | yes |
| bumps | 88.6 / 86.8 | 4.6 / 4.9 | 854.8 / 715.1 | 216.4 / 71.4 | yes |
| rough | 39.7 / 49.9 | 0.3 / 9.8 | 274.7 / 278.3 | 248.4 / 229.6 | yes |
| mud | 23.9 / 23.2 | 1.6 / 1.2 | 23.0 / 23.0 | 0.9 / 2.2 | yes |
| slick ramp | 31.2 / 33.3 | 0.5 / 0.5 | 624.4 / 675.6 | 621.7 / 675.6 | yes |
| course | 36.5 / 158.5 | 2.3 / 2.9 | 847.7 / 875.2 | 60.0 / 259.3 | yes |

* **Level ground is the same.** `flat_forward` (21.9 / 21.2 mm, attitude 0.63 / 0.62 deg) and `mud` match closely, and the time on each shell and belt matches to the percent (66.3 / 21.0 / 8.3 / 4.4% against 66.3 / 21.1 / 8.3 / 4.3%), which also checks the belt and patch handling.
* **The controller comparison is the same.** In both simulators the contact-aware controller holds every grade and the flat-floor controller ends 0.6-0.85 m short on grades and banks.
* **Where they differ is the contact model.** DART's rigid contacts give higher peak loads (up to about 5.6 against 1.9 times weight on the ramp) and the friction is a box, not MuJoCo's elliptic cone. On the shell-driven 8 deg ramp Gazebo stops 36 mm short where MuJoCo stops 2 mm short; MuJoCo itself stops about 30 mm short from other starts (OFFROAD.md, Limits). Unregulated heading drifts differently. The largest errors on the course, washboard and rough ground are smaller in Gazebo; the original report notes that these depend on where surface handovers fall.

## The ERC 2021 Mars yard

**The map.** Fictionlab's blog post describes the ERC Mars yard model built from a drone survey. The models are in Leo Rover's [`leo_simulator-ros2`](https://github.com/LeoRover/leo_simulator-ros2) (`leo_gz_worlds`, MIT). `marsyard2021` is a 47 m x 47 m DEM (500 x 500 samples, 9.4 cm apart, 6.6 m of relief) plus lava and rock meshes; it is the finest of the three (2022: 19.5 cm; 2020: one large scanned mesh). [`mars.py`](src/spherical_wheel/gazebo/mars.py) loads it as Leo's model does, with one change: the DEM's collision is a triangle mesh with one vertex per DEM sample, because Gazebo's DEM loader resamples the grid (measured 4 mm median, 17 mm 90th-percentile differences from the samples). The visual is Leo's textured height map and the lava and rock meshes are used unchanged. The mirror's ground model is the same triangulation, exactly.

**What the yard asks of a 0.24 m wheel.** Survey of the DEM (`results/gazebo/mars/terrain_map.png`):

| Measure | Value |
| --- | --- |
| Cell slope, median / 90th percentile | 13.0 / 32.4 deg |
| Share of the yard below 8 / 12 / 16.7 deg | 28 / 47 / 66% |
| Lava and rock footprint (more than 1 cm above the ground) | 5.7% of the yard; median height 9 cm, up to 55 cm |

Even the open plain has 5-10 deg cell-to-cell slopes at the DEM's resolution and about 14 mm of height variation below the wheel's scale. The holding limits of the declared model are 16.7 deg across a shell, 43 deg along the shaft-driven direction and 35 deg for friction 0.7 (OFFROAD.md).

**Routes** were chosen by scoring thousands of candidates against the DEM smoothed to wheel scale and the rasterised rocks:

| Route | Length | Wheel-scale slope, max / mean | Net height | Start, heading | What it tests |
| --- | --- | --- | --- | --- | --- |
| plain_traverse | 8.0 m | 8.6 / 4.8 deg | -0.14 m | (-1.8, 7.2), 113 deg | Easiest 8 m of open plain; about 14 mm of height variation below wheel scale. |
| plain_traverse_shell | 8.0 m | 8.6 / 4.8 deg | -0.14 m | (-1.8, 7.2), 203 deg | The same plain, rolling on a shell (platform turned 90 deg to the route). |
| plain_fast | 8.0 m | 8.6 / 4.8 deg | -0.14 m | (-1.8, 7.2), 113 deg | The same plain at twice the cruise speed (0.2 m/s). |
| moderate_climb | 6.0 m | 12.3 / 6.4 deg | +0.19 m | (12.4, -1.8), 284 deg | 6 m climbing 0.19 m over moderate slopes. |
| crater_rim_descent | 5.0 m | 15.7 / 9.9 deg | -0.72 m | (-13.9, 13.3), 67 deg | 5 m down the left crater rim, 0.72 m drop. |
| crater_rim_climb | 5.0 m | 15.7 / 9.9 deg | +0.72 m | (-12.0, 17.9), 247 deg | The same rim driven uphill: 0.72 m climb. |
| steep_descent | 5.0 m | 18.3 / 11.0 deg | -0.82 m | (-6.7, 7.9), 187 deg | 5 m into a crater, 0.82 m drop, steeper than the 16.7 deg side-slope bound. |
| lava_crossing | 5.0 m | 11.2 / 5.6 deg | -0.07 m | (-4.6, -0.8), 161 deg | 5 m with about a third of the path over lava up to 6.3 cm high. |
| plain_circle | 2.8 m | 4.2 / 2.7 deg | +0.00 m | (12.4, -1.9), 90 deg | A 0.45 m radius circle on gentle rock-free ground, counter-clockwise. |
| plain_figure8 | 2.7 m | 6.9 / 4.8 deg | +0.00 m | (-1.6, 9.6), 45 deg | A 0.9 m figure-eight on the plain: changing curvature and turn direction. |
| plain_patrol | 14.9 m | 9.7 / 5.1 deg | +0.00 m | (1.5, 10.6), 77 deg | A closed 15 m patrol loop on the plain west of the lava field, clear of features over 2 cm by 0.25 m. |

Each run starts at rest on the ground in static balance, with the shaft-driven direction along the route unless stated, moves at 0.1 m/s cruise (0.2 m/s for `plain_fast`) and has one second before and three after. The acceptance gates are the mobility evaluation's, fixed before running: moving RMS <= 50 mm, 95th percentile <= 100 mm, max <= 200 mm, final <= 50 mm, attitude error <= 8 deg, saturation <= 10%, settled in the last second, and closure <= 50 mm for closed routes.

### Results

Each cell: outcome, then largest / final tracking error and largest attitude error. Every run with its figure is in `results/gazebo/mars/<route>/<controller>/`; all of them on one map: `results/gazebo/mars/overview.png`.

**Default start (wheel phase 0.30), three controller settings.**

| Route | Contact-aware | Contact-aware + heading hold | Flat baseline |
| --- | --- | --- | --- |
| plain_traverse | finished: 168 / 35 mm, 10 deg | finished: 343 / 40 mm, 12 deg | lost path: 1000 / 1000 mm, 12 deg |
| plain_traverse_shell | finished: 256 / 36 mm, 10 deg | finished: 195 / 5 mm, 9 deg | fell: 732 / 732 mm, 55 deg |
| plain_fast | finished: 285 / 18 mm, 14 deg | finished: 166 / 38 mm, 12 deg | finished: 828 / 103 mm, 34 deg |
| moderate_climb | finished: 289 / 9 mm, 13 deg | finished: 306 / 29 mm, 14 deg | lost path: 1000 / 1000 mm, 10 deg |
| crater_rim_descent | finished: 326 / 22 mm, 14 deg | finished: 170 / 40 mm, 6 deg | lost path: 1002 / 1002 mm, 19 deg |
| crater_rim_climb | finished: 159 / 6 mm, 8 deg | finished: 499 / 50 mm, 23 deg | fell: 840 / 840 mm, 49 deg |
| steep_descent | fell: 460 / 460 mm, 32 deg | finished: 410 / 54 mm, 18 deg | lost path: 1000 / 1000 mm, 11 deg |
| lava_crossing | fell: 119 / 119 mm, 18 deg | fell: 55 / 55 mm, 15 deg | lost path: 1000 / 1000 mm, 13 deg |
| plain_circle | finished: 74 / 15 mm, 7 deg | passed all: 122 / 4 mm, 7 deg | finished: 930 / 166 mm, 14 deg |
| plain_figure8 | finished: 148 / 35 mm, 9 deg | finished: 102 / 19 mm, 8 deg | finished: 418 / 167 mm, 7 deg |
| plain_patrol | finished: 344 / 12 mm, 19 deg | finished: 157 / 2 mm, 11 deg | lost path: 1000 / 1000 mm, 22 deg |

**Contact-aware controller from four starting wheel phases** (0.30, 0.90, 2.20 and -1.00 rad; 44 runs). Single runs on this yard are sensitive: rerunning the patrol loop after a 1e-5 m/s change in its reference moved its largest error from 157 to 344 mm, so the medians are the figures to compare.

| Route | Finished | Passed all gates | Max error mm, median (worst) | Final error mm, median (worst) | Max attitude error deg, median (worst) |
| --- | --- | --- | --- | --- | --- |
| plain_traverse | 4/4 | 0/4 | 116 (168) | 8 (35) | 9.4 (11.9) |
| plain_traverse_shell | 4/4 | 0/4 | 169 (256) | 16 (36) | 11.0 (14.4) |
| plain_fast | 4/4 | 0/4 | 191 (338) | 11 (18) | 11.0 (14.0) |
| moderate_climb | 4/4 | 0/4 | 221 (289) | 35 (45) | 8.0 (12.8) |
| crater_rim_descent | 4/4 | 0/4 | 261 (372) | 31 (49) | 15.3 (23.3) |
| crater_rim_climb | 4/4 | 0/4 | 86 (159) | 7 (17) | 5.1 (8.0) |
| steep_descent | 1/4 | 0/4 | 387 (460) | 269 (460) | 43.3 (58.7) |
| lava_crossing | 0/4 | 0/4 | 91 (119) | 88 (119) | 18.0 (23.9) |
| plain_circle | 4/4 | 2/4 | 99 (114) | 10 (42) | 7.3 (7.7) |
| plain_figure8 | 4/4 | 0/4 | 137 (167) | 31 (55) | 10.2 (10.6) |
| plain_patrol | 4/4 | 0/4 | 299 (344) | 10 (20) | 15.7 (26.5) |

What the runs show:

* **The contact-aware controller drives the yard; the flat-floor controller does not.** The contact-aware controller finished 37 of 44 runs, including every start on the open plain, the circle, the figure-eight, the 15 m patrol loop, the 12 deg climb and both directions over the 0.72 m crater rim. Its final position error is a median of 7-35 mm on those routes. The flat-floor controller finished 3 of 11 routes, all nearly level, 0.4-0.9 m off the path, and lost or fell on every route with a sustained grade, as on the analytic grades.
* **Two limits.** On the steep descent the faceted DEM under the wheel reaches 27 deg; once the heading has drifted, the wheel crosses that slope on a shell, whose holding bound is 16.7 deg, slides and falls (finished 1 of 4 starts). The lava crossing climbs a 6 cm lava edge, half the wheel radius, saturating the shaft and both shells; it then slides off the edge (0 of 4). A step is outside the smooth-ground design of this controller (OFFROAD.md, Limits), and the contact force there does not come from one support point.
* **Tracking mostly meets the targets; attitude and settling do not.** Only 2 of 44 runs pass every gate. Final and RMS errors are mostly within target, and on the routes it finished the largest errors (a median of 86-299 mm) come from handovers over facet edges; peak contact loads reach 6-13 times weight on those edges with DART's rigid contact. The largest attitude error exceeds 8 deg on most routes, and the end-of-run settling test fails because the platform is still turning.
* **Heading is the controller's weak point here.** Heading is unregulated by default, and on the yard the platform turned by up to 2,840 deg over 8 m. The existing heading hold (0.2 N m/rad, through the idle shell) turned the steep descent into a finish and gave the only route that passes every gate, but it made the plain traverse and the crater-rim climb worse, and the heading still swings by hundreds of degrees: the idle shell's reaction leaves it little authority. Holding heading on rough ground needs more than this option provides.

### Follow-up: why the slope runs fall, and fall-line steering

A follow-up study (`results/gazebo/mars_yaw_study/REPORT.md`, kept local like the rest of `results/`) traces every slope fall above to heading. The wheel holds a grade along its shaft-driven direction up to 43 deg but across it only 16.7 deg on a shell and 7.8 deg on a belt cap. In all six saved slope falls the shaft-driven direction had drifted 51-82 deg off the fall line first. The drift comes from the flat-floor gain library, which uses the unloaded shell as a roll reaction wheel whose reaction turns the platform. A new option, `heading_reference='fall_line'` with shell steering bounded by a roll budget (preset `terrain_aware_fall_line`), turns the shaft-driven direction onto the fall line, mostly while a pole is down. Rerun with the current code from 27 paired starts on the three slope routes, it finished 25 against 16 (exact McNemar p = 0.004), and it finished all 14 runs on the easier routes with unchanged tracking. The lava crossing stays out of reach: its edges need more friction and torque than the model has. The saved runs above came from an earlier state of the code and do not reproduce exactly; the study reran its comparisons.

### Original local LQR against the contact-aware controller, with replays

`MUJOCO_GL=egl python -m spherical_wheel.gazebo mars-lqr` runs the original interactive lab's `local_lqr` (the unmodified `LocalLQRAdapter` of `scripts/compare-local-lqr-offroad.py`) and the contact-aware controller on ten distinct routes, each pair from one identical starting state: at rest in the same balanced lean, heading 0 and wheel phase 0.30. Heading 0 is the local LQR's own operating point, because it uses raw world-frame errors with gains designed at heading 0; the contact-aware runs in the tables above instead start turned along each route. Each route has a side-by-side MP4 replay rendered from the logged Gazebo states with the project's MuJoCo renderer (the same DEM triangulation, Leo's lava and rock meshes cropped around the route); routes longer than 45 s play at 2x, stated in the caption.

| Route | Start slope | Original local LQR | Contact-aware (force sensing) | Replay |
| --- | --- | --- | --- | --- |
| plain_traverse | 11.2 deg | fell at 1.2 s | finished: max 150 mm, final 20 mm | `results/gazebo/mars_local_lqr/plain_traverse/replay.mp4` |
| plain_fast | 11.2 deg | fell at 1.2 s | finished: max 220 mm, final 7 mm | `results/gazebo/mars_local_lqr/plain_fast/replay.mp4` |
| moderate_climb | 2.5 deg | fell at 7.4 s | finished: max 125 mm, final 37 mm | `results/gazebo/mars_local_lqr/moderate_climb/replay.mp4` |
| crater_rim_descent | 3.3 deg | fell at 2.0 s | fell at 36.2 s | `results/gazebo/mars_local_lqr/crater_rim_descent/replay.mp4` |
| crater_rim_climb | 8.4 deg | lost path at 9.1 s | fell at 33.5 s | `results/gazebo/mars_local_lqr/crater_rim_climb/replay.mp4` |
| steep_descent | 11.6 deg | fell at 1.6 s | fell at 11.0 s | `results/gazebo/mars_local_lqr/steep_descent/replay.mp4` |
| lava_crossing | 11.1 deg | fell at 1.1 s | fell at 9.8 s | `results/gazebo/mars_local_lqr/lava_crossing/replay.mp4` |
| plain_circle | 2.3 deg | fell at 6.7 s | finished: max 89 mm, final 34 mm | `results/gazebo/mars_local_lqr/plain_circle/replay.mp4` |
| plain_figure8 | 7.6 deg | fell at 10.2 s | finished: max 149 mm, final 41 mm | `results/gazebo/mars_local_lqr/plain_figure8/replay.mp4` |
| plain_patrol | 5.3 deg | fell at 2.5 s | finished: max 278 mm, final 16 mm | `results/gazebo/mars_local_lqr/plain_patrol/replay.mp4` |

* **The local LQR falls on every route, mostly within two seconds.** Every Mars start is on a sloped facet and both controllers start in the lean that balances on it. The local LQR has no slope model, sees that lean as an attitude error and saturates all three drives in its first command. From the same start on the same ground, MuJoCo gives the same first command and the same falls (plain traverse 1.27 s against 1.25 s in Gazebo; circle 6.43 against 6.67 s), so this is the controller, not the simulator. On Gazebo's level plane the same local LQR drives the 1 m `flat_forward` path with a 21 mm largest error and 1 mm final error, so the falls come from the slopes.
* **The contact-aware controller finished 6 of the 10 routes from this start.** Started at heading 0 rather than along the route, it crosses the 15-16 deg crater rims partly on a shell, whose holding bound is 16.7 deg, and falls on both rim routes, which it finished when started along them; the steep descent and the lava crossing fail as before.

### Shape routes with the contact-aware controller

`MUJOCO_GL=egl python -m spherical_wheel.gazebo mars-shapes` runs four shapes (`mars.SHAPE_ROUTES`) with the contact-aware controller only, and renders one replay per route at `results/gazebo/mars_shapes/<route>/replay.mp4` (played at 2x). Each starts at rest with the shaft-driven direction along the first leg, wheel phase 0.30, and cruises at 0.1 m/s. The triangle, line and zigzag come to rest at every corner (`PolylinePath`); the circle holds a constant speed (`CirclePath`). Each was placed at the lowest worst-case wheel-scale slope found in the terrain survey, at least 0.35 m from lava and rock. These are single runs from one start.

| Route | Shape | Wheel-scale slope, max / mean | Result | Moving RMS / max / final mm | Closure mm | Max attitude error deg |
| --- | --- | --- | --- | --- | --- | --- |
| shape_circle | 1 m radius, counter-clockwise, from (-14.8, -6.0) | 6.4 / 3.2 deg | finished; all gates but settling | 32 / 80 / 1 | 11 | 5.2 |
| shape_triangle | 2 m sides, counter-clockwise, from (2.6, 17.4) | 5.6 / 2.4 deg | passed all gates | 26 / 84 / 2 | 10 | 3.8 |
| shape_line_and_back | 3 m out at 285 deg and back, from (-19.2, 3.2) | 4.4 / 2.4 deg | passed all gates | 27 / 64 / 5 | 10 | 4.7 |
| shape_zigzag | five 1.5 m legs at +/-45 deg about 15 deg, from (0.6, 16.6) | 6.7 / 3.8 deg | passed all gates | 29 / 96 / 8 | - | 4.1 |

The circle misses only the settling gate: the platform is still turning at the end. Its heading moved up to 644 deg over the lap, and 404 deg on the line and back, because heading is unregulated (see above).

### Contact-aware control without force sensing (`momentum_observer`)

`MUJOCO_GL=egl python -m spherical_wheel.gazebo mars-observer` runs all 15 routes (the 11 above and the 4 shapes) twice from one identical start each: at rest in static balance, wheel phase 0.30, shaft-driven direction along the route. `terrain_aware` reads the ground force and moment from Gazebo's solved contacts. `momentum_observer` estimates them from the motion, the drive torques and the MuJoCo model (OFFROAD.md); DART produces the motion, so the observer's model is not the one being integrated. Both still read the per-surface loads, the rolling feedforward's contact points and the preview's ground plane from Gazebo. `terrain_aware` was rerun alongside with the current code, so the two columns compare with each other, not with the tables above. Each route has a side-by-side replay at `results/gazebo/mars_observer/<route>/replay.mp4` (kept local, like the rest of `results/`).

| Route | `terrain_aware` | `momentum_observer` | Observer against solved force, median / 95th percentile N |
| --- | --- | --- | --- |
| plain_traverse | finished: 110 / 2 mm | finished: 158 / 57 mm | 0.02 / 2.6 |
| plain_traverse_shell | finished: 219 / 17 mm | finished: 279 / 36 mm | 0.02 / 3.1 |
| plain_fast | finished: 143 / 4 mm | finished: 329 / 8 mm | 0.05 / 2.5 |
| moderate_climb | finished: 405 / 32 mm | finished: 169 / 20 mm | 0.03 / 2.1 |
| crater_rim_descent | finished: 418 / 29 mm | fell at 33.8 s | 0.03 / 4.0 |
| crater_rim_climb | finished: 134 / 7 mm | finished: 436 / 4 mm | 0.03 / 1.8 |
| steep_descent | fell at 7.4 s | finished: 294 / 24 mm | 0.04 / 2.9 |
| lava_crossing | finished: 302 / 83 mm | fell at 17.2 s | 0.03 / 5.1 |
| plain_circle | finished: 74 / 11 mm | finished: 93 / 12 mm | 0.00 / 1.9 |
| plain_figure8 | finished: 130 / 29 mm | finished: 190 / 40 mm | 0.01 / 2.2 |
| plain_patrol | finished: 285 / 53 mm | finished: 336 / 5 mm | 0.03 / 2.0 |
| shape_circle | finished: 192 / 2 mm | finished: 115 / 14 mm | 0.02 / 1.6 |
| shape_triangle | finished: 84 / 2 mm | finished: 50 / 7 mm | 0.01 / 1.1 |
| shape_line_and_back | finished: 75 / 3 mm | finished: 73 / 3 mm | 0.01 / 0.9 |
| shape_zigzag | finished: 92 / 6 mm | finished: 62 / 2 mm | 0.01 / 0.9 |

Cells give largest / final tracking error.

* **The estimate matches Gazebo's contact force.** The median difference is 0.00-0.05 N on every route, against a weight of about 18 N. The 95th percentile is 0.9-5.1 N, at impacts, because the observer reports the average over a 5 ms interval and the solver an instant. The sensed slope direction has a median error of 0.03 deg or less on 14 of 15 routes (3.3 deg on the lava crossing), against 0.05-0.10 deg with the solved contacts on 13 of 15.
* **Outcomes differ on three routes, where both controllers reach the shell torque limit.** `momentum_observer` finished 13 of 15 and `terrain_aware` 14 of 15. The steep descent and the lava crossing are at the edge for the contact-aware controller anyway (1 of 4 and 0 of 4 starts above). The crater rim descent counts against the observer: `terrain_aware` finished it from all four starts above. One start per route cannot rank the two there.
* **Tracking is mixed.** On the 12 routes both finished, the observer's largest error is larger on 7, smaller on 4 and about equal on 1. With one start per route and the start-to-start spread seen above, these differences are not conclusive.
* **Repeated Gazebo runs are not bit-identical.** Rerunning plain_fast with the same code and start matched the saved run for the first 3.5 s, then drifted apart from a 1e-12 N difference in the solved contact force. The observer's largest error came out at 144 mm in one rerun against 329 mm in the table, and a rerun of `terrain_aware` also drifted. Which run drifts varies between reruns, so the source is in the Gazebo harness, not in either controller. Single-run largest errors on this yard are therefore only indicative.

## Limits

* **Simulation of a provisional model**, as in OFFROAD.md: assumed masses, inertias, friction and torque bounds, ideal belt speed servos, no motor curves, ground-truth state and contact wrench (`momentum_observer` estimates the wrench but keeps the rest), no sensor noise or delay.
* **Two contact models, neither exact.** MuJoCo's is soft with an elliptic friction cone; DART's is rigid with a box friction pyramid, a 50 µm allowance and DART's default 1 mm/s penetration correction. Agreement between them describes shared behavior; it does not rule out solver-specific force artifacts. I-69 documents one such Dantzig approximation. It is not a hardware prediction.
* **The yard's DEM is the surface.** At 9.4 cm resolution with linear facets, the wheel rolls over facet edges; the real yard is softer and has detail below that scale. Rocks and lava are the meshes Leo's world uses. Gazebo keeps at most 20 contacts per collision pair (SDF default), which a 12 cm sphere on a 9.4 cm-faceted surface does not approach at rest.
* **Heading is unregulated by default.** On the yard the platform can turn several full revolutions while the wheel follows its path; the existing heading hold is evaluated separately above.
* **Sampled runs.** Four starts per route for the contact-aware controller and one for the other settings; finishing these routes is not a guarantee for others.

## Files

| Path | Purpose |
| --- | --- |
| `gazebo/plugin/SphericalWheelBridge.cc` | Lockstep bridge: commands, patch rule, belts, contact readout, resets |
| `gazebo/physics/` | gz-sim 8.10 physics system with exact state and no DART sleeping (Apache-2.0, modified) |
| `src/spherical_wheel/gazebo/model_sdf.py` | SDF model from the MuJoCo model's inputs |
| `src/spherical_wheel/gazebo/worlds.py` | Worlds: plane, terrain meshes, physics settings |
| `src/spherical_wheel/gazebo/bridge.py`, `plant.py` | In-process server and the plant with a kinematic mirror |
| `src/spherical_wheel/gazebo/parity.py` | Gazebo against MuJoCo on the off-road scenarios |
| `src/spherical_wheel/gazebo/mars.py` | Mars yard ground model, world, routes, runs and report |
| `src/spherical_wheel/gazebo/mars_compare.py` | Original local LQR against the contact-aware controller on the yard; replay videos |
| `src/spherical_wheel/gazebo/mars_shapes.py` | Circle, triangle, line-and-back and zigzag routes with the contact-aware controller; replay videos |
| `src/spherical_wheel/gazebo/mars_observer.py` | Every route with `terrain_aware` and `momentum_observer` from matched starts; side-by-side replay videos |
| `tests/test_gazebo.py` | Equivalence and parity tests |
| `environment/gazebo.yml`, `scripts/setup-gazebo.sh`, `scripts/fetch-marsyard.sh` | Environment, build and data |
