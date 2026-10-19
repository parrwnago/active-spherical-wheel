# Interactive spherical wheel lab

The browser interface runs the actual MuJoCo plant and feedback controller. You can tilt the starting platform, push it during balance or tracking, watch the response, and export the measured run. The view is a live JPEG stream of the current simulated state. MuJoCo continues the physics independently of browser refreshes.

## Start it on Linux or Ubuntu under WSL2

Use Python 3.10–3.12 and a normal Chrome or Firefox browser. From the extracted project directory:

```bash
bash scripts/run-lab.sh
```

The launcher creates the local Python environment and installs pinned dependencies if `.venv/bin/python` is absent. Initial setup needs an internet connection. It then starts the lab and requests that your browser open it.

With the environment already installed, the manual command is:

```bash
.venv/bin/python -m spherical_wheel lab --port 8765
```

Open **http://127.0.0.1:8765** on the same Linux computer running the server. The address works while that command is running. Keep its terminal open and use Ctrl+C to stop. If automatic browser opening is unavailable, enter the address yourself. A different available port can be selected with `--port 8766`, then opened at the printed address.

No additional CAD model or export is needed for these experiments. The supplied geometry, provisional fork/platform, and physical assumptions are already included. The first uncached **Transition aware** startup builds a local gain library and takes longer; later startups reuse a valid fingerprinted cache. A new contact scenario can also take a few seconds.

### Ubuntu under WSL2

Run the launcher or manual Python command **inside your Ubuntu terminal**. Then open **http://localhost:8765** in Chrome, Edge, or Firefox **on Windows**. Keep the Ubuntu terminal running. Microsoft documents this [Windows-to-WSL localhost connection](https://learn.microsoft.com/en-us/windows/wsl/networking#accessing-linux-networking-apps-from-windows-localhost); a separate Linux browser is unnecessary.

The current Ubuntu 22.04 / WSL2 environment was checked with WSLg available. Both its default graphics backend and explicit EGL produced the live MuJoCo JPEG and state API. A Windows-native HTTP client also fetched both endpoints successfully through localhost. The [saved WSL check](results/wsl_smoke/summary.json) records both attempts.

If your graphics setup reports an OpenGL error or the live view fails, stop the lab with Ctrl+C and retry inside Ubuntu:

```bash
MUJOCO_GL=egl bash scripts/run-lab.sh
```

Or, with the environment already installed:

```bash
MUJOCO_GL=egl .venv/bin/python -m spherical_wheel lab --port 8765
```

EGL renders the scene on the Ubuntu side and the Windows browser receives the resulting images. It was verified in the current WSL2 environment. Microsoft also documents [WSLg graphics support](https://learn.microsoft.com/en-us/windows/wsl/tutorials/gui-apps) for Linux GUI applications; the separate MuJoCo desktop `viewer` uses that graphical path.

## Try transverse balance at a pole

1. Keep **Balance controller** and **Polar belt drives** enabled.
2. Set **Initial tilt on reset** to **Roll 2°**, **Pitch 0°**.
3. Select **Polar belt** below the view, or press **Reset** if it is already selected. The simulation starts paused.
4. Press **Run**. Watch tilt decrease and the belt command respond. Select **Front view** with the camera button to see the transverse motion clearly.
5. Set **Force 0.5 N**, **Pulse duration 0.10 s**. Press A or D, click a left/right direction button, or drag horizontally in the view.

The transverse belt is independently driven and remains mounted on the carrier. At polar ground contact, shell rotation about its own axis has little leverage for rolling in this direction. Belt surface motion generates a ground friction force that moves the support point beneath the upper body's center of mass. Recovery requires contact, enough friction, and sufficient actuator capacity.

The independent [polar belt check](results/polar_belt_check/README.md) verified recovery from both +2° and −2° pure roll offsets, and from ±0.5 N lateral pushes lasting 0.1 s. With belt commands forced to zero, the corresponding cases touched the floor. These particular checks isolate the contacting belt: the other three torque commands were effectively zero.

To reproduce that historical comparison, first select **Original local LQR**, then switch **Polar belt drives** off, reset the same 2° roll case, and run it. After a fall, enable the belts and select **Transition aware** again, then reset before another attempt. The new controller can leave the pole and use other contacts, so a disabled-belt trial under the new mode need not reproduce the original fall. Reset preserves the mode and switch settings.

Turning the belts off holds their prescribed surface speeds at zero while retaining contact and friction. It represents a zero-speed servo comparison; an unpowered, freely rolling belt needs a different actuator model. Switching the balance controller off also removes its motor torque commands and sets belt speeds to zero.

## Apply pushes and follow trajectories

| Interaction | Effect |
| --- | --- |
| W / S or up/down buttons | World +X / −X force: forward/backward |
| A / D or left/right buttons | World −Y / +Y force: transverse left/right |
| Drag up/down in the view | World +X / −X force |
| Drag right/left in the view | World +Y / −Y force |
| Space | Pause or resume |
| R | Reset the selected contact scenario |
| Camera button | Cycle perspective, front, side, and top views |

Keyboard shortcuts apply when focus is outside an input field. A drag scales force with its length, up to the slider setting. Use the direction buttons when you want the full selected force. Directions stay tied to the world axes when the camera changes.

A push is an external force on the fork/platform, measured in newtons and applied for the chosen number of simulation seconds. For example, 0.5 N for 0.10 s supplies 0.05 N s of impulse. It is logged separately from the three motor torques in N m and the two belt surface speeds in m/s. Active pulses add: two simultaneous 0.5 N pushes in the same direction produce 1 N net force, while opposite pushes can cancel. Pause freezes simulation time, including the remaining duration of an active force pulse. The browser requires Run before accepting a new push.

To track a path, reset with zero initial tilt, select **Forward**, **Lateral**, or **Curve**, and set distance and duration. A useful first curve is **40 mm** forward with **20 mm** lateral excursion over **10 s**. Press **Start trajectory**; it resumes the simulation automatically. The controller holds the endpoint after the path ends. **Hold** captures the current position as a stationary reference.

You can apply pushes throughout tracking. The display reports tilt, tracking error, ground-contact forces, and the five actuator commands. The plots compare the actual and target paths and show tilt over time. Try the same path from **Shell** and **Polar belt** orientations: the local controller changes as support moves between a belt and a shell.

The **Transition** scenario starts near the shell/belt boundary with a preset **60 mm** forward, **5 mm** lateral curve over **10 s**. Select it and press **Run** to use that preset. Starting a different trajectory replaces the preset.

The default **Transition aware** mode blends local gains using the actual support loads and the carrier's orientation relative to the floor, distinguishing both hemispheres and both belts. It follows the platform's heading when calculating horizontal errors and wraps cyclic joint-angle errors. The requested world-space position target stays fixed; absolute yaw is unregulated. Rejected gain points remain recorded.

**Prepare incoming drives** uses a short kinematic lookahead to prepare an approaching unloaded shell or belt before it receives load. It coordinates tangential surface velocity at the ground rather than raw RPM, because shell and belt drives have different directions and contact lever arms. Shell preparation uses bounded motor torque with its reaction on the carrier; belt preparation retains the model's ideal surface-speed law. The preview is not a dynamic contact forecast or MPC.

To compare controllers, export the current run, choose **Original local LQR** under **Feedback mode**, reset the same starting case, and repeat the same pushes or path. Incoming-drive preparation is inactive in that baseline mode. Within **Transition aware**, disable **Prepare incoming drives** to compare feedback alone with feedback plus preparation. Controller mode, preparation state, and preparation commands are retained in the saved run.

See [CONTROL_UPGRADE.md](CONTROL_UPGRADE.md) for the controller changes and current integrated-evidence status. Sampled recoveries do not establish stability for every path, push, or orientation. Stronger pushes or disabled actuators can cause a fall, after which the interface requires Reset.

## Save and inspect a run

Press **Export run** to download CSV data and save a portable copy under:

```text
results/live/<run_id>/
  run.csv
  rollout.npz
  configuration.yaml
  events.json
  summary.json
  model/
```

The saved model includes generated MJCF and its meshes. CSV and NPZ data retain actual states, requested and applied actuator commands, target coordinates, and external force pulses. Inputs and forces logged at `time_s` produced that state over the interval beginning at `interval_start_s`. Export before Reset if you want to retain the current run.

The browser's path history covers about 30 s; its tilt chart shows the last 20 s. Run export retains up to 120,000 controller intervals, approximately 10 min at the default 200 Hz. Longer runs retain the most recent samples and report the discarded count.

## Evidence and current limits

The earlier Linux Chromium [browser checks](results/lab_ui/browser_qa.json) exercised keyboard and drag pushes, pause, a visible fall with belts disabled, CSV export, and a 0.5 N push during a 10 s curved trajectory. The shell tracking case finished with about 1.16 mm position error. A separate earlier [polar tracking regression](results/lab_polar_tracking/README.md) followed the 40 mm / 20 mm curve with a lateral push, completed without falling, and finished with 4.39 mm position error and 0.01896° tilt. Its support moved from belt A to shell A. These saved checks describe their recorded controller versions; they do not establish the upgraded controller's performance. Updated integrated comparisons are tracked in [CONTROL_UPGRADE.md](CONTROL_UPGRADE.md). A [desktop screenshot](results/lab_ui/desktop.png) shows the interface.

The model uses provisional masses, inertias, friction, and actuator limits. Belts are ideal moving-surface speed servos with no motor current, load response, rotor inertia, or full belt routing dynamics. The selected analytical contact treatment supports the declared flat horizontal floor. Feedback uses simulator state; sensor estimation and real hardware timing remain future work.

Measured component masses/inertias, final payload properties, exposed belt geometry, and motor/gear limits will improve physical fidelity. They are not required to run the current interface.

If the page reports a rendering error, its data controls and logged physics remain available. Read the reported error and terminal output; the interface does not substitute a prerecorded animation. Headless Linux defaults to MuJoCo's EGL backend. The separate desktop `viewer` command requires a graphical display.
