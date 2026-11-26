# I50: three separate deviation events in the saved I49 runs

This is a read-only audit of the three completed I49 Gazebo runs. No controller replay, forecast, new physics, or parameter change was performed. Windows and peak-selection rules were recorded before each saved-array reduction. The primary windows were 6.4–7.4, 17–19, 30–34, 31.8–32.1, and 39–44 seconds. All three caps were retained.

The latest comparison movie has a two-second intro and plays motion at twice real time. Playback **18.0 s, frame360, is recorded32.0 s**. Recorded18 s is also included as the alternative interpretation.

## What is established

| Event | 0.30 baseline | 0.60 / 0.90 runs | Interpretation |
|---|---|---|---|
| First sustained20 mm local departure | 6.860 s | 6.850 s | The extra belt capacity did not remove the first departure. |
| Motion around playback18 s | Extreme rolling-rate references and alternating torque clipping around31.65 s, followed by shell/belt handoffs | No joint-reference magnitude above5 rad/s in30–34 s; actual motion and later handoff still change rapidly | A common visual moment contains different states and support conditions across arms. |
| Largest local path excursion |190.901 mm at43.985 s |108.537 mm at39.630 s | These maxima are spatial departures, not just timed phase lag. Neither maximum hits the artificial ±2 s projection boundary. |

The local distance uses the same corrected I41 polyline projection restricted to a reference branch within±2 s of nominal time. Time-aligned trajectory error is reported separately. Some other raised-cap late samples do hit this projection boundary; the raw report marks them, rather than calling all such distances pure cross-track errors. The0.60 and0.90 body trajectories coincide, although their applied belt-A speeds differ at nine held samples in the earlier event.

## The31.650 s command spike has a clear algorithmic origin

The baseline observer and saved solver loads both select shell B. At31.650 s, the two active planar rolling directions are only **0.0578° apart**, with scaled condition number **3850.51**. The allocation scales5 rad/s and0.30 m/s are numerical preferences, not hardware limits. The analytically reconstructed current-pose matrix maps the saved drive references back to the requested planar velocity with a residual below4e−13 m/s: this verifies the allocation calculation, not its dynamic feasibility.

The resulting joint-rate references are **[488.601,0,−1760.525] rad/s**, while actual joint rates are approximately[−0.079,−4.813,−1.690] rad/s. Their joint-velocity LQR contribution is **[111.695,−5.529,−141.539] N m**. The complete request is[111.389,−5.365,−141.067] N m and is clipped to[1.5,−0.6,−0.6] N m. Direct rolling feedforward and synchronization deltas are both zero at this sample. Joint-position feedback contributes only0.0565 N m at the shaft.

Five milliseconds later, the shaft request reverses by−120.139 N m and shell-B request by+150.690 N m. The exact symmetric LQR jump split attributes approximately[−120.140,+150.638] N m on these two drives to the changed error vector, versus[0,+0.05394] N m to the changed gain. Joint-velocity feedback dominates the changed error contribution. Large reference spikes already occur earlier in the30–34 s window;31.650 s is its maximum, not a claim that all disturbance starts then.

This identifies how the extreme command is produced. It does not prove that the31.650 s burst caused all later route error, or that every observed twitch has this cause. The raised-cap runs have different configurations at this time and do not encounter this near-singular allocation in the selected window.

## The large terminal baseline drift starts before its late handoff

The continuous late run containing the baseline maximum crosses20 mm at38.640 s,50 mm at40.625 s,100 mm at42.110 s, and150 mm at43.725 s. At38.640 and40.625 s, both observer and solver support are shell B, the current rolling geometry is well conditioned, and the torques are not capped.

| Recorded time | Local error | Actual planar velocity, m/s | Desired planar velocity, m/s | Relative lean |
|---|---:|---|---|---:|
|38.640 s|20.02 mm|[−0.0040,+0.0218]|[+0.00654,+0.00653]|0.046°|
|40.625 s|50.08 mm|[+0.00645,+0.04060]|[+0.000183,+0.000183]|1.485°|
|41.000 s|78.54 mm|[−0.00256,+0.06057]|[0,0]|0.870°|
|42.110 s|100.11 mm|[+0.05964,+0.03261]|[0,0]|0.734°|

At41 s, shell-B position and linear-velocity feedback contribute+0.19093 and+0.58766 N m. Attitude, angular-velocity and joint-velocity feedback contribute−0.36497,−0.29001 and−0.13953 N m; support feedforward contributes+0.04617 N m. Their net is only+0.03025 N m. This cancellation is an exact description of ordinary coupled feedback arithmetic, not evidence that any one term is erroneous. The controller is attempting a balance/motion compromise while the actual robot continues to move after the desired trajectory stops.

The first significant belt-B endpoint in this late window is42.915 s, after the error already exceeds100 mm. Noisy shell/belt routing and belt-B requests above0.30 m/s follow; belt B exceeds its requested cap for0.605 s in39–44 s. Those effects can aggravate recovery but cannot explain the earlier onset. At43.865 s the belt-B request reaches0.97014 m/s while applied speed is0.30 m/s.

The raised-cap maximum at39.630 s also occurs on accurately identified shell B with uncapped commands, moderate rolling condition number1.91 and relative lean1.283°. Accurate support identity and a well-conditioned instantaneous allocation therefore do not guarantee good closed-loop trajectory tracking. They do not validate the effective normal, wrench model, local equilibrium or feedback tradeoff.

## What remains unresolved

The logs identify the origin of a major command spike and distinguish late drift from a later handoff. They do not identify one exact physical cause for the terminal closed-loop drift. A replay of this single native history cannot independently vary support, reference balance, dynamics or command limits.

Saved contact truth is the last1 ms solver sample at5 ms logging endpoints, not the average over the whole hold. The margin diagnostics use per-patch or global load-weighted mean points, not individual solver points. Logged DEM normal is a geometric terrain-reference proxy, not the force-weighted collision normal. No per-command acquisition, CPU, send/receipt or actuator-response timestamps were recorded, so an unmeasured computation delay cannot be inferred.

The separate synchronization audit reconstructs the handoff interface. In the raised-cap run at32.170 s, belt-B preparation weight switches from1 to0 as inferred support crosses its cutoff; the command changes from approximately−0.0121 to−0.3034 m/s. This is a different, directly observable command discontinuity. Its effect requires an intervention to establish causality. The full native preview target/gap/lookahead metadata were not saved.

## Evidence and reproduction

- [Compact numerical report](compact_stage03/summary.json), [event table](compact_stage03/event_table.csv).
- [Whole-route timed/local errors](plots_stage01/whole_route_errors.png).
- [Rolling-reference and torque spike](plots_stage01/baseline_rate_and_torque_spike.png).
- [Late drift, speed, support and belt cap](plots_stage01/late_drift_context.png).
- Primary all-window rows and detailed report remain local understage01; focused rate/onset results understage02_focused.
- Every stage preserves source and input hashes before arithmetic and verifies unchanged inputs afterward. The compact-source syntax error was preserved underdevelopment_syntax_failure and corrected before that stage ran; it produced no scientific outcome.

Each script creates a fresh fixed stage directory and refuses to overwrite it:

```bash
.venv/bin/python results/belt_handoff_event_20261126/deviation_audit/analyze_deviation.py
.venv/bin/python results/belt_handoff_event_20261126/deviation_audit/focus_chronology.py
.venv/bin/python results/belt_handoff_event_20261126/deviation_audit/summarize_events.py
.venv/bin/python results/belt_handoff_event_20261126/deviation_audit/plot_events.py
```

The reproduction target must be a new family/copy if the completed stage directories already exist. No completed stage should be removed to rerun it.
