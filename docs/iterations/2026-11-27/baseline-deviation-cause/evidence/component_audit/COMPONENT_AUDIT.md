# I-77 original-baseline component audit

Started 2026-11-27; causal follow-up 2026-11-28 EST. This audits the original I-76 **N** controller, not the trajectory-facing heading variant. No production source is changed. Source and input metadata were registered in `registered_before_source_analysis.json` before analysis; the selected three-arm physics protocol is separately frozen in `causal_branches_stage01/protocol.json`.

## What actually runs

The nine N case provenances bind the same hardware-style controller settings and flat gain-bank bytes. The simulator supplies exact body/joint state and the previous actual clipped input to a contact-free mirror. During command calculation the original I-76 guards prohibit physical contact/terrain reads. These are simulation-state measurements, not hardware evidence.

| Order | Active component and behavior | Why it matters |
| --- | --- | --- |
| 1 | Momentum observer updates once from current q/v and the previous actually held motor input. Its generalized-force residual and filtered wrench describe the completed interval. | The previous 5 ms interval and observer filtering differ from instantaneous current geometry. Block stepping does not model wall-clock computation delay. |
| 2 | SupportLocator combines that wrench with current known wheel geometry. Records determine contact family/point and inferred loads; low-load records disappear. | A small equivalent-point error does not guarantee correct drive identity. Geometry and wrench have different temporal meanings. |
| 3 | Normal/load estimation and rim handling filter the sensed result. | The normal can lag a change even when contact geometry is close. Friction/slip diagnostics do not constrain the command. |
| 4 | Native rolling least squares computes three joint-rate references and two belt surface-speed feedforward commands for desired XY velocity. If stationary, these are zero; a moving contact dropout can hold the last references/feedforward. | Baseline LS is unbounded and undamped. Its `[5,5,5,.3,.3]` scales are normalization preferences, not hard motor limits. Near-dependent drive directions can create very large joint-rate targets. |
| 5 | Ground-relative attitude reference, native gain routing, static support torque, and LQR feedback form `u_eq + u_roll + u_support - K e`. | Large joint-rate targets enter velocity error and therefore feedback torque. The scheduled shell/belt rows depend on estimated support; shaft row remains the ordinary design row. |
| 6 | Incoming-drive preparation predicts constant-qvel geometry for 120 ms against a local filtered plane and blends selected drive commands. | This is a short kinematic contact preview, not a terrain/contact dynamics forecast. It aims for zero tangential velocity of the incoming surface, not equality of outgoing/incoming motor angular speeds. |
| 7 | Native N heading helper contributes zero; unload fade can blend joint torques toward reference-rate damping. | Free yaw does not remove angular-rate feedback. Fade still uses the original raw rolling references. |
| 8 | Final component-wise clipping applies `[1.5,.6,.6]` Nm joint torque and `[.3,.3]` m/s prescribed belt surface-speed bounds. | The LQR soft effort penalty does not anticipate this clipping. The last two channels are speed commands, not motor torques. |
| 9 | Physics advances 5 ms, then material joint references integrate raw rolling rates and preparation targets. | A recorded final-time command is emitted but not held after termination. Endpoints must distinguish state before the new command from its subsequent effect. |

Source anchors in the registered bytes: `src/spherical_wheel/contact_sensing.py` (`MomentumObserver`, `SupportLocator`); `experiments.py:79` (rolling feedforward); `offroad_control.py:522` (attitude), `:547` (command ordering), `:768` (terrain feedback), `:873` (fade), and `synchronization.py:154` (preview).

## Active settings and dormant alternatives

N uses `mode=hardware`, `rolling_allocation=least_squares`, synchronization enabled, intrinsic-normal observer filtering at 100 rad/s, normal/load time constants 0.02 s, rim time constant 0.2 s, minimum support load 5% weight, gap margin 2 mm, and rim hold speed 0.02 m/s. It uses ground-aware lean with gain1 and a 28 degree cap, support feedforward1, `shaft_error=rolling`, and `track_contact_point=False`. Phase scheduling uses world vertical (`schedule_on_ground_normal=False`). The fixed gain bank is `generated/offroad/flat/control_schedule.npz`, SHA256 `45ab9990aa3961d1202533acf51d5d8a9294eefe5a4c7e6cfde6aa0b3ac462c1`.

The feasible/bounded rolling allocator, motion-reference relaxation, and shaped tilt/rate reference are **inactive**. Listed 20 rad/s / 100 rad/s² rolling-reference parameters do not bound the native least-squares output. Preparation has its own finite target ceilings; final physical command caps still apply. Heading reference is `none` and heading gain/budget are zero.

## Less obvious reference details

* The controller sets the height target to current height and adjusts vertical reference velocity from the estimated plane. It rotates XY errors into current heading and carries current yaw in the target quaternion.
* Shell joint position errors are explicitly zeroed. With `shaft_error=rolling`, shaft position error is overwritten by `heading-relative x error / R - pitch error`. Thus integrated nominal shaft position drift is not directly a baseline position-feedback explanation.
* Normal and desired acceleration change the desired attitude, but baseline angular-rate reference remains zero. There is no dynamically consistent derivative of that quaternion reference and no inverse-dynamics acceleration-torque feedforward. Desired acceleration affects lean. A finite attitude lag is plausible; this source fact alone does not establish the cause of a particular departure.
* Native rolling references solve with zero desired body angular rate. Actual free-yaw/body rotation or slip can violate that nominal rolling assumption. Angular-rate feedback remains present even though N applies no heading hold.
* Least squares attempts to match desired velocity. It does not deliberately relax the timed XY trajectory in this baseline. At a stationary endpoint the rolling solve is bypassed entirely. An endpoint offset therefore cannot simply be described as the bounded allocator choosing trajectory deviation.
* Finite-gain feedback, derived shaft-position coupling, contact/model/support-normal bias, command limits, and preparation can all affect tracking. There is no integral zero-offset guarantee. Whether one dominates requires the event ledger and causal test, not its presence in the code.

## Narrow causal experiment

`causal_branch.py` imports the original I-76 runner and replays the full physical prefix from reset in a fresh process. It does not restore only q/v into an unknown solver state. It truncates the unchanged 45 s straight reference at 24 s for diagnosis and explicitly labels that limited horizon.

The post-native-feedback hook calls the native function once with original raw references, preserving its caches. It copies only the returned feedback reference/error and changes qvel reference6:9 / error15:18. Active command recomputation uses the same equilibrium + raw rolling feedforward + native support - native K @ used error expression. Inactive/identical rows return original objects/bytes. The observer, estimated geometry/normal, gain routing, support feedforward, preparation, unload fade, and final clipping algorithms remain native; their later values may change through actual physical motion.

Registered arms: native24; a single row4117/20.585 s hold of saved row4116 feedback joint-rate references; and a fixed half-open20.400–20.650 s window clipping only feedback joint-rate references to ±5 rad/s. The latter is an explanatory finite alternative at the native normalization scale, not a proposed motor rating or tuned controller. It is not claimed to repair the underlying geometry.

The armed cluster window begins at row4080, but its first actual reference change is row4083/20.415 s. The runtime gate checks state, previous input, nominal history, and native feedback **through the first actual changed-reference row**, and complete commands strictly before it. Native24 must reproduce all shared precision fields through24 s before either intervention is run; the explicit exception is the truncated run's final unheld-command mask. Current-state native-policy counterfactual deltas are distinguished from the historical native trajectory after physical branches diverge. Requested and actually applied changes are logged separately, since clipping can suppress an intervention.

The primary state endpoint row4130/20.650 s precedes application of the released native command. Historical selected-error peak22.635 s and24 s are additional endpoints; later effects include release transients. This is causal diagnosis, not candidate acceptance or a broader trajectory campaign.

## Further isolated APIs, only if the event evidence justifies them

The original runner permits result-owned monkeypatches without production edits. A rolling-geometry diagnostic can replace only the records passed to `rolling_feedforward_records`, while retaining native observer/normal/gains/support. Such a truth-assisted intervention needs an explicit current-vs-interval timing and contact-load definition; pressure geometry is not automatically a perfect observer oracle. A preparation diagnostic should preserve the native private preview/history but zero both returned blend weights and target blend weights, since `after_step` also reads targets. A gain-routing diagnostic must preserve the pinned ordinary shaft row and avoid accidentally changing support/load caches. None of these additional interventions is launched by this audit.

## Setup and verification history

The original three scoped synthetic hook contracts passed in `runner_contracts_stage01`. That source was preserved before strengthening the first-actual-change prefix gate and adding native all-field verification. `runner_contracts_stage02` records a Python-path setup failure: resolving the virtual-environment interpreter symlink bypassed its package path. No physics/case arrays were read by that test. With the exact virtual-environment path retained, `runner_contracts_stage03` passes the same three scoped functions against the corrected bytes. These are scoped runner contracts, not the full repository suite.

Experiment outcomes live with the registered arm outputs and the parent's independent precision verification. This source audit does not turn a plausible mechanism into a confirmed physical cause.
