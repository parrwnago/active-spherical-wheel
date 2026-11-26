# Contact observer and timing audit for the presentation

This is a source-and-saved-data audit of I-41, I-42 and I-46. It adds no simulation, controller command, estimator variant or new experiment. The selected baseline is gain-scheduled LQR, momentum-observer support inference, rolling/support feedforward and the existing incoming-drive synchronizer. The code calls this measurement-boundary configuration `mode='hardware'`; these results remain simulated experiments.

**Main finding:** the available data do not establish computation latency as the cause of contact-routing disagreement. The simulator advances in finite blocking batches after the command is computed. Filtering and the preceding-interval measurement convention do introduce memory, while current-pose geometric classification and idealized contact assumptions introduce a different source of disagreement. Removing the additional wrench filter did not pass the registered I-46 improvement screen.

## 1. What the observer actually computes

Let $H(q)$ be the generalized mass matrix, $v$ the nine generalized velocities, $b(q,v)$ the gravity/Coriolis/centrifugal bias, $\tau_p$ the modeled passive forces, $Bu$ the modeled applied joint-motor forces, and $f_e$ the unmodeled external generalized force. The implementation uses

$$H\dot v+b=Bu+\tau_p+f_e,\qquad p=Hv,$$

and therefore the momentum balance

$$\dot p=Bu+\tau_p-b+\dot H v+f_e.$$

At consecutive recorded states $k-1,k$, with $\Delta t=t_k-t_{k-1}$ and the **previously held input** $u_{k-1}$, it approximates the known-force impulse by

$$I_k=\Delta t\left[Bu_{k-1}+\frac{\tau_{p,k-1}+\tau_{p,k}}2-\frac{b_{k-1}+b_k}2\right]
+(H_k-H_{k-1})\frac{v_{k-1}+v_k}2,$$

then forms the interval residual

$$\bar f_{e,k}=\frac{H_kv_k-H_{k-1}v_{k-1}-I_k}{\Delta t}.$$

This uses state differences and modeled forces, not a directly measured acceleration or a solved contact list. The modeled input is the command held over the completed interval, not the newly computed command. Motor-control/force bounds and gains are included in `_torques`; the selected belt model is ideal surface speed. The net residual also includes unmodeled external disturbances; it is not intrinsically labeled as ground contact.

The residual is followed by a first-order filter:

$$r_k=r_{k-1}+\left(1-e^{-K\Delta t}\right)(\bar f_{e,k}-r_{k-1}).$$

Here $K=100\,\mathrm{s}^{-1}$, $\Delta t=5$ ms and the update coefficient is approximately 0.39347. The filter has a 10 ms time constant. **A time constant is not a fixed 10 ms delay:** it is a weighted history of the interval residual. The interval residual itself summarizes the preceding 5 ms and is not instantaneous solved force. At initialization the observer starts from a modeled nonaccelerating balance; the inspected transition is well beyond initialization.

Sources: [observer derivation and input boundary](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:227), [bounded modeled input](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:285), [discrete update](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:330).

## 2. Frames and timestamps

The free-joint translational velocities are world-frame quantities; its angular velocities are body-frame quantities. Gazebo publishes world angular velocity, which `GazeboPlant._pull` rotates into the body frame for the MuJoCo measurement mirror. The generalized momentum calculation uses that convention. Before filtering, the first three external-force rows are already world-frame; the three moment rows are rotated into world coordinates using the sign-consistent midpoint quaternion for the completed interval. The resulting moment is about the wheel-center/free-joint origin. The support locator receives a world force and world center moment.

The control sequence is:

```text
state q_k,v_k and prior applied u_(k-1)
  -> interval momentum residual -> filtered wrench
  -> support locator at CURRENT q_k
  -> LQR + rolling/support feedforward + existing synchronization
  -> bounded new u_k
  -> five completed 1 ms physics steps under that held input
  -> next state q_(k+1),v_(k+1)
```

The support records are therefore a combination of a history-bearing wrench and current configuration. Neither the current point nor its patch label is produced by a full future contact prediction. The additional ground-normal filter is separate: selected metadata specify 20 ms normal filtering and 200 ms rim treatment. These do not directly replace the raw inferred per-patch load shares used for gain/holding routing. The existing synchronizer makes a 120 ms constant-velocity geometry preview and can prepare incoming drives; the 200 ms dynamic forecast and terrain-memory previews remain shadow diagnostics.

Sources: [world-to-body angular velocity](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/gazebo/plant.py:150), [moment frame conversion](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:312), [observer then current-pose locator](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:550), [normal/rim settings](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:197), [active synchronizer](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/synchronization.py:154).

## 3. From a net wrench to a support point and patch

For the ideal single spherical support, the assumed center moment is

$$m_c=-R\,n\times F,\qquad p_c=c-Rn.$$

The code computes

$$n_\perp=-\frac{F\times m_c}{R\|F\|^2},\qquad
n=\sqrt{1-\|n_\perp\|^2}\,\frac F{\|F\|}+n_\perp,$$

with normalization. Very small force returns no normal. If $\|n_\perp\|^2\ge1$, the implementation normalizes the transverse component; this is a bounded construction under an inconsistent/saturated inverse relation, not proof that a physical single-point contact explains the wrench.

Each support surface is an axial band of the known sphere. For patch sign $s\in\{+1,-1\}$ and current patch-axis unit vector $a(q)$, classify the support vector by $x_s=s\,a(q)^T(p_c-c)$. The actual generated XML used by the audit contains:

| Patch family | Signed axial interval |
| --- | --- |
| Shell A/B | 10.000000 mm to 115.65465836 mm |
| Belt A/B | 115.65465836 mm to 120.000000 mm |

At a shared bound, the ordered locator loop returns the first matching patch within its numerical tolerance. Between the shells, the locator constructs the two known rim points. For rim axial coordinates $x_A,x_B$, the modeled share is $w_A=(x-x_B)/(x_A-x_B)$, clipped to $[0,1]$, and $w_B=1-w_A$. These shares multiply the inferred compressive load. Below 5% of model weight the selected locator returns no records.

This rim lever rule is a specific model. In general multiple simultaneous contacts are not uniquely recoverable from a single net force/moment: different point/force sets can produce the same resultant wrench. The current output is an equivalent support construction, not a general multi-contact posterior.

Sources: [intrinsic normal](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:201), [patch metadata](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:406), [band and rim lever rule](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:429), [record loads](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/contact_sensing.py:449).

## 4. How those records affect the present controller

Inferred patch loads weight scheduled LQR gains and route static holding torque. The native least-squares rolling feedforward uses inferred patch identities, contact points and Jacobians; it does **not** load-weight those constraints. Load weighting belongs to the separate bounded/feasible allocator branches. The filtered normal affects the attitude target/support direction; during contact dropout the native feedback can retain its previously held loads/gain. I-46 keeps all these histories and the native trajectory intact and changes only a shadow gain/holding share calculation.

Sources: [gain weights](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/recovery_control.py:272), [holding torque](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:531), [native feedback and held loads](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:768), [rolling feedforward branches](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/experiments.py:79), [I-46 shadow scope](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/support_routing_audit.py:118).

## 5. Computation latency is different from filter memory

`run_offroad` computes the command synchronously, then calls `plant.step`. That step sends the input and requests a finite blocking Gazebo run. Simulated physics does not continue advancing while Python calculates between those finite runs. A slower calculation increases experiment wall time; it does not automatically create a corresponding simulated actuation delay. This statement describes the implemented lockstep execution, not a claim that calculation takes zero real time.

The saved arrays contain simulation times, previous inputs, applied inputs, reconstructed wrench, support records and evaluation contacts. I-46 also saves each 1 ms bridge state/contact set. Run summaries record aggregate `wall_time_s`, and audit wrappers record aggregate `elapsed_s`. There are no per-command CPU-start/finish, send/receipt, deadline-miss or transport-delay arrays in the audited datasets. A workspace-wide file-pattern inventory found no `.bag`, `.db3`, `.mcap` or ROS-bag metadata artifacts. These are NPZ/JSON/CSV/log/video records, not ROS-bag recordings.

Consequently, a CPU/transport latency explanation is **unestablished by these data**. The confirmed timing effects are the completed-interval measurement, explicit filter, finite 5 ms control holds, last-solver versus integrated-state convention, and possible rapid geometric label changes.

Sources: [synchronous loop](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_experiments.py:248), [bounded physics step](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/gazebo/plant.py:173), [blocking server run](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/gazebo/bridge.py:82), [aggregate wall time only](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/gazebo/experiments.py:52), [artifact inventory](artifact_inventory.json).

## 6. Exact historical samples: point closeness does not settle patch identity

The I-46 native-history reconstruction, original I-40 trace and fresh I-42 baseline are byte-identical for the audited state/command/wrench fields. The following are the same historical physical samples, not newly simulated outcomes. Truth is the last completed 1 ms contact record; estimator geometry uses the current integrated pose.

| Time s | Last-solver loaded patch | Filtered route | Interval-mean route | Filtered point gap mm | Nearest ideal-band boundary mm |
| --- | --- | --- | --- | ---: | ---: |
| 6.700 | belt A | belt A | belt A | 0.430 | 0.02788 |
| 6.725 | shell A | belt A | shell A | 0.462 | 0.00536 |
| 6.825 | belt A | belt A | belt A | 0.139 | 0.03924 |
| 6.855 | belt A | belt A | shell A | 0.193 | 0.02101 |
| 6.860 | belt A | shell A | shell A | 0.183 | 0.02590 |
| 6.865 | shell A | shell A | belt A | 0.331 | 0.01462 |

At 6.855 the interval-mean point is closer to the saved equivalent point (0.087 mm versus 0.193 mm), yet its patch label disagrees with the last-solver loaded patch while the filtered label agrees. At 6.860 the native gain matrix changes by 23.8%; belt-A request changes to zero and shell-A request is −0.569 Nm. The estimated point is close, but its shell upper-bound margin is only 25.9 micrometres.

There is a crucial qualification: the **saved true equivalent belt point itself** projects 78.2 micrometres outside the ideal belt band at the current integrated pose at 6.860 s (14.0 micrometres at 6.855). Its solver/contact time, current pose, finite collision representation and ideal locator bands are not identical geometric objects. Thus “support-information disagreement” is the defensible description; this sample alone does not prove instantaneous geometric misclassification or isolate filter lag. No temporal alignment was adjusted to remove the discrepancy.

At 6.825 both routes agree on belt A, but the native belt request is 0.835 m/s against a 0.300 m/s bound. In I-42 supplying true feedback shares removes some routing differences without preventing the 6.860 s departure. Correcting a label is therefore not established as a complete explanation or cure for that excursion.

Evidence: [all 41 samples](historical_samples_6p7_to_6p9.json), [display CSV](historical_display_samples.csv), [I-42 context rows](I42_context_samples.json), [I-42 recorded intervention result](/home/parrwnago/active-spherical-wheel/docs/iterations/2026-11-25/handoff-factorial/README.md:161).

## 7. Seven fresh transitions: “always late” is not supported

These are the seven common stable shell/belt transfers from the registered I-46 comparison, three on the selected natural-Mars corridor and four on flat ground. Stability requires at least 15 ms between first and last sample. Negative offset means estimated routing is **early** relative to the stated truth clock. All values are quantized by the 5 ms controller cadence.

| Case | Transfer / preceding-mean onset s | Filtered offset ms | Interval-mean offset ms | Filtered / mean offset against next-held mean ms |
| --- | --- | ---: | ---: | ---: |
| Mars | shell→belt, 3.050 | 0 | 0 | +5 / +5 |
| Mars | belt→shell, 3.870 | −10 | −5 | −5 / 0 |
| Mars | shell→belt, 11.585 | −5 | −5 | 0 / 0 |
| Flat | shell→belt, 2.890 | −5 | −5 | 0 / 0 |
| Flat | belt→shell, 4.150 | −5 | −5 | 0 / 0 |
| Flat | shell→belt, 10.185 | −5 | −5 | 0 / 0 |
| Flat | belt→shell, 11.330 | −5 | −5 | 0 / 0 |

The next-held target uses the actual native trajectory's following interval as an **evaluation-only** lookup. It is not provided to either estimator and is not the truth of an executed fast controller. Shifting the target clock changes apparent early/late offsets; no universal label delay can be inferred by reading a single convention.

| Routing-mismatch-equivalent time | Mars filtered / mean | Flat filtered / mean |
| --- | ---: | ---: |
| Preceding 5 ms mean truth | 5 / 0 ms | 10 / 10 ms |
| Last 1 ms solver truth | 5 / 0 ms | 10 / 10 ms |
| Actual next-held-interval mean, auxiliary | 5 / 5 ms | 0 / 0 ms |

These durations are sample indicators multiplied by 5 ms, not directly measured durations of wrong actuation during the next hold. The fresh paired median timing improvement is 0 ms; maximum normalized requested command jumps are unchanged. I-46 therefore retains the wrench filter. Historical last-1-ms disagreement improves 90→55 ms, but the first truth-correcting shadow cue at 6.725 follows the first native belt request at its bound at 6.700 and the maximum command jump does not improve. That first bound-hit is an actual requested-command criterion, including the existing synchronization contribution, not a claim that all native LQR components already saturate then.

Evidence: [seven-transition JSON](seven_fresh_transitions.json), [CSV](seven_fresh_transitions.csv), [canonical I-46 report](/home/parrwnago/active-spherical-wheel/results/support_routing_timing_20261126/analysis/stage01/report.json), [independent numerical QA](/home/parrwnago/active-spherical-wheel/results/support_routing_timing_20261126/review/reduction_numeric_stage01/report.json).

## 8. Professor questions and defensible answers

**What is estimated, and what is directly provided?** Exact simulated configuration/velocity and previous applied input are provided. The observer estimates a net external wrench; the locator infers equivalent support records from that wrench and current geometry. Solved contacts are evaluator-only in the selected baseline.

**Is this a predictive contact observer?** Its primary wrench/support output is reactive. The separate active 120 ms synchronizer predicts geometric exposure/gap under constant velocity to prepare incoming drives. It does not forecast actual support forces/load transfer. The longer dynamic/map forecasts remain diagnostic.

**Does a 10 ms time constant mean the actuator is always one or two samples late?** No. The filtered wrench carries history, but current-pose nonlinear classification can switch early or late, and the apparent offset depends on whether truth is last-solver, preceding-mean or following-interval force. The seven fresh transitions demonstrate this distinction.

**Could calculation time cause the observed delay?** Not through unmodeled real-time progression in this lockstep setup: the physics run is finite and blocking. Per-command computation/transport timing was not recorded, so a separate latency contribution is not measured here.

**If the point estimate is within 0.2 mm, why can the selected motor differ?** The relevant band margin can be only 0.026 mm. A close equivalent point can lie on the other side of a hard shell/belt boundary. In addition, current integrated pose and last-solver equivalent contact have a convention mismatch; the true mean point can itself fall slightly outside the ideal current-pose band.

**Does the faster interval residual solve the mismatch?** It changes some historical routing and improves one fresh transition by one 5 ms sample, but fails the registered material-benefit screen. It can also choose a disagreeing patch when the filtered route agrees. Its shadow command is never applied.

**Would perfect contact identity fix the trajectory?** Not demonstrated. I-42's true-share feedback intervention changes routing but does not prevent the same departure; large balance demand is already present when both inferred and solved support agree. Contact uncertainty is relevant, but not established as the sole cause.

**Can this recover general simultaneous contacts?** No. The single equivalent sphere point and special two-rim lever model do not identify an arbitrary contact-force distribution from one resultant wrench. General multi-contact observability remains limited.

## 9. What is confirmed versus still a hypothesis

Confirmed: the implemented discrete observer/frames/input ordering; explicit filter memory; strong current-pose patch-boundary sensitivity; measured support-information disagreements; active short kinematic synchronization; unchanged native driving in the shadow studies; insufficient I-46 benefit to replace the filter.

Still unestablished: CPU/communication latency as the source of these simulated corrections; a single fixed delay explaining all label differences; arbitrary multi-contact identification; contact truth alone resolving the excursion; any closed-loop improvement from the unactuated interval-mean branch.

The JSON audit, source snapshots and input hashes accompany this narrative. No old evidence was changed.
