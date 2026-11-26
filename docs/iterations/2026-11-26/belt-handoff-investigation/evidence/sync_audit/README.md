# I50 synchronization and handoff audit

The saved evidence contains two distinct control problems. The baseline has a nearly singular rolling allocation during repeated shellB/beltB support alternation near31.65s. The raised-cap arms later switch abruptly between incoming-belt preparation and loaded LQR/rolling commands while support alternates. The existing preview is active; this is not evidence that the controller never prepares for contact.

Video playback18s is recorded32s (frame360). Literal recorded18s is playback11s and has no belt preparation in the fixed17–19s audit window.

## What “synchronized” currently means

The preview advances the current configuration using fixed measured velocity for up to120ms over the current estimated local plane. It does not predict new terrain, contact forces, impacts or friction, and does not run a dynamic observer forecast. [Source](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/synchronization.py:180).

It calculates tangential world contact-point velocity from the full Jacobian, including base translation/rotation, common shaft and loaded shell motion. Belts add their carrier-localY surface velocity rotated into world coordinates. Hemisphere angular velocity is not compared directly to belt metres/second; the point Jacobian supplies the effective lever and direction. [Velocity calculation](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/synchronization.py:93).

For a belt, with tangent projectionP, material velocityv, and projected belt axisa, the target is `u*=−(a·v)/(a·a+1e−8)`, clipped to the operating belt bound. This minimizes the modeled incoming point's world tangential velocity toward zero/no-slip. A shell solves the analogous angular-rate target through its point Jacobian, then applies a proportional torque target, capped by torque bounds. Shell rate20rad/s and blend0.35 are numerical settings, not measured motor ratings. [Belt target](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/synchronization.py:224) · [Shell target](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/synchronization.py:235).

The load-weighted outgoing velocity is calculated for diagnostic fields but is not subtracted in the target equations. That is a design distinction, not automatically a code bug: copying an already slipping outgoing surface could preserve slip.

Preparation requires approaching geometry, a predicted support gap≤20mm and an activation margin within12mm. The weight falls with inferred load fraction and becomes zero at5% of the inferred total normal load. It can become zero in one5ms interval when the locator changes support from one patch to another. The loaded LQR/feedforward branch then controls that drive. There is no explicit constraint requiring the incoming preparation command and the newly active loaded command to meet continuously. [Gate](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/synchronization.py:200) · [Loaded branch](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:768).

## Baseline allocation failure during alternating handoff

At31.645s the estimated support is beltB, with shaft reference0.28754rad/s and beltB reference0.056972m/s. At recorded31.650s it switches to shellB; both truth and observer select shellB and all synchronization weights/deltas are zero. An independent Jacobian reconstruction at the exact saved equivalent contact point gives the horizontal matrix for common-shaft and shellB rates:

```
D = [[−0.00444453838, −0.00120050496],
     [−0.11536135011, −0.03199871713]]
```

Its drive directions differ by only0.057775°. Using the native5rad/s preference scales, singular values are0.5990274 and0.000155571m/s, condition3850.51. The desired velocity can be matched algebraically with shaft+488.60137 and shellB−1760.52505rad/s. Independent reconstruction agrees with the saved rates within4.63e−10rad/s; its velocity residual is8.16e−16m/s. Those are references, not achieved speeds.

The resulting raw torques are `[+111.389,−5.365,−141.067]Nm`, clipped to `[+1.5,−0.6,−0.6]Nm`. At31.655s the rate references reverse to−30.413/+110.401rad/s. The raised arms at the same31.650s are well conditioned (1.163;85.498° directions), with rates+0.49847/−0.34446rad/s. Their largest raw torque/jump within30–34s occurs later and is not the same nearly singular matrix. [Native rolling matrix](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/experiments.py:79) · [Exact reconstruction](stage05_rolling_geometry/report.json).

This verifies the older nearly coincident-drive-direction mechanism during an already alternating shellB/beltB handoff in the current baseline. It occurs before the32.17s raised-arm mode change, but it is not a failure before all contact transfers. The two capacity histories are different, so these are distinct observed events, not a controlled temporal causal separation.

## Raised-arm preparation-to-loaded handoff

For both0.60/0.90 arms, beltB preparation is already active at31.670s. At32.000s its blend is0.757 and its recovered bounded target is−0.009438m/s. The current virtual belt-point mismatch to the current estimated load-weighted support velocity is3.53mm/s. At32.160s the blend is1 and mismatch is7.58mm/s. These are modeled current-point residuals, not a claim that true physical slip or the unlogged future prediction is known exactly.

| Recorded time | Observed sequence |
|---|---|
|32.165s|Physical beltB load18.484N; observer still shellB. Preparation command−0.012112m/s. Truth-contact kinematic slip diagnostic0.0000935m/s.|
|32.170s|Observer changes to beltB; preparation weight1→0. Command changes−0.012112→−0.303361m/s. New LQR contribution−0.319806 plus rolling+0.016445m/s.|
|32.175s|After that5ms hold, solver beltB load22.104N and truth-contact kinematic slip diagnostic0.131466m/s.|
|32.185s|Physical support is shellB again; observer still beltB. Belt command−0.451307m/s.|
|32.190s|Observer changes to shellB; belt command−0.451307→0m/s.|

At32.170s the loaded command is inside the0.60/0.90 cap, so final clipping does not explain its0.291249m/s step. Across31.5–32.5s, the baseline crosses the inferred5% beltB threshold32times; each raised arm crosses9times. This describes support/control chatter, not an independently proven sensor-latency cause. The transition command delta is saved before final clipping; scalar preparation targets can be recovered as `pre_sync + delta/weight` where the weight is meaningful. [Blend and logging](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:691) · [Final total-command clip](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/offroad_control.py:721) · [Recovered targets and crossings](stage03_current_kinematics/report.json).

A zero mismatch to the “outgoing” average after a single patch becomes the only estimated support is tautological: that patch equals its own weighted average. It is not evidence of successful physical speed matching.

## Conclusion and remaining gap

The audit establishes both an allocation singularity during the baseline’s repeated support alternation and a later discontinuous preparation/loaded-control handoff in the raised arms. Existing anticipation is limited, but absent anticipation is not the observed story. A useful future test would separate smooth transfer of the total drive command from improved contact prediction; no such change or new trial was made here. Direct matching to outgoing slip is not adopted or demonstrated as a fix.

Full future target points, chosen lookahead times, predicted gaps/margins and predicted residuals were not logged by I49. Actual1ms contact history inside each5ms hold is also absent. The saved evidence therefore supports the sequence above, not a claim that one mechanism explains every path deviation or that contact timing alone is the cause.

## Preservation and validation

All arithmetic sources and input hashes were captured before each stage. Only private current-pose kinematics/Jacobians were used; dynamics, integration and solved-contact query functions were guarded. No controller/observer/predictor or preview was replayed, and no existing source/default was edited. A guard-setup error and a later syntax error stopped before their respective row processing; both original sources/logs are retained before fresh corrected stages. An independent source review found no material frame, column or target-recovery issue.

[Aggregate status](report.json) · [Full5ms windows](stage01/report.json) · [Current kinematics](stage03_current_kinematics/report.json) · [Rolling matrix](stage05_rolling_geometry/report.json).
