# I52: what contact-relative scheduling changes

The proposed12-run factorial is a suitable bounded test: the original figure-eight starts at−1.00,+0.30,+0.90 rad, each with baseline, the exact I51 joint-reference guard, whole-run normal-relative phase scheduling, and both. All arms retain the0.30 m/s cap, original flat gain bank, observer, route, yaw45° and44 s duration. No additional flat simulation is needed for this stage.

This tests a **phase-indexing approximation and its interaction with bounded joint references**. It does not create a fully terrain-consistent LQR.

## The exact consistency limits

The selected phase is

```
phi = atan2(−(R_carrier.T @ direction)[0],
               (R_carrier.T @ direction)[2])
```

The flag replaces `direction=worldZ` with the current filtered estimated normal. It does not rebuild the equilibrium or dynamics, transform all state/error coordinates, rotate actuator coordinates, or changeQ/R. The bank was designed by finite differences around static flat-ground operating points over the5 ms held-command interval.

The live errors remain the existing hybrid convention: heading-frame XY position and linear velocity; quaternion position difference; body-frame angular velocity; free shell material positions; and rolling shaft error `e_shaft=e_x/R−e_pitch`. The reference carries actual yaw and uses the estimated slope lean, but its angular-rate target remains zero. Joint references are LQR state targets, not hard velocity servos.

After the support/load/phase gain blend, the entire shaft row is replaced by the current run's ordinary local-design row. That anchor is constant within a run and intentionally differs with starting phase. The normal-relative flag therefore does not even reschedule all five command rows. Gain-bank/configuration hashes should match all12 arms; ordinary-design hashes and initial states should match only **within each four-arm starting-phase group**. Initial feedback gains may differ between baseline and normal-relative arms at time zero.

The phase also discards the carrier-Y component of the normal. With identity carrier rotation, both worldZ and `[0,sin(alpha),cos(alpha)]` produce phase zero despite different cross-grade geometry. A scalar phase cannot encode every slope, gravity direction, yaw, contact position or multi-contact configuration. The helper logs whether the XZ projection is defined; saved pose and normal allow reconstruction of the omitted lateral component.

Even individually stable flat-point gains do not establish stability for load-weighted interpolation, state-dependent phase switching, dropout-held gains and a replaced shaft row on nonflat dynamics. Static lean and holding feedforward compensate an equilibrium condition; they do not reproduce all derivatives of the tilted-gravity/contact dynamics. A complete change of state/input coordinates would require appropriate transformations ofA/B, equilibrium and feedback (schematically `K_original=U*K_terrain*T`), rather than only changing a lookup angle.

These are limitations of the approximation, not proof that any one of them caused a particular old fall.

## Correction to the historical evidence

The recovered report is [the preserved legacy tuning JSON](stage02/preserved/results/offroad/tuning/tuning.json), SHA256`8fe261bf6ba05280018f848566f3ddb33a0c85f2b2bc1ac999e514eab22abb91`.

| Legacy normal-phase flag | Mean reported cost | Falls |
|---|---:|---:|
|False|2.745654738817209|0 /14|
|True|2.6436046468742234|0 /14|

The improvement is3.7168%, below the declared5% adoption margin, soFalse was retained. This was legacy off-road tuning through the MuJoCo pipeline with `terrain_aware` sensing, not the current Gazebo observer-only experiment. It should be called **a sub-margin improvement that was not adopted**, not a failed/falling trial.

A current source comment separately states that normal-relative scheduling increased falls on Mars. The referenced `results/gazebo/mars_yaw_study` directory is absent in this workspace, and quantitative source data/counts for that claim were not recovered. Preserve that statement as an unverified legacy comment rather than conflating it with the recovered tuning result. This does not establish that the old experiment never ran. Current explanatory source snapshots are explicitly not presented as the exact executable source of that older tuning campaign.

The matched I51 late-only switch remains stronger episode-specific evidence: it changed the gain definition after an identical38 s history and improved late position error, while increasing yaw excursion and remaining unsettled. It does not validate whole-run normal-relative scheduling across configurations.

## Measurements that make the factorial falsifiable

Report actual rates against **both** native least-squares targets and the used guarded targets. Also retain native/used target curves, actual rates, guard modifications and the current nominal rolling residual. Lower error to a changed target alone can move the goalpost; reference bounds do not guarantee torque feasibility or better body tracking.

Keep phase effects and guard effects separate at each starting phase. The descriptive interaction is `C_NR−C_N−C_R+C_B`; these deterministic runs are not statistical replicates. Retain all cases, failures, original timed/local trajectory error, absolute/reference lean, settling, all six fixed turns, yaw/body-z rate and actuator limits. Configuration-stratified comparisons must acknowledge that the arms evolve along different state histories.

The original initial-placement code depends on terrain, ordinary design, phase, lean factor and yaw, not these two factors. Require identical physical initial states within each phase group. The fact that exact estimated normal=worldZ makes both phase formulas identical is a useful pure wiring fixture; a new physical flat run would instead introduce a separate question about normal-estimation transients.

## Preserved evidence

- [Source/math report](stage02/report.json) and [historical evidence table](stage02/historical_schedule_evidence.json).
- [Driver/helper source-only review](runner_review_stage03/report.json), with the full diff from I51 and source snapshots. Final protocol/source binding is still required before launch.
- The first report-builder attempt assumed the old phase/yaw fields were under `mars`; the saved schema places them under `route`. The KeyError, original source and prior inputs remain instage01. Stage02 corrects only that schema lookup; no scientific criterion or old result was changed.

No I52 native run, controller/observer replay, model dynamics or saved-state forecast was performed by this audit. Source locations are `offroad_control.py:305,381,513,768`, `recovery_control.py:245`, `control.py:13,47,66`, `sim.py:97`, and `offroad_experiments.py:141` in the preserved current-source tree.
