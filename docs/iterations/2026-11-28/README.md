# Research presentation notes — 2026-11-28

**I-77 returns to the original baseline and tests causes before another design.**
[Full investigation](../2026-11-27/baseline-deviation-cause/README.md) ·
[straight causal comparison](../2026-11-27/baseline-deviation-cause/evidence/component_audit/causal_publication_stage02/causal_comparison.png) ·
[aligned contact evidence](../2026-11-27/baseline-deviation-cause/evidence/contact_trace/analysis_stage01/aligned_contacts.png).

- Original gain-scheduled LQR, momentum observer, inferred contacts,
  rolling/holding feedforward, preparation and final caps remain selected.
  Heading helpers and prior estimator/allocator candidates stay experimental.
- Straight −1 has nearly parallel drive directions (0.389°), not simply the
  wrong contact label. The recorded pressure point still yields a badly
  conditioned inverse. Three 24 s exact-prefix diagnostics show a 250 ms
  feedback-reference intervention reduces later geometric peak from 148.934
  to 21.979 mm over 20.4–24 s. Removing the maximum 5 ms pulse alone gives
  150.665 mm. At the 20.650 s primary endpoint the cluster treatment is slightly
  worse. Preserve both the delayed benefit and the immediate adverse result.
- Forty-three reference/applied rows change in the cluster treatment. The
  ±5 rad/s diagnostic uses the old normalization scale; it is not a motor
  rating, a globally tested controller or a new default. Native24 and both
  intervention prefixes receive independent all-field verification.
- Eight −1's late near-path crossing retains velocity error. Trace farther back:
  exported pressure-weighted solver normals change rapidly, the observer closely reconstructs
  signed interval wrenches, and the normal-derived target moves rapidly while
  angular-rate reference remains zero. Estimated normal, contact radial
  direction, surface normal and nearest-centre DEM tangent must stay distinct.
- The unchanged eight replay captures 1,500 substeps over 300 holds and matches
  all 79 recorded precision fields through 97.5 s, with its explicitly shortened
  terminal held-command mask. Filtered physical-wrench direction vs observer:
  max 0.02345°, RMS 0.00974°. This is a qualified window-specific result.
- Preserve failed interpreter-path test setup, velocity-report bookkeeping,
  reference-attribution equilibrium anchor, and prephysics horizon/source
  corrections. Three scoped hook functions are counted once; the full repository
  suite is not rerun. No full-route acceptance is claimed.

The final attitude hold/release gives no immediate path improvement, but reduces
later geometric peak over 96.4–103 s from 316.714 to 62.191 mm. Release adds a
0.491 N m shaft-command step, so its effect is not isolated from holding. All
79 pre-intervention fields and all 20,601 feedback-error/LQR rows verify.
Saved original-native, fixed held and current branch-local attitude targets
must be evaluated separately: larger saved-target error alone is not a balance
failure once trajectories differ. No new controller is accepted. All five
targeted physical diagnostics finish their requested shortened horizons.
