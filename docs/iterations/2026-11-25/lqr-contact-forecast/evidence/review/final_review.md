# I-40 bounded final review

Reviewed the authoritative `analysis/final_report.json`, its corrected warning
summary stage, protocol, per-anchor incoming-event records and campaign outputs.
This review performs saved-data arithmetic and reads only; no physics, geometry,
forecast rollout, controller execution or unit-test suite is added.

No material publication blocker was found in the requested scope:

- The denominator is 113 forecasts: 96 uniform and 17 event-conditioned,
  producing 339 nested 50/100/200 ms endpoints. These are not independent runs.
- All eight actual runs completed. Independently recomputed final-second gates
  agree: four of seven controlled cases pass (hold, flat X, out-and-back,
  triangle); three fail. The Mars case fails its unchanged existing route gates.
  The two gate types remain separate rather than being presented as one 4/8
  controller qualification.
- Independently summed corrected incoming-warning TP/FP/FN agree with the
  per-anchor records. Pooled dynamic is 10/0/19 versus kinematic 16/19/13;
  uniform is 3/0/3 versus 2/18/4; event-conditioned is 7/0/16 versus 14/1/9.
  Actual-positive denominators are respectively 29, 6 and 23 for both methods.
  The original generator-consumption error and original stage are retained.
- Independent unique-event coverage arithmetic agrees: 62 actual incoming-patch
  events in the traces, 15 covered by all forecast windows, with seven dynamic
  and nine kinematic patch warnings. Uniform windows cover six, with three/two
  warned; event-conditioned windows cover 11, with five/eight warned. Overlapping
  windows are deduplicated by case, patch and actual onset time.
- These warning counts are a patch-warning/opportunity proxy, not a one-to-one
  timing-matched physical-event detector or independent physical-event recall.
  Kinematic exposure warnings are not predicted contact force distributions.
- Twelve of 113 calibration fits are outside the existing load-fit tolerance:
  five uniform and seven event-conditioned. All fits remain bracketed; do not
  describe those twelve as unbracketed. The failed-fit stratum remains included.
- The nominated `flat_y_transfer` case has no qualifying transfer; the report
  retains that coverage limitation. Late X is explicitly a separate stress
  illustration. Its isolation refers to stable 15 ms dominant-family changes,
  not absence of all raw contact/force chatter or a clean physical handover.
- The new slope-X mesh extent/tessellation differs from historical I-32, so
  no exact historical trajectory reproduction is claimed. Fresh forecast
  histories are their own actual reference and reconstruct exactly.
- Live LQR is unchanged. The report explicitly makes no controller-adoption,
  trajectory-improvement, general multi-contact or real-time-readiness claim.
  Its runtime summary excludes reconstruction/construction/serialization and
  records concurrent-worker contention.

The final report preserves endpoint/trajectory results from the original
analysis, uses corrected warning arithmetic, and labels the added runtime and
calibration strata. No later case or anchor was substituted to improve a result.
