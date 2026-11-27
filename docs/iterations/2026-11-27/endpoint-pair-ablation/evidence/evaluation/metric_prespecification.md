# I-71 evaluation plan before restricted inference outcomes

This is a stopping ablation on the same saved I-58 episodes.  No new physics,
contact-force fits, controller edits, calibration labels in inference or tuning.

`reference_i70/evaluate_temporal.py` is a byte-identical copy of the frozen I-70
evaluator, SHA256 `33993c7fc892d71a4530666090af957cb77614543b03fe3b4316bfdc8b351008`.
All original scientific metrics and sixteen adoption gates are reused through
its pure functions.  Primary/loaded truth domains, confidence, timing and point
matching are unchanged.  The frozen twenty-one metric contracts are reference
evidence, not new I-71 test counts.

The new projection reads only the frozen truth/N/F/A whitelist from I-70's
evaluation packets after all three I-71 predictions are sealed.  It reads none
of I-70's emission/HMM point fields.  Every new emission, no-edge and primary-edge
point field is copied from I-71 predictions, with an exact clock check.

Prediction probabilities retain the twenty-one-mode external API, with all
endpoint pair modes (indices 8 through 19) padded with exact zero.  The original
four singleton rolling/slipping pairs and unknown remain at their original
indices.  A known emitted support has at most one finite endpoint-family point.
Two-contact force fits for transitions remain an inference-side interval model;
this evaluator never converts them into an endpoint pair prediction.

All 471 historical mixed-pressure rows remain in the full loaded-set metric.
The restricted estimator deliberately cannot predict those pair support sets;
it receives errors rather than a changed evaluation denominator.  Pressure-label
truth remains sampled final-1ms positive pressure every 5 ms, not full force
routing or direct slipping measurements.

The separate continuation decision has eight fixed checks:

- Primary restricted-edge pooled dominant-singleton errors strictly fewer than
  its matching restricted memory-free frame control.
- No error-count regression against that frame control in each of the three
  individual episodes, using identical primary denominators.
- Missed pressure transitions, reverted attempts and false support changes each
  no greater than the matching frame control, using the unchanged I-70 timing
  definitions.
- Pooled absolute transition latency p95 at most 50 ms.  Missing latency is
  unscorable and fails this check.

Unknown rows, pair rows, confident coverage and confident errors are retained in
the continuation context.  Unknown does not remove a primary error or provide
confident coverage.  Continuation does not relax or replace any adoption gate.
Failure stops this HMM version.  Passing continuation would be a diagnostic
result only; this experiment makes no live-controller change.

The new wrapper also reports all old I-70 versus restricted I-71 singleton and
loaded-support class counts and asserts that frozen N/F/A singleton counts and
truth-domain counts remain identical.  Nine new synthetic contracts verify
projection, strict continuation logic, per-phase/timing failures, unknown
reporting, restricted API and exact metric-source reuse.
