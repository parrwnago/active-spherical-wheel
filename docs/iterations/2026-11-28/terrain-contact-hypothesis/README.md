# I-79 — terrain change is plausible; wrong preview is not yet demonstrated

2026-11-28. Completed existing-evidence/code review. **No new simulation,
controller change or candidate acceptance.** Original native N remains selected.

The hypothesis is: a sudden terrain change invalidates the local-plane contact
preview, the wrong drive is prepared, and that preparation contributes to the
departure. This chain is plausible but has not been verified for the selected
baseline episodes. Three mechanisms need separate evidence:

| Mechanism | What it means | What the current evidence establishes |
| --- | --- | --- |
| Wrong future preview | The surface expected to approach support over the next 120 ms differs from what actually loads. | Possible with unseen terrain or changing motion; a matched prediction/outcome ledger is missing. |
| Current-estimate timing/error | Current support/normal estimate differs from the physical contact information relevant to that command. | Interval averaging and filters have temporal effects; instantaneous pressure and filtered wrench are different quantities. |
| Difficult response to real change | Contact genuinely changes, is observed reasonably, and moves the balance target faster than the robot follows. | Supported in the selected figure-eight precursor; this does not establish the entire departure cause. |

## What the baseline actually predicts

[Synchronization preview](../../../../src/spherical_wheel/synchronization.py#L154)
extrapolates current body/joint velocity for 120 ms against one frozen estimated
plane. It does not integrate future dynamics or observe unseen terrain. A robot
on flat ground can therefore prepare for continued flat support just before
encountering a slope. That is a possible **future-preview error**.

[Native control](../../../../src/spherical_wheel/offroad_control.py#L692) uses this
preview for incoming-drive preparation. It does not use a predicted candidate
as proof that the candidate currently carries load. The momentum observer and
support locator provide current inferred support separately. Their completed
5 ms interval, wrench filter and subsequent normal filter can lag changes;
lockstep simulation does not demonstrate wall-clock calculation delay.

## What the existing runs say

[I-77 aligned physical-contact evidence](../../2026-11-27/baseline-deviation-cause/README.md)
shows rapid physical normal changes in the eight's 96–97.5 s precursor. The
observer's direction agrees with the same-filter physical interval-wrench
direction within 0.02345° maximum. This diagnostic filter starts from the native
filtered wrench at 96 s; it is not a new independent initial-condition oracle.

The **subsequently filtered controller normal** differs from current pressure
normal by up to 6.076°. At 96.400 s the gap is 5.025°, desired-up rate is
84.315°/s, and actual-up rate is about 0.46°/s. The selected rows still carry
shell-B pressure. A shell/belt transfer is not necessary for this disturbance.
These facts allow a real contact change plus filtering and a demanding balance
response; they do not prove false contact identity or a preview failure.

The straight event differs. At 20.585 s, estimated/pressure families agree and
point discrepancy is only 0.140 mm. Substituting pressure geometry still gives
condition 383.97 and large opposing references. Wrong point estimation alone
does not remove that weak configuration.

Nor do changing solver normals alone identify a sudden terrain slope: contact
selection, aggregation and wheel geometry also matter. Actual contact positions
and terrain-facet identities/normals must distinguish those explanations. The
saved nearest-centre DEM tangent is only a proxy.

## Missing evidence and proposed diagnostic

The I-76/I-78 precision logger saves preparation weights/deltas and current
estimates/truth, but omits the full original preview targets: selected horizons,
future margins/gaps and prepared speeds. I-77's fine contact trace does not add
those past predictions. Therefore these saved runs cannot directly score the
original preview at time t against contact at t+horizon.

**Proposed, not run:** one short unchanged-native replay around the selected
precursor, saving the original preview anchor, plane, motion, targets and
horizons, then comparing with subsequent physical contacts. Keep observer,
filtered normal, lean target and applied commands alongside them. Only if wrong
preparation precedes the departure should a separate matched-prefix
preparation-only intervention test causality. Both returned and stored target
blend weights must be neutralized because `after_step` also uses preparation.
No camera, estimator replacement, broad tuning or new governor is justified by
this review alone.

## Evidence preservation

[Extracted existing values](evidence/report.json) ·
[input/source binding](evidence/registered_before_extraction.json) ·
[exact extraction source](evidence/review.py) · [manifest](evidence/manifest.json).
Ten inputs remain unchanged; five fixed timestamps are copied from the prior
I-77 report. This is a qualified retrospective extraction, not a new independent
raw-physics reconstruction, new simulation trial or test-suite result.

Local reproduction uses
`results/terrain_contact_hypothesis_20261128/review.py`; it deliberately refuses
to overwrite its closed output directory. Restore the captured source and use
a new stage for another extraction. Existing I-77/I-78 evidence is unchanged.
