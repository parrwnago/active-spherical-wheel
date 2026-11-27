# Contact-count terminology correction

Independent source review identified an overstatement in the initial point-check
interpretation. The field `truth_patch_contact_counts` is **not the number of
all physical solver contact records**.

In the exact archived I-58 logging helper
[truth_contact_row](count_definition_source/run-lqr-contact-forecast.py), lines
142–162, every matching record contributes its nonnegative normal force to
`loads` and its force-weighted position to `sums` (lines 150–152). The count
increments only when `force > .05` N (line 153). Patch means subsequently divide
the accumulated position sums by the accumulated loads (lines 157–159).

Consequently, `truth_patch_contact_counts.sum(axis=1) == 1` means **one resolved
wheel contact record above 0.05 N**, with possible additional weaker records.
Those weaker records can still contribute to the logged load-weighted targets.
The test does not establish one physical contact or rule out multi-contact
averaging. The 44.04 mm maximum remains present in this one-resolved-contact
subset, but that fact cannot exclude averaging as part of its explanation.

Frozen `stage01/` source, protocol, reports, arrays, masks and descriptive
statistics remain unchanged. The legacy field/subset name
`strict_one_physical_point` must be read with this correction; it is retained
for reproducibility and compatibility, not as an accurate physical assertion.
The original parent README is preserved as
`README_STAGE01_BEFORE_COUNT_ERRATUM.md`, and its replacement uses the corrected
term. Original RMS/maximum values, primary family-classification gates and
controller/default decisions are unaffected.

`count_erratum_binding.json` records source hashes and verifies that the
historical I-58 helper equals the current tracked helper. This is a documentation
correction after source review, with no mathematical rerun, threshold change,
new physical experiment or alternate interpretation selected to improve scores.
