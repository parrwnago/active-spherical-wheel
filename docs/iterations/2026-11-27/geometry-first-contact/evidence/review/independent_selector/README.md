# Independent I-72 selector audit

The corrected audit passes all three 8,801-frame cases. Every discrete choice,
candidate ID/row, mask, acceptance flag, raw/accepted family, physical point and
native-fallback output matches production exactly. Fixed-score comparisons use
`1e-12 absolute + 1e-12 relative`; the largest observed numerical difference is
2.22e-16. The alternate `math.fsum` distance accumulation changes no choices.

The verifier imports no production selector, frozen ranker, packet decoder or
evaluation module. It independently decodes sparse coordinates, loops through
each original active family, applies exact squared-distance/ID tie rules,
computes all four original scalar scores and acceptance rules, and reconstructs
raw physical points and C/C_VG fallback. Native estimated loads enter only the
separate fallback reconstruction. No truth or baseline comparison is read and
no new geometry or force fitting is performed.

The first attempt is preserved in `pre_execution_stage01`, `stage01/report.json`
and `stage01_run.log`. It found only a diagnostic convention mismatch: the
reference used positive infinity for an unavailable family distance, while
production records NaN. Finite distances and every selection/scoring decision
already passed. Stage 02 changes only that unavailable-distance sentinel and
adds an assertion to an existing contract. Scientific algorithms, thresholds
and production outputs are unchanged.

There are **four unique inclusive contracts**, repeated for the corrected seal;
the two four-test logs must not be added together. They cover geometry/ID ties
and complete-candidate scores, low-excitation/missing-input behavior, exact
confidence-boundary conditions, and native fallback semantics.

`stage02/report.json` contains every field comparison. Independent full,
prefix-101 and partitioned-batch calculations agree; saved production prefixes
match that current-frame reference. The production selector was not imported
or rerun by this verifier. Decoded input arrays and all bound files remain
unchanged; production in-memory immutability has its own separate contracts.

All source versions, both stage logs/reports, pre-read/corrected seals and the
completion receipt are bound by `artifact_manifest.json`.
