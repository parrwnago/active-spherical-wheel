# Independent I-71 recurrence audit

The corrected audit passes all three 8,801-frame cases and all 27 prediction
fields per case. The maximum probability differences from production are
5.55e-16, 6.66e-16 and 6.66e-16, below the declared 1e-12 allowance.
Support/current/previous decisions, physical points, candidate IDs and selected
edge indices match exactly. No contact truth, baseline decisions or references
are read.

This implementation derives the nine allowed modes directly from the fixed
protocol numbers. It multiplies probabilities and likelihoods in probability
space, with a common numerical likelihood shift. It does not import the
production restricted filter, frozen helper or inference runner. It independently
constructs the conditioned prior, original diagonal retention, redistributed
off-diagonal affinities, node/edge observations and support-first decisions.

The first attempt (`stage01_run.log`) failed while formatting a boolean
`history_reset` comparison: NumPy does not support boolean subtraction. Its
source, three graph-contract results and pre-read input/output seal are retained
under `pre_execution_stage01`. It stopped before writing an audit report.

The stage-02 correction changes only that comparison formatter: boolean
differences use an exact logical comparison. Probability recurrence, graph,
parameters and scientific inputs are unchanged. One focused boolean-comparison
contract was added. The corrected suite has **four inclusive contracts**, not
three plus four. `pre_execution_stage02` preserves the corrected source, test
log and seal before the corrected audit. It accurately records that the failed
first attempt had already read the first case's predictions.

`stage02/report.json` contains every field's numerical or exact-comparison
result. Independent full/prefix-101 recurrences are identical, and the saved
production first 101 rows match that causal reference. This does not claim that
production prefix code was separately imported or executed.

`artifact_manifest.json` binds the source snapshots, both run logs, seals,
corrected report and completion receipt. No filter, input packet, old artifact
or scientific default was modified by this audit.
