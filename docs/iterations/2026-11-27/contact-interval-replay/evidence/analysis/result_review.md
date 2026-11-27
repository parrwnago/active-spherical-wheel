# I-68 findings from the unchanged native replay

The momentum observer is tracking the recorded contact impulses closely in this
run. The main discrepancy is between a preceding 5 ms load history and a
current, single-family point-force interpretation. The new raw records also
expose force-bearing shell rows that normal-pressure truth summaries omit.
This changes the interpretation of the old 35.205 s example substantially.

The replay completed all 8,800 holds / 44,000 physics steps. The parent campaign
compared all 59 shared precision fields byte for byte with I58; this analyzer
independently checked its five old/new comparison fields exactly. Every step
had known tags and finite force-map inputs. Rejected rows contributed zero force
in the analyzed primary window; no commanded external wrench was present.

## Matched intervals resolve most of the apparent load disagreement

These are three-joint vector RMS errors in Nm, against the raw, unfiltered
observer `interval_mean`. They are descriptive consistency comparisons, not a
calibrated accuracy certificate.

| Fixed group | Five-step reference, pre-pose | Last-step reference, pre-pose | Five-step reference, post-pose |
|---|---:|---:|---:|
| 1 <= t < 41 s, 8,000 rows | 0.0000866 | 0.116254 | 0.001641 |
| 32 <= t < 36 s, 800 rows | 0.0001227 | 0.100878 | 0.001332 |

Both pose controls satisfy the registered comparison in every fixed group,
including the good-case window. Force and base-moment agreement improve too:
primary pre-pose RMS is 0.00473 N and 0.00102 Nm, compared with 4.645 N and
0.2223 Nm using only the last solve. The small remaining difference includes
the mirror/dynamics integration approximation; the bridge exports point forces
and does not independently expose every possible generalized constraint load.

## Two different events

At **32.110 s**, the last solve has positive normal load only on belt B, but the
preceding interval includes shell B and belt B. The observer's shell-B component
is `+0.0161928 Nm`; the integrated reported shell-B force map gives
`+0.0162282 Nm`, while the last solve alone gives zero. The apparent torque on
an endpoint-inactive joint is therefore largely explained by resolved contact
history here. This is a time-window interpretation issue; it is not evidence
of computation latency.

At **35.205 s**, positive normal pressure is assigned only to belt B throughout
the interval. Nevertheless, admitted shell-B rows carry nonzero reported force
with essentially zero force projected on their published normal. Two examples
in the fixed interval are:

| Completed step | Shell-B force, world N | F dot published normal |
|---|---|---:|
| 35.203 s | (-12.9989, 23.0230, -2.24596) | 0 |
| 35.205 s | (-4.99814, 11.63965, -1.17042) | 0 |

Their mapped shell-B interval load is `-0.2066057 Nm`, closely matching the
observer's `-0.2066269 Nm`. The old endpoint normal-load label “belt B only” did
not mean that only belt B had a force-bearing solver row. Geometry replacement
within a belt family cannot reproduce a shell joint load; the full recorded
force set can. This supports revisiting the load/reference interpretation
before replacing the momentum observer or fitting another point-selection
weight.

The zero-normal/nonzero-tangent rows require a separate contact-model/reporting
audit. They must not yet be labeled a proved friction-law violation, solver
failure, or reporting bug. Friction discretization, solver behavior and the
published force/normal pairing must be checked. Their close generalized-load
agreement makes them material to the observer comparison.

## Rolling and normal evidence

At 32.110 s, pressure-weighted slip RMS drops from 0.0150 m/s at the pre-solve
state to 0.000281 m/s at the post-solve state. At the good 7.875 s control it
drops from 0.0409 to 0.000329 m/s. These pre/post differences warn against
calling all pre-solve contact approach velocity persistent sliding.

At 35.205 s it remains large: 0.1326 m/s before and 0.1392 m/s after, with a
post-solve maximum of 0.1596 m/s among positive-normal-load contacts. The same
kind of elevated post-solve mismatch appears at 35.055 s. Thus a strict rolling
velocity likelihood has a concrete limitation in this later episode. These
kinematic diagnostics use the actual solver points/normals and held belt speed;
they do not use the inferred point. Zero-normal-force contacts are not weighted
into the pressure-weighted slip aggregate, which is another explicit coverage
limit.

At 35.205 s the filtered intrinsic-normal hint is 7.24 degrees from both the
interval pressure-mean normal and the endpoint pressure-mean normal; the live
estimated normal differs by 12.71 degrees at the endpoint. At 7.875 s the
intrinsic discrepancy is 1.05 degrees from the interval pressure-mean normal
and 1.08 degrees from the endpoint pressure-mean normal. An
equivalent-contact normal derived from net force/moment need not equal every
physical surface normal under multiple force-bearing contacts. These angles do
not establish a universal normal-prior bias or calibrate a new uncertainty.

The good control also contains the interval effect: its shell-B observer load
`+0.0784592 Nm` matches the integrated `+0.0784503 Nm`, while the last solve has
zero shell-B contribution. Consequently, contact history alone does not prove
that a particular trajectory correction was harmful.

## Result and scope

Keep the live LQR, momentum observer and feedforward unchanged. This run does
not demonstrate improved tracking. It establishes that the observer/load
comparison should use matched intervals and all force-bearing rows, and that
the later episode also has substantial rolling mismatch. A narrowly registered
audit of the zero-normal/tangential rows is more defensible than another weight
sweep. The existing force-map result cannot uniquely identify the underlying
contact-solver cause.

Evidence: `stage02/report.json`, `stage02/{interval_frames,substep_frames}.npz`
and `stage02/artifact_manifest.json`. The original lazy-NPZ CLI was interrupted
before output because it repeatedly decompressed arrays; `stage01_run.log`
preserves that attempt. `replay_cached.py` eagerly loads the inputs and calls
the unchanged, pre-data-sealed scientific arithmetic. No extra rollout was
needed for that execution correction.

## Separately registered post-hoc row audit

After seeing the fixed-event anomaly, the parent registered the numerical
screen `abs(F dot n) <= 1e-10*max(1,||F||)` and
`||F-(F dot n)n|| > 1e-8 N`. Its source and input hashes were sealed before the
full-row count. This is a zero-normal/nonzero-tangent detection rule, not a
friction acceptance threshold.

The screen finds 1,119 admitted rows in 290 of 8,000 primary intervals, out of
182,062 raw rows. In 32--36 s it finds 29 rows in 13 of 800 intervals. None of
the selected rows is rejected. The largest selected absolute normal projection
is 8.88e-16 N; the largest tangent force is 38.0405 N. Published normal lengths
deviate from one by at most 1.11e-16, so a non-unit published normal does not
explain this numerical pattern.

The fixed 32.110 s interval has no selected row. The 35.055 s interval has one
shell-B row, contributing -0.255359 Nm to its interval-mean shell-B load; the
35.205 s interval has two, contributing -0.206606 Nm. The good 7.875 s interval
also has two selected shell-A rows. Therefore, presence of this pattern alone
does not explain poor tracking or separate useful from harmful estimates.

The most focused next step is a source-and-convention audit of how the installed
DART/gz contact solver constructs friction impulses and publishes the paired
normal/force rows, using these frozen records. Check whether the tangent-only
forces are the intended multi-point friction approximation, an inadequately
resolved solve, or a reporting association issue before changing the contact
estimator to accommodate them. If the source cannot resolve it, propose a small
isolated, registered contact test; do not silently launch another campaign.
The recorded post-solve rolling mismatch remains a separate model limitation
regardless of which reporting/solver explanation is established.

Post-hoc evidence: `posthoc_zero_normal_stage01/{report.json,frames.npz}` and
`posthoc_zero_normal_source_seal.json`. The initial result review is preserved
as `result_review_before_posthoc_counts.md`.

Narrative clarification: the version immediately before distinguishing the
good-case interval and endpoint normal angles is preserved as
`result_review_before_normal_clarification.md`, with its prior receipt. No
numerical source, output or experiment changed.
