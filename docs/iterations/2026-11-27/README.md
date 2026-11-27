# Research presentation notes — 2026-11-27

**I-63: observable transition conditions do not rescue the tested load check.**
[Full record](contact-condition-diagnostic/README.md),
[overview PDF](contact-condition-diagnostic/evidence/analysis/plots_stage01/condition_diagnostic.pdf)
and [17 annotated event windows](contact-condition-diagnostic/evidence/analysis/plots_stage01/event_windows.pdf).

- The user's dynamic-threshold idea is tested with one predeclared causal
  condition score: observer interval-versus-filtered change and signed
  shell/belt surface distance/rate, combined by maximum. These are inferred
  uncertainty clues, not independent true-contact sensors.
- Surface arclength replaces an unexecuted axial-distance proposal before
  scoring; a 12 mm axial window would cover the entire 4.35 mm-deep polar cap.
- Every primary state has known risk. All six harmful overrides have high
  score, 0.892–0.959; the worst bad point has r=0.938. Loosening the check in
  those conditions would also protect that mistake.
- The primary affine allowance still forces 99 useful losses to remove the
  worst point. Only six harmful changes can be prevented, giving a lower bound
  of 217 errors versus A's 124. This is a bound, not a new achieved result.
- The registered stop prevents 18-schedule fitting and a new driving episode.
  Independent geometry/bounds and exact NumPy replay agree. Preserve twelve
  feature contracts and fifteen separate evaluator contracts, without calling
  them a controller/full-suite run.
- Live LQR, momentum observer, feedforward and existing preparation remain
  selected. This excludes one rule, not all dynamic adaptation. Matching
  predicted/observed load history is the next proposed investigation.

**I-62: tolerance tuning cannot resolve the load-check tradeoff.**
[Full record](contact-load-tuning/README.md) and
[tradeoff PDF](contact-load-tuning/evidence/analysis/plots_stage02/tolerance_tradeoffs.pdf).

- The user's tuning suggestion is tested with 3,721 shared `(a,b)` pairs on
  the same three saved starts, using whole-start training/evaluation splits.
  Every training fold and the full-data fit have zero qualified settings.
- The best pooled label-only witness changes 124→123 errors and retains all
  294 useful corrections, but removes only one of six harmful changes and
  leaves the worst point discrepancy at 29.84 mm. It is not adopted.
- Removing half the harmful changes costs at least 111 useful corrections on
  the grid (best error count 232). Meeting the native point maximum costs at
  least 99 useful corrections (grid-best error count 221).
- An independent dominance proof extends that 99-loss point requirement to
  any nonnegative shared affine tolerance on these fixed inputs: even removing
  all six harmful changes gives at least 217 errors, versus A's 124.
- These are correlated historical states and previously inspected episodes,
  not unseen validation or new driving evidence. The live LQR, momentum
  observer, feedforward and existing preparation stay selected.
- Preserve the failed numerical benchmark attempt and its summation-layout
  correction, all grid results and 11 inclusive replay/selection contracts.
  Continue investigating the load model's timing/transfer assumptions rather
  than interpreting a smaller residual as guaranteed correct contact.

**I-61: adding joint-load consistency does not improve the guarded estimator.**
[Full record](contact-load-consistency/README.md) and
[comparison figure](contact-load-consistency/evidence/analysis/plots_stage01/overview.png).

- Keep I-60's velocity ranking fixed; use the three extra observer joint rows
  only to veto an accepted contact proposal, falling back to the native estimate.
- Correct comparator: I-60 accepted/fallback has 124 errors, rather than 109
  for all raw rankings. The new veto increases that to 153.
- It removes one of six harmful overrides, but also loses 30 of 294 useful
  corrections. Five of 13 fixed gates fail; the conditional physical trial
  does not run. Worst guarded point discrepancy stays 29.84 mm.
- The illustrative bad shell-B proposal at 35.205 s still passes the load
  tolerance. A useful shell-A correction at 7.875 s fails it instead.
- All 30 lost useful corrections occur near transfers and fail compatibility
  even with the logged dominant-family point. Stable-point compatibility >99%
  therefore does not establish transfer validity. Filtering, timing and force
  distribution are possible contributors; no unique cause is proven.
- Preserve the rejected trial. Keep native LQR, momentum observer, feedforward
  and existing preparation selected; the I-60 velocity signal is still useful
  offline. Next proposed diagnostic: time-consistent load/transfer hypotheses.

Implementation checks pass (45 selected core tests, nine separate evaluator
contracts, independent replay); the scientific candidate fails. The archive
retains fixture/receipt/version-roundoff corrections separately from that result.

**I-60: the user's velocity-consistency idea supplies useful contact evidence.**
[Full record](velocity-contact-ranking/README.md),
[classification figure](velocity-contact-ranking/evidence/analysis/plots_stage02/overview.png)
and [point tradeoff figure](velocity-contact-ranking/evidence/analysis/point_plots_stage01/point_discrepancy.pdf).

Keep the presentation distinction clear:

- Selected control remains native gain-scheduled LQR, momentum observer,
  rolling/holding feedforward and 120 ms kinematic incoming-drive preparation.
- I-60 runs a new estimator **offline** on recorded body/joint velocities and
  prior held belt speeds. It ranks nearby physically valid contacts; desired
  velocities and solver truth are excluded from inference. Its normal search
  area still comes from the observer.
- On 23,423 dominant-support eligible samples from three starts, wrong/unknown
  family labels decrease 412→109, or 1.759%→0.465%. All 12 fixed shadow gates
  pass. These are correlated samples, not independent robot trials.
- Accepted predictions have 18 errors in 6,902 samples. Coverage is 86.60% of
  ambiguous motion-eligible states, **29.47% of all eligible states**. A small
  velocity residual and a large score margin do not prove the contact is right.
- Selected-point RMS improves 3.134→2.672 mm, but maximum discrepancy grows
  26.91→44.04 mm. The extreme failures come from remaining wrong-family choices.
  Average improvement is not sufficient for direct feedforward replacement.
- No new driving run, compensating command or tracking improvement is claimed.
  Next proposed experiment: bounded geometry changes and slip-aware hypotheses
  before controlled integration. The baseline stays selected.

Suggested explanation: “We used measured motion to ask which contact hypothesis
best explains rolling. It reduced contact-family errors offline. It can still
choose an unloaded surface when the real loaded surface does not roll without
slip, so we retained the original controller and documented the outliers.”

Counterexample for questions: at -1 rad and 35.205 s, true/native support is
belt B. A shell-B candidate fits velocity much better and is wrongly accepted.
This motivates explicit uncertainty/slip handling rather than treating a speed
mismatch as a unique inverse for contact.

The source's contact count includes only records above 0.05 N. “One resolved
contact” must not be presented as proof of exactly one physical contact; weaker
records can still contribute to load-weighted means. The correction and original
wording are preserved in the evidence.

Verification: 28 selected core tests (22 new + six existing), seven separate
evaluator contracts, independent arithmetic/kinematics checks, and standalone
NumPy replay. These scopes do not add to a new full-suite count. Sources and
protocol precede scoring; all feature scores precede truth-label evaluation.

[Previous presentation notes — 2026-11-26](../2026-11-26/README.md)
· [Current teammate guide](../../TEAMMATE_HANDOFF.md).
