# Research presentation notes — 2026-11-27

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
