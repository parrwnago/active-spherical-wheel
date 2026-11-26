# Targeted support-information probe — I-24–I-25

2026-11-25 · [Iteration log](../../../../ITERATION_LOG.md#i-24--targeted-support-disagreement-audit-and-saved-state-command-probe)
· [Evidence/source manifest](evidence/manifest.json)

**The disagreement affects the command, but correcting contact inputs in the
selected late window does not resolve the excursion.** Four matched Gazebo
runs complete; all fail the existing acceptance gates. No estimator, feedback
law or default setting is changed by this diagnostic study.

## The exact question

The user selected the saved example at 28.975 s: estimated belt A, actual
shell A, despite only .3155 mm equivalent-point error and .0198° normal error.
We investigated its duration and command effect, then tested a short-window
correction. This revisits the **adaptive bounded allocation** figure-eight at
phase -1.00 from W20. Baseline here means that existing experimental setting,
not the repository's global least-squares default.

The original single-contact weak-drive geometry remains a separate mechanism
from shell/belt information transfer. This probe does not establish that either
mechanism alone explains the full trajectory behavior.

## Historical audit: duration and information timing

At 28.975 s, estimated belt-A load is 18.730 N and last-step actual shell-A
load is 21.905 N. The difference appears at one saved sample, with agreement
at 28.970 and 28.980. It occupies a 5 ms sample-held indicator, not proof of
continuously wrong support between samples. Across 28.8–29.45 s, 30/130 loaded
comparisons disagree: 150 ms sample-held, 18 estimated switches, 30 actual
last-step switches. Load-share margins are both 1 at the quoted instant;
these margins are not calibrated uncertainty.

Rows are logged before the next control command is applied. There is no
one-row truth offset. However, **the contact logger reports the last completed
1 ms physics step; the observer reconstructs the preceding 5 ms interval and
filters its wrench with a 10 ms time constant.** Historical data does not
contain every physics substep. A sampled label difference must therefore be
described as support-information disagreement, not automatically geometric
misclassification. The new runs record the five individual physics substeps.

The first sampled disagreement in this transition window is 28.925 s. Lean
error already exceeds 3° at 28.585, 5° at 28.770 and 8° at 28.895. The recorded
burst follows initial lean growth; correcting it tests amplification/recovery.

See the [transition plot](evidence/audit/transition_zoom.png),
[earlier episode context](evidence/audit/episode_context.png) and
[historical audit](evidence/audit/audit.json).

## Fixed-state command calculation

At the quoted instant, the inferred point is about 12.59 micrometres inside
the model's belt band, while the physical point is 16.34 micrometres inside
the shell band. Relabeling the inferred point would put it outside its patch.
We first isolate feedback load routing while retaining its point, wrench,
normal, references and synchronization geometry.

Native feedback is reconstructed using exact archived feedback-law methods
and cached gains on 141 saved states. All reconstruction checks pass; maximum
LQR difference is 1.33e-5 in command units. Replacing only feedback support
shares, preserving estimated total load, gives this counterfactual at 28.975 s:

| Requested command | Native | True-share feedback |
| --- | ---: | ---: |
| Belt A, m/s | -1.7065 | -0.04782 |
| Shell A, N m | -0.06144 | +0.68338 |

The shell request clips at +.6 N m. Correct routing moves demand onto the
loaded shell but does not remove saturation. Across the independent saved
states, requested saturation occurs at 90/141 native versus 86/141 corrected
states. Those commands were **not integrated**; this calculation does not
establish recovery or tracking improvement.

See the [command plot](evidence/shadow/shadow_feedback_routing.png),
[validated shadow report](evidence/shadow/shadow_report.json) and
[reconstruction checks](evidence/shadow/reconstruction_validation.json).

## Four matched Gazebo comparisons

Every case uses the same 44 s Mars figure-eight, initial phase -1.00, exact
simulation-state surrogate, controller parameters, gain library and actuator
bounds. No physical, observer or controller reset occurs during intervention.
All branches retain the original timed path and destination as the evaluator.

Diagnostic commands are computed at **28.900–29.385 s inclusive**; the final
command is held through 29.390 s. Outside this window, native inputs resume.
The physical state and retained controller history naturally carry effects
past the window; they are not restored to the baseline trajectory.

| Branch | Information changed during window |
| --- | --- |
| Baseline | None; existing observer/locator/control |
| Last-step feedback shares | Feedback gain/holding-torque load routing uses latest 1 ms physical shares, scaled to original inferred total load |
| Interval-mean feedback shares | Same limited routing change, using all five preceding 1 ms substep loads |
| Complete contact records | Full valid last-step contact records replace locator output for allocation, feedback, synchronization and slip/rim processing |

The first two interventions retain inferred point/normal, rolling allocation
and synchronization inputs. The full-record intervention uses physical points,
normals, forces/load shares and identities, remapped by actual geom names onto
the contact-free state mirror. It is deliberately broader than identity alone.
The momentum-observer wrench remains native; downstream normal/friction filters
can respond to changed slip/rim inputs. Truth inputs are diagnostic comparisons,
not a proposed input source for an actual robot. Neither physical share source
uses the observer's exponential wrench filter; the five-substep mean is itself
an average and is not identical to filtered wrench inference.

| Full-run metric | Baseline | Last-step feedback | Mean feedback | Complete records |
| --- | ---: | ---: | ---: | ---: |
| Moving RMS path error, mm | 77.08 | 73.54 | 75.11 | 79.52 |
| Peak path error, mm | 317.78 | 311.58 | 340.87 | 322.67 |
| Final goal error, mm | 87.66 | 34.44 | 206.53 | 48.61 |
| Peak reference-relative lean error, ° | 13.91 | 14.48 | 16.76 | 14.56 |
| Completes duration | Yes | Yes | Yes | Yes |
| Passes all existing gates | No | No | No | No |

Show the [four-way comparison](evidence/comparison/comparison.png).
Moving RMS above comes from the run summaries' moving-only tracking metric;
the [window metrics CSV](evidence/comparison/metrics.csv) also reports position
RMS over each entire selected window, including settling where applicable.
Those quantities have different evaluation masks.

Final distance improves in two branches, but peak lean increases in every
branch. Complete contact records also worsen RMS and peak tracking. Endpoint
accuracy alone would conceal these regressions. These are single-case
screening comparisons, not a robustness qualification.

During the intervention itself, peak lean error is 13.91° baseline versus
14.48° / 14.35° / 14.56° for the three corrections. Requested saturation falls
from 90.8% to 81.6% / 83.7% / 85.7%, but immediate peak lean error does not improve.
Window lean RMS decreases slightly in the mean-share and complete-record
branches, so these outcomes include tradeoffs.
The complete record bundle differs from its own native locator by up to
9.15 mm in equivalent point and 95.27 N in normal load: an instantaneous-truth
versus filtered-estimate contrast broader than an identity-label change.

The fresh baseline reproduces historical saved float32 qpos, qvel, requested
and applied commands, estimated loads and goals **bit for bit over all 44 s**.
Exact double-precision warm-prefix equality is checked for the new paired
branches. All four collect 700 physics-substep load samples in the narrow
audit window, with held commands unchanged between substeps. At the baseline
28.975 s sample, the preceding interval mean is also 100% shell A (about
22.02 N), so this particular disagreement persists against interval-mean truth.

The [comparison report](evidence/comparison/report.json) contains scope checks,
first differences, timestamps and source limits. This reproduces recorded
outputs, not every historical source byte: the exact old allocator snapshot
was not recovered, so current-source matched runs are labeled accordingly.

## Videos and presentation

- [Complete contact-record comparison](../../../../results/support_identity_probe_20261125/videos/probe_baseline_vs_contact_records.mp4): baseline versus the broader oracle, with native and routed inputs distinguished.
- [Interval-mean feedback comparison](../../../../results/support_identity_probe_20261125/videos/probe_baseline_vs_interval_mean.mp4): a retained unsuccessful narrower intervention.
- [Historical support timing](../../../../results/support_identity_probe_20261125/videos/historical_information_timing.mp4): slowed original event, with no intervention and no reconstructed interval mean.

These replay saved Gazebo/DART states through MuJoCo rendering; video creation
runs no physics. Metadata, final-frame checks, stills and renderer source are
preserved in the evidence archive. The local
[presentation pack](../../../../results/support_identity_probe_20261125/presentation-pack.zip)
contains these notes, the log, compact evidence/source stages and selected clips.
Earlier packs remain frozen; full NPZs remain in the original local results.

## Verification, source stages and reproduction

Four simulations execute 35,204 controller calls with source/input guards and
intervention/invariance assertions. They are separate from the 141 fixed-state
calculations and are not additional unit-test counts. A first comparison
analysis selected the oracle shadow outside its active window and failed an
assertion. The selection logic was corrected; failed output is preserved,
and no physics rerun was required for that analysis fix.
The final analysis also uses the validated .005 s control period rather than
accumulating float32 timestamp roundoff. Elapsed timestamps, sample-indicator
duration and held-command duration are distinguished; the terminal observation
does not actuate a further interval. The prior analysis stage is preserved.

The first three branches share their exact launch harness. The fourth has a
later launch snapshot for full-record replacement; core controller sources
remain unchanged. Later documentation/metadata clarifications are identified
separately from executed source. See the [manifest](evidence/manifest.json) for
origins, source stages and hashes. Historical source copies use `.py.txt` to
avoid accidental test collection. The reports and source snapshots preserve
the failed results as well as the valid mechanism checks.

Probe NPZs store per-support and equivalent-point/load summaries, record
counts and force differences, not every individual contact-record dictionary
or per-contact normal. Timing-load samples are retained at all five substeps
in the declared audit window; historical data lacks that information.

From the repository root, inside the environment in [GAZEBO.md](../../../../GAZEBO.md),
use a fresh output directory for each study. Existing run folders are refused:

```bash
python scripts/probe-support-identity.py --output results/new_identity_probe --variant baseline
python scripts/probe-support-identity.py --output results/new_identity_probe --variant last_substep
python scripts/probe-support-identity.py --output results/new_identity_probe --variant interval_mean
python scripts/probe-support-identity.py --output results/new_identity_probe --variant contact_records
```

The analysis/render helpers are archived with their original locations. Restore
them to those paths when they derive paths from their own location. Current-
source reruns are new experiments; no general bitwise guarantee is promised.

**Decision:** retain estimator and defaults. The known disagreement matters to
commands, but these short-window corrections do not fix the excursion. Earlier
balance/weak-geometry behavior remains the next control question; this probe
does not validate a new recovery policy or establish that estimation never
contributes elsewhere. No particle filter or handover-smoothing rule is adopted.
