# I-47 — presentation study guide and saved-log investigation

2026-11-26. **Documentation and saved-data analysis; no new simulation or control change.**

[Download the study guide PDF](control-and-observer-study-guide.pdf).

The user requested a complete explanation of the selected dynamics, LQR,
feedforward, momentum observer, support estimator and active preview, plus an
investigation of saturation, weak drive geometry, contact timing and turning.
The presentation scope excludes camera development and deployment work. The
33-page guide is preparation/backup material; its five-minute talk outline is
on page 29. It includes equations, worked examples, practice questions,
third-party/method citations and a contribution/AI-assistance checklist.

The controller remains gain-scheduled LQR with inferred-contact rolling
references, holding feedforward and 120 ms constant-velocity incoming-drive
preparation. Selected mode is named `hardware` in code, but the experiments
and this guide concern simulation. Body state is supplied from simulation;
solved contacts remain evaluator-only in that selected control path.

## Questions and findings

1. **Why does effort cost not enforce the actuator limit?** The final clip does
   enforce the five numeric input bounds. Input-bound normalization and the
   Riccati effort penalty do not impose online inequalities. Native rolling
   least squares does not activate experimental bounded reference branches.
   At 6.825 s, exact belt-A request 0.8347418461 m/s equals LQR
   0.8438042577 minus rolling feedforward 0.0090624115; applied input is
   0.3000 m/s. Attitude plus angular-rate terms contribute 88.604% of that LQR
   term. The desired body speed is only about 0.0451 m/s. This identifies the
   request's origin at this sample, not a unique physical cause of departure.
2. **Why can geometry lose authority?** A single-shell-B example at roll −20°
   becomes rank-deficient at shaft phase −90°: both instantaneous drive
   velocity columns lie along x. At the same attitude but phase −34.377°,
   the two directions are independent. Another original diagnostic sample at
   31.650 s has condition number 3961; undamped rate requests are about
   +502.66/−1811.23 rad/s. A saved-pose damping calculation reduces them while
   accepting 56.84 mm/s residual. These are rates, not torques, and this is
   distinct from the 6.825 s balance-demand event.
3. **Is computation time causing old contact data?** The saved Gazebo loop is
   blocking and lockstep; simulated physics does not advance while Python
   calculates between finite physics runs. Per-command compute/transport delay
   was not recorded. No workspace ROS-bag artifacts were found; the records
   are NPZ/JSON/CSV/logs. The inferred preceding-interval wrench, filtering,
   current-pose patch bands and solver/state sampling conventions are distinct
   mechanisms. At 6.860 s, point gap is 0.183 mm and band margin 0.0259 mm;
   the saved true equivalent point itself projects outside the ideal current
   band by 0.0782 mm. Support disagreement is confirmed; a unique latency or
   classification cause is not isolated. I-46's seven transfers still give
   0 ms median timing improvement from faster routing.
4. **What do the turn results show?** First Mars turn has 86.46 mm timed
   error, 11.52 mm local path distance, zero geometric overshoot and 1.101 s
   reversal lag. The corresponding flat case has 8.03 mm overshoot and
   0.193 s lag. Masking preparation and shaping lean improve some metrics
   while worsening others. No unique turn fix is established.

## What is implemented versus proposed

The PDF explains the selected existing loop and prior experimental branches.
It does not activate a constrained controller, faster support selector or new
estimator. Constraint-consistent references/priorities and turn diagnostics
are proposed directions. A box-constrained projection alone may reproduce
clipping; bounded joint-rate references alone do not bound the full feedback
command or actual joint speed. A general contact-switching/saturation stability
proof is not claimed.

## Source and review evidence

Read-only audit families:

- [Controller/dynamics/constraints audit](../../../../results/presentation_preparation_20261126/controller_audit/controller_audit.md),
  including saved component decomposition and source-line/hash citations.
- [Observer/contact/timing audit](../../../../results/presentation_preparation_20261126/contact_audit/contact_observer_audit.md),
  including 41 historical samples and seven clock-qualified fresh transfers.
- [Geometry/turning audit](../../../../results/presentation_preparation_20261126/motion_audit/README.md),
  including independently reconstructed matrices and all six turn metrics.

The input/source/archive snapshot precedes the work: 145 source/document paths
and 16 earlier presentation archives. New analysis uses saved arrays and direct
algebra; no controller, observer or physics trajectory was executed. Earlier
results/bundles and runtime files remain protected. The PDF build retains its
source/content and draft failures: a math-renderer syntax error and two
source-footer overflow pages were corrected before delivery. Independent
scientific reviews caught wording errors about the disabled heading helper,
optional frozen-phase design and angle-only versus rate-consistent ablation;
those corrections change the explanation, not old experiment results.

Compact evidence and authoring manifests are preserved under `evidence/`.
Final PDF/text/figure/link/hash checks and independent content/layout reviews
are recorded in the local family. No additional unit-test or simulation count
is claimed for this document-only iteration.


## Delivery checks

The final PDF has 33 correctly bookmarked pages and is 1,150,057 bytes;
SHA-256 `4be727dd3c29942fc10015597cb51c7f63d68b5981096a9f8a395280ae7bfcdf`.
Independent controller, contact and motion content/layout reviews pass. Root
rendered all pages and checked text bounds, page/bookmark order and links.
143 protected pre-existing origins and all 16 older presentation archives
remain unchanged; only the two living research logs were updated. The new
PDF/source/audit bundle retains draft/review corrections separately.

[Compact evidence manifest](evidence/manifest.json)
· [Local preparation bundle](../../../../results/presentation_preparation_20261126/presentation-pack.zip).
