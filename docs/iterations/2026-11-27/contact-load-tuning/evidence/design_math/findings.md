# I-62 independent threshold-capability findings

This is a post-release interpretation of the frozen I-61 replay and registered I-62 grid. No simulator or live-controller outcome is added.

The joint-load tolerance is tunable, but this particular shared affine gate cannot meet the registered requirements on these recorded states. The strongest limitation is a feature-ordering conflict, rather than a missing grid resolution.

The only A point exceeding the native pooled maximum occurs at start -1.0 rad, 35.205 s: A has 29.842858743 mm equivalent-point error, native at that row has 15.155053906 mm, and the native pooled maximum is 26.907642232 mm. Meeting the maximum-point requirement therefore mandates rejecting this proposal.

For a valid proposal, write m=max(abs(extra joint residual)) and L=R||F||. A shared gate accepts when m<=a+bL, a,b>=0. If a useful row i satisfies m_i>=m_bad and m_i L_bad>=m_bad L_i, accepting it necessarily accepts the mandatory-rejected bad row. The original-input audit finds 99 useful corrections with this property. Exact rational signs confirm all 99 with strictly positive margins: minimum absolute residual margin 0.008504580 N m and minimum affine separating margin 0.006025550 N m. The point violation itself is 2.935216511 mm above the native maximum.

There are only six harmful A corrections to remove. Since errors_C=errors_A+useful_lost-harmful_removed, satisfying the maximum-point gate imposes at least 124+99-6=217 label errors for any nonnegative shared affine tolerance on the stored features. This continuous lower bound is separate from the tighter finite-grid result: among 296 peak-safe registered grid points, the fewest label errors is 221. Also requiring at least 50% harmful removal gives at least 232 errors. These statements do not imply an impossibility for richer features, another contact estimator, another dataset, or a controller.

The independent audit also rules out qualifying grid fits in every training fold using only the point-maximum, harmful-removal and family-error constraints:

| Excluded start | Training A errors | Fewest errors with peak bound and >=50% harmful removal |
| --- | ---: | ---: |
| -1.0 rad | 110 | 223 |
| +0.3 rad | 60 | 92 |
| +0.9 rad | 78 | 149 |

Conditional feasible counts, minima and full useful-loss/harmful-retention frontiers match the canonical stage02 grid exactly. The independent verification does not rely on its SSE/p95 arithmetic.

Evidence: `bounds_audit.json`, `strict_margin_certificate.json` (99 witness rows), and `grid_bounds_crosscheck.json`. `audit_bounds.py` was frozen before I-62 arithmetic. The strict-sign supplement was written after the initial bound was seen and is explicitly post-release; its first output is retained, and its byte-identical confirmatory replay followed a separate source receipt. No first-run preregistration is claimed for that supplement.

The useful next research question is how to distinguish the wrong low-residual proposal from correct higher-residual proposals. These results do not establish why their residuals differ. Keeping the baseline and recording the failed tolerance tuning is justified; silently relaxing the gates or installing a threshold is not supported by this comparison.
