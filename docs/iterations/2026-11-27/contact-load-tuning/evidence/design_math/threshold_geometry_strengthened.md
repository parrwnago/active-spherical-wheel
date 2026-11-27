# I-62 threshold geometry and independent audit plan

Frozen before reading I-62 sweep outcomes. This note uses only already inspected I-61 evidence and the registered I-62 protocol. It reports no new sweep or simulator outcome. Candidate ranking, points, force fit, runtime source and live defaults remain fixed.

## Acceptance regions

For proposal i, let L_i=R_i ||F_i|| (N m) and m_i=max_j |delta_ij| over the three extra joint residuals. Pass exactly when the existing validity flag is true and m_i <= a+b L_i. Invalid features do not pass. The offline grid is a,b=0,.005,...,.300: 61 by 61, or 3,721 points. At (0,0), only an exactly zero valid residual passes. Direct comparison avoids division by zero; the runtime helper's a>0 requirement is unchanged. Use the registered arithmetic order a+(b*R)*F.

Loosening either parameter only enlarges the pass set. Both useful and harmful overrides can increase; total family errors are not monotone. At fixed b, acceptance changes at a=m_i-b L_i. Every metric uses its fixed primary or finite-point mask, never a parameter-dependent subset.

## Known-pair limitation

Prior source: I-61 stage01 science.json, m100_B fixed events.

At 7.875 s, a useful correction had maximum absolute residual 0.3005718241059226 N m, original threshold 0.26593807596244046 N m, L_good=2.3593807596244046 N m, and original maximum normalized residual 1.1302323784141899. The original gate rejected it.

At 35.205 s, a harmful correction had maximum absolute residual 0.1727067408402148 N m, original threshold 0.25105652663277817 N m, L_bad=2.2105652663277817 N m, and original maximum normalized residual 0.6879197412494835. The original gate accepted it.

For nonnegative a,b and positive denominator,

    (a+b L_good)/(a+b L_bad) <= L_good/L_bad = 1.0673201083738355.

Accepting this good point while rejecting this bad point would require that ratio to exceed m_good/m_bad=1.740359540361. Thus no shared nonnegative affine tolerance can separate this pair in the desired direction. Zero denominator also cannot accept the nonzero good residual. Uniform scaling of the original tolerance cannot separate them either. This is pair-specific: it does not prove that aggregate tuning cannot improve, or that richer features cannot separate the cases.

## Counts constrain the available tradeoff

On a fixed mask, E_C=E_N-U_pass+H_pass, where U_pass and H_pass are useful and harmful A corrections kept by the new gate. Equivalently,

    E_C-E_A = useful_lost - harmful_removed.

I-61 had E_N=412, U=294, H=6 and E_A=124. Pooled nonregression therefore permits at most six lost useful corrections. Even if all six harmful points are removed, useful retention must be at least 288/294=97.9591836734694%. The nominal 90% useful-retention gate alone is weaker. Report integer counts and episodes; six correlated harmful states do not justify confident population-level claims.

## Point metrics and mandatory peak rejection

C chooses each row's full A proposal (family and point) if passed, otherwise N. Same-family point changes matter. On a common finite point mask,

    SSE_C = SSE_N + sum_pass(d_A^2-d_N^2).

Percentiles need the actual selected per-row distances. Maximum cannot be approximated as max(global native maximum, maximum accepted proposal): accepting a proposal can replace the native peak row.

Let M_N be the native maximum on the particular training/evaluation mask. Any accepted A point with d_A>M_N+1e-12 mm is unsafe for the maximum gate and must be rejected. Every such row k imposes a+b L_k<m_k (strict since passing is inclusive). Training unsafe sets use training data only. The raw unguarded I-60 point maximum must not substitute for policy A's maximum.

A useful dominance bound follows: if useful row i has m_i>=m_k and m_i*L_k>=m_k*L_i for any mandatory-rejected unsafe row k, accepting i necessarily accepts k. To see this, minimize a+bL_k subject to a+bL_i>=m_i, a,b>=0: for L_i>0 the minimum is m_i*min(1,L_k/L_i); for L_i=0 it is m_i. The product criterion handles both without division. Thus i must be rejected by every nonnegative shared affine threshold satisfying that maximum-point gate. The union gives a lower bound on useful losses; count each row once. The simpler sufficient case L_i<=L_k and m_i>=m_k is included, but the stronger condition also captures the known pair above. The union bound need not be tight when several unsafe rows constrain a useful row jointly.

More generally define U(b)=min_k(m_k-b L_k). Maximum feasibility requires 0<=a<U(b), b>=0. For the registered grid, independently enumerate this mandatory-peak-feasible set and find its exact minimum useful loss. That is a grid-specific lower bound, not impossibility for other features or thresholds. No new numerical bound is reported before release.

## Registered selection and validation

The authoritative contract is ../protocol/protocol.json. Fit on two whole starts and apply once to the third. Training qualification requires >=50% harmful removal, >=90% useful retention, pooled errors<=A, each phase error-rate excess<=.001 and harmful count<=A, pooled point RMS/p95<=A, point maximum<=N (1e-12 mm allowance), and at least one retained proposal. Among qualified candidates choose, in order: fewest errors, smallest point SSE, fewest vetoes, smallest integer a index, then b index. No qualified candidate means explicit off=A with failed selection. Pooled OOF assessment uses the same gates and requires all three training fits to qualify. Oracle geometry remains diagnostic only.

All three historical starts were previously inspected. This is retrospective internal cross-validation, not an unseen holdout. Three fold-specific settings assess a tuning procedure, not a single validated deployment setting. A full-data refit is descriptive training evidence. Offline contact metrics do not demonstrate trajectory improvement.

## Independent audit after release

Verify fixed masks and A parity, zero/original gate boundaries, count identity, actual mixed point metrics, fold selection without validation leakage, explicit failed-fold fallback, pooled OOF gates, descriptive Pareto tradeoffs, and mandatory-peak useful-loss bounds. Bind source/result hashes. Keep failure evidence and avoid claims beyond the registered grid unless a separate continuous proof supports them.
