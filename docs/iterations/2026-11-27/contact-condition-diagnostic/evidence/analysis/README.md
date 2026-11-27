# I-63 condition diagnostic: stopped before adaptive fitting

The frozen combined risk does not resolve I-62's threshold conflict. The exact-sign audit still finds **99 useful corrections** that must be rejected if the worst accepted point is rejected, for the whole nonnegative affine class `a + b*R*norm(F) + c*r`. Only six harmful corrections are available to remove. Consequently pooled label errors must be at least `124 + 99 - 6 = 217` if the point maximum is restored to native's level. This is a necessary bound on these fixed recorded inputs and this score-specific threshold class, not an impossibility for all dynamic estimators.

The registered stop condition fired. **No 18-schedule fit, out-of-fold tuning, new episode, controller change, or physics run occurred.** The c=0 class was already excluded by I-62; the new result concerns the registered positive-c combined-risk extension. The prior I-62 grid minimum of 221 is not an I-63 candidate or new result.

All 23,423 primary rows, including 6,902 accepted proposals, have known risk. Thus this outcome is not caused by unknown coverage. The sole unsafe point at -1 rad, 35.205 s has `u=0.51934`, `s=r=0.93760`. The same 99 prior witnesses match by case/time and split 4/68/27 across starts -1/+0.3/+0.9 rad. Their median risk is 0.62673. The minimum exact-sign risk-product margin is 0.01574246 N m, comfortably above rounding boundaries. Minimum residual and force-product margins remain 0.00850458 N m and 0.01331987 N m².

High risk also occurs in useful corrections: the useful group median is 0.97080. Descriptive useful-versus-harmful pair comparisons favor higher useful combined risk in 66.84% of pairs, but this aggregate comparison does not remove the necessary-condition conflict. Six harmful frames form six single-frame clusters under exact 5 ms adjacency; these are not six independent robot trials. All runs were already inspected previously.

`stage01/science.json` retains complete group summaries, frame/event tables, inherited control checks, all exact-sign witnesses and the stopping decision. Per-case diagnostic-frame NPZs retain the pooled I-62 99-witness tags; the tags are not recomputed from a different per-phase maximum. Event traces use the registered windows and causal features; subsequent observations appear only as plotting context. `i62_metrics.py` is byte-identical to I-62's canonical scorer.

## Verification

The analyzer's **15 synthetic contracts** passed before historical arithmetic. They cover unknown-risk precedence, c=0 parity, preserved policy-force inputs, exact-sign dominance including equality, prior-witness identity, selection/fallback and event grouping. This is a scoped replay suite, not the repository controller suite.

`replay_producer_stage01/comparison.json` records a successful full replay under NumPy 1.26.4: all 108 regenerated feature arrays and all six analysis output NPZs are byte-identical; scientific JSON has no numeric or structural differences. `replay_source_receipt.json` predates this replay helper's first execution. The unexecuted earlier helper with a superseded packet-layout assumption remains under `replay_design_before_execution/`; it produced no replay outcome.

The replay uses original cached kinematic axes and observer primitives. It does not regenerate robot kinematics, run MuJoCo/ROS/Gazebo, replace scientific values, or act on the controller. The helper currently requires strict feature-array byte parity and reports a retained failure before analysis if it differs; a different NumPy platform must not silently substitute canonical arrays. Root's archive-isolated system-NumPy check is separate from the producer result recorded here.

From a standalone I-63 evidence package root containing `analysis/`, `features/`, `portable_inputs/` and `protocol/`:

```sh
python3 analysis/replay_condition_analysis.py \
  --analysis-source analysis \
  --protocol protocol/protocol.json \
  --data-source portable_inputs/i62_base \
  --feature-source features/stage01 \
  --feature-helper features/feature_packets.py \
  --old-witnesses portable_inputs/i62_old_witnesses.json \
  --reference analysis/stage01 \
  --output /tmp/i63-full-replay
```

The output path must not already exist. Receipts bind the frozen source, protocol, feature manifests and canonical outputs. A successful replay validates reproducibility of this offline analysis, not improved trajectory tracking.
