# I-70 post-result temporal-error diagnostic

This is a separately registered explanatory analysis after I-70's outcomes were
known.  It changes no inference, parameters, acceptance gates, controller or
physics.  Source and nine input files were sealed before its new calculations
in `pre_execution_stage01/source_input_seal.json`.  Four synthetic contracts
passed; these are separate selected diagnostic tests, not the full repo suite.

Directly counting the recorded pressure shares confirms that all 471 loaded
nondominant primary rows have two families at or above 10% share.  The remaining
23,423 loaded primary rows have one such family; none have three or four.  This
describes thresholded sampled positive pressure, not the number of physical
contact points or all force-bearing bodies.

The apparent tradeoff is explicit in the paired decisions:

| Comparison | Singleton decisions gained | Singleton decisions lost | Net error change |
|---|---:|---:|---:|
| A to new frame evidence | 41 | 222 | +181 |
| A to primary HMM | 26 | 466 | +440 |
| Frame evidence to primary HMM | 68 | 327 | +259 |
| No-edge HMM to primary HMM | 86 | 4 | -82 |

The primary HMM correctly recognizes 453 of the 471 mixed-pressure pair rows;
A cannot emit pairs.  Those pair successes offset the loss of 440 dominant-
singleton decisions, leaving only thirteen more correct set decisions overall.
The edge model improves the no-edge HMM by 82 singleton decisions but loses five
correct pair decisions (458 to 453).  It does not recover the baseline result.

Of the original 294 useful I-60 corrections, frame evidence preserves 255 and
the primary HMM preserves 207.  The HMM loses 87, repairs none of the original
six harmful overrides, and repairs none of the eighteen accepted-wrong rows.

Most false pairs come from phase +0.3: 197 of the primary HMM's 201 pair outputs
on dominant-singleton truth, including all 97 high-confidence false pairs.
Together with 53 high-confidence wrong singletons, these explain the 150 errors
that fail the covered-confidence gate.  Unknowns remain separate: 223 unknown
outputs are primary errors; 156 assign at least 0.8 mass to unknown, but unknown
never counts as covered confidence in the registered metric.

Error spells require constant output support, error type, sampled truth family
and truth support, with adjacent 5 ms samples.  Their duration is row count
times 5 ms, not a continuous physical contact-error measurement.  Primary HMM
maximum spell durations are 25 ms for unknown, 45 ms for erroneous pairs and
55 ms for wrong singletons.

Mechanically selected examples, with fixed case-order/earliest-row tie breaks:

- Longest primary erroneous pair: **+0.3, 15.725–15.765 s**, nine samples/45 ms.
  The HMM emits shell A+B while sampled pressure supports shell B.  Frame
  evidence already selects shell B at both ends.  The no-edge HMM persists for
  one additional sample, 50 ms.  This is consistent with temporal retention
  contributing to the error; it does not isolate the transition prior from the
  likelihood and its model assumptions.
- Highest-confidence nonunknown primary error: **+0.9, 31.830 s**.  The HMM
  selects belt B with mass 0.993972 while sampled pressure supports shell B.
  Frame evidence already selects belt B with mass 0.874904.  Temporal inference
  strengthens an existing wrong explanation here.
- Highest-confidence error including unknown: **-1, 36.090 s**.  All three new
  methods emit unknown with mass 1 while pressure supports shell B.  Unknown is
  a failure to identify support, not evidence of no physical contact.

`stage01/report.json` contains all per-case counts, paired decision tables,
confidence/error distributions and mechanically selected examples.
`stage01/spells.json` lists every error spell.
`stage01/*_diagnostic.npz` supplies exact time, masks, truth cardinalities, method
support/confidence/correctness/error type and spell IDs for plotting.  Nothing
in this diagnostic proves a causal tracking benefit or a universally superior
contact-estimation method.
