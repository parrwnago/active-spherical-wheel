# Research iteration records

For controller, contact-estimation, and simulation experiment work, update
`ITERATION_LOG.md` as part of the task. The user needs to present the development
process, including unsuccessful trials; retain the hypothesis and learning even
when a candidate is not adopted.

- Give material iterations stable IDs. Record the question, change, comparison
  conditions, evidence, outcome, limitation, and resulting decision.
- Distinguish proposed work from implemented work and completed experiments.
- Record regressions, rejected candidates, setup errors, and their corrections.
  Do not replace an earlier result with a later, better run.
- Identify the simulator, controller mode, starting conditions, parameter values,
  and baseline campaign. Keep completion separate from acceptance-gate success.
- Link plots, videos, raw reports, and reproduction commands. Preserve compact
  evidence snapshots under `docs/iterations/`; `results/` is ignored and local.
  Keep large rollouts/videos local or in an explicitly identified shareable bundle.
- If tested code is uncommitted, preserve the relevant source or patch before
  changing it again. Label later source snapshots accurately; do not describe
  them as the exact source of older trials.
- Do not add overlapping test counts or describe a subset as the full repo suite.
  Label simulator state, inferred contacts, assumed limits, and hardware evidence
  accurately. Separate observations from causal hypotheses.
- For documentation-only changes, verify evidence, links, and archive contents;
  rerun simulations only when the experiment requires it.

Use the entry template at the end of `ITERATION_LOG.md`. Update the dated
presentation notes when a material result changes the research story.
