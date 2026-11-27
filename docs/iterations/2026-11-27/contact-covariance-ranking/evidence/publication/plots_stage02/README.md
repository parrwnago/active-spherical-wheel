Final cached-result figures for I-65. `covariance_comparison.pdf` has one page;
`event_windows.pdf` has eight pages: one explanation page and seven pages
covering every one of the 14 report event anchors, two events per page.
Independent PDF page-object counts agree with the writer's counts.

The overview PNG, explanation and first event-window figure were visually
inspected. The initial stage01 render/source remains preserved; stage02 fixes
a text-card/footer overlap and gate-label collision and clarifies interpretation.
The plots use existing results only, with no fitting, inference or new decisions.

The main finding combines fitted mean subtraction and covariance weighting;
their separate effects were not isolated. All 6,602 ambiguous accepted pairs
are duplicate alternatives. Distances are empirical scores, not probabilities.
The candidate improves relative scoring controls but fails against original A;
these three already examined runs establish no physical driving improvement.

Event windows show recorded +/-100 ms context. Scaled preference is only a
display transform of cached scores; cached classifications remain unchanged.
Later observations in the shaded region did not enter earlier inference.

To regenerate from cached experiment outputs and the original I-64 base:

```sh
python plot_results.py --root I65_ROOT --previous I64_EVIDENCE --output /tmp/i65_plots
```

Input/source/output hashes are in `plot_manifest.json`; `visual_qa.json`
documents page counts and layout checks. Canonical scientific results are
unchanged.
