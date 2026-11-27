# I-60 independent review

The blind design and source reviews preceded actual-data scoring. The rank and
evaluation scripts were independently written and sealed before their own
arithmetic, without importing the inference helper or evaluator. Later F/V,
geometry and point checks are explicitly post-primary verification, not new
acceptance gates or retuning.

- `design_stage01/`: blind review, pre-outcome confidence-denominator addendum,
  protocol/source/test/extractor/evaluator approvals and exact source bindings.
- `rank_stage01/report.json`: independently reconstructs all four methods and
  all 48 ranking output fields on three complete 8,801-row traces. Labels,
  acceptance, ties, family minima and confidence decisions agree.
- `evaluation_stage01/report.json`: independently derives truth/native labels,
  eligibility, measured excitation, stable masks, every fixed-window count and
  statistic, pooled metrics and all twelve gate values. All comparisons match;
  no gate fails. The descriptive timing algorithm was source-reviewed, not
  independently rerun by this reviewer.
- `post_primary_stage01/report.json`: fused and velocity-only raw family labels
  are identical at all 26,403 saved states. Primary accepted sets differ at
  three states: fused alone accepts one wrong +0.3-rad state and one correct
  +0.9-rad state; velocity alone accepts one correct +0.3-rad state. This does
  not establish an added benefit from the extra moment score. The prespecified
  stable ±10 ms truth subset still improves from 75 to 3 label disagreements
  over 21,983 eligible rows; this does not remove every clock/model ambiguity.
- `geometry_stage01/report.json`: 27 fixed protocol-boundary states, all 102
  geometric slots per state, independent band/rim geometry and direct-point
  Jacobians. Validity/frame flags agree exactly; largest material-velocity
  discrepancy is 6.33e-15 m/s. No inference-helper import, physics, dynamics or
  solved-contact query is used.
- `portable_stage01/report.json`: independently checks every retained compact
  feature value, original candidate identity, all four methods' family minima,
  all 48 stored score fields and minimal context against the full originals,
  byte for byte. Compact evidence supports the four frozen scores; it cannot
  regenerate geometry or justify new weights without the original full bank.
- `point_stage01/report.json`: independently reproduces all point-statistic
  groups to at most 1.34e-15 mm arithmetic difference. Candidate coordinates at
  1 s and each phase's worst fused raw-error sample match independent original-ID
  geometry exactly; native band/rim-share equivalent points agree within
  1.78e-15 m. Those six coordinate samples are post-primary validation choices,
  not a new or representative performance trial.

Primary dominant-support label agreement improves from 412/23,423 disagreements
to 109/23,423. Fused acceptance covers 6,902/7,970 ambiguous moving eligible rows
(86.6%), but only 6,902/23,423 of all primary eligible rows (29.5%). Accepted
disagreements are 18/6,902, versus 306/6,902 for native on the same rows. These
are correlated static observations in exact-state simulation, not independent
trials, calibrated posterior confidence, or demonstrated driving improvement.

Point quality is qualified. Pooled raw-equivalent RMS improves 3.134→2.672 mm,
while maximum distance worsens 26.91→44.04 mm and accepted maximum worsens
23.76→29.84 mm. The correct-family subset's maximum instead improves to
25.32 mm, so the larger reported tails occur among remaining wrong-family
selections. Better average identity agreement is insufficient to replace live
rolling-feedforward geometry without further controlled work.

The source review caught a point-count terminology error: frozen historical
`truth_patch_contact_counts` counts only records above 0.05 N. The legacy
`strict_one_physical_point` field therefore means one **resolved** contact;
weaker contacts may remain in load-weighted targets. The frozen numeric stage
was retained and a separate count erratum corrects interpretation.

No live controller, observer, feedforward, reference or default was changed by
this reviewer, and no new physics ran. No additional repository test suite was
invoked; the reviewed 28 selected synthetic passes are the same preserved
22 new plus 6 existing test invocation, not an added count. The system Python
lacks MuJoCo; NumPy audits use system Python and geometric checks use the repo
virtual environment, as recorded in the geometry registration.
