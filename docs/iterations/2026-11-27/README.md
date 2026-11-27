# Research presentation notes — 2026-11-27

**I-73: fresh S-path contact-reference intervention completes, with mixed
tracking and no demonstrated settling improvement.**
[Full record and media](contact-reference-control/README.md),
[motion comparison](contact-reference-control/evidence/publication/plots_stage01/motion_comparison.png),
[two-page PDF](contact-reference-control/evidence/publication/plots_stage01/control_comparison.pdf).

- Nine new 45 s Gazebo/DART runs: N/native, A/guarded velocity, C/geometry-first,
  at −1.0/+0.3/+0.9 rad; fixed S, 3 s initial hold +32 s motion +10 s final hold.
  Original LQR/observer/gains/load routing/support/preparation/caps remain selected.
- Only rolling reference/feedforward records change. Native force/load/gain
  routes remain in use; this is not a complete estimator replacement. The final
  zero-speed hold bypasses new rolling selection and tests arrival/history effects.
- Moving timed XY RMS N/A/C: **47.31/44.39/45.38 mm**, **22.85/27.05/27.53 mm**,
  **23.37/23.53/23.47 mm**. No uniform improvement. C worsens late rotation at
  −1.0 and +0.3, and +0.3 final error grows 7.02→28.42 mm.
- All nine complete without physical failure; all 101 setup checks pass.
  C has 1.040/1.525/0.100 s of reference intervention. Twelve of 36 primary
  control checks fail. Keep exposure separate from applied-command causality.
- No case meets all four settling criteria for sustained 2 s. All fail rotation;
  several return close to goal. Free heading means this is not proof of a fall
  or failure to lean correctly. Show the full goal hold and body/reference tilt.
- Preserve failed map sites and revised facet screen (8°→12°) before physics;
  preserve the verifier schema correction and float32 media-loader failures.
  Final focused functions: 18 adapter/runner, 4 route, 22 evaluation inclusive
  of earlier 21, 4 graphics inclusive of earlier 3. Not a full repo-suite count.
- Candidate not adopted. Keep I-72's rejection, original live pipeline and parked
  solver/HMM work. Fresh driving data does not justify a post-result winner switch.
- Proposed follow-up: inspect native +0.9 versus +0.3 goal-hold command terms
  in existing logs before one targeted controller change. Heading is free but
  LQR angular-rate feedback exists; rotation alone is not a balance verdict.
  This is planning, with no new simulation, tuning or demonstrated correction.

**I-72: geometry-first velocity selection reduces point extremes, with a small
family/coverage regression; configured candidate not adopted.**
[Full record and replay](geometry-first-contact/README.md),
[three-page PDF](geometry-first-contact/evidence/publication/plots_stage02/geometry_first_contact_report.pdf)
and [physical candidate clouds](geometry-first-contact/evidence/publication/plots_stage02/physical_candidates.png).

- Geometry first: nearest original valid physicalpoint per family to native
  equivalentpointN; then exactI60fused/velocity scores andconfidence. No HMM,
  refit, threshold change, newphysics or livecontroller import.
- Same23423primaryrows: C127familyerrors versusA124; phases14/67/46 versus
  14/64/46. Acceptance6848vs6902, fixedoldambiguous-motion7970 coverage
  85.9222%vs86.5997%. Four of ten gates fail; two coverages shareanumerator.
- Samefullpointrows,100%coverage: N/A/C RMS3.1361/2.7591/2.9100mm,
  p957.3885/5.7970/6.3428mm, max26.9076/29.8429/25.5659mm. CbeatsallNpoint
  guards andAmax, butworsensA RMS/p95. Global guards do not imply everyrowbetter.
- At35.205s shellBwrongidentitypersists, point29.84→25.57mm; at7.875s both
  correctshellA butCpoint8.00mmvsA6.47mm. Oneof6oldharmful overrides repaired;
  usefulcorrections286/294retained. All471pair-pressure rowsremain counted.
- Thirty-nine componentcontractfunctions pass,5includedsubtests notadded;
  fourseparateindependentcontracts andall79fields/case agree. Sentinel-only
  verifiercorrection preserved. Isolated replay237arrays exact. No trackinggain.
- Preserve configuredrejection/spatialtradeoff. Keep originalLQR, observer,
  supportlocator, feedforward andpreparation; no post-result tuning orHMMrestart.

**I-71: endpoint-pair restriction does not rescue this HMM; stop the version.**
[Full record and portable replay](endpoint-pair-ablation/README.md),
[three-page explanation/results PDF](endpoint-pair-ablation/evidence/publication/plots_stage01/endpoint_pair_ablation_report.pdf)
and [comparison image](endpoint-pair-ablation/evidence/publication/plots_stage01/overview.png).

- Freeze I-70 costs/points and remove current pair states, while retaining
  two-family previous-interval transfer evidence. Keep diagonal holding
  probabilities and all dwell/noise/fit parameters fixed; condition only allowed
  priors/jump destinations. No new physics or live controller changes.
- Restricted frame303errors; no-edgehistory492; primary425 versus oldI70primary
  564 and guardedA124. Allthree phases regress versus matchingframe56→83,
  110→160,137→182. Four of eight predeclared continuation checks fail.
- Keep all471true pair-pressure rows as expressivity failures. Full-loadedset
  errors896, so eliminating pair outputs is not a generalmulticontact solution.
- Current exactlymatched23,153pointrows: A→primaryRMS2.61048→2.32530mm,
  max26.90764→29.45375mm,coverage98.8473%. Different matched rows fromI70.
  All71pressure transitions acquired,p9540ms. Seven of sixteen adoption gates fail.
- Twenty-seven new input/inference/evaluation contracts pass; original21metric
  contracts are reused. Independent recurrence/point/prefix checks agree,
  and isolated replay matches81arrays exactly. Preserve verifier-only boolean
  formatter failure and correction separately from the scientific stop.
- Keep LQR, momentum observer, geometry support, feedforward and incoming
  preparation. No further tuning of this HMM under the failed continuation gate.

**I-70: temporal contact modes implemented; first prototype not adopted.**
[Full record and portable replay](temporal-contact-modes/README.md),
[four-page explanation/results PDF](temporal-contact-modes/evidence/plots_stage02/temporal_contact_report.pdf)
and [comparison image](temporal-contact-modes/evidence/plots_stage02/overview.png).

- New opt-in21-mode model separates rolling/slip, single/two-family support
  and unknown. Raw5ms observer load, measured material velocity and known
  geometry feed constrained force fits and a causal transition-aware HMM.
- Three saved Mars starts,26,403frames; no newphysics, tuning or solver work.
  Alloutputs seal before truth evaluation. Live LQR/observer/feedforward and
  short incoming-drive preparation stay unchanged.
- Primary dominant-family errors564 versus native412 and offline guarded124.
  The564include140wrong singleton,223unknown and201pair outputs. Allthree
  phases regress and6/16adoption gates fail;69selected softwarecontracts pass.
- Pairlabels453/471correct, but total loadedset accuracy gains only13/23,894
  rows overguardedA. Exactlymatched pointRMS2.5455→2.2295mm whilemaximum
  worsens26.9076→29.4537mm, at98.19%finite-singleton coverage.
- All71defined transitions acquired, p95absolute latency40ms versusA20ms.
  Temporal beliefs can reinforce bad evidence:150confidentnonunknown mistakes.
  Posthoc finds+.3rad197/201falsepairs; paired gains26 versuslosses466.
- Retain failedprototype and limitations. Independent recurrence/force fits
  agree; isolated uncached replay reproduces108output and18featurearrays.
  This is offline estimator evidence, not improved trajectory tracking.

**I-69: native solver impulses explain the unusual zero-normal shell forces.**
[Full record](contact-force-boundary/README.md) and
[one-page explanation PDF](contact-force-boundary/evidence/publication/plots_stage02/contact_force_cause.pdf).

- Installed Dantzig freezes tangent bounds using an intermediate normal solve;
  later pivots can change normals. A controlled SPD problem reproduces final
  normal0/tangent.7 with success. Default PGS fails; no solver switch adopted.
- A private delegate capture preserves all59precision and36substep arrays
  byte-exact on the original44s native Mars replay. Actual shellB impulses at
  35.203/35.205s have zero normal, nonzero tangent, and recreate the old forces.
- All20native calls return success,0PGS;12/140applied contacts exceed final
  component bounds. All140match exports,25nonzero contacts have unique families.
- The good control also contains the effect. Specific force anomaly explained;
  whole trajectory deviation and a corrected solver remain unresolved.
- Keep LQR/observer/feedforward/preparation as baseline. The proposed solver
  follow-up was later parked at the user's request; preserve it as history.

**I-68: matching the observer interval changes our interpretation of contact errors.**
[Full record](contact-interval-replay/README.md),
[comparison PDF](contact-interval-replay/evidence/publication/plots_stage02/interval_comparison.pdf)
and [four event pages](contact-interval-replay/evidence/publication/plots_stage02/event_windows.pdf).

- One unchanged native −1rad Mars replay captures44,000physics steps. All59
  original precision fields reproduce byte-exactly; live controller is unchanged.
- Joint-load discrepancy is0.0000866Nm with integrated5ms point forces versus
  0.1163Nm with only the last1ms solve. Post-pose control also improves.
- At32.110s shell-to-belt transfer occurs inside the observer interval; prior
  shell impulse explains its apparently inactive shell joint load.
- At35.205s, zero shell normal pressure hides admitted tangential shell forces;
  their torque matches the observer. Native wrench sums include them, whereas
  pressure-based family labels do not. Earlier “belt-only” statements need that
  qualification; historical error counts are not relabeled in this iteration.
- Posthoc screen finds this pattern in290/8,000primaryintervals, including good
  control. Its installed solver/contact-boundary cause is unresolved. Postsolve
  slip0.139m/s at the later event also limits the strict rolling assumption.
- Keep LQR/observer/feedforward/preparation; audit contact-force/normal reporting
  and label semantics before tuning around this behavior. No tracking gain yet.
- Independent trace/Jacobian checks and lossless cached replay pass. Preserve
  failed setup/analysis stages and figure layouts; portable capture14.13MB.

**I-67: the original bank reproduces all targeted reference-point repairs.**
[Full record](contact-bank-audit/README.md),
[comparison PDF](contact-bank-audit/evidence/publication/plots_stage02/bank_capacity_comparison.pdf)
and [eight-page candidate-cloud appendix](contact-bank-audit/evidence/publication/plots_stage02/event_windows.pdf).
The [standalone ChatGPT brief](../../CHATGPT_CONTROL_CONTEXT.md) includes the
whole dynamics/controller/observer pipeline and this latest result.

- Freeze original102slots, admission masks, fused objective, I-65 calibration
  and comparison gates. Audit705 saved rows, including all300 useful/harmful
  opportunities and the14 fixed I-66 event windows; no new physics.
- All29 I-66 newly repaired decisions have an existing usable point that
  reproduces the correct preference. The bank supports27/41 focus cases;
  14 remain unresolved under both sampled reference and existing-bank comparison.
- Focus median selected distance6.26mm versus nearest usable0.81mm; nearest
  pre-admission distance is identical. Conditional all-useful capacity280/294
  and harmful5/6 are oracle-assisted existence counts, not achieved control gains.
- Original fused IDs reproduce correctly. Normal-prior penalty favors the
  farther selected point in26/41 focus cases, velocity in15/41 and allsix
  harmful cases. This explains the objective tradeoff, not physical slip.
- Minimum load score is not closest geometry: harmful median point distance
  remains19.50mm under that choice. Keep original live controller/observer/
  feedforward/preparation; next proposed target is point selection, not bank size.
- Fourteen inclusive geometry plus12 separate evaluator tests pass. Independent
  hinge/eigen comparisons and cached177-array replay agree. Preserve the helper
  admission-offset clarification, independent scope correction and figure layouts.

**I-66: candidate geometry explains part of the load-ranking failure;
current single-family load representation remains limited.**
[Full record](contact-geometry-load-audit/README.md),
[comparison PDF](contact-geometry-load-audit/evidence/publication/plots_stage03/geometry_load_comparison.pdf)
and [eight-page event appendix](contact-geometry-load-audit/evidence/publication/plots_stage03/event_windows.pdf).

- Replace only the known-correct physical candidate point with the recorded
  family centroid, keeping pose, raw observer force and I-65 calibration fixed.
  This is evaluation-only; the centroid is a last-reported 1 ms sample while
  the observer describes the preceding 5 ms. Exact interval truth is absent.
- On all 41 lost useful corrections, raw joint-vector RMS falls 24.1%,
  0.13926→0.10568 N m. Thirty-seven improve, four worsen. The registered
  criterion fails because the sole -1 rad focus frame worsens.
- Truth-assisted point replacement restores 27/41 lost useful preferences;
  all useful preferences rise 253→276/294 with four new losses, harmful
  rejection improves 2→4/6. No deployed error rate is claimed.
- At 35.205 s the correct belt candidate is 26.04 mm from the sampled centroid;
  replacement fixes its relative ranking. At 32.110 s a 2.79 mm replacement
  worsens error, while allowing the full inferred wrench reduces it strongly.
- Some joint-load rows cannot be produced by any point/wrench on the current
  material body. Their structural error floor is real in the model but cannot
  identify interval contact transfer versus model/actuation error. Do not
  attribute the remaining two harmful preferences entirely to that floor.
- Every useful/harmful endpoint has one positive sampled family normal load;
  the normal-share proxy therefore equals the single-centroid model there.
  This does not establish strict single contact or exclude hidden transfers.
- Fifteen geometry and seventeen separate evaluator tests pass. Independent
  hinge/eigen arithmetic and a separate structural check agree. Producer
  replay reproduces 177 arrays exactly; cross-version decisions also agree.
- Keep live LQR/observer/feedforward/preparation unchanged. Next proposed:
  candidate-bank coverage versus selected-point quality, followed by matched
  substep logging if needed to isolate remaining timing/load routing.

**I-65: a different joint-error pattern score improves relative ranking but
still rejects too many useful corrections.**
[Full record](contact-covariance-ranking/README.md),
[comparison PDF](contact-covariance-ranking/evidence/publication/plots_stage02/covariance_comparison.pdf)
and [event appendix](contact-covariance-ranking/evidence/publication/plots_stage02/event_windows.pdf).

- Replace largest-component scoring with a shared empirical bias/covariance
  score of all three joint-load errors. Fixed shrinkage, variance floor and
  ambiguity rules; fit on two whole episodes and evaluate the third.
- This is an actual change of calculation, not another scalar-threshold
  coefficient sweep. Simulator family labels select training examples only;
  held-out labels evaluate. The three episodes remain previously seen data.
- Useful correct preferences improve 244→253/294 and harmful correct
  preferences 1→2/6 against the same raw-interval maximum-error ranking.
  Both minimum discrimination targets still fail.
- The shadow estimator loses 41 useful corrections to prevent two harmful
  changes: 124→163 family-error frames. Most losses, 37, occur at +0.3 rad.
  Worst point discrepancy remains 29.84 mm. Matched-history control also fails.
- All proposals are scorable. Ambiguous pairs retain the original estimate;
  all 300 useful/harmful opportunities remain distinguishable, so abstentions
  do not hide the failed result. Only six harmful frames exist across runs.
- Keep the live LQR, momentum observer, feedforward and incoming preparation.
  No new driving/video or tracking gain is claimed. Next proposed question is
  candidate-point versus interval load-sharing adequacy at these same events.
- Fourteen method and ten separate evaluator contracts pass. Producer replay
  reproduces all 129 arrays exactly; cross-version replay preserves discrete
  decisions within registered numerical limits. Failed launch/review attempts
  and their corrections remain separate evidence.

**I-64: matching load history improves model consistency, not the contact veto.**
[Full record](contact-load-history/README.md),
[comparison PDF](contact-load-history/evidence/analysis/plots_stage01/load_history_comparison.pdf)
and [eleven event windows](contact-load-history/evidence/analysis/plots_stage01/event_windows.pdf).

- Predict joint load from the observer's raw interval force and frozen causal
  contact history, then apply the same 100 rad/s filter. Preserve endpoint
  geometry as an approximation and exclude 200 ms after a history reset.
- On 6,884 identical accepted correct-family frames, vector RMS improves
  0.05828→0.03721 N m (36.2%); p95 0.13549→0.07141. Every starting phase
  improves and all critical events remain scorable. Model-consistency Q1 passes.
- Current-family Q2 fails: useful correct preferences 212→243/294, harmful
  correct preferences 2→1/6. Smaller mismatch does not establish the family.
- The matched fixed veto yields 128 errors versus A's 124. It discards four
  useful corrections instead of 30, but prevents none of six harmful changes.
  All fixed shadow policies fail; native LQR/observer/feedforward remains live.
- No new driving, threshold fitting or physical tracking improvement. Preserve
  this partial success and separate load-model validation from contact choice.
- Nineteen feature and fourteen separate analysis contracts pass. Producer
  replay is byte-exact; independent computations agree. Cross-version raw float
  bytes differ by tiny amounts, so portable v2 verifies exact decisions and
  bounded summary statistics with explicit limits, without replacing values.

**I-63: observable transition conditions do not rescue the tested load check.**
[Full record](contact-condition-diagnostic/README.md),
[overview PDF](contact-condition-diagnostic/evidence/analysis/plots_stage01/condition_diagnostic.pdf)
and [17 annotated event windows](contact-condition-diagnostic/evidence/analysis/plots_stage01/event_windows.pdf).

- The user's dynamic-threshold idea is tested with one predeclared causal
  condition score: observer interval-versus-filtered change and signed
  shell/belt surface distance/rate, combined by maximum. These are inferred
  uncertainty clues, not independent true-contact sensors.
- Surface arclength replaces an unexecuted axial-distance proposal before
  scoring; a 12 mm axial window would cover the entire 4.35 mm-deep polar cap.
- Every primary state has known risk. All six harmful overrides have high
  score, 0.892–0.959; the worst bad point has r=0.938. Loosening the check in
  those conditions would also protect that mistake.
- The primary affine allowance still forces 99 useful losses to remove the
  worst point. Only six harmful changes can be prevented, giving a lower bound
  of 217 errors versus A's 124. This is a bound, not a new achieved result.
- The registered stop prevents 18-schedule fitting and a new driving episode.
  Independent geometry/bounds and exact NumPy replay agree. Preserve twelve
  feature contracts and fifteen separate evaluator contracts, without calling
  them a controller/full-suite run.
- Live LQR, momentum observer, feedforward and existing preparation remain
  selected. This excludes one rule, not all dynamic adaptation. Matching
  predicted/observed load history is the next proposed investigation.

**I-62: tolerance tuning cannot resolve the load-check tradeoff.**
[Full record](contact-load-tuning/README.md) and
[tradeoff PDF](contact-load-tuning/evidence/analysis/plots_stage02/tolerance_tradeoffs.pdf).

- The user's tuning suggestion is tested with 3,721 shared `(a,b)` pairs on
  the same three saved starts, using whole-start training/evaluation splits.
  Every training fold and the full-data fit have zero qualified settings.
- The best pooled label-only witness changes 124→123 errors and retains all
  294 useful corrections, but removes only one of six harmful changes and
  leaves the worst point discrepancy at 29.84 mm. It is not adopted.
- Removing half the harmful changes costs at least 111 useful corrections on
  the grid (best error count 232). Meeting the native point maximum costs at
  least 99 useful corrections (grid-best error count 221).
- An independent dominance proof extends that 99-loss point requirement to
  any nonnegative shared affine tolerance on these fixed inputs: even removing
  all six harmful changes gives at least 217 errors, versus A's 124.
- These are correlated historical states and previously inspected episodes,
  not unseen validation or new driving evidence. The live LQR, momentum
  observer, feedforward and existing preparation stay selected.
- Preserve the failed numerical benchmark attempt and its summation-layout
  correction, all grid results and 11 inclusive replay/selection contracts.
  Continue investigating the load model's timing/transfer assumptions rather
  than interpreting a smaller residual as guaranteed correct contact.

**I-61: adding joint-load consistency does not improve the guarded estimator.**
[Full record](contact-load-consistency/README.md) and
[comparison figure](contact-load-consistency/evidence/analysis/plots_stage01/overview.png).

- Keep I-60's velocity ranking fixed; use the three extra observer joint rows
  only to veto an accepted contact proposal, falling back to the native estimate.
- Correct comparator: I-60 accepted/fallback has 124 errors, rather than 109
  for all raw rankings. The new veto increases that to 153.
- It removes one of six harmful overrides, but also loses 30 of 294 useful
  corrections. Five of 13 fixed gates fail; the conditional physical trial
  does not run. Worst guarded point discrepancy stays 29.84 mm.
- The illustrative bad shell-B proposal at 35.205 s still passes the load
  tolerance. A useful shell-A correction at 7.875 s fails it instead.
- All 30 lost useful corrections occur near transfers and fail compatibility
  even with the logged dominant-family point. Stable-point compatibility >99%
  therefore does not establish transfer validity. Filtering, timing and force
  distribution are possible contributors; no unique cause is proven.
- Preserve the rejected trial. Keep native LQR, momentum observer, feedforward
  and existing preparation selected; the I-60 velocity signal is still useful
  offline. Next proposed diagnostic: time-consistent load/transfer hypotheses.

Implementation checks pass (45 selected core tests, nine separate evaluator
contracts, independent replay); the scientific candidate fails. The archive
retains fixture/receipt/version-roundoff corrections separately from that result.

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
