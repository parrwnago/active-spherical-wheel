# I-65 read-only scientific result review

This review reads the completed stage01 report and protocol; it does not perform new historical scoring, fitting, threshold selection or a new experiment.

The method is materially different from an affine scalar cutoff: it uses shared empirical bias/covariance and all three residual coordinates. It gives a modest relative-ranking gain on the frozen alternatives. Primary M1 correct preferences improve from244/294 useful and1/6 harmful under relative maxabs to253/294 and2/6 under shared covariance. The registered90% useful and50% harmful requirements still fail. Its relative shadow veto loses41 useful corrections to prevent two harmful overrides, changing A's124 errors to163, while preserving the29.842859mm point maximum. M2 is control-only and also fails (245 useful,2 harmful,171 shadow errors). No estimator/controller integration is supported.

The I-65 relative-maxabs shadow has173 errors; the I-64 raw M1 absolute-bound shadow had132. They are different decision policies and must not be described as the same baseline or as an apples-to-apples covariance regression.

Calibration succeeded in every fold, with2151-2448 selected timestamps per training episode, shared means and finite regularized covariances. M1 normalized mean components are small (roughly .004-.014 in the shaft component across folds); the negative outcome is not an empty calibration or a numerical failure. Correct-family teacher rows remain only a surrogate for correct physical point and correct5ms load model. Episodes were already inspected and six harmful frames are sparse/correlated, so these are retrospective internal results.

All candidate scores are known. At least6590 of6602 ambiguous accepted pairs are duplicate correct same-family hypotheses:6884 F-correct frames minus294 native-wrong/useful frames leaves6590 both-family-correct, and each run's benchmark has zero same-family/different-candidate cases. The remaining12 are accepted-wrong ambiguous pairs. All300 useful/harmful decision opportunities are known and nonambiguous. Abstention therefore does not conceal the failing discrimination result. This accounting does not claim an exact same-family breakdown for those remaining12 without an additional saved-mask report.

The saved both-above-training-p95 diagnostic counts are M1:47/282/78 across the three starts (407 accepted frames), and M2:33/270/117 (420). This is descriptive model-fit evidence, not a calibrated out-of-distribution probability or permission to add a new cutoff. The +0.3 held-out episode contributes most of these examples.

Fixed event evidence:

- At7.875s, true-family F scores9.184 versus native-family alternative314.694 and is retained.
- At35.205s, wrong shell F scores132.654 versus correct-family belt alternative137.811; both are far above that fold's training95th-percentile score9.561. Relative preference for F is not evidence that either model fits well.
- First useful rejection at -1rad32.110s prefers wrong Nalt3.187 over true-family F9.912.
- First useful rejection at +0.3rad13.470s prefers wrong Nalt47.770 over F55.257, both above the corresponding training reference7.365.
- At +0.9rad6.850s the first useful rejection similarly prefers wrong Nalt34.307 over F47.204.

Event native_point_mm describes the ORIGINAL native-equivalent fallback point; it is not the physical Nalt point used to compute the candidate score. Do not use that field as Nalt geometry accuracy. Raw proposal point distances also differ from policy-A output distances on unaccepted frames.

## One proposed next diagnostic, not executed

Keep the calibration, candidates and live baseline fixed. At the already selected good/bad/first-lost examples, assess the physical F and Nalt point errors separately. Then use the sampled true point/family only as an evaluation geometry ceiling, with the same raw interval force and endpoint Jacobian. If the sampled-geometry model explains joint loads where a frozen correct-family candidate does not, that points toward candidate geometry. If it also fails, inspect interval transfer, single-resultant-force assumptions and multicontact load routing before changing the metric again.

Last-substep sampled truth is not exact5ms contact/force history, so this ceiling can localize a limitation but cannot establish a unique physical cause. No oracle data would enter inference or controller commands. This proposal requires a separately specified experiment; it was not run here.

Failure of this one empirical quadratic ranking does not invalidate the positive I-60 velocity-consistency result or all contact estimation. It does not establish the cause of trajectory drift. Keep native LQR, momentum observer and feedforward selected.
