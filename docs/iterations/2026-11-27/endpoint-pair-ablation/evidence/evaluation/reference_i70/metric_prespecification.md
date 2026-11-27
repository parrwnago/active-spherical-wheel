# I-70 evaluation specification before inference outcomes

This is a retrospective offline comparison on the three original I-58 native
44 s episodes: `m100_B`, `p030_B`, `p090_B`.  It makes no physical trajectory
improvement claim.  Native LQR, momentum observer, feedforward and preview stay
unchanged.  Evaluation labels are never passed to inference.

The primary window is the half-open `[1,41)` s window, 8000 samples per episode.
Dominant-support labels require finite nonnegative family positive-normal-load
values, total at least 1 N, a unique maximum and maximum fraction at least 0.9.
The denominator is independent of every prediction.  An unknown or pair output
is an error for this singleton-label metric.  This is sampled dominant positive
pressure, not all-force routing, first physical touch, or unique physical contact.
Original truth is the final 1 ms substep sampled every 5 ms; it is not truth for
the full observer interval.

Five primary comparisons are native `N`, raw I-60 fused `F`, accepted/fallback
I-60 `A`, untemporal emission MAP and causal HMM MAP.  The latter two sum the
rolling/slipping probabilities for each support set before MAP.  There are 11
observable support sets: four singletons, six lexicographic pairs and unknown.
No 21-mode calibration claim is possible without rolling/slipping labels.
An additional `state_hmm` control omits transition-edge likelihoods; it is reported
as a secondary ablation and never replaces primary `hmm` in acceptance gates.

For multiset evaluation, a family is in the sampled support set if it carries at
least 10% of total positive pressure.  All finite loaded primary rows remain in
the denominator.  A truth set containing three or four families is outside the
mode bank and is counted as an error, with an explicit expressivity count.
Unknown is always an error on this loaded domain.  Categorical set Brier is
evaluated on representable loaded truth sets; wrong unknown probabilities remain
in its 11-category vector and incur error.  Four family-inclusion Briers are
reported separately, including all loaded truth sets.  Brier is an empirical
model assessment, not a guarantee that beliefs are calibrated probabilities.

Confidence is the mass of the MAP 11-category support set.  At threshold 0.8,
unknown outputs do not contribute coverage.  Confident pairs contribute coverage
but are incorrect on dominant-singleton truth.  Confidence reliability uses ten
fixed equal-width bins.  Coverage/error is also reported at fixed thresholds
0,0.1,...,1.0; no threshold is selected from these curves.

Timing truth runs require at least 50 ms contiguous dominant-family samples.
Consecutive different stable runs define an event if the intervening gap is at
most 100 ms.  Its reference is the first sample of the new run, not physical first
touch.  A predicted singleton acquisition requires a run lasting at least 25 ms
and an onset within +/-100 ms; choose the closest unused eligible onset with an
earlier-index tie break.  Report signed/absolute latencies, misses and short new-
family attempts that reverted before 25 ms.  False switches are changed output
support sets between adjacent samples of the same stable truth run, split into
singleton toggles, unknown transitions and pair transitions.  Phases with no
events have no latency estimate.  Pooled absence of any scorable acquisition
fails the latency gate rather than giving an artificial zero latency.

Point distance uses the selected singleton's physical point and sampled dominant
family's raw positive-pressure centroid in world coordinates, in millimetres.
Wrong-family singleton distances remain in the all-singleton summary.  Pairs and
unknowns have no scalar point and count as missing coverage, not zero distance.
Correct-family-only and all-method matched subsets are separate summaries; none
can replace full-denominator assignment results.  RMS, median, p95, maximum,
coverage and the top ten point outliers per method are retained.  Fixed protocol
anchors retain all outcomes, including failures.

Root-approved acceptance gates, fixed before new inference outcomes:

- HMM pooled dominant-singleton errors at most 90% of `A` errors.
- Each episode's HMM error fraction at most `A` plus 0.001 (0.1 percentage point).
- HMM confidence-at-least-0.8 coverage at least 80% of primary dominant rows.
- HMM high-confidence wrong count at most all `A` primary errors.
- Full loaded-domain multiset accuracy no worse than `A` singleton-as-set.
- Representable-domain 11-category set Brier no worse than `A` one-hot.
- Pooled absolute transition latency median at most 20 ms and p95 at most 50 ms.
- Missed events, reverted attempts and false switches each no more than emission
  MAP control.
- HMM finite-singleton point coverage at least 80% of dominant primary rows.
- HMM point RMS and maximum no worse than `A` plus 1e-6 m (0.001 mm), on exactly
  the same primary rows with finite singleton points for both methods.  Retain
  wrong-family outputs.  Report any HMM-scorable/A-missing rows explicitly.

Completion is separate from passing these gates.  Every failure is reported, and
there is no inference-parameter sweep, calibration-label fitting or promotion to
the controller in this iteration.
