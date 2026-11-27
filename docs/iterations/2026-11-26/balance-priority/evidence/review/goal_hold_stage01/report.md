# Post-primary exploratory goal hold, all three B/P pairs

This audit was registered separately from acceptance testing and uses fixed 41–44 s and 43.5–44 s windows. All six B/P cases are available in final `read002`; partial `read001`, which lacked +0.9 P, remains preserved. The final reference world goal is constant and shared by B/P. Positive measured radial velocity points toward that goal. This is saved-data diagnosis, not a new trial or changed gate.

| P phase | Distance at 41 s, mm | Distance at 44 s, mm | Change over last 0.5 s, mm | Mean radial velocity last 0.5 s, mm/s |
| --- | ---: | ---: | ---: | ---: |
| -1.0 | 8.38 | 6.29 | -0.859 | +1.713 |
| +0.3 | 23.48 | 52.62 | -0.071 | +0.157 |
| +0.9 | 29.15 | 63.85 | +10.028 | -20.091 |

The +0.3 P case does not show convincing recovery by 44 s. It first reaches 19.01 mm at 41.54 s, then moves outward to roughly 53 mm. The last half second is almost flat around 52.6 mm: mean approach is only 0.157 mm/s and final radial velocity is slightly outward at 0.300 mm/s. Longer-run recovery was not observed. Native B also does not settle: it passes within 1.69 mm at 43.455 s, then departs to 20.82 mm at 44 s. Its smaller final error does not demonstrate stable arrival.

The +0.9 P case is clearly moving away in the final half second: mean radial velocity is -20.09 mm/s and final radial velocity -11.62 mm/s. Distance rises by 34.70 mm over the whole hold. It performs better than its native B comparator, which reaches 190.19 mm, but is not returning to goal within the recorded interval.

In the +0.3 P final half second there is no native amplitude clipping, no old-rate diagnostic exceedance, and estimated/truth support labels agree. P's saved G20 travel adjustment projected toward the goal averages only -0.00394 mm/s. For +0.9 P the same final interval has no clipping/rate exceedance/support disagreement, and its mean G20 radial adjustment is -0.00758 mm/s versus measured outward velocity of about -20.09 mm/s. These are different quantities: G20 is the frozen differential counterfactual relative to clipped-native action, not an absolute velocity forecast or measured causal intervention. The tiny logged adjustment does not establish that all allocation effects or earlier accumulated state errors are negligible; it does make direct current allocator travel suppression an unsupported sole explanation for the ongoing drift.

Contact assignment still deserves localized investigation. In +0.3 P, shell-A/belt-A label-disagreement bursts occupy about 15.7% of held commands over 41–44 s. The first occurs at 42.265 s, after the outward trend begins near 41.54 s. Disagreement therefore cannot, by timing alone, explain the initial outward motion. It may affect the later transient. Matching final labels or endpoint contacts do not certify continuous contact correctness inside a 5 ms hold.

The next diagnostic target is the existing native goal-hold behavior at the actual P states: split its pose/velocity feedback, native contact/drive references and yaw/phase coupling, then check whether the requested motion has a sustained inward component and whether physical motion follows it. For +0.3, inspect 41.5–42.3 s before the first observed label disagreement, then the later shell/belt chatter. For +0.9, inspect the final steady outward interval despite matching support and no clipping. This is a specific investigation target, not a unique root-cause claim or recommendation to add estimator/controller complexity.

Original settling remains failed for every B/P pair. For -1 P, only angular rate fails in the last original second; for +0.3 P all three lean/speed/angular conditions fail somewhere in that second; for +0.9 P planar speed and angular rate fail. The exact maxima and reasons are in each read's JSON, with full-rate arrays and PNG/PDF plots.

Command data retain separate per-input components: first three are N m, last two m/s. Any raw aggregate command-vector norm in the generic report is not a physically homogeneous norm; use the per-input values for interpretation. Terminal command values are explicitly unheld, and command exposure statistics exclude them. No source/runtime tuning, new physics, model re-evaluation or acceptance-gate changes were performed.
