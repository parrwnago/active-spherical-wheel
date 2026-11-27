# I58 independent final all-seven saved-data audit

All independently recomputed core metrics and gate booleans agree with canonical `analysis/stage02`. Every run completed 44 seconds. All amplitude, numerical priority, coverage and execution controls pass. The initial -1 rad phase passes all 26 gates, but both broader phases fail behavioral gates. The three-phase screen therefore fails; the lead result must not be described as general acceptance or justify automatic default adoption.

Historical shared precision comparisons are independently byte exact: 59 fields for lead B and A, and 56 fields each for the +0.3/+0.9 native baselines. These are per-file shared-field counts, not added tests or independent trials. The unchanged audit source, initial audit and unavailable-scope record are preserved separately.

| Phase | B/P moving XY RMS, mm | B/P final error, mm | Failed P gates |
| --- | ---: | ---: | --- |
| -1.0 | 51.08 / 33.43 | 42.65 / 6.29 | None |
| +0.3 | 56.96 / 54.37 | 20.82 / 52.62 | Final XY; six-turn overshoot conjunction |
| +0.9 | 50.49 / 48.37 | 190.19 / 63.85 | Late yaw peak; full planar speed peak; six-turn overshoot conjunction |

For +0.3, the sixth turn overshoot rises by 12.90 mm, exceeding the allowed 10 mm increase. For +0.9, the third and fourth turn overshoots rise by 65.64 mm and 26.17 mm. Late yaw peak rises from 1.835 to 3.084 rad/s and full planar speed peak from 0.3192 to 0.5015 m/s. Lower RMS or final error does not override these failed preregistered conditions.

P restores the intended local modeled total transverse response in all three phases: maximum absolute discrepancies are approximately 4.69e-16, 9.99e-16 and 5.31e-16 rad/s. That implementation property coexists with the observed behavioral regressions. It is not a physical stability guarantee. Rates beyond the old 60/6 diagnostic values remain allowed in P.

None of the seven runs passes original final settling. Every conditional settling comparison passes because its native comparator also fails settling. Keep actual settling outcomes visible rather than calling them successful endpoint regulation. The lead physical-acceleration caveat also remains: P's 8.9546 m/s² 50 ms peak exceeds native B's 8.0775 while narrowly passing the selected 8.9853 threshold.

The independent audit recomputes XY, raw lean/tilt, world yaw/heading, planar speed, full 3D 50 ms acceleration, caps, input history, summed diagnostic priority, continuity, six geometric overshoots and settling using its own NumPy formulas. Reversal-lag details are not separately rederived. G itself is not reconstructed here; priority uses saved first-plus-secondary diagnostic vectors. Frozen bank/execution conditions remain provenance assertions. No new physics, controller/model/observer calls, fitting, runtime edits or threshold changes were made.
