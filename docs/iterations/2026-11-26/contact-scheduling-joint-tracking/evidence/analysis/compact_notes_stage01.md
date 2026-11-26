# I-52: normal-relative scheduling and joint-reference tracking

Canonical report SHA256: `1694871c0cab6649005c9f0b12dc709ba498bfe70d550dd94503f217050b3c39`. All twelve valid cells completed44s. One native process exited139 before producing a scored trace; its original files and the single registered exact-source retry are retained. This is13 process attempts and12 scored configurations.

The same original Mars figure-eight, .30m/s belt cap, flat gain bank and observer were used at three initial phases. Four policies were compared within each phase: B original, R the exact I-51 joint-reference guard, N whole-run estimated-normal gain-phase lookup, NR both.

| Start(rad) | Policy | Moving RMS(mm) | Peak(mm) | Final(mm) | Peak ref.lean(deg) | Settled | Original route |
|---:|:---|---:|---:|---:|---:|:---:|:---:|
| -1.00 | B | 51.08 | 116.26 | 42.65 | 7.40 | False | False |
| -1.00 | R | 76.79 | 306.79 | 43.27 | 13.20 | False | False |
| -1.00 | N | 39.19 | 87.89 | 12.69 | 5.07 | True | True |
| -1.00 | NR | 49.69 | 148.61 | 31.78 | 12.52 | False | False |
| +0.30 | B | 56.96 | 154.28 | 20.82 | 9.48 | False | False |
| +0.30 | R | 48.41 | 143.30 | 32.00 | 9.35 | False | False |
| +0.30 | N | 53.94 | 192.14 | 16.57 | 10.71 | False | False |
| +0.30 | NR | 53.47 | 179.98 | 17.96 | 10.22 | True | False |
| +0.90 | B | 50.49 | 190.90 | 190.19 | 8.71 | False | False |
| +0.90 | R | 44.21 | 97.19 | 32.69 | 6.35 | False | False |
| +0.90 | N | 43.28 | 93.65 | 36.31 | 7.76 | False | False |
| +0.90 | NR | 44.97 | 99.39 | 9.53 | 6.20 | True | True |

N at−1.00rad and NR at+0.90rad improve several outcomes and pass the original settling/route gates. NR at+0.30rad settles, but fails the original route gates. These successes remain visible; none passes every registered turn/regression requirement, and no policy passes the full screen across all three phases. All72 fixed turn rows are retained in [turn_metrics.csv](stage01/turn_metrics.csv).

The results also expose regressions. R at−1.00rad increases moving RMS from51.08 to76.79mm, peak error from116.26 to306.79mm, and peak reference-relative lean from7.40 to13.20deg. N/NR at+0.30rad have larger peak error and lean despite some lower RMS/final errors. The within-phase comparisons and difference-of-differences remain separate in [factorial_effects.csv](stage01/factorial_effects.csv); no pooled average overrides a failed phase.

Joint-reference and actual joint-rate tracking are different quantities. Both actual−native-target and actual−used-target errors are reported for each motor and as the three-joint error-vector norm. The legacy `speed_clip_held_samples_byjoint` field counts NATIVE targets outside the20rad/s box even when the guard is OFF: these are potential box violations, not actual guard activation. Actual modified samples/duration and guard diagnostics are separate.

For example, R-only at−1.00rad modifies55 held commands; its speed ceiling never binds. Actual-minus-used joint-error vector RMS worsens from3.319 to4.963rad/s. This is a real tracking regression in the changed closed-loop history, not a benefit established by smaller targets. Conversely, the very large native LS mismatch in+0.30rad R is an unbounded desired-rate mismatch; it must not be presented as an achievable motor tracking demand or a measured motor-response model.

Introduced rolling-reference residual is reconstructed only at modified R/NR rows using private current-pose Jacobians and saved ESTIMATED contacts. Exact single-patch global points and plural per-patch float32 points are labeled separately. This reference mismatch is distinct from actual-motion consistency, true slip, dynamic feasibility and body-path error.

N changes only a lookup angle in the existing flat gain bank. It leaves the original error/input frames and ordinary shaft-gain row intact, and one projected angle omits the lateral normal component. The report retains carrier-Y/XZ diagnostics and distinguishes a requested phase from periods when a previous gain is held. These limitations are observations about the implemented representation, not proof of a unique cause for every regression.

Turn timing retains both the registered SIGNED lag guard and changes in ABSOLUTE timing error. A signed-lag guard failure is not automatically a larger absolute timing error. Completion, original route gates, provisional screens and mechanistic reference bounds remain separate.

Evidence: [compact JSON](compact_stage01/summary.json), [canonical report](stage01/report.json), [all-phase tracking](stage01/tracking_all_phases.png), [headline tradeoffs](stage01/headline_tradeoffs.png), [all six turns per case](stage01/turns_all_phases.png), [joint actual/target tracking](compact_stage01/joint_tracking_all_phases.png). The joint-error plot uses a log scale to expose the range of native LS demands; its reference bounds are declared simulation assumptions, not physical motor limits.

No repository defaults, weights, estimator or hardware model changed. Three deterministic starts on one route are configuration checks, not statistical replication or general terrain qualification.
