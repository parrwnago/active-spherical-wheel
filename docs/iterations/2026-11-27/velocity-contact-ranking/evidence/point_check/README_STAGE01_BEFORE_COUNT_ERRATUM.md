# I-60 descriptive selected-point check after primary classification results

The post-primary protocol was registered after the positive family results and
before inspecting selected-point distances. No estimator score, bank, gate,
normal cone or live command was changed. This is descriptive evidence, not a
new acceptance screen or a controller trial.

The selected original candidate IDs are reconstructed from exact recorded pose,
the same stored observer-derived normal hint and frozen spherical-band/rim
projection. Only private forward kinematics and four selected projections per
frame are evaluated; no full velocity bank, Jacobians or physics are rerun.

For the 23,423 dominant-support eligible primary states pooled across starts,
fused selected points improve average distances but worsen worst cases:

| Matched subset / raw equivalent-point target | Count | Native RMS, mm | Fused RMS, mm | Native maximum, mm | Fused maximum, mm |
| --- | ---: | ---: | ---: | ---: | ---: |
| All dominant-support eligible | 23,423 | 3.134 | 2.672 | 26.91 | 44.04 |
| Fused accepted states | 6,902 | 3.870 | 2.729 | 23.76 | 29.84 |
| Exactly one logged physical point | 20,851 | 2.992 | 2.569 | 26.91 | 44.04 |

The raw-equivalent mean improves 1.207 to 0.982 mm and the 95th percentile 7.336
to 5.110 mm. On the matched all-eligible sets, RMS improves at each start:
-1 rad 2.843 to 2.216 mm, +0.3 3.340 to 2.725, +0.9 3.209 to 3.017. Maximum
distance worsens at each start: 26.91 to 29.84 mm, 22.72 to 28.70, and 25.23
to 44.04. The largest error also occurs in the strict one-point subset, so it
cannot be dismissed solely as a load-weighted average of multiple points.

Using the dominant-family load-weighted point gives nearly identical findings;
radial-R normalization of both target types also leaves them essentially
unchanged. Those normalized targets are comparison proxies, not corrected
truth. Geometry/wrench-only alternatives worsen pooled RMS to 3.264 mm;
velocity-only RMS is 2.666 mm, slightly below fused 2.672. None establishes a
benefit from turning the added moment score into a control decision.

Every reported comparison uses identical finite native/candidate/target rows
within its subset. Accepted-subset native metrics are recomputed on those same
accepted rows; they must not be compared with a different full-data denominator.
The source and JSON retain all four methods, four targets, all/correct-family/
accepted/strict-point subsets, missing coverage, movement from the native point,
and full descriptive statistics. The candidate's maximum move from the native
point is about 20.917 mm under the fixed ten-degree search cone.

Limitations: logged equivalent and patch points are sampled last-1-ms
load-weighted means, not necessarily physical contacts. Estimator geometry uses
current pose and filtered historical wrench. The strict point-count subset and
radial comparisons reduce particular ambiguities but do not remove all clock,
finite-surface or modeling conventions. Better family classification and lower
average point error are insufficient to justify replacing rolling-feedforward
geometry immediately: tails worsen, including among accepted states. No improved
closed-loop behavior or resolved physical root cause is claimed.

The compact selected points, targets, masks and native comparison points total
about 3 MB under `stage01/`. Source/input/protocol hashes precede point arithmetic.
Recompute every case/pooled statistic with only Python 3 and NumPy:

```sh
python3 PATH/TO/point_check/replay_points.py \
  --data-source PATH/TO/point_check/stage01 \
  --output NEW_POINT_REPLAY.json
```

This replays descriptive distance arithmetic. Regenerating world points also
requires the hash-bound original raw states/model; the compact replay does not
claim to replace that geometric reconstruction.
