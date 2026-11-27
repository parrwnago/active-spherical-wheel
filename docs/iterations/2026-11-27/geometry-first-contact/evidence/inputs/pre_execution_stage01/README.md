# I-72 truth-free geometry-first inputs

Each case has two files. `rank_packet.npz` contains all nine original I60
projected-feature arrays unchanged, explicit original candidate IDs, the direct
native equivalent anchor N and sparse physical candidate coordinates.
`native_fallback.npz` contains native point/load/family information for the
consumer's fallback branch only. Native family and loads never enter ranking.

Coordinates are copied byte for byte from frozen **I70 `features.points_m`**
where the original I60 `valid` mask is true. All 102 IDs and original masks
remain. Invalid coordinates decode as NaN. The extractor does not calculate
distances, select candidates, call a ranker/NNLS, or generate new geometry.
Any missing original-valid physical coordinate is an explicit error; it is
never removed by a new eligibility gate. I70's temporal frame-zero mask is not
read or substituted for the original I60 mask.

This does not claim byte equality with unstored original I60 full geometry.
The earlier I70 feature-parity receipt (maximum difference below 1.59e-14)
is copied as historical provenance. New coordinate byte parity refers only
to the exact frozen I70 cache used here.

The anchor is `measurement_context.exact_estimated_contact_point_m`, copied
from guarded native observer/locator diagnostics. It can be an interior
load-weighted equivalent point at two rims; it is neither Nalt nor truth and
is not projected onto a sphere. `native_anchor_provenance.md` records its
source chain. A copied pure I60 `dominant_labels` function checks the fallback
family rule using estimated loads alone; no evaluator module, labels or
baseline comparison is loaded.

`packet_io.load_case(stage_root, case)` verifies file bindings and returns:

- `time_s`, `projected_features`, `world_points`, `native_anchor_point_m`,
  `candidate_ids` for the consumer's explicit ranking call;
- `native_fallback`, kept as a separate dictionary for post-ranking fallback.

Four unique synthetic contracts verify feature/anchor byte copying, explicit
ID order and sparse coordinates, missing-point errors, and that changing native
loads changes fallback only. The preliminary tested version is preserved;
repeated runs of those same four contracts are not additional test counts.
Final source/input bindings precede saved-data projection. No truth, reference,
desired velocity, future state, HMM history or new sensor data is read.
