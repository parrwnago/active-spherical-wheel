# I-70 measured-state input preparation

This directory prepares offline inference inputs. It runs no physics, reads no
solver contact/terrain/truth labels and changes no live controller or estimator.
Measured body state is still simulation state, as explicitly allowed for this
stage. The applied belt speed is an achieved surface speed only under the
existing ideal prescribed-belt simulation model.

The explicit field whitelists in `prepare.py` read three frozen native I58
episodes through their I60 measured-state caches, and the known-body pose maps
from I66. Native estimated family/load labels are not read or used as priors.
The I67 bank helper is copied byte for byte and reconstructs the original 102
seed-major surface/rim candidates from the fixed 10-degree normal seeds.
Projected slots rejected by the original cone/admission checks are retained
with invalid masks. Original candidate validity is retained; no scoring, geometry search expansion or
threshold tuning occurs here.

Each case produces:

- `primitives.npz`: time, q/v, previous actual input, raw interval observer9,
  filtered observer9, filtered intrinsic-normal hint, original validity,
  excitation, known sphere/band metadata and cached body axes/centre.
- `features.npz`: local full-bank point/normal/material-velocity/tangent-velocity
  cache. This larger convenience file is derivable and need not be archived.
- `metadata.json`: source/input bindings, whitelists and parity results.
- `artifact_manifest.json`: exact output bindings.

Load NPZ arrays eagerly once into dictionaries. The method interface is
`input_features.frame_features(primitives, index, cached=None)`. With the local
cache, the same call avoids bank reconstruction; without it, portable replay
derives the bank and velocities from measured primitives. It returns the raw
9D observation, full 102-point bank, inward force normals, material velocities,
9-by-3 world-force maps, family/index/kind metadata and validity flags.

Material velocity includes the imposed belt tangent speed; the rolling residual
is its projection onto the candidate tangent plane. Both forms are exposed, so
the method must project the full velocity before using a rolling likelihood.
Candidate normals are radial, inward sphere normals, which remain a model
assumption at rims and on rough terrain.

Force maps use the physical candidate's endpoint pose. Their free-base angular
rows are `R_mid * J_native_angular`, with `R_mid` exactly the normalized,
hemisphere-corrected sum of previous/current fork quaternions. This aligns the
moment convention with the raw observer interval. It does not reconstruct the
actual five substeps or prove a single contact was active for the whole 5 ms.
Scalar joint loads retain their native joint coordinates.

Frame zero has no preceding interval and all method candidates are invalid.
Non-5-ms adjacency also invalidates interval candidates. Geometry may remain
available for inspection, but must not manufacture a load-history observation.

Six synthetic integration contracts check original geometry/rolling vectors,
MuJoCo force Jacobians and midpoint convention, missing-interval handling,
past-only access, truth exclusion, and cache equivalence. The source/input seal
is in `pre_execution_stage01/source_input_seal.json`, before new historical
feature arithmetic. Extraction checks every original validity mask exactly and
the valid original motion/angle/moment features within 1e-12.

Canonical preparation:

```sh
.venv/bin/python results/temporal_contact_modes_20261127/inputs/prepare.py \
  --measurement-root results/velocity_contact_rank_20261127/campaign/features_stage01 \
  --pose-root results/contact_geometry_load_audit_20261127/features/stage01 \
  --output NEW_OUTPUT_DIRECTORY
```

The three cases are processed independently. Do not overwrite an earlier output
directory after a failure; preserve its partial inputs/logs and use a new stage
if a correction becomes necessary.
