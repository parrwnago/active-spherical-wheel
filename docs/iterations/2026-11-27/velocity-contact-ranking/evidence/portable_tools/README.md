# I-60 minima-preserving portable rank archive

`rank_stage01/` is a 6.34 MB standalone NumPy package, retaining all 8,801 original
timestamps from each of the three preselected native B cases. No data rows are
downsampled. All 48 ranking-output fields per case reproduce byte-for-byte,
including confidence/unknown/tie fields, family score tables and original
candidate indices.

The full 102-slot geometric candidate banks remain sealed locally. For each
frame and each of four families, the compact bank retains the union of that
family's winning candidate under each of the four frozen scoring methods.
Those at most four candidates are sorted by original candidate index, preserving
stable tie resolution. There are 16 fixed family-major slots; unused slots are
invalid. The compact array `source_candidate_index` maps every retained slot
back to its original slot. Replaying maps the recomputed per-family minima back
before comparing with original outputs. Scores are not tuned during packing.

Why this preserves the fixed calculation: every original minimum for every
method/family is retained, and removing nonminimum alternatives cannot produce
a smaller score. Sorting by original index retains the same lowest-index
winner at exact ties. All confidence gates depend on these family minima and
their residual vectors, which are retained unchanged.

The portable replay cannot regenerate the geometric candidate bank, independently
recover world contact coordinates, or support arbitrary new weights. Original
candidate identity and its exact selected features are preserved. New scoring
experiments require the original full banks; new feature-generation validation
also requires raw states and robot geometry. The original source/array hashes
and manifests remain attached rather than claiming those arrays are included.

Reproduce fixed rankings without Gazebo, MuJoCo or repository imports:

```sh
python3 PATH/TO/rank_stage01/replay_rank.py \
  --data-source PATH/TO/rank_stage01 \
  --output NEW_RANK_REPLAY_DIRECTORY
```

Both `replay_stage01/rank_replay.json` and
`external_replay_stage01/rank_replay.json` record successful complete replay.
The latter executes using system Python from `/tmp`, outside the repository.
Only NumPy is imported by ranking. `load_and_rank_case(data_source, case)`
returns minimal context, freshly recomputed original-index flat scores, and
parity information for the separate end-to-end evaluation replay.

The minimal context contains `time_s`, `exact_estimated_support_loads_N` and
`measured_excitation_m_s` unchanged. It contains no truth. Truth belongs only
to the independent evaluator. Packer/replayer code was frozen before accessing
the actual feature tables; source and four synthetic projection-contract groups
are under `pre_execution_stage01/`. These correlated contract groups are not
additional repository tests or physical trials.

To regenerate a new compact package from the three complete sealed local cases:

```sh
python3 PATH/TO/pack_rank.py \
  --case-directories FULL_LOCAL/m100_B FULL_LOCAL/p030_B FULL_LOCAL/p090_B \
  --output NEW_PORTABLE_PACKAGE
```

The packer validates all three input manifests before opening numerical inputs,
refuses an existing output directory, records bindings before projection,
verifies every ranking field and seals the resulting archive.
