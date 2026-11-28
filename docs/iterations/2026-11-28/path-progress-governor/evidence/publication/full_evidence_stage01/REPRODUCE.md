# I-78 full local evidence archive

This archive preserves the closed six-case I-78 path-progress-governor experiment,
its registered sources/inputs and exact raw records, failed setup/contract stages,
canonical analysis and later corrections, independent verification and videos.
There is no new simulation or parameter search in producing this archive.

`project/` paths are relative to the original repository root. `_archive/` holds
the pre-archive bindings, excluded-file inventory, prerequisite inventory and
member inventory. Absolute paths inside original JSON/provenance are historical
records and are intentionally retained; use `project/<repository-relative path>`
for the corresponding captured bytes. The capture does not rewrite old records.

The original I76 registered source/input paths, expected SHA256, and their archive
members are in `external_prerequisites.json`. The three original straight-path
native precision files used for I78 replay verification are included. Other I76
campaign episodes referenced by its whole-campaign seal are not I78 inputs and
are not claimed to be fully copied. Required direct XML/SDF/config files are kept;
repeated generated meshes/terrain copies and render scene caches can be rebuilt
from the captured configuration, model source and registered original terrain
assets. Every omitted existing file has a path/size/SHA/reason inventory.

No virtual environment, Gazebo environment, credential helper or unrelated
repository files are archived. The two exact simulator plugin binaries are
included as registered inputs; runtime/environment records preserve versions.
Rebuilding on another machine may produce different binaries/numerical results.
The archive supports inspection and exact captured-array analysis independently
of any new physics run. Do not assume bit-identical physics on a new environment.

To inspect without rerunning physics, extract to a fresh directory, then review:

- `project/results/path_progress_governor_20261128/protocol/protocol.json`
- `.../evaluation/completed_stage01/report.json` and `.../evaluation/publication_stage01/`
- `.../governor/verification/` and `.../campaign/outputs_sealed.json`
- `.../publication/publication_report.json` and the three `video_*_stage01/` MP4s.

Verify each member against `_archive/archive_inventory.json`; the inventory
excludes itself from its digest list to avoid self-reference. The adjacent
`archive_verification.json` records a full streaming member-hash verification,
archive digest/size, and a final unchanged-file/unchanged-selection check.

For original-host re-analysis from retained paths (new, empty output stage):

```sh
.venv/bin/python results/path_progress_governor_20261128/evaluation/evaluate_stage02.py --help
.venv/bin/python results/path_progress_governor_20261128/publication/render.py --help
```

Use the recorded commands in each `registered_before_arrays.json` and the source
README for exact flags. The runner's `--run` launches new physics; it is not
needed to inspect the retained results. The evaluator preserves its original
registered version and separately identified structural-mask/boundary corrections.
Completion, acceptance and causal hypotheses remain distinct in those records.

Videos share physical elapsed time at 1x; ended panels freeze visibly. They are
MuJoCo display reconstructions of saved Gazebo/DART physics, not replay physics.
