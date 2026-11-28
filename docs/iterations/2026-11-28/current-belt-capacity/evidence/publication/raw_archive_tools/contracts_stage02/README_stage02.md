# I-82 final raw archive — supplemental source closure

Use `build_archive_stage02.py` for the final archive. The original tested
`build_archive.py`, its README and four-contract stage remain unchanged.
Stage02 retains those contracts and adds one supplemental-source closure
contract; its final five functions are not added to the earlier four.

The physical experiment remains bound by its original **93 source / 59 input**
protocol. After physics, closed analysis/media registrations identified four
external helper sources that also need restoration at their original paths:

- `results/tangent_heading_routes_20261127/evaluation/metrics.py`
- `results/slope_response_trace_20261128/analysis/core.py`
- `results/tangent_heading_routes_20261127/publication/video_tools/render.py`
- `results/path_progress_governor_20261128/publication/render.py`

The new helper reads only three explicitly named closed registration receipts,
checks these exact historical SHA256 values and rejects unknown external
dependencies. It archives the four sources at `project/<original relative path>`
and binds their receipts in adjacent/embedded `supplemental_dependencies.json`.
This is an archive dependency correction, not a changed physics registration,
analysis result, gain, cap, metric or acceptance gate.

The completed portable visual ZIP is explicitly included and checked against
its closed report. Its JSON/Markdown metadata remain included. Duplicate expanded
portable movies/images are omitted because canonical MP4s and metadata are
included. Scene/workspace/interpreter caches and curation outputs are inventoried
as before. Canonical full/event movie files are independently checked against
the final media artifact manifest before and after tar construction.

After root closes science, media and the stable curation helper:

```bash
.venv/bin/python results/current_belt_capacity_20261128/publication/raw_archive_tools/build_archive_stage02.py \
  --protocol results/current_belt_capacity_20261128/protocol/frozen_stage01/protocol.json \
  --seal results/current_belt_capacity_20261128/campaign/outputs_sealed.json \
  --seal-base results/current_belt_capacity_20261128 \
  --observed-exit results/current_belt_capacity_20261128/campaign/B030.observed_exit.json \
  --observed-exit results/current_belt_capacity_20261128/campaign/B060.observed_exit.json \
  --case030 results/current_belt_capacity_20261128/campaign/runs/B030 \
  --case060 results/current_belt_capacity_20261128/campaign/runs/B060 \
  --media-manifest results/current_belt_capacity_20261128/publication/video_stage01/artifact_manifest.json \
  --visual-bundle results/current_belt_capacity_20261128/publication/visual_bundle_stage01/I82-visual-comparison.zip \
  --output results/current_belt_capacity_20261128/publication/full_evidence_stage01 \
  --build
```

Run with direct tool stdout or a shell log outside the evidence family. Every
member is registered before construction and stream-verified afterward; final
checks cover the physical bindings, supplemental bindings, sealed outputs,
media and selected/omitted-file set. No new simulation or video is produced.
