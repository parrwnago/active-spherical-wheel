# I-82 final raw archive — path-formatting correction

The canonical packaging helper is `build_archive_stage03.py`. Its six scoped
contracts include the previous five and a regression for relative versus
absolute CLI paths. This is packaging only; physics, analysis and media are
unchanged.

The stage02 helper passed its five scoped contracts, but the first real build
stopped before tar construction while formatting a relative visual-bundle
report path. Preserve that failure in `../full_evidence_stage01/`, including
the exact failed source copy and traceback. Stage03 resolves the path before
making it repository relative and writes a fresh `full_evidence_stage02/`.

Supplemental source closure, selection and verification are described in
`README_stage02.md`. Four explicitly registered analysis/media dependencies
are restored at their original repository paths; the physical registration
remains 93 sources and 59 inputs. The closed portable visual ZIP and canonical
movies are included. All members are streamed and checked, followed by a check
that original bindings and selected/omitted evidence stayed unchanged.

After root closes science/media and freezes the curation helpers, run from the
repository root:

```bash
.venv/bin/python results/current_belt_capacity_20261128/publication/raw_archive_tools/build_archive_stage03.py \
  --protocol results/current_belt_capacity_20261128/protocol/frozen_stage01/protocol.json \
  --seal results/current_belt_capacity_20261128/campaign/outputs_sealed.json \
  --seal-base results/current_belt_capacity_20261128 \
  --observed-exit results/current_belt_capacity_20261128/campaign/B030.observed_exit.json \
  --observed-exit results/current_belt_capacity_20261128/campaign/B060.observed_exit.json \
  --case030 results/current_belt_capacity_20261128/campaign/runs/B030 \
  --case060 results/current_belt_capacity_20261128/campaign/runs/B060 \
  --media-manifest results/current_belt_capacity_20261128/publication/video_stage01/artifact_manifest.json \
  --visual-bundle results/current_belt_capacity_20261128/publication/visual_bundle_stage01/I82-visual-comparison.zip \
  --output results/current_belt_capacity_20261128/publication/full_evidence_stage02 \
  --build
```

The output `REPRODUCE.md` supplies restoration, verification and experiment
reproduction commands. The successful final `archive_verification.json` binds
the complete tar bytes and checked member count; the failed prior stage is
retained as part of the iteration record.
