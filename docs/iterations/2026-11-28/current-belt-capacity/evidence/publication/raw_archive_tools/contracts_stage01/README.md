# I-82 full raw evidence packaging

This helper packages only the **closed** current-case 0.30/0.60 m/s comparison,
its exact registered source/input prerequisites, both recorded physics arms,
analysis and independent verification, and the full comparison/event videos.
It does not launch physics, render a video, change a cap, or promote a controller.

The tool walks only `results/current_belt_capacity_20261128/` and the exact
source/input paths in the frozen I-82 protocol. It never scans the surrounding
repository/home. Credential paths, unrelated user files and virtual environments
are outside its scope.

Before the final build, root must declare **both scientific proofs and media
closed**, and finish its stable `curation/` helper. Then run with tool stdout
directly, or put any outer shell log **outside the evidence family**:

```bash
.venv/bin/python results/current_belt_capacity_20261128/publication/raw_archive_tools/build_archive.py \
  --protocol results/current_belt_capacity_20261128/protocol/frozen_stage01/protocol.json \
  --seal results/current_belt_capacity_20261128/campaign/outputs_sealed.json \
  --seal-base results/current_belt_capacity_20261128 \
  --observed-exit B030_ACTUAL_OBSERVED_EXIT_JSON \
  --observed-exit B060_ACTUAL_OBSERVED_EXIT_JSON \
  --case030 B030_OUTER_DIRECTORY \
  --case060 B060_OUTER_DIRECTORY \
  --media-manifest CLOSED_MEDIA_MANIFEST_JSON \
  --output results/current_belt_capacity_20261128/publication/full_evidence_stage01 \
  --build
```

Both process exits must be actual observed zero return codes. B030 must finish
the full 133 seconds and reproduce all 79 original precision fields. B060 may
have a physical early stop; its failed/truncated physics and video must remain
explicitly preserved. Process completion is not physical completion or acceptance.
The helper checks identical original configuration, bank, ordinary design and
gain-cache identities, plus each arm's actual physical/mirror operating caps.

The source/input registrations and closed-output seal are checked before and
after construction. Every selected file is bound before archiving; every regular
tar member is stream-decoded and verified for exact size, SHA256 and member set.
Final checks reject changed or added included/excluded files. Receipts and the
current archive output are excluded from recursion; a failed attempt remains
preserved and must not be overwritten.

Explicit omission inventory retains path/size/hash/reason for interpreter/test
caches, repeated generated workspace mesh/terrain/visual assets, render scenes,
portable visual-bundle duplicate staging, prior archives and `curation/` files.
Original registered prerequisites override asset exclusions. Direct physical
model XML/SDF/configuration remains included. Original videos and closed media
metadata remain included even when duplicate visual-bundle copies are omitted.

Four pure packaging contracts use only temporary synthetic files. Their count
is separate from runner/evaluation contracts and physics experiments. Earlier
unsealed smoke repeats are not added. The final raw archive stays local; its
verification/inventory/restoration metadata can be copied into tracked compact
evidence after the build.
