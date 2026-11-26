# Frozen research workspace recipe

This directory makes the current I-52 native experiment runnable from a clone
without silently rewriting old tested files or their hash checks. Start with
[the teammate handoff](../docs/TEAMMATE_HANDOFF.md).

`frozen_workspace/` contains approximately 65 MiB of exact frozen source and
registered historical input files. These are preflight dependencies, not all
twelve original raw outputs. [The manifest](frozen_workspace/manifest.json)
records origin paths, stored paths, hashes, sizes and required live source hashes.
Sources are preserved for lineage even when newer source exists elsewhere.

From the repository root:

```bash
python scripts/restore-research-workspace.py --check
python scripts/restore-research-workspace.py
python scripts/prepare-research-campaign.py --output results/teammate_i52
```

The first command is read-only and returns nonzero if valid targets are missing.
Restore validates every payload/live-source/existing-target hash before copying
anything. It creates only missing permitted workspace files and refuses changed
targets. Preparation creates exact protocol/jobs, a fresh twelve-case resolver,
new-dataset metadata and manual run commands. It starts no simulator. Existing
campaign directories are refused.

Activate/build the Gazebo environment documented in [GAZEBO.md](../GAZEBO.md)
before executing the generated commands. The frozen runner restores its original
`results/` paths because its ROOT/HERE and source-hash semantics depend on them.
Do not execute archived docs snapshots directly or edit old protocol hashes to
make a changed implementation pass.

The frozen scientific reducers preserve **original-dataset assumptions**,
including the separately recorded native-crash retry. This package intentionally
claims native-run reproduction, not a generic fresh-dataset analysis launcher.
Each new run directly writes summaries, logs and provenance. Fresh full-study
analysis needs its own accurate dataset declaration and path adapter. No original
crash is invented for new data. Original complete raw bundles stay local; tracked
compact reports, selected videos and the exact event-window subset are available
through the handoff.

`workspace.py` is standard-library-only. Its integrity/path/copy tests are in
[test_research_workspace.py](../tests/test_research_workspace.py). Those tests
and restoration/preparation execute no controller, observer, gain design or physics.
