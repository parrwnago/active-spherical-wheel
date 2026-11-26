# Frozen I52 prerequisites

This directory preserves the exact original protocol, jobs, experimental helper
sources and 17 registered prerequisite data files. `manifest.json` binds every
stored file to its original repository-relative path, byte count and SHA-256.
Its source list also binds the live controller, model, plugin and setup files.
The payload is about 65 MiB; it does not include the twelve original raw cases.

From the repository root, use an ordinary Python interpreter:

```bash
python scripts/restore-research-workspace.py --check
python scripts/restore-research-workspace.py
python scripts/prepare-research-campaign.py --output results/reproductions/i52_fresh
```

The first command is read-only and returns 1 when valid targets are missing.
Restore verifies all inputs before copying, creates missing files only, accepts
identical files, and refuses different existing files. It writes only beneath
ignored `results/` and `generated/offroad/`. The I40 helper snapshot is checked
against the live script and is never restored over it. No command here runs a
controller, changes a default, builds a gain bank or starts a simulator.

Preparation refuses an existing output directory. It copies the unchanged
protocol/jobs and registers twelve planned case paths, no technical attempts,
and zero native executions. `RUN_COMMANDS.md` lists the individual commands to
run after the Gazebo environment, plugins and pinned Mars assets are set up.
Use the documented numerical environment, one BLAS thread and at most three
fresh case processes. Exact cross-platform trajectory parity is not promised.

Run frozen source only from its restored original path. Running a source copy
directly inside this directory or historical `docs/.../evidence/` changes its
path-derived repository root and is unsupported.

The frozen analysis sources are included for understanding and lineage. They
retain their original raw-data/provenance/retry assumptions. They are not a
fresh-campaign analysis launcher. A fresh reduction needs a separately declared
path adapter and all its required outputs; do not copy the original study's
technical-crash interpretation into a new dataset. The canonical original
reports and the unsuccessful outcomes remain in `docs/iterations/`.
