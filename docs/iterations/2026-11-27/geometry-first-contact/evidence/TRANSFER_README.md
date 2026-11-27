This directory preserves the final I-72 local result tree using identical relative
paths and file bytes. `export_plan.json` was written before source files were
copied. `export_manifest.json` lists every included source file and every omitted
file with its original SHA-256, size, and reason. `transfer_receipt.json` binds the
plan, manifest, and original artifact-binding audit.

Only interpreter caches and the three explicitly identified regenerated replay
prediction files were omitted. Canonical inference predictions, full input
packets, the compact historical evaluation-input copies, both plot stages,
preliminary test snapshots, and the independent verifier's failed first stage
and corrected second stage are retained. The source experiment was unchanged.

Recorded manifest paths are resolved against their manifest folder, with an
explicit experiment-root fallback for original root-relative paths. Paths with
the `results/geometry_first_contact_20261127/` prefix refer to the experiment
root. Original references outside that root are metadata only and were not
opened. Earlier failed-stage source seals are preserved with their snapshots;
they are not treated as declarations about subsequently corrected live files.

The audit verifies original in-scope artifact file bindings, final inference
source/input references within the experiment, plot output bindings, and all 57
fields in the six compact evaluation-input copies. Copying and byte verification
do not recompute scientific scores, labels, fits, inference, or metrics.

The transfer helper, plan, audit, and this note are generated publication support
files. The manifest excludes itself and its receipt to avoid recursive hashing.
The outer publication archive may bind all support files, including the run log.

Reproduction of the transfer (to a fresh evidence directory containing only the
helper and this note) uses:

```bash
python3 transfer_evidence.py --source /absolute/path/to/results/geometry_first_contact_20261127
```
