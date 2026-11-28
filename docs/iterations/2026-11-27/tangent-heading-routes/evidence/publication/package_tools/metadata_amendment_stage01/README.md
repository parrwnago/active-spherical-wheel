# I-76 metadata coverage amendment

The tested initial collector omitted `.jsonl` from compact suffixes. Recovery
stdout JSONL was absent from both compact selection and the raw list, including
required members of the final 149-file output seal. Canonical per-case model
YAML snapshots were also omitted by the nonrecursive per-case selection; the
unchanged renderer needs all18 snapshots. Selected-file archive QA
alone would not establish that restored evaluation was runnable.

This separate stage preserves the initial compact manifest, copies all review
JSONL with exact source/seal bindings and all18 model YAML files bound to
sealed prephysics provenance and protocol config hashes. It retains correction
source and one focused missing/corrupt-JSONL/YAML regression, and explicitly regenerates the final compact
manifest. The tested builder and lossless raw bundle stay byte-unchanged. No
physics, score, controller or video is rerun.

Run the ordinary archived restore QA on the amended record, then execute
`check_restore_closure.py` against that actual temporary restore. All 149
root-sealed output artifacts, including required recovery JSONL, must exist
with exact original sizes and hashes, plus all18 saved model YAML inputs.
Registered result-owned prerequisites for unchanged evaluation, plotting, audit,
replay and rendering are reviewed and checked in that actual restored fixture. Final QA receipts belong outside the
immutable compact evidence tree.
