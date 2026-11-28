# I-76 metadata coverage amendment

The tested initial collector omitted `.jsonl` from compact suffixes. Recovery
stdout JSONL was absent from both compact selection and the raw list, including
required members of the final 149-file output seal. Selected-file archive QA
alone would not establish that restored evaluation was runnable.

This separate stage preserves the initial compact manifest, copies all review
JSONL with exact source/seal bindings, retains correction source and a focused
missing/corrupt-JSONL regression, and explicitly regenerates the final compact
manifest. The tested builder and lossless raw bundle stay byte-unchanged. No
physics, score, controller or video is rerun.

Run the ordinary archived restore QA on the amended record, then execute
`check_restore_closure.py` against that actual temporary restore. All 149
root-sealed output artifacts, including required recovery JSONL, must exist
with exact original sizes and hashes. Final QA receipts belong outside the
immutable compact evidence tree.
