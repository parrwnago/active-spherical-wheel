# Full I-78 local evidence archive

`build_archive.py --plan` checks the inclusion policy and all registered
source/input/output bindings without creating an archive. `--freeze` must run
only after the root agent declares all scientific verification/analysis closed
and its separate curation helper has been written. It creates one fresh archive
and refuses to overwrite an existing capture.

The archive contains all I78 raw records, source snapshots, contracts and setup
failures, canonical analysis/corrections, independent proofs, model/config files,
videos and publication receipts. All 100 frozen canonical sources and 66 inputs
are also included, with original I76 prerequisite path/hash inventory. The
excluded-file inventory records rebuildable workspaces/render scenes, bytecode,
duplicate visual-bundle copies/archives and separate repository curation files.
No broad repository/home search or credential-helper access occurs.

Every selected input is bound before archiving, every regular member is fully
streamed and hash-checked afterward, and all bindings and the selected file set
are checked again. No simulation, controller, observer or raw-array loading is
part of archive creation. `archive_verification.json` is the final receipt;
`REPRODUCE.md` explains retained paths and reproduction limits.

The three focused contracts in `test_archive.py` check direct-model/cache scope,
complete member hash verification and rejection of duplicate/escaping members.
They operate on temporary synthetic byte streams, without rollout arrays.
