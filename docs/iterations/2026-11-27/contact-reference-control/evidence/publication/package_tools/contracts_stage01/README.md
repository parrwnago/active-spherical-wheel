# I-73 compact record and full raw replay inputs

The compact evidence tree preserves the protocol, exact pre-physics source
snapshot, failed map stages and their revisions, test chronology, nine small
run reports/process logs, evaluation sources/science, final render sources and
figures, and the original frozen gain-cache bytes. It does not contain the full
precision/rollout/signal files or runtime libraries/video files.

`publication/plot_samples_50ms/` contains explicitly **plot-only** decimation:
every tenth recorded 5 ms frame, with its original row index and actual terminal
state retained. No extrapolation or metric replacement occurs. These samples
cannot reproduce full-rate peaks, settling gates or acceptance decisions.

`publication/local_raw_bundle_inventory.json` identifies a separate local,
shareable tar bundle and hashes each original member. That lossless bundle
contains the full raw inputs required for fresh evaluation, full-rate signal
outputs, large video files and the two registered runtime libraries. Libraries
are provenance/reproduction dependencies and are never installed by restoration.

From a repository checkout, restore a published compact record to the dedicated
result path, using an evidence path appropriate to that checkout:

```sh
python docs/iterations/2026-11-27/contact-reference-control/evidence/publication/package_tools/restore_record.py \
  --evidence docs/iterations/2026-11-27/contact-reference-control/evidence \
  --repo-root .
```

Add `--raw-bundle /path/to/i73_full_raw_stage01.tar.gz` to restore the byte-exact
large **result** files needed for full evaluation. Add `--install-gain-cache`
only to copy the one registered frozen cache into
`generated/offroad/flat/control_schedule.npz` if absent. Differing existing
files are refused; identical files are retained. Existing symlinks along a
restore destination are refused. No live source, plugin or unrelated repository
file is overwritten. `--check-runtime` additionally verifies all original
source/assets/binary/cache bindings without installing missing dependencies.
`--verify-only` checks the record and restore conflicts without writing files.

After restoring the full raw bundle, fresh evaluation uses the frozen original
NumPy programs and full precision files:

```sh
python results/contact_reference_control_20261127/evaluation/evaluate.py \
  --protocol results/contact_reference_control_20261127/protocol/protocol.json \
  --runs-root results/contact_reference_control_20261127/campaign/runs \
  --all-runs-seal results/contact_reference_control_20261127/campaign/outputs_sealed.json \
  --output /tmp/i73_fresh_evaluation
```

The output must be new. Sources and full inputs remain unchanged. Reproduction
does not turn a failed control screen into acceptance. An archived incomplete
run remains incomplete. Rerunning physics requires the separately verified
Gazebo/DART environment and a new registered output campaign; restoration never
starts physics or overwrites canonical run folders.

`verify_released_inputs.py` hashes all original runtime sources, registered
inputs, gain/library bytes and all sealed case artifacts without opening case
arrays. `build_record.py` is curator tooling. It is executed only after the
root's second, post-render release, with the released all-nine seal hash, into
a new archive stage and new local raw tar. Its source/input registration precedes
copying or decimation. Raw files and science sources are not changed.
