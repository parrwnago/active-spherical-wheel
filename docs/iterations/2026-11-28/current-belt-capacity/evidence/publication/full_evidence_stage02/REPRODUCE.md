# I-82 full closed capacity-comparison evidence archive

This preserves two newly recorded Gazebo/DART executions of the same I76
figure-eight, shaft start -1 rad, yaw158 degrees, original133-second reference:
B030 uses0.30 m/s and B060 uses0.60 m/s operating limits on both belts. The
original0.30 design/configuration, gain bank, torque limits, observer and
controller algorithms remain fixed. Operating caps change after design
construction in both physical plant and measured-state mirror. Native incoming
preparation sees those caps too; this is not a final-clipping-only intervention.

Full-route completion, process completion and acceptance are distinct. B030
must reproduce all79 original precision fields through26,601 rows exactly.
B060 can legitimately stop early for a native physical failure; its arrays,
actual end and failure record remain included. A finished process does not
make an early physical stop successful. Higher-cap histories can diverge well
before the user's101.5-second event; this is a whole-run capacity comparison,
not a same-state local counterfactual. No cap/gain tuning follows outcomes.

`project/` holds original repository-relative paths. `_archive/` holds the
pre-archive registration, excluded-file inventory, prerequisite inventory,
this guide and member inventory. Historical absolute paths are preserved;
locate corresponding bytes at project/<repository-relative path>. Restore
captured source at its recorded path for read-only analysis into NEW stages.

All exact sources and inputs from the frozen I82 protocol are included,
including registered original I76 records, terrain/gain/plugin inputs and
runner sources. Original whole-campaign seals may reference unrelated runs;
only listed I82 prerequisites are claimed captured. No virtual environment,
credential helper, unrelated firmware or unrelated user file is included.

Four separately registered analysis/media source dependencies also appear at
their ORIGINAL repository-relative paths: original I76 evaluation/metrics.py,
I81 analysis/core.py, I76 publication/video_tools/render.py and I78 publication/
render.py. Their exact historical hashes and closed registration receipts are
listed in _archive/supplemental_dependencies.json; these are not retroactively
added to the physical experiment's93-source/59-input registration.

Repeated generated workspace mesh/terrain/visual assets, render scene caches,
interpreter caches and prior archives are omitted with path/size/SHA/reason
inventory. Direct physical/model XML/SDF/config files remain included. Exact
registered prerequisites override asset omission. New physics on a rebuilt
environment is not promised bit-identical to the captured recordings.

The completed portable visual ZIP and its closed metadata ARE included as an
explicit exception to prior-archive omission. Duplicate expanded movies/images
inside its staging folder are omitted because canonical media is included.

Start with:

- project/results/current_belt_capacity_20261128/protocol/frozen_stage01/protocol.json
- .../campaign/runs/B030/outcome.json and native_comparison.json
- .../campaign/runs/B060/outcome.json and native_comparison.json
- .../campaign/runs/ARM/child/campaign/runs/eight_m100_N/precision.npz and rollout.npz
- .../campaign/runs/ARM/substeps.npz and capture_receipt.json
- .../evaluation/ or .../analysis/ completed reports and .../verification/
- .../publication/ recorded full-route MP4, event clip and media metadata

Outer I82 arm/cap receipts are authoritative; inner child I76 job/cap labels
are inherited runner metadata. The full video and event clip use the same
physical elapsed time at1x. Recorded Gazebo/DART states use MuJoCo visuals
without replaying physics/controller/observer. A failed or ended panel freezes
visibly at its actual endpoint. Motion and acceptance are not rescaled.

The short100.5–101.8s contact capture splits each originalrun(5) into five
run(1) calls without extra control updates. B030 exact native parity checks
that instrumentation preserves original motion. B060 partial capture is
retained if physical termination precedes its end. Saved contacts are sampled
solver forces, not an exact impulse oracle. Each emitted command acts AFTER
its recorded state; prior held commands correspond to completed forces.

Verify all members against _archive/archive_inventory.json. The inventory
does not list its own digest; adjacent archive_verification.json binds it and
records one complete streaming member-hash verification, unchanged frozen
sources/inputs/output seal and unchanged evidence/omission selection.

Read-only captured-array analysis requires no new physics. Consult each
registered_before_*.json and README for exact commands and use empty output
stages. The runner's --run explicitly launches new physics and is unnecessary
for inspecting this evidence. Do not overwrite closed records.
