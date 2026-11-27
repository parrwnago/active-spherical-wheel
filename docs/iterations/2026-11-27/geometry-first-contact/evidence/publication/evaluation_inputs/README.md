# Self-contained I-72 evaluation inputs

These six compact NPZ files contain exactly the frozen
`evaluation/prepare_evaluation.py` whitelists:

- `old_i70/CASE/evaluation_packet.npz`: its 10 `BASE_FIELDS` arrays.
- `old_i60/CASE_evaluation.npz`: its 9 `OLD_FIELDS` arrays.

All 57 arrays across the three cases preserve the original dtype, shape and
bytes. This includes the original acceptance fields, family labels and fixed
evaluation-domain inputs; no confidence, domain or label is recomputed during
packaging. The source files match those bound before the actual I-72 truth
projection. Total NPZ payload is 2,857,930 bytes.

`registered_before_copy.json` binds the copy tool, frozen projector, original
pre-truth seal and all six original files before copying. `metadata.json`
records every original file SHA256, compact file SHA256 and per-array binding,
plus source/input preservation and byte-equality checks after archive reload.
NPZ compression bytes differ from the larger originals because non-whitelisted
arrays are omitted. `package_inputs.py` is copy-only provenance tooling; it
requires the original repository files if rerun. Replay and evaluation use the
compact inputs directly and need no parent iteration directory.

From the published `evidence/` root, first complete the strict inference replay
with fresh output paths:

```sh
python publication/portable_tools/replay_inference.py \
  --package . --output /tmp/i72_prediction_replay

python evaluation/prepare_evaluation.py \
  --baseline-root publication/evaluation_inputs/old_i70 \
  --old-evaluation-root publication/evaluation_inputs/old_i60 \
  --prediction-root /tmp/i72_prediction_replay \
  --inference-seal inference/all_three_sealed.json \
  --output /tmp/i72_fresh_evaluation_packets

python evaluation/evaluate_selector.py \
  --protocol protocol/protocol.json \
  --packet-root /tmp/i72_fresh_evaluation_packets \
  --output /tmp/i72_fresh_evaluation
```

The inference-seal argument is the original all-three canonical completion
receipt. The preceding strict replay separately verifies that every fresh
prediction array equals that canonical inference. Both scientific evaluation
programs are unchanged. They read evaluation-only truth here; these files are
never inputs to contact selection or ranking. No simulation or new fitting is
performed by these commands.
