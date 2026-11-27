This package replays I-69's cached numerical analysis using Python and NumPy.
It makes no solver calls and starts no simulator or controller.

From this directory:

```sh
python3 portable_tools/replay_capture.py --package portable_inputs --output /tmp/i69-replay-new
```

The output directory must not already exist. Against the archived canonical
`bench/native_capture_stage01` output, strict verification is:

```sh
python3 verify_replay.py --reference ../bench/native_capture_stage01 --actual /tmp/i69-replay-new --output /tmp/i69-strict-new.json
```

The producer used NumPy **1.26.4** and reproduced the entire report and all
**220** output arrays byte-for-byte. NumPy **1.21.5** keeps all mathematical
input values and discrete decisions exact, but its floating-point reductions
produce small differences in derived force projections, norms and KKT values.
Its original strict failure is retained. Separate, authorized validation accepts
only these listed derived operations within
`1e-10 * max(1, abs(reference))`; it never changes a computed value:

```sh
python3 verify_portability.py --reference ../bench/native_capture_stage01 --actual /tmp/i69-replay-new --package portable_inputs --output /tmp/i69-bounded-new.json
```

For the recorded NumPy 1.21.5 run, the maximum array difference was
`1.39e-17` and maximum report difference `3.55e-15`. This run passed the bounded
portability check and was **not byte-exact**. Its flags, IDs, counts, index arrays
and original mathematical input bindings remained exact.

The compressed data totals **28,319 bytes**, plus metadata. It contains all
160 captured records, preserving every scalar and all mathematical matrix and
other array values. Uninitialized matrix-padding columns are deliberately
discarded and replaced with zero padding. The manifest retains the original
local binary hash and per-field bindings, while making no raw-file-byte
restoration claim. Padding is not used by the frozen scientific arithmetic.

Only the **20 fixed substeps** needed for matching are stored from each trace.
Their original global step/contact indices are preserved. The in-memory
matching adapter has unknown portions marked NaN or integer−1, with explicit
observation masks. These are partial-context inputs, not restored full traces.
The new/historical trace file identities and selected-value bindings remain
separate; identical selected payloads are deduplicated only after comparison.

Full 44-second native precision/trace parity was checked separately during the
campaign. Those receipts are preserved as provenance; this compact replay does
not rerun that full-history comparison or demonstrate a control improvement.
The unchanged decoder, checker and analyzer are included with hash bindings.
