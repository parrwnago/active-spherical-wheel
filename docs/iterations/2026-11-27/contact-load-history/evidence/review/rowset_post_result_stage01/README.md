# I-64 post-result row-set clarification

This explanatory check was added after the registered I-64 results and after the counts were already observed. It is not a new acceptance gate, model, parameter choice or inference method. The original sealed review manifest and files remain unchanged.

M0 loses30 useful corrections and M2 loses4. Their lost-frame intersection is empty: all30 M0 rejections are restored, while M2 introduces4 different useful-correction rejections. The net reduction is26. The new4 are p030_B at 14.415s, p090_B at 17.530s, p090_B at 17.770s, p090_B at 24.955s (use the exact logged times and full-rate indices in report.json as authoritative).

The source replays the previously fixed shadow-policy semantics against immutable independent score outputs and the original primary/useful masks. It verifies the M0 keeper flags equal the original I-61 flags. `report.json` preserves every restored/new/lost row's case, full-rate index, exact timestamp, families and scores; `source_and_inputs_before_replay.json` binds source and inputs before this replay while explicitly disclosing prior outcome knowledge. No physics or controller change occurs.

Reproduce to a new report path from repository root:

```bash
.venv/bin/python results/contact_load_history_20261127/review/rowset_post_result_stage01/rowset_diagnostic.py \
  --root results/contact_load_history_20261127 \
  --output /tmp/i64_post_result_rowsets_new.json
```
