# I57 saved-response analysis

No script in this directory imports a controller, observer, MuJoCo or Gazebo. Native endpoints are read only after execution. Current registered analysis source is `frozen_stage02/`; `frozen_stage01/` is preserved with the first synthetic plotting attempt. The sole source change was shortening an overlong colorbar label after visual inspection found clipping in the single-event synthetic figure. The numerical scoring and metric source are unchanged between these freezes.

Eight focused pure arithmetic checks passed (angular-frame invariance, common-frame projection, unresolved and false-zero cases, sign, exact response and half/full linearity). A fabricated 15-branch schema fixture exercised the complete reducer and plotting path with exact controls. It is synthetic data, not physical evidence; neither these checks nor repeated fixture runs are a repository test-suite count. Both synthetic reduction attempts are retained, with the second figure visually checked.

After the parent confirms the native campaign is complete, reproduce using the exact final prediction stage:

```bash
.venv/bin/python results/loaded_contact_validation_20261126/analysis/reduce_responses.py \
  --study results/loaded_contact_validation_20261126 \
  --model-stage results/loaded_contact_validation_20261126/model_tools/predictions_stage01 \
  --output results/loaded_contact_validation_20261126/analysis/stage01 \
  --frozen-source results/loaded_contact_validation_20261126/analysis/frozen_stage02
```

Output includes pre-arithmetic source/input hashes, exact source snapshots, a protocol copy, full endpoint/model/continuation projections in `portable_inputs/`, paired response arrays, JSON report, response and linearity CSV tables, and scientific PNG/PDF figures. Full prefix arrays stay local; report entries retain their hashes and per-field bytewise parity checks. Missing or failed branches remain explicit. No event is physically interpretable unless its continuation and governed duplicate controls pass exactly and every available branch prefix matches. Continued original commands and the duplicate are controls, not response-accuracy probes.

Scoring uses the frozen engineering floors and thresholds. Unresolved small signals do not pass directional accuracy. The transverse predicted-nearzero absolute test is separate from resolved-vector scoring. All endpoint support changes remain in the analysis. Contacts are last-solve five-millisecond endpoint snapshots and do not certify continuity within a hold or integrated impulses. Individual hinge groups avoid hiding a bad loaded-rotor response behind a large free-rotor response.
