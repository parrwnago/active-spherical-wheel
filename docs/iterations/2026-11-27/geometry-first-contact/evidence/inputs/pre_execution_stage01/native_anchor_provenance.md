# Native equivalent anchor N: pre-extraction source review

Recommended source: `measurement_context.npz`, field
`exact_estimated_contact_point_m`, in each frozen I60 native case.
`exact_estimated_support_loads_N` is available separately for fallback-only
native family recovery. Neither is a physical bank alternative or solver truth.

The source chain is explicit:

1. The guarded I58 native recording uses controller mode `hardware`, as recorded
   in its provenance. This name denotes the simulation's measurement boundary;
   there is no physical robot evidence.
2. In that mode, `offroad_control.command()` updates the momentum observer,
   passes its force/moment to `SupportLocator.records()`, and calls
   `measure_records()` on those inferred records. It reads no solved contact
   in that branch.
3. The diagnostic `estimated_contact_point_m` copies that inferred measurement's
   `point_m`; its family loads likewise copy inferred measurement loads.
4. The guarded logger prefixes these diagnostics with `exact_` in precision
   output. The native branch preserves the original controller diagnostics.
5. I60's explicit precision-field whitelist copies the exact estimated point
   and loads into its truth-free `measurement_context.npz`. Its pre-arithmetic
   registration names those fields and records no truth fields accessed.

`measure_records()` computes a normal-load-weighted mean of the located support
points. For a single patch this equals that located physical point. For two
rims it can lie between the physical supports, inside the sphere. Thus the
anchor must be copied directly, without normalization or projection onto the
sphere. It supplies no new independent sensing beyond the existing observer
and known-body geometry.

This is distinct from Nalt, the alternative physical point selected from the
candidate bank within the native family, and from `contact_point_m` or
`truth_equivalent_point_m`, which belong to evaluation. No mixed point-check or
evaluation packet is needed for the inference input.

The old native family rule is a unique positive maximum of finite nonnegative
estimated support loads, with total at least `finfo(float).tiny`; exact ties or
missing support remain -1. This rule is documented in I60's `dominant_labels`
source. I72 may reproduce it for fallback, but those loads/family labels must
not enter candidate ranking.

Source locations reviewed (source/provenance only, before saved I72 arithmetic):

- `src/spherical_wheel/offroad_control.py`: hardware locator construction,
  `command()` measurement branch and recorded estimated point/loads.
- `src/spherical_wheel/contact_sensing.py`: `SupportLocator.records()` and
  `measure_records()`.
- `results/constrained_handoff_20261126/campaign_tools/run_case.py` and the
  guarded I58 source snapshot: exact diagnostic-field copying.
- `results/velocity_contact_rank_20261127/campaign_tools/extract_features.py`:
  `ALLOWED_FIELDS`, direct whitelist copy and measurement-context output.
- I60 `registered_before_arithmetic.json` and I58 native provenance.

The final source/input seal will bind the actual files used for extraction.
