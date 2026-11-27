# I-59 — test the retained startup-feedback explanation

2026-11-26. Offline analysis of the six I-58 B/P recordings; **zero new physics
runs and no controller or default change**.

The retained startup shaft-gain row does **not** produce a material command
difference at these recorded states. This corrects the earlier suggestion that
startup-specific feedback settings might substantially explain the different
trajectories. Source code permits that path, but the actual three designs have
almost identical shaft rows and essentially zero equilibrium commands.

[Comparison figure (PDF)](evidence/analysis/plots_stage01/anchor_effect.pdf)
· [PNG](evidence/analysis/plots_stage01/anchor_effect.png).

## Question and fixed comparison

The user observed that the -1 rad start often looked better. Starting phase
changes the physical configuration, but `reset(phase)` also selects an ordinary
local LQR design. `_terrain_feedback` retains that design's shaft row while
scheduling the other four command rows using current phase/support. We tested
whether this retained row itself explains meaningful command differences.

At every saved state, keep motion, inferred contacts/loads, normal, transformed
feedback error, references/history, gain rows 1–4, feedforward, preparation
targets/weights, heading and unloaded fade fixed. Swap only shaft row zero among
the -1.0, +0.3 and +0.9 rad designs. Trim-only and row-plus-trim swaps are separate
secondary diagnostics. Do not reset the controller or replace the whole design.

The six datasets are native B and experimental P at all three starts, each
with 8,801 timestamps from 0–44 s. The 52,806 rows are correlated saved states,
not new trials. All anchors and fixed full/moving/goal windows are included;
fixed event windows are 4.605–5.105, 35.675–36.175 and 43.75–44 s. The terminal
command was recorded but not subsequently held.

The output boundary is **native request and native amplitude-clipped command**.
For B this is the actual input pipeline. For P it is a native-policy shadow at
the observed P states; this audit does not resolve P's subsequent allocator or
claim counterfactual final P commands.

[Protocol](evidence/protocol/protocol.json) and
[input/source seal](evidence/protocol/input_source_seal.json) were frozen before
numeric outcomes. Absolute engineering resolutions are 0.001 N m per torque
and 0.0001 m/s per belt; the separate normalized resolution is 0.001 of a cap.
These are screening resolutions, not statistical significance or benefit gates.

## Result and correction

The startup rows and trims are exactly constant within every trace; B/P pairs
at the same start agree byte-for-byte. Across the three starts, the largest
shaft-row coefficient difference is 5.6831e-8; the largest equilibrium-command
difference is 5.0397e-13 N m. They are numerically different, not byte-identical.

Multiplying those row differences by every actual logged error gives a maximum
pre-fade shaft-torque difference of **2.52744e-8 N m**. Preparation never blends
the shaft command; fade lies in [0,1] and amplitude clipping cannot enlarge a
command difference. This is consequently an upper bound on both the native
requested and clipped shaft effects, about **40,000 times below** the declared
0.001 N m resolution. Other inputs have exactly zero row-only perturbation.

| Recorded start / trace | Approximate shaft-effect upper bound over all anchors, N m |
| --- | ---: |
| -1.0 B | 1.819e-9 |
| -1.0 P native shadow | 1.163e-9 |
| +0.3 B | 1.932e-9 |
| +0.3 P native shadow | 2.965e-9 |
| +0.9 B | 2.411e-8 |
| +0.9 P native shadow | 2.527e-8 |

[Coefficient and unfaded-effect report](evidence/analysis/coefficient_stage01/coefficients_and_pre_fade_effects.json)
retains exact values; compact full-rate inputs are archived separately. The bound does not require
treating a rounded fade factor as exact. Trim differences are also far below
the fixed resolution; adding them cannot change this conclusion.

The independent full-pipeline reconstruction gives:

| Intervention | Maximum native-request change, N m | Maximum native-clipped change, N m |
| --- | ---: | ---: |
| Shaft row only | 2.52744e-8 | 1.93331e-9 |
| Equilibrium trim only | 5.03972e-13 | 5.04041e-13 |
| Row plus trim | 2.52746e-8 | 1.93306e-9 |

The maxima are in torque channels; belt changes are zero. No state crosses
either absolute or normalized resolution, even including fade uncertainty.
[Independent full-pipeline report](evidence/review/outcome_stage01/report.md).
Reported magnitudes are rounded; analytical increments and subtraction of
floating-point command arrays differ by a few e-15 N m in the headline maximum.

**Correction to the earlier explanation:** the selected shaft row is retained
from startup, but its practical numerical effect is negligible in this audit.
We should not explain the observed trajectory ranking as different retained
startup LQR gains, or change that row as the next fix on this evidence.

## Command reconstruction and checks

The native pipeline is trim + rolling/holding feedforward - K error,
preparation blend, heading addition, unloaded fade, then amplitude clipping.
With fixed preparation targets and weights `w`, and torque fade `f`, the exact
fixed-state perturbation is:

```text
delta_pre = delta_trim - delta_K * recorded_error
delta_request = diag(f,f,f,1,1) * (1-w) * delta_pre
new_native_applied = clip(old_native_request + delta_request, -caps, caps)
```

The additive preparation and fade corrections must change with their incoming
commands. The generic clipping delta in P logs belongs to final P output;
native reconstruction uses `exact_native_clipping_command_delta` instead.

Independent stage-by-stage checks cover all six traces. The scalar stage helper
closes to 1.39e-17; a separately vectorized reconstruction including full K/error
products closes to 1.42e-14 command units. Both are below the fixed 1e-10
numerical tolerance. A zero intervention reproduces recorded native requests
and applied commands byte-for-byte. Fade telemetry is float32; exact stage
deltas constrain its factor, and conservative factor intervals are retained.
All states are identifiable under the declared control criterion, but this
does not make the rounded telemetry exact.

[Source mapping](evidence/command_tools/SOURCE_MAPPING.md),
[line-number erratum](evidence/command_tools/SOURCE_MAPPING_STAGE02_ERRATUM.md),
[independent stage controls](evidence/command_tools/saved_control_stage01/report.json)
and [blind design review](evidence/review/design_stage01/report.md) preserve the
mapping, checks and chronology. Minor setup/path corrections are retained in
the protocol supplement. No runtime source was edited.

[Canonical full-window results](evidence/analysis/pipeline_stage01/science.json)
and the [independent comparison](evidence/review/canonical_stage01/report.json)
agree across all 54 dataset/anchor/intervention combinations and six overlapping
fixed windows. These are correlated algebraic comparisons, not additional tests
or robot trials. Numerical round-off differences leave every classification
unchanged.

## What remains unresolved

The complete I-58 trajectories genuinely depend on initial configuration, and
the cross-start screen still fails. This audit isolates only the retained
shaft row and equilibrium trim at already-observed states. It does not alter
mechanics, regenerate references, change current support scheduling, or evolve
new trajectories. It cannot prove that arbitrarily small perturbations never
accumulate over a different simulation.

The next useful diagnostic is the **first divergence in current support,
scheduled gains, attitude/velocity errors and evolving references** across
starts, before the large later excursion. That targets the mechanical and
history-dependent paths left outside this test. It remains a proposed follow-up;
no new causal explanation or controller improvement is claimed here. Native
LQR, observer, rolling/holding feedforward and kinematic preparation stay selected.

Earlier -1 rad success was not universal: in I-52, reference-guard RMS was
76.789 mm at -1 versus 44.213 mm at +0.9. Preserve the
[I-58 mixed results](../balance-priority/README.md) and
[I-52 cross-start table](../contact-scheduling-joint-tracking/README.md).

## Reproduce the primary result without Gazebo

Python 3 and NumPy suffice. From the repository root, write a new output file:

```bash
python3 docs/iterations/2026-11-26/startup-anchor-audit/evidence/analysis/replay_coefficients.py \
  --data-source docs/iterations/2026-11-26/startup-anchor-audit/evidence/analysis/coefficient_compact_stage01 \
  --output /tmp/i59-coefficient-replay.json
```

The replay verifies archive hashes, recomputes all six effect tables and compares
every saved delta array byte-for-byte. The compact inputs preserve every
original timestamp/error/anchor and primary effect, without downsampling.
[Projection bindings](evidence/analysis/coefficient_compact_stage01/projection_binding.json)
link them to the richer original arrays. Per-coordinate contributions are
recomputed rather than duplicated in the repository.

The detailed native-pipeline and independent raw-log analyses also retain their
sources, reports, input hashes, controls and local replay receipts. Their larger
input/counterfactual arrays stay in ignored
`results/startup_anchor_audit_20261126/`; those reruns require the listed local
originals and are not advertised as standalone clone replays. The portable
manifest inventories these intentional omissions. Core coefficient/effect-bound
reproduction needs only the tracked compact archive above.

The [published-archive replay receipt](evidence/publication/published_coefficient_replay.json)
confirms exact effect-table and delta-array reproduction. The
[publication checks](evidence/publication/publication_qa_stage02.json) verify
archive hashes, new documentation links and unchanged runtime sources.
[Portable manifest](evidence/portable_manifest.json).
