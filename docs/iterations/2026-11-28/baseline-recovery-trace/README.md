# I-86 — baseline recovery with a 1 ms force and command trace

2026-11-28. **Completed diagnostic; native controller retained.** One fresh
Gazebo/DART prefix reproduces I-82 B060 through 106.5 s. All 79 precision fields
match across 21,301 rows, apart from the declared final unheld-command flag.
No controller, estimator, gain, reference, limit or physics setting changes.
This is not an intervention, full-route acceptance test or tested recovery fix.

## Watch and inspect

- [Slow-motion event video, 104.9–106.4 s at 0.2×](../../../../results/baseline_recovery_trace_20261128/publication/video_stage01/baseline_recovery_104p9_106p4_slow.mp4)
- [Force/recovery plot](evidence/analysis/completed_stage01/recovery_trace.png)
  · [PDF](evidence/analysis/completed_stage01/recovery_trace.pdf)
- [Primary report](evidence/analysis/completed_stage01/report.json)
  · [Independent force/contact check](evidence/verification/completed_stage01/report.json)
  · [Independent/primary numerical agreement](evidence/verification/primary_agreement_stage01/report.json)
- [Shareable video, reports and context ZIP](../../../../results/baseline_recovery_trace_20261128/publication/visual_bundle_stage01/I86-baseline-recovery-diagnosis.zip)

The clip uses recorded Gazebo/DART poses; MuJoCo is display-only. Telemetry
shows the preceding held commands separately from the new state-row command.
All 301 frames decode; encoded samples and identical robot geometry were checked.

## What changed in our understanding

**Multi-contact is not the explanation in this event.** Across the entire
105.0–106.3 s trace, all positive normal pressure is on shell A. Loaded solver
points span at most 1.2245 mm; there is no second loaded motor family or second
10 mm spatial group at any fixed 1%, 5% or 10% pressure-share sensitivity.
Several points in one small solver manifold are not independent supports.
This does not dismiss multi-contact on other terrain or during transfers.

**The initial unloading/reloading is real at the recorded solver resolution.**
Two completed 1 ms solves, ending at 105.525 and 105.724 s, have no exported
contact rows and zero ground force. Gravity accounts for COM acceleration at
the first. The next solve reloads the shell. This establishes isolated
contact-free solves, not a complete 5 ms airborne episode or its geometric cause.

**The initial kick does not explain the entire later velocity excursion.**
Over 105.525–105.530 s, wheel-centre velocity changes by approximately
[4.154, 3.666] mm/s horizontally, while whole-robot COM velocity changes by
[0.493, 0.454] mm/s. The remainder is wheel-centre motion relative to the COM.
Internal motion and reorientation matter; wheel-centre velocity cannot be
interpreted as whole-robot momentum divided by mass.

The much larger, later velocity residual peaks at 107.139 mm/s while the
native balance target changes from about 9.3° to 5.6° tilt. Commands remain
unsaturated, intended-path distance stays below 8.237 mm and native desired-up
error below 1.2583°. Rolling feedforward/preparation and belt commands are zero
in the focus window. Large opposing LQR error-block contributions can coexist
with a small net correction. Their magnitudes alone are not causal effects.

A separately registered [posthoc peak addendum](evidence/analysis/posthoc_interpretation_stage01/report.json)
finds the peak at 105.745 s. Wheel-centre X velocity is -122.048 mm/s, whole-COM
X velocity -55.847 mm/s and their relative term -66.201 mm/s. From 105.530 s
to this peak, signed X changes satisfy -106.982 = -38.555 - 68.428 mm/s.
This is a kinematic identity, not an independent causal partition. At the peak,
desired-up direction changes at 24.692°/s, actual up at 30.912°/s, while the
supplied angular-velocity reference remains zero.

**The momentum observer is accurate against the corresponding wrench here.**
Over the fixed 105.4–106.1 s focus window, completed-interval force discrepancy
is 0.000296 N RMS. Intrinsic normals computed from the same filtered physical
wrench agree to 0.00636° RMS. The controller's subsequently filtered normal
differs from the interval pressure direction by up to 4.499°; those are
different observables, not proof of an incorrect momentum estimate.

## Force balance, timing and limits

Capture 1,300 completed 1 ms solves over 260 unchanged 5 ms holds, plus 261
read-only native command/observer/preview snapshots. Whole-robot COM kinematics
are reconstructed offline from recorded states and exact inertial properties.
An independent implementation uses MuJoCo kinematics, without dynamics steps.

Across all 260 intervals, the independent contact-plus-gravity versus COM
momentum-rate residual is 0.000276 N RMS, maximum 0.001011 N. The primary
report's narrower 140-interval focus gives 0.000376 N RMS. These are different
windows, not conflicting results. Primary and independent reductions agree
over 52 array/vector comparisons comprising 53,131 scalar values, maximum
absolute difference 8.26e-13. These checks are not additional physical trials.

Forces times 1 ms are sampled impulse estimates, not exact exported solver
impulses. Use fixed world axes or the explicitly fixed 105.4 s downhill basis;
do not integrate changing-direction projections as one momentum balance.
New commands act after their recorded states. A new command at 105.530 s
cannot cause the velocity change already present at that state, although the
preceding held command may contribute.

## Decision and the next hypothesis

Keep native LQR, momentum observer, inferred-contact feedforward and preparation.
Terrain assistance stays parked. Defer multi-contact estimation work because
the selected recovery event has only one loaded patch.

The evidence narrows the next causal question to how the balance-reference,
holding and feedback response turns a short contact transition into a larger
wheel-centre velocity excursion. It does not establish which controller block
should change or that the necessary balancing motion can be eliminated.
An eventual test must separate the physical kick from subsequent recovery,
retain native-reference balance scoring and report both local and sustained
motion. One proposed causal ablation keeps the native angle rule, holding,
routing and gains unchanged and changes only its angular-rate reference to
match the changing desired-up direction. That isolates the zero-rate convention;
it is not a claim that supplying this rate improves recovery. I-43 changed the
target angle through smoothing as well as supplying a consistent rate and found
mixed results; that result remains relevant. No controller change or claimed
cure is included in I-86.

## Conditions, sources and reproduction

Original I-76 rough-stress Mars figure-eight, shaft start -1 rad, yaw 158°,
nominal 133 s path, diagnostic stop 106.5 s. Original 0.30-based model/config
and LQR gain bank; I-82 B060 physical/mirror operating belt bounds 0.60 m/s,
motor bounds [1.5, 0.6, 0.6] N m. Exact simulated body state is allowed.
Physical contacts and terrain are diagnostic-only, never inputs to this live
observer-based controller. No heading helper, retiming or camera assistance.

Physics registration binds 99 sources and 66 inputs before launch; protocol
SHA256 `e2255a7ef6bda5271cfb56fb34d26b3076c9e20239e5bfe5aecddf2db3dcf85f`.
The observed process exit is zero. The closed output seal binds 646 files,
SHA256 `399521947dfabf300ccfa549b85a5fa773db7ca4e0c4ab863306c53397818c0d`.
Analysis definitions and independent verifier were frozen separately before new
array reads. Later peak interpretation, if supplied, is explicitly posthoc.

Six unique runner contracts pass in both Python environments; repeated runs
are not added. Four primary reduction contracts, six independent diagnostic
contracts and three rendering contracts are separate scopes. No full repo
suite is claimed. Production code/defaults and older evidence remain unchanged.

Raw data, exact source and launch commands live under
`results/baseline_recovery_trace_20261128/`.
[Compact manifest](evidence/manifest.json) binds 70 files plus its manifest;
the source archive has 114 independently hash-verified members. The
[full local raw archive](../../../../results/baseline_recovery_trace_20261128/publication/raw_evidence_stage01/i86_full_evidence.tar.gz)
contains 1,335 size/SHA-verified project members, 172,449,228 bytes, SHA256
`47aae4ef3f0efc169b162c1baa9aeca445cb90a54689a04242baba1b1824425e`.
[Inventory](evidence/publication/raw_evidence_stage01/inventory.json),
[external prerequisites](evidence/publication/raw_evidence_stage01/external_prerequisites.json)
and [verification](evidence/publication/raw_evidence_stage01/verification.json)
retain original restoration paths. The installed simulator environment is not
an archive member. The visual ZIP includes revision21 standalone context and
the current iteration evidence; older source/history requires the repository.
To inspect closed evidence without new physics:

```bash
.venv/bin/python results/baseline_recovery_trace_20261128/verification/verify_trace.py \
  --run results/baseline_recovery_trace_20261128/campaign/native106p5_stage01 \
  --output results/baseline_recovery_trace_20261128/verification/NEW_REVIEW
```

The recorded launcher executes a new physical prefix; it is not needed to
review saved data. Use new output directories and restore prerequisite paths
from the source/archive inventory before reproduction.
