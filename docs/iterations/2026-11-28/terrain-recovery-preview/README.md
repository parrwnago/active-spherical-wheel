# I-84 — the map covers this event, but incoming preparation is inactive

Status: completed causal depth-memory and command-opportunity audit.
**Registered decision: stop this candidate before a live trial.** No new
physics, observer update, controller command execution or driving improvement
is claimed. All action differences below are fixed-history shadow proposals.

## Result and reason

Previously delivered four-camera observations supply complete accepted future
centre/foot fits at **all 261 focus anchors** over 100.5–101.8 s, and all
901 dense anchors over 99.5–104 s. Nevertheless, **both native and mapped
preparation weights are zero at every dense anchor**. The corresponding
shadow applied-command difference is exactly zero. This includes the lead-in
before the focus; the absence of a command change is not just an artifact of
checking after the intended anticipation time.

| Registered audit quantity | Result |
| --- | ---: |
| Saved native control rows traversed | 26,601 |
| Unique audited anchors | 1,158 |
| Uniform / dense anchors, overlap | 267 / 901, overlap 10 |
| Focus anchors | 261 |
| Focus anchors with accepted future support fits | 261 / 261 |
| Focus accepted unique patch/pose queries | 4,176 / 4,176 |
| Dense accepted unique patch/pose queries | 14,416 / 14,416 |
| Focus or dense anchors with nonzero preparation blend | 0 |
| Focus qualifying applied-command opportunities | 0; required at least 10 |
| Maximum native reconstruction error | 0 |
| Maximum current load/velocity/margin guard error | 0 |

The narrow mechanism changes preparation of an **incoming unloaded drive**.
It has no eligible action in this event, even with accepted mapped geometry.
This rules out useful action through this interface on the tested recorded
history; it does not reject depth maps or establish that a loaded controller
could not benefit from terrain information. Accepted fits also do not prove
future contact identity or actual load distribution.

The method can change preparation elsewhere: on 267 uniform anchors it has
94 accepted-support/nonzero-blend cases, 50 meeting the same normalized applied
change threshold. Those events were not selected for an unregistered live
trial after seeing their outcomes. At 44 uniform anchors no selected future
patch has an accepted complete fit; all future patch/pose queries are accepted
at 222 of 267 anchors. All unknown and rejected queries remain counted.

## Evidence

- [Two-page terrain-preview PDF](evidence/publication/completed_stage02/terrain_preview_shadow_assessment.pdf)
- [Coverage, preparation and command timeline](evidence/publication/completed_stage02/focus_shadow_applicability.png)
- [Previously delivered depth points and future query locations](evidence/publication/completed_stage02/causal_depth_queries_100p5.png)
- [Independent audit](evidence/verification/completed_stage01/report.json)
- [Registered gate and completed audit report](evidence/audit/stage01/report.json)
- [Separately registered descriptive decomposition](evidence/analysis/decomposition_stage01/report.json)
- [Observed process exit](evidence/audit/stage01/observed_exit.json)
- [Frozen registration](evidence/protocol/frozen_stage01/registration.json)
- [Static contract receipt](evidence/contracts/stage01/receipt.json)

## Candidate and information boundary

The user approved investigating both recovery and anticipation. Following the
[I-83 recovery assessment](../recovery-assessment/README.md), this narrow
experiment asks whether remembered terrain can usefully change the existing
incoming-drive preparation. Both comparison arms would retain the original
LQR gains, momentum observer, current contact/load inference, holding feedback
and 0.60 m/s operating belt limits. There is no heading helper or path retiming.

Four virtual depth views, capture-time exact simulated pose, 20 Hz capture and
50 ms delivery feed observed-only terrain memory. Future patch support queries
use delivered observations at the current anchor time. Current contact geometry,
actual tangent velocities and load guards stay observer based. Missing or
inconsistent centre/foot map fits fall back to the exact native preview.
The existing 120 ms constant-velocity horizon, activation, blend and command
bounds are retained. The map predicts geometry, not actual load transfer.

The registered gate requires native reconstruction and preservation of current
load/velocity/margin values within 1e−9, plus at least ten focus anchors with
accepted selected future support, a nonzero preparation blend, and normalized
applied-command change at least 0.001 using the same actual bounds. It is fixed
before the audit and remains failed. A larger prediction count or fit coverage
alone cannot pass it. The shadow calculation retains original heading/fade
and final clipping when computing a prospective applied command.

The crop comes from analytic route/domain bounds plus optical range margin,
not future executed positions. The optical adapter sees declared collision
geometry and modeled robot occlusion; the mapper receives only finite-view
depth/calibration/poses/timestamps, never hit IDs, normals or solved contacts.
Only captured and delivered observations may enter memory at each anchor.

Captured 2,661 four-camera batches contain **10,644 frames**. Of those batches,
2,660 are delivered by the final state and one remains pending; the last capture
is not made available early. Capture-time localization and depths are ideal,
the terrain is static, and the preview uses constant velocity. This is not
SLAM, a hardware sensor demonstration, exact future contact-force prediction
or a dynamic load-transfer model.

## Integrity, limitations and preservation

The registration SHA256 is
`bb8712febb1e5a57b0f274806c88482fa7a253d2d69b977d254ba7379f6d404b`.
It binds 100 sources and 191 inputs, including all referenced mirror/optical
model assets, source geometry and exact CAD inputs. It also verifies unchanged
I-82 93-source/59-input registration and 456 sealed outputs. The optical scene
and its asset closure are generated/sealed before recorded arrays are opened.
Eight focused static contracts pass; they are not physical trials or a full
repository suite.

Independent reconstruction checks every saved anchor's native block sum,
preparation blend, fade, clipping, current-state guards and prospective applied
command. Its largest native residual is 1.74e−18 and shadow-command residual
6.94e−18. Capture poses, four-camera calibration, validity, ranges and causality
also pass, with maximum capture-pose discrepancy 1.78e−15. Six independent
synthetic verification contracts are a separate scope. The raw-data audit
confirms the failed opportunity gate and the full dense-window explanation.

The figures distinguish four accepted geometric queries from zero eligible
preparation blends. The sparse depth-point picture uses only observations
delivered by 100.5 s; its latest capture is 100.450 s. Sparse plotted gaps do
not imply the full mapper lacked support. An earlier figure-layout stage is
retained; final stage02 changes only axis/label clarity, not science or data.

The process returns zero. Scientific completion is distinct from the failed
action-utility gate. A separate posthoc decomposition is registered before its
row reduction; it explains the zero result without changing thresholds or
turning it into an acceptance. A metadata hash helper initially used an API
unavailable in the Python environment; its failure and streamed-SHA correction
are preserved without changing the audit.

Independent review notes a general zero-horizon quaternion-normalization edge
in exact current/future pose classification. The explicit current guards have
zero error on all audited anchors here. This dataset result is not a general
proof of that classifier; the experimental code is not promoted for deployment.

All raw depths, masks, calibration, capture transforms/times and delivery times
are retained locally: `audit/stage01/depth_batches.npz`, 205,318,137 bytes,
SHA256 `7bd1f19f6c25723ff93c777f55fd45ff4a63f1c58443a697aa5e7719efb2c4b2`.
The 46,433,954-byte `rows.json` preserves every audited query, rejection,
native/map preview and shadow command; SHA256
`b5d160d5a67ffd4ace3d5199c0dbb221a964372463a6de0bed9b6b93dce8fadc`.

## Preservation

The [compact manifest](evidence/manifest.json) binds 49 files / 5,309,997 bytes
plus itself. Its [source/metadata archive](evidence/source-and-diagnostics.tar.gz)
has 158 verified members. Complete rows, raw camera and signal arrays, scene
assets and videos remain in the identified local raw archive rather than Git.

[Combined I-83/I-84 raw archive](../../../../results/terrain_recovery_preview_20261128/publication/evidence_archive_stage01/i83_i84_evidence.tar.gz):
383,343,894 bytes, SHA256
`481f4d49a4ca54a005a97250a78d40dc1e8327106f971b2e9f9a1a5463bd3753`.
Every project member is stream-hashed and size-checked: 662 project files plus
four exact metadata members, 666 regular members total. It restores 314
registered original-path prerequisites, includes all raw depth and full query
rows, and inventories 19 cache/curation omissions. See the
[verification receipt](evidence/publication/evidence_archive_stage01/verification.json),
[member inventory](evidence/publication/evidence_archive_stage01/inventory.json)
and [restoration instructions](evidence/publication/evidence_archive_stage01/REPRODUCE.md).

[Small presentation ZIP](../../../../results/recovery_assessment_20261128/publication/visual_bundle_stage01/I83-I84-recovery-and-terrain.zip)
contains the recovery video, both PDFs, plots, audits and context revision 19;
its fourteen members pass CRC/SHA verification. Restore raw evidence into an
isolated checkout with the recorded environment; it is not a full simulator
installation or the whole repository.

## Reproduce and decision

Restore exact sources and registered assets at their original paths, then use
a fresh output directory with the recorded Python/MuJoCo environment:

```bash
.venv/bin/python results/terrain_recovery_preview_20261128/runtime/shadow_audit.py \
  --registration results/terrain_recovery_preview_20261128/protocol/frozen_stage01/registration.json \
  --output results/terrain_recovery_preview_20261128/audit/new_stage --run-audit
```

This captures ideal virtual depth from saved states and computes shadow
preparation; it does not launch Gazebo or actuate a controller.

Keep the original LQR/momentum-observer architecture and I-82's leading 0.60
capacity variant. Preserve I-83's mixed sustained-recovery timing. Do not run
a live comparison of an event policy that applies unchanged commands across
the registered activation window. A map-informed **loaded-motion reference**,
such as bounded lean/rate or approach-braking preparation, is a different
proposed experiment. It is not implemented or established by this audit.

Local working evidence: `results/terrain_recovery_preview_20261128/`.
The [iteration log](../../../../ITERATION_LOG.md) distinguishes proposals,
implementation, audits, physical trials and acceptance. Earlier I-44/I-45
geometry/contact-forecast outcomes remain retained.
