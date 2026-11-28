# I-83 — earlier return, with mixed timing for sustained recovery

Status: completed existing-record analysis and independent verification.
No new physics or controller execution.

## Result

The 0.60 m/s arm returns to acceptable path motion much earlier initially,
but that first return lasts only 1.705 s. A later interruption prevents it
from satisfying the registered two-second dwell then. Its first two-second
path-motion interval starts 0.720 s later than the 0.30 arm's, but it remains
within those bands through the recording endpoint. The 0.30 arm leaves again.
Thus the higher limit improves several recovery measures without improving
every definition of recovery time. I-82's positive whole-route comparison
remains valid; this new question adds detail.

| Path-motion measure | 0.30 m/s | 0.60 m/s |
| --- | ---: | ---: |
| First acceptable state, elapsed from anchor | 9.460 s | 3.380 s |
| First entry that lasts at least 1 s | 16.455 s | 3.380 s |
| First entry that lasts at least 2 s, primary | 16.455 s | 17.175 s |
| Primary entry / confirmation on video clock | 116.955 / 118.955 s | 117.675 / 119.675 s |
| Qualifying later exits after primary confirmation | 2 | 0 |
| Within path-motion bands continuously through 133 s | No | Yes |
| Full motion recovery including rotation | Not reached | Not reached |

Both arms are already outside the bands at 100.5 s: the disturbance onset
is left-censored. These are elapsed returns from the **common 100.5 s anchor**,
not identified physical disturbance-to-recovery latencies. A two-second entry
requires the following two seconds to confirm; confirmation is not the entry.

For 0.60, the first path-motion-good interval is 103.880–105.585 s.
At 105.590–106.000 s, velocity error exceeds its band while path distance
and both attitude conditions remain acceptable. A later geometric exit starts
at 108.045 s. No causal claim that these are all one original disturbance is
made. From 117.675 s onward, all path-motion conditions stay acceptable.
The 0.30 arm's first later qualifying exit starts at 126.555 s and it ends
outside the path-motion bands.

Geometry alone yields two-second entry at 114.925 s for 0.30 versus 103.880 s
for 0.60. Timed-motion two-second entry is 116.955 versus 117.615 s. Both
strict full-motion criteria have no acceptable state after the anchor because
of residual rotation, and are explicitly right-censored. Brief path crossing,
near-goal arrival and complete stationary settling are different outcomes.

Over 100.5–133 s, integrated excess path distance beyond 20 mm is
0.906236 versus 0.190451 m·s. In the fixed 100.5–104 s focus window, peak
intended-section departure is 316.714 versus 37.526 mm. These help describe
severity alongside time, with no changed I-82 gate or automatic adoption.

## Evidence

- [Recovery-annotated video, 100.5–133 s at 1×](../../../../results/recovery_assessment_20261128/publication/video_stage01/recovery_100p5_133_B030_B060.mp4)
- [Two-page recovery PDF](evidence/evaluation/completed_stage01/recovery_assessment.pdf)
- [Full follow-through plot](evidence/evaluation/completed_stage01/followthrough_recovery.png)
- [100.5–104 s focus plot](evidence/evaluation/completed_stage01/focus_recovery.png)
- [All five recovery ledgers and command statistics](evidence/evaluation/completed_stage01/report.json)
- [Independent audit](evidence/verification/completed_stage01/report.json)
- [Original comparison movie](../../../../results/current_belt_capacity_20261128/publication/video_stage01/eight_m100_B030_B060.mp4)
- [Shareable recovery/terrain ZIP, including both PDFs and context revision 19](../../../../results/recovery_assessment_20261128/publication/visual_bundle_stage01/I83-I84-recovery-and-terrain.zip)

## Definitions and interpretation

The user prioritizes returning to controlled motion after terrain disturbance,
while allowing temporary path departure. This assessment reuses both completed
[I-82 recordings](../current-belt-capacity/README.md), with no new simulation or
controller change. Their previous outcomes are already known. New descriptive
definitions are fixed before these new reductions and do not replace I-82's
original acceptance screen.

The common anchor is 100.5 s, the start of the earlier fine-contact episode.
It is not identified physical disturbance onset. Departure search ends at
104 s; both recordings are followed through 133 s. An intended last-quarter
path projection prevents another branch of the figure-eight from counting as
a successful return. Timed-reference error remains separately reported.

Path-motion recovery requires distance within 20 mm, XY velocity within
20 mm/s of the nominal velocity at projected path progress, own-target up error
within 2° and absolute tilt within a common 15° guard. These engineering bands
must persist for two seconds; one-second dwell is a declared sensitivity.
Rotation below 0.05 rad/s is a separate stricter full-motion condition.

Already outside at the anchor means left-censored onset. No qualifying
departure differs from zero-latency recovery. Insufficient dwell at the end
means right censoring. Later exits, remaining good fraction and a sustained
good tail through the endpoint are separate results; later terrain changes can
create new disturbances. A brief crossing of the path does not establish
controlled recovery.

Local working evidence: `results/recovery_assessment_20261128/`.
The [iteration log](../../../../ITERATION_LOG.md) retains source registration,
setup errors, results and the resulting decision.

The frozen protocol SHA256 is
`85f16fd35cbadd3ead3741b9d76f539df26cc7b58f8b3e882ff9de12a094e25c`:
9 sources and 17 inputs, with all original I-82 93 source, 59 input and 456
closed-output bindings rechecked. Fifteen final metric contracts include the
earlier 13; fourteen independent contracts form a separate scope. Preserve
draft clock-count and overstrict quaternion-angle fixture failures, and the
pre-reduction half-open focus-command-window correction. They caused no new
physics or post-outcome retuning of these criteria.

The independent audit reconstructs 60 signal arrays and all ten criterion
arrays from the raw states, sample by sample over 26,601 rows per arm. It
matches 1,317 numerical recovery-ledger leaves plus all nested schemas,
classifications, null values, censor reasons, exit counts and tail states.
This overlaps primary verification; these counts are not added into a
repository test-suite total.

The new video appends recovery indicators to the original I-82 movie without
changing its scene or frames. Decisions use the full 200 Hz state record;
display sampling is 10 fps. First entry, one-second sensitivity and the primary
two-second entry/confirmation are separately labeled, with full later-exit
ribbons and residual-spin status. All 326 encoded frames decode. File size
2,112,610 bytes; SHA256
`63d7d39019fdd012a25c6efa44611bc99ee32700b42489665a2d8a67c5ca6b4b`.
Three pure presentation contracts are a separate scope; no physical replay is
used to create the annotations.

## Reproduction and decision

The [compact manifest](evidence/manifest.json) binds 51 files / 2,303,242 bytes
plus itself, including a [92-member source/metadata archive](evidence/source-and-diagnostics.tar.gz).
Large signals and videos remain local. The combined
[I-83/I-84 raw evidence archive](../../../../results/terrain_recovery_preview_20261128/publication/evidence_archive_stage01/i83_i84_evidence.tar.gz)
preserves all new raw records, exact prerequisites at original paths and media;
the [I-84 preservation section](../terrain-recovery-preview/README.md#preservation)
gives its stream-verification receipt. The small shareable ZIP is 4,590,393 bytes,
14 CRC/SHA-verified members, SHA256
`a6e534b57e55e6392706727bab870a6ca9c91f4abe17f0fd58b123c9da4d109a`.

Restore the exact sources from the compact source archive to their original
paths and use the original raw arrays in the identified
[I-82 raw bundle](../current-belt-capacity/README.md). Use a fresh output directory:

```bash
.venv/bin/python results/recovery_assessment_20261128/evaluation/evaluate.py \
  --definitions results/recovery_assessment_20261128/protocol/definitions.json \
  --seal results/current_belt_capacity_20261128/campaign/outputs_sealed.json \
  --runs results/current_belt_capacity_20261128/campaign/runs \
  --output results/recovery_assessment_20261128/evaluation/new_assessment
```

Keep the original controller and the leading 0.60 capacity variant for further
comparison without erasing the mixed dwell result. The next question is whether
an observed-terrain preview provides concrete command opportunities. The
[separate I-84 audit](../terrain-recovery-preview/README.md) precedes any live
terrain-informed trial. Recovery evaluation should report speed, durability,
severity and residual rotation together rather than selecting a favorable dwell.
