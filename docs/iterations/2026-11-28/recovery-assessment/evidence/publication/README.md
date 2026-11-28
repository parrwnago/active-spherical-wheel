# I-83 recovery-annotated recording

`video_stage01/recovery_100p5_133_B030_B060.mp4` uses original I-82 video frames
1005–1330, covering physical time 100.5–133 s at 1× and 10 fps. Its 326 inclusive
frames produce a 32.6 s container duration. The original MP4 is unchanged.
No geometry is rendered and no controller, observer or physics is executed.

A separate appended panel distinguishes instantaneous band membership, first
entry, the one-second sensitivity and the primary two-second interval with its
confirmation time. Full follow-through ribbons and later-exit counts prevent an
early temporary return from being presented as permanent recovery. Strict spin
recovery, left-censored onset and the common anchor are explicit. Decisions use
all 200 Hz analysis states; labels are displayed at 10 fps. The ribbons and
recorded entry times are retrospective annotations, not live control inputs.

Three pure presentation contracts pass in `contracts_stage01/`. Source, original
movie, analysis, independent verification, font and definition hashes are bound
before array/video opening. Every output frame decodes successfully; encoded
116.8 s and 133 s frames were visually inspected for readable labels and the
original two-panel scene. `video_stage01/metadata.json` and its artifact manifest
record exact hashes, bytes and frame clocks.

Reproduce into a new empty directory:

```sh
.venv/bin/python results/recovery_assessment_20261128/publication/annotate_recovery.py \
  --canonical results/recovery_assessment_20261128/evaluation/completed_stage01 \
  --verification results/recovery_assessment_20261128/verification/completed_stage01 \
  --definitions results/recovery_assessment_20261128/protocol/definitions.json \
  --root-registration results/recovery_assessment_20261128/protocol/frozen_stage01/protocol.json \
  --source-video results/current_belt_capacity_20261128/publication/video_stage01/eight_m100_B030_B060.mp4 \
  --source-metadata results/current_belt_capacity_20261128/publication/video_stage01/metadata.json \
  --output results/recovery_assessment_20261128/publication/new_video_stage
```
