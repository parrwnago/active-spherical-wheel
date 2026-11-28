# I-83: sustained recovery assessment

New descriptive recovery assessment of existing I-82 recordings; old outcomes already known. Definitions fixed before these new reductions, not blind preregistration and not a change to I-82 acceptance gates.

All elapsed times use the common 100.5 s clock anchor, not a measured disturbance onset.

| Criterion | Arm | Classification | First entry (s) | 2s entry / confirmation (s) | Later qualifying exits | Sustained through end |
| --- | --- | --- | ---: | --- | ---: | --- |
| geometric | B030 | left_censored_onset | 105.465 | 114.925 / 116.925 | 2 | False |
| geometric | B060 | left_censored_onset | 103.880 | 103.880 / 105.880 | 1 | True |
| path_motion | B030 | left_censored_onset | 109.960 | 116.955 / 118.955 | 2 | False |
| path_motion | B060 | left_censored_onset | 103.880 | 117.675 / 119.675 | 0 | True |
| path_full | B030 | left_censored_onset | — | — / — | 0 | False |
| path_full | B060 | left_censored_onset | — | — / — | 0 | False |
| timed_motion | B030 | left_censored_onset | 115.435 | 116.955 / 118.955 | 2 | False |
| timed_motion | B060 | left_censored_onset | 104.550 | 117.615 / 119.615 | 0 | True |
| timed_full | B030 | left_censored_onset | — | — / — | 0 | False |
| timed_full | B060 | left_censored_onset | — | — / — | 0 | False |

A first threshold crossing is not sustained recovery. Right-censored and no-departure cases retain null recovery times.
Intended-section distance, spatial nominal velocity and raw-reference attitude are separate from clocked tracking and world-upright tilt.
Spin is a separate stricter recovery condition. Later exits can be new terrain disturbances.
The 1s sensitivity, clipping at own/original bounds and complete recovery records are in report.json.
