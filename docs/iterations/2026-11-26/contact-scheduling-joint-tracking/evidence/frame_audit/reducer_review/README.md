# I52 independent source and numerical review

Source prereview cleared the frozen scientific reducer and the later resolver-only copy before the first outcome reduction. No numerical-review script imported the reducer/helpers or ran a controller, model or simulation.

All12 resolved cases completed44 s. The original m100_N process139 attempt remains a separate preserved technical failure; the single registered serial retry resolves that cell.

| Case | Moving RMS(mm) | Peak(mm) | Final(mm) | Peak relative lean(deg) | Settled |
|---|---:|---:|---:|---:|---|
|m100_B|51.080|116.256|42.648|7.399|False|
|m100_R|76.789|306.794|43.269|13.196|False|
|m100_N|39.195|87.890|12.686|5.074|True|
|m100_NR|49.688|148.608|31.785|12.517|False|
|p030_B|56.963|154.280|20.821|9.484|False|
|p030_R|48.415|143.301|32.001|9.355|False|
|p030_N|53.936|192.143|16.575|10.706|False|
|p030_NR|53.466|179.981|17.963|10.218|True|
|p090_B|50.486|190.901|190.191|8.709|False|
|p090_R|44.213|97.186|32.687|6.346|False|
|p090_N|43.284|93.652|36.311|7.757|False|
|p090_NR|44.972|99.390|9.527|6.195|True|

The independent checks reproduce all12 headline sets, all72 fixed-turn overshoots/reversal lags and coverage flags, both native/used joint-error targets, held support denominators, factorial differences/interactions, original route gates and the complete registered screens. These are1057 numerical assertions plus45 screen assertions, not extra repository tests or independent experiments.

The results depend on configuration. Normal-phase lookup lowers moving RMS at all three starts, but other metrics regress. The reference guard worsens the−1 rad start substantially. At+0.9 rad the combined candidate finishes9.527 mm from the goal and settles, yet its turn regressions fail the broader screen. No policy passes the all-three-phase screen.

A lower error to a guarded target alone is insufficient evidence of better tracking. At+0.3 rad, R has actual-minus-native joint-error RMS157.697 rad/s versus actual-minus-used2.918 rad/s; both measurements remain in the report. These are current state-reference mismatches, not measured motor servo responses.

The original signed-lag guard is retained. A positive signed-lag change does not necessarily worsen absolute timing error; it can move an early reversal toward zero. Spatial overshoot regressions independently remain, so no criterion was adjusted after seeing outcomes.

[Authoritative review JSON](final_report.json) points to the exact source/retry-resolver snapshots and the two sealed numerical stages. The canonical source, report and old artifacts were not modified.
