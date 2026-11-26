# I-53 — acceleration-burst audit and teammate publication

2026-11-26. **Saved-data investigation and portable research handoff completed;
publication requested on the current main branch. Controller defaults unchanged.**

The user asks to push the repository with enough material for a teammate to
understand and continue the work. They also question whether I-52 is a meaningful
improvement and identify violent motion near 10 s of its −1.00-rad R chapter.

## Assessment

The general tracking changes are modest and configuration-dependent. The
+0.90 NR endpoint/settling improvement is useful, but it does not establish
robust whole-route performance. R is a substantial regression at −1.00, not
an acceptable default. N is calmer in the observed window and lowers whole-
route RMS at the three starts, while transients and turn regressions remain.
No candidate passes the complete I-52 screen.

Original playback 10.000 s maps exactly to recorded 32.000 s. Fractional
times use the continuous 4× clock mapping between original 200 ms pose frames.
The trajectory is still moving until recorded 41 s, so this is not final
settling despite its apparent proximity to the goal in the image.

Fixed recorded 28–36 s window:

| Arm | Peak planar speed | 50 ms acceleration RMS | Peak relative lean | Timed position RMS |
| --- | ---: | ---: | ---: | ---: |
| B | 0.472 m/s | 2.596 m/s² | 7.399° | 60.553 mm |
| R | 1.025 m/s | 2.225 m/s² | 13.196° | 153.475 mm |
| N | 0.263 m/s | 1.653 m/s² | 5.074° | 43.852 mm |
| NR | 0.660 m/s | 2.803 m/s² | 12.517° | 75.020 mm |

Even acceleration RMS can hide a concentrated burst. R reaches its speed
peak at 32.305 s and whole-route timed-error peak 306.794 mm at 33.235 s.
Its reference slew still obeys 100 rad/s², while measured actual joint
finite-difference acceleration has a component near 1,525 rad/s². These
reference bounds are not limits on actual acceleration or total feedback.

At 31.770 s, estimated shell-B→belt-B transfer produces a 1.699 m/s belt
request, clipped to 0.30 m/s. Later demand reaches 2.694 m/s while applied
speed remains capped. At 32.250 s, both evaluated and inferred support switch
belt B→shell B. The shell-B request changes −0.092→−7.719 N m, clipping at
−0.600 N m. Its exact LQR jump split is predominantly changing K (−7.602 N m)
rather than changing error (−0.011 N m). Lean error was already about 13.17°.
The gain acts on a bad entering state and the command remains saturated
through subsequent holds, preceding the 32.305 s speed peak.

At that frozen state, current rolling geometry is moderately conditioned,
preparation delta is zero, and substituting guarded references actually
softens demand by about +0.098 N m. Both native/guarded alternatives would
still clip. This identifies a proximate support-scheduled command discontinuity,
not a proof that the instantaneous reference clamp uniquely causes the burst.
R changed the earlier closed-loop history. Commands at t act on the next
5 ms interval; they cannot cause an increment already ending at t.

The next useful controller target is support/gain handoff with balance and
command feasibility preserved. Another blanket reference smoothing change
is not established as a fix. No new controller or Gazebo campaign is introduced
by this saved-data audit.

## What the teammate receives

- [Current handoff](../../../TEAMMATE_HANDOFF.md): loop, units, equations,
  active/parked modules, limits, code map, reading order and AI assistance.
- The existing detailed [study PDF](../presentation-preparation/control-and-observer-study-guide.pdf).
- Full current source, experimental modules, scripts and tests, plus the
  retained iteration ledger and compact evidence for successful/rejected work.
- [Selected media](../../../handoff/media/README.md), including the I-52
  overview and the slow four-way replay of this burst.
- [Exact-value event subset](../../../handoff/data/README.md): selected
  precision fields over 28–36 s, source hashes, no resampling/dtype conversion.
- [Portable workspace recipe](../../../../research/README.md): approximately
  65 MiB of exact prerequisite sources/inputs, with manifest validation,
  non-overwriting restoration and unexecuted fresh-campaign preparation.

The recipe supports fresh native runs. It does not silently claim that old
original-dataset analysis/retry metadata describes fresh data. Complete original
raw studies and large presentation ZIPs remain local; current compact records,
selected data/media and required native-run dependencies are committed. Older
archived `results/` links may still point to those local full datasets.
Unrelated local firmware/ESP32 work is outside this publication scope.

## Verification and preservation

The pre-change capture preserves 168 source/document origins and 21 older
presentation archives. No control/runtime/config files were edited in I-53.
All earlier evidence remains unchanged. Current research changes from earlier
iterations are included in the requested publication; this is not a claim that
they were committed when their original trials ran.

The existing repository suite passes **579 tests with 6 skipped**. The final
new workspace-tool suite passes **21 tests** in a separate invocation; this
includes earlier tool tests, so earlier 17-test counts are not additive.
Original and corrected development stages are retained, including adversarial
copy/symlink/concurrency test failures before the final correction. Restoring
the actual existing workspace is a zero-write operation with hash/mtime/size
preservation across 118 checked originals. An isolated fixture restores 28
files and prepares twelve jobs with zero native executions.

The full-suite run changed one initially clean generated mass-report number
from `0.36` to `0.36000000000000004`; the test output is preserved locally and
the original exact file was restored. A first pytest-log redirection stopped
before collection because the new output directory was absent; it is recorded
separately from the successful test run. No new trajectory outcomes replaced
older results. Secret-pattern scans and final staged scope/size checks accompany
the publication; credentials and local authentication helper files are excluded.

The slow replay passes its media/clock/hash checks. Event plots distinguish
5 ms finite differences, 50 ms velocity differences and actual/requested
references. The original first plot stage and the annotation-spacing revision
are both retained. Stronger causal claims would require separate controlled
interventions; this audit does not attribute the burst solely to estimation
delay, slip, a current Jacobian singularity or the instantaneous guard action.

The [final publication checks](evidence/final_publication_checks.json) verify
86 relative links in the live guides/indexes, 140 exact event-subset arrays,
all 29 frozen payload entries and 90 required live sources. A fresh twelve-case
preparation and the actual Gazebo-environment read-only preflight pass without
launching a trajectory. The publication scan finds no matching secret patterns.
The [staged scope snapshot](evidence/staged_scope_before_record.json) excludes
unrelated firmware, local environments, raw result roots and authentication
helpers. The whole staged whitespace check flags generated/historical evidence
and an existing test's blank final line; these bytes are preserved for their
recorded hashes. Current live handoff pages and new workspace code pass the
separate whitespace check. The snapshot precedes adding its own evidence record.

[Compact event findings](evidence/event_summary.json)
· [Event plot](../../../handoff/media/i53-handoff-detail.pdf)
· [Context plot](../../../handoff/media/i53-context.pdf)
· [Portable media hashes](../../../handoff/media/manifest.json)
· [Evidence inventory](evidence/manifest.json).
