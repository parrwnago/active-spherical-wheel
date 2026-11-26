# Research presentation notes — 2026-11-26

- **I-49 belt-capacity sweep completed:** [record](belt-capacity-sweep/README.md).
  Same fixed LQR design, three fresh44s Marsfigure8 runs at.30/.60/.90m/s.
  Highercaps reduce peak191→110mm/final190→49mm/relativelean8.71→6.05°,
  but movingRMS50.49→52.52mm and settling/turn gates still fail. .60/.90body
  motion matches exactly despite45ms of differentappliedbeltA commands and
  differentcalculatedslip. Useful capacity tradeoff; no motor/default selected.
- **I-48 saved-contact clock/radius diagnostic:**
  [record](contact-clock-disentangling/README.md).
  Matching the plugin's radial-R convention removes4 Mars/10flat individual
  admitted-point band violations; prior-pose shift alone does not. Old logs
  support this comparison, not an unrecorded computation-delay estimate.
  Controller and observer remain unchanged; no new simulation.
- **I-47 presentation preparation; saved-data analysis only:**
  [PDF and source/evidence record](presentation-preparation/README.md).
  Explains the full current loop, dynamics and observer math, soft cost versus
  clipping, single-contact directional singularity, lockstep/measurement clocks
  and turn metric distinctions. Includes a five-minute outline and professor
  Q&A. No new simulation or controller change; earlier outcomes remain intact.
- **I-46 completed; filter retained:** [current support-routing audit](support-routing-timing/README.md).
  Selected a gentle Mars corridor before control outcomes; both 16 s Mars/flat
  cases completed with transfers in both directions. Seven common transitions
  give 0 ms median timing improvement; mismatch-equivalent Mars 5→0 ms, flat
  10→10 ms. Command maxima unchanged; registered benefit gates fail. Historical
  90→55 ms disagreement helps classification, but the useful cue follows belt-A
  saturation and leaves the largest jump unchanged. Keep the baseline stack.
  54 selected unique tests pass each environment; failed site criteria, initial
  flat setup and pre-reduction tie bug are retained. No faster routing actuated.
- **I-45 completed; current map-to-contact integration parked:** [configuration isolation](contact-preview-pose-isolation/README.md).
  On 244 matched resolved-contact patch-mean targets, memory RMS improves
  11.492→7.722 mm with hindsight actual configuration, but observer tangent is
  still lower at 7.476 mm. Motion contributes substantially; no material map
  advantage meets the registered rule. Tails improve, median/identity remain
  worse. Dense actual-pose memory has 0/33 complete contact previews. All 714 CV
  rows reproduce; 61 selected tests pass in each environment. No new physics,
  depth capture, controller changes or demonstrated driving gain.
- **I-44 implemented/analyzed, offline:** [four-camera terrain memory](terrain-map-preview/README.md).
  Four depth views and exact simulated state recover terrain at 277/327 uniform
  forecast queries (84.7%); current views alone cover none of those underbody
  locations. Memory passes the registered terrain-utility screen, but early
  bump coverage is only 3/33 and matched future-contact point RMS worsens
  7.90→11.30 mm (271 pairs). Keep LQR/observer/feedforward unchanged. One reused
  Gazebo trace, zero new physics runs; 75 selected unique tests pass in each
  environment. No SLAM/state estimator or tracking-improvement claim.
- The prior [I-43 experiment](../2026-11-25/tilt-reference/README.md) remains
  rejected for adoption: reduced early demand did not pass whole-route lean and
  turning criteria. Terrain sensing is a new hypothesis, not a correction to
  those preserved results.
