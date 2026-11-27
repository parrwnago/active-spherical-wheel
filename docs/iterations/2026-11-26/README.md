# Research presentation notes — 2026-11-26

- **I-58 balance priority before optional smoothing completed:**
  [record](balance-priority/README.md). Seven full 44 s runs; exact control
  reproduction. P restores nominal local balance response everywhere and
  improves tracking RMS at all three starts. The -1 rad lead passes all gates
  (RMS 51.08→33.43 mm, final 42.65→6.29), but +.30 fails endpoint/turn checks
  and +.90 fails yaw/speed/turn checks. All actual settling tests fail. Keep P
  experimental and native LQR selected. Goal-hold audit shows +.30 nearly
  stalled 52.6 mm away and +.90 still moving outward at44; inspect native
  goal-hold/reference dynamics next. No post-outcome tuning or default change.
- **I-57 loaded-contact response validation completed:**
  [record](loaded-contact-validation/README.md). All 30 fresh-prefix Gazebo
  probes and exact controls complete. Dominant response errors are ~1–2%; five
  weak late cross-coupling scores fail, 165 remain unresolved. At the late state,
  the optional slew bound forces +0.898 N m shaft torque versus the native
  -1.5 request, with a physically confirmed balance-response cost. Separate
  exploratory analysis finds current+Gdelta omits native future motion: held
  20 ms yaw proxy -0.006 versus measured +0.762 rad/s. Whole-trace one-step
  affine predictions improve average error but retain large tails. Keep native
  LQR; next isolate balance priority over optional smoothing. No default change,
  new controller, contact-transfer or multicontact validation is claimed.
- **I-56 bounded handoff candidate:** [record](constrained-handoff/README.md).
  Three fresh 44 s Gazebo comparisons use native B, a bounds/continuity ablation
  and a full yaw/momentum-aware total-command allocator. Existing LQR, observer,
  references, gain bank and defaults remain unchanged. Native B reproduces all
  56 original precision fields exactly. The full candidate regresses peak
  tracking 116.26→136.16 mm, final error 42.65→76.72 mm and absolute tilt
  13.72→18.96°. It fails the registered screen, so the conditional broader
  campaign stops and no candidate is adopted. The failed preparation-cost
  fixture, correction and exact source/protocol history are retained.
- **I-55 matched-prefix causal diagnostics:** [record](causal-replay/README.md).
  Four short 32.5 s Gazebo runs. Unchanged batch and 1 ms-read versions match
  all 56 original precision fields at 6,501 samples exactly. Removing only
  unloaded-shell A torque reduces the first 5 ms yaw-rate increase 55.2%,
  with an opposing shell/platform momentum effect; later motion rebounds.
  Holding attitude reduces yaw but worsens fixed untreated-target error
  9.848→15.932°, despite only 0.495° error to the easier held target.
  No intervention is a controller fix. Registered 25 ms yaw budget passes;
  55 ms fails at 11.8% residual. No source-supported contact-publication lag
  explains that residual. Raw 1 ms event traces and all adverse results
  are retained; controller defaults remain unchanged.
- **I-54 acceleration and platform-spin investigation:**
  [record](acceleration-spin-cause/README.md). R already spins before the
  32.250 s handoff, peaks at 6.635 rad/s, and makes 2.748 heading turns through
  44 s. Near-vertical shell axes and opposing shell/platform momentum changes
  support a reaction-wheel contribution; native rate feedback later reverses
  torque. The entering relative attitude error includes a target swing:
  desired up moves 15.621° while actual absolute tilt decreases over
  31.400–31.745 s. Separate heading control is off, but angular-rate LQR
  feedback remains. Exact support-share/gain algebra, prior guard history,
  model-derived momentum and causal limitations are retained. No new physics
  campaign or controller/default change.
- **I-53 teammate publication and acceleration audit:**
  [record](teammate-handoff/README.md), [start-here guide](../../TEAMMATE_HANDOFF.md).
  The user's playback 10 s cue is recorded 32 s. R reaches 1.025 m/s after a
  support/gain switch requests −7.719 N m shell torque, clipped at −0.600;
  reference slew constraints do not bound actual acceleration. N is calmer in this
  window but still transient. Keep the overall improvement qualified.
  Current source/tests/history, selected media/data and frozen native-run
  restoration recipe were published on main in commit `37f3cdc`; defaults unchanged.
- **I-52 scheduling × joint-reference configuration study:**
  [record](contact-scheduling-joint-tracking/README.md). Twelve complete 44 s
  cells across phases −1/+0.3/+0.9. Ground-normal phase lowers RMS at all
  starts, but +0.3 peak/lean and turns regress. Fixed reference slew limiting
  worsens −1 RMS 51.08→76.79 mm and lean 7.40→13.20°. The combination at +0.9
  finishes 9.53 mm from goal and passes original settling/route gates, but four
  turn overshoots worsen. No candidate passes the full screen across starts;
  defaults remain. One unscorable native crash plus its unchanged serial retry
  are retained separately. Historical correction: the recovered old MuJoCo
  normal-phase sweep improved 3.72% with zero falls, below its 5% margin;
  previous generic regression wording was too broad.
- **I-51 reference/handoff isolation completed:** [record](allocation-handoff-isolation/README.md).
  Five fresh 44 s cases complete. Baseline uses plain LS, with original XY and
  body targets; the old 191 mm drift was not intentional feasible-reference
  relaxation. Reference slew limiting improves RMS 50.49→44.21 mm and final
  190.19→32.69 mm, but turn/settling gates fail. Belt handoff slew improves RMS
  52.52→42.06 mm yet worsens peak relative lean 6.05→11.68°. A separate phase
  diagnostic matches the original first 38 s exactly, then changes only
  normal-relative scheduling: final 190.19→53.99 mm, late RMS 84.65→56.71 mm,
  whole moving RMS essentially unchanged. This implicates scheduling in that
  episode; no time-gated/default policy is adopted. All tradeoffs retained.
- **I-50 saved-log handoff/deviation audit:** [record](belt-handoff-investigation/README.md).
  Video18s maps to recorded32s. Baseline31.650s shell re-entry has almost
  parallel drive directions, infeasible +489/−1761rad/s references and clipped
  joint-speed LQR torque demand. Raised32.170s is a separate discontinuity:
  prepared belt−.012→loadedLQR−.303m/s in5ms, then support chatter. Existing
  kinematic preview prepares incoming no-slip speed but guarantees neither
  contact onset nor continuous loaded-command takeover. Late191mm departure
  starts under correct support, accurate normal, nonsingular allocation and
  unsaturated commands; its full physical cause remains unresolved. No new
  physics/default changes. Slow saved-state replay and exact evidence retained.
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
