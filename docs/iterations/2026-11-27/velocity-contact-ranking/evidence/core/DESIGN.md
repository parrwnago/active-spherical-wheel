# I-60 fixed contact-candidate core

The new opt-in module is independent of the live controller. It evaluates supplied
measurements and known wheel geometry using private kinematics. No reference,
next command, solver contact, terrain normal or terrain-height input is accepted.
The feature bank is conditioned on an inferred normal, normally obtained from
the momentum observer; its velocity arm is therefore not fully independent of
that observer.

Seventeen normal seeds (centre, five-degree ring, ten-degree ring; eight fixed
azimuths each) are projected onto each of four spherical patch bands and two
equatorial shell rim edges. Candidates outside the ten-degree hint cone, outside
their own band or below one newton compressive force are invalid. Rims retain
their shell's family label. There are 102 slots but only four family labels.
Their radial normal is a restricted local model, not a claim to know true rim
surface normals or a general multi-contact force decomposition.

Measured rigid-point velocity is computed from one centre Jacobian per body,
then omega-cross-offset. The tangent residual adds previous-held ideal belt
surface speed in the current geometry's local y direction. No requested torque
is interpreted as an achieved joint speed. Moment consistency is
`(M - offset cross F)/(R*norm(F))`.

The four scores are geometry alone, geometry plus normalized moment squared,
geometry plus velocity residual squared divided by .02 m/s squared, and all
three terms. Geometry uses angle/ten-degrees squared. Each family takes the
minimum COMPLETE candidate score; components are never minimized separately.
All scores and margins are uncalibrated, not probabilities.

Confidence requires at least two valid families, motion availability at least
.02 m/s, family score gap at least .25, winner residual at most .02 m/s,
winner/runner residual-vector contrast at least .01 m/s and no exact score tie.
Motion availability is norm(base translation) plus R times norm(base angular
rate), plus R times norm(joint rates), plus norm(previous two belt speeds).
It does not prove observability. Raw rankings remain diagnostic; accepted family
is unknown when these gates fail. A singleton is explicitly geometry-forced.

`evaluate` returns rich geometry and measured-motion features. `project_features`
removes geometry, canonicalizes invalid entries to zero and retains NumPy-only
rank inputs, including residual vectors for the contrast gate. `rank_hypotheses`
accepts one frame or batched arrays, without MuJoCo installed.

Synthetic tests cover same body motion with different joint speeds, rotated
belt axes and prior-held speed, no solver/dynamics calls, direct-point Jacobian
equivalence, geometry/band/cone/rim validity, input/model/live-state immutability,
joint candidate-score minima, duplicate-family/singleton handling, ties,
low-motion/sliding/invalid-load abstention, batched parity and NumPy-only import.
The final selected run is 22 new plus six existing tests: 28 passes. Earlier
19/27 runs overlap and must not be added. An explicit counterexample preserves
that a slipping true contact can let an incorrect no-slip hypothesis pass the
heuristic confidence gates. No setup/test failure occurred; later
source stages preserve deliberate coverage/contract additions before data.

`final_source_seal.json` binds the module/protocol before historical features;
`final_source_seal_stage02.json` updates only the expanded test binding, with the
module byte-unchanged and still before historical feature extraction.
The synthetic source snapshots retain chronology. No actual
data outcome or controller improvement is claimed by this core implementation.
