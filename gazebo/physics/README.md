# Exact-state physics system

`Physics.cc`, `Physics.hh`, `EntityFeatureMap.hh` and `CanonicalLinkModelTracker.hh` are
gz-sim 8.10.0's `src/systems/physics` (Apache License 2.0, Open Robotics), with one change
to `Physics.cc`, marked `ASW exact state`. `DartsimRetrieveWorld.hh` is gz-physics 7.5.0's
`dartsim/src/World.hh` (Apache-2.0), the interface its DART plugin uses to hand out the DART world.

Stock gz-sim copies a link's world pose and velocity into the entity-component manager only
when the physics engine reports that the pose moved by more than 1e-6 (1 µm, or 1e-6 in a
quaternion component). Its change checks also use a 1e-6 tolerance. A balancing wheel near
rest moves less than that in one 1 ms step, so its reported base velocity stays frozen at an
old value for many steps. Observed: a constant vz = -9.81e-3 m/s (exactly -g dt) while the
wheel stood still. Feedback control needs the current state.

The modified system writes every link's pose and velocities on every step and compares
values exactly.

It also switches off DART's automatic deactivation. DART 6.19 freezes a skeleton that has
moved slower than 1 cm/s and 0.05 rad/s for 0.5 s: gravity, contacts and integration are
skipped until it is disturbed. The wheel standing still under feedback met that test and
froze, with every contact reporting zero force. DART, contact handling, joint handling and every other behaviour are
upstream code. Worlds load it with
`<plugin filename="AswExactPhysics" name="gz::sim::systems::Physics"/>`.
