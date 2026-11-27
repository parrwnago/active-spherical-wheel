# I-69 Gazebo contact-export boundary review

**Read-only source audit; no new physics or runtime changes.** No demonstrated point/normal/force pairing error explains I68’s admitted zero-pressure shell forces. The transport path does contain a conditional collision-order sign hazard that needs original ordering metadata to establish relevance. It cannot by itself turn nonzero normal projection into zero.

## What is paired and refreshed

The exact-tag [gz-physics7.5.0 adapter](https://github.com/gazebosim/gz-physics/blob/966563e1a403e90bf9a9cbf642dd0f58809f4a36/dartsim/src/SimulationFeatures.cc) copies one DART contact’s IDs, point, force, normal and depth together. It either emits that complete item or omits it; it does not join independently indexed normal and force arrays. [The API](https://github.com/gazebosim/gz-physics/blob/966563e1a403e90bf9a9cbf642dd0f58809f4a36/include/gz/physics/GetContacts.hh) specifies world-frame force on the original first body.

`PhysicsPrivate::UpdateCollisions` retains paired contact/extra-data pointers while their owning container remains alive. Each outgoing message is rebuilt. In nominal DART conversion, extra data is always supplied, so the hypothetical sparse-metadata index issue is not demonstrated. Bridge normal and force use the same message index; this preserves their pairing even in that hypothetical case.

The contact comparator ignores force, but [Component::SetData](https://github.com/gazebosim/gz-sim/blob/4684b525e951bfbf12349e119a5ec90a99ae1a04/include/gz/sim/components/Component.hh) always assigns the new data. It only suppresses the change flag. The bridge reads Data every PostUpdate, so this is not a mechanism that freezes the force. Physics updates contacts in Update, before bridge PostUpdate. Blocking Python steps and read-copy queries add no intervening physics.

## Conditional sign hazard, with a narrow implication

The [gz-sim8.10.0 contact assembly](https://github.com/gazebosim/gz-sim/blob/4684b525e951bfbf12349e119a5ec90a99ae1a04/src/systems/physics/Physics.cc) puts the same contact under both collision entities. On the second entity’s message it relabels collision1, yet still places the original first-body force into body1 without a sign check. A consumer can therefore receive the wrong sign if its entity was original collision2. The bridge consumes each wheel entity’s body1 message.

Original collision order was not captured in I68. This is a source-level conditional hazard, not an observed cause. Sign changes preserve `abs(F dot normal)`, so they cannot alone explain a tangent force whose reported normal projection is zero.

The inspected local `Physics::Update` and `UpdateCollisions` functions exactly match that upstream tag. Their source was not modified by this audit. The normal and force APIs’ installed headers also match the exact downloaded tag bytes. Point messages are duplicated for the two entities, but the bridge visits wheel messages only; ground copies are not simply counted twice. Own-own wheel pairs are skipped.

## What remains unresolved

At I68’s35.205s event, the positive-depth shell-B records retain their full world forces; only their pressure weight is zero. A dominant-positive-pressure label cannot identify all reported force-bearing bodies. The existing momentum agreement supports retaining full reported forces in the comparison, not declaring every solver force physically correct.

This source review does not locate a DART constraint/LCP cause, prove installed ABI compatibility, or justify friction without compression. Needed missing metadata are original pair IDs, contact/constraint identity, and solved impulse/bounds. `source_manifest.json` binds exact-tag URLs/commits, downloaded licensed sources, installed headers and binaries. `report.json` gives the individual findings and limitations. No engine rebuild or new drive run occurred.
