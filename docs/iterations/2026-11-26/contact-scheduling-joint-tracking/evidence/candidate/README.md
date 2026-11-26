# I52 phase-scheduling × joint-reference adapter

The candidate adapter and seven pure composition/frame/source contracts are implemented and frozen. These contracts establish input handling and composition, not robot performance. No controller, observer, MuJoCo model, predictor or physics was executed by this helper owner.

The12 registered conditions are starting carrier phases−1.00,+0.30,+0.90, each with native baseline(B), exact old reference guard(R), estimated-normal lookup phase(N), and both(NR). All use the original44s Mars figure-eight and0.30m/s belt bound. Flags are configured before controller construction; there is no38s gate. Root owns the native campaign/protocol.

The reference guard is imported only after verifying the exact frozen I51 SHA80c01ce649ce507dde6e96950fbfeb8b9b7de269a5d51c8980529d5970d5a2a4. Its20rad/s and100rad/s² settings and history semantics are unchanged. No additional damping, speed cost, weight law, total-belt slew guard or runtime/default edit is introduced.

`prepare(controller,vref,normal,reference_guard=...,normal_phase=...,touching=...)` computes both current carrier phases and optionally calls that old reference guard. It modifies only the three joint reference entries and the old guard’s owned history. The harness calls original `_terrain_feedback` exactly once afterward. The helper itself calls no controller method. Baseline and N-only preparation do not create reference-guard history. `validate_feedback` checks the returned phase/joint references and, when given the controller, that K’s shaft row equals that controller’s own ordinary local row.

For carrier rotationR and unit world verticalZ, native phase is `atan2(−(RᵀZ)x,(RᵀZ)z)`. The normal option substitutes the current observer-derived filtered normaln. Both values, the chosen value, native/used/actual joint rates and guard flags are recorded. A vanishing xz projection is flagged as an undefined phase coordinate rather than claimed as a reliable configuration. When unloaded, original feedback can keep a previously loaded gain even while reporting the current phase; the optional touching field makes that distinction visible.

This is only a lookup-phase approximation using the existing flat-floor gain bank. It does not generate new terrain-oriented A/B models, transform every state/input/error frame, change gravity, or establish stability. XY errors remain heading-relative; quaternion and body-angular velocity conventions remain native. Original feedback restores the ordinary shaft row after support-weighted gain scheduling. Ordinary local K and initialization should match within each four-arm starting-phase group; they are not expected to be identical across the three different starting phases. The global gain bank/configuration stays shared.

Pure checks cover exact old-source binding and tamper rejection; flat-normal identity; independently derived tilted-plane phase; oblique coordinate/order/wrapping and undefined cases; all12 factor compositions on identical structured inputs; returned phase/rate/OWNordinary-row validation; input rejection and reset/clone isolation. Identical-input orthogonality does not imply additive or identical closed-loop outcomes, because each arm’s state/observer history can diverge.

The tests intentionally use structural fake controllers containing arrays and no controller/model methods. Tested source and input hashes were captured before execution. Seven I52 contract groups pass; no failures occurred, and these are not additional counts for the old I51 guard tests or a full repository suite.

[Exact contracts/source snapshots](development/stage01/report.json) · [Adapter](scheduling_reference.py) · [Verifier](verify_scheduling_reference.py).
