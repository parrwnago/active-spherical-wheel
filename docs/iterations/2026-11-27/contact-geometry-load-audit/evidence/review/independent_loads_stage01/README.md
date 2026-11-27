Independent I-66 numerical review; no model fitting or new physics.

`verify_loads.py` uses hinge axes, hinge anchors and body ancestry to calculate
sampled-point, net-wrench and normal-share proxy loads. It imports neither the
canonical audit nor its scoring module. Frozen I-65 covariance scores are
recomputed by eigen decomposition. The source receipt precedes this arithmetic.
All route masks, compatibility decisions, cohort counts and counterfactual
decisions match exactly. Maximum load difference is 4.44e-16 Nm; maximum score
difference is 1.82e-12. Per-case and pooled vector statistics also agree. The
compact report contains checks and mask/decision hashes, not full array dumps.

`verify_floor.py` is a separately registered, append-only explanation after the
main result. It confirms that a joint outside a family's ancestry receives
zero predicted C/T/W external load, leaving that raw observer component as a
structural residual floor. It verifies the root's floor statistics exactly.
All 300 useful/harmful opportunities have one positive sampled family, and the
sampled-point and share-proxy predictions agree there. This does not imply one
physical contact, constant routing during the preceding 5 ms, or true force
allocation. Neither calculation determines the physical cause of remaining
load disagreement or authorizes a live change.

The main report is `output_stage01/report.json`; the post-result algebra report
is `floor_output_stage01/report.json`. The original sources, registration
receipts and execution logs are retained separately. These are independent
arithmetic checks, not additional simulator trials or full repository tests.
