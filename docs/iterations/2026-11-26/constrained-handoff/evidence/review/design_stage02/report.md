# I-56 independent source review, design stage 02

The registered current source is consistent with its stated local differential hierarchy. No runtime or protocol edits were made. This review was registered and the exact inputs copied before reading campaign outcomes. Four focused synthetic algebra probes passed; no physics ran. These are not a repository test-suite count and do not add to previous checks.

## What the hierarchy actually protects

Stage one uses only `G[3:5]`, the fork/body-frame transverse angular-velocity differential response. It minimizes disagreement with the amplitude-clipped native total input under the final amplitude and actual-previous-input slew box. Stage two locks that attained stage-one response. It does not lock actual platform lean, full angular response, world yaw, contact force feasibility, balance stability, trajectory tracking, or the native response when hard slew makes that response unreachable. The source documentation and frozen protocol make these limits explicit.

The algebra probe with independent yaw authority changed yaw while preserving both body-transverse rows, confirming this narrow interpretation. A second probe produced a large unavoidable first-stage discrepancy under slew while the stage-two discrepancy remained zero. Thus a small secondary residual cannot establish that the native balance action was delivered.

## Input continuity and numerical feasibility

The allocator is stateless and centers every slew box on the supplied actual previous applied input. The adapter passes `plant.last_input`, and the runner checks that its mirror equals the physical last input before every call. There is no first-call exemption. A nonzero-previous-input synthetic probe verified that the first box uses that input, rather than zero or a saved optimizer history.

The SVD row-space equality compresses the two-row primary map to its numerical rank. The stage-one point remains an explicitly feasible stage-two seed. Normal solver output is checked for finiteness, bounds, compressed equality, and the original full primary map after clipping. Failure triggers enumerated five-input bound-face QP recovery. A residual violation after recovery restores the stage-one point. This preserves the numerical hierarchy; it does not establish closed-loop stability. Extreme rank truncation and the local approximate model remain physical limitations.

## Information boundary and protocol

The adapter reconstructs contact records from the already-updated observer wrench and current kinematics, and evaluates a private model. The runner fingerprints the native information, observer state/history, references and mirror state around the adapter. Actual previous inputs and simulated body/joint states are allowed. No camera, terrain lookup, truth-contact query or extra observer update appears in the candidate boundary. The mismatch-dependent tangential confidence is an estimated material-velocity consistency proxy, not measured Coulomb friction or ground-truth slip.

The main/ablation settings differ only in `yaw_weight` and `momentum_weight`, both zero for the ablation. Fixed baseline, starting conditions, original raw targets, source hashes, 44-second completion and all acceptance gates were recorded before outcomes. The conditional rule stops the broader campaign when the full lead candidate fails; the ablation is not an authorized substitute selected after outcomes.

The 20 ms G model horizon and 5 ms hard slew interval have different roles. The soft held-contact sensitivity is not an absolute contact forecast; `v_current + G(u-u_native_clipped)` is a correction proxy. A small proxy yaw or momentum does not predict small next-step motion. Soft contact is not constrained unilateral/friction feasible. Final input-rate bounds do not bound actual body or joint acceleration. Ordinary final-request saturation becomes tautologically zero because the returned final request is already bounded; native saturation and active final bounds must be reported separately.

## Review corrections and provenance erratum

An initial message incorrectly claimed that the current no-support yaw cost lacked its guard. Direct reinspection of the registered `6d9ee4e...` live source and identical archived bytes confirms `auxiliary_enabled = total > 1e-9` gates yaw, while zero unloaded weights gate momentum. The claimed mismatch is withdrawn; no code or protocol edit is required. A no-support synthetic probe confirmed exact equality with explicit yaw/momentum-disabled settings. Native/continuity terms and hard boxes still apply, so no-support behavior is not a bitwise bypass of the allocator.

The earlier stage-01 historical script and stdout are preserved exactly, and its archived-source replay matches those outputs. However, its historical imported library bytes were not hashed at import time; the later recorded `05808089...` source was captured while concurrent edits were possible. Exact historical module identity is therefore retrospectively inferred, not certified. The replay is SHA-bound; the original imported source is not. Stage-01 replay and historical checks are the same 90 allocator cases plus 35 QP comparisons, not additive counts. Its early review notes about absent no-support/post-clipping guards do not apply to the registered current source; direct inspection also finds those guards in the later 0580 archive. Current stage-02 input identity is bound before outcome access.

A launcher setup error (`python` absent) occurred before checks ran; rerunning the unchanged script with `.venv/bin/python` passed. No physics or runtime changes resulted.
