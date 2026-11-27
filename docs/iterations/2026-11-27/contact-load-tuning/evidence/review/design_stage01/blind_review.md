# I-62 independent design review before the new threshold sweep

I-60 and I-61 outcomes, including all three episodes, are known. No I-62 sweep,
feasible-count or selected-fold outcome was read for this review. This is a
new, explicitly retrospective tuning study; it must not overwrite the fixed
I-61 veto's five failed gates or describe the old result as subsequently passing.

## Fixed intervention and zero boundary

Keep every I-60 candidate, score, accepted proposal and I-61 force/load feature
unchanged. Only one globally shared pair a,b changes the three-joint rule:
valid AND max(abs(joint residual)) <= a + (b R) ||F||. A proposal is used only
when the frozen I-60 acceptance flag and this rule both pass; otherwise use the
exact native family and point. Same-family accepted point changes remain part
of the policy. No point cap, hysteresis, force fit, reranking or per-joint/
per-phase coefficients enter this experiment.

Construct the 61×61 grid from integer indices times 0.005, a,b in [0,0.30].
The a=0 boundary extends the offline study beyond the frozen I-61 runtime
kernel, which requires a>0. Evaluate the component inequality directly, without
division or modifying that kernel. At zero tolerance only an exactly zero
valid residual passes. Native N, veto-off A and (.03,.10) are distinct explicit
references; zero tolerance is not automatically equivalent to native.
The (.03,.10) policy and primary metrics must reproduce I-61 before tuning.

## Denominators and exact point metrics

Use the original half-open primary window and dominant-support eligibility,
including failures/unknown labels. The fixed point comparison mask uses finite
N, finite A (accepted proposal or native fallback), and finite equivalent truth.
It must not require a raw unaccepted proposal or only retained proposals.
Freeze that mask before evaluating any threshold. All N/A/tuned policies use
the same rows and point target; report coverage explicitly.

For each grid cell retain counts, harmful overrides, useful corrections,
retained/vetoed proposals, point SSE/RMS, exact empirical p95 and exact maximum.
Do not infer tail improvement from RMS or a coarse histogram. Efficient blocks
of threshold comparisons followed by direct native/proposed distance selection
and partition/percentile calculations are sufficient; do not create an
unnecessary dynamics or optimization framework. Invalid load features cannot
become accepted by a large tolerance.

Distances and SSE units and floating reduction conventions must be explicit.
Keep actual SSE values; any numerical tie convention must be registered before
outcomes. A scalar difference in machine roundoff must not silently change a
fold parameter while the report claims exact deterministic selection.

## Training selection and retrospective group assessment

The registered training rule is reasonable: filter by every training gate;
then fewest family errors, smallest point SSE, fewest vetoes, smallest integer
a index and b index. Training uses two complete start episodes. The held-out
episode cannot influence feasibility, objectives or tie breaks. A failed
feasible search remains a failed fit; explicit veto-off A may be evaluated as
diagnostic fallback, but cannot count as a qualified fold.

The training gates require 50% harmful-override removal, 90% useful retention,
pooled errors no worse than A, per-phase error/harm guards, point RMS/p95 no
worse than A and maximum no worse than N, with the declared numeric allowance,
and a nonempty retained proposal set. Oracle compatibility is diagnostic only
in this new study, not a fitted covariance or an added acceptance gate.
Rejecting every proposal must fail useful retention/benefit qualification.

Known I-61 +0.3-rad A has zero harmful native-correct overrides. A relative
harm reduction on that held-out phase is therefore unscorable, not a 100%
improvement or a passing benefit gate. Report that fact; use the registered
per-phase nonincrease guard and pooled out-of-fold opportunities for the
relative reduction criterion. The pooled opportunity denominator is nonzero.

All three training fits must qualify before an overall out-of-fold success is
possible. Out-of-fold predictions come from three separately trained global
pairs; they do not demonstrate that one common fitted pair has independently
passed all three episodes. A full-data fit uses all three only as descriptive
retrospective selection, and needs a fresh case before any promotion.

Because the episodes informed earlier design, leave-one-start-out assessment
is retrospective internal group validation, not an unseen or statistically
independent holdout. A grid cell is not a new robot trial. Do not choose a
favorable pooled cell after inspecting held-out metrics or silently replace
the fixed training rule by a nearest miss.

## Possible feasibility obstruction, not an assumed result

The previously observed -1-rad bad point at 35.205 s has joint residual maximum
L=0.17270674084 N m and R||F||=2.2105652663 N m. Its accepted point error
29.843 mm exceeds the relevant native maximum 26.908 mm. Any feasible group
containing that state must reject it, hence a+2.2105652663 b < L, since passing
uses <=. This is a necessary condition from an already known I-61 event, not
an I-62 grid result.

For nonnegative coefficients, any useful correction i with
L_i >= L_bad max(1,RF_i/RF_bad) must also be rejected whenever this bad state
is rejected. Counting such unavoidable losses against the permitted retention
budget can prove a genuine obstruction if the count is large enough. No such
count or whole-grid conclusion is assumed here. A finite grid with no feasible
cell alone does not establish infeasibility of every continuous a,b pair.

## Source controls and independent audit

Freeze protocol, sources and exact I-61 input hashes before the new sweep.
Verify the old pair, zero-tolerance boundary, all-invalid and veto-off/native
references, fixed matched masks and tie-breaking. Independently reconstruct
grid counts, exact point metrics, every training gate, feasible counts and
fold/full-data selection from compact inputs. Check that held-out arrays are
excluded from training selection. Compare chosen policies on all held-out
frames and pooled/per-phase out-of-fold gates, preserving failed fits.

The additional joint rows remain filtered previous-load information compared
with current geometry; stable oracle points remain a single-resultant
compatibility proxy. Tuning two engineering tolerances does not calibrate a
posterior probability, validate real hardware or prove improved trajectory
control. No runtime or physics change is part of this iteration.
