# I-69 standalone probe design

Pre-execution design: do not compile or execute until the parent freezes this
source, installed headers/libraries, inputs and protocol. No native runtime
source, plugin or binary is modified by this probe.

The existing installed DART `BoxedLcpSolver::solve` interface accepts a padded
row-major matrix and mutable solution, right-hand side, bounds and friction
index arrays. Preserve every original input before the call. Check solutions
against those originals; mutated post-call arrays may reflect elimination,
permutation and friction-bound freezing.

The frozen controlled witness is `A=[[1,1.5,0],[1.5,4,0],[0,0,2]]`, `b=[1,4,0]`,
`lo=[0,-.7,-.7]`, `hi=[infinity,.7,.7]`, `findex=[-1,0,0]`, `nub=0`, initial
`x=0`, and ODE row stride four. It is symmetric positive definite. Run installed
Dantzig with early termination both true and false, and installed default PGS
with early termination false. No option sweep or solver tuning is performed.

The mathematical friction check uses the final normal impulse for each
dependent row: `lo_eff=lo_original*abs(x[normal_index])` and similarly for the
upper bound. Compare this with the bounds retained/mutated by the solver, and
report original `w=A*x-b`. A zero normal must imply zero tangent under these
dependent component bounds. Do not use the circular Coulomb norm as a pass
condition: DART's independent two-tangent component boxes allow a different
combined norm. Dantzig restores solution `x` to its original order but may leave
`A,b,lo,hi,findex` permuted/mutated; post-call auxiliary entries are not aligned
with original variable indices. They are recorded for provenance, not compared
entry by entry as if the order were unchanged.

The known coupled solution for this witness is `x_N=1/(1+1.5*.7)`,
`x_T=.7*x_N`, `x_T2=0`; this algebraic comparison is not a result from an
experimental solver run. PGS is not presumed to converge on this witness.

A prescribed witness can expose an implementation mechanism but cannot establish
the cause of the robot's recorded 35.205 s event. The parent separately owns a
private solve interposer and an unchanged native replay with byte-parity checks.
Once that capture schema is frozen, this directory will independently check
the original captured matrices, final-dependent bounds and complementarity.
For every captured problem retain algorithm, call ID, native time/step, original
and final arrays, success/early-termination flags and matrix stride. A failed
primary call must not be mislabeled as the final applied solver result.
