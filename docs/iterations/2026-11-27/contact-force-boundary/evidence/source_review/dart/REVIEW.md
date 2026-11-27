The strongest source-level explanation is a one-time friction-bound calculation
inside the installed solver's ODE/Dantzig implementation. It is a concrete,
testable mechanism. This source review alone does not establish that it caused
the recorded I-68 contact rows.

All source downloads use the official DART `v6.19.5` tag. Its annotated tag
`563dff7707bc878e52793cbe794d9c5bb910a8c1` resolves to commit
`ce5c6526035ccc9a22d895670b2579052a2b8903`. URLs, SHA256 hashes, retrieval
time, failed optional path lookups, and tag responses are preserved. The
installed package is `dartsim-cpp-6.19.5-h000b19c_0` in `build/gazebo-env`.
The five reviewed public headers are byte-identical across official downloads,
the installed environment, and its cached package. The external ODE solver
binary also matches its cached package byte-for-byte. The installed DART core's
executable `.text` matches the package exactly; its differing `.rodata` is
consistent with the package's explicit binary prefix-relocation record.
These bindings support the audited API/source path without replacing runtime
capture as evidence of which solver actually ran.

| Boundary | Exact source locations | Finding |
| --- | --- | --- |
| Contact setup | `dart/constraint/ContactConstraint.cpp:394` | Contact coordinates are normal plus two tangent directions. Normal impulse has lower bound zero; each tangent starts with coefficient bounds ±μ and `findex=0` referencing its contact normal. Surface-motion targets modify the velocity right-hand side. |
| Global assembly | `dart/constraint/BoxedLcpConstraintSolver.cpp:191` | Each local friction index is shifted to its global normal index. The primary receives a zero initial solution and `nub=0`. Default construction uses Dantzig with PGS fallback; actual configured calls require capture. |
| Dantzig entry | `dart/constraint/DantzigBoxedLcpSolver.cpp:51` | Delegates to the external ODE `dSolveLCP`; incoming `nub` is ignored and zero is passed. |
| Friction ordering | `dart/external/odelcpsolver/lcp.cpp:495` | Dependent friction rows are moved after the other rows. |
| Frozen friction boxes | `dart/external/odelcpsolver/lcp.cpp:852` | On reaching the first friction row, all dependent bounds are computed once from the normal impulses already solved. Later pivots can change those normal impulses; this block does not run again. The returned solution can therefore obey its frozen boxes while failing boxes recomputed from its final normal values. |
| Return ordering | `dart/external/odelcpsolver/lcp.cpp:759` | Only solution and residual vectors are unpermuted. Mutated A, b, bounds and friction indices must not be interpreted as original-order terms. |
| Applying/reporting force | `dart/constraint/ContactConstraint.cpp:647` | World contact force is constructed from the normal and both tangent impulses divided by the time step. The same impulse components are applied to the bodies. There is no final projection of tangential impulse onto bounds from the final normal impulse. |
| Solver acceptance | `dart/constraint/BoxedLcpConstraintSolver.cpp:279` | The primary Boolean and NaN check determine fallback. A finite fallback can be used even if its Boolean is false. No final independent friction-bound or complementarity audit is performed before application. |
| PGS comparison | `dart/constraint/PgsBoxedLcpSolver.cpp:132` | Recomputes each dependent bound from the currently stored normal each coordinate update. Default settings are 30 iterations and no randomized ordering. This is a comparison control, not evidence that changing the production solver improves the robot. |

The public input uses a row-major matrix with `n` rows and padded stride
`dPAD(n)`: unchanged for `n<=1`, otherwise rounded up to a multiple of four.
Both solvers use this stride. Release assembly does not initialize all matrix
padding; finite/symmetry checks should inspect the active `n×n` coefficients.
Capture the complete original terms before delegation because the solvers can
modify them. Check final coupled bounds and complementarity with original
coefficients/indexing and returned original-order solution, not mutated arrays.

The proposed confirmation is an isolated call to the installed solvers on a
fixed, positive-definite coupled contact matrix, followed by an unchanged native
replay with a delegating solver interposer. The replay should log original and
returned terms, solver type/return status, and fallback calls around the four
already fixed events, and require the existing 59-field native precision parity.
This review made no runtime changes, compiled no probe, and launched no physics.

Independent checks must distinguish the per-axis box approximation from a
circular friction cone: a tangential resultant can exceed μ times normal by
up to the box's diagonal factor without violating its two component bounds.
With a genuinely zero normal impulse, either formulation requires zero tangent
impulses. The observed force/normal boundary and the algorithmic witness still
need to be connected by captured native terms before making a causal claim.
