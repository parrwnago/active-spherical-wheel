# I-69 installed DART controlled probe

Completed phase A, using the prescribed 3-by-3 positive-definite problem and the
existing installed `libdart`. No native library/plugin/controller was edited or
rebuilt. Source, headers, libraries, protocol and environment were sealed in
`stage01/source_input_seal.json` before compilation or execution.

| Installed solver call | Returned success | Final normal impulse | Final first tangent impulse | Final-dependent bound violation |
|---|---:|---:|---:|---:|
| Dantzig, early termination true | true | 0 | 0.7 | 0.7 |
| Dantzig, early termination false | true | 0 | 0.7 | 0.7 |
| PGS, default options | false | 0.0393073 | 0.0275151 | 0 |

Dantzig's result satisfies the frozen constant-box complementarity equations
exactly. It fails the bound obtained by recomputing friction capacity from the
final normal impulse. This is consistent with the documented ODE approximation:
normal impulses are solved first, tangent bounds are formed from that solution
once, and subsequent tangent pivots may change the normal impulse without
recomputing those bounds.

In plain terms, the tangent allowance was set while the contact was pressing
against the ground. The later solve removed that pressing impulse, while
retaining the earlier tangent allowance. The controlled example then produces
tangential impulse with zero final normal impulse.

PGS does not provide a demonstrated fix. Its returned vector respects the
final-dependent boxes but fails normal complementarity; the unit-step projected
KKT residual is 0.91942, and the solver reports failure. No PGS options were
tuned. Native DART's policy for applying a finite failed fallback must be
checked separately in any actual-scene interpretation.

This establishes a mechanism in the installed solver, not yet the exact cause
of the recorded robot event. Phase B needs original native LCP arrays and
contact-impulse association, with byte-exact replay parity. A returned solver
success flag alone is not a final-dependent friction consistency certificate.

Artifacts: `stage01/probe_output.json`, `stage01/analysis.json`, build/runtime
logs, linked-library report and `stage01/artifact_manifest.json`.

Compile only into this diagnostic directory:

```sh
/usr/bin/c++ -std=c++17 -O2 \
  -Ibuild/gazebo-env/include -Ibuild/gazebo-env/include/eigen3 \
  results/contact_force_boundary_20261127/bench/controlled_probe.cpp \
  -Lbuild/gazebo-env/lib \
  -Wl,-rpath,/home/parrwnago/active-spherical-wheel/build/gazebo-env/lib \
  -ldart -o results/contact_force_boundary_20261127/bench/stage01/controlled_probe
```

The probe writes one JSON object to stdout. Recompute the original-array
checks with `analyse_probe.py --input PROBE_JSON --output NEW_ANALYSIS_JSON`.
The JSON's null upper bound denotes infinity. Dantzig's post-call auxiliary
arrays may remain permuted; only the final solution is interpreted in original
index order.
