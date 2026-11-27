# Private I-69 hook build and contract

This is a diagnostic library, not a replacement installed solver/plugin. The
source seal in `preflight_stage01/source_input_seal.json` precedes compilation.
The following command reproduces the recorded build from the repository root
with this workspace's Gazebo environment; change the absolute prefix for a clone:

```sh
build/tools/bin/micromamba run -p build/gazebo-env c++ -std=c++17 -O2 -fPIC -shared \
  results/contact_force_boundary_20261127/instrumentation/solver_probe.cc \
  -I "$PWD/build/gazebo-env/include" -I "$PWD/build/gazebo-env/include/eigen3" \
  -L "$PWD/build/gazebo-env/lib" -Wl,-rpath,"$PWD/build/gazebo-env/lib" \
  -ldart -ldl -o results/contact_force_boundary_20261127/instrumentation/preflight_stage01/libsolver_probe.so
```

The recorded compile exited0 with no console output. This recipe is written
after execution from the tool invocation, not presented as a pre-build receipt.
Binary SHA256: `8f792900c0546929cbb13bee88894915bc2fe7c60d4b057a9478f1bf3c11f96e`.

For the standalone controlled executable only, `LD_PRELOAD` selects that private
library, `ASW_I69_CAPTURE_ALL=1` enables recording and `ASW_I69_PROBE_PATH` names
a new output file. The three-call contract preserved the original stdout JSON
exactly. Its seal/results and packets are in `preflight_stage01/` locally.

For native replay, the launcher applies preload only to child Python, leaves
capture inactive during setup and uses the C context setter for four fixed
holds. Installed libraries/plugins retain their hashes. Source/package/header
bindings and every applied force reconstruction validate the ABI use. Full
native source/asset/input prerequisites remain those in the phase-B protocol;
cached analysis replay requires only Python and NumPy.
