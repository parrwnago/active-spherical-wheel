These assets are recovered from the user's `Active sphere2.zip` SolidWorks files. They are saved display triangles, not an exact solid-model translation. The original files were read locally without modification or upload. `manifest.json` records source hashes, decoder version, units, measured dimensions, and limitations. `assembly_components.json` lists every recovered component and its original assembly bounds.

`visuals/shell_positive.obj` and `visuals/shell_negative.obj` contain each shell's actual skin and equatorial ring. `visuals/carrier.obj` contains 116 internal component instances, including the polar module frames and exposed rollers; bearings and standoffs are omitted to reduce rendering cost. Four instances of imported GIM3505 motor parts have no recoverable saved display triangles. Their supplied STEP source remains available in the original CAD package.

The visual OBJ files use meters and a sphere-centered carrier frame: shell symmetry along x, observed polar roller-array travel along y, and z completing the right-handed frame. This places the provisional common shaft along y, consistent with the marked handoff views. The unfinished shaft and fork are absent from CAD, so this azimuth remains an interpretation. Apply no additional position or rotation to these three visual meshes. Attach shell_positive to the positive-x output shell, shell_negative to the negative-x output shell, and carrier to the fixed internal carrier. The frames/rollers shown in the carrier mesh are static visual detail; moving belt material is represented separately in the simulation.

Saved mesh vertices establish an outer spherical radius of 120 mm, inner radius of 110 mm, a polar aperture outer radius of 32 mm at axial coordinate 115.654655 mm, and an equatorial opening spanning axial coordinates -10 to +10 mm. The compact carrier display mesh lies within radius 119.166 mm. These are dimensions of the stored tessellation, not a manufacturing tolerance or mass-property report.

Use these meshes for display only. Convex collision over a shell mesh would bridge its polar opening; the simulator uses its separately defined collision surfaces. The ring mesh also has 372 open or nonmanifold edges. Material, mass, inertia, friction, motor capability, and belt compliance are not determined by this extraction.

To reproduce the assets using the original extracted CAD directory:

```bash
python3 assets/cad/recover_saved_meshes.py /path/to/extracted/cad
```

The extraction uses a small vendored subset of [sldprt-export](https://github.com/XRTC5/sldprt-export) at commit `ebde7524b3876c5d78c7e2248f2eba9b5725a6c0` under its MIT license (`vendor/LICENSE.sldprt-export`). Its container-layout description credits the [cadmpeg SolidWorks format documentation](https://github.com/cadmpeg/cadmpeg/blob/main/docs/formats/sldprt.md). Display and assembly mesh-cache blocks passed their stored CRC checks. The three final visual OBJ files loaded successfully in MuJoCo.
