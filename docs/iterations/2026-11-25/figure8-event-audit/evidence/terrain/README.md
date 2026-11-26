# I-41 terrain geometry and matched flat counterfactual

The saved Mars trace first crosses the predeclared sustained 20 mm local-path-distance threshold at 6.860 s, before the first reference velocity-component reversal at 11.775 s. This audit evaluates terrain geometry and the existing reference formula. It does not modify the observer, LQR, commands, gains, or controller defaults.

One new 44 s Gazebo/DART flat figure-eight run completed with 8,801 guarded 5 ms controller samples and no fall. Its moving RMS position error is 16.821 mm, peak time-aligned error 35.450 mm, and final time-aligned error 0.115 mm. The reused Mars trace has 50.486 mm moving RMS, 190.901 mm peak, and 190.191 mm final error. The same timed reference and derivatives are byte-identical, as checked by the event-analysis owner. Terrain relief and obstacles are removed globally; the standard terrain-specific balanced initial pose and observer history differ. This comparison does not isolate one slope as a cause.

At the first Mars departure, the DEM slope under both actual and desired XY is 12.597 degrees. The observer normal is 0.038 degrees from the geometric OBJ facet beneath the equivalent contact XY at that instant. Actual platform tilt is 8.595 degrees and logged reference tilt 12.190 degrees; their full up-vector separation is 4.705 degrees. Prior normal and attitude transitions remain relevant even when the instantaneous normal agrees. No causal diagnosis follows from one trace.

| Time (s) | Desired-XY DEM slope | Contact-XY OBJ facet slope | Estimated slope | Actual platform tilt | Logged reference tilt |
| --- | ---: | ---: | ---: | ---: | ---: |
| 6.360 | 12.597° | 9.395° | 9.305° | 7.979° | 9.051° |
| 6.660 | 12.597° | 9.732° | 9.412° | 6.833° | 9.150° |
| 6.860 | 12.597° | 12.597° | 12.562° | 8.595° | 12.190° |
| 7.060 | 12.597° | 12.054° | 11.775° | 13.163° | 11.399° |

The geometric facet evaluated at the saved future equivalent-contact position differs from the current 6.860 s observer plane by 7.778 degrees at 50, 100, and 200 ms. This demonstrates a representation change in the saved trajectory; it is not a dynamic-forecast score or proof that a different contact predictor would improve control.

## Normal definitions and limitations

The logged `ground_normal` is `MarsYard.closest_point`'s piecewise-planar DEM tangent near the wheel center. It is not a smooth terrain normal and is not the collision engine's force-weighted contact normal. OBJ `vn` records are rendering normals. The audit computes geometric triangle normals from the actual rounded OBJ vertex coordinates and the original writer triangulation.

The equivalent physical contact point can average several contacts and lie over another triangle. At 30.025 s, the logged closest tangent has slope 4.525 degrees, the projected contact facet has slope 12.361 degrees, and the observer estimates 12.358 degrees. Thus a large observer-versus-logged-tangent angle cannot by itself establish an observer error against physical contact truth. Rocks/lava are excluded from every DEM evaluation here. Exact OBJ quantization can select a different adjacent facet at an edge; the OBJ-versus-unrounded-DEM discrepancy exceeds 0.1 degrees in 200 of 8,801 equivalent-contact projections, versus one body projection and no desired-path projections. Facet switches at projected equivalent-contact XY are not evidence of physical collision-facet switches.

The field `truth_normal_substituted_reference_tilt_deg` in the first-stage NPZ/report means applying the existing controller formula with the evaluator's DEM closest tangent substituted for the estimated normal. It is not a physical ground-truth contact normal or a required-lean benchmark. The corresponding report states this definition explicitly. The addendum repeats this warning. Geometry remains evaluated even for unloaded samples; first-departure and turn context is the relevant loaded event audit, while global geometry statistics do not assert contact existence.

## Evidence and stages

- [Terrain report](report.json), [exact input hashes](provenance.json), [event snapshots](event_terrain_context.json), and [terrain series](terrain_series.npz).
- [Path and terrain map](terrain_paths.png), [full timeline](terrain_timeline.png), and [first departure geometry](facet_addendum/first_departure_geometry.png).
- [Derived future-plane diagnostic](facet_addendum/report.json) and [its source/input hashes](facet_addendum/provenance.json).
- [Flat prelaunch specification](flat_control/flat_case_spec.json), [completion/source audit](flat_control/completion_audit.json), [raw summary](flat_control/campaign/runs/flat_figure8/summary.json), and [launch log](flat_control/launch.log).

The first-stage artifact manifest is retained as written before the separate facet addendum and this delivery note. Each addendum has its own manifest. `delivery_manifest.json` inventories final compact evidence, excluding generated Gazebo workspace files and Python bytecode. No earlier source, result, or manifest was replaced.

The exact flat wrapper and original helper/controller snapshots are preserved before execution. It reuses the I-40 native runner while changing only local terrain construction. The floor uses the saved initial Mars evaluator height 1.12813138961792 m, 500 × 500 vertices over [-23.5,23.5]² m, the same original DEM writer ordering/diagonal/precision, and nominal friction 0.7. Full commands and source snapshots are in the flat construction and run folders. Python compilation passed before execution; no new unit tests or additional simulations were needed for this geometry-only analysis.
