# I51 joint-reference guard: design and saved-input validation

Status: isolated results-owned candidate implemented and frozen. Seven pure/saved-input contract groups pass; no native simulation or controller replay was run by this component owner. Live/runtime/default files are unchanged.

The native least-squares allocator tries to match its requested kinematic velocity. It does not intentionally relax the position target. This candidate retains that allocator and the original XY, body velocity and angular-rate references. Immediately before the one native feedback call, it changes only the three joint-rate references. Belt feedforward, estimated contacts, normal, gains and holding torque stay native. A reference outside the current speed/slew box is clipped to that box; a reference already inside is returned bit-for-bit. Recovery history can keep limiting a well-conditioned state after an earlier violation.

The fixed limits are the existing declared20rad/s and100rad/s² simulation assumptions from AllocationSettings. The20rad/s ceiling also matches incoming-shell preparation. They are not new hardware specifications, an actual speed guarantee or outcome-tuned parameters. At5ms the allowed reference increment is0.5rad/s. First history is zero; subsequent updates use measured elapsed command time. Held/dropout/zero-motion targets pass through the same hook. Stopping can therefore require reference deceleration rather than an immediate target reset.

API: `apply(controller,vref)` owns `controller._i51_joint_reference_guard` and mutates only `vref[6:9]`; the harness calls original feedback once afterward. `reset(controller)`, `JointReferenceBoxGuard.reset/update/state_dict` support lifecycle and clone checks. A fresh one-controller-per-case run needs no reset after initial construction. Main command logging, `_held` reference copies, `_vref` and `after_step` see the guarded vector because mutation precedes native feedback.

This is a reference guard, not a constrained dynamic allocator or trajectory governor. It accepts a kinematic rolling residual rather than requiring enormous joint references, while the LQR still sees the original body/position goals. It does not ensure total requested torque, actual speed, friction feasibility, balance or closed-loop tracking. Body-motion deviation remains an outcome, not an explicitly optimized position relaxation.

## Prior negative evidence reviewed before implementation

- I05 uniform damping0.01 reduced the +0.90 reference peak1760.5→4.4rad/s, but the −1.00 moving RMS worsened51.1→86.4mm.
- I06 bounded least squares used20rad/s,100rad/s² and belt0.3m/s/3m/s². All five references obeyed bounds, but +0.90 RMS worsened50.5→67.6mm; actual unloaded-shell speed reached44.3rad/s.
- I07 stronger effort improved +0.90 peak/final error but worsened triangle RMS24.4→35.2mm. I08 adaptive effort worsened −1.00 RMS41.3→77.1mm. Neither is evidence that finite reference demand alone fixes tracking.
- I12 showed direction/configuration and channel changes matter beyond tilt; a shell/belt boundary produced a30.16mm/s feasibility jump. I13 observed LQR steps5.79 actuator bounds despite bounded/slew-limited references. I14 difficult direction tracking remained275.07→278.52mm, with actual joint speed25.51rad/s despite20rad/s references.
- I42 preparation masking/share diagnostics did not remove the initial6.860s departure. I43 tilt shaping improved early demand/error but worsened full raw-reference lean and four turns. These interventions do not justify a generic smoothing-success claim.

The preserved exact log reviewed is `development/stage01/source_snapshot/ITERATION_LOG.md` (I05–08 lines176–299, I12–14 lines446–587, I42 line2389 onward, I43 line2515 onward). Existing runtime limit definitions are in the stage source snapshot of `rolling_allocation.py` at14; their meaning is stated in that module header. This candidate deliberately avoids their global effort/change optimization and belt/reference-body relaxation.

## Saved-input evidence, with no candidate dynamics claim

The pure guard filters8,801 ORIGINAL native reference inputs. It modifies419 samples, first at6.700s; output max10.0196rad/s and reference acceleration100rad/s². This is not the trajectory or contact history a closed-loop candidate would produce.

At the saved31.650s singular state, original joint targets `[488.60137,0,−1760.52505]` become `[0.787542,0,+4.519568]rad/s` under that original-input filter history. The exact current rolling residual increases from3.83e−13 to0.210134m/s; the history has not yet reversed the shell target. Original XY/body references and belt feedforward remain intact. A well-conditioned feasible local target is exactly unchanged in its declared initial box.

Same-state feedback algebra, holding the old state/K/loads fixed, changes raw torque `[111.389,−5.365,−141.067]` to `[−0.1071,+0.2320,+0.7305]Nm`. This is not controller replay or predicted motion; shellB still exceeds its0.6Nm bound. It directly illustrates why reference limits do not guarantee total-command feasibility.

[Pure contracts](development/stage01/report.json) · [Saved case/residual/algebra](development/stage01/saved_case.json).
