# Motion evidence for the preparation PDF

This is a saved-data audit. It ran no physics, model forward pass, controller, or new contact estimator. The numeric example below reproduces the saved study's values using only rotation, cross-product and least-squares arithmetic. All old inputs were hashed before and after; they remained unchanged. Full values, checks and source paths are in [motion_audit.json](motion_audit.json).

## Why the common shaft and one hemisphere can lose a direction

The two rotation axes are different in three dimensions, but the useful **horizontal contact-velocity vectors** can become parallel. One loaded hemisphere is enough; both hemispheres need not touch the ground.

Let `r` be the vector from wheel centre to the located contact, `R` the fork orientation, and `φ` the shaft angle. The repository's shaft is about fork y and the hemisphere joints are about carrier x:

```
a_shaft = R e_y
a_shell = R R_y(φ) e_x
J_contact = P_xy [a_shaft × r, a_shell × r]
v_body_xy + J_contact [shaft_rate, loaded_shell_rate]ᵀ = 0
B = −J_contact
```

This freezes the body's angular motion, as the existing horizontal feedforward calculation does. `B` maps **joint rates in rad/s** to horizontal body velocity in m/s; its entries have units m per rad. It is not a torque-to-velocity model. The code's allocation normalization is `d = S z`, with joint scales 5 rad/s and belt scales 0.3 m/s. Those scales are preferences; they are not the reference bounds or actuator torque limits.

The saved grid contains `9 roll × 9 pitch × 73 shaft × 3 normals = 17,739` configurations and `× 8 directions = 141,912` requested-direction cases. Its yaw and shell material angles are zero. It includes flat ground and two 10° slope normals. There are 15,914 clear patch-interior cases; the rim fallback and boundary cases are separately labelled. Uniform-grid fractions are not probabilities of encountering a failure while driving.

### One verified single-contact example

Flat normal `[0,0,1]`, radius 0.12 m, **roll −20°**, pitch/yaw zero, shaft −90°, one shell-B support. The point lies inside shell B, clear of the shell/belt boundary. From the saved model axes and `r = [0,0,−0.12] m`:

```
B = [ 0.1127631145   0.0410424172 ] m/rad
    [ 0              0           ]
       common shaft   shell B
```

Physical singular values are `[0.12, ~0] m/rad`; singular values of `B × diag(5,5 rad/s)` are `[0.6, ~0] m/s`, matching the saved study. This distinction prevents confusion about normalization.

- A +x request of 0.06 m/s is feasible: minimum-normalized-norm rates are `[0.4698463, 0.1710101] rad/s`.
- A +y request of 0.06 m/s is outside this instantaneous map's range. The least-squares result leaves 0.06 m/s residual, even without speed bounds. A cost cannot create that missing direction.
- At the **same −20° roll and same shell-B contact**, changing shaft phase to −0.6 rad (−34.377°) gives `B = [[0.1127631, 0.0231743], [0, −0.0990403]]`; normalized singular values become `[0.6, 0.4653371] m/s`. The y request is then feasible. This second example is the same study's saved supplementary example, not a point on its 5° grid.

[Figure: drive directions](figures/drive_direction_example.png)

This establishes why **absolute platform tilt alone is inadequate**. Shaft phase, the contact vector/normal, fork orientation and requested direction matter. On flat ground, changing yaw rotates both horizontal columns together, preserving their singular values while rotating the weak direction relative to a fixed world-direction request. On a slope, yaw relative to the terrain normal can change the geometry itself. The ideal axisymmetric shell's material angle does not itself rotate its joint axis; its rate and the evolving shaft/body/contact configuration remain relevant.

### The actual early bad case: near alignment, huge opposing references

The saved `plain_figure8/phase_+0.90` event at 31.650 s has only shell B carrying estimated support. Reconstructing from its **float32 saved pose and estimated point** gives:

```
B ≈ [ 0.004444464   0.001201386 ] m/rad
    [ 0.115361318   0.031998645 ]
```

Its normalized singular values are approximately `[0.5990272, 0.0001512163] m/s`, condition number 3961.39. The two columns point almost the same way. Matching the desired `[0.0580885, 0.0311705] m/s` algebraically requires reconstructed rates `[502.664, −1811.226] rad/s`. Their large opposing contributions cancel in the well-driven direction to produce a small residual velocity. The actual logged references were `[488.601, −1760.525] rad/s`; the reconstructed matrix is approximate, so these must not be presented as identical.

A saved instantaneous counterfactual adds a normalized **speed-demand** penalty:

`min_z ||B S z − v_des||² + λ² ||z||²`, with `S = diag(5,5) rad/s` and `λ = 0.01 m/s`.

It gives rates `[0.383258, −0.339637] rad/s`, achieved velocity `[0.0012953, 0.0333452] m/s`, and residual **56.835 mm/s**. The reduced demand is obtained by allowing tracking error in the poorly driven direction. This is not measured mechanical effort, torque, power or energy, and it is not a successful closed-loop result. The existing full allocator also has reference-change costs and assumed speed/acceleration bounds.

At that saved sample the requested shaft/shell torques were about `111.39 / −141.07`, whereas applied values were clipped to `1.5 / −0.6`. The separately logged synchronization command delta was zero **at that sample**. This distinguishes the immediate LQR correction from a direct synchronization blend there; it does not rule out earlier contact-transfer or state-history effects. A support transition occurred around the original event, but a transition is not necessary for the geometric rank loss demonstrated above.

## What the figure-eight turning evidence actually says

The six audited events are **reference x- or y-velocity component reversals**, not full stops. The reference vector speed at those events is 58–112 mm/s. The following quantities answer different questions:

| Quantity | Meaning |
| --- | --- |
| Timed position error | Distance from the robot to the reference at the same clock time. Lag contributes to it. |
| Local geometric path distance | Closest reference-polyline point within ±2 s, constrained to the local branch. It need not equal the nominal-time Frenet normal error. |
| Geometric component overshoot | Positive excursion beyond the reference x/y extremum, in the incoming direction, within the fixed event window. |
| Component reversal lag | Time of the first sustained outgoing-sign actual velocity crossing minus the reference component crossing; negative means early. |

The saved first y reversal illustrates the distinction. At reference time **11.775143 s**, Mars has **86.464 mm timed error**, but only **11.520 mm local path distance**. Its actual y reversal is **1.101 s late**, yet its geometric y overshoot is **zero**: it turns without reaching the planned y maximum. Calling this simply “overshoot” hides the delay/corner-cutting behavior. The flat run has **8.028 mm** geometric overshoot and **0.193 s** lag at the same reference event.

| Event | Time (s) | Mars overshoot (mm) | Flat overshoot (mm) | Mars lag (s) | Flat lag (s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1: y | 11.775 | 0.00 | 8.03 | +1.101 | +0.193 |
| 2: x | 15.377 | 0.00 | 15.62 | +0.018 | −0.029 |
| 3: y | 18.301 | 0.00 | 23.44 | −0.469 | +0.172 |
| 4: y | 23.699 | 9.84 | 28.96 | −0.444 | +0.150 |
| 5: x | 26.623 | 28.02 | 14.97 | −0.518 | +0.002 |
| 6: y | 30.225 | 60.20 | 26.49 | +0.569 | −0.026 |

[Figure: geometry and timing at the six turns](figures/turn_geometry_and_timing.png)

The Mars and flat saved time grids and reference positions/velocities match exactly. Their terrain-dependent balanced initial states differ, so this is a global terrain/setup contrast, not isolation of one small slope. Flat-ground overshoots demonstrate that terrain relief is **not necessary** for overshoot. They do not identify a unique controller defect or eliminate dynamics, reference design, contact routing, yaw and model effects.

The first sustained Mars departure from the local path starts at **6.860 s**, before the first of these reversals at 11.775 s. The matched flat run first crosses the same sustained 20 mm local-distance criterion at **17.735 s**. A generic component-reversal explanation therefore cannot explain the onset of the earlier Mars departure.

## Interventions changed tradeoffs; they did not settle a universal cause

- **I42 synchronization mask, 4–10 s:** moving timed RMS improves from 50.486 to 41.681 mm and the old full-route gates pass in that one run, but geometric overshoot worsens at **4 of 6** later turns. First-turn reversal lag improves from 1.101 to 0.832 s while geometric overshoot grows from 0 to 18.048 mm. All six turns occur **after synchronization was restored**; the intervened physical/controller history carries forward. These results are not a direct test of disabling synchronization at each turn.
- **I43 shaped tilt plus consistent angular-rate reference:** early local-path RMS improves from 38.661 to 29.893 mm and the examined belt demand is reduced, but peak lean relative to the **original raw reference** rises from 8.709° to 13.581°, and **4 of 6** turn overshoots regress. The registered screen fails; the candidate was not adopted. A smoother reference does not establish a globally better trajectory or attitude response.

[Figure: saved intervention tradeoffs](figures/intervention_turn_tradeoffs.png)

**Confirmed:** same-contact geometry can lose authority; the saved bad event contains enormous opposing rate references and clipped feedback; delay, geometric overshoot and timed tracking error differ; overshoots also occur on flat ground; the tested interventions have mixed outcomes.

**Still unresolved:** the dominant cause of each individual turning error, how much comes from evolving contact geometry versus reference/dynamics/feedback/history, and whether a new allocation or reference policy would improve all acceptance metrics. The configuration example is not proof that the six audited figure-eight errors were caused by that same singularity. Neither the saved event associations nor these single-run interventions justify that extrapolation.

## Exact implementation and evidence sources

- Axes: [model.py, shaft joint](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/model.py:303), [hemisphere joints](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/model.py:319).
- Contact Jacobian and sign/normalization: [experiments.py](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/experiments.py:79); [normalized regularization](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/experiments.py:29).
- Effort/change objective and limits: [rolling_allocation.py](/home/parrwnago/active-spherical-wheel/src/spherical_wheel/rolling_allocation.py:35).
- Grid source: `results/configuration_study_20261125/map/source_snapshot/scripts/map-rolling-configurations.py`; saved summary, provenance, configurations CSV and same-attitude JSON are individually hashed in `inputs_before.json`.
- Original near-singular event: `results/contact_diagnostics_20261125/allocation_event_evidence.json` and matching `baseline/plain_figure8/phase_+0.90/rollout.npz`.
- Corrected branch-aware turn definitions and evidence: `results/figure8_event_audit_20261125/events/selection_stage02_corrected/` and `events/full_audit/report.json`.
- Later tradeoffs: `results/handoff_factorial_20261125/analysis/turn_addendum/report.json` and `results/tilt_reference_20261125/analysis/stage01/report.json`. The I42 turn addendum is explicitly secondary/post-primary analysis; no primary gate was changed.

Reproduce the saved-data synthesis with `.venv/bin/python results/presentation_preparation_20261126/motion_audit/build_motion_audit.py` in a fresh output directory after preserving this one. The builder refuses to overwrite its final JSON. The exact executed source is retained in `source_snapshot/`.
