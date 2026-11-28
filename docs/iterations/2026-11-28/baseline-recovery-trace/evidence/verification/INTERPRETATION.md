# I-86 independent saved-state and support audit

One new unchanged native B060 replay ends at 106.5 s. All 79 precision fields
match the original I-82 B060 prefix over 21,301 rows, with only the declared
shortened final unheld flag adjusted. The 105.0–106.3 s trace contains 1,300
completed 1 ms solves under 260 correctly aligned 5 ms held commands. This
is a diagnostic prefix, not a full-route recovery trial or controller fix.

[Independent report](completed_stage01/report.json) and
[primary agreement](primary_agreement_stage01/report.json) preserve the checks.
The latter compares 52 arrays/vectors containing 53,131 scalar values,
including analytic versus independent MuJoCo kinematic COM calculations,
contact forces, pressure, held commands and all five fixed momentum windows.
Maximum absolute disagreement is 8.26034e-13. No dynamics integration is used
by this independent audit; it is not a second physical experiment.

## Physical accounting

Model mass is 1.86 kg. Whole-COM force-balance residual norm has RMS
0.000293668 N and maximum 0.00239117 N at 1 ms; at 5 ms the corresponding
values are 0.000276214 N and 0.00101107 N. All/admitted/native force selections
agree here; rejected rows contribute zero force. Integrated whole-window
momentum residual is [1.85613e-6, 6.38702e-7, 2.12499e-7] N s in world XYZ.
These residuals are descriptive consistency results. Exported force multiplied
by dt is sampled quadrature, not an exact solver impulse export.

The two no-positive-pressure times are 105.525 and 105.724 s. At each, the raw
export also contains zero contact rows and zero signed contact force. The
recorded 1 ms solve ending 105.525 s has vertical COM velocity change
-0.00981002 m/s, consistent with model gravity. This now resolves an actual
single 1 ms contact-free solve; it does not establish five milliseconds of
airborne motion. At 105.725 s support pressure returns at 36.0679 N.

Over the fixed 105.52–105.53 s kick window, root velocity changes
[2.39846, 2.25075, -0.508169] mm/s, whereas whole-COM velocity changes
[0.474523, 0.457834, -0.933873] mm/s. Root motion includes rotation relative to
the COM. Over the later 105.53–106.0 s response window, whole-COM velocity
changes [-15.8387, -3.78121, 0.501377] mm/s. The later motion is therefore
not simply the initial kick continuing unchanged. This does not attribute the
later response uniquely to a controller block or establish a beneficial fix.
The command newly issued at an interval end cannot cause the already recorded
state; the previous held command may have contributed.

## Multi-contact relevance in this window

Every admitted positive-pressure contribution belongs to shell A's tag 0:
23.5248 N s scalar normal-pressure sample integral, or 100% of that integral.
All other shell, belt, rim and body pressure shares are zero. The maximum
distance between any simultaneously loaded solver points is 1.22449 mm.
No sample contains two 10 mm-separated spatial components or two loaded motor
families at any of the predeclared 1%, 5%, 10% relative-pressure sensitivities.
The maximum secondary spatial-group and motor-family pressure shares are both
zero. Raw solver point counts are not independent support counts.

Thus this recording provides no reason to add a multi-contact estimator to
solve this particular recovery event. It does not exclude meaningful
multi-contact elsewhere, and the previous loaded-rim/V-groove fixture results
remain valid. The present evidence supports focusing on native recovery
through a single-shell contact/normal transition. Initial geometry/impulse,
held-command effects and later normal/reference/controller response still
require a causal intervention to separate fully.
