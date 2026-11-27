# Additional I-64 filter identities, before historical arithmetic

The primary useful comparison is against I-61's instantaneous Jacobian times filtered force on identical frames. Filtering a raw interval residual lowers high-frequency variation by construction; this alone does not demonstrate improved physical model fidelity. For a consistent independently propagated accumulator, gamma_filtered minus Filter(H_i F_interval_i) equals Filter(gamma_interval_i minus H_i F_interval_i), apart from explicitly controlled initialization.

Repeatedly initializing with the observed previous filtered gamma instead is only a one-step identity: the residual is (1-lambda_i) times the raw interval residual. Report that as an algebraic control, not an accuracy improvement. A shared history propagated from frozen past A assignments can separate accumulated historical bias from the contribution of the current candidate, but the resulting history score is not proof of the current contact.

Changing H with robot configuration/contact means Filter(H F) generally differs from H_current Filter(F). This noncommutation is the timing/geometry hypothesis to test. Fixed-H synthetic fixtures should commute exactly with matching initialization; changing-H fixtures and frozen-history transition fixtures should expose the distinction.

If midpoint geometry is not fixed before protocol/source sealing, it must not be added after outcomes to improve the result. Averaging maps from two different contact families assumes a half-interval transfer; it is neither measured midpoint contact nor an exact time integral.
