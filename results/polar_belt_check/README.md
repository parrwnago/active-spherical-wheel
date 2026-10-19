# Polar transverse belt check

This independent check uses the default provisional MuJoCo model at carrier shaft phase π/2. Positive-x polar belt A touches the horizontal floor; belt B is airborne. At that phase the belt's local y surface-speed axis is the world lateral y axis.

The exact same full-state LQR gains are used with the belt commands enabled and with both belt commands forced to zero. Belt collision geometry and friction are present in both cases. No gain redesign compensates for the disabled commands.

| Input | Active belts | Belt commands disabled |
| --- | --- | --- |
| Initial roll +2° or −2°, no external force | Final tilt 0.000408° after 8 s | Fork touches floor at 0.975 s |
| World lateral push +0.5 N or −0.5 N for 0.1 s, starting at 1 s | Peak tilt 0.582623°; final tilt 0.001096° after 8 s | Fork touches floor at 2.08 s |

Peak active-belt speed is 0.20806 m/s for the initial-roll case and 0.06327 m/s for the push. In these symmetric cases the other three torque commands stay below 6.3 × 10⁻¹³ N m and the airborne belt command is zero. Thus the contacting transverse belt supplies the meaningful correction in these particular roll-only cases. The finite push impulse is ±0.05 N s; the tilt-only runs have zero applied world force or torque.

This result supports the intended transverse balancing mechanism in the modeled polar contact configuration. It does not establish a recovery envelope, stability for arbitrary perturbations, or real belt motor capacity. The physical parameters are provisional; the belt uses an ideal surface-speed servo without motor torque, current, load response, or rotor dynamics.

Run from the repository root:

```bash
.venv/bin/python results/polar_belt_check/reproduce.py
.venv/bin/python results/polar_belt_check/plot.py
```

`summary.json` contains the full configuration-independent run metadata and actual summary measurements. `model/` preserves generated model/configuration/mass data; `design.npz` preserves the gains. Each per-case NPZ contains named columns for state, requested command, bounded applied input, explicit disturbance, tilt, belt normal load, and floor-contact failure. `polar_belt_ablation.png` compares the positive-direction cases; the saved negative-direction cases are symmetric to numerical precision.
