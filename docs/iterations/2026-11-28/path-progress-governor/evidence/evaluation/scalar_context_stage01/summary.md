# I-78 independent audit and +0.3 peak contexts

Independent scalar leaves: 564; all agree with unchanged canonical report: True.
Reconstructed performance/goal/stall checks: 82; all decisions agree: True.

This is a posthoc audit of the existing six episodes, not additional simulations or a repository test suite.

| +0.3 policy/event | Time (s) | Virtual time (s) | Clock rate | Tilt (deg) | Speed (m/s) | Own-target up error (deg) | Ref speed (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| N absolute_tilt_peak | 20.255 | 20.255000 | 1.000000 | 9.396590 | 0.060125 | 1.628984 | 0.069450 |
| N planar_speed_peak | 19.730 | 19.730000 | 1.000000 | 7.617683 | 0.181902 | 1.229081 | 0.070020 |
| G absolute_tilt_peak | 20.070 | 19.583064 | 0.740825 | 9.969799 | 0.102820 | 2.278724 | 0.051951 |
| G planar_speed_peak | 19.730 | 19.373248 | 0.550877 | 7.421121 | 0.228362 | 1.376346 | 0.038691 |

Failed frozen checks:
- 0.3_absolute_tilt_peak_deg: 9.96979943202; limit 9.89659026026; native 9.39659026026.
- 0.3_planar_speed_peak_m_s: 0.22836237586; limit 0.210092664916; native 0.181902422651.

Posthoc observational contexts. Retiming changes the physical state history, own raw attitude target and future contact evolution. These synchronized logs alone do not isolate which mechanism caused the +0.3 peak regressions.
