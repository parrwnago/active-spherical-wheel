# Source line citation correction

The sealed `SOURCE_MAPPING.md` is retained unchanged. Its prose gives several approximate command-stage line ranges with small offsets. Exact current source locations, verified with `rg -n -F`, are:

| Operation | File | Exact line |
|---|---|---:|
| Record pre-sync `lqr_command = -K @ error` | `src/spherical_wheel/offroad_control.py` | 688 |
| Observer-record synchronization blend | same | 700 |
| Alternate synchronization blend | same | 707 |
| Record transition delta | same | 709 |
| Add heading torque | same | 713 |
| Apply unload fade | same | 717 |
| Native amplitude clipping | same | 721 |
| Replace shaft gain row with ordinary design row | same | 786 |
| Native pre-sync command composition | same | 816 |
| Unload-fade method | same | 873 |
| Torque-only fade operation | same | 888 |
| Initialize clipped synchronization commands | `src/spherical_wheel/synchronization.py` | 174 |
| Overwrite belt target | same | 232 |
| Overwrite shell torque target | same | 243 |
| Store patch blend weight | same | 250 |
| Obtain native controller result | `results/balance_priority_20261126/campaign_tools/run_case.py` | 248 |
| Overwrite generic clipping with final candidate clipping | same | 271 |
| Preserve native clipping delta separately | same | 282 |

This is a citation correction only. The source hashes, helper, pipeline mapping, numerical criteria and arithmetic are unchanged.
