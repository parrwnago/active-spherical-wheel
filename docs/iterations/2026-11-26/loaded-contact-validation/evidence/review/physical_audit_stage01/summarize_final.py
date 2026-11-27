"""Post-outcome summary of sealed independent I57 audit; thresholds unchanged."""
from pathlib import Path
from collections import Counter
from datetime import datetime,timezone
import hashlib,json
import numpy as np
HERE=Path(__file__).resolve().parent
PATH=HERE/'final_all30/report.json'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
r=json.loads(PATH.read_text());loaded=Counter();free=Counter();linearity=Counter();null_failures=[];failure_rows=[];maxlin=[]
for event,e in r['events'].items():
 for label,pair in e['pairs'].items():
  for h,d in pair.items():
   for group,m in d['loaded'].items():
    loaded[m['status']]+=1;free[d['free'][group]['status']]+=1
    if m['status']not in('resolved_pass','unresolved_below_floor'):
     failure_rows.append(dict(event=event,pair=label,horizon=h,group=group,metrics=m))
    if group=='transverse'and m['predicted_nullspace']and not m['observed_quiet']:
     null_failures.append(dict(event=event,pair=label,horizon=h,metrics=m))
 for column,d in e['nonlinearity'].items():
  for h,groups in d['horizons'].items():
   for group,m in groups.items():
    linearity['unresolved'if m['unresolved']else'pass'if m['passes']else'fail']+=1
    if not m['unresolved']:maxlin.append((m['ratio'],event,column,h,group))
 full=e['pairs']['governed_minus_native'];half=e['pairs']['midpoint_minus_native']
 for h,d in full.items():
  for group,m in d['loaded'].items():
   o=np.asarray(m['observed']);s=np.asarray(half[h]['loaded'][group]['observed']);floor=m['floor']
   ratio=float(np.linalg.norm(o-2*s)/max(np.linalg.norm(o),floor));unresolved=max(np.linalg.norm(o),np.linalg.norm(s))<=floor
   linearity['unresolved'if unresolved else'pass'if ratio<=.25 else'fail']+=1
   if not unresolved:maxlin.append((ratio,event,'native_to_governed',h,group))
summary=dict(iteration='I-57',generated_utc=datetime.now(timezone.utc).isoformat(),
 purpose=__doc__,source_sha256=sha(Path(__file__)),independent_report_sha256=sha(PATH),
 canonical_report_sha256=sha(HERE.parents[1]/'analysis/stage01/report.json'),
 completed_cases=len(r['available_cases']),missing=len(r['missing_cases']),execution_failures=list(r['failures']),
 all_independent_case_checks=all(x['all_checks']for x in r['cases'].values()),
 both_duplicates_exact=all(e['duplicate_all_exact']for e in r['events'].values()),
 loaded_group_counts=dict(loaded),free_group_counts=dict(free),critical_model_group_failures=failure_rows,
 predicted_small_transverse_failures=null_failures,
 full_half_linearity_counts=dict(linearity),maximum_resolved_full_half_discrepancy=max(maxlin),
 endpoint_mode_change_cases=[k for k,v in r['cases'].items()if v['contact']['endpoint_mode_changed']],
 interpretation=[
 'Loaded major finite native-to-governed responses are accurate at both sampled single-support states. This is a short response validation, not contact/handoff prediction or a full trajectory cause.',
 'Early finite pair: free dynamics predicts opposite planar direction and excessive transverse response; loaded agrees. Late finite pair: free errors remain much larger but some meet the loose engineering screen.',
 'All predicted-small transverse responses stay below the registered absolute2mrad/s floor. This is absolute quietness within this experiment, not resolved directional accuracy.',
 'Five small late20ms cross-coupling failures remain: planar responses to full/half shellA and shellB torque probes, and yaw response to the full loadedbeltB speed probe. They are retained; model is not given a blanket pass.',
 'Full/half responses are nearly linear at fixed initial conditions. This rules out strong amplitude nonlinearity in these probes as a sole explanation for remaining errors; it does not prove constant geometry.',
 'No endpoint mode changes were observed. Contacts inside each5ms hold were not recorded. No simulator result is hardware evidence.',
 'At late anchor, changing shaft command from+1.197503Nm to native−1.5Nm at assumed60Nm/s requires44.96ms. First admissible+.897503Nm is exactly governed. Native/midpoint branches violate candidate slew as declared amplitude-only counterfactuals.'
 ])
(HERE/'final_summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:summary[k]for k in('completed_cases','loaded_group_counts','full_half_linearity_counts','maximum_resolved_full_half_discrepancy','predicted_small_transverse_failures')}))
