"""Independent I52 route/provisional screen arithmetic from raw/proven headsets."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil
import numpy as np
H=Path(__file__).resolve().parent;F=H.parents[1];ROOT=F.parents[1];PHASES=('m100','p030','p090');VARIANTS=('B','R','N','NR')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def main():
 out=H/'screen_stage02';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'verify_screens.py')
 paths=[F/'analysis/stage01/report.json',H/'numeric_stage01/report.json',F/'protocol.json',F/'campaign/resolved_case_paths.json'];mapping=json.loads(paths[-1].read_text())['cases']
 paths.extend(ROOT/p/'precision.npz'for p in mapping.values());hashes={str(p.relative_to(ROOT)):sha(p)for p in paths};save(out/'before_arithmetic.json',{'utc':datetime.now(timezone.utc).isoformat(),'source_sha256':sha(__file__),'input_sha256':hashes,'scope':'Independentrawroutegatesandfixedregisteredprovisionalscreens; numericheadlines/72turnsfromindependentlyverifiedstage. No physics/controller/model/reducer/helper imports.'})
 r=json.loads(paths[0].read_text());qa=json.loads(paths[1].read_text());proto=json.loads(paths[2].read_text());limit=proto['provisional_screen'];checks=[];gates={}
 for case,info in qa['cases'].items():
  assert info['available']
  with np.load(ROOT/mapping[case]/'precision.npz')as p:
   h=info['headlines'];closure=float(np.linalg.norm(p['exact_qpos'][-1,:2]-p['exact_qpos'][0,:2]));saturation=float(np.mean(np.any(abs(p['exact_native_requested'][:-1])>p['exact_input_bounds'][:-1],axis=1)))
  g=dict(completed=bool(info['completed']),rms=h['moving_rms_m']<=.05,p95=h['moving_p95_m']<=.1,max=h['full_peak_m']<=.2,final=h['final_m']<=.05,attitude=h['peak_reference_error_deg']<=8,saturation=saturation<=.1,settled=h['settled'],closure=closure<=.05)
  assert g==r['cases'][case]['original_route_gates']['gates'],case;checks.append(case+'/all_original_routegates');gates[case]=g
 comparisons={}
 for phase in PHASES:
  comparisons[phase]={}
  for name,e in r['per_phase'][phase]['comparisons'].items():
   b=qa['cases'][e['baseline_id']];c=qa['cases'][e['candidate_id']];bh=b['headlines'];ch=c['headlines'];changes=[]
   for i,(bt,ct)in enumerate(zip(b['turns'],c['turns'])):
    valid=bt['window_complete']and ct['window_complete']and bt['actual_reversal_lag_s']is not None and ct['actual_reversal_lag_s']is not None
    overshoot=None if bt['geometric_component_overshoot_m']is None or ct['geometric_component_overshoot_m']is None else ct['geometric_component_overshoot_m']-bt['geometric_component_overshoot_m'];lag=None if not valid else ct['actual_reversal_lag_s']-bt['actual_reversal_lag_s']
    changes.append(dict(event_index=i+1,coverage=bool(valid),overshoot_pass=overshoot is not None and overshoot<=limit['every_turn_overshoot_increase_max_m']+1e-12,lag_pass=lag is not None and lag<=limit['every_turn_reversal_lag_increase_max_s']+1e-10,overshoot_difference_m=overshoot,signed_lag_difference_s=lag))
   test=dict(completed_no_fall=c['completed']and r['cases'][e['candidate_id']]['failure']is None,moving_window_complete=c['completed_duration_s']>=41-1e-9,moving_RMS_10pct=ch['moving_rms_m']<=bh['moving_rms_m']*(1-limit['moving_RMS_relative_reduction_min'])+1e-12,peak_error_no_regression=ch['full_peak_m']<=bh['full_peak_m']+limit['peak_error_increase_max_m']+1e-12,final_error_no_regression=ch['final_m']<=bh['final_m']+limit['final_error_increase_max_m']+1e-12,peak_relative_lean_no_regression=ch['peak_reference_error_deg']<=bh['peak_reference_error_deg']+limit['peak_reference_lean_increase_max_deg']+1e-10,original_settled=ch['settled'],original_route_gates=all(gates[e['candidate_id']].values()),all_six_turn_coverage=len(changes)==6 and all(x['coverage']for x in changes),every_turn_overshoot=all(x['overshoot_pass']for x in changes),every_turn_lag=all(x['lag_pass']for x in changes),common_planned44_coverage=b['completed_duration_s']>=44-1e-9 and c['completed_duration_s']>=44-1e-9)
   assert test==e['checks'],(phase,name,test,e['checks']);assert all(test.values())==e['passes_all_conditions'];checks.extend([phase+'/'+name+'/all_screen_conditions',phase+'/'+name+'/screen_verdict'])
   comparisons[phase][name]={'checks':test,'passes_all_conditions':all(test.values()),'turn_changes':changes}
 policies={v:r['integrity']['passed']and all(comparisons[p][v+'-B']['passes_all_conditions']for p in PHASES)for v in('R','N','NR')}
 for v,result in policies.items():assert result==r['policy_screens'][v]['passes_all_three_phases'];checks.append(v+'/allthreephase_policy')
 for rel,h in hashes.items():assert sha(ROOT/rel)==h
 save(out/'report.json',{'status':'pass','assertion_count':len(checks),'checks':checks,'original_route_gates':gates,'comparisons':comparisons,'policy_verdicts':policies,'scope':'Independentarithmeticassertions, notnewrepo tests. Signed-lagguardunchanged; no outcome-basedcriterionchanges.'})
 save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
 print('ScreensQApassed',len(checks))
if __name__=='__main__':main()
