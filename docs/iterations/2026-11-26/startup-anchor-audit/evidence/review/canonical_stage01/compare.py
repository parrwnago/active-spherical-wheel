"""Compare sealed independent I59 raw-log algebra with canonical pipeline data."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];BASE=ROOT/'results/startup_anchor_audit_20261126';CAN=BASE/'analysis/pipeline_stage01';OWN=BASE/'review/outcome_stage01';CASES=('m100_B','m100_P','p030_B','p030_P','p090_B','p090_P');MAP={'row_only':'gain_row_only','trim_only':'trim_only','combined':'combined_diagnostic'};ANCHORS=('m100_B','p030_B','p090_B')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):
 with np.load(p,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}
def save(p,x):Path(p).write_text(json.dumps(x,indent=2)+'\n')
paths=[Path(__file__),CAN/'science.json',CAN/'artifact_manifest.json',OWN/'report.json',BASE/'protocol/protocol.json']+[CAN/(c+'_derivatives.npz')for c in CASES]+[OWN/(c+'_independent_counterfactuals.npz')for c in CASES]+[CAN/'inputs'/(c+'.npz')for c in CASES]
hashes={str(p.relative_to(ROOT)):sha(p)for p in paths};save(HERE/'source_inputs_before_arithmetic.json',dict(registered_utc=datetime.now(timezone.utc).isoformat(),input_sha256=hashes,scope='Post-outcome numericalcrosscomparison; no fit, primarygatechange orphysics.'))
c=json.loads((CAN/'science.json').read_text());o=json.loads((OWN/'report.json').read_text());protocol=json.loads((BASE/'protocol/protocol.json').read_text());windows={**protocol['windows_s'],**protocol['event_windows_s']};ABS=np.array(protocol['absolute_material_thresholds']);NORM=protocol['normalized_material_threshold'];TOL=protocol['numerical_closure_max_abs'];maxima={};comparisons=0;summary_checks=0;classification_checks=0;fail=[]
def difference(key,a,b):
 global summary_checks
 a,b=np.asarray(a),np.asarray(b);value=float(np.max(abs(a-b),initial=0));maxima[key]=max(maxima.get(key,0),value);summary_checks+=1
 if value>TOL:fail.append(dict(kind=key,residual=value))
for pair in c['coefficient_pairs']:
 own=o['coefficient_pairs'][pair['actual']+'__'+pair['alternative']]
 for a,b in [('K0_byte_exact','row0_byte_equal'),('trim_byte_exact','trim_byte_equal')]:assert pair[a]==own[b]
 difference('coefficient_K0_max',pair['K0_max_abs_difference'],own['row0_max_abs_difference']);difference('coefficient_trim_max',pair['trim_max_abs_difference'],own['trim_max_abs_difference'])
for cid in CASES:
 a=load(OWN/(cid+'_independent_counterfactuals.npz'));b=load(CAN/(cid+'_derivatives.npz'));raw=load(CAN/'inputs'/(cid+'.npz'));t=a['time_s'];caps=raw['exact_input_bounds'];base=a['base_native_request'];applied=a['base_native_applied'];assert np.array_equal(t,b['time_s']);fade=np.column_stack((a['fade_point'],a['fade_low'],a['fade_high']));difference('fade_point_and_interval',fade,b['fade_estimate_lower_upper'])
 assert c['cases'][cid]['controls']['constant_K0']==o['within_trace_constancy'][cid]['row0_byte_constant'];assert c['cases'][cid]['controls']['constant_trim']==o['within_trace_constancy'][cid]['trim_byte_constant'];assert np.all(b['fade_identifiable'])
 for anchor in ANCHORS:
  for mode,canonical_mode in MAP.items():
   comparisons+=1;prefix=mode+'__'+anchor;d=a[prefix+'__requested_delta'];clipped_delta=a[prefix+'__clipped_delta'];difference('fullrate_analytical_request_delta',d,b[anchor+'_'+canonical_mode+'_delta_request']);difference('fullrate_native_clipped_delta',clipped_delta,b[anchor+'_'+canonical_mode+'_delta_native_clipped']);request=base+d;realized=request-base
   low=base+a[prefix+'__requested_delta_lower'];high=base+a[prefix+'__requested_delta_upper'];cliplo=applied+a[prefix+'__clipped_delta_lower'];cliphi=applied+a[prefix+'__clipped_delta_upper'];clipped=applied+clipped_delta
   for name,(lo,hi)in windows.items():
    select=(t>=lo-1e-9)&(t<=hi+1e-9);weight=np.clip(np.minimum(t[1:],hi)-np.maximum(t[:-1],lo),0,None);expected=c['cases'][cid]['alternatives'][anchor][canonical_mode][name];own=o['comparisons'][cid+'__'+mode+'__'+anchor]
    def duration(cond):return np.sum(cond[:-1]*weight[:,None],axis=0)
    # Match canonical reported raw differences after finite-precision addition;
    # analytical d itself is compared independently above and retained below.
    metrics=dict(raw_max_abs_by_input=np.max(abs(realized[select]),axis=0),clipped_max_abs_by_input=np.max(abs(clipped_delta[select]),axis=0),raw_RMS_by_input=np.sqrt(np.mean(realized[select]**2,axis=0)),clipped_RMS_by_input=np.sqrt(np.mean(clipped_delta[select]**2,axis=0)),raw_material_duration_by_input_s=duration(abs(realized)>ABS),clipped_material_duration_by_input_s=duration(abs(clipped_delta)>ABS),raw_normalized_material_duration_s=float(np.sum(weight*(np.max(abs(realized[:-1])/caps[:-1],axis=1)>NORM))),clipped_normalized_material_duration_s=float(np.sum(weight*(np.max(abs(clipped_delta[:-1])/caps[:-1],axis=1)>NORM))),baseline_native_outside_duration_by_input_s=duration(abs(base)>caps+1e-12),alternative_native_outside_duration_by_input_s=duration(abs(request)>caps+1e-12),masked_material_duration_by_input_s=duration((abs(realized)>ABS)&(abs(clipped_delta)<=ABS)),raw_interval_max_width_by_input=np.max((high-low)[select],axis=0),clipped_interval_max_width_by_input=np.max((cliphi-cliplo)[select],axis=0),worst_raw_abs_upper_bound_by_input=np.max(np.maximum(abs(low-base),abs(high-base))[select],axis=0),worst_clipped_abs_upper_bound_by_input=np.max(np.maximum(abs(cliplo-applied),abs(cliphi-applied))[select],axis=0))
    assert int(select.sum())==expected['samples'];difference('held_window_exposure',float(weight.sum()),expected['held_exposure_s'])
    for key,value in metrics.items():difference('summary_'+key,value,expected[key])
    # Independent classifications include interval guarantees; canonical supplies
    # point exposures plus upper bounds, all below both frozen thresholds.
    for boundary in ('native_requested','native_clipped'):
     s=own[boundary][name];assert s['point_normalized_material_states']==s['possible_normalized_material_states']==0;assert not any(s['point_absolute_material_states_by_input']);assert not any(s['possible_absolute_material_states_by_input']);classification_checks+=4
    difference('analytical_vs_rounded_raw_max',np.max(abs(d[select]),axis=0),expected['raw_max_abs_by_input'])
assert not fail,fail
assert hashes=={str(p.relative_to(ROOT)):sha(p)for p in paths}
result=dict(completed_utc=datetime.now(timezone.utc).isoformat(),all_agree_within_closure_tolerance=True,command_combinations=comparisons,fixed_windows_per_combination=len(windows),numeric_summary_checks=summary_checks,classification_assertions=classification_checks,residual_maxima=maxima,failures=fail,physics_calls=0,
 scope='54correlatedstate-interventioncombinations×6overlappingwindows, notphysicaltrialsorindependenttests. Canonicalrawsummaries subtract after finiteaddition; independentanalyticdeltas retained separately. Boundary>versus>=doesnotaffectany observedclassification: allordersbelowthreshold.')
save(HERE/'report.json',result);save(HERE/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(HERE)),sha256=sha(p),bytes=p.stat().st_size)for p in sorted(HERE.rglob('*'))if p.is_file()and p!=HERE/'artifact_manifest.json']))
print(json.dumps(result,indent=2))
