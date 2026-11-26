"""Consolidate sealed I51 late diagnostic reports; no new physical calculations."""
from pathlib import Path
from datetime import datetime,timezone
import json,hashlib,shutil
H=Path(__file__).resolve().parent
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 out=H/'compact_stage05';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/'build_compact.py')
 sources=[H/'stage02/report.json',H/'stage04_phase_frames/report.json',Path(__file__)]
 (out/'before_consolidation.json').write_text(json.dumps({'utc':datetime.now(timezone.utc).isoformat(),'input_sha256':{str(p.relative_to(H)):sha(p)for p in sources},'scope':'Report consolidation only; no new geometry, dynamics or controller evaluation.'},indent=2)+'\n')
 a=json.loads(sources[0].read_text());b=json.loads(sources[1].read_text());result={'scope':a['scope'],'active_settings':a['known_source_settings'],'conclusions':{'intentional_relaxation':'Not active: I49 least_squares has zero damping and no bounded/feasible allocator or projected motionreference. Exact XY and XYvelocity targets remainoriginal onall8801rows. Storedboundedreferenceparametersareinactive.','soft_feedback':'LQRhasweightedstateandinputcosts, nothardjointvelocityservos. Exactnominalkinematicallocationdoesnotguaranteeactualmotiontracking.','late_observed_motion':'Baseline38–42 drift occurs withnegligibleallocationresidual andsmallcontactslip, moderateconditioning, uncappeddrives. Continuedjointrotation andfreeheadingrotationprovidekinematiccontributions.','static_reference':'Estimatedplane staticforce algebra finds no grossrequestedup/COMbalanceerror afterdesiredaccelerationstops. This is notdynamicstabilityproof orvalidatedphysicalterrainmodel.','frames':'Savedheadinglinearvelocityerrors/bodyangularvelocityerrorsandnativegaininterpolationreproduce. No simplecurrentframecalculationbugidentified.','next_proposed_only':'Aftercurrentfourarmstudy, one matchedlate-windowestimatednormal-relativegain-phase ablation[38,42), preserveordinaryshaftrow/allotherreferences/observer/control; comparejointtracking,yaw/lean androuteerror. Priorglobalnormal-schedulingnegatives mustremainexplicit. No newrunauthorizedorperformedbythisaudit.'},'cases':{},'limitations':a['limitations']+b['limitations']}
 for c,d in a['cases'].items():result['cases'][c]={'unchanged_targets':d['unchanged_targets'],'windows':d['windows'],'snapshots':d['snapshots'],'phase_and_velocity_identity':b['cases'][c]}
 (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
 (out/'artifact_manifest.json').write_text(json.dumps({'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']},indent=2)+'\n')
if __name__=='__main__':main()
