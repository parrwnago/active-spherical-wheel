"""Declared early I55 native/zero-A reduction while final arm is still running."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,json,xml.etree.ElementTree as ET
import numpy as np
import analyze_causal as ac
import budget_math as bm

H=Path(__file__).resolve().parent;FAMILY=H.parent;ROOT=FAMILY.parents[1]
CASES=('native_batch','native_substeps','zero_free_shellA')
def main(out):
    out.mkdir(exist_ok=False)
    proof=FAMILY/'review/budget_source_review_stage01/report.json'
    paths=[Path(__file__),H/'analyze_causal.py',H/'budget_math.py',FAMILY/'protocol.json',proof,H/'checks_stage02_corrected/report.json',ac.OLD/'precision.npz']
    if(FAMILY/'protocol_clarifications.json').exists():paths.append(FAMILY/'protocol_clarifications.json')
    parents={};prov={}
    for c in CASES:
        f=FAMILY/'campaign'/c;parents[c]=f
        for name in ['i55_provenance.json','i55_sidecar.npz','i55_artifact_manifest.json','campaign/runs/m100_R/precision.npz']:paths.append(f/name)
        assert not(f/'i55_failure.json').exists()
        prov[c]=json.loads((f/'i55_provenance.json').read_text());assert prov[c]['success']
        if c!='native_batch':paths.append(f/'substeps.npz')
        for k in ['mirror_xml','physical_sdf']:paths.append(Path(prov[c]['models'][k]))
        xml=Path(prov[c]['models']['mirror_xml'])
        for node in ET.parse(xml).getroot().findall('./asset/mesh'):paths.append((xml.parent/node.attrib['file']).resolve())
    paths=list(dict.fromkeys(paths));hashes={str(p.relative_to(ROOT)):ac.sha(p)for p in paths};ac.copy_source(out,[p for p in paths if p.suffix=='.py']+[FAMILY/'protocol.json',proof])
    ac.save(out/'registered_before_reduction.json',{'utc':datetime.now(timezone.utc).isoformat(),'scope':'First saved-data scientific reduction of three completed registered arms; hold_attitude not yet available and not omitted from final study. No gate/threshold changes.','cases':CASES,'source_and_input_sha256':hashes,'primary_center':'midpoint systemCOM; prior/post sensitivity, same individual contact points','fixed_intervals_s':ac.INTERVALS,'first_1ms_5ms_endpoints_s':ac.SUBSTEP_ENDPOINTS,'force_semantics':'Captured versioned DART/gz-physics source supports force=solver impulse/dt; reconstruction F*dt remains conditional on pass-through sign/order/completeness. Never flip sign after outcome.','linear_convention_QA':'M DeltaVcom versus dt*(sum ALL raw forces +Mg+knownpush), no posthoc sign selection.','disclosure':'Root preliminary state-difference numbers were received after endpoint/gate definitions were set but before this numerical reduction. No new NPZ outcomes were read by this reducer before registration.'})
    p={c:ac.load(parents[c]/'campaign/runs/m100_R/precision.npz')for c in CASES};s={c:ac.load(parents[c]/'i55_sidecar.npz')for c in CASES};old=ac.load(ac.OLD/'precision.npz')
    report={'scope':'Partial early reduction; final attitude arm pending','gates':{'native_historical':ac.prefix_compare(p['native_batch'],old),'native_stepping_equivalence':ac.prefix_compare(p['native_substeps'],p['native_batch'])},'cases':{}}
    traces={}
    for c in CASES:
        row={'selective_scope':ac.selected_scope(c,p[c],s[c])}
        if c!='native_batch':
            z=ac.load(parents[c]/'substeps.npz');traces[c]=z;row['clock']=ac.clock_gate(p[c],z);models=prov[c]['models'];mb=bm.momenta(models['mirror_xml'],models['physical_sdf'],z['bridge_state_before']);ma=bm.momenta(models['mirror_xml'],models['physical_sdf'],z['bridge_state']);row['intervals'],row['coverage']=ac.budgets(z,mb,ma);row['substep_endpoints']=ac.substep_endpoints(p[c],s[c],z,mb,ma);row['endpoints']={str(t):ac.endpoint(p[c],s[c],t,s['native_substeps'])for t in ac.ENDPOINTS};row['inertia_checks']=ma['inertia_checks'];row['chain_omega_max_gap']=max(mb['max_chain_omega_gap_rad_s'],ma['max_chain_omega_gap_rad_s'])
            np.savez_compressed(out/(c+'_momenta.npz'),time_s=z['time_s'],H_before=mb['H_by_body_world'],H_after=ma['H_by_body_world'],COM_before=mb['systemCOM_world_m'],COM_after=ma['systemCOM_world_m'])
        report['cases'][c]=row
    z=traces['zero_free_shellA'];n=traces['native_substeps'];m=(z['time_s']>32.25+1e-9)&(z['time_s']<=32.255+1e-9);notA=[0,2,3,4]
    report['gates']['first_hold_non_A_inputs_exact']=ac.exact(z['held_input'][m][:,notA],n['held_input'][m][:,notA])
    report['gates']['zeroA_prefix']=ac.prefix_compare(p['zero_free_shellA'],p['native_substeps'],p['zero_free_shellA']['time_s']<32.25-1e-9)
    hm=s['zero_free_shellA']['time_s']<=32.25+1e-9;report['gates']['history_through_t0_exact']=ac.exact(s['zero_free_shellA']['history_before_sha256'][hm],s['native_substeps']['history_before_sha256'][hm])
    report['force_source_proof_sha256']=ac.sha(proof)
    ac.save(out/'report.json',report)
    for rel,h in hashes.items():assert ac.sha(ROOT/rel)==h,rel
    ac.save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':ac.sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
    print('Early three-arm report sealed',out/'report.json')
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);a=parser.parse_args();main(a.output.resolve())
