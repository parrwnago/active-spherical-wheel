"""Explicit post-primary descriptive release check motivated by the fixed plot."""
from pathlib import Path
from datetime import datetime,timezone
import json,shutil,numpy as np
import analyze_causal as ac
import budget_math as bm
H=Path(__file__).resolve().parent;F=H.parent;ROOT=F.parents[1]
def main():
    out=H/'release_followup_stage01';out.mkdir(exist_ok=False);shutil.copyfile(__file__,out/Path(__file__).name)
    paths=[Path(__file__),H/'stage01/report.json',H/'presentation_stage02_three_targets/attitude_causal_response.png']
    for c in ['native_substeps','hold_attitude']:paths.extend([F/'campaign'/c/'substeps.npz',F/'campaign'/c/'i55_sidecar.npz'])
    hashes={str(p.relative_to(ROOT)):ac.sha(p)for p in paths}
    ac.save(out/'declared_before_readout.json',{'utc':datetime.now(timezone.utc).isoformat(),'source_input_sha256':hashes,'scope':'SECONDARY DESCRIPTIVE ONLY, motivated by already-seen fixed-window plot. Does not modify registered primary/gates/canonicalreport. Protocol requires releaseeffectsseparately.','rule':'For both native_substeps and hold_attitude, report maxabs world-z rate over ALLpostrelease31.745–32.500, plus first20ms individualstates. No favorable alternatewindow orcaserun.'})
    report={}
    for c in ['native_substeps','hold_attitude']:
        z=ac.load(F/'campaign'/c/'substeps.npz');side=ac.load(F/'campaign'/c/'i55_sidecar.npz');ids=np.flatnonzero((z['time_s']>31.745+1e-9)&(z['time_s']<=32.5+1e-9));i=int(ids[np.argmax(abs(z['bridge_state'][ids,14]))]);j=ac.index(side['time_s'],31.745);first=np.flatnonzero((z['time_s']>31.745+1e-9)&(z['time_s']<=31.765+1e-9))
        report[c]={'postrelease_max_abs_world_z_rate_rad_s':abs(z['bridge_state'][i,14]),'peak_time_s':z['time_s'][i],'first_released_command':side['final_applied'][j],'first_released_request':side['final_requested'][j],'first20ms_rows':[{'time_s':z['time_s'][k],'world_z_rate_rad_s':z['bridge_state'][k,14],'held_input':z['held_input'][k]}for k in first]}
    ac.save(out/'report.json',report)
    for rel,h in hashes.items():assert ac.sha(ROOT/rel)==h
    ac.save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':ac.sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
    print(json.dumps({c:{k:v for k,v in d.items()if k!='first20ms_rows'}for c,d in report.items()}))
if __name__=='__main__':main()
