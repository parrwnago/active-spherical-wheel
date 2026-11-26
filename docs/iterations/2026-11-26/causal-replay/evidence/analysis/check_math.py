"""Pure boundary checks plus parity with already-sealed I54 static accounting."""
from pathlib import Path
import hashlib,json,shutil
import numpy as np
import budget_math as bm
import analyze_causal as ac
H=Path(__file__).resolve().parent;ROOT=H.parents[2]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    out=H/'checks_stage02_corrected';out.mkdir(exist_ok=False)
    reference=ROOT/'results/acceleration_spin_cause_20261126/mechanics_audit/stage02_corrected/report.json'
    prior=json.loads(reference.read_text());xml=ROOT/prior['input_model'];sdf=ROOT/prior['physical_model']
    paths=[Path(__file__),H/'budget_math.py',H/'analyze_causal.py',reference,xml,sdf,H.parent/'protocol.json']
    for p in paths:
        dest=out/'source_snapshot'/p.relative_to(ROOT);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(p,dest)
    ac.save(out/'before_checks.json',{'source_and_input_sha256':{str(p.relative_to(ROOT)):sha(p)for p in paths},'scope':'No new I55 outcome arrays. Synthetic frame/force/transport/prefix/closure fixtures plus known I54 static momentum parity.'})
    checks=[]
    q=np.array([[np.sqrt(.5),0,0,np.sqrt(.5)]]);states=np.zeros((1,24));states[:,5:9]=q;states[:,12]=1
    _,v=bm.state_qv(states);assert np.allclose(v[0,3:6],[0,-1,0],atol=1e-15);checks.append('world-to-body free-joint angular velocity convention')
    c=np.zeros((2,16));c[0,[0,1,5,8,12]]=[0,1,1,2,0];c[1,[0,2,4,7,12]]=[6,1,1,3,1]
    z=bm.contact_moments(c,[0,2],[[0,0,0]]);assert np.allclose(z['torque_by_body_world_Nm'][0,:,2],[-3,0,2,0]);assert z['rejected_rows_count']==1;checks.append('all forces retained incl rejected shell and body; correct body-tag mapping')
    assert np.array_equal(bm.TAG_BODY,[2,3,1,1,2,3,0]);checks.append('smooth/rim/belt/body topology mapping')
    for center,wanted in [([0,0,0],-1),([.1,.2,0],-.6),([.2,.4,0],-.2)]:assert abs(bm.contact_moments(c,[0,2],[center])['torque_by_body_world_Nm'][0,:,2].sum()-wanted)<1e-14
    checks.append('fixed contact point prior/mid/post center shift identity')
    masses=np.array([1.,1.]);r=np.array([[1.,0,0],[-1.,0,0]]);dv=np.array([0,.3,0]);transport=-masses[:,None]*np.cross(r,dv);assert np.allclose(transport[:,2],[-.3,.3])and np.allclose(transport.sum(axis=0),0);checks.append('moving-COM transport sign and whole-system cancellation')
    assert bm.closure(.001,.0011)['passes_desired_closure'];assert not bm.closure(.001,.0011001)['passes_desired_closure'];checks.append('registered yaw closure tolerance boundary')
    a={'time_s':np.arange(3.),'x':np.array([1.,np.nan,3.]),'label':np.array(['a','b','c'])};b={'time_s':np.arange(5.),'x':np.array([1.,np.nan,3.,4.,5.]),'label':np.array(['a','b','c','d','e'])};assert ac.prefix_compare(a,b)['all_shared_fields_exact'];checks.append('numeric NaN/string shared-prefix exactness')
    states=[]
    for row in prior['snapshots']:
        q=np.asarray(row['qpos']);v=np.asarray(row['qvel']);s=np.zeros(24);s[2:5]=q[:3];s[5:9]=q[3:7];s[9:12]=v[:3];s[12:15]=bm.rotation(q[3:7])@v[3:6];s[15:18]=q[7:10];s[18:21]=v[6:9];states.append(s)
    m=bm.momenta(xml,sdf,np.array(states))
    expected=np.array([[r['H_about_systemCOM_world_kg_m2_s'][name]for name in bm.NAMES]for r in prior['snapshots']]);gap=np.max(abs(m['H_by_body_world']-expected));assert gap<1e-12;checks.append('all six known I54 per-body H snapshots match after raw world-state conversion')
    assert np.max(abs(np.einsum('j,njk->nk',m['body_masses_kg'],m['bodyCOM_world_m'])/m['mass_kg']-m['systemCOM_world_m']))<1e-14;checks.append('mass-weighted system COM identity')
    ac.save(out/'report.json',{'passes':True,'checks':checks,'known_I54_H_max_gap_kg_m2_s':gap,'new_I55_outcomes_read':False})
    ac.save(out/'artifact_manifest.json',{'files':[{'path':str(p.relative_to(out)),'bytes':p.stat().st_size,'sha256':sha(p)}for p in sorted(out.rglob('*'))if p.is_file()and p.name!='artifact_manifest.json']})
    print(json.dumps({'passes':True,'checks':len(checks),'I54_H_max_gap':float(gap)}))
if __name__=='__main__':main()
