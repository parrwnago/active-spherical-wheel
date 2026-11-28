"""I78 independent all-closed-case command-ledger and verification closure."""
from pathlib import Path
from datetime import datetime,timezone
import argparse,hashlib,json,shutil
import numpy as np
ROOT=Path(__file__).resolve().parents[4]

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,x):
    with p.open('x')as f:json.dump(x,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
def load(p):
    with np.load(p,allow_pickle=False)as z:return{k:z[k].copy()for k in z.files}
def compare(a,b,name):
    if not np.all(np.isfinite(a))or not np.all(np.isfinite(b)):
        raise AssertionError('Nonfinite used command/reference term: '+name)
    if not np.allclose(a,b,rtol=1e-12,atol=2e-12):raise AssertionError('Command ledger differs: '+name)
    return float(np.max(np.abs(np.asarray(a)-np.asarray(b))))

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--protocol',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    protocol_path=a.protocol.resolve();base=protocol_path.parent.parent
    protocol=json.loads(protocol_path.read_text());jobs=protocol['cases']
    if len(jobs)!=6:raise AssertionError('All six fixed native/governed cases required')
    inputs=[protocol_path,Path(__file__).resolve()];cases=[]
    for job in jobs:
        cid=job['case_id'];folder=base/'campaign/runs'/cid;parent_id=cid[:-1]+'N'
        manifest_path=folder/'artifact_manifest.json';manifest=json.loads(manifest_path.read_text())['files']
        precision=folder/'child/campaign/runs'/parent_id/'precision.npz'
        sidecar=folder/'governor_diagnostics.npz';outcome=folder/'i78_outcome.json'
        proof=base/'governor/verification'/f'{cid}_stage01/report.json'
        if not json.loads(proof.read_text())['passed']:raise AssertionError('Independent case proof is not passing')
        for p in (precision,sidecar,outcome):
            bound=manifest[str(p.relative_to(folder))]
            if sha(p)!=bound['sha256']or p.stat().st_size!=bound['bytes']:
                raise AssertionError('Closed case changed: '+str(p))
        cases.append((job,precision,sidecar,proof));inputs.extend([precision,sidecar,outcome,proof,manifest_path])
    out=a.output.resolve();out.mkdir(parents=True,exist_ok=False);shutil.copy2(Path(__file__),out/Path(__file__).name)
    bindings={str(p.relative_to(ROOT)):dict(bytes=p.stat().st_size,sha256=sha(p))for p in inputs}
    write(out/'registered_before_array_reads.json',dict(iteration='I-78',utc=datetime.now(timezone.utc).isoformat(),
        files=bindings,all_six_cases_closed=True,method='Independent all-case saved-vector algebra, raw reference equality and existing independent verification closure. No controller/model/observer calls or physics.',
        metric_scope='Integrity and bounded actual-output checks; not acceptance or hard future feasibility.'))
    rows={}
    for job,precision,sidecar,proof in cases:
        p,s=load(precision),load(sidecar);n=len(p['time_s'])
        lqr=-np.einsum('nij,nj->ni',p['exact_feedback_gain'],p['exact_error'])
        request=(p['exact_equilibrium_command']+p['exact_rolling_feedforward']+p['exact_support_command']+
                 p['exact_lqr_command']+p['exact_transition_command_delta']+p['exact_heading_command']+p['exact_fade_command_delta'])
        applied=np.clip(p['exact_requested'],-p['exact_input_bounds'],p['exact_input_bounds'])
        residuals=dict(lqr=compare(lqr,p['exact_lqr_command'],'LQR'),
            request=compare(request,p['exact_requested'],'full requested input'),
            applied=compare(applied,p['exact_applied'],'clipped actual input'),
            clip_delta=compare(p['exact_applied']-p['exact_requested'],p['exact_clipping_command_delta'],'clipping delta'),
            derived_shaft=compare(p['exact_error'][:,0]/.12-p['exact_error'][:,4],p['exact_error'][:,6],'derived shaft error'))
        np.testing.assert_array_equal(p['exact_reference_qpos'][:,:2],p['exact_reference_xy'])
        np.testing.assert_array_equal(p['exact_feedback_reference_xy'],p['exact_reference_xy'])
        np.testing.assert_array_equal(p['exact_reference_qvel'][:,:2],p['exact_feedback_reference_qvel'][:,:2])
        if np.any(p['exact_feedback_reference_qvel'][:,3:6]):raise AssertionError('New angular-rate reference was introduced')
        if np.any(p['exact_error'][:,7:9]):raise AssertionError('Shell material-position feedback changed')
        if np.any(p['exact_heading_command']):raise AssertionError('Heading helper was enabled')
        held=np.arange(n)<n-1
        np.testing.assert_array_equal(p['command_held_after_row'],held)
        np.testing.assert_array_equal(s['command_held_after_row'],held)
        np.testing.assert_array_equal(p['exact_previous_applied'][1:],p['exact_applied'][:-1])
        proof_report=json.loads(proof.read_text())
        rows[job['case_id']]=dict(rows=n,independent_case_proof_passed=True,
            actual_input_box_passed=True,max_ledger_residuals=residuals,
            original_prefix_rows=proof_report['complete_original_prefix_rows'],
            first_changed_reference_row=proof_report['first_changed_reference_row'],
            reference_xy_unprojected=True,body_angular_rate_reference_remains_zero=True,
            input_timing_and_terminal_masks_verified=True)
    for rel,b in bindings.items():
        p=ROOT/rel
        if sha(p)!=b['sha256']or p.stat().st_size!=b['bytes']:raise AssertionError('Bound input/source changed')
    write(out/'report.json',dict(iteration='I-78',passed=True,cases=rows,
        verified_case_count=len(rows),verified_control_rows=sum(v['rows']for v in rows.values()),
        new_physics=False,production_edits=False,source_inputs_unchanged=True,
        scope='Six independent per-case clock/reference/native-prefix proofs plus native command ledgers. Correlated rows are not independent experiments or repository-suite tests. Acceptance is evaluated separately.'))
    write(out/'artifact_manifest.json',dict(files={p.name:dict(bytes=p.stat().st_size,sha256=sha(p))for p in sorted(out.iterdir())if p.is_file()}))
    print(json.dumps(dict(passed=True,cases=len(rows),rows=sum(v['rows']for v in rows.values()))),flush=True)

if __name__=='__main__':main()
