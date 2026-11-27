"""Project frozen I-61 arrays for the registered I-62 sweep; no new geometry."""
from datetime import datetime,timezone
from pathlib import Path
import argparse,hashlib,json,shutil
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
CASES=('m100_B','p030_B','p090_B')


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,x):Path(p).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def load(p):
    with np.load(p,allow_pickle=False) as z:return {k:z[k].copy() for k in z.files}


def run(protocol_path,source_root,output):
    protocol=json.loads(protocol_path.read_text())
    if protocol['iteration']!='I-62':raise ValueError('Expected I-62')
    seal_path=protocol_path.parent/'input_source_seal.json';seal=json.loads(seal_path.read_text())
    for item in seal['files']:
        p=Path(item['path']);p=p if p.is_absolute() else ROOT/p
        if sha(p)!=item['sha256']:raise ValueError('Changed root-bound original: '+str(p))
    output.mkdir(parents=True,exist_ok=False)
    save(output/'registered_before_projection.json',dict(iteration='I-62',registered_utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(Path(__file__)),protocol_sha256=sha(protocol_path),input_seal_sha256=sha(seal_path),numpy_version=np.__version__,
        prior='All original I-61 data/results were already examined; no new threshold-sweep outcome inspected before this source/input registration.'))
    shutil.copy2(Path(__file__),output/'prepare_inputs.py');shutil.copy2(protocol_path,output/'protocol.json')
    bindings=[];counts={}
    for case in CASES:
        feature_path=source_root/'campaign/features_stage01'/case/'load_features_and_policies.npz'
        eval_path=source_root/'analysis/stage01'/(case+'_evaluation.npz')
        feature=load(feature_path);evaluation=load(eval_path)
        if not np.array_equal(feature['time_s'],evaluation['time_s']):raise ValueError('Source times differ')
        t=feature['time_s']
        if len(t)!=8801 or not np.all(abs(np.diff(t)-.005)<1e-9):raise ValueError('Not the original full-rate history')
        primary=evaluation['primary_mask']&evaluation['truth_eligible']
        common=primary&np.isfinite(feature['N_point_m']).all(axis=1)&np.isfinite(feature['A_point_m']).all(axis=1)
        common &= np.isfinite(evaluation['truth_equivalent_point_m']).all(axis=1)
        target=evaluation['truth_equivalent_point_m']
        pack=dict(time_s=t.copy(),gate_valid=feature['proposed_gate__valid'].copy(),
            joint_max_abs_Nm=np.max(abs(feature['proposed__residual'][:,6:9]),axis=1),
            radius_m=feature['proposed__radius_m'].copy(),force_norm_N=feature['proposed__force_norm_N'].copy(),
            i60_accepted=feature['i60_accepted'].copy(),N_family=feature['N_family'].copy(),A_family=feature['A_family'].copy(),
            truth_family=evaluation['truth_family'].copy(),primary_eligible=primary,point_mask=common,
            native_distance_m=np.linalg.norm(feature['N_point_m']-target,axis=1),
            proposed_distance_m=np.linalg.norm(feature['proposed_point_m']-target,axis=1),
            oracle_valid=evaluation['oracle_geometry_valid'].copy(),
            oracle_joint_max_abs_Nm=np.max(abs(evaluation['oracle_joint_residual_Nm']),axis=1),
            oracle_mask=primary&evaluation['stable_truth_mask'],
            i61_keep_proposal=feature['C_uses_proposal'].copy(),
            i61_C_family=feature['C_family'].copy(),
            i61_distance_m=np.linalg.norm(feature['C_point_m']-target,axis=1))
        np.savez_compressed(output/(case+'.npz'),**pack)
        counts[case]=dict(rows=len(t),primary_eligible=int(primary.sum()),point_matched=int(common.sum()))
        bindings.append(dict(case=case,feature_path=str(feature_path),feature_sha256=sha(feature_path),
            evaluation_path=str(eval_path),evaluation_sha256=sha(eval_path),
            fields={k:dict(dtype=v.dtype.str,shape=list(v.shape),sha256=hashlib.sha256(v.tobytes()).hexdigest())for k,v in pack.items()}))
    save(output/'projection_bindings.json',dict(iteration='I-62',cases=bindings,counts=counts,
        qualification='Lossless full-rate inputs for frozen shared-bound policies and retrospective metrics. Distances are scalar cached Euclidean errors, not regenerated geometry. Point mask includes finite native/A fallback/target, independent of tested parameters.'))
    for item in seal['files']:
        p=Path(item['path']);p=p if p.is_absolute() else ROOT/p
        if sha(p)!=item['sha256']:raise ValueError('Original changed during projection')
    save(output/'artifact_manifest.json',dict(files=[dict(path=str(p.relative_to(output)),sha256=sha(p),bytes=p.stat().st_size)
        for p in sorted(output.rglob('*'))if p.is_file()and '__pycache__'not in p.parts and p.name!='artifact_manifest.json']))
    print(json.dumps(dict(cases=3,counts=counts,bytes=sum(p.stat().st_size for p in output.rglob('*')if p.is_file()))))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--protocol',type=Path,required=True)
    p.add_argument('--source-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    run(a.protocol.resolve(),a.source_root.resolve(),a.output.resolve())
