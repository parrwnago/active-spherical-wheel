"""Independent lossless input decode and all-field I-61 packet recomposition."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import numpy as np


def load(path):
    with np.load(path,allow_pickle=False)as z:return {k:z[k].copy()for k in z.files}


def same(a,b):return a.shape==b.shape and a.dtype==b.dtype and a.tobytes()==b.tobytes()


def decode(encoded,recipes):
    out={}
    for name,r in recipes.items():
        kind=r['encoding']
        if kind=='plain':value=encoded[name].copy()
        elif kind=='alias':value=out[r['reference']].copy()
        elif kind=='first_six':value=out[r['reference']][:,:6].copy()
        elif kind=='constant':value=np.full(r['shape'],encoded[name].reshape(-1)[0],dtype=np.dtype(r['dtype']))
        elif kind=='xor64':
            reference=np.ascontiguousarray(out[r['reference']]).view(np.uint64)
            value=(reference^encoded[name]).view(np.dtype(r['dtype'])).reshape(r['shape']).copy()
        else:raise ValueError(kind)
        assert list(value.shape)==r['shape']and value.dtype.str==r['dtype']
        assert hashlib.sha256(value.tobytes()).hexdigest()==r['array_sha256']
        out[name]=value
    return out


def derive(inputs,options):
    out={key:value.copy()for key,value in inputs.items()}
    for prefix in ('proposed','native_alternative'):
        valid_input=out[prefix+'__valid']
        residual=out['observer_generalized']-out[prefix+'__prediction']
        # This branch corresponds to the original generator's invalid feature
        # result. It cannot alter a finite valid-row arithmetic result.
        residual[~valid_input]=np.nan
        extra=residual[:,6:9].copy();force=out[prefix+'__force_norm_N'];radius=out[prefix+'__radius_m']
        valid=valid_input&np.isfinite(residual).all(axis=1)&np.isfinite(force)&np.isfinite(radius)&(force>=0)&(radius>0)
        threshold=options['absolute_tolerance_Nm']+options['relative_tolerance']*radius*force
        normalized=extra/threshold[:,None];maximum=np.max(abs(normalized),axis=1)
        # Independent componentwise decision, not a copied returned gate mask.
        passes=valid&np.all(abs(extra)<=threshold[:,None],axis=1)
        out[prefix+'__residual']=residual;out[prefix+'__extra_joint_residual_Nm']=extra.copy()
        gate=dict(valid=valid,threshold_Nm=threshold,extra_joint_residual_Nm=extra,
            normalized_joint_residual=normalized,max_normalized_joint_residual=maximum,
            passes=passes,vetoed=valid&~passes,unknown=~valid)
        for key,value in gate.items():out[prefix+'_gate__'+key]=value
    out['native_relative_max_ratio_difference']=out['proposed_gate__max_normalized_joint_residual']-out['native_alternative_gate__max_normalized_joint_residual']
    accepted=out['i60_accepted'];gated=accepted&out['proposed_gate__passes']
    out['N_family']=out['native_family'].copy();out['N_point_m']=out['native_point_m'].copy()
    for name,mask in [('A',accepted),('C',gated)]:
        out[name+'_family']=np.where(mask,out['proposed_family'],out['native_family'])
        out[name+'_point_m']=np.where(mask[:,None],out['proposed_point_m'],out['native_point_m'])
        out[name+'_uses_proposal']=mask.copy()
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--compact-root',type=Path,required=True);p.add_argument('--original-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    cases={}
    for case in ('m100_B','p030_B','p090_B'):
        folder=a.compact_root/case;metadata=json.loads((folder/'metadata.json').read_text())
        original=load(a.original_root/case/'load_features_and_policies.npz')
        encoded=load(folder/'inputs.npz');inputs=decode(encoded,metadata['input_recipes'])
        assert all(same(value,original[key])for key,value in inputs.items())
        packet=derive(inputs,metadata['gate_options']);assert set(packet)==set(original)
        checked={key:same(value,original[key])for key,value in packet.items()}
        assert all(checked.values()),[key for key,ok in checked.items()if not ok]
        cases[case]=dict(rows=len(packet['time_s']),decoded_input_fields=len(inputs),derived_fields=len(packet)-len(inputs),
                         total_packet_fields=len(packet),all_fields_byte_exact=True,
                         decoded_inputs_byte_exact_against_original=True,
                         recomputed_gate_and_policy_booleans_unpatched=True,
                         XOR_input_fields=[key for key,r in metadata['input_recipes'].items()if r['encoding']=='xor64'])
    result=dict(iteration='I-61',completed_utc=datetime.now(timezone.utc).isoformat(),cases=cases,
                all_checks_pass=True,new_physics_runs=0,
                scope='No production packer/replayer/core imports. Independently decode retained geometry/measurement inputs and compute residuals, component vetoes and policies; all 59 fields match original bytes.',
                XOR_qualification='XOR covers finite values as LOSSLESS INPUT COMPRESSION. It is not an output correction. No derived numeric/boolean field is supplied as a replacement patch.',
                NaN_qualification='Only original-invalid residual rows are explicitly set to NaN, matching the feature contract; finite valid-row results and boolean decisions are untouched.',
                geometry_scope='Jacobian predictions/validity/maps remain preserved inputs, not regenerated by this NumPy replay. Independent analytic all-row geometry verification is a separate report.')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
