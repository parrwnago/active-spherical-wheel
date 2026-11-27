"""Independently check retained minima against full original feature arrays."""
from pathlib import Path
import argparse
import json
import numpy as np

MODES=('geometry','wrench','velocity','fused')


def load(path):
    with np.load(path,allow_pickle=False)as data:return {key:data[key].copy()for key in data.files}


def same(a,b):
    a,b=np.asarray(a),np.asarray(b)
    return a.dtype==b.dtype and a.shape==b.shape and a.tobytes()==b.tobytes()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--original-root',type=Path,required=True)
    p.add_argument('--compact-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    cases={}
    for case in ('m100_B','p030_B','p090_B'):
        full=load(a.original_root/case/'projected_features.npz')
        compact=load(a.compact_root/case/'projected_features.npz')
        raw_scores=load(a.original_root/case/'scores.npz')
        saved_scores=load(a.compact_root/case/'scores.npz')
        context=load(a.original_root/case/'measurement_context.npz')
        small_context=load(a.compact_root/case/'measurement_context.npz')
        ids=compact['source_candidate_index'];keep=ids>=0;row,slot=np.nonzero(keep)
        assert np.array_equal(keep,compact['valid'])
        assert np.array_equal(compact['family_index'][slot],full['family_index'][ids[row,slot]])
        assert full['valid'][row,ids[row,slot]].all()
        fields={}
        for key in ('normal_angle_rad','velocity_residual_norm_m_s','moment_residual_norm','velocity_residual_m_s'):
            fields[key]=same(compact[key][row,slot],full[key][row,ids[row,slot]])
            assert fields[key]
        for key in ('frame_valid','excitation_m_s'):
            assert same(full[key],compact[key])
        for mode in MODES:
            minima=raw_scores[mode+'__candidate_index']
            for family in range(4):
                present=(ids==minima[:,family,None]).any(axis=1)
                assert present[minima[:,family]>=0].all()
        assert set(raw_scores)==set(saved_scores)
        assert all(same(raw_scores[key],saved_scores[key])for key in raw_scores)
        assert all(same(value,context[key])for key,value in small_context.items())
        cases[case]=dict(rows=len(ids),retained_fields_byte_exact=fields,
                         all_four_methods_family_minima_retained=True,
                         family_and_original_candidate_identity_exact=True,
                         stored_original_score_fields_byte_exact=len(raw_scores),
                         context_fields_byte_exact=list(small_context))
    result=dict(iteration='I-60',cases=cases,all_checks_pass=True,new_physics_runs=0,
                scope='Independent array-level source-ID/field/minimum preservation. No packer/ranker import; fixed weights only, not a new candidate generator or truth evaluation.')
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))


if __name__=='__main__':main()
