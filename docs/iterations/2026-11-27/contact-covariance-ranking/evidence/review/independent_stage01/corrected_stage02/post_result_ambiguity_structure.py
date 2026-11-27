from pathlib import Path
import json
import numpy as np
CASES=('m100_B','p030_B','p090_B')
previous=Path('docs/iterations/2026-11-27/contact-load-history/evidence')
canonical=Path('results/contact_covariance_rank_20261127/analysis/stage01')
out={}
for case in CASES:
 with np.load(previous/'portable_inputs/i62_base'/(case+'.npz'),allow_pickle=False) as z:b={k:z[k].copy() for k in z.files}
 with np.load(previous/'features/portable_stage01'/case/'primitives.npz',allow_pickle=False) as z:p={k:z[k].copy() for k in z.files}
 with np.load(canonical/(case+'_oof.npz'),allow_pickle=False) as z:a={k:z[k].copy() for k in z.files}
 selected=b['primary_eligible']&b['i60_accepted'];amb=selected&(a['M1__shared_covariance_code']==2)
 same_family=p['proposed_family']==p['native_family'];same_index=p['proposed_candidate_index']==p['native_candidate_index']
 same_H=np.all(p['H_F']==p['H_Nalt'],axis=(1,2));diff=a['M1__e_F']-a['M1__e_Nalt']
 neither=(b['N_family']!=b['truth_family'])&(b['A_family']!=b['truth_family'])
 out[case]=dict(accepted=int(selected.sum()),ambiguous=int(amb.sum()),same_family=int(np.sum(selected&same_family)),
  ambiguous_not_same_family=int(np.sum(amb&~same_family)),same_family_not_ambiguous=int(np.sum(selected&same_family&~amb)),
  ambiguous_same_candidate_index=int(np.sum(amb&same_index)),ambiguous_H_byte_values_equal=int(np.sum(amb&same_H)),
  maximum_ambiguous_e_difference=float(np.max(abs(diff[amb]))),neither_family_correct=int(np.sum(selected&neither)),
  neither_family_correct_and_ambiguous=int(np.sum(amb&neither)))
print(json.dumps(out,indent=2))
