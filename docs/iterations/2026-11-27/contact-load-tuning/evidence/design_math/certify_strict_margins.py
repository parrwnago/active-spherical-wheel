"""Post-release exact-sign check of the independently reported bound.

Written after audit_bounds.json reported 99 forced useful rows. It adds no
selection criterion, changes no frozen evaluator, and is not preregistered.
Exact Fraction signs use the stored binary64 force/radius/residual values.
"""
from fractions import Fraction
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
spec = importlib.util.spec_from_file_location('bound_source', HERE/'audit_bounds.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)
FEATURE = ROOT/'results/contact_load_consistency_20261127/campaign/features_stage01'
EVAL = ROOT/'results/contact_load_consistency_20261127/analysis/stage01'
parts = [audit.load_case(EVAL/(case+'_evaluation.npz'),
    FEATURE/case/'load_features_and_policies.npz', phase)
    for phase,case in enumerate(audit.CASES)]
x = {k: np.concatenate([p[k] for p in parts]) for k in parts[0]}
M = float(np.max(x['dN'][x['point_mask']]))
unsafe = x['accepted'] & x['point_mask'] & (x['dA'] > M+audit.EPS_MM) & x['valid']
F = lambda v: Fraction.from_float(float(v))
rows = []
for i in np.flatnonzero(x['useful'] & x['valid']):
    mi = F(x['m'][i]); Li = F(x['radius'][i])*F(x['force'][i])
    for k in np.flatnonzero(unsafe):
        mk = F(x['m'][k]); Lk = F(x['radius'][k])*F(x['force'][k])
        margin1 = mi-mk
        margin2 = mi*Lk-mk*Li
        if margin1 > 0 and margin2 > 0:
            ratio_margin = mi-mk*max(Fraction(1),Li/Lk)
            rows.append(dict(case=audit.CASES[int(x['phase'][i])],time_s=float(x['time'][i]),
                m_Nm=float(mi),force_N=float(x['force'][i]),radius_m=float(x['radius'][i]),
                unsafe_case=audit.CASES[int(x['phase'][k])],unsafe_time_s=float(x['time'][k]),
                absolute_margin_Nm=float(margin1),crossproduct_margin_Nm2=float(margin2),
                separating_margin_Nm=float(ratio_margin),
                exact_separating_margin_numerator=str(ratio_margin.numerator),
                exact_separating_margin_denominator=str(ratio_margin.denominator)))
            break
result = dict(status='post-release supplementary exact-sign certificate; not preregistered',
    frozen_audit_source_sha256=hashlib.sha256((HERE/'audit_bounds.py').read_bytes()).hexdigest(),
    certificate_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    original_audit_result_sha256=hashlib.sha256((HERE/'bounds_audit.json').read_bytes()).hexdigest(),
    interpretation='Each row has strictly positive exact signs for both nonnegative affine dominance inequalities. This confirms a necessary loss bound for stored numerical inputs; it is not a physical/general estimator impossibility.',
    strict_forced_useful_count=len(rows),total_harmful_A=int(x['harmful'].sum()),
    min_absolute_margin_Nm=min(r['absolute_margin_Nm'] for r in rows),
    min_crossproduct_margin_Nm2=min(r['crossproduct_margin_Nm2'] for r in rows),
    min_separating_margin_Nm=min(r['separating_margin_Nm'] for r in rows),
    unsafe_point_gap_above_native_max_mm=float(np.min(x['dA'][unsafe]-M)),
    witnesses=rows)
(HERE/'strict_margin_certificate.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='witnesses'},indent=2))
