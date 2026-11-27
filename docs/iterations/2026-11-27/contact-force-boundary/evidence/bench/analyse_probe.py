"""Independent original-array bound/KKT check for the prescribed DART probe."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def bounds(lo, hi, findex, x):
    lower, upper = np.array(lo, float), np.array(hi, float)
    for i, normal in enumerate(findex):
        if normal >= 0:
            lower[i] *= abs(x[normal]); upper[i] *= abs(x[normal])
    return lower, upper


def residuals(a, b, x, lower, upper):
    w = a @ x-b
    violation = np.maximum.reduce([np.zeros(len(x)), lower-x, x-upper])
    projected = x-np.clip(x-w, lower, upper)
    return dict(w_Ax_minus_b=w.tolist(), bound_violation=violation.tolist(),
                max_bound_violation=float(violation.max()),
                unit_step_projected_kkt_residual=projected.tolist(),
                max_abs_unit_step_projected_kkt_residual=float(np.max(abs(projected))))


def analyse(packet):
    out = dict(scope='Prescribed witness only; not the original robot problem',
               kkt_convention='w=A_original*x_final-b_original; projected residual x-clip(x-w,lo_eff,hi_eff). Unit algebraic step, not a physical error scale.',
               post_auxiliary_arrays='Dantzig may leave A,b,lo,hi,findex permuted; no entrywise original-order interpretation.',
               runs=[])
    for run in packet['runs']:
        a = np.array(run['A_original_row_major'], float).reshape(run['n'], run['n'])
        b, x, lo = (np.array(run[key], float) for key in ('b_original', 'x', 'lo_original'))
        hi = np.array([np.inf if value is None else value for value in run['hi_original_null_is_infinity']])
        findex = np.array(run['findex_original'], int)
        lower, upper = bounds(lo, hi, findex, x)
        witness_reference = np.array([1/2.05, .7/2.05, 0.])
        ref_lo, ref_hi = bounds(lo, hi, findex, witness_reference)
        record = dict(label=run['label'], solver=run['solver'], returned_success=run['success'],
                      early_termination=run['early_termination'], x=x.tolist(),
                      eigenvalues_A=np.linalg.eigvalsh(a).tolist(),
                      dynamic_final_normal_bounds=dict(lo=lower.tolist(), hi=[None if np.isinf(v) else float(v) for v in upper]),
                      dynamic_final_normal_check=residuals(a, b, x, lower, upper),
                      original_constant_box_check=residuals(a, b, x, lo, hi),
                      analytical_coupled_reference=dict(x=witness_reference.tolist(),
                                                       residuals=residuals(a, b, witness_reference, ref_lo, ref_hi)),
                      error_from_analytical_coupled_reference=(x-witness_reference).tolist())
        if not np.allclose(np.array(run['w_original_Ax_minus_b']), a @ x-b, atol=1e-14, rtol=0):
            raise ValueError('C++/NumPy original-system w disagreement')
        out['runs'].append(record)
    return out


def main():
    p = argparse.ArgumentParser(); p.add_argument('--input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True); args = p.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    report = analyse(json.loads(args.input.read_text()))
    report['probe_output_sha256'] = hashlib.sha256(args.input.read_bytes()).hexdigest()
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+'\n')
    print(json.dumps(report, allow_nan=False))


if __name__ == '__main__':
    main()
