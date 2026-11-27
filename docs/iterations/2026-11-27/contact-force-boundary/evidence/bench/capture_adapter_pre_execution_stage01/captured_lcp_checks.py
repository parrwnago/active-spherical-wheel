"""Original-order numerical checks; no solves, permutations or fitted choices."""
from __future__ import annotations

import numpy as np


def check_original_problem(*, n, stride, A_before, b_before, lo_before, hi_before,
                           findex_before, x_after, nub=0):
    n, stride, nub = int(n), int(stride), int(nub)
    expected_stride = n if n <= 1 else ((n-1) | 3)+1
    if n < 1 or stride != expected_stride or nub != 0:
        raise ValueError('expected positive size, ODE dPAD stride, native nub0')
    padded = np.asarray(A_before, dtype=float).reshape(n, stride)
    a = padded[:, :n]
    b, lo, hi, x = (np.asarray(v, dtype=float) for v in
                    (b_before, lo_before, hi_before, x_after))
    raw_index = np.asarray(findex_before)
    if any(v.shape != (n,) for v in (b, lo, hi, x, raw_index)):
        raise ValueError('all vector inputs must have original n entries')
    if (not np.isfinite(raw_index).all() or not np.equal(raw_index, np.floor(raw_index)).all()
            or (raw_index < -1).any() or (raw_index >= n).any()):
        raise ValueError('invalid original findex')
    index = raw_index.astype(int)
    if (not np.isfinite(a).all() or not np.isfinite(b).all()
            or np.isnan(lo).any() or np.isnan(hi).any() or np.any(lo > hi)):
        raise ValueError('nonfinite matrix/rhs or invalid initial bounds')
    friction = index >= 0
    if not np.isfinite(x).all():
        return dict(valid_final_vector=False, n=n, stride=stride,
                    nonfinite_x_indices=np.flatnonzero(~np.isfinite(x)))
    lower, upper = lo.copy(), hi.copy()
    normal = np.full(n, np.nan)
    normal[friction] = x[index[friction]]
    # This matches documented Dantzig coefficient-to-box absolute scaling.
    lower[friction] = -np.abs(hi[friction]*normal[friction])
    upper[friction] = np.abs(hi[friction]*normal[friction])
    if not (np.isfinite(lower[friction]).all() and np.isfinite(upper[friction]).all()):
        raise ValueError('friction coefficient must be finite')
    w = a @ x-b
    violations = np.maximum.reduce([np.zeros(n), lower-x, x-upper])
    projected = x-np.clip(x-w, lower, upper)
    fixed_violations = np.maximum.reduce([np.zeros(n), lo-x, x-hi])
    scale = max(1., float(np.max(abs(x))))
    numerical_tolerance = 1e-10*scale
    magnitude = np.max(np.abs(a), initial=0.)
    symmetry = float(np.max(np.abs(a-a.T), initial=0.))
    return dict(valid_final_vector=True, n=n, stride=stride, nub=nub,
                matrix_symmetry_max_abs=symmetry,
                matrix_symmetry_relative=symmetry/max(1., float(magnitude)),
                x_final=x.copy(), w_original_Ax_minus_b=w,
                friction_row=friction, original_findex=index,
                paired_final_normal_impulse=normal,
                final_dependent_lo=lower, final_dependent_hi=upper,
                final_dependent_bound_violation=violations,
                final_dependent_max_bound_violation=float(np.max(violations)),
                friction_max_bound_violation=float(np.max(violations[friction], initial=0.)),
                friction_bound_violating_indices=np.flatnonzero(friction & (violations > numerical_tolerance)),
                unit_step_projected_kkt_residual=projected,
                max_abs_unit_step_projected_kkt_residual=float(np.max(abs(projected))),
                original_constant_box_violation=fixed_violations,
                numerical_bound_detection_tolerance=numerical_tolerance,
                numerical_detection_scope='Only floating point screening; no physical calibration.',
                ignored_padding_columns=stride-n,
                algebraic_convention='w=A_original*x_final-b_original; x-clip(x-w,lo_eff,hi_eff), algebraic unit step.')
