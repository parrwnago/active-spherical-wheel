"""Shared residual calibration and covariance scores; NumPy only.

Residuals use one common engineering scale, 0.03 Nm + 0.10 R ||F_filtered||.
The caller supplies training rows and episode weights. No candidate-specific
fit, covariance, normalization or posterior probability is constructed here.
"""
import numpy as np

SCORE_TIE_RTOL=1e-9


def normalize_residuals(raw_residual_Nm,radius_m,filtered_force_norm_N):
    """Return (dimensionless residuals, validity), preserving unknown rows.

    Radius must be positive and force magnitude nonnegative. Missing values
    yield NaNs and false validity; no force-floor or candidate policy is added.
    Both candidates must use the same radius and measured filtered force.
    """
    raw=np.asarray(raw_residual_Nm,dtype=float)
    if raw.ndim<1 or raw.shape[-1]!=3:raise ValueError('Expected residuals with three joint components')
    shape=raw.shape[:-1]
    try:
        radius=np.broadcast_to(np.asarray(radius_m,float),shape)
        force=np.broadcast_to(np.asarray(filtered_force_norm_N,float),shape)
    except ValueError as error:raise ValueError('Radius and force must broadcast over residual rows') from error
    with np.errstate(over='ignore',invalid='ignore'):
        scale=.03+(.10*radius)*force
    valid=np.isfinite(raw).all(axis=-1)&np.isfinite(radius)&(radius>0)&np.isfinite(force)&(force>=0)&np.isfinite(scale)&(scale>0)
    normalized=np.full(raw.shape,np.nan,dtype=float)
    np.divide(raw,scale[...,None],out=normalized,where=valid[...,None])
    return normalized,valid


def fit_shared_covariance(training_residuals,weights,*,rho,floor_std):
    """Fit a weighted mean and population covariance from finite N-by-3 rows.

    Caller-supplied nonnegative weights are normalized to sum to one; the
    function does not infer episodes. To weight episodes equally, their total
    supplied weights must be equal, irrespective of their row counts.

    Regularization is fixed by parameters, without numerical retuning:
    Sreg=(1-rho)S+rho*trace(S)/3*I+floor_std**2*I.
    rho lies in [0,1]; floor_std is a strictly positive dimensionless standard
    deviation. Scores are squared distances, not calibrated probabilities.
    """
    data=np.asarray(training_residuals,dtype=float);weight=np.asarray(weights,dtype=float)
    if data.ndim!=2 or data.shape[1]!=3 or data.shape[0]==0:raise ValueError('Finite nonempty N-by-3 training rows required')
    if not np.isfinite(data).all():raise ValueError('Training residual rows must all be finite')
    if weight.shape!=(len(data),) or not np.isfinite(weight).all() or np.any(weight<0):raise ValueError('One finite nonnegative weight per training row required')
    total=float(np.sum(weight,dtype=np.float64))
    if not np.isfinite(total) or total<=0:raise ValueError('Training weights must have a finite positive sum')
    if not np.isscalar(rho) or not np.isfinite(rho) or not 0<=rho<=1:raise ValueError('rho must be finite and in [0,1]')
    if not np.isscalar(floor_std) or not np.isfinite(floor_std) or floor_std<=0:raise ValueError('floor_std must be positive and finite')
    with np.errstate(over='ignore',under='ignore',invalid='ignore'):
        floor_variance=float(floor_std)*float(floor_std)
    if not np.isfinite(floor_variance) or floor_variance<=0:raise ValueError('floor_std squared must be positive and finite in float64')
    normalized_weight=weight/total
    with np.errstate(over='ignore',invalid='ignore'):
        mean=np.sum(data*normalized_weight[:,None],axis=0,dtype=np.float64)
        centered=data-mean
        population=np.einsum('ni,nj,n->ij',centered,centered,normalized_weight,optimize=False)
        # Symmetry enforces the mathematical covariance after floating sums.
        population=.5*population+.5*population.T
        covariance=(1.-float(rho))*population+float(rho)*(np.trace(population)/3.)*np.eye(3)+floor_variance*np.eye(3)
    if not np.isfinite(mean).all() or not np.isfinite(covariance).all():raise ValueError('Training covariance overflowed float64')
    try:cholesky=np.linalg.cholesky(covariance)
    except np.linalg.LinAlgError as error:raise ValueError('Fixed regularization is not positive definite at float64 precision') from error
    return dict(mean=mean,population_covariance=population,covariance=covariance,cholesky=cholesky,
        normalized_weights=normalized_weight,rho=float(rho),floor_std=float(floor_std))


def score_residuals(residuals,model):
    """Squared solve(L,e-mean) norm; nonfinite inference rows remain unknown."""
    values=np.asarray(residuals,dtype=float)
    if values.ndim<1 or values.shape[-1]!=3:raise ValueError('Expected three-component normalized residuals')
    mean=np.asarray(model['mean'],dtype=float);factor=np.asarray(model['cholesky'],dtype=float)
    if (mean.shape!=(3,) or factor.shape!=(3,3) or not np.isfinite(mean).all()
            or not np.isfinite(factor).all() or np.any(np.diag(factor)<=0)
            or not np.array_equal(factor,np.tril(factor))):raise ValueError('Finite mean and positive lower-triangular Cholesky required')
    flat=values.reshape(-1,3);valid=np.isfinite(flat).all(axis=1);scores=np.full(len(flat),np.nan)
    if np.any(valid):
        with np.errstate(over='ignore',invalid='ignore'):
            whitened=np.linalg.solve(factor,(flat[valid]-mean).T)
            computed=np.sum(whitened*whitened,axis=0,dtype=np.float64)
        scores[valid]=np.where(np.isfinite(computed),computed,np.nan)
    return scores.reshape(values.shape[:-1])


def score_pair(residual_F,residual_N,model,*,pattern_epsilon=0.,valid_F=True,valid_N=True):
    """Use the same fitted model for F and N; ties/ambiguity are unknown.

    Pattern ambiguity is ||eF-eN|| <= pattern_epsilon, measured before the
    shared covariance transformation. pattern_epsilon=0 flags identical rows;
    the experiment protocol may pass a larger fixed value. Otherwise a score
    tie is abs(sF-sN)<=1e-9*max(1,sF,sN). No preference is forced for missing
    rows, pattern ambiguity or score ties. External validity flags preserve
    warmup/availability rules even when their residual arrays are finite.
    """
    if not np.isscalar(pattern_epsilon) or not np.isfinite(pattern_epsilon) or pattern_epsilon<0:raise ValueError('Nonnegative finite pattern_epsilon required')
    first=np.asarray(residual_F,float);second=np.asarray(residual_N,float)
    if first.ndim<1 or second.ndim<1 or first.shape[-1]!=3 or second.shape[-1]!=3:raise ValueError('Expected three-component normalized residual pairs')
    try:
        first,second=np.broadcast_arrays(first,second);shape=first.shape[:-1]
        available_F=np.broadcast_to(np.asarray(valid_F,bool),shape);available_N=np.broadcast_to(np.asarray(valid_N,bool),shape)
    except ValueError as error:raise ValueError('Pair residuals and validity must broadcast') from error
    score_F=score_residuals(first,model);score_N=score_residuals(second,model)
    valid=available_F&available_N&np.isfinite(score_F)&np.isfinite(score_N)
    with np.errstate(over='ignore',invalid='ignore'):
        pattern_distance=np.linalg.norm(first-second,axis=-1)
        tolerance=SCORE_TIE_RTOL*np.maximum(1.,np.maximum(score_F,score_N))
        raw_score_tie=np.abs(score_F-score_N)<=tolerance
    ambiguous=valid&(pattern_distance<=pattern_epsilon)
    tie=valid&~ambiguous&raw_score_tie
    distinguishable=valid&~ambiguous&~tie
    lower_F=distinguishable&(score_F<score_N);lower_N=distinguishable&(score_N<score_F)
    unknown=~(lower_F|lower_N)
    decision=np.full(shape,'unknown',dtype='<U7');decision[lower_F]='F';decision[lower_N]='N'
    reason=np.full(shape,'invalid',dtype='<U9');reason[ambiguous]='pattern';reason[tie]='score_tie';reason[lower_F|lower_N]='known'
    return dict(score_F=score_F,score_N=score_N,pattern_distance=pattern_distance,
        score_tie_tolerance=tolerance,valid_pair=valid,ambiguous=ambiguous,tie=tie,
        invalid=~valid,F_lower=lower_F,N_lower=lower_N,unknown=unknown,decision=decision,unknown_reason=reason)
