"""I71 singleton/unknown endpoints, retaining I70 mixed-interval edge evidence.

All settings and relative preferences are frozen I70 values. Removing pair
destinations changes where a transition goes. It does not change the original
diagonal holding probability: only off-diagonal mass is redistributed.
The21-mode output is padded with exact zeros at disabled pair endpoints.
"""
from dataclasses import asdict
import numpy as np
import frozen_temporal_contact_filter as frozen


ALLOWED_MODES=np.r_[np.arange(8),20]
ALLOWED_MODES.setflags(write=False)
PAIR_MODES=np.arange(8,20)
PAIR_MODES.setflags(write=False)


def initial_prior(settings=None):
    p=frozen.base_prior(settings);p[PAIR_MODES]=0.
    return p/p.sum()


def transition_matrix(dt_s,settings=None):
    """Original holding probabilities; original allowed off-diagonal odds."""
    original=frozen.transition_matrix(dt_s,settings)
    result=np.zeros((21,21))
    for i in ALLOWED_MODES:
        weights=original[i,ALLOWED_MODES].copy()
        weights[ALLOWED_MODES==i]=0.
        stay=original[i,i]
        if 1.-stay>0:
            result[i,ALLOWED_MODES]=(1.-stay)*weights/weights.sum()
        result[i,i]=stay
    return result


def masked_emission(log_emission):
    a=np.asarray(log_emission,float)
    if a.shape!=(21,) or np.isnan(a).any() or np.isposinf(a).any() or not np.isfinite(a[20]):
        raise ValueError('21 valid log potentials with finite unknown required')
    a=a.copy();a[PAIR_MODES]=-np.inf
    return a


class RestrictedTemporalFilter:
    """Explicit I70 causal recurrence with the registered reduced graph."""
    def __init__(self,settings=None):
        self.settings=settings or frozen.FilterSettings();self.reset()

    def reset(self):
        self.probability=initial_prior(self.settings);self.previous_time=None

    def update(self,time_s,log_emission,edge_log_emission=None):
        if not np.isfinite(time_s):raise ValueError('Finite timestamp required')
        emission=masked_emission(log_emission)
        edge=None if edge_log_emission is None else np.asarray(edge_log_emission,float)
        if edge is not None:
            if edge.shape!=(21,21) or np.isnan(edge).any() or np.isposinf(edge).any() or not np.isfinite(edge[:,20]).all():
                raise ValueError('Valid21x21 transition evidence required')
            edge=edge.copy();edge[:,PAIR_MODES]=-np.inf;edge[PAIR_MODES,:]=-np.inf
        first=self.previous_time is None
        elapsed=self.settings.nominal_dt_s if first else float(time_s-self.previous_time)
        if elapsed<=0:raise ValueError('Timestamps must strictly increase')
        reset=not first and elapsed>self.settings.reset_gap_s
        if reset:self.probability=initial_prior(self.settings);first=True
        effective_dt=self.settings.nominal_dt_s if first else elapsed
        weight=min(1.,effective_dt/self.settings.evidence_correlation_s)
        joint=None
        with np.errstate(divide='ignore'):
            if first:logp=np.log(self.probability)+weight*emission
            else:
                prior=np.log(self.probability)[:,None]+np.log(transition_matrix(elapsed,self.settings))
                observed=np.broadcast_to(emission,(21,21)) if edge is None else edge
                joint=prior+weight*observed
                logp=frozen._logsumexp(joint,axis=0)
        self.probability=np.exp(logp-frozen._logsumexp(logp));self.previous_time=float(time_s)
        if np.any(self.probability[PAIR_MODES]!=0.):raise ArithmeticError('Disabled pair endpoint acquired probability')
        result=frozen.summarize(self.probability);group=result['support_index']
        if joint is None:
            current=int(np.argmax(np.where(frozen.MODE_GROUP==group,self.probability,-1.)));previous=-1
        else:
            selected=np.where((frozen.MODE_GROUP==group)[None,:],joint,-np.inf)
            previous,current=map(int,np.unravel_index(np.argmax(selected),selected.shape))
        result.update(selected_previous_mode=previous,selected_current_mode=current,
            evidence_weight=weight,history_reset=reset,elapsed_s=elapsed)
        return result


def frame_update(log_emission,settings=None):
    settings=settings or frozen.FilterSettings();emission=masked_emission(log_emission)
    weight=min(1.,settings.nominal_dt_s/settings.evidence_correlation_s)
    with np.errstate(divide='ignore'):logp=np.log(initial_prior(settings))+weight*emission
    result=frozen.summarize(np.exp(logp-frozen._logsumexp(logp)))
    result.update(selected_previous_mode=-1,
        selected_current_mode=int(np.argmax(np.where(frozen.MODE_GROUP==result['support_index'],result['probability'],-1.))))
    return result
