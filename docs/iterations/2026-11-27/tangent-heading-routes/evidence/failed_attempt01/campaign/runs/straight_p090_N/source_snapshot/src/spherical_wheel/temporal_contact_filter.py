"""Opt-in causal contact-mode filter; never imported by the live controller.

The likelihood model lives separately. Beliefs are conditional on its assumed
noise/model and are not a claim of empirically calibrated probabilities.
Transition-conditioned observations can describe a preceding interval whose
contact set differs from the current endpoint. No future samples are used.
"""
from dataclasses import dataclass
import itertools
import math
import numpy as np

FAMILIES=('shell_A','shell_B','belt_A','belt_B')
SUPPORT_SETS=tuple((i,) for i in range(4))+tuple(itertools.combinations(range(4),2))+((),)
MODE_SUPPORTS=tuple(s for s in SUPPORT_SETS[:-1] for _ in range(2))+((),)
MODE_GROUP=np.r_[np.repeat(np.arange(10),2),10]
MODE_NAMES=tuple('+'.join(FAMILIES[i] for i in s)+'/'+motion
    for s in SUPPORT_SETS[:-1] for motion in ('rolling','slip'))+('unknown',)


@dataclass(frozen=True)
class FilterSettings:
    nominal_dt_s: float=.005
    known_dwell_s: float=.05
    unknown_dwell_s: float=.02
    evidence_correlation_s: float=.02
    reset_gap_s: float=.05
    singleton_prior_mass: float=.8
    pair_prior_mass: float=.1
    unknown_prior_mass: float=.1
    rolling_prior_fraction: float=.9

    def __post_init__(self):
        values=np.array(list(vars(self).values()),float)
        if not np.isfinite(values).all() or np.any(values<=0):raise ValueError('Finite positive settings required')
        if not 0<self.rolling_prior_fraction<1:raise ValueError('Rolling fraction must be in(0,1)')
        if abs(self.singleton_prior_mass+self.pair_prior_mass+self.unknown_prior_mass-1)>1e-12:
            raise ValueError('Support prior masses must sum to one')


def base_prior(settings=None):
    s=settings or FilterSettings();p=np.empty(21)
    for k,contacts in enumerate(SUPPORT_SETS[:-1]):
        mass=(s.singleton_prior_mass/4 if len(contacts)==1 else s.pair_prior_mass/6)
        p[2*k]=mass*s.rolling_prior_fraction;p[2*k+1]=mass*(1-s.rolling_prior_fraction)
    p[20]=s.unknown_prior_mass
    return p


def transition_matrix(dt_s,settings=None):
    """One-jump holding-time approximation with fixed geometry-mode affinities.

    Retention exp(-dt/dwell); destination probabilities use the documented
    base prior and contact-set adjacency. This is not an exact CTMC matrix
    exponential and does not predict unknown terrain.
    """
    if not np.isfinite(dt_s) or dt_s<=0:raise ValueError('Positive finite elapsed time required')
    s=settings or FilterSettings();prior=base_prior(s);p=np.zeros((21,21))
    for i,old in enumerate(MODE_SUPPORTS):
        affinity=np.zeros(21)
        for j,new in enumerate(MODE_SUPPORTS):
            if i==j:continue
            if not old:factor=1.
            elif not new:factor=2.
            elif old==new:factor=4.
            elif set(old)<=set(new) or set(new)<=set(old):factor=3.
            elif len(old)==len(new)==1:factor=1.
            elif set(old)&set(new):factor=.5
            else:factor=.1
            affinity[j]=factor*prior[j]
        stay=math.exp(-dt_s/(s.unknown_dwell_s if not old else s.known_dwell_s))
        p[i]=affinity/affinity.sum()*(1-stay);p[i,i]=stay
    return p


def _logsumexp(a,axis=None):
    a=np.asarray(a,float);maximum=np.max(a,axis=axis,keepdims=True)
    safe=np.where(np.isfinite(maximum),maximum,0.)
    with np.errstate(divide='ignore',invalid='ignore'):
        out=safe+np.log(np.sum(np.exp(a-safe),axis=axis,keepdims=True))
    return np.squeeze(out,axis=axis) if axis is not None else float(out.reshape(-1)[0])


def summarize(probability):
    p=np.asarray(probability,float)
    if p.shape!=(21,) or not np.isfinite(p).all() or np.any(p<0) or abs(p.sum()-1)>1e-10:
        raise ValueError('Normalized21-mode distribution required')
    groups=np.bincount(MODE_GROUP,weights=p,minlength=11)
    selected=int(np.argmax(groups))
    inclusion=np.array([sum(p[k] for k,s in enumerate(MODE_SUPPORTS) if f in s) for f in range(4)])
    return dict(probability=p.copy(),support_probability=groups,support_index=selected,
        confidence=float(groups[selected]),unknown_probability=float(groups[10]),
        family_inclusion_probability=inclusion,
        singleton_family=selected if selected<4 else -1)


class TemporalContactFilter:
    def __init__(self,settings=None):
        self.settings=settings or FilterSettings();self.reset()

    def reset(self):
        self.probability=base_prior(self.settings);self.previous_time=None

    def update(self,time_s,log_emission,edge_log_emission=None):
        """One endpoint update. Rows of edge evidence are previous modes.

        Unknown evidence must be finite. NaN/+inf are rejected rather than
        silently converted into confidence. Missing known hypotheses use-inf.
        Equal timestamps are rejected to prevent double-counting observations.
        """
        if not np.isfinite(time_s):raise ValueError('Finite timestamp required')
        emission=np.asarray(log_emission,float)
        if emission.shape!=(21,) or np.isnan(emission).any() or np.isposinf(emission).any() or not np.isfinite(emission[20]):
            raise ValueError('21valid log potentials and finite unknown potential required')
        edge=None if edge_log_emission is None else np.asarray(edge_log_emission,float)
        if edge is not None and (edge.shape!=(21,21) or np.isnan(edge).any() or np.isposinf(edge).any() or not np.isfinite(edge[:,20]).all()):
            raise ValueError('Valid21x21transition-observation potentials required')
        first=self.previous_time is None
        elapsed=self.settings.nominal_dt_s if first else float(time_s-self.previous_time)
        if elapsed<=0:raise ValueError('Timestamps must strictly increase')
        reset=not first and elapsed>self.settings.reset_gap_s
        if reset:self.probability=base_prior(self.settings);first=True
        effective_dt=self.settings.nominal_dt_s if first else elapsed
        weight=min(1.,effective_dt/self.settings.evidence_correlation_s)
        joint=None
        if first:
            logp=np.log(self.probability)+weight*emission
        else:
            transition=transition_matrix(elapsed,self.settings)
            with np.errstate(divide='ignore'):
                logprior=np.log(self.probability)[:,None]+np.log(transition)
            observed=np.broadcast_to(emission,(21,21)) if edge is None else edge
            joint=logprior+weight*observed
            logp=_logsumexp(joint,axis=0)
        self.probability=np.exp(logp-_logsumexp(logp));self.previous_time=float(time_s)
        result=summarize(self.probability)
        group=result['support_index']
        if joint is None:
            current=int(np.argmax(np.where(MODE_GROUP==group,self.probability,-1.)))
            predecessor=-1
        else:
            selected=np.where((MODE_GROUP==group)[None,:],joint,-np.inf)
            predecessor,current=map(int,np.unravel_index(np.argmax(selected),selected.shape))
        result.update(selected_previous_mode=predecessor,selected_current_mode=current)
        result.update(evidence_weight=weight,history_reset=reset,elapsed_s=elapsed)
        return result


def filter_sequence(time_s,log_emission,edge_log_emission=None,*,settings=None):
    times=np.asarray(time_s,float);emission=np.asarray(log_emission,float)
    if times.ndim!=1 or emission.shape!=(len(times),21):raise ValueError('Nx21 evidence and N times required')
    edges=None if edge_log_emission is None else np.asarray(edge_log_emission,float)
    if edges is not None and edges.shape!=(len(times),21,21):raise ValueError('Nx21x21edge evidence required')
    estimator=TemporalContactFilter(settings);rows=[]
    for k,t in enumerate(times):rows.append(estimator.update(t,emission[k],None if edges is None else edges[k]))
    if not rows:return dict(probability=np.empty((0,21)),support_probability=np.empty((0,11)),support_index=np.empty(0,int),confidence=np.empty(0))
    return {key:np.asarray([row[key] for row in rows]) for key in rows[0]}


def frame_sequence(log_emission,*,settings=None):
    """Fixed prior/evidence model without temporal history; ablation control."""
    s=settings or FilterSettings();a=np.asarray(log_emission,float)
    if a.ndim!=2 or a.shape[1]!=21:raise ValueError('Nx21log evidence required')
    weight=min(1.,s.nominal_dt_s/s.evidence_correlation_s);rows=[]
    for e in a:
        if np.isnan(e).any() or np.isposinf(e).any() or not np.isfinite(e[20]):raise ValueError('Invalid log evidence')
        logp=np.log(base_prior(s))+weight*e
        out=summarize(np.exp(logp-_logsumexp(logp)))
        out['selected_current_mode']=int(np.argmax(np.where(MODE_GROUP==out['support_index'],out['probability'],-1.)))
        out['selected_previous_mode']=-1
        rows.append(out)
    return {key:np.asarray([row[key] for row in rows]) for key in rows[0]}
