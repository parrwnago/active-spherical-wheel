"""Equilibrium, complete-step finite differences, and local state feedback."""
from __future__ import annotations

import json
from pathlib import Path
import numpy as np
import mujoco
import yaml
from scipy.optimize import least_squares
from scipy.linalg import solve_discrete_are, orth


def find_equilibrium(plant, phase=None):
    """Find static force balance near upright, retaining the actual contact geometry."""
    radius = plant.config['geometry']['sphere_radius_m']
    phase = plant.config['simulation'].get('initial_shaft_phase_rad',.30) if phase is None else phase
    acceleration_scale = np.array([1,1,1,.12,.12,.12,.02,.02,.02])

    def assign(z):
        plant.reset(tilt=z[1:3], shaft_phase=phase)
        plant.data.qpos[2] = z[0]
        plant.apply_input(np.r_[z[3:], 0., 0.])
        mujoco.mj_forward(plant.model, plant.data)

    def residual(z):
        assign(z)
        return np.r_[plant.data.qacc * acceleration_scale, z[3:] * 1e-6]

    fit = least_squares(residual, [radius-.0001, 0, 0, 0, 0, 0],
                        bounds=([radius-.005,-.15,-.15,*(-plant.bounds[:3])],
                                [radius+.001,.15,.15,*plant.bounds[:3]]),
                        xtol=1e-12,ftol=1e-12,gtol=1e-12,max_nfev=300,
                        diff_step=1e-5, x_scale=[.001,.01,.01,.1,.1,.1])
    assign(fit.x)
    qpos, qvel = plant.data.qpos.copy(), plant.data.qvel.copy()
    u = np.r_[fit.x[3:],0.,0.]
    acceleration = plant.data.qacc.copy()
    contacts = plant.contact_summary()
    plant.step(u)
    residual_step = plant.error(qpos,qvel)
    plant.reset(qpos=qpos,qvel=qvel)
    return {'qpos':qpos, 'qvel':qvel, 'u':u, 'phase':float(phase),
            'acceleration':acceleration, 'step_residual':residual_step,
            'fit_cost':float(fit.cost), 'optimizer_success':bool(fit.success), 'contacts':contacts}


def linearize(plant, equilibrium, epsilon=1e-5):
    """Differentiate the complete held-command controller interval, including belts."""
    q0,v0,u0 = (equilibrium[k] for k in ('qpos','qvel','u'))
    plant.reset(qpos=q0,qvel=v0)
    base = plant.capture()
    nx = 2*plant.model.nv
    def evaluate(dx,du):
        plant.restore(base)
        plant.perturb(dx)
        plant.step(u0+du)
        return plant.error(q0,v0)
    zero = np.zeros(nx)
    f0 = evaluate(zero,np.zeros(5))
    A = np.column_stack([(evaluate(np.eye(nx)[i]*epsilon,np.zeros(5))-evaluate(-np.eye(nx)[i]*epsilon,np.zeros(5)))/(2*epsilon) for i in range(nx)])
    B = np.column_stack([(evaluate(zero,np.eye(5)[i]*epsilon)-evaluate(zero,-np.eye(5)[i]*epsilon))/(2*epsilon) for i in range(5)])
    plant.restore(base)
    return A,B,f0


def design_lqr(plant, equilibrium, epsilon=1e-5, frozen_phase=False):
    A,B,f0=linearize(plant,equilibrium,epsilon)
    # Frozen material phases and unregulated yaw angle are omitted locally.
    # Height/contact compression and all velocities remain in the local dynamics.
    keep=np.array([0,1,2,3,4,*range(9,18)]) if frozen_phase else np.arange(18)
    Ar,Br=A[np.ix_(keep,keep)], B[keep]
    full_scales=np.array([.15,.15,.005,.15,.15,.5,1.,1.,1.,1.,1.,.1,1.,1.,1.,10.,10.,10.])
    scales=full_scales[keep]
    As=Ar*scales[None,:]/scales[:,None]
    Bs=Br*plant.bounds[None,:]/scales[:,None]
    # Compute reachable subspace; retain all stable modes in diagnostics even if
    # unactuated. An unstable discarded mode is a design failure.
    reach=Bs.copy()
    for _ in range(Ar.shape[0]):
        reach=np.column_stack((reach,As@reach[:,-Bs.shape[1]:]))
    U,s,_=np.linalg.svd(reach,full_matrices=False)
    rank=int(np.sum(s>max(s[0]*1e-8,1e-10)))
    T=U[:,:rank]
    Tn=U[:,rank:]
    omitted=np.linalg.eigvals(Tn.T@As@Tn) if Tn.size else np.array([])
    if np.any(np.abs(omitted)>1.00001):
        raise RuntimeError(f'Unstabilizable growing modes in local model: {omitted}')
    Ac,Bc=T.T@As@T,T.T@Bs
    # Normalized penalties: prioritize balance, position and velocity; permit
    # unregulated yaw and internal cyclic motion under their declared bounds.
    Q=np.diag(np.array([2,2,1,30,30,.01,.001,.001,.001,2,2,.1,4,4,.05,.01,.01,.01])[keep])
    R=np.diag([.3,.3,.3,1.,1.])
    P=solve_discrete_are(Ac,Bc,T.T@Q@T,R)
    Kc=np.linalg.solve(R+Bc.T@P@Bc,Bc.T@P@Ac)
    Kscaled=Kc@T.T
    K=np.zeros((5,A.shape[0]))
    K[:,keep]=plant.bounds[:,None]*Kscaled/scales[None,:]
    closed=np.linalg.eigvals(A-B@K)
    return {'A':A,'B':B,'K':K,'f0':f0,'keep':keep,'reachable_rank':rank,
            'reach_singular_values':s, 'discarded_eigenvalues':omitted,
            'open_eigenvalues':np.linalg.eigvals(Ar), 'closed_eigenvalues':closed,
            'epsilon':float(epsilon),'state_scales':scales,
            'method': 'frozen_phase_feedback' if frozen_phase else 'full_state_lqr',
            'full_closed_spectral_radius':float(max(abs(closed)))}


def design_balance_feedback(plant,equilibrium,epsilon=1e-5):
    """Pragmatic frozen-phase feedback; full-state stability is NOT certified."""
    return design_lqr(plant,equilibrium,epsilon,frozen_phase=True)


def jsonable(value):
    if isinstance(value,np.ndarray):
        return ([{'real':float(v.real),'imag':float(v.imag)} for v in value] if np.iscomplexobj(value) else value.tolist())
    if isinstance(value,(np.integer,np.floating)): return value.item()
    if isinstance(value,dict): return {k:jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [jsonable(v) for v in value]
    return value


def save_design(plant, output, phase=None, method='full_state_lqr'):
    output=Path(output); output.mkdir(parents=True,exist_ok=True)
    from .model import generate_model
    generate_model(plant.config,output/'model')
    (output/'configuration.yaml').write_text(yaml.safe_dump(plant.config,sort_keys=False))
    eq=find_equilibrium(plant,phase)
    design=design_lqr(plant,eq,frozen_phase=method=='frozen_phase_feedback')
    np.savez(output/'controller.npz',model_path=np.array('model/model.xml'),**{k:v for k,v in {**eq,**design}.items() if isinstance(v,(np.ndarray,float,int))})
    (output/'linear_analysis.json').write_text(json.dumps(jsonable({'engine_version':mujoco.__version__,'model_path':'model/model.xml','configuration':plant.config,'equilibrium':eq,'design':design}),indent=2))
    return eq,design
