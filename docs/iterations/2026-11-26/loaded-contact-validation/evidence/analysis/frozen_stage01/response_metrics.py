"""Pure paired-endpoint arithmetic for I57; imports no simulator/controller."""
import numpy as np

GROUPS = {
    'planar': dict(indices=(0,1), floor=2e-4, units='m/s'),
    'transverse': dict(indices=(3,4), floor=2e-3, units='rad/s'),
    'world_yaw': dict(indices=(9,), floor=2e-3, units='rad/s'),
    'shaft': dict(indices=(6,), floor=2e-2, units='rad/s'),
    'shell_A': dict(indices=(7,), floor=2e-2, units='rad/s'),
    'shell_B': dict(indices=(8,), floor=2e-2, units='rad/s'),
}
THRESHOLDS = dict(relative_error_max=.5, cosine_min=.8, gain_min=.5,
                  gain_max=2., nonlinearity_max=.25)


def rotation(quaternion):
    q=np.asarray(quaternion,float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-6:
        raise ValueError('Finite unit wxyz quaternion required')
    w,x,y,z=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def physical_difference(q_a,v_a,q_b,v_b,initial_rotation):
    """Rows0:9 initial-frame generalized difference; row9 world yaw difference."""
    v_a,v_b=np.asarray(v_a,float),np.asarray(v_b,float)
    if v_a.shape!=(9,) or v_b.shape!=(9,):raise ValueError('Nine qvel coordinates required')
    delta_world_omega=rotation(np.asarray(q_a)[3:7])@v_a[3:6]-rotation(np.asarray(q_b)[3:7])@v_b[3:6]
    result=np.r_[v_a[:3]-v_b[:3],np.asarray(initial_rotation).T@delta_world_omega,v_a[6:]-v_b[6:],delta_world_omega[2]]
    if not np.isfinite(result).all():raise ValueError('Nonfinite physical endpoint difference')
    return result


def prediction(response,delta_input,initial_rotation):
    p=np.asarray(response,float)@np.asarray(delta_input,float)
    return np.r_[p,(np.asarray(initial_rotation)@p[3:6])[2]]


def score(predicted,observed,floor,*,is_transverse=False):
    p,o=np.atleast_1d(predicted).astype(float),np.atleast_1d(observed).astype(float)
    if p.shape!=o.shape or not np.isfinite(np.r_[p,o]).all() or floor<=0:
        raise ValueError('Matching finite response vectors and positive floor required')
    pn,on=float(np.linalg.norm(p)),float(np.linalg.norm(o));error=float(np.linalg.norm(o-p))
    presolved,oresolved=pn>floor,on>floor
    cosine=float(np.clip(p@o/(pn*on),-1,1)) if presolved and oresolved else None
    gain=on/pn if presolved else None
    relative=error/max(on,floor)
    if not presolved and not oresolved:status='unresolved';passed=None
    elif not presolved:status='false_negligible_authority';passed=False
    elif not oresolved:status='overpredicted_authority';passed=False
    else:
        passed=bool(relative<=.5 and cosine>=.8 and .5<=gain<=2)
        status='resolved_pass' if passed else 'resolved_fail'
    return dict(predicted=p.tolist(),observed=o.tolist(),floor=float(floor),predicted_norm=pn,
        observed_norm=on,absolute_error=error,relative_vector_error=relative,directional_cosine=cosine,
        observed_over_predicted_gain=gain,status=status,passes_screen=passed,
        transverse_nearzero_absolute_pass=bool(on<=floor) if is_transverse and not presolved else None)


def score_groups(predicted,observed):
    return {name:dict(score(np.asarray(predicted)[list(spec['indices'])],np.asarray(observed)[list(spec['indices'])],
                spec['floor'],is_transverse=name=='transverse'),units=spec['units']) for name,spec in GROUPS.items()}


def linearity(full,half):
    result={}
    for name,spec in GROUPS.items():
        ix=list(spec['indices']);a,b=np.asarray(full)[ix],2*np.asarray(half)[ix]
        norm=float(np.linalg.norm(a));scaled_half_norm=float(np.linalg.norm(b));error=float(np.linalg.norm(a-b));ratio=error/max(norm,spec['floor'])
        unresolved=norm<=spec['floor'] and scaled_half_norm<=spec['floor']
        result[name]=dict(full_observed=a.tolist(),twice_half_observed=b.tolist(),absolute_nonlinearity=error,
            relative_nonlinearity=ratio,floor=spec['floor'],unresolved=unresolved,
            passes_screen=None if unresolved else bool(ratio<=.25),units=spec['units'])
    return result
