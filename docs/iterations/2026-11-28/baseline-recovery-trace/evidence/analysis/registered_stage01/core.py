"""I86 pure force/held-command reduction; existing I81 kinematics are reused.

No physics, controller, contact estimator or terrain query runs here.
"""
from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
LEGACY_PATH = ROOT/'results/slope_response_trace_20261128/analysis/core.py'
spec = importlib.util.spec_from_file_location('i86_frozen_i81_helpers', LEGACY_PATH)
legacy = importlib.util.module_from_spec(spec)
spec.loader.exec_module(legacy)
FAMILIES = legacy.FAMILIES


def model_properties(path):
    tree = ET.parse(path)
    fork = tree.find("worldbody/body[@name='fork']")
    bodies = {b.attrib['name']: b for b in fork.iter('body')}
    names = ('fork', 'carrier', 'shell_A', 'shell_B')
    if set(bodies) != set(names):
        raise ValueError('Whole-COM model requires captured four-body topology')
    result = {}
    for name in names:
        b = bodies[name]
        if name != 'fork' and np.any(np.fromstring(b.get('pos', '0 0 0'), sep=' ')):
            raise ValueError('Unexpected fixed child offset')
        if 'quat' in b.attrib or 'euler' in b.attrib:
            raise ValueError('Unexpected fixed body orientation')
        inertia = b.find('inertial')
        result[name] = dict(mass=float(inertia.get('mass')),
            inertial_pos=np.fromstring(inertia.get('pos', '0 0 0'), sep=' '))
    for name, axis in [('carrier', [0, 1, 0]), ('shell_A', [1, 0, 0]), ('shell_B', [1, 0, 0])]:
        np.testing.assert_array_equal(np.fromstring(bodies[name].find('joint').get('axis'), sep=' '), axis)
    return result, np.fromstring(tree.find('option').get('gravity'), sep=' ')


def force_sets(rows):
    """Keep all signed rows in closure; pressure-only masks are separate."""
    c = np.asarray(rows, float).reshape(-1, 16)
    tags = c[:, 0].astype(int)
    if np.any(c[:, 0] != tags) or np.any((tags < 0) | (tags > 6)):
        raise ValueError('Unknown contact tag')
    force = c[:, 7:10]
    admitted = c[:, 12] >= .5
    fn = np.einsum('ij,ij->i', c[:, 4:7], force)
    native = admitted & ((c[:, 10] > 0.) | (fn > 0.))
    family = legacy.TAG_TO_FAMILY[tags]
    return dict(all=force.sum(axis=0), admitted=force[admitted].sum(axis=0),
        native=force[native].sum(axis=0), rejected=force[~admitted].sum(axis=0),
        family=np.asarray([force[family == k].sum(axis=0) for k in range(4)]),
        body=force[tags == 6].sum(axis=0))


def validate_trace(trace, precision, definitions):
    n = definitions['expected_substeps']
    dt, control = definitions['physics_dt_s'], definitions['control_dt_s']
    lo, hi = definitions['trace_window_s']
    if len(trace['time_s']) != n or not np.all(trace['hold_completed']):
        raise AssertionError('Missing or incomplete trace')
    np.testing.assert_allclose(trace['time_s'], lo+np.arange(1, n+1)*dt, rtol=0, atol=1e-8)
    np.testing.assert_array_equal(trace['substep_index'], np.tile(np.arange(1, 6), n//5))
    holds = np.repeat(np.arange(round(lo/control), round(hi/control)), 5)
    np.testing.assert_array_equal(trace['hold_index'], holds)
    np.testing.assert_array_equal(trace['held_input'], precision['exact_applied'][holds])
    np.testing.assert_allclose(trace['control_start_time_s'], precision['time_s'][holds], rtol=0, atol=1e-8)
    np.testing.assert_array_equal(precision['exact_previous_applied'][holds+1], trace['held_input'])
    if np.any(np.diff(trace['contact_offsets']) < 0) or trace['contact_offsets'][-1] != len(trace['contact_rows']):
        raise AssertionError('Bad contact offsets')
    return dict(complete_substeps=n, complete_holds=n//5, held_command_identity=True,
        endpoint_is_next_observer_row=True, no_controller_intervention=True)


def fixed_basis(normal, gravity):
    n = legacy.unit(normal)
    down = legacy.unit(np.asarray(gravity)-n*np.dot(n, gravity))
    if not np.isfinite(down).all():
        raise ValueError('Fixed downhill direction undefined on level support')
    cross = legacy.unit(np.cross(n, down))
    basis = np.stack([down, cross, n])
    np.testing.assert_allclose(basis@basis.T, np.eye(3), rtol=0, atol=1e-12)
    return basis


def window_indices(times, lo, hi):
    """Completed intervals: (lo,hi], not command-start inclusive mask."""
    t = np.asarray(times)
    return np.flatnonzero((t > lo+1e-9) & (t <= hi+1e-9))


def sampled_balance(force, disturbance, gravity, mass, before_velocity, after_velocity, dt):
    net = np.asarray(force)+mass*np.asarray(gravity)+np.asarray(disturbance)[..., :3]
    required = mass*(np.asarray(after_velocity)-np.asarray(before_velocity))/dt
    return net, required, net-required


def reduce_trace(trace, precision, properties, gravity, definitions):
    validate = validate_trace(trace, precision, definitions)
    n = len(trace['time_s']); dt = definitions['physics_dt_s']
    mass = sum(p['mass'] for p in properties.values())
    arrays = {key: [] for key in ['force_all', 'force_admitted', 'force_native', 'force_rejected',
        'force_family', 'force_body', 'moment_all', 'pressure_family', 'pressure_normal_sum',
        'COM_position_before', 'COM_position_after', 'COM_velocity_before', 'COM_velocity_after',
        'root_velocity_before', 'root_velocity_after', 'root_position_after']}
    for j in range(n):
        rows = trace['contact_rows'][trace['contact_offsets'][j]:trace['contact_offsets'][j+1]]
        force = force_sets(rows)
        qb, vb = legacy.bridge_qv(trace['bridge_state_before'][j])
        qa, va = legacy.bridge_qv(trace['bridge_state'][j])
        pb, cb = legacy.com_state(qb, vb, properties)
        pa, ca = legacy.com_state(qa, va, properties)
        reduction = legacy.contact_reduction(rows, qb[:3], qa[:3])
        for key in ['all', 'admitted', 'native', 'rejected', 'family', 'body']:
            arrays['force_'+key].append(force[key])
        arrays['moment_all'].append(reduction['all_moment'])
        arrays['pressure_family'].append(reduction['pressure_by_family_N'])
        arrays['pressure_normal_sum'].append(reduction['positive_normal_sum'])
        for key, value in [('COM_position_before', pb), ('COM_position_after', pa),
            ('COM_velocity_before', cb), ('COM_velocity_after', ca),
            ('root_velocity_before', vb[:3]), ('root_velocity_after', va[:3]), ('root_position_after', qa[:3])]:
            arrays[key].append(value)
    a = {key: np.asarray(value) for key, value in arrays.items()}
    a['time_s'] = trace['time_s'].copy()
    a['held_input'] = trace['held_input'].copy()
    for key in ['all', 'admitted', 'native']:
        net, required, residual = sampled_balance(a['force_'+key], trace['disturbance_world'],
            gravity, mass, a['COM_velocity_before'], a['COM_velocity_after'], dt)
        a['net_'+key] = net
        a['COM_required_force'] = required
        a['residual_'+key] = residual
    a['root_acceleration'] = (a['root_velocity_after']-a['root_velocity_before'])/dt
    start = int(round(definitions['focus_window_s'][0]/definitions['control_dt_s']))
    basis = fixed_basis(precision['exact_estimated_normal'][start], gravity)
    for key in ['force_all', 'COM_required_force', 'net_all', 'residual_all']:
        a[key+'_fixed_basis'] = a[key]@basis.T
    return a, dict(**validate, mass_kg=mass, gravity_m_s2=gravity,
        fixed_basis_rows_down_cross_normal=basis, fixed_basis_anchor_s=definitions['focus_window_s'][0])


def interval_arrays(substeps, trace, precision, definitions):
    n = len(substeps['time_s'])//5
    r = dict(time_s=substeps['time_s'][4::5], hold_index=trace['hold_index'][::5],
        row_index=trace['hold_index'][::5]+1)
    for key in ['force_all', 'force_admitted', 'force_native', 'force_rejected', 'force_family',
        'force_body', 'moment_all', 'pressure_family', 'pressure_normal_sum', 'net_all',
        'net_admitted', 'net_native', 'COM_required_force', 'residual_all', 'residual_admitted',
        'residual_native', 'force_all_fixed_basis', 'COM_required_force_fixed_basis',
        'net_all_fixed_basis', 'residual_all_fixed_basis']:
        v = substeps[key]
        r[key] = v.reshape(n, 5, *v.shape[1:]).mean(axis=1)
    for key in ['COM_velocity_before', 'root_velocity_before']:
        r[key] = substeps[key][::5]
    for key in ['COM_velocity_after', 'root_velocity_after']:
        r[key] = substeps[key][4::5]
    r['held_input'] = substeps['held_input'][::5]
    r['last_pressure_family'] = substeps['pressure_family'][4::5]
    rows = r['row_index']
    r['observer_force_error'] = r['force_all']-precision['exact_observer_interval_mean'][rows, :3]
    r['observer_moment_error'] = r['moment_all']-precision['exact_observer_interval_mean'][rows, 3:6]
    r['observer_filter_force_error'] = []
    r['observer_filter_normal_error_deg'] = []
    r['controller_pressure_normal_error_deg'] = []
    first = int(trace['hold_index'][0])
    filtered = precision['exact_observer_wrench'][first].copy()
    from spherical_wheel.contact_sensing import intrinsic_normal
    for j, row in enumerate(rows):
        filtered = legacy.filter_step(filtered, np.r_[r['force_all'][j], r['moment_all'][j]])
        r['observer_filter_force_error'].append(filtered[:3]-precision['exact_observer_wrench'][row, :3])
        nf = intrinsic_normal(filtered[:3], filtered[3:], .12)
        no = intrinsic_normal(precision['exact_observer_wrench'][row, :3], precision['exact_observer_wrench'][row, 3:6], .12)
        r['observer_filter_normal_error_deg'].append(legacy.angle_deg(nf, no))
        r['controller_pressure_normal_error_deg'].append(legacy.angle_deg(
            precision['exact_estimated_normal'][row], r['pressure_normal_sum'][j]))
    for key in ['observer_filter_force_error', 'observer_filter_normal_error_deg', 'controller_pressure_normal_error_deg']:
        r[key] = np.asarray(r[key], float)
    return r


def window_balance(substeps, definitions):
    result = {}
    dt = definitions['physics_dt_s']
    for name, (lo, hi) in definitions['comparison_windows_s'].items():
        ids = window_indices(substeps['time_s'], lo, hi)
        if len(ids) != round((hi-lo)/dt):
            raise AssertionError('Fixed window missing complete physics intervals')
        impulse = substeps['net_all'][ids].sum(axis=0)*dt
        required = substeps['COM_required_force'][ids].sum(axis=0)*dt
        root_delta = substeps['root_velocity_after'][ids[-1]]-substeps['root_velocity_before'][ids[0]]
        com_delta = substeps['COM_velocity_after'][ids[-1]]-substeps['COM_velocity_before'][ids[0]]
        # v_root = v_COM - d(COM_position-root_position)/dt. This signed
        # vector identity distinguishes whole-body translation from motion
        # of the root relative to the COM; never divide their magnitudes.
        relative_contribution = root_delta-com_delta
        result[name] = dict(start_s=lo, end_s=hi, substeps=len(ids),
            contact_sampled_impulse_Ns=substeps['force_all'][ids].sum(axis=0)*dt,
            net_sampled_impulse_Ns=impulse, actual_COM_momentum_change_Ns=required,
            residual_sampled_impulse_Ns=impulse-required,
            root_velocity_change_m_s=root_delta,
            COM_velocity_change_m_s=com_delta,
            root_relative_to_COM_velocity_change_contribution_m_s=relative_contribution,
            signed_decomposition_closure_m_s=root_delta-com_delta-relative_contribution,
            net_sampled_impulse_fixed_basis_Ns=substeps['net_all_fixed_basis'][ids].sum(axis=0)*dt,
            family_contact_sampled_impulse_Ns=substeps['force_family'][ids].sum(axis=0)*dt)
    return result


def vector_stats(values):
    a = np.asarray(values, float)
    norms = np.linalg.norm(a, axis=-1)
    return dict(rows=len(a), component_mean=a.mean(axis=0),
        norm_rms=float(np.sqrt(np.mean(norms*norms))), norm_maximum=float(norms.max()))
