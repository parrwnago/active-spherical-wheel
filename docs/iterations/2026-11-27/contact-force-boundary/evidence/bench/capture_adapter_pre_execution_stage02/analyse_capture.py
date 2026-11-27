"""Private native capture adapter: no solves, geometry search or controller use."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np

from captured_lcp_checks import check_original_problem

TAG_FAMILY = np.array([0, 1, 2, 3, 0, 1, 4])


def clean(x):
    if isinstance(x, dict): return {k: clean(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)): return [clean(v) for v in x]
    if isinstance(x, np.ndarray): return clean(x.tolist())
    if isinstance(x, np.generic): return clean(x.item())
    if isinstance(x, float) and not np.isfinite(x): return None
    return x


def analyse(packets):
    solvers = {p['id']: p for p in packets if p['kind'] in (0, 1)}
    report = dict(scope='Captured native solve calls and applied contact impulses; no Gazebo-row fuzzy matching.',
                  solver_calls=[], applied_contacts=[], checks={})
    arrays = {}
    for ident, p in solvers.items():
        check = check_original_problem(n=p['n'], stride=p['stride'], nub=p['nub'],
                  A_before=p['before_A'], b_before=p['before_b'], lo_before=p['before_lo'],
                  hi_before=p['before_hi'], findex_before=p['before_findex'], x_after=p['after_x'])
        for key, value in check.items():
            if isinstance(value, np.ndarray): arrays[f'call_{ident}__{key}'] = value
        report['solver_calls'].append(dict(id=ident, solver=p['solver'], control_time_s=p['control_time_s'],
                   substep=p['substep'], returned_success=p['success'], early_termination=p['early'], **check))
    for p in packets:
        if p['kind'] != 2: continue
        parent = solvers.get(p['parent_solver_id'])
        if parent is None: raise ValueError('applied contact has no captured solver parent')
        if p['control_time_s'] != parent['control_time_s'] or p['substep'] != parent['substep']:
            raise ValueError('applied contact/solver context mismatch')
        offset, dim = p['offset'], p['dim']
        if offset < 0 or offset+dim > parent['n']: raise ValueError('invalid applied vector offset')
        matched = parent['after_x'][offset:offset+dim]
        difference = float(np.max(abs(p['lambda']-matched), initial=0))
        if difference > 1e-12: raise ValueError('applied impulse differs from parent solver solution')
        if not p['dt_s'] > 0: raise ValueError('nonpositive impulse duration')
        impulse = p['lambda']; normal = p['normal']
        expected_force = normal*impulse[0]/p['dt_s']
        if dim == 3:
            basis = p['basis'].reshape(3, 2, order='F')
            expected_force = expected_force+basis @ impulse[1:]/p['dt_s']
            local_limits = np.array([p['mu1'], p['mu2']])*abs(impulse[0])
            local_violation = np.maximum(0., abs(impulse[1:])-local_limits)
            coeff = parent['before_hi'][offset+1:offset+3]
            index_matches = np.array_equal(parent['before_findex'][offset+1:offset+3], [offset, offset])
            if not index_matches: raise ValueError('contact tangent rows lack matching parent normal index')
            if not np.allclose(coeff, [p['mu1'], p['mu2']], atol=1e-14, rtol=0):
                raise ValueError('contact friction coefficients differ from original LCP rows')
        else:
            basis = None; local_limits = np.empty(0); local_violation = np.empty(0)
        force_error = float(np.max(abs(expected_force-p['force'])))
        if force_error > 1e-9: raise ValueError('applied impulse/recorded force conversion mismatch')
        tolerance = 1e-10*max(1., float(np.max(abs(impulse))))
        report['applied_contacts'].append(dict(id=p['id'], parent_solver_id=parent['id'],
            parent_solver=parent['solver'], parent_returned_success=parent['success'],
            control_time_s=p['control_time_s'], substep=p['substep'], offset=offset, dim=dim,
            solution_buffer_code=p['buffer'], dt_s=p['dt_s'], impulse=impulse,
            point=p['point'], normal=normal, force=p['force'], depth=p['depth'],
            mu1=p['mu1'], mu2=p['mu2'], motion=p['motion'], basis=basis,
            final_normal_dependent_tangent_limits=local_limits,
            final_normal_dependent_tangent_violation=local_violation,
            exceeds_numerical_bound_tolerance=bool(np.any(local_violation > tolerance)),
            numerical_bound_detection_tolerance=tolerance,
            parent_solution_difference=difference, impulse_to_force_max_error_N=force_error,
            force_normal_projection_N=float(p['force'] @ normal),
            force_tangent_norm_N=float(np.linalg.norm(p['force']-normal*(p['force'] @ normal)))))
    applied = report['applied_contacts']
    report['checks'] = dict(solver_call_count=len(solvers), applied_contact_count=len(applied),
        dantzig_count=sum(p['kind']==0 for p in solvers.values()), pgs_count=sum(p['kind']==1 for p in solvers.values()),
        applied_contact_with_final_bound_violation_count=sum(p['exceeds_numerical_bound_tolerance'] for p in applied),
        maximum_applied_parent_solution_difference=max((p['parent_solution_difference'] for p in applied), default=0),
        maximum_impulse_to_force_difference_N=max((p['impulse_to_force_max_error_N'] for p in applied), default=0),
        final_dependent_bound_scope='Individual tangent component boxes, not a circular Coulomb norm.',
        success_scope='Recorded return flag; applied contact packets establish actual use, including finite failed fallback.')
    return clean(report), arrays


def match_contacts(report, trace):
    """Associate captured point/body1 force with copied bridge rows, sign-aware.

    Multiple coincident rows are retained as candidates. For nonzero force,
    a family is identified only when all eligible rows have that same family.
    """
    output = []
    for contact in report['applied_contacts']:
        ix = np.flatnonzero((abs(trace['control_start_time_s']-contact['control_time_s']) <= 1e-8)
                            & (trace['substep_index'] == contact['substep']))
        if len(ix) != 1:
            raise ValueError('capture context does not identify one copied substep')
        i = int(ix[0]); lo, hi = trace['contact_offsets'][i:i+2]
        raw = trace['contact_rows'][lo:hi]
        point, force, normal = (np.asarray(contact[key], float) for key in ('point', 'force', 'normal'))
        distance = np.linalg.norm(raw[:, 1:4]-point, axis=1)
        positive_error = np.linalg.norm(raw[:, 7:10]-force, axis=1)
        negative_error = np.linalg.norm(raw[:, 7:10]+force, axis=1)
        force_error = np.minimum(positive_error, negative_error)
        force_tolerance = 1e-9*max(1., float(np.linalg.norm(force)))
        candidate = np.flatnonzero((distance <= 1e-10) & (force_error <= force_tolerance))
        tags = raw[candidate, 0].astype(int)
        families = np.unique(TAG_FAMILY[tags])
        nonzero = bool(np.linalg.norm(force) > 1e-8)
        signs = np.where(positive_error[candidate] <= negative_error[candidate], 1., -1.)
        normal_error = np.linalg.norm(raw[candidate, 4:7]-signs[:, None]*normal, axis=1)
        output.append(dict(contact_id=contact['id'], control_start_time_s=contact['control_time_s'],
            control_end_time_s=float(trace['control_end_time_s'][i]), substep=contact['substep'],
            nonzero_force=nonzero, family_unambiguous=bool(len(families)==1),
            candidate_row_indices=(candidate+lo).tolist(), candidate_tags=tags.tolist(),
            candidate_families=families.tolist(), candidate_force_sign=signs.tolist(),
            candidate_admitted=raw[candidate, 12].tolist(),
            point_error_m=distance[candidate].tolist(), force_error_N=force_error[candidate].tolist(),
            normal_error=normal_error.tolist(), force_match_tolerance_N=force_tolerance,
            nonzero_association_valid=bool(not nonzero or len(families)==1)))
    return dict(matches=output, nonzero_contact_count=sum(x['nonzero_force'] for x in output),
                nonzero_association_failure_count=sum(not x['nonzero_association_valid'] for x in output),
                matched_contact_count=sum(bool(x['candidate_row_indices']) for x in output),
                scope='Point tolerance1e-10m; force sign accounts for body order; zero-force coincident families may be ambiguous.')


def main():
    p = argparse.ArgumentParser(); p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--reader', type=Path, default=Path(__file__).parent.parent/'instrumentation/read_probe.py')
    p.add_argument('--trace', type=Path, required=True)
    p.add_argument('--historical-trace', type=Path)
    p.add_argument('--output', type=Path, required=True); args = p.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    spec = importlib.util.spec_from_file_location('i69_reader', args.reader)
    reader = importlib.util.module_from_spec(spec); spec.loader.exec_module(reader)
    report, arrays = analyse(reader.read_packets(args.capture))
    for label, path in (('new_trace_matches', args.trace), ('historical_trace_matches', args.historical_trace)):
        if path:
            with np.load(path, allow_pickle=False) as packet:
                trace = {key: packet[key] for key in packet.files}
            report[label] = match_contacts(report, trace)
            report[label]['trace_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
    args.output.mkdir(parents=True)
    report['capture_sha256'] = hashlib.sha256(args.capture.read_bytes()).hexdigest()
    report['reader_sha256'] = hashlib.sha256(args.reader.read_bytes()).hexdigest()
    np.savez_compressed(args.output/'checks.npz', **arrays)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, sort_keys=True, allow_nan=False)+'\n')
    manifest = {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in args.output.iterdir()}
    (args.output/'artifact_manifest.json').write_text(json.dumps(manifest, indent=2, sort_keys=True)+'\n')
    print(json.dumps(report['checks'], allow_nan=False))


if __name__ == '__main__': main()
