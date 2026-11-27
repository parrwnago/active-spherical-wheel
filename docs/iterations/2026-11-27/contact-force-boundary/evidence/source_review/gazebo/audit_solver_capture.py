"""Independent I69 binary decode, native parity and contact-export association.

No physics, solver calls, fitting or controller changes. Does not import the
canonical packet decoder. Original raw solver bytes are the input.
"""
from pathlib import Path
from datetime import datetime, timezone
import argparse
import hashlib
import json
import struct
import numpy as np

TARGETS = (1575, 6422, 7011, 7041)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    with np.load(path, allow_pickle=False) as z:
        return {key: z[key].copy() for key in z.files}


def decode(path):
    content = Path(path).read_bytes(); offset = 0; records = []
    def scalar(pattern):
        nonlocal offset
        size = struct.calcsize(pattern)
        if offset+size > len(content):
            raise ValueError('Truncated scalar')
        value = struct.unpack_from(pattern, content, offset); offset += size
        return value
    def array(kind, count):
        nonlocal offset
        size = np.dtype(kind).itemsize*count
        if offset+size > len(content):
            raise ValueError('Truncated array')
        value = np.frombuffer(content, dtype=kind, offset=offset, count=count).copy()
        offset += size
        return value
    while offset < len(content):
        start = offset
        magic, schema, kind, ident, time, step = scalar('<IIIQdi')
        if magic != 0x49363950 or schema != 1 or kind not in (0, 1, 2):
            raise ValueError('Unexpected packet header')
        row = dict(kind=int(kind), id=int(ident), time_s=time, substep=int(step), byte_offset=start)
        if kind in (0, 1):
            n, stride, nub, early, success = scalar('<iiiii')
            if not 1 <= n <= 4096 or stride != (n if n <= 1 else 4*((n+3)//4)):
                raise ValueError('Unexpected solver shape')
            row.update(n=n, stride=stride, nub=nub, early=bool(early), success=bool(success))
            for side in ('in', 'out'):
                for field, length in (('A', n*stride), ('x', n), ('b', n), ('lo', n), ('hi', n)):
                    row[side+'_'+field] = array('<f8', length)
                row[side+'_findex'] = array('<i4', n)
        else:
            parent, dim, index, buffer, dt, mu1, mu2 = scalar('<Qiiiddd')
            if dim not in (1, 3):
                raise ValueError('Unexpected contact dimension')
            row.update(parent=int(parent), dim=dim, index=index, buffer=buffer, dt=dt, mu1=mu1, mu2=mu2)
            for field, count in (('lambda', dim), ('point', 3), ('normal', 3), ('force', 3), ('depth', 1), ('basis', 6), ('motion', 3)):
                row[field] = array('<f8', count)
        row['byte_end'] = offset
        records.append(row)
    return records, len(content), offset


def raw_parity(new, old):
    keys = sorted(set(new) | set(old)); result = {}
    for key in keys:
        if key not in new or key not in old:
            result[key] = False
        else:
            a, b = new[key], old[key]
            result[key] = bool(a.shape == b.shape and a.dtype == b.dtype and a.tobytes() == b.tobytes())
    return result


def run(campaign, historical, old_trace, output):
    output.mkdir(parents=True, exist_ok=False)
    paths = {'solver': campaign/'solver_packets.bin', 'trace': campaign/'substeps.npz',
        'precision': campaign/'campaign/runs/m100_B/precision.npz',
        'old_precision': historical, 'old_trace': old_trace}
    before = dict(iteration='I-69', utc=datetime.now(timezone.utc).isoformat(),
        source_sha256=sha(__file__), numpy_version=np.__version__,
        inputs={key: dict(path=str(path), sha256=sha(path)) for key, path in paths.items()},
        tolerances=dict(force_reconstruction_N=1e-10, point_join_m=1e-12, normal_join=1e-12,
                        force_export_join_N=1e-10, context_clock_s=1e-9),
        scope='Independent binary decode, byte parity, force reconstruction and raw contact association; no solver execution or new contact scoring.')
    (output/'registered_before_arithmetic.json').write_text(json.dumps(before, indent=2)+'\n')
    precision = raw_parity(load(paths['precision']), load(historical))
    trace = load(paths['trace']); previous = load(old_trace)
    trace_parity = raw_parity(trace, previous)
    records, byte_length, consumed = decode(paths['solver'])
    identifiers = [row['id'] for row in records]
    solvers = {row['id']: row for row in records if row['kind'] < 2}
    contact_rows = []; maximum_force_error = 0.; maximum_lambda_error = 0.
    for row in records:
        if row['kind'] != 2:
            continue
        fixed_end = int(round(row['time_s']/.005))+1
        mask = (abs(trace['control_start_time_s']-row['time_s']) <= 1e-9) & (trace['substep_index'] == row['substep'])
        found = np.flatnonzero(mask)
        if len(found) != 1:
            raise ValueError('Captured contact does not map to one raw physics step')
        index = int(found[0]); lo, hi = trace['contact_offsets'][index:index+2]
        packet = trace['contact_rows'][lo:hi]
        predicted = row['normal']*row['lambda'][0]
        if row['dim'] == 3:
            basis = row['basis'].reshape(3, 2, order='F')
            predicted = predicted+basis[:, 0]*row['lambda'][1]+basis[:, 1]*row['lambda'][2]
        predicted /= row['dt']
        force_error = float(np.max(np.abs(predicted-row['force'])))
        maximum_force_error = max(maximum_force_error, force_error)
        direction = row['normal'].copy()
        if direction @ (trace['bridge_state_before'][index, 2:5]-row['point']) < 0:
            direction *= -1
        point_error = np.linalg.norm(packet[:, 1:4]-row['point'], axis=1)
        normal_error = np.linalg.norm(packet[:, 4:7]-direction, axis=1)
        plus_error = np.linalg.norm(packet[:, 7:10]-row['force'], axis=1)
        minus_error = np.linalg.norm(packet[:, 7:10]+row['force'], axis=1)
        geometry = (point_error <= 1e-12) & (normal_error <= 1e-12)
        plus = geometry & (plus_error <= 1e-10); minus = geometry & (minus_error <= 1e-10)
        matches = plus | minus
        parent = solvers.get(row['parent'])
        lambda_error = None; mapping_valid = False
        if parent is not None and row['index'] >= 0 and row['index']+row['dim'] <= parent['n']:
            # For primary-buffer fallback applies, BoxedLcp copies backup x
            # into primary x; compare the latest solver's solution at that row.
            expected = parent['out_x'][row['index']:row['index']+row['dim']]
            lambda_error = float(np.max(np.abs(expected-row['lambda'])))
            mapping_valid = bool(abs(parent['time_s']-row['time_s']) <= 1e-9 and parent['substep'] == row['substep'])
            maximum_lambda_error = max(maximum_lambda_error, lambda_error)
        contact_rows.append(dict(packet_id=row['id'], parent_solver_id=row['parent'], dimension=row['dim'],
            lambda_offset=row['index'], buffer=row['buffer'], context_end_index=fixed_end,
            substep=row['substep'], trace_index=index, force_reconstruction_max_abs_N=force_error,
            parent_context_and_offset_valid=mapping_valid, parent_lambda_max_abs_gap=lambda_error,
            trace_match_count=int(matches.sum()), matching_tags=sorted(set(packet[matches, 0].astype(int).tolist())),
            positive_force_match_count=int(plus.sum()), negative_force_match_count=int(minus.sum()),
            nonzero_force=bool(np.linalg.norm(row['force']) > 1e-10),
            logged_normal_projection_N=float(row['normal'] @ row['force']),
            point=row['point'].tolist(), normal=row['normal'].tolist(), force_N=row['force'].tolist(),
            impulse=row['lambda'].tolist()))
    contexts = sorted(set((int(round(row['time_s']/.005))+1, row['substep']) for row in records))
    expected_contexts = [(endpoint, substep) for endpoint in TARGETS for substep in range(1, 6)]
    checks = dict(precision_exact59=len(precision) == 59 and all(precision.values()),
        trace_numeric_fields_exact=all(value for key,value in trace_parity.items() if key != 'metadata_json'),
        identical_trace_field_sets=set(trace) == set(previous), raw_rows44000=len(trace['time_s']) == 44000,
        binary_fully_decoded=byte_length == consumed and bool(records),
        unique_increasing_packet_ids=len(set(identifiers)) == len(identifiers) and identifiers == sorted(identifiers),
        exact20_target_contexts=contexts == expected_contexts,
        every_contact_force_reconstructed=bool(contact_rows) and maximum_force_error <= 1e-10,
        every_contact_has_export_match=bool(contact_rows) and all(row['trace_match_count'] > 0 for row in contact_rows),
        every_contact_parent_context_offset_valid=all(row['parent_context_and_offset_valid'] for row in contact_rows),
        every_contact_lambda_matches_parent_solution=all(row['parent_lambda_max_abs_gap'] is not None and row['parent_lambda_max_abs_gap'] <= 1e-12 for row in contact_rows))
    report = dict(iteration='I-69', completed_utc=datetime.now(timezone.utc).isoformat(), checks=checks,
        all_checks_pass=all(checks.values()), precision_fields=precision, trace_fields=trace_parity,
        packets=len(records), solver_packets=len(solvers), contact_packets=len(contact_rows),
        fixed_contexts=[list(x) for x in contexts],
        maximum_force_reconstruction_abs_gap_N=maximum_force_error,
        maximum_applied_lambda_parent_gap=maximum_lambda_error,
        contacts=contact_rows,
        qualifications=['Force sign is checked both ways because original collision order was not exported; match counts preserve ambiguity.',
            'Zero-force coincident collision slots may have several matching tags. They are not silently assigned a unique material family.',
            'This audit validates observed interception/export association, not Coulomb physical validity or a control improvement.'])
    (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(all_checks_pass=report['all_checks_pass'], failed=[key for key,value in checks.items() if not value],
        packets=len(records), solver_packets=len(solvers), contact_packets=len(contact_rows),
        maximum_force_reconstruction_abs_gap_N=maximum_force_error)))
    if not report['all_checks_pass']:
        raise SystemExit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('campaign', 'historical', 'old-trace', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    run(args.campaign.resolve(), args.historical.resolve(), args.old_trace.resolve(), args.output.resolve())
