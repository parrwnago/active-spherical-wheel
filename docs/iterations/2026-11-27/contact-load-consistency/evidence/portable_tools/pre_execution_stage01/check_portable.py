"""Synthetic lossless-encoding and same-family policy replay contracts."""
from pathlib import Path
import json
import numpy as np

from pack_gate import encode_inputs, signature
from replay_gate import compare_packet, decode_inputs, derive_packet, load_core


def main():
    core = load_core(Path('src/spherical_wheel/contact_load_consistency.py'))
    random = np.random.default_rng(1161);rows = 11
    gamma = random.normal(0., 1., (rows, 9));gamma[:, 2] += 20.
    native_point = random.normal(0., .1, (rows, 3))
    proposed_point = native_point.copy();proposed_point[:, 0] += .003
    inputs = dict(time_s=np.arange(rows)*.005, native_family=np.zeros(rows, np.int64),
        proposed_family=np.zeros(rows, np.int64), i60_accepted=np.ones(rows, bool),
        native_point_m=native_point, proposed_point_m=proposed_point,
        native_alternative_point_m=proposed_point.copy(),
        proposed_candidate_index=np.zeros(rows, np.int64), native_candidate_index=np.zeros(rows, np.int64),
        observer_generalized=gamma, observer_wrench=gamma[:, :6].copy(),
        observer_interval_mean=gamma+random.normal(0., .01, gamma.shape))
    for prefix in ('proposed', 'native_alternative'):
        inputs[prefix+'__valid'] = np.ones(rows, bool)
        inputs[prefix+'__prediction'] = gamma.copy()
        inputs[prefix+'__force_norm_N'] = np.linalg.norm(gamma[:, :3], axis=1)
        inputs[prefix+'__radius_m'] = np.full(rows, .12)
        inputs[prefix+'__point_geometry_valid'] = np.ones(rows, bool)
        inputs[prefix+'__wrench_matches_generalized'] = np.ones(rows, bool)
    inputs['proposed__prediction'][0, -1] -= 1.
    inputs['proposed__valid'][-1] = False;inputs['proposed__prediction'][-1] = np.nan
    inputs['native_alternative__valid'][-2] = False
    inputs['native_alternative__prediction'][-2] = np.nan
    inputs['native_point_m'][1, 0] = -0.
    inputs['map__valid'] = np.ones(rows, bool)
    inputs['map__centre_m'] = random.normal(0., .1, (rows, 3))
    inputs['map__joint_at_centre'] = np.zeros((rows, 4, 3))
    inputs['map__joint_moment_map'] = random.normal(0., 1., (rows, 4, 3, 3))
    inputs['map__force_norm_N'] = inputs['proposed__force_norm_N'].copy()
    inputs['map__radius_m'] = inputs['proposed__radius_m'].copy()
    options = dict(absolute_tolerance_Nm=.03, relative_tolerance=.10)
    packet = derive_packet(inputs, core, options)
    # Same family, different point: A changes the point; failed C veto restores N.
    assert packet['A_family'][0] == packet['N_family'][0]
    assert not np.array_equal(packet['A_point_m'][0], packet['N_point_m'][0])
    assert np.array_equal(packet['C_point_m'][0], packet['N_point_m'][0])
    assert not packet['C_uses_proposal'][0]
    expected = {key: signature(value) for key, value in packet.items()}
    encoded, recipes = encode_inputs(packet)
    decoded = decode_inputs(encoded, recipes)
    fields = compare_packet(derive_packet(decoded, core, options), expected)
    print(json.dumps(dict(full_packet_fields=len(fields), all_fields_byte_exact=True,
        same_family_point_policy_preserved=True, invalid_prediction_nan_preserved=True,
        signed_zero_preserved=True, scope='Synthetic compression/replay contracts; no historical outcomes or physical trials')))


if __name__ == '__main__':
    main()
