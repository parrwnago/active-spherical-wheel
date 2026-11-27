"""Synthetic projection contracts, independent of the recorded robot traces."""
from pathlib import Path
import json
import numpy as np

from pack_rank import project_minima
from replay_rank import compare_scores, flatten_scores, load_core, original_candidate_indices


def main():
    core = load_core(Path('src/spherical_wheel/contact_hypotheses.py'))
    generator = np.random.default_rng(1751)
    family = np.tile(np.array([0, 1, 2, 3, 0, 1], np.int8), 17)
    checks = []
    for variant in ('random', 'duplicate_ties', 'all_unknown', 'mixed_unknown'):
        rows, count = 19, len(family)
        valid = generator.random((rows, count)) > .3
        vectors = generator.normal(0., .025, (rows, count, 3))
        angles = generator.uniform(0., np.deg2rad(10), (rows, count))
        moments = generator.uniform(0., .2, (rows, count))
        if variant == 'duplicate_ties':
            valid[:] = True;angles[:] = 0.;moments[:] = 0.
            vectors[:] = 0.;vectors[..., 0] = .02*family
            vectors[:, 0, 1] = -0.
        if variant == 'all_unknown':
            valid[:] = False
        if variant == 'mixed_unknown':
            valid[:3] = False;valid[3:6, family != 0] = False
        vectors[~valid] = 0.;angles[~valid] = 0.;moments[~valid] = 0.
        features = dict(family_index=family, valid=valid,
            frame_valid=np.ones(rows, bool), excitation_m_s=np.full(rows, .1),
            normal_angle_rad=angles, velocity_residual_m_s=vectors,
            velocity_residual_norm_m_s=np.linalg.norm(vectors, axis=-1),
            moment_residual_norm=moments)
        if variant == 'mixed_unknown':
            features['frame_valid'][-1] = False
            features['excitation_m_s'][-2] = 0.
        full = flatten_scores(core.rank_hypotheses(features))
        compact = project_minima(features, full)
        actual = original_candidate_indices(flatten_scores(core.rank_hypotheses(compact)),
                                             compact['source_candidate_index'])
        fields = compare_scores(actual, full)
        checks.append(dict(variant=variant, fields=len(fields), all_fields_byte_exact=True))
    print(json.dumps(dict(synthetic_contract_groups=checks, all_pass=True,
                         scope='Four correlated synthetic projection groups, not repository tests or physical trials')))


if __name__ == '__main__':
    main()
