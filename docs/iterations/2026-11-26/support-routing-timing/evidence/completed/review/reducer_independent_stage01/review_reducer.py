"""Independent adversarial synthetic checks of the preserved I46 reducer."""
from pathlib import Path
import copy
import hashlib
import json
import types

import numpy as np


STAGE = Path(__file__).resolve().parent
FAMILY = STAGE.parents[1]
ROOT = FAMILY.parents[1]


def main():
    declaration = json.loads((STAGE/'declaration_before_checks.json').read_text())
    source = STAGE/'source/reduce_routing.py'
    module = types.ModuleType('frozen_independent_I46_reducer')
    module.__file__ = str(ROOT/declaration['sources'][0]['origin'])
    exec(compile(source.read_text(), str(source), 'exec'), module.__dict__)
    checks = []
    def record(name, condition, evidence):
        checks.append(dict(name=name, passed=bool(condition), evidence=evidence))

    truth = [dict(time_s=1.1, direction='shell_to_belt')]
    estimates = [dict(time_s=1.05, direction='shell_to_belt'), dict(time_s=1.15, direction='shell_to_belt')]
    actual = module.match_events(truth, estimates)
    record('earlier_on_symmetric_saved_grid_offset_tie', actual['matches'][0]['estimate_index'] == 0,
           dict(result=actual, absolute_offsets_s=[abs(e['time_s']-1.1) for e in estimates], expected_index=0))

    spec = dict(stable_dwell_s=.015, max_gap_between_stable_runs_s=.05, pair_window_s=[-.1, .15])
    t = np.arange(201)*.005
    true = np.where(t < .4, 0, np.where(t < .6, 1, 0))
    estimated = np.where(t < .41, 0, np.where(t < .655, 1, 0))
    loads = np.zeros((len(t), 4)); loads[np.arange(len(t)), np.where(true == 0, 0, 2)] = 10.
    est = np.zeros_like(loads); est[np.arange(len(t)), np.where(estimated == 0, 0, 2)] = 10.
    metrics = module.truth_metrics(t, loads, np.ones(len(t), bool), np.zeros(len(t)),
        dict(native=est, fast=est), [.405, .65], 20., spec)
    m = metrics['sources']['native']
    record('full_context_avoids_false_extra_from_outside_window_truth', not m['extra_estimated_events'], m['extra_estimated_events'])
    record('estimated_event_outside_scoring_window_can_match_inside_truth',
        len(m['event_matches']) == 1 and abs(m['event_matches'][0]['estimated_time_s']-.655) < 1e-12,
        m['event_matches'])
    labels = np.r_[np.zeros(10, int), np.full(20, -1), np.ones(10, int)]
    censored = module.stable_family_events(np.arange(40)*.005, labels)
    record('long_unknown_gap_censors_instead_of_fabricating_handoff',
        not censored['events'] and len(censored['censored_starts']) == 2, censored)

    t6 = np.arange(6)*.005
    loads = np.tile([10., 0., 0., 0.], (6, 1))
    est = loads.copy(); est[1] = 0.; est[2] = [5., 5., 0., 0.]
    body = np.zeros(6); body[3] = 1.01
    metrics = module.truth_metrics(t6, loads, np.ones(6, bool), body,
        dict(native=est, fast=est), [0., .03], 20., spec)
    m = metrics['sources']['native']
    record('unknown_and_tie_wrong_body_excluded_terminal_unactuated',
        metrics['primary_loaded_samples'] == 4 and m['wrong_patch_samples'] == 2
        and m['missing_estimate_samples'] == 1 and m['tie_estimate_samples'] == 1,
        dict(primary=metrics['primary_loaded_samples'], wrong=m['wrong_patch_samples'], missing=m['missing_estimate_samples'], tie=m['tie_estimate_samples']))
    record('invalid_gaps_not_bridged_by_share_variation', m['variation_edges'] == 0, m['variation_edges'])

    commands = np.zeros((6, 5)); commands[1:, 3] = .3
    raw = dict(input_bounds=np.tile([2., 1., 1., .3, .3], (6, 1)),
        native_pre_sync_command=commands, shadow_pre_sync_command=commands,
        native_pre_sync_clipped_command=commands, shadow_pre_sync_clipped_command=commands,
        shadow_pre_sync_command_delta=np.zeros_like(commands))
    cm = module.command_metrics(t6, raw, [0., .03])
    record('belt_steps_normalized_by_speed_bound', cm['native']['requested']['normalized_adjacent_step']['maximum'] == 1., cm)

    protocol = json.loads((STAGE/'source/protocol.json').read_text())
    gate_spec = protocol['registered_gate']
    directions = dict(shell_to_belt=1, belt_to_shell=1)
    case = dict(completed=True, window_complete=True, exposure=dict(guard_passed=True),
        truth_tables=dict(preceding_mean=dict(scored_truth_event_direction_counts=directions,
            common_event_direction_counts=directions, common_paired_events=[
                dict(truth_index=0, direction='shell_to_belt', absolute_timing_improvement_s=.01),
                dict(truth_index=1, direction='belt_to_shell', absolute_timing_improvement_s=.01)],
            sources=dict(native=dict(routing_mismatch_equivalent_s=1., extra_estimated_events=[]),
                         fast=dict(routing_mismatch_equivalent_s=.7, extra_estimated_events=[])))),
        commands=dict(native=dict(requested=dict(normalized_adjacent_step=dict(maximum=1.))),
                      fast=dict(requested=dict(normalized_adjacent_step=dict(maximum=1.)))))
    cases = {k: copy.deepcopy(case) for k in ('A', 'B')}
    noise = {k: dict(sources=dict(native=dict(label_flip_fraction=.1), fast=dict(label_flip_fraction=.1))) for k in cases}
    gate = module.registered_gate(cases, noise, ['A', 'B'], gate_spec)
    record('all_registered_gate_prerequisites_can_pass_synthetic_fixture', gate['passes_all_conditions'], gate['checks'])
    cases['A']['commands']['fast']['requested']['normalized_adjacent_step']['maximum'] = 1.06
    cases['B']['commands']['fast']['requested']['normalized_adjacent_step']['maximum'] = .5
    noise['A']['sources']['fast']['label_flip_fraction'] = .16
    noise['B']['sources']['fast']['label_flip_fraction'] = 0.
    gate = module.registered_gate(cases, noise, ['A', 'B'], gate_spec)
    record('per_case_command_noise_failure_not_hidden_by_other_case_gain',
        not gate['checks']['each_case_command_and_noise_nonregression'], gate['case_nonregression'])
    cases = {k: copy.deepcopy(case) for k in ('A', 'B')}
    cases['A']['truth_tables']['preceding_mean']['sources']['native']['routing_mismatch_equivalent_s'] = 0.
    cases['A']['truth_tables']['preceding_mean']['sources']['fast']['routing_mismatch_equivalent_s'] = 0.
    gate = module.registered_gate(cases, noise, ['A', 'B'], gate_spec)
    record('zero_baseline_mismatch_is_no_demonstrated_benefit',
        gate['case_benefit']['A']['relative_reduction'] is None and not gate['checks']['each_case_wrong_duration_benefit'],
        gate['case_benefit']['A'])

    manual = [
        dict(topic='historical_truth', passed=True, finding='Historical adapter constructs only last1ms truth from original saved records; does not invent preceding-mean truth.'),
        dict(topic='effective_held_support', passed=True, finding='Primary sources use native_feedback_effective_loads_N and shadow_used_feedback_effective_loads_N; raw locator loads are separate.'),
        dict(topic='shared_draw_noise_and_geometry', passed=True, finding='One perturbation matrix per saved pose reused in both sources; same current qpos and locator, logged baseline loads parity asserted; exact model/asset hashes retained. No physics/observer/controller updates.'),
        dict(topic='noise_scope', passed=True, finding='Static paired local wrench perturbation assesses raw locator sensitivity, explicitly distinct from noisy measurement propagation through filters.'),
    ]
    result = dict(passed=all(c['passed'] for c in checks), synthetic_checks=checks, manual_review=manual,
        scientific_blockers=[c for c in checks if not c['passed']],
        scope='No raw campaign reduction or new physics; frozen-source review only.')
    (STAGE/'report.json').write_text(json.dumps(module.safe(result), indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(passed=result['passed'], blockers=[c['name'] for c in result['scientific_blockers']])))


if __name__ == '__main__':
    main()
