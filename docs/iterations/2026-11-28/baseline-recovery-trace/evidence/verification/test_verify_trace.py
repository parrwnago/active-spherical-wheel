"""Focused contracts for distinctions used by the I86 diagnostic."""
import importlib.util
from pathlib import Path
import numpy as np

spec = importlib.util.spec_from_file_location('i86_independent_trace', Path(__file__).with_name('verify_trace.py'))
v = importlib.util.module_from_spec(spec); spec.loader.exec_module(v)


def contacts(items):
    result = np.zeros((len(items), 16))
    for i, (tag, x, pressure) in enumerate(items):
        result[i, 0] = tag; result[i, 1] = x
        result[i, 6] = 1; result[i, 9] = pressure; result[i, 12] = 1
    return result


def test_solver_manifold_points_do_not_become_separate_spatial_supports():
    s = v.contact_summary(contacts([(0, 0., 8.), (0, .003, 2.)]))
    assert s['loaded_solver_points'] == 2
    assert len(s['spatial_groups']) == 1
    assert s['group_counts_at_share'] == [1, 1, 1]


def test_two_separated_points_on_same_shell_remain_visible():
    s = v.contact_summary(contacts([(0, -.04, 5.), (0, .04, 5.)]))
    assert len(s['spatial_groups']) == 2
    assert s['family_counts_at_share'] == [1, 1, 1]
    assert s['second_spatial_group_share'] == .5
    assert s['same_family_spatial_group_shares'][0] == [.5, .5]


def test_adjacent_different_drives_have_load_shares_without_forced_spatial_split():
    s = v.contact_summary(contacts([(0, 0., 6.), (2, .001, 4.)]))
    assert len(s['spatial_groups']) == 1
    assert s['family_counts_at_share'] == [2, 2, 2]
    assert s['second_family_share'] == .4


def test_unloaded_or_rejected_contact_does_not_create_loaded_support():
    c = contacts([(0, -.04, 10.), (0, .04, 0.), (2, .04, 9.)])
    c[2, 12] = 0
    s = v.contact_summary(c)
    assert s['raw_solver_points'] == 3 and s['loaded_solver_points'] == 1
    assert s['total_pressure_N'] == 10.
    np.testing.assert_allclose(s['signed_force_N'], [0., 0., 19.])


def test_sensitivity_reports_small_secondary_load_without_retuning():
    s = v.contact_summary(contacts([(0, -.04, 96.), (2, .04, 4.)]))
    assert s['group_counts_at_share'] == [2, 1, 1]
    assert s['family_counts_at_share'] == [2, 1, 1]
    assert s['second_spatial_group_share'] == .04


def test_empty_contact_export_has_finite_zero_loaded_quantities():
    s = v.contact_summary(np.empty((0, 16)))
    assert s['total_pressure_N'] == 0. and s['spatial_groups'] == []
    assert s['group_counts_at_share'] == [0, 0, 0]
    np.testing.assert_array_equal(s['family_shares'], np.zeros(4))
