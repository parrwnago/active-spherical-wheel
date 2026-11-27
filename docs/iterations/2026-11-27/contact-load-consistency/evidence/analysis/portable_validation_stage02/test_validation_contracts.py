"""Validation-adapter tests; no canonical experiment data or decisions change."""
import copy
import importlib.util
from pathlib import Path
import pytest

path=Path(__file__).with_name('replay_evaluation.py')
spec=importlib.util.spec_from_file_location('_i61_validation_adapter',path)
R=importlib.util.module_from_spec(spec);spec.loader.exec_module(R)


def example():
    return dict(aggregate_primary={'count':1},gates=[{'passed':False}],
        cases={'m100_B':dict(windows={'primary':{'error_fraction':.1}},fixed_events={
            'earliest_accepted_wrong':{'score_margin':.2,'distances_to_equivalent_reference_mm':{'N':1.,'A':2.,'C':3.}}})})


def test_exact_report_remains_exact():
    data=example();result=R.verify_science(data,copy.deepcopy(data));assert result['full_science_exact']


def test_only_tiny_fixed_event_distance_is_allowed_and_preserved():
    expected=example();actual=copy.deepcopy(expected)
    actual['cases']['m100_B']['fixed_events']['earliest_accepted_wrong']['distances_to_equivalent_reference_mm']['N']+=5e-13
    saved=copy.deepcopy(actual);result=R.verify_science(actual,expected)
    assert not result['full_science_exact'] and result['primary_and_fixed_window_metrics_and_gates_exact']
    assert actual==saved and len(result['event_scalar_roundoff'])==1


def test_primary_metric_difference_is_rejected():
    expected=example();actual=copy.deepcopy(expected);actual['aggregate_primary']['count']=2
    with pytest.raises(ValueError):R.verify_science(actual,expected)


def test_fixed_window_metric_roundoff_is_not_exempted():
    expected=example();actual=copy.deepcopy(expected);actual['cases']['m100_B']['windows']['primary']['error_fraction']+=1e-15
    with pytest.raises(ValueError):R.verify_science(actual,expected)


def test_event_non_distance_field_difference_is_rejected():
    expected=example();actual=copy.deepcopy(expected);actual['cases']['m100_B']['fixed_events']['earliest_accepted_wrong']['score_margin']+=1e-15
    with pytest.raises(ValueError):R.verify_science(actual,expected)


def test_event_distance_above_declared_tolerance_is_rejected():
    expected=example();actual=copy.deepcopy(expected)
    actual['cases']['m100_B']['fixed_events']['earliest_accepted_wrong']['distances_to_equivalent_reference_mm']['N']+=1e-9
    with pytest.raises(ValueError):R.verify_science(actual,expected)


def test_unlisted_event_name_is_not_exempted():
    expected=example();expected['cases']['m100_B']['fixed_events']['unlisted']=copy.deepcopy(expected['cases']['m100_B']['fixed_events']['earliest_accepted_wrong'])
    actual=copy.deepcopy(expected);actual['cases']['m100_B']['fixed_events']['unlisted']['distances_to_equivalent_reference_mm']['N']+=1e-15
    with pytest.raises(ValueError):R.verify_science(actual,expected)
