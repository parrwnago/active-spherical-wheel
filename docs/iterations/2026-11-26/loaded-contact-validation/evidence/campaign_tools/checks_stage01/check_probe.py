"""Meaningful pure replay-contract checks; no Gazebo/model/controller imports."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest
p=Path(__file__).with_name('run_probe.py')
spec=importlib.util.spec_from_file_location('_pure_i57_probe',p)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def raw():
    a=np.arange(10*5,dtype=np.float64).reshape(10,5)/100
    return dict(exact_applied=a,exact_native_applied=-a)


def test_bytes_compare_signed_zero_not_only_numeric_equality():
    assert np.array_equal(np.array([0.]),np.array([-0.]))
    assert not m.bit_exact(np.array([0.]),np.array([-0.]))
    assert m.bit_exact(np.array([1.]),np.array([1.]))
    assert not m.bit_exact(np.array([1.]),np.array([1.],np.float32))


def test_continuation_replays_four_distinct_original_holds():
    _,sequence=m.center_and_sequence(dict(mode='continue_saved'),dict(index=1),raw())
    assert m.bit_exact(sequence,raw()['exact_applied'][1:5])
    assert not np.array_equal(sequence[0],sequence[1])


def test_hold_governed_does_not_continue_changing_native_policy():
    source=raw();center,sequence=m.center_and_sequence(dict(mode='hold_governed'),dict(index=1),source)
    assert np.array_equal(center,source['exact_applied'][1])
    assert np.array_equal(sequence,np.tile(center,(4,1)))


def test_hold_native_uses_native_at_same_governed_anchor():
    source=raw();center,sequence=m.center_and_sequence(dict(mode='hold_native'),dict(index=1),source)
    assert np.array_equal(center,source['exact_native_applied'][1])
    assert np.array_equal(sequence,np.tile(center,(4,1)))


def test_midpoint_is_arithmetic_of_same_anchor_bounded_centers():
    center,sequence=m.center_and_sequence(dict(mode='hold_midpoint'),dict(index=1),raw())
    assert np.array_equal(center,np.zeros(5))
    assert np.array_equal(sequence,np.zeros((4,5)))


def test_perturbation_is_exact_declared_offset_and_held():
    source=raw();delta=np.array([.05,0,0,0,0])
    center,sequence=m.center_and_sequence(dict(mode='hold_governed',delta_input=delta),dict(index=1),source)
    assert np.array_equal(sequence,np.tile(center+delta,(4,1)))
    assert np.array_equal(source['exact_applied'],raw()['exact_applied'])


def test_out_of_bounds_probe_is_rejected_without_silent_clipping():
    with pytest.raises(ValueError,match='clipping'):
        m.center_and_sequence(dict(mode='hold_governed',delta_input=[2,0,0,0,0]),dict(index=1),raw())


def test_numpy_scalar_and_zero_dimension_clock_have_identical_bytes():
    assert m.bit_exact(np.asarray(.005,dtype=np.float64),np.float64(.005))
    assert not m.bit_exact(np.asarray(.005,dtype=np.float64),np.asarray([.005]))
