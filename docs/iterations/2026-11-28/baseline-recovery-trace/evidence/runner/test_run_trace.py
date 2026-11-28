"""I86 capture contracts; no physics, controller design or rollouts."""
from pathlib import Path
from types import SimpleNamespace
import importlib.util
import json

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('i86_runner_contract_source', HERE/'run_trace.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def fake_controller(time=105.):
    data = SimpleNamespace(qpos=np.arange(10, dtype=float), qvel=np.arange(9, dtype=float), time=time)
    private = SimpleNamespace(qpos=np.arange(10, dtype=float), qvel=np.arange(9, dtype=float),
                              time=0., xpos=np.zeros((2, 3)), xmat=np.zeros((2, 9)))
    observer = SimpleNamespace(generalized=np.arange(9.), interval_mean=np.arange(9.)+.5,
        _previous=(104.995, np.eye(9), np.arange(9.), np.zeros(9)), update=lambda *a, **kw: None)
    sync = SimpleNamespace(_data=private, _plane=(np.zeros(3), np.array([0., 0., 1.])),
                           preview=lambda *a, **kw: None)
    return SimpleNamespace(plant=SimpleNamespace(data=data, last_input=np.arange(5.)),
        nominal_joints=np.arange(3.), _vref=np.arange(9.), _held={'gain': np.eye(3)},
        _plane=(np.zeros(3), np.array([0., 0., 1.])), _sync={'commands': np.arange(5.)},
        _slow_normal=np.array([0., 0., 1.]), _yaw_error=None,
        observer=observer, estimator=SimpleNamespace(normal=np.array([0., 0., 1.]), loaded=True),
        synchronizer=sync)


def test_exactly_once_capture_returns_native_objects_and_copies_state():
    controller = fake_controller()
    applied, info = np.arange(5.), dict(requested=np.arange(5.)+1., optional=np.nan)
    def native(c, *args):
        c.observer.update()
        c.synchronizer.preview()
        return applied, info
    rows, counts = [], {}
    returned = m.captured_command(native, rows, counts)(controller, np.zeros(2), np.zeros(2), np.zeros(2))
    assert returned[0] is applied and returned[1] is info
    assert counts == dict(commands=1, previews=1, observer_updates=1)
    assert len(rows) == 1 and rows[0]['capture_state_guard_passed']
    rows[0]['qpos'][0] = 999.
    assert controller.plant.data.qpos[0] == 0.
    assert rows[0]['info']['optional'] is None
    json.dumps(rows, allow_nan=False)


@pytest.mark.parametrize('duplicate', ['observer', 'preview'])
def test_rejects_duplicate_native_update_or_preview(duplicate):
    def native(c, *args):
        c.observer.update()
        c.synchronizer.preview()
        if duplicate == 'observer':
            c.observer.update()
        else:
            c.synchronizer.preview()
        return np.zeros(5), dict(requested=np.zeros(5))
    with pytest.raises(AssertionError, match='Exactly one'):
        m.captured_command(native, [], {})(fake_controller(), np.zeros(2), np.zeros(2), np.zeros(2))


def test_window_is_closed_for_commands_and_has_261_rows():
    assert m.in_capture_window(105.) and m.in_capture_window(106.3)
    assert not m.in_capture_window(104.995) and not m.in_capture_window(106.305)
    assert sum(m.in_capture_window(i*.005) for i in range(21301)) == 261
    assert m.SETTINGS['expected_substeps'] == 1300
    assert m.SETTINGS['expected_holds'] == 260
    assert m.SETTINGS['design_belt_cap_m_s'] == .3 and m.SETTINGS['operating_belt_cap_m_s'] == .6


def test_settings_rejects_silent_horizon_or_cap_change():
    m.validate_settings(dict(iteration='I-86', runner_settings=m.SETTINGS.copy()))
    for changed in [dict(m.SETTINGS, physical_stop_s=106.), dict(m.SETTINGS, operating_belt_cap_m_s=.3)]:
        with pytest.raises(ValueError):
            m.validate_settings(dict(iteration='I-86', runner_settings=changed))


def test_prefix_allows_only_terminal_unheld_flag_and_detects_signed_zero(tmp_path):
    n = m.SETTINGS['expected_precision_rows']
    old = dict(time_s=np.arange(n+1)*.005, command_held_after_row=np.ones(n+1, bool),
               **{f'signal{i}': np.zeros((n+1, 1)) for i in range(77)})
    new = {k: v[:n].copy() for k, v in old.items()}
    new['command_held_after_row'][-1] = False
    np.savez_compressed(tmp_path/'old.npz', **old)
    np.savez_compressed(tmp_path/'new.npz', **new)
    assert m.verify_prefix(tmp_path/'old.npz', tmp_path/'new.npz')['passed']
    new['signal0'][0, 0] = -0.
    np.savez_compressed(tmp_path/'new.npz', **new)
    assert not m.verify_prefix(tmp_path/'old.npz', tmp_path/'new.npz')['passed']
