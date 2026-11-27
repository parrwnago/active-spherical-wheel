"""Dispatch/boundary contracts; fake P allocator, no model construction or physics."""
from pathlib import Path
from importlib.util import spec_from_file_location,module_from_spec
from types import ModuleType,SimpleNamespace
import sys
import pytest
HERE=Path(__file__).resolve().parent


@pytest.fixture
def adapter(monkeypatch):
 fake=ModuleType('spherical_wheel.balance_priority_control')
 class FakePriority:
  def __init__(self,settings):self.settings=settings
 fake.BalancePriorityAllocator=FakePriority
 monkeypatch.setitem(sys.modules,fake.__name__,fake)
 spec=spec_from_file_location('_i58_adapter_contract',HERE/'adapter.py');module=module_from_spec(spec);spec.loader.exec_module(module)
 return module,FakePriority


def test_create_only_replaces_allocator_and_keeps_exact_parsed_settings(adapter,monkeypatch):
 module,klass=adapter;settings=object();model=object();controller=object();declaration=object();calls=[]
 old=dict(model=model,allocator=SimpleNamespace(settings=settings),settings=object(),slip_scale=object())
 original=dict(old)
 def create(c,s):calls.append((c,s));return old
 monkeypatch.setattr(module.frozen,'create',create)
 out=module.create(controller,declaration)
 assert calls==[(controller,declaration)]and out is old
 assert type(out['allocator'])is klass and out['allocator'].settings is settings
 assert all(out[k]is v for k,v in original.items()if k!='allocator')


def test_apply_delegates_same_inputs_to_original_frozen_extraction(adapter,monkeypatch):
 module,_=adapter;args=tuple(object()for _ in range(5));result=object();calls=[]
 def apply(*received):calls.append(received);return result
 monkeypatch.setattr(module.frozen,'apply',apply)
 assert module.apply(*args)is result
 assert len(calls)==1 and all(a is b for a,b in zip(calls[0],args))


def test_invalid_original_settings_are_not_bypassed(adapter,monkeypatch):
 module,_=adapter
 def fail(*args):raise ValueError('original settings rejected')
 monkeypatch.setattr(module.frozen,'create',fail)
 with pytest.raises(ValueError,match='original settings rejected'):module.create(object(),object())
