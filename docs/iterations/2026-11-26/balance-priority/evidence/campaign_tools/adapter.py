"""I58 P adapter: reuse frozen I56 extraction, replace only allocator policy."""
from pathlib import Path
import importlib.util
from spherical_wheel.balance_priority_control import BalancePriorityAllocator

ROOT=Path(__file__).resolve().parents[3]
ORIGINAL=ROOT/'results/constrained_handoff_20261126/campaign_tools/adapter.py'
spec=importlib.util.spec_from_file_location('_i58_frozen_i56_adapter',ORIGINAL)
frozen=importlib.util.module_from_spec(spec)
spec.loader.exec_module(frozen)


def create(controller,settings):
    instance=frozen.create(controller,settings)
    instance['allocator']=BalancePriorityAllocator(instance['allocator'].settings)
    return instance


def apply(instance,controller,native_requested,native_applied,info):
    return frozen.apply(instance,controller,native_requested,native_applied,info)
