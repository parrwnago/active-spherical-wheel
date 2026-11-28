"""I78 result-owned reference selection and diagnostic sidecar.

The only live intervention is the path's coherent p/v/a reference. Nominal
probe commands are never applied. Controller/observer/native histories advance
once per live command, independently of the private current-state probe.
"""
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch
import importlib.util
import sys
import numpy as np

from native_probe import NativeCommandProbe

ROOT=Path(__file__).resolve().parents[3]
BASE=Path(__file__).resolve().parents[1]
GUARDS=('spherical_wheel.offroad_control.measure_contact',
    'spherical_wheel.offroad_control.records_from_mujoco',
    'spherical_wheel.offroad_control.rolling_feedforward',
    'spherical_wheel.synchronization.ContactSynchronizer._normal_loads',
    'mujoco.mj_contactForce','mujoco.mj_step','mujoco.mj_step1','mujoco.mj_step2')

def forbidden(*args,**kwargs):
    raise AssertionError('I78 reference selection attempted dynamics integration or a physical contact/terrain query')

def load_core():
    name='_i78_frozen_governor_core'
    if name in sys.modules:return sys.modules[name]
    path=BASE/'governor/progress_governor.py'
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
    return module

class GovernorSession:
    def __init__(self,policy,native_command,settings=None):
        if policy not in ('N','G'):raise ValueError('Only original native N and governed G are permitted')
        self.policy=policy;self.native_command=native_command;self.settings=settings
        self.requested_horizon_s=45. if policy=='N'else 75.
        self.core=load_core();self.governor=None;self.probe=None
        self.rows=[];self.pending=None;self.route=None
        self.first_complete_sample_s=None;self.dense_finished_at_s=None

    def select(self,controller,physical,path,now):
        self.route=path
        nominal=tuple(np.asarray(x,float).copy()for x in path(now))
        if self.policy=='N':
            u,du,ddu,_=path.progress(now)
            end=float(path.start_s+path.duration_s)
            complete=now>=end-1e-9
            if complete and self.first_complete_sample_s is None:
                self.first_complete_sample_s=float(now);self.dense_finished_at_s=end
            diagnostics=dict(time_s=now,virtual_time_s=now,virtual_delay_s=0.,
                clock_rate=1.,clock_acceleration_s_inv=0.,cascade_state=1.,
                previous_target_rate=1.,new_target_rate=1.,
                progress_u=u,progress_rate_s_inv=du,progress_acceleration_s_inv2=ddu,
                reference_complete=complete,finished_at_s=self.dense_finished_at_s,
                first_complete_sample_s=self.first_complete_sample_s,
                completion_time_kind='Original nominal clock; not robot goal settling',
                initial_hold_forced_native=now<=path.start_s,nominal_probe_calls=0,
                retiming_ever_active=False,authority_factor=np.nan,
                joint_reference_factor=np.nan,command_factor=np.nan,
                authority_sigma_min_m_s=np.nan,maximum_nominal_joint_reference_rad_s=np.nan,
                nominal_command_ratio=np.nan,raw_target_rate=np.nan)
            selection=dict(reference=nominal,nominal_reference=nominal,nominal_probe=None,diagnostics=diagnostics)
        else:
            if self.governor is None:
                settings=self.core.GovernorSettings(**(self.settings or {}))
                self.governor=self.core.ProgressGovernor(path,settings)
                self.probe=NativeCommandProbe(controller,self.native_command)
            before=self.probe.calls
            terrain=physical.terrain
            with ExitStack()as guards:
                for name in GUARDS:guards.enter_context(patch(name,forbidden))
                guards.enter_context(patch.object(physical,'contact_records',forbidden))
                for name in ('height','heights_at','normal_at','closest_point','evaluate','_plane'):
                    guards.enter_context(patch.object(terrain,name,forbidden))
                selection=self.governor.select(now,self.probe.evaluate)
            if self.probe.calls!=before+1:raise AssertionError('Exactly one nominal private probe per command required')
            self.first_complete_sample_s=selection['diagnostics']['first_complete_sample_s']
            self.dense_finished_at_s=selection['diagnostics']['finished_at_s']
        self.pending=dict(selection=selection,nominal_clock_reference=nominal)
        return selection['reference']

    def confirm(self,controller,applied,info,reference):
        if self.pending is None:raise AssertionError('Live command without reference selection')
        pending=self.pending;selection=pending['selection'];diag=selection['diagnostics']
        native_identity=(self.probe.confirm_live(applied,info,reference)if self.probe else False)
        probe=selection['nominal_probe']
        pi={}if probe is None else probe[1]['i78_probe']
        def array(name,shape):
            return np.full(shape,np.nan)if probe is None else np.asarray(probe[1][name]).copy()
        row=dict(time_s=float(controller.plant.data.time),
            nominal_clock_xy=np.array(pending['nominal_clock_reference'][0],copy=True),
            nominal_clock_velocity_m_s=np.array(pending['nominal_clock_reference'][1],copy=True),
            nominal_clock_acceleration_m_s2=np.array(pending['nominal_clock_reference'][2],copy=True),
            nominal_virtual_xy=np.array(selection['nominal_reference'][0],copy=True),
            nominal_virtual_velocity_m_s=np.array(selection['nominal_reference'][1],copy=True),
            nominal_virtual_acceleration_m_s2=np.array(selection['nominal_reference'][2],copy=True),
            governed_xy=np.array(reference[0],copy=True),governed_velocity_m_s=np.array(reference[1],copy=True),
            governed_acceleration_m_s2=np.array(reference[2],copy=True),
            probe_requested=array('requested',(5,)),probe_applied=(np.full(5,np.nan)if probe is None else np.asarray(probe[0]).copy()),
            probe_joint_reference=(np.full(3,np.nan)if probe is None else np.asarray(probe[1]['allocation_drive_reference'])[:3].copy()),
            probe_authority_matrix=np.array(pi.get('authority_matrix',np.full((2,5),np.nan)),copy=True),
            probe_scaled_singular_values=np.array(pi.get('authority_scaled_singular_values',np.full(2,np.nan)),copy=True),
            probe_full_stack_singular_values=np.array(pi.get('full_stack_scaled_singular_values',np.full(5,np.nan)),copy=True),
            probe_full_stack_rank=int(pi.get('full_stack_rank',0)),
            probe_full_stack_condition=float(pi.get('full_stack_condition_resolved',np.nan)),
            probe_full_stack_residual_m_s=float(pi.get('full_stack_requested_rolling_residual_norm_m_s',np.nan)),
            probe_record_count=int(pi.get('record_count',0)),
            private_observer_updates=int(pi.get('probe_control_updates',0)),
            private_dynamic_steps=int(pi.get('probe_dynamic_steps',0)),
            private_probe_present=probe is not None,
            live_probe_sensor_stages_checked=probe is not None,
            live_probe_sensor_stages_match=probe is not None,
            full_output_identity_checked=bool(native_identity),
            full_output_identity_when_references_equal=bool(native_identity),
            no_physical_contact_or_terrain_read_during_selection=True)
        row.update({'gov__'+name:(np.nan if value is None else value)for name,value in diag.items()})
        row['gov__finished_at_valid']=diag['finished_at_s']is not None
        row['gov__first_complete_sample_valid']=diag['first_complete_sample_s']is not None
        self.rows.append(row);self.pending=None

    def should_stop(self,now):
        if self.policy=='N':return False
        arrival=self.first_complete_sample_s
        return arrival is not None and now>=arrival+10.-1e-9

    @property
    def planned_stop_s(self):
        arrival=self.first_complete_sample_s
        if self.policy=='N' or arrival is None:return self.requested_horizon_s
        return min(self.requested_horizon_s,arrival+10.)

    def arrays(self):
        if self.pending is not None:raise AssertionError('Incomplete selected command')
        if not self.rows:return{}
        arrays={name:np.asarray([row[name]for row in self.rows])for name in self.rows[0]}
        if any(value.dtype==object for value in arrays.values()):raise TypeError('Governor sidecar cannot contain object arrays')
        arrays['command_held_after_row']=np.arange(len(self.rows))<len(self.rows)-1
        return arrays

    def outcome(self,summary):
        now=float(self.rows[-1]['time_s'])if self.rows else 0.
        reference_complete=self.first_complete_sample_s is not None
        hold_complete=reference_complete and now>=self.first_complete_sample_s+10.-1e-9
        survived=summary.get('failure')is None
        return dict(iteration='I-78',policy=self.policy,actual_end_s=now,
            maximum_physical_budget_s=self.requested_horizon_s,
            requested_stop_s=self.planned_stop_s,
            reference_complete=reference_complete,finished_at_s=self.dense_finished_at_s,
            first_complete_sample_s=self.first_complete_sample_s,
            full_goal_hold_completed=hold_complete,
            physical_goal_hold_s=(max(0.,now-self.first_complete_sample_s)if reference_complete else 0.),
            diagnostic_completed=bool(survived and reference_complete and hold_complete),
            survived_without_native_failure=survived,native_failure=summary.get('failure'),
            terminal_command_held=False,default_changed=False,
            parent_summary_qualification='Inherited native summary retains fixed nominal metric windows; completed means the dynamic requested stop was reached without native failure. I78 reference progress/arrival/10s goal-reference hold and independent physical-goal/settling gates remain separate.')
