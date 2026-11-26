"""Three targeted I49 operating-bound integration checks; no dynamics/Gazebo.

One real StateMirror exercises its actual follow/apply-input path. A controlled
velocity fixture exercises the actual incoming-belt clipping branch. Synthetic
physical/design holders avoid constructing a simulator or rebuilding gains.
"""
from pathlib import Path
from types import SimpleNamespace, ModuleType
import argparse
import copy
import hashlib
import json

import numpy as np

from spherical_wheel.mirror import StateMirror
from spherical_wheel.model import load_config
from spherical_wheel.synchronization import ContactSynchronizer


HERE=Path(__file__).resolve().parent
FAMILY=HERE.parent
ROOT=FAMILY.parents[1]
RUNNER=FAMILY/'run_capacity_case.py'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--environment',required=True,choices=('venv','gazebo'))
    args=parser.parse_args();stage=HERE/'campaign_checks_stage01'
    output=stage/f'checks_{args.environment}.json'
    if output.exists():raise FileExistsError('Preserve previous independent check execution')
    source_manifest=json.loads((stage/'source_manifest.json').read_text())
    assert all(sha(ROOT/row['path'])==row['sha256'] for row in source_manifest['files'])
    runner=ModuleType('_i49_capacity_independent');runner.__file__=str(RUNNER)
    exec(compile(RUNNER.read_text(),str(RUNNER),'exec'),runner.__dict__)
    protocol=json.loads((FAMILY/'protocol.json').read_text())
    assert all(sha(ROOT/name)==digest for name,digest in protocol['source_sha256'].items())
    seed=ROOT/protocol['inputs']['gain_cache'];seed_before=sha(seed)
    config=load_config();baseline_config=copy.deepcopy(config)
    mirror=StateMirror(config,stage/f'_mirror_workspace_{args.environment}')
    data=SimpleNamespace(qpos=mirror.data.qpos.copy(),qvel=np.zeros(9),time=1.)
    physical=SimpleNamespace(config=copy.deepcopy(config),bounds=runner.BASE_BOUNDS.copy(),data=data,last_input=np.zeros(5))
    designer=SimpleNamespace(config=copy.deepcopy(config),bounds=runner.BASE_BOUNDS.copy())
    point=dict(phase=.9,K=np.arange(90,dtype=float).reshape(5,18),eq={'qpos':data.qpos.copy()},regime='shell',side='A')
    design=SimpleNamespace(designer=designer,points=[point],weights='flat')
    workspace=SimpleNamespace(config=copy.deepcopy(config),design=design)
    original_design=runner.design_identity(design)
    ctrlrange=mirror.model.actuator_ctrlrange.copy();torque_bounds=mirror.bounds[:3].copy()
    records=[]
    for cap,previous_speed in [(.3,.25),(.6,.55),(.9,.85)]:
        # Reinitialize only the synthetic test fixture to the registered design
        # envelope; no simulator/control state is being reset or compared here.
        physical.bounds[:]=runner.BASE_BOUNDS;mirror.bounds[:]=runner.BASE_BOUNDS
        config_hash=runner.apply_operating_cap(workspace,physical,mirror,cap)
        expected=np.r_[torque_bounds,cap,cap]
        np.testing.assert_array_equal(physical.bounds,expected)
        np.testing.assert_array_equal(mirror.bounds,expected)
        np.testing.assert_array_equal(designer.bounds,runner.BASE_BOUNDS)
        assert runner.digest(runner.design_identity(design))==runner.digest(original_design)
        assert workspace.config==baseline_config and physical.config==baseline_config and mirror.config==baseline_config
        np.testing.assert_array_equal(mirror.model.actuator_ctrlrange,ctrlrange)
        physical.last_input=np.array([.1,.05,-.05,previous_speed,-previous_speed])
        mirror.follow(physical)
        np.testing.assert_array_equal(mirror.last_input,physical.last_input)
        np.testing.assert_array_equal(mirror.data.ctrl,physical.last_input[:3])
        np.testing.assert_array_equal(mirror.model.geom_surfacevel[mirror.belt_ids,1],physical.last_input[3:])
        assert mirror.data.time==data.time and mirror.data.ncon==0 and mirror.ground is mirror.terrain is None
        np.testing.assert_array_equal(mirror.data.qpos,data.qpos);np.testing.assert_array_equal(mirror.data.qvel,data.qvel)
        sync=ContactSynchronizer(mirror);original_velocity=sync._velocity

        def controlled_velocity(patch,state,qvel,include_surface=True):
            if patch['dof'] is not None:return original_velocity(patch,state,qvel,include_surface)
            axis=state.geom_xmat[patch['geom']].reshape(3,3)[:,1]
            # Produces a +2m/s unconstrained target for each belt, so old .30
            # clipping would fail the .60/.90 checks deterministically.
            return -2.*axis,np.zeros((3,mirror.model.nv)),np.eye(3),0.,0.

        sync._velocity=controlled_velocity
        preview=sync.preview(np.array([2.,1.,-1.,2.,-2.]),records=[],ground=(np.zeros(3),np.array([0.,0.,1.])))
        belt_targets=[target for target in preview['targets'] if target['name'].startswith('belt_')]
        assert len(belt_targets)==2
        np.testing.assert_array_equal([target['target_surface_speed_m_s'] for target in belt_targets],[cap,cap])
        assert preview['commands'][0]==torque_bounds[0]
        assert all(abs(target.get('target_torque_Nm',0.))<=.6 for target in preview['targets'])
        assert sync.shell_speed_limit_rad_s==20.
        runner.verify_operating_cap(workspace,physical,mirror,cap,config_hash)
        records.append(dict(cap_m_s=cap,previous_belt_speed_preserved_m_s=previous_speed,
            incoming_belt_targets_m_s=[target['target_surface_speed_m_s'] for target in belt_targets]))
    assert sha(seed)==seed_before==protocol['input_sha256']['gain_cache']
    assert all(sha(ROOT/row['path'])==row['sha256'] for row in source_manifest['files'])
    report=dict(passed=True,environment=args.environment,unique_checks=3,
        checks=dict(only_operating_belt_bounds_change=True,mirror_follow_preserves_actual_high_cap_input=True,
                    incoming_preparation_uses_new_cap_with_old_torque_shell_bounds=True),
        cases=records,source_manifest_unchanged=True,gain_cache_unchanged=True,
        scope='Synthetic design/physical fixtures + one actual StateMirror/synchronizer path. Kinematics only, no Gazebo/dynamics integration/gain design/controller rollout.',
        runner_sha256=sha(RUNNER),gain_cache_sha256=seed_before)
    output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()
