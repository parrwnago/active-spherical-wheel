"""I76 native versus trajectory-tangent heading, one fresh Gazebo case.

The existing OffroadController is unmodified. N keeps the selected native parameter values;
T changes heading target/rate and the heading helper's Euler-rate feedback.
No alternate contact selection, new gain design, force injection or reset.
"""
from contextlib import ExitStack
from datetime import datetime, timezone
from importlib.metadata import version
from pathlib import Path
from unittest.mock import patch
import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys
import time
import traceback

import numpy as np


ROOT = Path(__file__).resolve().parents[3]
BASE = Path(__file__).resolve().parents[1]
FAMILIES = ('shell_A', 'shell_B', 'belt_A', 'belt_B')
BOUNDS = np.array([1.5, .6, .6, .3, .3])
GUARDS = ('spherical_wheel.offroad_control.measure_contact',
    'spherical_wheel.offroad_control.records_from_mujoco',
    'spherical_wheel.offroad_control.rolling_feedforward',
    'spherical_wheel.synchronization.ContactSynchronizer._normal_loads',
    'mujoco.mj_contactForce')
INFO_FIELDS = ('feedback_gain', 'error', 'reference_qpos', 'reference_qvel',
    'feedback_reference_qvel', 'equilibrium_command', 'rolling_feedforward',
    'support_command', 'lqr_command', 'transition_command_delta', 'heading_command',
    'fade_command_delta', 'clipping_command_delta', 'sync_weights', 'input_bounds',
    'estimated_support_loads_N', 'estimated_contact_point_m', 'allocation_drive_reference',
    'reference_quat', 'reference_acceleration_m_s2', 'feedback_reference_xy')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def safe(value):
    if isinstance(value, Path): return str(value)
    if isinstance(value, dict): return {str(key): safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)): return [safe(item) for item in value]
    if isinstance(value, np.ndarray): return safe(value.tolist())
    if isinstance(value, np.generic): return safe(value.item())
    if isinstance(value, float) and not np.isfinite(value): return None
    return value


def save(path, value):
    Path(path).write_text(json.dumps(safe(value), indent=2, sort_keys=True, allow_nan=False) + '\n')


def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def fingerprint(value):
    return hashlib.sha256(json.dumps(safe(value), sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def forbidden(*args, **kwargs):
    raise AssertionError('Control attempted a solved-contact, terrain or physical-state query')


def route_module():
    return load_module(BASE/'runtime/route.py','_i76_registered_route')


def registered_cases():
    route=route_module()
    return [dict(case_id=f'{route_id}_{phase}_{policy}',route_id=route_id,policy=policy,
            phase_rad=angle,yaw_deg=route.make_route(route_id).heading_deg,
            duration_s=route.make_route(route_id).total_duration_s)
        for route_id in ('straight','arc','eight')
        for phase,angle in (('m100',-1.),('p030',.3),('p090',.9)) for policy in ('N','T')]


def validate_protocol(protocol, case_id):
    if protocol['iteration']!='I-76':raise ValueError('I76 protocol required')
    expected=registered_cases();jobs=protocol['cases']
    if jobs!=expected:raise ValueError('Exactly18 frozen route/phase/policy cases with route-specific aligned yaw and clocks required')
    validate_heading_parameters(protocol)
    choices=[job for job in jobs if job['case_id']==case_id]
    if len(choices)!=1:raise ValueError('One registered case ID required')
    return choices[0]


HEADING_SETTINGS = dict(heading_reference='start',heading_actuation='shells',
    heading_gain=.2,heading_damping=.06,heading_torque_limit_Nm=.15,
    heading_roll_budget_Nm=.05)
REFERENCE_SETTINGS=dict(max_rate_rad_s=.35,max_acceleration_rad_s2=.5)
BASELINE_SETTINGS = dict(mode='hardware',rolling_allocation='least_squares',track_contact_point=False)


def validate_heading_parameters(protocol):
    if protocol['parameters'] != BASELINE_SETTINGS:
        raise ValueError('Native baseline parameter dictionary must equal the original I73 dictionary')
    if protocol.get('heading_reference_parameters') != REFERENCE_SETTINGS:
        raise ValueError('Fixed analytic reference bounds required')
    if protocol.get('heading_parameters') != HEADING_SETTINGS:
        raise ValueError('T must use the one fixed predeclared bounded-heading setting')


def parameters_for(protocol,job):
    validate_heading_parameters(protocol)
    result = dict(protocol['parameters'])
    if job['policy']=='T': result.update(protocol['heading_parameters'])
    elif job['policy']!='N': raise ValueError('Only N and T are registered')
    return result


def preflight(base, case_id):
    path = base / 'protocol/protocol.json'
    protocol = json.loads(path.read_text())
    job = validate_protocol(protocol, case_id)
    for group in ('source_sha256', 'input_sha256'):
        for relative, expected in protocol[group].items():
            if sha(ROOT / relative) != expected: raise ValueError('Registered source/input changed: ' + relative)
    cache = ROOT / protocol['gain_cache']['path']
    if sha(cache) != protocol['gain_cache']['sha256']: raise ValueError('Frozen flat gain cache changed')
    from spherical_wheel.gazebo.bridge import plugin_path
    if sha(plugin_path()) != protocol['bridge_binary_sha256']: raise ValueError('Frozen bridge plugin changed')
    map_entry = protocol['map_preflight']
    report_path = ROOT / map_entry['report_path']
    if sha(report_path) != map_entry['sha256']: raise ValueError('Registered map-only fixture report changed')
    if not json.loads(report_path.read_text())['passed']: raise ValueError('Map-only fixture screen did not pass')
    return protocol, job


def design_identity(design):
    return fingerprint([{key: point[key] for key in ('phase','K','eq','regime','side')} for point in design.points])


def snapshot(base, directory, protocol, job):
    sources = []
    paths = {ROOT / relative for relative in protocol['source_sha256']}
    paths.add(base / 'protocol/protocol.json')
    for source in sorted(paths):
        relative = source.relative_to(ROOT)
        destination = directory / 'source_snapshot' / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        sources.append(dict(origin=str(relative), snapshot=str(destination.relative_to(directory)), sha256=sha(source)))
    return dict(iteration='I-76', captured_before_physics_utc=datetime.now(timezone.utc).isoformat(),
        git_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        case=job, pid=os.getpid(), sources=sources, protocol_sha256=sha(base/'protocol/protocol.json'),
        measurement_boundary='Current exact simulated q/v and last actual held input; observer/known-geometry selection only. Physical contacts/terrain prohibited during command.',
        intervention='T uses planned tangent angle/rate and exact Euler-rate damping in the existing load/lever/torque-budget helper. Native LQR/observer/contacts/rolling/support/preparation/caps retained.',
        counterfactual='None. Native N is the independent closed-loop comparator; heading contribution is logged before fade/clipping.',
        dropout='Original native held loads/gain/reference history and stationary rolling branches unchanged.',
        default_changed=False)


def truth_row(plant, measurement):
    loads = np.zeros(4); sums = np.zeros((4,3)); counts = np.zeros(4,np.int64)
    for record in plant.contact_records():
        for family, name in enumerate(FAMILIES):
            if record['name'] == name or record['name'].startswith(name+'_'):
                pressure = max(0.,float(record['normal_force']))
                loads[family] += pressure; sums[family] += pressure*np.asarray(record['pos'])
                counts[family] += int(pressure>.05); break
    np.testing.assert_allclose(loads, [measurement['loads_N'][name] for name in FAMILIES], atol=1e-10, rtol=1e-10)
    points = np.full((4,3),np.nan); known = loads>1e-12; points[known]=sums[known]/loads[known,None]
    return dict(truth_time_s=float(plant.data.time),truth_patch_loads_N=loads,
        truth_patch_points_m=points,truth_patch_contact_counts=counts,
        truth_equivalent_point_m=np.asarray(measurement['point_m']).copy())


def assert_native_feedback(controller, info, xy, acceleration):
    """Check native support/gain/raw-attitude inputs, not trajectory equality."""
    normal = np.asarray(info['estimate']['normal'])
    held = controller._held
    loads = dict(info['measurement']['loads_N']) if held is None else dict(held['loads'])
    expected_gain = np.array(controller.ordinary['design']['K'] if held is None else held['gain'],copy=True)
    expected_gain[0] = controller.ordinary['design']['K'][0]
    if not np.array_equal(info['feedback_gain'],expected_gain): raise AssertionError('Native feedback gain routing changed')
    expected_support = controller.params.support_feedforward*controller.support_torque(normal,loads)
    if not np.array_equal(info['support_command'],expected_support): raise AssertionError('Native support torque changed')
    if not np.array_equal(info['reference_quat'],controller.reference_attitude(normal,acceleration)):
        raise AssertionError('Raw reference attitude changed')
    if not np.array_equal(info['reference_qpos'][:2],xy): raise AssertionError('Original raw timed XY target changed')


def run_case(base, case_id):
    base = Path(base).resolve()
    protocol, job = preflight(base, case_id)
    directory = base / 'campaign/runs' / case_id
    directory.mkdir(parents=True, exist_ok=False)
    provenance = snapshot(base,directory,protocol,job)
    save(directory/'provenance_before.json',provenance)
    adapter_module = load_module(base/'adapter/tangent_heading_adapter.py','_i76_tangent_heading_adapter')
    sys.path.insert(0,str(base/'runtime'))
    reference_module = load_module(base/'runtime/heading_reference.py','_i76_heading_reference')
    route = load_module(base/'runtime/route.py','_i76_route')
    import spherical_wheel.offroad_experiments as experiments
    from spherical_wheel.offroad_control import OffroadController, OffroadParams
    from spherical_wheel.gazebo.experiments import GazeboWorkspace, run_gazebo
    from spherical_wheel.gazebo.mars import MarsYard
    case_parameters = parameters_for(protocol,job)
    path = route.make_route(job['route_id'])
    reference = reference_module.PlannedTangentReference(**protocol['heading_reference_parameters'],path=path)
    if path.total_duration_s != float(job['duration_s']): raise ValueError('Frozen path duration mismatch')
    check_times = np.linspace(path.start_s,path.start_s+path.duration_s,7)
    check_points = np.array([path(t)[0] for t in check_times])
    save(directory/'runtime_before_physics.json',dict(route=path.spec(),
        ground_check_times_s=check_times,ground_check_points_xy_m=check_points,
        unchanged_ground_check='Original Gazebo verify_ground median<=200um and at least50% points within200um; no tolerance change.',
        visual_bounds_xy_m=route.world_bounds(job['route_id']),control_interval_s=.005,requested_duration_s=job['duration_s']))
    rows, truth = [], []
    workspace = None
    started = time.monotonic()
    try:
        terrain = MarsYard()
        workspace = GazeboWorkspace(directory=base/'campaign/_workspaces'/case_id,
            seed_cache=ROOT/protocol['gain_cache']['path'],
            plant_options=dict(ground='marsyard2021_dem',visual_bounds=route.world_bounds(job['route_id']),ground_check_points=check_points))
        if sha(workspace.cache_path) != protocol['gain_cache']['sha256'] or not workspace.design.metadata['cache_loaded']:
            raise AssertionError('Frozen gain bank unexpectedly rebuilt')
        design_hash = design_identity(workspace.design)
        physical, mirror = workspace.plant(terrain), workspace.mirror()
        np.testing.assert_array_equal(physical.bounds,BOUNDS);np.testing.assert_array_equal(mirror.bounds,BOUNDS)
        original_command = OffroadController.command
        original_evaluate = experiments.measure_contact
        ordinary_hash = None; latched_heading = None; adapter = None

        def measured(plant):
            value = original_evaluate(plant)
            truth.append(truth_row(plant,value))
            return value

        def command(controller,xy,velocity,acceleration):
            nonlocal ordinary_hash,latched_heading,adapter
            assert controller.plant is mirror and controller.design is workspace.design
            assert mirror.ground is None and mirror.terrain is None and mirror.data.ncon == 0
            np.testing.assert_array_equal(mirror.data.qpos,physical.data.qpos)
            np.testing.assert_array_equal(mirror.data.qvel,physical.data.qvel)
            np.testing.assert_array_equal(mirror.last_input,physical.last_input)
            assert mirror.data.time == physical.data.time
            identity = fingerprint(dict(equilibrium=controller.ordinary['equilibrium'],
                design={key:controller.ordinary['design'][key] for key in ('K','A','B','state_scales','keep')}))
            if ordinary_hash is None: ordinary_hash=identity
            if identity != ordinary_hash: raise AssertionError('Ordinary gain/design changed')
            if latched_heading is None: latched_heading=float(controller.heading_reference)
            if float(controller.heading_reference)!=latched_heading: raise AssertionError('Native initial-heading latch changed after reset')
            if adapter is None: adapter=adapter_module.TangentHeadingAdapter(controller,job['policy'],reference)
            previous = mirror.last_input.copy(); nominal = controller.nominal_joints.copy()
            original_update = controller.observer.update
            update_count = 0
            def updated(*args,**kwargs):
                nonlocal update_count
                update_count += 1
                return original_update(*args,**kwargs)
            with ExitStack() as guards:
                for target in GUARDS: guards.enter_context(patch(target,forbidden))
                guards.enter_context(patch.object(physical,'contact_records',forbidden))
                for name in ('height','heights_at','normal_at','closest_point','evaluate','_plane'):
                    guards.enter_context(patch.object(terrain,name,forbidden))
                guards.enter_context(patch.object(controller.observer,'update',updated))
                with adapter.command_context():
                    applied,info=original_command(controller,xy,velocity,acceleration)
                diagnostics=adapter.diagnostics(info)
                if not rows and abs(float(diagnostics['raw_error_rad']))>np.deg2rad(1.)+1e-12:
                    raise AssertionError('Initial body heading differs from planned tangent by more than1degree')
                if job['policy']=='N' and np.any(info['heading_command']):
                    raise AssertionError('Native N unexpectedly enabled heading helper')
                assert_native_feedback(controller,info,xy,acceleration)
            if update_count != 1: raise AssertionError('Exactly one native observer update per command required')
            requested=np.asarray(info['requested']).copy();applied=np.asarray(applied).copy()
            np.testing.assert_array_equal(applied,np.clip(requested,-mirror.bounds,mirror.bounds))
            row=dict(time_s=float(mirror.data.time),exact_qpos=mirror.data.qpos.copy(),exact_qvel=mirror.data.qvel.copy(),
                exact_applied=applied,exact_previous_applied=previous,exact_requested=requested,
                exact_nominal_joints_before=nominal,exact_reference_xy=np.asarray(xy).copy(),
                exact_reference_velocity_m_s=np.asarray(velocity).copy(),exact_reference_acceleration_m_s2=np.asarray(acceleration).copy(),
                exact_observer_wrench=np.r_[controller.observer.wrench()],
                exact_observer_generalized=controller.observer.generalized.copy(),
                exact_observer_interval_mean=controller.observer.interval_mean.copy(),
                exact_estimated_normal=np.asarray(info['estimate']['normal']).copy(),
                exact_effective_phase_rad=float(info['effective_phase_rad']),exact_drive_fade=float(info['drive_fade']),
                native_observer_update_count=np.int64(update_count),native_feedback_boundary_verified=True,
                mirror_previous_applied_matches_physical=True,no_truth_or_terrain_read_during_command=True)
            row.update({'exact_'+key:np.asarray(info[key]).copy() for key in INFO_FIELDS})
            row['exact_heading_reference_rad']=float(controller.heading_reference)
            row['exact_heading_error_rad']=float(info['heading_error_rad'])
            row.update({'heading__'+key:np.asarray(value).copy() for key,value in diagnostics.items()})
            rows.append(row)
            if len(rows)%400==0:
                print(json.dumps(dict(case=case_id,time_s=float(mirror.data.time),elapsed_s=time.monotonic()-started)),flush=True)
            return applied,info

        with patch.object(OffroadController,'command',command),patch.object(experiments,'measure_contact',measured):
            log,summary=run_gazebo(workspace,terrain,path,OffroadParams(**case_parameters),
                phase=job['phase_rad'],yaw_deg=job['yaw_deg'],duration_s=job['duration_s'],record_diagnostics=True)
        if not (len(rows)==len(truth)==len(log['time_s'])): raise AssertionError('Control/truth clocks differ')
        precision={key:np.asarray([row[key] for row in rows]) for key in rows[0]}
        precision.update({key:np.asarray([row[key] for row in truth]) for key in truth[0]})
        np.testing.assert_array_equal(precision['time_s'],precision['truth_time_s'])
        np.testing.assert_array_equal(precision['exact_previous_applied'][1:],precision['exact_applied'][:-1])
        precision['command_held_after_row']=np.arange(len(rows))<len(rows)-1
        np.savez_compressed(directory/'precision.npz',**precision)
        summary['tangent_heading_experiment']=dict(iteration='I-76',case=job,source_boundary=provenance['measurement_boundary'],
            heading_target_policy='N retains native initial latch; T follows analytic desired route tangent and Euler rate, defined geometrically at zero-speed endpoints.',
            counterfactual_scope=provenance['counterfactual'],default_changed=False)
        experiments.save_run(directory,log,summary,terrain)
        preflight(base,case_id)
        if sha(workspace.cache_path)!=protocol['gain_cache']['sha256'] or design_identity(workspace.design)!=design_hash:
            raise AssertionError('Frozen gain bank changed during run')
        provenance.update(completed=bool(summary['completed']),failure=summary['failure'],
            actual_duration_s=float(summary['completed_duration_s']),elapsed_s=time.monotonic()-started,
            guarded_calls=len(rows),parameters=OffroadParams(**case_parameters).as_dict(),
            ordinary_design_sha256=ordinary_hash,fixed_bank_numeric_sha256=design_hash,
            fixed_bank_bytes_sha256=sha(workspace.cache_path),frozen_bank_loaded=True,
            versions={name:version(name) for name in ('numpy','scipy','mujoco')},
            models=dict(mirror_xml=str(workspace.directory/'mirror/model.xml'),
                        physical_sdf=str(physical.world_path.parent/'models/active_spherical_wheel/model.sdf')),
            source_hashes_unchanged_after=True)
        save(directory/'provenance.json',provenance)
        save(directory/'artifact_manifest.json',dict(files=[dict(path=str(path.relative_to(directory)),
            bytes=path.stat().st_size,sha256=sha(path)) for path in sorted(directory.iterdir()) if path.is_file()]))
        print(json.dumps(dict(case=case_id,completed=summary['completed'],failure=summary['failure'],frames=len(rows))),flush=True)
    except Exception:
        if rows:
            np.savez_compressed(directory/'partial_precision.npz',
                **{key:np.asarray([row[key] for row in rows]) for key in rows[0]})
        if truth:
            np.savez_compressed(directory/'partial_truth.npz',
                **{key:np.asarray([row[key] for row in truth]) for key in truth[0]})
        save(directory/'failure.json',dict(case=job,traceback=traceback.format_exc(),elapsed_s=time.monotonic()-started))
        raise
    finally:
        if workspace is not None and workspace._current is not None:
            workspace._current[1].close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,default=BASE)
    parser.add_argument('--case',required=True)
    parser.add_argument('--preflight-only',action='store_true')
    args=parser.parse_args()
    if args.preflight_only:
        protocol,job=preflight(args.base.resolve(),args.case)
        print(json.dumps(dict(passed=True,case=job,physics_run=False)))
    else:
        run_case(args.base,args.case)
