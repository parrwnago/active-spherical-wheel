"""Retrospective original-native support/motion review, no physics replay."""
from pathlib import Path
import hashlib
import json
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
RUN = ROOT / 'results/tangent_heading_routes_20261127/campaign/runs/eight_m100_N'


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def finite_stats(a):
    a = np.asarray(a)
    assert np.all(np.isfinite(a))
    return dict(min=float(a.min()), max=float(a.max()), mean=float(a.mean()),
                rms=float(np.sqrt(np.mean(a*a))))


def episodes(mask, times):
    indices = np.flatnonzero(mask)
    if not len(indices):
        return []
    groups = np.split(indices, np.flatnonzero(np.diff(indices)>1)+1)
    return [dict(first_s=float(times[g[0]]), last_s=float(times[g[-1]]),
                 sampled_rows=int(len(g)), sampled_grid_occupancy_s=float(len(g)*.005),
                 caveat='5 ms grid occupancy is not proof of continuous duration between samples')
            for g in groups]


def main():
    receipt = OUT / 'registered_before_analysis.json'
    if receipt.exists():
        raise RuntimeError('Refuse to overwrite closed stage')
    inputs = [RUN/'precision.npz', RUN/'rollout.npz', RUN/'summary.json',
              ROOT/'results/tangent_heading_routes_20261127/campaign_tools/run_case.py',
              ROOT/'src/spherical_wheel/contact_sensing.py',
              ROOT/'src/spherical_wheel/offroad_experiments.py']
    binding = dict(iteration='I-80', scope='Original I76 eight_m100_N only; existing 99–103 s episode. Retrospective, no blind causal-test claim.',
                   window_s=[99.,103.], source_sha256=sha(Path(__file__)),
                   prior_schema_access=True, no_new_physics=True,
                   snapshot_times_s=[99.,99.175,100.04,100.5,101.,101.5,102.135,103.],
                   input_files=[dict(path=str(p.relative_to(ROOT)), bytes=p.stat().st_size, sha256=sha(p)) for p in inputs])
    receipt.write_text(json.dumps(binding,indent=2)+'\n')
    with np.load(RUN/'precision.npz',allow_pickle=False) as z:
        d={k:z[k].copy() for k in z.files}
    with np.load(RUN/'rollout.npz',allow_pickle=False) as z:
        f={k:z[k].copy() for k in z.files}
    t=d['time_s']; mask=(t>=99.-1e-8)&(t<=103.+1e-8); indices=np.flatnonzero(mask)
    q=d['exact_qpos'];v=d['exact_qvel']; load=d['truth_patch_loads_N'].sum(axis=1)
    force=f['contact_force_N'].astype(float); speed=np.linalg.norm(v[:,:2],axis=1)
    vz=v[:,2]; az=np.diff(vz)/np.diff(t)
    # These derivatives belong to the free-joint/root point, not system COM.
    valid_intervals=np.flatnonzero(mask[:-1]&mask[1:])
    expected_vz=np.diff(q[:,2])/np.diff(t)
    vmean=(vz[:-1]+vz[1:])/2
    req=d['exact_requested']; app=d['exact_applied']; bounds=d['exact_input_bounds']
    clipped=np.any(np.abs(req)>bounds+1e-12,axis=1)
    ve=v[:,:2]-d['exact_reference_velocity_m_s'];xyerr=np.linalg.norm(q[:,:2]-d['exact_reference_xy'],axis=1)
    samples=[]
    for target in binding['snapshot_times_s']:
        i=int(np.argmin(abs(t-target)));assert abs(t[i]-target)<1e-8
        samples.append(dict(time_s=float(t[i]),qpos_xyz_m=q[i,:3].tolist(),
                    root_velocity_xyz_m_s=v[i,:3].tolist(), xy_speed_m_s=float(speed[i]),
                    xy_reference_velocity_m_s=d['exact_reference_velocity_m_s'][i].tolist(),
                    xy_velocity_error_m_s=ve[i].tolist(), timed_xy_error_m=float(xyerr[i]),
                    pressure_patch_loads_N=d['truth_patch_loads_N'][i].tolist(),
                    pressure_total_N=float(load[i]),pressure_contact_counts=d['truth_patch_contact_counts'][i].tolist(),
                    net_contact_force_world_N=force[i].tolist(),
                    contact_equivalent_point_m=d['truth_equivalent_point_m'][i].tolist(),
                    terrain_centre_height_proxy_m=float(f['ground_height_m'][i]),
                    terrain_centre_normal_proxy=f['ground_normal'][i].astype(float).tolist(),
                    slip_speed_m_s=float(f['slip_speed_m_s'][i]),
                    body_tilt_deg=float(np.degrees(f['tilt_rad'][i])),
                    own_target_up_error_deg=float(np.degrees(f['attitude_error_rad'][i])),
                    world_z_rate_rad_s=float(d['heading__world_z_rate_rad_s'][i]),
                    requested_command=req[i].tolist(),applied_command=app[i].tolist(),
                    command_clipped=bool(clipped[i]),
                    prior_applied_command=d['exact_previous_applied'][i].tolist(),
                    next_interval_root_acceleration_z_m_s2=float(az[i]) if i<len(az) else None,
                    previous_interval_root_acceleration_z_m_s2=float(az[i-1]) if i else None))
    result=dict(iteration='I-80', case='eight_m100_N', no_new_physics=True,
                snapshot_rows=samples, window_s=[float(t[indices[0]]),float(t[indices[-1]])],
                window_rows=int(len(indices)),root_height_change_m=float(q[indices[-1],2]-q[indices[0],2]),
                root_height_m=finite_stats(q[mask,2]),root_vertical_speed_m_s=finite_stats(vz[mask]),
                root_vertical_acceleration_5ms_mean_m_s2=finite_stats(az[valid_intervals]),
                xy_speed_m_s=finite_stats(speed[mask]),xy_timed_error_m=finite_stats(xyerr[mask]),
                pressure_sum_N=finite_stats(load[mask]), net_ground_force_norm_N=finite_stats(np.linalg.norm(force[mask],axis=1)),
                zero_positive_pressure_sample_episodes=episodes(load[mask]<=1e-9,t[mask]),
                below_0p05N_pressure_sample_episodes=episodes(load[mask]<.05,t[mask]),
                clipped_sample_episodes=episodes(clipped[mask],t[mask]),
                maximum_root_height_velocity_quadrature_disagreement_m_s=float(np.max(abs(expected_vz[valid_intervals]-vmean[valid_intervals]))),
                limitations=[
                    'Contact pressure is last completed 1 ms solver-sample data saved every 5 ms, not full interval impulse.',
                    'Zero positive pressure at a saved row does not itself prove collision separation or continuous ballistic flight.',
                    'Nonzero pressure at every saved row cannot rule out sub-5 ms unloading.',
                    'Root/free-joint point vertical acceleration is not whole-system centre-of-mass acceleration.',
                    'Saved ground height/normal is nearest-centre terrain proxy, not actual contact facet.',
                    'Current emitted command acts after current state; saturation may be response, not initiating cause.',
                    'No force work or gravitational energy attribution calculated; downhill motion need not be a wrong drive command.'
                ])
    assert all(sha(ROOT/x['path'])==x['sha256'] for x in binding['input_files'])
    (OUT/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    manifest=dict(inputs_unchanged=True,no_new_physics=True,files=[dict(path=p.name,sha256=sha(p),bytes=p.stat().st_size) for p in sorted(OUT.iterdir()) if p.is_file()])
    (OUT/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ['window_rows','root_height_change_m','root_height_m','root_vertical_speed_m_s','pressure_sum_N','zero_positive_pressure_sample_episodes','below_0p05N_pressure_sample_episodes','xy_speed_m_s']}))


if __name__=='__main__':
    main()
