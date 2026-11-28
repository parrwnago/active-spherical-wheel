"""Read-only geometry of original I76 eight -1 N, fixed 100--102.5 s window.

Queries the retained Gazebo DEM collision OBJ at recorded centre and positive-
pressure patch-mean XY. It does not execute simulator/controller or infer a
unique physical contact from an aggregate point.
"""
from pathlib import Path
import hashlib
import json
import math

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
CASE = ROOT / 'results/tangent_heading_routes_20261127/campaign/runs/eight_m100_N'
WORK = ROOT / 'results/tangent_heading_routes_20261127/campaign/_workspaces/eight_m100_N/gazebo/world'
MESH = WORK / 'marsyard2021_dem.obj'
DEM = ROOT / 'assets/gazebo/third_party/leo_simulator-ros2/leo_gz_worlds/models/marsyard2021_terrain/dem/marsyard_terrain_hm.tif'
INPUTS = [CASE/'precision.npz', CASE/'rollout.npz', CASE/'provenance.json',
          CASE/'terrain.json', WORK/'asw_marsyard2021.sdf', MESH, DEM,
          ROOT/'src/spherical_wheel/gazebo/mars.py']


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for part in iter(lambda: stream.read(1 << 20), b''):
            h.update(part)
    return h.hexdigest()


def save(name, value):
    (OUT/name).write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')


def main():
    marker = OUT/'registered_before_arrays.json'
    if marker.exists():
        raise RuntimeError('Closed stage; do not overwrite')
    bound = [{'path': str(p.relative_to(ROOT)), 'bytes': p.stat().st_size,
              'sha256': sha(p)} for p in INPUTS]
    save('registered_before_arrays.json', dict(
        iteration='I-80', scope='Retrospective geometric inspection; fixed user-cued 100--102.5 s, not causal intervention.',
        source_sha256=sha(Path(__file__)), inputs=bound, no_new_physics=True,
        selected_times_s=[100.,100.5,101.,101.5,102.,102.5],
        qualifications=['Actual collision OBJ queries, not a nearest-centre tangent proxy.',
                        'Per-patch recorded positive-pressure mean may average several contacts; querying its XY is not identification of every actual loaded facet.',
                        'Local DEM excludes rock/lava meshes; original route corridor has separate clearance screening.']))
    from PIL import Image
    dem = np.asarray(Image.open(DEM), float)
    n = len(dem); verts = []; face_count = 0; first_faces = []
    with MESH.open() as stream:
        for line in stream:
            if line.startswith('v '):
                verts.append([float(x) for x in line.split()[1:4]])
            elif line.startswith('f '):
                face_count += 1
                if len(first_faces) < 3:
                    first_faces.append([int(x.split('/')[0])-1 for x in line.split()[1:]])
    vertices = np.asarray(verts)
    grid = vertices.reshape(n,n,3)
    assert face_count == 2*(n-1)**2
    assert first_faces == [[0,n+1,1],[1,n+2,2],[2,n+3,3]]
    assert np.max(abs(grid[:,:,2]-dem)) <= 0.500001e-6
    assert np.all(grid[:,:,0] == grid[0,:,0][None,:])
    assert np.all(grid[:,:,1] == grid[:,0,1][:,None])

    def surface(xy):
        x,y = map(float,xy)
        c = int(np.clip(np.searchsorted(grid[0,:,0],x,side='right')-1,0,n-2))
        r = int(np.clip(np.searchsorted(-grid[:,0,1],-y,side='right')-1,0,n-2))
        a,b,cc,d=grid[r,c],grid[r,c+1],grid[r+1,c+1],grid[r+1,c]
        u=(x-a[0])/(b[0]-a[0]);v=(a[1]-y)/(a[1]-d[1]);upper=u>=v
        ids=[r*n+c,(r+1)*n+c+1,r*n+c+1] if upper else [r*n+c,(r+1)*n+c,(r+1)*n+c+1]
        triangle=vertices[ids]
        normal=np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0]);normal/=np.linalg.norm(normal)
        assert normal[2]>0
        height=float(triangle[0,2]-(normal[0]*(x-triangle[0,0])+normal[1]*(y-triangle[0,1]))/normal[2])
        return dict(height_m=height,normal=normal.tolist(),gradient=[float(-normal[0]/normal[2]),float(-normal[1]/normal[2])],
                    slope_deg=math.degrees(math.acos(float(normal[2]))),
                    facet=[r,c,'abc' if upper else 'acd'],
                    face_index_zero_based=r*(n-1)+c+(0 if upper else (n-1)**2))

    with np.load(CASE/'precision.npz') as raw:
        t=raw['time_s']; selected=(t>=100.-1e-8)&(t<=102.5+1e-8)
        idx=np.flatnonzero(selected);q=raw['exact_qpos'][idx];v=raw['exact_qvel'][idx]
        loads=raw['truth_patch_loads_N'][idx];points=raw['truth_patch_points_m'][idx]
        ref=raw['exact_reference_xy'][idx];times=t[idx]
    frames=[]
    for j,tm in enumerate(times):
        center=surface(q[j,:2]); patch=[]
        for k,load in enumerate(loads[j]):
            if load>.05 and np.isfinite(points[j,k]).all():
                geom=surface(points[j,k,:2]);patch.append(dict(index=k,load_N=float(load),point_m=points[j,k].tolist(),
                    point_minus_dem_m=float(points[j,k,2]-geom['height_m']),dem=geom))
        frames.append(dict(time_s=float(tm),center_m=q[j,:3].tolist(),linear_velocity_m_s=v[j,:3].tolist(),
                           center_dem=center,reference_xy=ref[j].tolist(),pressure_patch_means=patch))
    snapshots=[]
    for when in [100.,100.5,101.,101.5,102.,102.5]:
        row=min(frames,key=lambda x:abs(x['time_s']-when));snapshots.append(row)
    center_h=np.array([x['center_dem']['height_m'] for x in frames]);z=q[:,2]
    facet_changes=[]
    for left,right in zip(frames,frames[1:]):
        if left['center_dem']['facet']!=right['center_dem']['facet']:
            n0=np.array(left['center_dem']['normal']);n1=np.array(right['center_dem']['normal'])
            facet_changes.append(dict(time_s=right['time_s'],from_facet=left['center_dem']['facet'],to_facet=right['center_dem']['facet'],
                normal_change_deg=math.degrees(math.acos(float(np.clip(n0@n1,-1,1))))))
    center_grad=np.array([x['center_dem']['gradient'] for x in frames]);toprate=np.einsum('ij,ij->i',center_grad,v[:,:2])
    report=dict(iteration='I-80',case='eight_m100_N',window_s=[100.,102.5],rows=len(frames),
        input_mesh_verified=dict(vertices=len(verts),faces=face_count,height_rounding_max_m=float(np.max(abs(grid[:,:,2]-dem)))),
        scope='Recorded state and retained collision-mesh geometry, no replay physics or contact-force oracle.',
        snapshots=snapshots,center_facet_changes=facet_changes,
        summary=dict(center_z_change_m=float(z[-1]-z[0]),center_z_min_m=float(z.min()),center_z_max_m=float(z.max()),
                     center_xy_dem_height_change_m=float(center_h[-1]-center_h[0]),
                     center_vertical_velocity_min_m_s=float(v[:,2].min()),center_vertical_velocity_max_m_s=float(v[:,2].max()),
                     center_dem_slope_min_deg=min(x['center_dem']['slope_deg'] for x in frames),
                     center_dem_slope_max_deg=max(x['center_dem']['slope_deg'] for x in frames),
                     center_xy_dem_material_height_rate_min_m_s=float(toprate.min()),center_xy_dem_material_height_rate_max_m_s=float(toprate.max())),
        qualifications=['Centre-underlying terrain is not necessarily the loaded contact location.',
                        'Contact points are positive-pressure patch means from saved final 1 ms solves; multiple contacts may put a mean between real contacts.',
                        'Changing terrain gradient is real mesh geometry but alone does not establish force/command causality or free fall.'])
    save('report.json',report)
    save('frames.json',frames)
    import matplotlib;matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    margin=.1;xs=np.linspace(q[:,0].min()-margin,q[:,0].max()+margin,100);ys=np.linspace(q[:,1].min()-margin,q[:,1].max()+margin,100)
    heights=np.array([[surface([x,y])['height_m'] for x in xs] for y in ys])
    ax=axes[0,0];im=ax.contourf(xs,ys,heights,15,cmap='terrain');ax.plot(q[:,0],q[:,1],color='cyan',label='Wheel centre XY');ax.plot(ref[:,0],ref[:,1],'--',color='black',label='Timed reference');ax.scatter([snapshots[3]['center_m'][0]],[snapshots[3]['center_m'][1]],color='red',label='101.5 s');ax.set_aspect('equal');ax.legend(fontsize=8);fig.colorbar(im,ax=ax,label='Collision DEM height (m)');ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)')
    ax=axes[0,1];ax.plot(times,z-z[0],label='Wheel-centre height change');ax.plot(times,center_h-center_h[0],label='DEM below centre height change');ax.set_ylabel('Height change (m)');ax.legend(fontsize=8)
    ax=axes[1,0];ax.plot(times,v[:,2],label='Centre vertical velocity');ax.plot(times,toprate,label='DEM height derivative along actual XY');ax.set_ylabel('m/s');ax.legend(fontsize=8)
    ax=axes[1,1];ax.plot(times,[x['center_dem']['slope_deg'] for x in frames],label='Facet slope below centre');ax.set_ylabel('degrees');ax.legend(fontsize=8)
    for ax in axes.flat:ax.grid(alpha=.3)
    for ax in [axes[0,1],axes[1,0],axes[1,1]]:ax.set_xlabel('Recorded time (s)');ax.axvline(101.5,color='red',alpha=.5)
    fig.suptitle('I-80: recorded eight -1 native, retained Gazebo DEM geometry\nCentre profile is not a contact-force or free-fall diagnosis')
    fig.tight_layout();fig.savefig(OUT/'terrain_profile.png',dpi=150);plt.close(fig)
    assert all(sha(ROOT/x['path'])==x['sha256'] for x in bound)
    save('artifact_manifest.json',dict(inputs_unchanged=True,no_new_physics=True,files=[dict(path=p.name,bytes=p.stat().st_size,sha256=sha(p)) for p in sorted(OUT.iterdir()) if p.is_file() and p.name!='artifact_manifest.json']))
    print(json.dumps(report['summary']))


if __name__=='__main__':main()
