"""I46 geometry-only deterministic Mars site selection; no robot execution."""
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
from time import perf_counter

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np
from scipy.spatial import ConvexHull

from spherical_wheel.gazebo import mars

HERE = Path(__file__).resolve().parent
REPO = next(p for p in HERE.parents if (p / 'src/spherical_wheel').is_dir())
SETTINGS = dict(scan_bounds_xy_m=[-3., 1., 6., 11.], grid_step_m=.10,
    old_site_xy_m=[-1.6, 9.6], heading_deg=45., length_m=.18,
    protected_capsule_radius_m=.22, capsule_circle_sides=32,
    fitted_slope_deg_min=2., fitted_slope_deg_max=6., plane_rmse_m_max=.001,
    plane_max_abs_m_max=.003,
    feature_yaw_rad=1.5707963268,
    ranking=['squared_distance_to_old_site', 'plane_rmse_m', 'center_x', 'center_y'],
    planarity='Exact area moments over DEM triangles clipped to conservative circumscribed32-gon capsule; max residual at clipped vertices.',
    feature_rejection='Exact2D route-segment to transformed raw feature triangle projection distance<=.22m; degenerate projected edges retained.',
    nonqualification='No threshold changes or physics if zero sites qualify.')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')


def cross(a, b):
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def point_segment_distance(points, starts, ends):
    delta = ends - starts
    denom = np.sum(delta * delta, axis=-1)
    numerator = np.sum((points - starts) * delta, axis=-1)
    ratio = np.divide(numerator, denom,
                      out=np.zeros_like(numerator, dtype=float), where=denom > 1e-24)
    nearest = starts + np.clip(ratio, 0., 1.)[..., None] * delta
    return np.linalg.norm(points - nearest, axis=-1)


def segment_triangle_distances(start, end, triangles):
    """Exact Euclidean XY distance, including degenerate projected triangles."""
    t = np.asarray(triangles, float)
    if not len(t):
        return np.empty(0)
    edges, stops = t, np.roll(t, -1, axis=1)
    distance = np.minimum(point_segment_distance(t, start, end).min(axis=1),
        np.minimum(point_segment_distance(start, edges, stops).min(axis=1),
                   point_segment_distance(end, edges, stops).min(axis=1)))
    direction = end - start
    side = stops - edges
    denominator = cross(direction, side)
    q = edges - start
    good = np.abs(denominator) > 1e-14
    a = np.divide(cross(q, side), denominator, out=np.full_like(denominator, np.inf), where=good)
    b = np.divide(cross(q, direction), denominator, out=np.full_like(denominator, np.inf), where=good)
    intersects = np.any(good & (a >= -1e-12) & (a <= 1 + 1e-12) & (b >= -1e-12) & (b <= 1 + 1e-12), axis=1)
    nondegenerate = np.abs(cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])) > 1e-14
    for point in (start, end):
        signed = cross(side, point - edges)
        intersects |= nondegenerate & (np.all(signed >= -1e-12, axis=1) | np.all(signed <= 1e-12, axis=1))
    return np.where(intersects, 0., distance)


def capsule_polygon(center):
    heading = np.radians(SETTINGS['heading_deg'])
    direction = np.array([np.cos(heading), np.sin(heading)])
    start = center - .5 * SETTINGS['length_m'] * direction
    end = center + .5 * SETTINGS['length_m'] * direction
    count = SETTINGS['capsule_circle_sides']
    theta = 2 * np.pi * (np.arange(count) + .5) / count
    radius = SETTINGS['protected_capsule_radius_m'] / np.cos(np.pi / count)
    circle = radius * np.column_stack([np.cos(theta), np.sin(theta)])
    points = np.vstack([start + circle, end + circle])
    return start, end, points[ConvexHull(points).vertices]


def clip_triangle(triangle, polygon):
    points = [v.copy() for v in triangle]
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        if not points:
            break
        updated = []
        previous = points[-1]
        dp = cross(b - a, previous[:2] - a)
        for current in points:
            dc = cross(b - a, current[:2] - a)
            inside_p, inside_c = dp >= -1e-12, dc >= -1e-12
            if inside_p != inside_c:
                ratio = float(dp / (dp - dc))
                updated.append(previous + ratio * (current - previous))
            if inside_c:
                updated.append(current)
            previous, dp = current, dc
        points = updated
    return np.asarray(points, float).reshape(-1, 3)


def fit_triangles(triangles, center):
    triangles = np.asarray(triangles)
    area = .5 * np.abs(cross(triangles[:, 1, :2] - triangles[:, 0, :2], triangles[:, 2, :2] - triangles[:, 0, :2]))
    keep = area > 1e-14
    triangles, area = triangles[keep], area[keep]
    if not len(triangles):
        raise ValueError('Empty DEM intersection')
    # Center both XY and height to avoid subtracting metre-scale moments for a mm residual.
    z0 = float(np.mean(triangles[:, :, 2]))
    phi = np.concatenate([triangles[:, :, :2] - center, np.ones((len(triangles), 3, 1))], axis=2)
    z = triangles[:, :, 2] - z0
    sp, sz = phi.sum(axis=1), z.sum(axis=1)
    pp = (np.einsum('nki,nkj->nij', phi, phi) + np.einsum('ni,nj->nij', sp, sp)) / 12
    pz = (np.einsum('nki,nk->ni', phi, z) + sp * sz[:, None]) / 12
    zz = ((z * z).sum(axis=1) + sz * sz) / 12
    matrix = np.einsum('n,nij->ij', area, pp)
    vector = np.einsum('n,ni->i', area, pz)
    coefficients = np.linalg.solve(matrix, vector)
    square = float(np.dot(area, zz) - 2 * coefficients @ vector + coefficients @ matrix @ coefficients)
    rms = float(np.sqrt(max(0., square / area.sum())))
    residual = z - np.einsum('nki,i->nk', phi, coefficients)
    normal = np.r_[-coefficients[:2], 1.]
    normal /= np.linalg.norm(normal)
    direction = np.array([np.cos(np.radians(SETTINGS['heading_deg'])), np.sin(np.radians(SETTINGS['heading_deg']))])
    return dict(plane_point_m=[float(center[0]), float(center[1]), float(z0 + coefficients[2])],
        plane_normal=normal.tolist(), plane_gradient=coefficients[:2].tolist(),
        plane_slope_deg=float(np.degrees(np.arctan(np.linalg.norm(coefficients[:2])))),
        along_path_grade_deg=float(np.degrees(np.arctan(coefficients[:2] @ direction))),
        plane_rmse_m=rms, plane_max_abs_m=float(np.max(np.abs(residual))),
        clipped_area_m2=float(area.sum()), clipped_triangles=len(triangles))


def dem_corridor(yard, center, polygon):
    lo, hi = polygon.min(axis=0), polygon.max(axis=0)
    c0 = max(0, int(np.floor((lo[0] + yard.size / 2) / yard.spacing)))
    c1 = min(yard.count - 2, int(np.floor((hi[0] + yard.size / 2) / yard.spacing)))
    r0 = max(0, int(np.floor((yard.size / 2 - hi[1]) / yard.spacing)))
    r1 = min(yard.count - 2, int(np.floor((yard.size / 2 - lo[1]) / yard.spacing)))
    clipped = []
    for r in range(r0, r1 + 1):
        for c in range(c0, c1 + 1):
            vertices = np.array([[-yard.size / 2 + (c + dc) * yard.spacing,
                                  yard.size / 2 - (r + dr) * yard.spacing, yard.heights[r + dr, c + dc]]
                                 for dr, dc in ((0, 0), (0, 1), (1, 1), (1, 0))])
            for indices in ((0, 1, 2), (0, 2, 3)):
                cut = clip_triangle(vertices[list(indices)], polygon)
                clipped.extend(np.array([cut[0], cut[k], cut[k + 1]]) for k in range(1, len(cut) - 1))
    return fit_triangles(clipped, center)


def read_obj(path):
    vertices, faces = [], []
    for line in path.read_text().splitlines():
        if line.startswith('v '):
            vertices.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            indices = [int(x.split('/')[0]) for x in line.split()[1:]]
            indices = [i - 1 if i > 0 else len(vertices) + i for i in indices]
            faces.extend((indices[0], indices[k], indices[k + 1]) for k in range(1, len(indices) - 1))
    return np.asarray(vertices), np.asarray(faces, int)


def validation_checks():
    triangle = np.array([[[0., 0.], [1., 0.], [0., 1.]], [[2., 0.], [2., 0.], [2., 1.]]])
    np.testing.assert_allclose(segment_triangle_distances(np.array([.2, .2]), np.array([.4, .4]), triangle), [0., 1.6])
    assert segment_triangle_distances(np.array([1.5, .5]), np.array([2.5, .5]), triangle)[1] == 0
    polygon = np.array([[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]])
    t = np.array([[-2., -2., .3], [2., -2., 1.1], [0., 2., .3]])
    cut = clip_triangle(t, polygon)
    assert np.max(np.abs(cut[:, :2])) <= 1. + 1e-12
    square = np.array([[[-1., -1., 0.], [1., -1., .2], [1., 1., .6]], [[-1., -1., 0.], [1., 1., .6], [-1., 1., .4]]])
    fit = fit_triangles(square, np.zeros(2))
    np.testing.assert_allclose(fit['plane_gradient'], [.1, .2], atol=1e-14)
    assert fit['plane_rmse_m'] < 1e-8 and fit['plane_max_abs_m'] < 1e-14
    assert abs(fit['clipped_area_m2'] - 4) < 1e-14
    start, end, polygon = capsule_polygon(np.zeros(2))
    for endpoint in (start, end):
        circle = endpoint + .22 * np.column_stack([np.cos(np.linspace(0, 2 * np.pi, 1025)), np.sin(np.linspace(0, 2 * np.pi, 1025))])
        assert all(np.min(cross(b - a, circle - a)) >= -1e-12 for a, b in zip(polygon, np.roll(polygon, -1, axis=0)))
    return dict(passed=True, checks=['route_inside_triangle', 'degenerate_projected_edge_distance_and_intersection',
        'clipped_vertex_bounds', 'exact_planar_moments_gradient_area', 'capsule_conservative_circle_containment'])


def main():
    stage = HERE / 'development/stage01'
    stage.mkdir(parents=True, exist_ok=False)
    source = stage / 'source'
    source.mkdir()
    for path in (Path(__file__), Path(mars.__file__)):
        shutil.copyfile(path, source / path.name)
    assets = [mars.DEM] + [mars.YARD / 'meshes' / (name + '.obj') for name, _ in mars.FEATURES]
    save(stage / 'declaration_before_scan.json', dict(declared_utc=datetime.now(timezone.utc).isoformat(), settings=SETTINGS,
        sources=[dict(path=str(p), sha256=sha(p)) for p in (Path(__file__), Path(mars.__file__))],
        assets=[dict(path=str(p), sha256=sha(p)) for p in assets],
        scope='Geometry-only selection before control outcomes; no controller, physics, observer or saved tracking data read.'))
    save(stage / 'helper_checks.json', validation_checks())
    started = perf_counter()
    yard = mars.MarsYard()
    triangles = []
    feature_sources = []
    for name, dz in mars.FEATURES:
        path = mars.YARD / 'meshes' / (name + '.obj')
        vertices, faces = read_obj(path)
        c, s = np.cos(SETTINGS['feature_yaw_rad']), np.sin(SETTINGS['feature_yaw_rad'])
        vertices = np.column_stack([c * vertices[:, 0] - s * vertices[:, 1], s * vertices[:, 0] + c * vertices[:, 1], vertices[:, 2] + dz])
        triangles.append(vertices[faces, :2])
        feature_sources.append(dict(name=name, yaw_rad=SETTINGS['feature_yaw_rad'], z_offset_m=dz, triangles=len(faces), sha256=sha(path)))
    features = np.concatenate(triangles)
    low, high = features.min(axis=1), features.max(axis=1)
    x0, x1, y0, y1 = SETTINGS['scan_bounds_xy_m']
    xs = np.linspace(x0, x1, round((x1 - x0) / SETTINGS['grid_step_m']) + 1)
    ys = np.linspace(y0, y1, round((y1 - y0) / SETTINGS['grid_step_m']) + 1)
    rows = []
    for x in xs:
        for y in ys:
            center = np.array([x, y])
            start, end, polygon = capsule_polygon(center)
            radius = SETTINGS['protected_capsule_radius_m']
            lo, hi = np.minimum(start, end) - radius, np.maximum(start, end) + radius
            mask = np.all(high >= lo, axis=1) & np.all(low <= hi, axis=1)
            distances = segment_triangle_distances(start, end, features[mask])
            blocked = bool(len(distances) and distances.min() <= radius + 1e-12)
            row = dict(center_xy_m=center.tolist(), start_xy_m=start.tolist(), end_xy_m=end.tolist(),
                heading_deg=SETTINGS['heading_deg'], length_m=SETTINGS['length_m'],
                squared_distance_to_old_site=float(np.sum((center - SETTINGS['old_site_xy_m']) ** 2)),
                feature_blocked=blocked, rejection_reasons=[])
            if blocked:
                row['rejection_reasons'].append('feature_triangle_projection_within_protected_capsule')
            else:
                fit = dem_corridor(yard, center, polygon)
                row.update(fit)
                if not SETTINGS['fitted_slope_deg_min'] <= fit['plane_slope_deg'] <= SETTINGS['fitted_slope_deg_max']:
                    row['rejection_reasons'].append('fitted_slope_outside_2_to_6_deg')
                if fit['plane_rmse_m'] > SETTINGS['plane_rmse_m_max']:
                    row['rejection_reasons'].append('plane_RMS_above_1mm')
                if fit['plane_max_abs_m'] > SETTINGS['plane_max_abs_m_max']:
                    row['rejection_reasons'].append('plane_max_abs_above_3mm')
            row['qualified'] = not row['rejection_reasons']
            rows.append(row)
    qualified = sorted((r for r in rows if r['qualified']), key=lambda r: (r['squared_distance_to_old_site'], r['plane_rmse_m'], *r['center_xy_m']))
    selected = dict(qualified[0]) if qualified else None
    if selected:
        selected['minimum_feature_clearance_m'] = float(segment_triangle_distances(np.asarray(selected['start_xy_m']), np.asarray(selected['end_xy_m']), features).min())
        assert selected['minimum_feature_clearance_m'] > .22
    save(HERE / 'candidates.json', rows)
    report = dict(iteration='I-46', status='site_selected_geometry_only' if selected else 'no_qualifying_site_no_physics',
        selected=selected, settings=SETTINGS, candidates=len(rows), qualified=len(qualified),
        rejected=len(rows) - len(qualified), rejection_counts=dict(Counter(reason for r in rows for reason in r['rejection_reasons'])),
        feature_rejected_planarity_not_evaluated=sum(r['feature_blocked'] for r in rows),
        feature_sources=feature_sources, DEM=dict(path=str(mars.DEM), sha256=sha(mars.DEM), samples=yard.count, spacing_m=yard.spacing,
            collision_rule='Native MarsYard diagonal from (row,col) to (row+1,col+1), no visual heightmap interpolation.'),
        ranking_rule=SETTINGS['ranking'], elapsed_s=perf_counter() - started,
        limitations=['Geometry selection does not guarantee actual shell/belt force transfers or controller success.',
            'Fit is a local terrain geometry description, not a source available to the selected observer-only controller.',
            'Protected footprint and feature XY rejection are conservative; no control outcome was read.'])
    save(HERE / 'report.json', report)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    allxy = np.array([r['center_xy_m'] for r in rows]); passes = np.array([r['qualified'] for r in rows])
    for ax, bounds in zip(axes, ([x0 - .3, x1 + .3, y0 - .3, y1 + .3],
                                  [selected['center_xy_m'][0] - .7, selected['center_xy_m'][0] + .7, selected['center_xy_m'][1] - .7, selected['center_xy_m'][1] + .7] if selected else [x0, x1, y0, y1])):
        xx, yy = np.linspace(bounds[0], bounds[1], 220), np.linspace(bounds[2], bounds[3], 220)
        image = ax.imshow(yard.grid(xx, yy), origin='lower', extent=bounds, cmap='terrain', aspect='equal')
        visible = np.all(high >= np.array(bounds)[[0, 2]], axis=1) & np.all(low <= np.array(bounds)[[1, 3]], axis=1)
        ax.add_collection(PolyCollection(features[visible], facecolor='black', edgecolor='none', alpha=.4))
        ax.scatter(allxy[~passes, 0], allxy[~passes, 1], s=3, c='gray', alpha=.4)
        if passes.any():
            ax.scatter(allxy[passes, 0], allxy[passes, 1], s=7, c='limegreen', label='qualified geometry')
        ax.scatter(*SETTINGS['old_site_xy_m'], marker='x', c='blue', s=70, label='previous region')
        if selected:
            start, end, polygon = capsule_polygon(np.asarray(selected['center_xy_m']))
            ax.fill(polygon[:, 0], polygon[:, 1], facecolor='none', edgecolor='red', linewidth=2)
            ax.plot([start[0], end[0]], [start[1], end[1]], c='red', linewidth=3)
            ax.scatter(*start, c='white', edgecolor='red', s=35, zorder=6)
        ax.set(xlim=bounds[:2], ylim=bounds[2:], xlabel='Mars world X (m)', ylabel='Mars world Y (m)')
    axes[0].set_title('Fixed scan; terrain geometry only')
    axes[1].set_title('Selected .18m out/back and .22m protected radius' if selected else 'No site passes fixed thresholds')
    axes[0].legend(fontsize=8, loc='lower left')
    fig.colorbar(image, ax=axes, label='DEM collision height (m)', shrink=.8)
    fig.savefig(HERE / 'map.png', dpi=160)
    plt.close(fig)
    for path in assets:
        expected = next(r['sha256'] for r in json.loads((stage / 'declaration_before_scan.json').read_text())['assets'] if r['path'] == str(path))
        assert sha(path) == expected
    paths = [p for p in HERE.rglob('*') if p.is_file() and p.name != 'artifact_manifest.json']
    save(HERE / 'artifact_manifest.json', dict(files=[dict(path=str(p.relative_to(HERE)), bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(paths)]))


if __name__ == '__main__':
    main()
