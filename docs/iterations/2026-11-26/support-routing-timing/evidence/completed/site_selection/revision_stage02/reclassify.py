"""Authorized geometry-only revision: same measured sites, relaxed planarity."""
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import types

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
import numpy as np
from spherical_wheel.gazebo import mars

HERE = Path(__file__).resolve().parent
BASE = HERE.parent
CRITERIA = dict(fitted_slope_deg_min=2., fitted_slope_deg_max=6.,
    plane_rmse_m_max=.003, plane_max_abs_m_max=.010,
    protected_capsule_radius_m=.22,
    ranking=['squared_distance_to_old_site', 'plane_rmse_m', 'center_x', 'center_y'])


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')


def main():
    source = HERE / 'source'
    source.mkdir(exist_ok=False)
    for path in (Path(__file__), BASE / 'select_site.py'):
        shutil.copyfile(path, source / path.name)
    inputs = [BASE / 'candidates.json', BASE / 'report.json', BASE / 'review_stage01/report.json']
    before = {str(path): sha(path) for path in inputs}
    save(HERE / 'declaration_before_reclassification.json', dict(
        declared_utc=datetime.now(timezone.utc).isoformat(), criteria=CRITERIA,
        scope='Reclassify SAME2091 measured geometry candidates before any I46 control outcomes; no new grid, controller, observer or physics.',
        rationale='Strict1mm/3mm corridor plane criteria produced no candidate and independent dense-grid checks corroborated the geometry. Natural Mars3mm/10mm limits are about half old-site residuals; require paired flat control for timing analysis.',
        input_sha256=before, source_sha256={p.name: sha(p) for p in source.iterdir()},
        source_stage_retained='Original report/candidates and strict criteria remain unchanged.'))
    original = json.loads((BASE / 'candidates.json').read_text())
    rows = []
    for old in original:
        row = dict(old)
        row['strict_stage01_qualified'] = row.pop('qualified')
        row['strict_stage01_rejection_reasons'] = row.pop('rejection_reasons')
        reasons = []
        if row['feature_blocked']:
            reasons.append('feature_triangle_projection_within_protected_capsule')
        else:
            if not 2. <= row['plane_slope_deg'] <= 6.:
                reasons.append('fitted_slope_outside_2_to_6_deg')
            if row['plane_rmse_m'] > .003:
                reasons.append('plane_RMS_above_3mm')
            if row['plane_max_abs_m'] > .010:
                reasons.append('plane_max_abs_above_10mm')
        row.update(qualified=not reasons, rejection_reasons=reasons)
        rows.append(row)
    qualified = sorted((r for r in rows if r['qualified']), key=lambda r: (r['squared_distance_to_old_site'], r['plane_rmse_m'], *r['center_xy_m']))
    selected = dict(qualified[0]) if qualified else None
    module = types.ModuleType('_strict_selection_geometry')
    module.__file__ = str(BASE / 'select_site.py')
    exec(compile((BASE / 'select_site.py').read_text(), module.__file__, 'exec'), module.__dict__)
    features = []
    assets = []
    for name, dz in mars.FEATURES:
        path = mars.YARD / 'meshes' / (name + '.obj')
        vertices, faces = module.read_obj(path)
        c, s = np.cos(1.5707963268), np.sin(1.5707963268)
        vertices = np.column_stack([c * vertices[:, 0] - s * vertices[:, 1], s * vertices[:, 0] + c * vertices[:, 1], vertices[:, 2] + dz])
        features.append(vertices[faces, :2])
        assets.append(dict(path=str(path), sha256=sha(path)))
    features = np.concatenate(features)
    if selected:
        distance = float(module.segment_triangle_distances(np.array(selected['start_xy_m']), np.array(selected['end_xy_m']), features).min())
        assert distance > .22
        selected['minimum_feature_clearance_m'] = distance
        selected['feature_clearance_definition'] = 'Exact minimum XY route-center-segment to raw transformed feature triangle distance; clearance beyond protected capsule is distance-.22m.'
        selected['feature_clearance_beyond_capsule_m'] = distance - .22
    save(HERE / 'candidates.json', rows)
    report = dict(iteration='I-46', stage='revision_stage02',
        status='site_selected_geometry_only' if selected else 'no_qualifying_site_no_physics',
        selected=selected, criteria=CRITERIA, candidates=len(rows), qualified=len(qualified),
        rejected=len(rows) - len(qualified),
        rejection_counts=dict(Counter(reason for r in rows for reason in r['rejection_reasons'])),
        strict_stage01_qualified=0, original_inputs_sha256=before, feature_assets=assets,
        method='No geometry re-fit and no new site grid; only fixed criteria reclassification. Exact selected feature clearance evaluated from same raw transformed triangles.',
        limitations=['Fitted2–6deg local slope does not mean every DEM facet has that grade.',
            'Natural Mars is not an exact plane;3mm RMS/10mm peak are setup criteria, not controller acceptance gates.',
            'Geometry-only selection does not guarantee actual shell/belt force transfers or good control.'])
    save(HERE / 'report.json', report)
    yard = mars.MarsYard()
    allxy = np.array([r['center_xy_m'] for r in rows])
    passes = np.array([r['qualified'] for r in rows])
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    bounds_all = [-3.3, 1.3, 5.7, 11.3]
    bounds_detail = [selected['center_xy_m'][0] - .7, selected['center_xy_m'][0] + .7,
                     selected['center_xy_m'][1] - .7, selected['center_xy_m'][1] + .7] if selected else bounds_all
    low, high = features.min(axis=1), features.max(axis=1)
    for ax, bounds in zip(axes, (bounds_all, bounds_detail)):
        xx, yy = np.linspace(bounds[0], bounds[1], 220), np.linspace(bounds[2], bounds[3], 220)
        image = ax.imshow(yard.grid(xx, yy), origin='lower', extent=bounds, cmap='terrain', aspect='equal')
        mask = np.all(high >= np.array(bounds)[[0, 2]], axis=1) & np.all(low <= np.array(bounds)[[1, 3]], axis=1)
        ax.add_collection(PolyCollection(features[mask], facecolor='black', edgecolor='none', alpha=.4))
        ax.scatter(allxy[~passes, 0], allxy[~passes, 1], s=3, c='gray', alpha=.4)
        if passes.any():
            ax.scatter(allxy[passes, 0], allxy[passes, 1], s=10, c='limegreen', label='revised geometry qualifies')
        ax.scatter(-1.6, 9.6, marker='x', c='blue', s=65, label='previous region')
        if selected:
            start, end, polygon = module.capsule_polygon(np.asarray(selected['center_xy_m']))
            ax.fill(polygon[:, 0], polygon[:, 1], edgecolor='red', facecolor='none', linewidth=2)
            ax.plot([start[0], end[0]], [start[1], end[1]], c='red', linewidth=3)
            ax.scatter(*start, c='white', edgecolor='red', zorder=6, s=40)
        ax.set(xlim=bounds[:2], ylim=bounds[2:], xlabel='Mars world X (m)', ylabel='Mars world Y (m)')
    axes[0].set_title('Same fixed scan; revised geometry criteria')
    axes[1].set_title('Selected natural-Mars .18m out/back; no control outcomes')
    axes[0].legend(fontsize=8, loc='lower left')
    fig.colorbar(image, ax=axes, label='DEM collision height (m)', shrink=.8)
    fig.savefig(HERE / 'map.png', dpi=160)
    plt.close(fig)
    for path in inputs:
        assert sha(path) == before[str(path)]
    paths = [p for p in HERE.rglob('*') if p.is_file() and p.name != 'artifact_manifest.json']
    save(HERE / 'artifact_manifest.json', dict(files=[dict(path=str(p.relative_to(HERE)), bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(paths)]))


if __name__ == '__main__':
    main()
