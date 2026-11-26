"""Post-selection geometry-only check of retained negative planarity result."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import types

import numpy as np
from scipy.spatial import ConvexHull
from spherical_wheel.gazebo.mars import MarsYard

HERE = Path(__file__).resolve().parent
BASE = HERE.parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + '\n')


def main():
    source = HERE / 'source'
    source.mkdir(exist_ok=False)
    for path in (Path(__file__), BASE / 'select_site.py'):
        shutil.copyfile(path, source / path.name)
    save(HERE / 'declaration_before_checks.json', dict(declared_utc=datetime.now(timezone.utc).isoformat(),
        scope='Post-selection saved-geometry integrity check, not candidate reselection or new physics.',
        sites='Old center and minimum-RMS clear candidate, chosen from completed geometric scan only.',
        independent_method='1mm regular XY grid inside protected polygon; native triangle heights; independently fit sample LS plane and compare exact area-moment result.',
        source_sha256={p.name: sha(p) for p in source.iterdir()},
        inputs_sha256={p.name: sha(p) for p in (BASE / 'candidates.json', BASE / 'report.json')}))
    module = types.ModuleType('_selection_geometry_source')
    module.__file__ = str(BASE / 'select_site.py')
    exec(compile((BASE / 'select_site.py').read_text(), module.__file__, 'exec'), module.__dict__)
    rows = json.loads((BASE / 'candidates.json').read_text())
    clear = [r for r in rows if not r['feature_blocked']]
    sites = [min(clear, key=lambda r: r['squared_distance_to_old_site']), min(clear, key=lambda r: r['plane_rmse_m'])]
    yard = MarsYard()
    checks = []
    for site in sites:
        center = np.array(site['center_xy_m'])
        _, _, polygon = module.capsule_polygon(center)
        lo, hi = polygon.min(axis=0), polygon.max(axis=0)
        x = np.arange(lo[0] + .0005, hi[0], .001)
        y = np.arange(lo[1] + .0005, hi[1], .001)
        xx, yy = np.meshgrid(x, y)
        xy = np.column_stack([xx.ravel(), yy.ravel()])
        hull = ConvexHull(polygon)
        keep = np.all(xy @ hull.equations[:, :2].T + hull.equations[:, 2] <= 1e-12, axis=1)
        xy = xy[keep]
        z = yard.heights_at(xy[:, 0], xy[:, 1])
        matrix = np.column_stack([xy - center, np.ones(len(xy))])
        coefficients = np.linalg.lstsq(matrix, z, rcond=None)[0]
        residual = z - matrix @ coefficients
        rms = float(np.linalg.norm(residual) / np.sqrt(len(residual)))
        peak = float(np.max(abs(residual)))
        slope = float(np.degrees(np.arctan(np.linalg.norm(coefficients[:2]))))
        assert abs(rms - site['plane_rmse_m']) < .0001
        assert abs(peak - site['plane_max_abs_m']) < .0003
        assert abs(slope - site['plane_slope_deg']) < .05
        assert rms > .001 and peak > .003
        checks.append(dict(center_xy_m=center.tolist(), samples=len(xy), sampled_rmse_m=rms,
            exact_area_rmse_m=site['plane_rmse_m'], sampled_max_abs_m=peak,
            exact_vertex_max_abs_m=site['plane_max_abs_m'], sampled_slope_deg=slope,
            exact_area_slope_deg=site['plane_slope_deg'], agrees_with_rejection=True))
    save(HERE / 'report.json', dict(passed=True, no_qualifying_site=True, checks=checks,
        interpretation='Independent dense-grid fits corroborate the retained negative site selection; no threshold, site, or control change.'))
    paths = [p for p in HERE.rglob('*') if p.is_file() and p.name != 'artifact_manifest.json']
    save(HERE / 'artifact_manifest.json', dict(files=[dict(path=str(p.relative_to(HERE)), bytes=p.stat().st_size, sha256=sha(p)) for p in sorted(paths)]))


if __name__ == '__main__':
    main()
