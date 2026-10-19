#!/usr/bin/env python3
"""Recover saved SolidWorks display triangles locally; never edits CAD inputs.

Usage: python3 assets/cad/recover_saved_meshes.py /path/to/extracted/cad
The vendored MIT parser only decodes cached display triangles, not the B-rep.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "vendor"))
from sldprt.assembly import read_assembly  # noqa: E402
from sldprt.blocks import find_block, read_blocks  # noqa: E402
from sldprt.mesh import Mesh  # noqa: E402
from sldprt.savedmesh import read_saved_mesh  # noqa: E402

ASSEMBLY_CENTER_M = (0.70439327094328, 0.9565676112215376, 1.490231356601876)
PART_NAMES = ("Shell_version2_Part2", "Shell_version3", "Belt Module", "Gear_Plate_Motor_Mount")


def assembly_to_sim(p_mm):
    """Center in meters; align polar axis x and roller-array travel y."""
    p = [p_mm[i] * .001 - ASSEMBLY_CENTER_M[i] for i in range(3)]
    return (p[1], .5 * p[0] - math.sqrt(3) * .5 * p[2],
            -math.sqrt(3) * .5 * p[0] - .5 * p[2])


def write_obj(mesh, path, transform=None):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("# Saved SolidWorks display triangles; coordinates in meters.\n")
        for p in mesh.vertices:
            q = transform(p) if transform else tuple(v * .001 for v in p)
            f.write("v %.10g %.10g %.10g\n" % q)
        for t in mesh.triangles:
            f.write("f %d %d %d\n" % tuple(i + 1 for i in t))


def combine(meshes):
    out = Mesh()
    for m in meshes:
        offset = len(out.vertices)
        out.vertices.extend(m.vertices)
        out.triangles.extend(tuple(i + offset for i in t) for t in m.triangles)
    return out


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source_dir", type=Path)
    args = parser.parse_args()
    source = args.source_dir
    manifest = {
        "status": "Recovered cached display geometry, not a B-rep translation; visual use only.",
        "parser": {
            "repository": "https://github.com/XRTC5/sldprt-export",
            "commit": "ebde7524b3876c5d78c7e2248f2eba9b5725a6c0",
            "license": "MIT; vendored modules and license in vendor/",
        },
        "source_archive": "Active sphere2.zip",
        "input_policy": "Inputs read locally and read-only; no model uploads.",
        "mesh_units": "meter",
        "native_shell_axis": "+z is the shell symmetry axis, centered at part origin",
        "native_to_positive_carrier_axes": {
            "formula": "[x,y,z]_carrier = [z,y,-x]_native",
            "rotation_axis_angle_degrees": [0, 1, 0, 90],
            "translation_m": [0, 0, 0],
        },
        "native_to_negative_carrier_axes": {
            "formula": "[x,y,z]_carrier = [-z,y,x]_native",
            "rotation_axis_angle_degrees": [0, 1, 0, -90],
            "translation_m": [0, 0, 0],
        },
        "assembly_to_visual_coordinates": {
            "source_center_m": ASSEMBLY_CENTER_M,
            "formula": "[x,y,z]_visual = [DY,0.5*DX-sqrt(3)/2*DZ,-sqrt(3)/2*DX-0.5*DZ], D=assembly-center",
            "note": "Preserves saved relative placements. Shell symmetry axis is x. Observed polar roller-array travel is y, chosen parallel to the provisional common-shaft y axis. The unfinished common shaft is absent from CAD; its azimuth is inferred from the user's marked views.",
        },
        "parts": {},
        "measured_geometry_m": {
            "sphere_outer_radius": .120,
            "sphere_inner_radius": .110,
            "polar_aperture_outer_radius": .032,
            "polar_aperture_axial_coordinate": .1156546548,
            "polar_aperture_half_angle_degrees": math.degrees(math.asin(32 / 120)),
            "equatorial_rim_axial_coordinate": .010,
            "equatorial_open_gap_between_shell_pieces": .020,
            "note": "Dimensions measured from saved mesh vertices, not a manufacturing tolerance report.",
        },
        "limitations": [
            "SolidWorks display tessellation can be incomplete, coarse, or stale.",
            "The ring display mesh has open/nonmanifold edges; use only for display.",
            "No material, mass, inertia, gear efficiency, belt compliance, or motor capability is inferred from these meshes.",
            "MuJoCo convex mesh collision would close the polar aperture; these are visual meshes only.",
            "Carrier visual mesh omits detailed bearings and standoffs for rendering speed, plus parts lacking cached triangles.",
        ],
    }
    for name in PART_NAMES:
        path = source / (name + ".SLDPRT")
        data = path.read_bytes()
        blocks = read_blocks(data)
        mesh = read_saved_mesh(blocks)
        if mesh is None:
            raise ValueError(f"No saved display mesh for {path.name}")
        write_obj(mesh, HERE / "native_meshes" / (name + ".obj"))
        lo, hi = mesh.bbox()
        used = next((b for b in blocks if b.name == "Contents/DisplayLists"), None)
        manifest["parts"][name] = {
            "source_file": path.name,
            "source_sha256": digest(path),
            "vertices": len(mesh.vertices),
            "triangles": len(mesh.triangles),
            "bounds_m": [[v * .001 for v in p] for p in (lo, hi)],
            "open_or_nonmanifold_edges": mesh.open_edges(),
            "display_block_crc_valid": used.verify() if used else None,
            "native_mesh": "native_meshes/" + name + ".obj",
        }
    path = source / "Active sphere2.SLDASM"
    files = {p.name.lower(): p for p in source.iterdir() if p.is_file()}

    def resolve(name):
        p = files.get(name.lower())
        return p.read_bytes() if p else None

    blocks = read_blocks(path.read_bytes())
    _, components, summary = read_assembly(blocks, resolve=resolve)
    rows = []
    shell_positive, shell_negative, carrier = [], [], []
    for i, c in enumerate(components):
        row = dict(index=i, name=c.name, hidden=c.hidden, mirrored=c.mirrored,
                   vertices=len(c.mesh.vertices), triangles=len(c.mesh.triangles),
                   bounds_mm=c.mesh.bbox())
        rows.append(row)
        if c.hidden:
            continue
        is_shell = c.name.startswith("Shell_version")
        if is_shell:
            # Shell pieces share the assembly origin. Their axial midpoint
            # identifies which pole they face without relying on instance ids.
            lo, hi = c.mesh.bbox()
            positive = (lo[1] + hi[1]) * .0005 > ASSEMBLY_CENTER_M[1]
            (shell_positive if positive else shell_negative).append(c.mesh)
        elif "Bearing" not in c.name and "Standoff" not in c.name:
            carrier.append(c.mesh)
    for name, meshes in (("shell_positive", shell_positive), ("shell_negative", shell_negative), ("carrier", carrier)):
        mesh = combine(meshes)
        write_obj(mesh, HERE / "visuals" / (name + ".obj"), transform=assembly_to_sim)
        manifest.setdefault("visual_meshes", {})[name] = {
            "path": "visuals/" + name + ".obj",
            "triangles": len(mesh.triangles), "components": len(meshes),
            "coordinates": "centered carrier frame, shell axis x, roller-array travel y, provisional common shaft y, meters",
        }
    manifest["assembly"] = {
        "source_file": path.name, "source_sha256": digest(path),
        "parser_summary": summary.__dict__,
        "note": "Reference count includes subassemblies; four leaf components lack recovered display triangles.",
        "missing_leaf_geometry": ["3505-8-SHS.STEP.SLDPRT (two instances)", "____GIM3505-8GDZ34.STEP.SLDPRT (two instances)"],
        "mesh_cache_crc_valid": all(b.verify() for b in blocks if b.name.startswith("FaceTessellations/")),
    }
    (HERE / "assembly_components.json").write_text(json.dumps({"report": summary.__dict__, "components": rows}, indent=2) + "\n")
    (HERE / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest["visual_meshes"], indent=2))
    print(summary.summary())


if __name__ == "__main__":
    main()
