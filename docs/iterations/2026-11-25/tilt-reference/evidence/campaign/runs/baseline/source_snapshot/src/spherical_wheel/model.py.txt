"""Generate the declared analytical MuJoCo plant and companion URDF.

The model uses rigid fork/platform geometry, a shaft revolute about fork y,
and independent shell revolutes about carrier x. No body is fixed to world.
Shell collision wedges stop at the equator and polar aperture. Their convex
hulls therefore cannot silently close either aperture. Belt caps belong to
the carrier; their prescribed surface speeds are applied by the simulator.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = REPO_ROOT / "config" / "model.yaml"


def load_config(path: str | Path | None = None) -> dict:
    """Read a complete model configuration in SI units."""
    with Path(path or DEFAULT_CONFIG).open() as stream:
        config = yaml.safe_load(stream)
    _validate_config(config)
    return config


def _validate_config(config: dict) -> None:
    g, s = config["geometry"], config["simulation"]
    if not 0 < g["shell_thickness_m"] < g["sphere_radius_m"]:
        raise ValueError("Shell thickness must be between zero and the sphere radius")
    if not 5 <= g["polar_cap_angle_deg"] <= 60:
        raise ValueError("Polar cap opening angle must be in [5, 60] degrees")
    if not 0 <= g.get("equatorial_gap_half_m", 0) < g["sphere_radius_m"] / 2:
        raise ValueError("Equatorial half gap must be nonnegative and less than half the radius")
    if g["collision_azimuth_sectors"] < 12 or g["collision_meridian_samples"] < 6:
        raise ValueError("At least 12 azimuth sectors and 6 meridian samples are required")
    arm_inner = g["fork"]["arm_offset_y_m"] - g["fork"]["arm_size_xyz_m"][1] / 2
    bridge_bottom = g["fork"]["bridge_center_z_m"] - g["fork"]["bridge_size_xyz_m"][2] / 2
    if min(arm_inner, bridge_bottom) <= g["sphere_radius_m"]:
        raise ValueError("Fork arms and bridge must clear the rotating sphere")
    if s["timestep_s"] <= 0 or s["control_interval_s"] < s["timestep_s"]:
        raise ValueError("Physics and controller intervals must be positive and compatible")
    steps = s["control_interval_s"] / s["timestep_s"]
    if not math.isclose(steps, round(steps), rel_tol=0, abs_tol=1e-9):
        raise ValueError("Controller interval must be an integer number of physics steps")
    for body in (g["fork"], g["platform"], g["carrier"]):
        if body["mass_kg"] <= 0:
            raise ValueError("All rigid-body masses must be positive")
    if min(g["shell_mass_each_kg"], g["polar_module_mass_each_kg"]) <= 0:
        raise ValueError("Shell and polar module masses must be positive")


def _fmt(values) -> str:
    if np.isscalar(values):
        return f"{float(values):.12g}"
    return " ".join(f"{float(v):.12g}" for v in values)


def _box_inertia(mass: float, size) -> np.ndarray:
    x, y, z = size
    return np.diag(np.array([y*y + z*z, x*x + z*z, x*x + y*y]) * mass / 12)


def _combine(parts: list[tuple[float, np.ndarray, np.ndarray]]) -> dict:
    mass = sum(p[0] for p in parts)
    com = sum(m * np.asarray(p) for m, p, _ in parts) / mass
    inertia = np.zeros((3, 3))
    for m, p, tensor in parts:
        d = np.asarray(p) - com
        inertia += tensor + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    values = np.linalg.eigvalsh(inertia)
    if min(values) <= 0 or max(values) > sum(values) - max(values) + 1e-10:
        raise ValueError("Mass properties violate positive inertia or triangle inequality")
    return {"mass_kg": float(mass), "com_m": com.tolist(), "inertia_kg_m2": inertia.tolist()}


def mass_properties(config: dict | None = None) -> dict:
    """Compute COM tensors, retaining parallel-axis terms and mass accounting."""
    c = config or load_config()
    g = c["geometry"]
    f, p = g["fork"], g["platform"]
    box_sizes = [f["arm_size_xyz_m"], f["arm_size_xyz_m"], f["bridge_size_xyz_m"]]
    box_centers = [[0, -f["arm_offset_y_m"], f["arm_center_z_m"]],
                   [0, f["arm_offset_y_m"], f["arm_center_z_m"]],
                   [0, 0, f["bridge_center_z_m"]]]
    volumes = np.array([np.prod(v) for v in box_sizes])
    fork_masses = volumes / sum(volumes) * f["mass_kg"]
    upper_parts = [(m, np.asarray(pos), _box_inertia(m, size))
                   for m, pos, size in zip(fork_masses, box_centers, box_sizes)]
    upper_parts.append((p["mass_kg"], np.asarray(p["center_xyz_m"]) + p["com_offset_xyz_m"],
                        _box_inertia(p["mass_kg"], p["size_xyz_m"])))
    result = {"fork": _combine(upper_parts)}
    radius, thickness = g["sphere_radius_m"], g["shell_thickness_m"]
    alpha = math.radians(g["polar_cap_angle_deg"])
    module_radius = radius * math.sin(alpha)
    module_length = radius * (1 - math.cos(alpha)) + g["polar_module_internal_depth_m"]
    module_com_x = radius - module_length / 2
    module_mass = g["polar_module_mass_each_kg"]
    module_inertia = np.diag([module_mass * module_radius**2 / 2,
                              module_mass * (3*module_radius**2 + module_length**2) / 12,
                              module_mass * (3*module_radius**2 + module_length**2) / 12])
    carrier = g["carrier"]
    result["carrier"] = _combine([
        (carrier["mass_kg"], np.zeros(3), _box_inertia(carrier["mass_kg"], carrier["size_xyz_m"])),
        (module_mass, np.array([module_com_x, 0, 0]), module_inertia),
        (module_mass, np.array([-module_com_x, 0, 0]), module_inertia)])
    # Uniform shell volume: r in [ri,R], x in [gap,r*cos(alpha)].
    ri = radius - thickness
    ca = math.cos(alpha)
    gap = g.get("equatorial_gap_half_m", 0.0)
    integral = lambda power: (radius**(power+1)-ri**(power+1))/(power+1)
    volume_factor = ca*integral(2)-gap*integral(1)
    shell_com = (ca**2*integral(3)-gap**2*integral(1))/(2*volume_factor)
    er2 = (ca*integral(4)-gap*integral(3))/volume_factor
    ex2 = (ca**3*integral(4)-gap**3*integral(1))/(3*volume_factor)
    m = g["shell_mass_each_kg"]
    ixx = m * (er2-ex2)
    iyy = m * (er2+ex2)/2 - m * shell_com**2
    for name, sign in (("shell_A", 1), ("shell_B", -1)):
        result[name] = _combine([(m, np.array([sign*shell_com, 0, 0]), np.diag([ixx, iyy, iyy]))])
    result["total_mass_kg"] = sum(result[name]["mass_kg"] for name in ("fork", "carrier", "shell_A", "shell_B"))
    result["policies"] = {
        "fork": "Uniform-density three fork boxes plus uniform platform box; full parallel-axis combination.",
        "carrier": "Central uniform solid box plus two equivalent solid cylinders along x for fixed belt modules.",
        "shells": "Uniform spherical shell volume with configured inner/outer radii, axial equatorial cut and conical polar aperture; exact analytic inertia for that approximation.",
        "platform_com_offset": "Moves the equivalent platform box and its COM together for mass-property calculation.",
        "actuation": "Equivalent output torque; planetary gearing, rotor inertia and belt motor load omitted.",
    }
    return result


def _write_obj(path: Path, vertices, faces) -> None:
    with path.open("w") as out:
        out.write("# Analytical collision geometry in meters; not a native CAD export.\n")
        for v in vertices:
            out.write("v " + _fmt(v) + "\n")
        for face in faces:
            out.write("f " + " ".join(str(i+1) for i in face) + "\n")


def _shell_wedge(radius: float, inner: float, alpha: float, nmer: int, az0: float, az1: float, sign: int, gap: float):
    vertices = []
    for r in (radius, inner):
        for az in (az0, az1):
            for theta in np.linspace(alpha, math.acos(gap/r), nmer+1):
                vertices.append([sign*r*math.cos(theta), r*math.sin(theta)*math.cos(az), r*math.sin(theta)*math.sin(az)])
    # Mesh faces describe the actual wedge surface. MuJoCo collides its convex
    # hull; hull inflation stays strictly inside this sector's bounding planes.
    n = nmer+1
    faces = []
    for layer in (0, 1):
        base = 2*n*layer
        for j in range(nmer):
            new_faces = [(base+j, base+n+j, base+n+j+1), (base+j, base+n+j+1, base+j+1)]
            faces.extend([tuple(reversed(face)) if layer == 0 else face for face in new_faces])
    for side in (0, 1):
        base = n*side
        for j in range(nmer):
            new_faces = [(base+j, base+j+1, base+2*n+j+1), (base+j, base+2*n+j+1, base+2*n+j)]
            faces.extend([tuple(reversed(face)) if side == 0 else face for face in new_faces])
    for j in (0, nmer):
        new_faces = [(j, j+2*n, j+3*n), (j, j+3*n, j+n)]
        faces.extend([tuple(reversed(face)) if j == 0 else face for face in new_faces])
    if sign < 0:
        faces = [tuple(reversed(face)) for face in faces]
    return vertices, faces


def _polar_cap(radius: float, alpha: float, nmer: int, naz: int, sign: int):
    vertices = [[sign*radius, 0, 0]]
    for theta in np.linspace(alpha/nmer, alpha, nmer):
        for az in np.arange(naz) * 2*math.pi/naz:
            vertices.append([sign*radius*math.cos(theta), radius*math.sin(theta)*math.cos(az), radius*math.sin(theta)*math.sin(az)])
    faces = []
    for j in range(naz):
        faces.append((0, 1+j, 1+(j+1)%naz))
    for i in range(nmer-1):
        a, b = 1+i*naz, 1+(i+1)*naz
        for j in range(naz):
            k = (j+1)%naz
            faces.extend([(a+j, b+j, b+k), (a+j, b+k, a+k)])
    # Closing rim disk is internal to the wheel, behind the exposed surface.
    for j in range(1, naz-1):
        start = 1+(nmer-1)*naz
        faces.append((start, start+j, start+j+1))
    if sign < 0:
        faces = [tuple(reversed(face)) for face in faces]
    return vertices, faces


def _make_meshes(config: dict, output_dir: Path) -> dict:
    g = config["geometry"]
    mesh_dir = output_dir / "meshes"
    mesh_dir.mkdir(parents=True, exist_ok=True)
    radius, n, k = g["sphere_radius_m"], int(g["collision_azimuth_sectors"]), int(g["collision_meridian_samples"])
    alpha = math.radians(g["polar_cap_angle_deg"])
    meshes = {"shell_A": [], "shell_B": [], "carrier": []}
    for body, sign in (("shell_A", 1), ("shell_B", -1)):
        for sector in range(n):
            name = f"{body}_sector_{sector:03d}"
            vertices, faces = _shell_wedge(radius, radius-g["shell_thickness_m"], alpha, k,
                                            2*math.pi*sector/n, 2*math.pi*(sector+1)/n, sign, g.get("equatorial_gap_half_m", 0.0))
            file = Path("meshes") / (name + ".obj")
            _write_obj(output_dir / file, vertices, faces)
            meshes[body].append({"name": name, "file": str(file)})
        name = body.replace("shell", "belt")
        vertices, faces = _polar_cap(radius, alpha, max(8, k//2), n, sign)
        file = Path("meshes") / (name + ".obj")
        _write_obj(output_dir / file, vertices, faces)
        meshes["carrier"].append({"name": name, "file": str(file)})
    return meshes


def _native_visuals(config: dict, output_dir: Path) -> dict:
    """Copy decoded native saved display meshes, with no collision or mass role."""
    if not config.get("visual", {}).get("native_cad_meshes", False):
        return {}
    sources = {"shell_A": "shell_positive.obj", "shell_B": "shell_negative.obj"}
    if config.get("visual", {}).get("include_carrier_details", True):
        sources["carrier"] = "carrier.obj"
    result = {}
    for body, filename in sources.items():
        source = REPO_ROOT / "assets" / "cad" / "visuals" / filename
        if not source.exists():
            continue  # Analytical geometry is a complete self-contained fallback.
        file = Path("meshes") / ("native_" + filename)
        target = output_dir / file
        if not target.exists() or target.stat().st_size != source.stat().st_size or target.stat().st_mtime < source.stat().st_mtime:
            shutil.copy2(source, target)
        result[body] = {"name": "native_"+body, "file": str(file)}
    return result


def _inertial_mjcf(body: ET.Element, props: dict) -> None:
    inertia = np.asarray(props["inertia_kg_m2"])
    ET.SubElement(body, "inertial", pos=_fmt(props["com_m"]), mass=_fmt(props["mass_kg"]),
                  fullinertia=_fmt([inertia[0,0], inertia[1,1], inertia[2,2], inertia[0,1], inertia[0,2], inertia[1,2]]))


def _fork_shapes(config: dict) -> list[dict]:
    f, p = config["geometry"]["fork"], config["geometry"]["platform"]
    return [
        {"name": "fork_arm_left", "size": f["arm_size_xyz_m"], "pos": [0, f["arm_offset_y_m"], f["arm_center_z_m"]]},
        {"name": "fork_arm_right", "size": f["arm_size_xyz_m"], "pos": [0, -f["arm_offset_y_m"], f["arm_center_z_m"]]},
        {"name": "fork_bridge", "size": f["bridge_size_xyz_m"], "pos": [0, 0, f["bridge_center_z_m"]]},
        {"name": "platform", "size": p["size_xyz_m"], "pos": p["center_xyz_m"]},
    ]


def generate_model(config: dict | None = None, output_dir: str | Path | None = None) -> Path:
    """Write MJCF, collision OBJ pieces, URDF and auditable mass properties."""
    c = copy.deepcopy(config) if config is not None else load_config()
    _validate_config(c)
    out = Path(output_dir or REPO_ROOT / "generated").resolve()
    out.mkdir(parents=True, exist_ok=True)
    g, s = c["geometry"], c["simulation"]
    props, meshes = mass_properties(c), _make_meshes(c, out)
    native = _native_visuals(c, out)
    root = ET.Element("mujoco", model="active_spherical_wheel_approximate")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true", inertiafromgeom="false", balanceinertia="false")
    ET.SubElement(root, "option", timestep=_fmt(s["timestep_s"]), gravity=_fmt(s["gravity_m_s2"]),
                  integrator="implicitfast", solver="Newton", iterations="60", tolerance="1e-10", cone="elliptic")
    ET.SubElement(root, "size", njmax="3000", nconmax="1000", nuser_geom="5")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "joint", damping=_fmt(s["joint_damping_Nms"]), armature="0")
    ET.SubElement(defaults, "geom", density="0", friction=_fmt(c["contact"]["friction"]),
                  condim=str(c["contact"]["contact_dimension"]), solref=_fmt(c["contact"]["solref"]),
                  solimp=_fmt(c["contact"]["solimp"]), contype="1", conaffinity="2")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="960", azimuth="130", elevation="-20")
    ET.SubElement(visual, "quality", shadowsize="2048")
    ET.SubElement(visual, "headlight", ambient="0.35 0.35 0.35", diffuse="0.65 0.65 0.65", specular="0.15 0.15 0.15")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", type="skybox", builtin="gradient", rgb1="0.90 0.94 0.98", rgb2="0.55 0.64 0.74", width="512", height="3072")
    ET.SubElement(asset, "texture", name="floor_tex", type="2d", builtin="checker", rgb1="0.60 0.64 0.69", rgb2="0.69 0.73 0.78", width="256", height="256")
    ET.SubElement(asset, "material", name="floor_mat", texture="floor_tex", texrepeat="2 2", texuniform="true", reflectance="0.02")
    for pieces in meshes.values():
        for mesh in pieces:
            ET.SubElement(asset, "mesh", name=mesh["name"], file=mesh["file"])
    for mesh in native.values():
        ET.SubElement(asset, "mesh", name=mesh["name"], file=mesh["file"])
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 -2 3", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(world, "light", pos="-2 1 2", dir="1 -0.5 -1", diffuse="0.4 0.4 0.4")
    radius, gap = g["sphere_radius_m"], g.get("equatorial_gap_half_m", 0.0)
    polar_rim = radius * math.cos(math.radians(g["polar_cap_angle_deg"]))
    ET.SubElement(world, "geom", name="floor", type="plane", size="10 10 0.1", material="floor_mat", contype="2", conaffinity="1", user=_fmt([0, 0, 0, -1, radius]))
    fork = ET.SubElement(world, "body", name="fork", pos=_fmt([0, 0, g["sphere_radius_m"]+s["initial_clearance_m"]]))
    ET.SubElement(fork, "freejoint", name="base")
    _inertial_mjcf(fork, props["fork"])
    for shape in _fork_shapes(c):
        ET.SubElement(fork, "geom", name=shape["name"], type="box", size=_fmt(np.asarray(shape["size"])/2), pos=_fmt(shape["pos"]), rgba="0.75 0.78 0.83 1")
    ET.SubElement(fork, "site", name="upper_com", pos=_fmt(props["fork"]["com_m"]), size="0.005", rgba="1 0.25 0.15 1", group="3")
    ET.SubElement(fork, "site", name="imu", pos=_fmt(g["platform"]["center_xyz_m"]), size="0.004", group="3")
    ET.SubElement(fork, "camera", name="overview", pos="0.52 -0.65 0.43", xyaxes="0.780869 0.624695 0 -0.25643 0.320537 0.911733", mode="trackcom")
    ET.SubElement(fork, "camera", name="side", pos="0 -0.8 0.28", xyaxes="1 0 0 0 0.14 0.990151", mode="trackcom")
    carrier = ET.SubElement(fork, "body", name="carrier")
    ET.SubElement(carrier, "joint", name="shaft", type="hinge", axis="0 1 0")
    _inertial_mjcf(carrier, props["carrier"])
    ET.SubElement(carrier, "geom", name="shaft_visual", type="cylinder", fromto=_fmt([0, -g["shaft_length_m"]/2, 0, 0, g["shaft_length_m"]/2, 0]),
                  size=_fmt(g["shaft_radius_m"]), contype="0", conaffinity="0", rgba="0.35 0.38 0.42 1")
    ET.SubElement(carrier, "geom", name="carrier_visual", type="box", size=_fmt(np.asarray(g["carrier"]["size_xyz_m"])/2), contype="0", conaffinity="0", rgba="0.65 0.55 0.3 1", group="3" if "carrier" in native else "0")
    if "carrier" in native:
        ET.SubElement(carrier, "geom", name="native_carrier_visual", type="mesh", mesh=native["carrier"]["name"], contype="0", conaffinity="0", rgba="0.63 0.61 0.55 1")
    for mesh in meshes["carrier"]:
        sign = 1 if mesh["name"] == "belt_A" else -1
        ET.SubElement(carrier, "geom", name=mesh["name"], type="mesh", mesh=mesh["name"], rgba="0.17 0.19 0.22 1", surfacevel="0 0 0", user=_fmt([sign, polar_rim, radius, 3, radius]))
        # Reserved bit4 is incompatible with every model collision mask. It
        # keeps the inactive proxy in MuJoCo's compiled BVH for later enable.
        ET.SubElement(carrier, "geom", name=mesh["name"]+"_smooth", type="sphere", size=_fmt(radius), contype="4", conaffinity="0", group="5", rgba="0 0 0 0", surfacevel="0 0 0", user=_fmt([sign, polar_rim, radius, 4, radius]))
    for name, rgba in (("shell_A", "0.08 0.54 0.69 1"), ("shell_B", "0.11 0.35 0.49 1")):
        shell = ET.SubElement(carrier, "body", name=name)
        ET.SubElement(shell, "joint", name=name+"_joint", type="hinge", axis="1 0 0")
        _inertial_mjcf(shell, props[name])
        sign = 1 if name == "shell_A" else -1
        for mesh in meshes[name]:
            ET.SubElement(shell, "geom", name=mesh["name"], type="mesh", mesh=mesh["name"], rgba=rgba, group="3" if name in native else "0", user=_fmt([sign, gap, polar_rim, 1, radius]))
        ET.SubElement(shell, "geom", name=name+"_smooth", type="sphere", size=_fmt(radius), contype="4", conaffinity="0", group="5", rgba="0 0 0 0", user=_fmt([sign, gap, polar_rim, 2, radius]))
        if name in native:
            ET.SubElement(shell, "geom", name="native_"+name+"_visual", type="mesh", mesh=native[name]["name"], contype="0", conaffinity="0", rgba=rgba)
    keyframes = ET.SubElement(root, "keyframe")
    ET.SubElement(keyframes, "key", name="initial", qpos=_fmt([0, 0, g["sphere_radius_m"]+s["initial_clearance_m"], 1, 0, 0, 0, s.get("initial_shaft_phase_rad", 0), 0, 0]))
    actuators = ET.SubElement(root, "actuator")
    for name, joint, limit in zip(("shaft_motor", "shell_A_motor", "shell_B_motor"), ("shaft", "shell_A_joint", "shell_B_joint"), c["actuation"]["torque_limits_Nm"]):
        ET.SubElement(actuators, "motor", name=name, joint=joint, ctrllimited="true", ctrlrange=_fmt([-limit, limit]), gear="1")
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "framequat", name="fork_orientation", objtype="body", objname="fork")
    ET.SubElement(sensors, "gyro", name="fork_gyro", site="imu")
    path = out / "model.xml"
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="unicode", xml_declaration=True)
    (out / "mass_properties.json").write_text(json.dumps(props, indent=2)+"\n")
    (out / "model_config.json").write_text(json.dumps(c, indent=2)+"\n")
    _write_urdf(c, out, props, meshes, native)
    return path


def _urdf_inertia(link: ET.Element, props: dict) -> None:
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz=_fmt(props["com_m"]), rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=_fmt(props["mass_kg"]))
    tensor = np.asarray(props["inertia_kg_m2"])
    ET.SubElement(inertial, "inertia", **{name: _fmt(tensor[i,j]) for name, i, j in
                  (("ixx",0,0),("iyy",1,1),("izz",2,2),("ixy",0,1),("ixz",0,2),("iyz",1,2))})


def _write_urdf(config: dict, out: Path, props: dict, meshes: dict, native: dict) -> Path:
    robot = ET.Element("robot", name="active_spherical_wheel_approximate")
    robot.append(ET.Comment("Free root fork. URDF preserves geometry, masses and 3 revolutes; moving belt surface control requires the MuJoCo simulator."))
    links = {}
    for name in ("fork", "carrier", "shell_A", "shell_B"):
        links[name] = ET.SubElement(robot, "link", name=name)
        _urdf_inertia(links[name], props[name])
    for shape in _fork_shapes(config):
        for kind in ("visual", "collision"):
            element = ET.SubElement(links["fork"], kind, name=shape["name"])
            ET.SubElement(element, "origin", xyz=_fmt(shape["pos"]), rpy="0 0 0")
            ET.SubElement(ET.SubElement(element, "geometry"), "box", size=_fmt(shape["size"]))
    for body, pieces in meshes.items():
        for piece in pieces:
            kinds = ("collision",) if body in native and body != "carrier" else ("visual", "collision")
            for kind in kinds:
                element = ET.SubElement(links[body], kind, name=piece["name"])
                ET.SubElement(ET.SubElement(element, "geometry"), "mesh", filename=piece["file"], scale="1 1 1")
    for body, piece in native.items():
        element = ET.SubElement(links[body], "visual", name=piece["name"])
        ET.SubElement(ET.SubElement(element, "geometry"), "mesh", filename=piece["file"], scale="1 1 1")
    for name, parent, child, axis, limit in (("shaft", "fork", "carrier", "0 1 0", config["actuation"]["torque_limits_Nm"][0]),
                                              ("shell_A_joint", "carrier", "shell_A", "1 0 0", config["actuation"]["torque_limits_Nm"][1]),
                                              ("shell_B_joint", "carrier", "shell_B", "1 0 0", config["actuation"]["torque_limits_Nm"][2])):
        joint = ET.SubElement(robot, "joint", name=name, type="continuous")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(joint, "axis", xyz=axis)
        ET.SubElement(joint, "limit", effort=_fmt(limit), velocity="100")
        ET.SubElement(joint, "dynamics", damping=_fmt(config["simulation"]["joint_damping_Nms"]), friction="0")
    ET.indent(robot)
    path = out / "model.urdf"
    ET.ElementTree(robot).write(path, encoding="unicode", xml_declaration=True)
    return path


def generate_urdf(config: dict | None = None, output_dir: str | Path | None = None) -> Path:
    """Generate the model assets and return the companion URDF path."""
    return generate_model(config, output_dir).with_suffix(".urdf")
