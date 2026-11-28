"""Targeted free shell-to-belt tracking experiment with scheduled local gains."""
from __future__ import annotations

import copy
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml

from .control import design_lqr, find_equilibrium, jsonable
from .experiments import plot_rollout, rolling_feedforward
from .model import generate_model, load_config
from .sim import Plant


def _reference(time_s, duration_s):
    r = np.clip(time_s / duration_s, 0.0, 1.0)
    s = 10*r**3 - 15*r**4 + 6*r**5
    ds = (30*r**2 - 60*r**3 + 30*r**4) / duration_s
    dds = (60*r - 180*r**2 + 120*r**3) / duration_s**2
    xy = np.array([0.06*s, 0.005*np.sin(np.pi*s)])
    first = np.array([0.06, 0.005*np.pi*np.cos(np.pi*s)])
    second = np.array([0.0, -0.005*np.pi**2*np.sin(np.pi*s)])
    return xy, first*ds, second*ds**2 + first*dds


def run_transition(output="results/transition", duration=10.0, config=None):
    """Attempt a 6 cm forward arc crossing the north shell/belt boundary.

    All controller designs use a separate plant.  Failed design points and
    failed rollouts are preserved in the output, without introducing support
    constraints or external stabilizing forces.
    """
    out = Path(output).resolve()
    out.mkdir(parents=True, exist_ok=True)
    cfg = copy.deepcopy(config if isinstance(config, dict) else load_config(config))
    path = generate_model(cfg, out / "model")
    designer = Plant(config=cfg, model_path=path)
    grid = np.array([1.10, 1.20, 1.28, 1.32, 1.40, 1.50, 1.57, 1.65])
    available, failed_designs = [], []
    for phase in grid:
        try:
            equilibrium = find_equilibrium(designer, phase)
            design = design_lqr(designer, equilibrium)
            belt_force = sum(c["normal_force_N"] for c in equilibrium["contacts"]
                             if any(n.startswith("belt_") for n in c["geoms"]))
            shell_force = sum(c["normal_force_N"] for c in equilibrium["contacts"]
                              if any(n.startswith("shell_") for n in c["geoms"]))
            regime = "belt" if belt_force > shell_force else "shell"
            available.append({"phase": float(phase), "regime": regime,
                              "equilibrium": equilibrium, "design": design})
        except (RuntimeError, np.linalg.LinAlgError) as error:
            failed_designs.append({"phase_rad": float(phase), "error": str(error)})
    if not available or not any(d["regime"] == "shell" for d in available) or not any(d["regime"] == "belt" for d in available):
        report = {"passed": False, "reason": "No usable gains in both contact regimes", "failed_gain_points": failed_designs}
        (out / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        return report
    initial = min(available, key=lambda d: abs(d["phase"] - 1.10))
    plant = Plant(config=cfg, model_path=path)
    plant.reset(qpos=initial["equilibrium"]["qpos"], qvel=initial["equilibrium"]["qvel"])
    nominal_joints = initial["equilibrium"]["qpos"][7:10].copy()
    boundary = np.pi/2 - np.deg2rad(cfg["geometry"]["polar_cap_angle_deg"])
    targets = {"complete_duration_s": float(duration), "cross_shell_to_belt": True,
               "max_tilt_deg": 8.0, "final_tilt_deg": 1.0,
               "rms_position_error_m": 0.02, "final_position_error_m": 0.02}
    (out / "targets.json").write_text(json.dumps(targets, indent=2) + "\n")
    (out / "configuration.yaml").write_text(yaml.safe_dump(cfg, sort_keys=False))
    (out / "gain_schedule.json").write_text(json.dumps(jsonable({
        "requested_grid_rad": grid, "failed_gain_points": failed_designs,
        "available": [{"phase_rad": d["phase"], "regime": d["regime"],
                       "full_closed_spectral_radius": d["design"]["full_closed_spectral_radius"],
                       "reachable_rank": d["design"]["reachable_rank"]} for d in available],
        "selection": "Nearest available phase in the actual normal-force-dominant contact regime.",
    }), indent=2) + "\n")
    np.savez_compressed(out / "controller.npz", phase=np.array([d["phase"] for d in available]),
                        K=np.array([d["design"]["K"] for d in available]),
                        qpos=np.array([d["equilibrium"]["qpos"] for d in available]))
    keys = ["time_s", "qpos", "qvel", "command", "input", "disturbance", "reference_xy",
            "reference_qpos", "reference_qvel", "tilt_rad", "relative_tilt_rad", "contact_mask",
            "normal_forces_N", "slip_speed_m_s", "scheduled_gain_phase_rad"]
    logs = {key: [] for key in keys}
    contacts_changed = []
    previous_labels = None
    selected = initial
    failure = False
    count = round(duration / plant.dt)
    for index in range(count + 1):
        time_s = index * plant.dt
        xy, velocity, acceleration = _reference(time_s, duration)
        mask, normal = np.zeros(5, dtype=int), np.zeros(5)
        labels = set()
        for contact in plant.contact_summary():
            for name in contact["geoms"]:
                category = next((j for j, prefix in enumerate(("shell_A", "shell_B", "belt_A", "belt_B", "fork"))
                                 if name.startswith(prefix)), None)
                if name == "platform":
                    category = 4
                if category is not None:
                    mask[category] = 1
                    normal[category] += contact["normal_force_N"]
                    if contact["normal_force_N"] > 0.01:
                        labels.add(name)
        if labels != previous_labels:
            contacts_changed.append({"time_s": time_s, "phase_rad": float(plant.data.qpos[7]),
                                     "geoms": sorted(labels), "normal_forces_N": normal.tolist()})
            previous_labels = labels
        if normal[:4].sum() > 0.01:
            regime = "belt" if normal[2:4].sum() > normal[:2].sum() else "shell"
            candidates = [d for d in available if d["regime"] == regime]
            selected = min(candidates, key=lambda d: abs(d["phase"] - plant.data.qpos[7]))
        qref = selected["equilibrium"]["qpos"].copy()
        qref[:2] = initial["equilibrium"]["qpos"][:2] + xy
        qref[7:10] = nominal_joints
        tangent = np.zeros(9)
        tangent[3:5] = [-np.arctan2(acceleration[1], 9.81), np.arctan2(acceleration[0], 9.81)]
        mujoco.mj_integratePos(plant.model, qref, tangent, 1.0)
        vref, feedforward = rolling_feedforward(plant, velocity)
        error = plant.error(qref, vref)
        command = selected["equilibrium"]["u"] + feedforward - selected["design"]["K"] @ error
        state = plant.snapshot()
        slip = 0.0
        for c in plant.data.contact:
            if c.efc_address >= 0:
                i = c.efc_address
                slip = max(slip, float(np.linalg.norm(plant.data.efc_vel[i+1:i+min(c.dim, 3)])))
        values = {"time_s": time_s, "qpos": plant.data.qpos.copy(), "qvel": plant.data.qvel.copy(),
                  "command": command, "input": np.clip(command, -plant.bounds, plant.bounds),
                  "disturbance": np.zeros(6), "reference_xy": qref[:2].copy(), "reference_qpos": qref,
                  "reference_qvel": vref, "tilt_rad": state["tilt_rad"],
                  "relative_tilt_rad": float(np.linalg.norm(error[3:5])), "contact_mask": mask,
                  "normal_forces_N": normal, "slip_speed_m_s": slip,
                  "scheduled_gain_phase_rad": selected["phase"]}
        for key in keys:
            logs[key].append(values[key])
        failure = failure or plant.balance_failed
        if failure or state["tilt_rad"] > np.deg2rad(75) or index == count:
            break
        plant.step(command)
        nominal_joints += plant.dt * vref[6:9]
    data = {key: np.asarray(value) for key, value in logs.items()}
    data["t"] = data["time_s"]
    data["model_path"] = np.array("model/model.xml")
    np.savez_compressed(out / "rollout.npz", **data)
    error_xy = np.linalg.norm(data["qpos"][:, :2] - data["reference_xy"], axis=1)
    rms = float(np.sqrt(np.mean(error_xy**2)))
    crossed = bool(np.any(data["normal_forces_N"][:, :2].sum(axis=1) > 1)
                   and np.any(data["normal_forces_N"][:, 2:4].sum(axis=1) > 1))
    completed = float(data["time_s"][-1])
    peak_tilt, final_tilt = float(np.rad2deg(data["tilt_rad"]).max()), float(np.rad2deg(data["tilt_rad"][-1]))
    passed = bool(completed >= duration-plant.dt/2 and not failure and crossed
                  and peak_tilt < 8 and final_tilt < 1 and rms < 0.02 and error_xy[-1] < 0.02)
    report = {"mode": "shell_to_belt_transition", "passed": passed,
              "contact_mode": cfg["contact"].get("mode", "faceted"), "engine_version": mujoco.__version__,
              "requested_duration_s": float(duration), "completed_duration_s": completed,
              "initial_phase_rad": 1.10, "shell_belt_boundary_phase_rad": float(boundary),
              "final_phase_rad": float(data["qpos"][-1, 7]), "final_reference_phase_rad": float(data["reference_qpos"][-1, 7]),
              "crossed_shell_to_belt": crossed, "fork_or_platform_floor_contact": failure,
              "max_tilt_deg": peak_tilt, "final_tilt_deg": final_tilt,
              "rms_position_error_m": rms, "final_position_error_m": float(error_xy[-1]),
              "final_xy_m": data["qpos"][-1, :2].tolist(),
              "max_input_fraction": np.max(np.abs(data["input"]) / plant.bounds, axis=0).tolist(),
              "saturated_command_fraction": float(np.mean(np.any(np.abs(data["command"]) > plant.bounds, axis=1))),
              "max_joint_speeds_rad_s": np.max(np.abs(data["qvel"][:, 6:9]), axis=0).tolist(),
              "max_slip_speed_m_s": float(data["slip_speed_m_s"].max()),
              "contact_changes": contacts_changed, "failed_gain_points": failed_designs, "targets": targets,
              "notes": ["No external forces or support constraints; starts from upright equilibrium.",
                        "Joint references integrate the instantaneous rolling allocation.",
                        "Unavailable local designs are recorded; nearby gains in the same actual contact regime are used.",
                        "Motor speed and electrical/thermal limits remain unmodeled."]}
    (out / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    plot_rollout(data, out / "plots.png")
    return report
