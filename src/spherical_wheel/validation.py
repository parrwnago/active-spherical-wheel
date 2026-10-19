"""Reproducible checks of the declared plant, independent of the controller.

These diagnostics validate a model, not the unmeasured hardware.  The kinematic
phase sweep deliberately holds the free base at its initial pose and is labelled
as a fixture; momentum and zero-input runs have no such support.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


def _name(model: mujoco.MjModel, kind: mujoco.mjtObj, index: int) -> str:
    return mujoco.mj_id2name(model, kind, int(index)) or f"unnamed_{index}"


def inspect_topology(model: mujoco.MjModel) -> dict[str, Any]:
    """Check the actual compiled freedom, masses and principal inertias."""
    movable = np.arange(1, model.nbody)
    inertia = model.body_inertia[movable]
    triangle_margin = inertia.sum(axis=1) - 2 * inertia.max(axis=1)
    free_count = int(np.count_nonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_FREE))
    hinge_count = int(np.count_nonzero(model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE))
    checks = {
        "one_free_base": free_count == 1,
        "three_relative_hinges": hinge_count == 3,
        "nine_velocity_degrees_of_freedom": model.nv == 9,
        "three_effort_actuators": model.nu == 3,
        "no_equality_constraints": model.neq == 0,
        "no_mocap_bodies": model.nmocap == 0,
        "positive_body_masses": bool(np.all(model.body_mass[movable] > 0)),
        "positive_principal_inertias": bool(np.all(inertia > 0)),
        "physical_inertia_triangle_inequality": bool(np.all(triangle_margin >= -1e-12)),
    }
    return {
        "status": "passed" if all(checks.values()) else "failed",
        "checks": checks,
        "nq": int(model.nq), "nv": int(model.nv), "nu": int(model.nu),
        "total_mass_kg": float(model.body_mass.sum()),
        "bodies": [
            {"name": _name(model, mujoco.mjtObj.mjOBJ_BODY, i),
             "mass_kg": float(model.body_mass[i]),
             "principal_inertia_kg_m2": model.body_inertia[i].tolist()}
            for i in movable
        ],
    }


def total_momentum(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    """Return world linear momentum and angular momentum about a fixed origin.

    MuJoCo's subtree_angmom is about the subtree COM.  The orbital term is
    included here so translation cannot conceal an angular momentum error.
    """
    mujoco.mj_forward(model, data)
    mujoco.mj_subtreeVel(model, data)
    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "fork")
    linear = model.body_subtreemass[root] * data.subtree_linvel[root]
    angular = data.subtree_angmom[root] + np.cross(data.subtree_com[root], linear)
    return linear.copy(), angular.copy()


def isolated_reaction_test(model_path: str | Path, duration_s: float = 0.08) -> dict[str, Any]:
    """Exercise each internal torque with all contact, gravity and loss disabled.

    Declared targets: linear momentum drift < 1e-7 kg m/s; angular momentum
    drift < 1e-6 kg m²/s; nonzero relative response and opposing parent reaction.
    RK4 and a 0.5 ms step isolate the topology from contact solver behaviour.
    """
    records = []
    for channel in range(3):
        model = mujoco.MjModel.from_xml_path(str(model_path))
        model.opt.gravity[:] = 0
        model.opt.timestep = 0.0005
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4
        model.geom_contype[:] = 0
        model.geom_conaffinity[:] = 0
        model.dof_damping[:] = 0
        model.dof_frictionloss[:] = 0
        model.jnt_stiffness[:] = 0
        data = mujoco.MjData(model)
        data.qpos[2] = 1.0
        data.qvel[:] = 0
        initial_p, initial_h = total_momentum(model, data)
        joint = int(model.actuator_trnid[channel, 0])
        dof = int(model.jnt_dofadr[joint])
        child = int(model.jnt_bodyid[joint])
        parent = int(model.body_parentid[child])
        axis = data.xaxis[joint].copy()
        data.ctrl[channel] = 0.4
        max_p_error = max_h_error = 0.0
        for _ in range(round(duration_s / model.opt.timestep)):
            mujoco.mj_step(model, data)
            p, h = total_momentum(model, data)
            max_p_error = max(max_p_error, float(np.linalg.norm(p - initial_p)))
            max_h_error = max(max_h_error, float(np.linalg.norm(h - initial_h)))
        parent_velocity = np.zeros(6)
        child_velocity = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, parent, parent_velocity, 0)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, child, child_velocity, 0)
        reaction = float(parent_velocity[:3] @ axis)
        relative = float(data.qvel[dof])
        checks = {
            "linear_momentum_conserved": max_p_error < 1e-7,
            "angular_momentum_conserved": max_h_error < 1e-6,
            "positive_relative_joint_response": relative > 1e-4,
            "opposing_parent_reaction": reaction < -1e-6,
        }
        records.append({
            "actuator": _name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, channel),
            "joint": _name(model, mujoco.mjtObj.mjOBJ_JOINT, joint),
            "status": "passed" if all(checks.values()) else "failed",
            "checks": checks,
            "torque_Nm": 0.4, "duration_s": duration_s,
            "max_linear_momentum_drift_kg_m_s": max_p_error,
            "max_angular_momentum_drift_kg_m2_s": max_h_error,
            "relative_joint_speed_rad_s": relative,
            "parent_angular_speed_on_initial_axis_rad_s": reaction,
            "child_angular_speed_on_initial_axis_rad_s": float(child_velocity[:3] @ axis),
        })
    return {
        "status": "passed" if all(r["status"] == "passed" for r in records) else "failed",
        "setup": "Free system: gravity, contact, damping, stiffness and friction losses disabled; RK4 dt=0.0005 s. Ideal belt commands are zero; omitted belt rotors are not validated.",
        "runs": records,
    }


def floor_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, Any]]:
    """Identify ground contacts involving the fork/platform failure surfaces."""
    records = []
    for i in range(data.ncon):
        contact = data.contact[i]
        names = [_name(model, mujoco.mjtObj.mjOBJ_GEOM, g) for g in contact.geom]
        if not any(n in ("floor", "ground") for n in names):
            continue
        other = int(contact.geom[1] if names[0] in ("floor", "ground") else contact.geom[0])
        geom_name = _name(model, mujoco.mjtObj.mjOBJ_GEOM, other)
        body_name = _name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[other])
        if body_name in ("fork", "platform") or geom_name.startswith(("fork", "platform")):
            records.append({"geom": geom_name, "distance_m": float(contact.dist),
                            "position_m": contact.pos.tolist()})
    return records


def _contact_records(model: mujoco.MjModel, data: mujoco.MjData) -> list[dict[str, Any]]:
    records = []
    for i in range(data.ncon):
        c = data.contact[i]
        names = [_name(model, mujoco.mjtObj.mjOBJ_GEOM, g) for g in c.geom]
        if not any(n in ("floor", "ground") for n in names):
            continue
        g = int(c.geom[1] if names[0] in ("floor", "ground") else c.geom[0])
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        jacp = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jacp, None, c.pos, int(model.geom_bodyid[g]))
        drive_map = np.zeros((2, 5))
        for channel in range(3):
            j = int(model.actuator_trnid[channel, 0])
            drive_map[:, channel] = jacp[:2, model.jnt_dofadr[j]]
        name = _name(model, mujoco.mjtObj.mjOBJ_GEOM, g)
        if name in ("belt_A", "belt_B", "belt_A_smooth", "belt_B_smooth"):
            drive_map[:, 3 if name.startswith("belt_A") else 4] = data.geom_xmat[g].reshape(3, 3)[:2, 1]
        velocity = jacp @ data.qvel
        records.append({"geom": name, "position_m": c.pos.tolist(),
                        "distance_m": float(c.dist), "normal_force_N": float(force[0]),
                        "body_contact_velocity_xy_m_s": velocity[:2].tolist(),
                        "local_drive_map_xy": drive_map.tolist()})
    return records


def phase_contact_sweep(model_path: str | Path, config=None) -> dict[str, Any]:
    """Fixture diagnostic of contact identity and the actual point Jacobians."""
    from .sim import Plant
    plant = Plant(config=config, model_path=model_path)
    model, data = plant.model, plant.data
    shaft = int(model.actuator_trnid[0, 0])
    phase_address = int(model.jnt_qposadr[shaft])
    records = []
    for phase in np.linspace(-np.pi / 2, np.pi / 2, 13):
        mujoco.mj_resetData(model, data)
        data.qpos[phase_address] = phase
        # A 0.5 mm deliberate penetration generates normal forces for inspection.
        data.qpos[2] -= 0.0005
        mujoco.mj_forward(model, data)
        contacts = _contact_records(model, data)
        columns = [np.array(c["local_drive_map_xy"]) for c in contacts]
        planar_map = np.concatenate(columns, axis=1) if columns else np.zeros((2, 0))
        singular = np.linalg.svd(planar_map, compute_uv=False).tolist() if columns else []
        records.append({"shaft_phase_deg": float(np.rad2deg(phase)),
                        "contacts": contacts, "planar_map_singular_values": singular})
    names = {c["geom"] for r in records for c in r["contacts"]}
    belts_exposed = any(n.startswith("belt_A") for n in names) and any(n.startswith("belt_B") for n in names)
    return {"status": "passed" if belts_exposed else "failed",
            "setup": "KINEMATIC FIXTURE: base pose held, 0.5 mm floor penetration, no dynamic balance claim. Map columns are shaft/shell A/shell B rad/s and belt A/belt B m/s. Concatenated contact maps are a geometric diagnostic, not a controllability proof.",
            "both_polar_belts_reach_floor": belts_exposed, "phases": records}


def polar_shell_leverage(model_path: str | Path) -> dict[str, Any]:
    """Query MuJoCo point Jacobians at a shell's ideal pole and equator."""
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    records = []
    for channel in (1, 2):
        j = int(model.actuator_trnid[channel, 0])
        b = int(model.jnt_bodyid[j])
        axis = data.xaxis[j].copy()
        center = data.xanchor[j].copy()
        # Unit-radius points make this a leverage ratio independent of dimensions.
        equator = np.cross(axis, [0, 0, 1])
        if np.linalg.norm(equator) < 0.1:
            equator = np.cross(axis, [0, 1, 0])
        equator /= np.linalg.norm(equator)
        magnitudes = []
        for p in (center + axis, center + equator):
            jp = np.zeros((3, model.nv))
            mujoco.mj_jac(model, data, jp, None, p, b)
            magnitudes.append(float(np.linalg.norm(jp[:, model.jnt_dofadr[j]])))
        records.append({"joint": _name(model, mujoco.mjtObj.mjOBJ_JOINT, j),
                        "pole_leverage": magnitudes[0], "equator_leverage": magnitudes[1],
                        "status": "passed" if magnitudes[0] < 1e-10 and magnitudes[1] > 0.99 else "failed"})
    return {"status": "passed" if all(r["status"] == "passed" for r in records) else "failed",
            "setup": "Ideal points one metre from each hinge anchor. Confirms the geometric shell-spin singularity; the opening/belt contacts are checked separately.",
            "shells": records}


def belt_wrapper_response(model_path: str | Path, config=None) -> dict[str, Any]:
    """Exercise each belt through the complete input wrapper after floor contact.

    The model starts with clearance, so testing the first step after reset would
    falsely report an ineffective belt.  Both trials begin at the same settled
    integration state, and compare zero against a finite speed command.
    """
    from .sim import Plant

    records = []
    for side, phase, channel in (("A", np.pi / 2, 3), ("B", -np.pi / 2, 4)):
        plant = Plant(config=config, model_path=model_path)
        plant.reset(shaft_phase=phase)
        for _ in range(round(0.05 / plant.dt)):
            plant.step(np.zeros(5))
        start = plant.capture()
        contact_names = [r["geom"] for r in _contact_records(plant.model, plant.data)]
        plant.step(np.zeros(5))
        zero_momentum, _ = total_momentum(plant.model, plant.data)
        zero_velocity = plant.data.qvel.copy()
        plant.restore(start)
        command = np.zeros(5)
        command[channel] = 0.15
        # Verify the wrapper activates support even after an all-zero model.
        plant.model.flg_surfacevel = 0
        plant.step(command)
        powered_momentum, _ = total_momentum(plant.model, plant.data)
        active_name = next((name for name in contact_names if name.startswith(f"belt_{side}")), f"belt_{side}")
        belt_id = mujoco.mj_name2id(plant.model, mujoco.mjtObj.mjOBJ_GEOM, active_name)
        direction = plant.data.geom_xmat[belt_id].reshape(3, 3)[:, 1]
        momentum_delta = powered_momentum - zero_momentum
        checks = {
            "intended_belt_contacts_floor": any(n.startswith(f"belt_{side}") for n in contact_names),
            "moving_surface_flag_enabled": bool(plant.model.flg_surfacevel),
            "finite_command_reaches_surface_field": bool(np.isclose(plant.model.geom_surfacevel[belt_id, 1], 0.15)),
            "ground_reaction_opposes_surface_motion": float(momentum_delta @ direction) < -1e-5,
        }
        records.append({"belt": f"belt_{side}", "status": "passed" if all(checks.values()) else "failed",
                        "checks": checks, "command_m_s": 0.15, "shaft_phase_deg": float(np.rad2deg(phase)),
                        "linear_momentum_delta_kg_m_s": momentum_delta.tolist(),
                        "base_velocity_delta_m_s": (plant.data.qvel - zero_velocity)[:3].tolist()})
    return {"status": "passed" if all(r["status"] == "passed" for r in records) else "failed",
            "setup": "Free dynamic plant, settled for 0.05 s at each pole. Zero/nonzero commands are compared from identical integration states through Plant.step; no externally applied force.",
            "belts": records}


def zero_input_response(model_path: str | Path, duration_s: float = 3.0, shaft_phase: float = 0.30, config=None) -> dict[str, Any]:
    """Record unconstrained response from simultaneous small roll/pitch tilt."""
    from .sim import Plant
    plant = Plant(config=config, model_path=model_path)
    model, data = plant.model, plant.data
    shaft = int(model.actuator_trnid[0, 0])
    data.qpos[model.jnt_qposadr[shaft]] = shaft_phase
    roll, pitch = np.deg2rad([2.0, 2.0])
    data.qpos[3:7] = [np.cos(roll / 2) * np.cos(pitch / 2),
                       np.sin(roll / 2) * np.cos(pitch / 2),
                       np.cos(roll / 2) * np.sin(pitch / 2),
                       -np.sin(roll / 2) * np.sin(pitch / 2)]
    fork = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "fork")
    samples = []
    failure = []
    for step in range(round(duration_s / model.opt.timestep) + 1):
        mujoco.mj_forward(model, data)
        tilt = np.arccos(np.clip(data.xmat[fork].reshape(3, 3)[2, 2], -1, 1))
        if step % max(1, round(0.02 / model.opt.timestep)) == 0:
            samples.append({"time_s": float(data.time), "tilt_deg": float(np.rad2deg(tilt)),
                            "base_position_m": data.qpos[:3].tolist()})
        failure = floor_contacts(model, data)
        if failure:
            break
        mujoco.mj_step(model, data)
    initial = samples[0]["tilt_deg"]
    maximum = max(s["tilt_deg"] for s in samples)
    return {"status": "passed" if failure or maximum > initial * 1.5 else "investigate",
            "setup": "Free base, gravity on, all three joint efforts and both prescribed belt speeds zero. Zero belt speed is an ideal holding servo, not a freely spinning unpowered belt.",
            "initial_tilt_deg": initial, "max_tilt_deg": maximum,
            "initial_shaft_phase_rad": shaft_phase,
            "fork_floor_contact": failure, "end_time_s": float(data.time),
            "samples": samples}


def run_physics_validation(model_path: str | Path, output_dir: str | Path, config=None) -> dict[str, Any]:
    """Save diagnostics without discarding failed milestones."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    from .model import load_config
    selected_config = load_config(config) if isinstance(config, (str, Path)) else (config or load_config())
    report = {
        "scope": "Provisional simulation physics checks, not physical measurements or certified hardware performance.",
        "mujoco_version": mujoco.__version__,
        "contact_model": selected_config.get("contact", {}).get("mode", "faceted"),
        "model": str(Path(model_path).resolve()),
        "topology": inspect_topology(model),
        "internal_torque_reactions": isolated_reaction_test(model_path),
        "polar_shell_leverage": polar_shell_leverage(model_path),
        "contact_phase_sweep": phase_contact_sweep(model_path, config=selected_config),
        "belt_input_wrapper": belt_wrapper_response(model_path, config=selected_config),
        "zero_input_response": zero_input_response(model_path, config=selected_config),
    }
    target = output_dir / "physics_validation.json"
    target.write_text(json.dumps(report, indent=2) + "\n")
    return report


def run_control_validation(model_path: str | Path, output_dir: str | Path, config=None) -> dict[str, Any]:
    """Record local analysis and nonlinear trials, including the LQR limitation.

    Project targets are declared before execution: a static step residual below
    1e-7; mixed-perturbation local prediction error below 2%; no full-model
    growing eigenvalue above 1.0001.  Trial tilt targets are 0.05 degrees for a
    0.1-degree perturbation and 0.5 degrees for a 2-degree perturbation.
    """
    from .control import design_balance_feedback, design_lqr, find_equilibrium, jsonable
    from .sim import Plant

    plant = Plant(config=config, model_path=model_path)
    equilibrium = find_equilibrium(plant)
    full = design_lqr(plant, equilibrium)
    baseline = design_balance_feedback(plant, equilibrium)
    rng = np.random.default_rng(73)
    dx, du = rng.normal(size=18), rng.normal(size=5)
    dx *= 1e-5 / np.linalg.norm(dx)
    du *= 1e-5 / np.linalg.norm(du)
    plant.reset(qpos=equilibrium["qpos"], qvel=equilibrium["qvel"])
    plant.perturb(dx)
    plant.step(equilibrium["u"] + du)
    actual = plant.error(equilibrium["qpos"], equilibrium["qvel"])
    predicted = full["f0"] + full["A"] @ dx + full["B"] @ du
    prediction_error = float(np.linalg.norm(actual - predicted) / np.linalg.norm(actual - full["f0"]))
    residual = float(np.linalg.norm(equilibrium["step_residual"]))

    def trial(design, degrees, duration_s):
        plant.reset(qpos=equilibrium["qpos"], qvel=equilibrium["qvel"])
        perturbation = np.zeros(18)
        perturbation[3:5] = np.deg2rad([degrees, degrees])
        plant.perturb(perturbation)
        max_tilt = plant.snapshot()["tilt_rad"]
        saturated_steps = 0
        peak_speed = np.zeros(3)
        failure = False
        for _ in range(round(duration_s / plant.dt)):
            command = equilibrium["u"] - design["K"] @ plant.error(equilibrium["qpos"], equilibrium["qvel"])
            saturated_steps += int(np.any(np.abs(command) > plant.bounds))
            state = plant.step(command)
            max_tilt = max(max_tilt, state["tilt_rad"])
            peak_speed = np.maximum(peak_speed, np.abs(state["qvel"][6:9]))
            failure = bool(state["balance_failed"])
            if failure:
                break
        target = 0.05 if degrees <= 0.1 else 0.5
        end_tilt = float(np.rad2deg(state["tilt_rad"]))
        passed = not failure and end_tilt <= target and np.linalg.norm(state["qvel"][3:6]) < 0.05
        return {"method": design["method"], "status": "passed" if passed else "failed",
                "initial_roll_and_pitch_deg": degrees, "requested_duration_s": duration_s,
                "end_time_s": state["time_s"], "final_tilt_deg": end_tilt,
                "target_final_tilt_deg": target, "peak_tilt_deg": float(np.rad2deg(max_tilt)),
                "fork_platform_floor_contact": failure,
                "final_body_angular_velocity_rad_s": state["qvel"][3:6].tolist(),
                "final_xy_error_m": (state["qpos"][:2] - equilibrium["qpos"][:2]).tolist(),
                "final_joint_speeds_rad_s": state["qvel"][6:9].tolist(),
                "peak_joint_speeds_rad_s": peak_speed.tolist(),
                "saturated_control_intervals": saturated_steps,
                "external_disturbance": "none"}

    full_eig = np.linalg.eigvals(full["A"] - full["B"] @ full["K"])
    baseline_eig = np.linalg.eigvals(baseline["A"] - baseline["B"] @ baseline["K"])
    report = {
        "scope": "Local simulation analysis and finite free-balance trials. Tracking, robustness across phase changes and hardware performance are not established.",
        "mujoco_version": mujoco.__version__,
        "contact_model": plant.config.get("contact", {}).get("mode", "faceted"),
        "equilibrium": {"status": "passed" if residual < 1e-7 else "failed",
                        "step_residual_norm": residual, "qpos": equilibrium["qpos"],
                        "input": equilibrium["u"], "contacts": equilibrium["contacts"]},
        "independent_local_prediction": {"status": "passed" if prediction_error < 0.02 else "failed",
                                         "relative_error": prediction_error, "seed": 73, "perturbation_norm": 1e-5},
        "full_state_lqr": {"status": "passed" if max(abs(full_eig)) <= 1.0001 else "failed",
                           "full_closed_spectral_radius": float(max(abs(full_eig))),
                           "full_closed_eigenvalues": full_eig,
                           "reachable_rank": full["reachable_rank"],
                           "discarded_eigenvalues": full["discarded_eigenvalues"]},
        "frozen_phase_baseline_local_stability": {
            "status": "passed" if max(abs(baseline_eig)) <= 1.0001 else "failed",
            "full_closed_spectral_radius": float(max(abs(baseline_eig))),
            "growing_full_eigenvalues": baseline_eig[abs(baseline_eig) > 1.0001],
            "interpretation": "Reduced-state design: any growing modes of the complete dynamics are reported here. A finite nonlinear trial does not certify local or global stability."},
        "nonlinear_trials": [trial(full, 0.1, 3.0), trial(full, 2.0, 5.0), trial(baseline, 2.0, 5.0)],
        "limits": ["Joint motor speed limits and electrical/thermal dynamics are not modeled.",
                   "Belts use ideal prescribed speed; motor load and rotor dynamics are omitted.",
                   "Faceted contact introduces artificial material-phase changes; optional analytic clipped contact is restricted to the modeled flat floor.",
                   "The successful baseline trial allows residual position offset and cyclic shell motion."],
    }
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output / "control_validation.json").write_text(json.dumps(jsonable(report), indent=2) + "\n")
    return report
