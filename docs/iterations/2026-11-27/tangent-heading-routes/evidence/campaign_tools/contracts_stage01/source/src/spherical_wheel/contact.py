"""Optional exact clipped spherical support contacts against the flat floor.

MuJoCo sphere primitives supply a smooth local sphere--plane contact only
when the ideal support point lies inside that body's physical shell/cap.
Outside that patch the primitive is excluded and the finite mesh supplies
rim/edge contact. Thus no enabled full sphere bridges a physical opening.

The generated MJCF starts with the proxies disabled. The installer must run
after compilation. This callback is for the declared fixed, flat floor only;
it does not extend the approximation to arbitrary obstacles or terrain.
"""
from __future__ import annotations

import mujoco
import numpy as np

# geom_user: sign, minimum local signed x, maximum local signed x, kind, R.
SHELL_MESH, SHELL_SMOOTH, BELT_MESH, BELT_SMOOTH = 1, 2, 3, 4
FLOOR = -1


def _clipped_floor_filter(model, data, geom1: int, geom2: int) -> int:
    # A custom filter replaces MuJoCo's mask filter. Preserve those masks,
    # including deliberately disabled contacts in free-space physics tests.
    if not ((model.geom_contype[geom1] & model.geom_conaffinity[geom2]) or
            (model.geom_contype[geom2] & model.geom_conaffinity[geom1])):
        return 1
    if model.nuser_geom < 5:
        return 0
    kind1, kind2 = model.geom_user[geom1, 3], model.geom_user[geom2, 3]
    if kind1 == FLOOR:
        floor, geom = geom1, geom2
    elif kind2 == FLOOR:
        floor, geom = geom2, geom1
    else:
        return 0
    # This flag belongs to this model, not a Python registry. Consequently a
    # later compiled raw/faceted model stays valid even with the callback set.
    if model.geom_user[floor, 0] != 1:
        return 0
    sign, lower, upper, kind, radius = model.geom_user[geom]
    if kind not in (SHELL_MESH, SHELL_SMOOTH, BELT_MESH, BELT_SMOOTH):
        return 0
    # All analytical surfaces have their sphere center at their body origin.
    # The ideal sphere support vector is -R*n_floor. Express it in the body's
    # frame, so shaft and upper-body rotation both change patch membership.
    normal = data.geom_xmat[floor].reshape(3, 3)[:, 2]
    body = model.geom_bodyid[geom]
    local_x = -radius * float(np.dot(data.xmat[body].reshape(3, 3)[:, 0], normal))
    inside = lower - 1e-12 <= sign * local_x <= upper + 1e-12
    smooth = kind in (SHELL_SMOOTH, BELT_SMOOTH)
    return int(inside != smooth)


def install_clipped_floor_contact(model, config: dict | None = None) -> bool:
    """Activate optional smooth clipped contacts; return whether activated.

    Set ``contact.mode: analytic_clipped_floor`` in the configuration. Native
    MuJoCo sphere--plane contact is selected only on the declared patch. The
    generated finite mesh remains the fallback at every boundary/opening.
    The callback is global but all mode flags and bounds live on each model.
    """
    mode = (config or {}).get("contact", {}).get("mode", "faceted")
    if mode != "analytic_clipped_floor":
        return False
    if model.nuser_geom < 5:
        raise ValueError("Clipped contacts require generated geom_user patch metadata")
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    if floor < 0 or model.geom_type[floor] != mujoco.mjtGeom.mjGEOM_PLANE:
        raise ValueError("Clipped contacts require the declared flat floor plane")
    if model.geom_bodyid[floor] != 0:
        raise ValueError("Clipped floor must be fixed to world")
    # Floor rotation is identity in the generated model. Reject tilted or
    # unusual layouts rather than silently claiming general terrain support.
    if not np.allclose(model.geom_quat[floor], [1, 0, 0, 0], atol=1e-12):
        raise ValueError("Clipped contacts currently support a horizontal floor")
    for name in ("shell_A_smooth", "shell_B_smooth", "belt_A_smooth", "belt_B_smooth"):
        geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        if geom < 0:
            raise ValueError(f"Missing clipped sphere proxy {name}")
        model.geom_contype[geom] = 1
        model.geom_conaffinity[geom] = 2
    model.geom_user[floor, 0] = 1
    mujoco.set_mjcb_contactfilter(_clipped_floor_filter)
    return True
