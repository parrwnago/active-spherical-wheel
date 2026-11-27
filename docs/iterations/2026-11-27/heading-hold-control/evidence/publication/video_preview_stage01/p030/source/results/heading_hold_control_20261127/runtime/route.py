"""I73 single preregistered, fixed-world, C2 S-curve reference.

No simulator state, contact information, terrain, or result-dependent parameters
enter this path.  Every arm uses the same original world origin and clock.
"""
import numpy as np


class SmoothSPath:
    kind = "i73_smooth_s"

    def __init__(self):
        self.origin = np.array([-2.4, 10.0], dtype=float)
        self.start_s = 3.0
        self.duration_s = 32.0
        self.goal_hold_s = 10.0
        self.total_duration_s = self.start_s + self.duration_s + self.goal_hold_s
        self.forward_m = 1.2
        self.lateral_m = 0.14
        self.geometry_heading_deg = 113.0
        angle = np.deg2rad(self.geometry_heading_deg)
        c, s = np.cos(angle), np.sin(angle)
        self.rotation = np.array([[c, -s], [s, c]])
        tangent = self.rotation @ np.array([self.forward_m, 2*np.pi*self.lateral_m])
        self.direction = tangent / np.linalg.norm(tangent)
        self.heading_deg = float(np.rad2deg(np.arctan2(tangent[1], tangent[0])))

    def __call__(self, time_s):
        elapsed = float(time_s) - self.start_s
        if elapsed <= 0:
            return self.origin.copy(), np.zeros(2), np.zeros(2)
        if elapsed >= self.duration_s:
            endpoint = self.origin + self.rotation @ np.array([self.forward_m, 0.0])
            return endpoint, np.zeros(2), np.zeros(2)
        r = elapsed / self.duration_s
        u = 10*r**3 - 15*r**4 + 6*r**5
        du = (30*r**2 - 60*r**3 + 30*r**4) / self.duration_s
        ddu = (60*r - 180*r**2 + 120*r**3) / self.duration_s**2
        theta = 2*np.pi*u
        curve = np.array([self.forward_m*u, self.lateral_m*np.sin(theta)])
        first = np.array([self.forward_m, 2*np.pi*self.lateral_m*np.cos(theta)])
        second = np.array([0.0, -(2*np.pi)**2*self.lateral_m*np.sin(theta)])
        return (self.origin + self.rotation @ curve,
                self.rotation @ first * du,
                self.rotation @ (second*du**2 + first*ddu))

    def spec(self):
        return {
            "kind": self.kind, "origin": self.origin.tolist(),
            "geometry": "origin + R113deg @ [1.2*u, 0.14*sin(2*pi*u)]",
            "progress": "u = 10*r^3 - 15*r^4 + 6*r^5; r=(t-3)/32 clipped to [0,1]",
            "start_s": self.start_s, "duration_s": self.duration_s,
            "goal_hold_s": self.goal_hold_s, "total_duration_s": self.total_duration_s,
            "forward_m": self.forward_m, "lateral_m": self.lateral_m,
            "geometry_heading_deg": self.geometry_heading_deg,
            "initial_heading_deg": self.heading_deg,
            "origin_policy": "Fixed original world origin, no measured-state reanchor or reset.",
            "endpoint_policy": "Exactly zero velocity and acceleration before and after motion.",
        }


def samples(count=1601, corridor_radius_m=0.15, corridor_count=25):
    """Deterministic offline fixture geometry for preflight/world coverage only."""
    path = SmoothSPath()
    times = np.linspace(path.start_s, path.start_s+path.duration_s, int(count))
    values = [path(t) for t in times]
    xy, velocity, acceleration = (np.array([v[k] for v in values]) for k in range(3))
    angles = np.linspace(0, 2*np.pi, int(corridor_count), endpoint=False)
    offsets = corridor_radius_m*np.column_stack((np.cos(angles), np.sin(angles)))
    corridor = np.concatenate([xy[:, None, :], xy[:, None, :] + offsets], axis=1)
    return dict(time_s=times, xy_m=xy, velocity_m_s=velocity,
                acceleration_m_s2=acceleration, corridor_xy_m=corridor.reshape(-1, 2))


def world_bounds(margin_m=1.5):
    xy = samples()["xy_m"]
    return (float(xy[:, 0].min()-margin_m), float(xy[:, 0].max()+margin_m),
            float(xy[:, 1].min()-margin_m), float(xy[:, 1].max()+margin_m))
