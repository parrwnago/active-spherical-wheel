"""I76 frozen analytic routes, driven only by the planned geometry and clock.

All routes share the original fixed site anchor and rotation.  The figure
eight has a fixed +.5 m local-forward offset from that anchor after the first
map-only setup screen failed. It starts and finishes at its crossing; its
geometric tangent is regular there and at every other parameter value,
including when temporal speed is zero.
"""
import numpy as np


ROUTE_IDS = ('straight', 'arc', 'eight')


class AnalyticRoute:
    def __init__(self, route_id):
        if route_id not in ROUTE_IDS:
            raise ValueError('A frozen I76 route ID is required')
        self.route_id = route_id
        self.kind = 'i76_' + route_id
        self.anchor_origin = np.array([-2.4, 10.0], dtype=float)
        self.planned_local_offset = np.array([.5, 0.] if route_id == 'eight' else [0., 0.])
        self.geometry_heading_deg = 113.0
        angle = np.deg2rad(self.geometry_heading_deg)
        c, s = np.cos(angle), np.sin(angle)
        self.rotation = np.array([[c, -s], [s, c]])
        self.origin = self.anchor_origin + self.rotation @ self.planned_local_offset
        self.start_s = 3.0
        self.duration_s = 120.0 if route_id == 'eight' else 32.0
        self.goal_hold_s = 10.0
        self.total_duration_s = self.start_s + self.duration_s + self.goal_hold_s
        tangent = self.rotation @ self.shape_derivatives(0.0)[1]
        self.direction = tangent / np.linalg.norm(tangent)
        self.heading_deg = float(np.rad2deg(self.tangent_heading(0.0)))

    def progress(self, time_s):
        elapsed = float(time_s) - self.start_s
        if not np.isfinite(elapsed):
            raise ValueError('Reference clock must be finite')
        if elapsed <= 0.0:
            return 0.0, 0.0, 0.0, False
        if elapsed >= self.duration_s:
            return 1.0, 0.0, 0.0, False
        r = elapsed / self.duration_s
        return (float(np.clip(10*r**3 - 15*r**4 + 6*r**5, 0.0, 1.0)),
                (30*r*r - 60*r**3 + 30*r**4) / self.duration_s,
                (60*r - 180*r*r + 120*r**3) / self.duration_s**2,
                True)

    def shape_derivatives(self, u):
        """Return local c(u), dc/du, d²c/du², d³c/du³ analytically."""
        u = float(u)
        if not np.isfinite(u) or not 0.0 <= u <= 1.0:
            raise ValueError('Finite geometric parameter in [0,1] required')
        if self.route_id == 'straight':
            return (np.array([1.2*u, 0.0]), np.array([1.2, 0.0]),
                    np.zeros(2), np.zeros(2))
        if self.route_id == 'arc':
            radius, k = .6, np.pi/2
            theta = k*u
            s, c = np.sin(theta), np.cos(theta)
            return (radius*np.array([s, 1-c]),
                    radius*k*np.array([c, s]),
                    radius*k*k*np.array([-s, c]),
                    -radius*k**3*np.array([c, s]))
        a, b, k = .5, .25, 2*np.pi
        theta = k*u
        s, c = np.sin(theta), np.cos(theta)
        s2, c2 = np.sin(2*theta), np.cos(2*theta)
        position = np.array([a*s, b*s2])
        if u in (0.0, 1.0):
            position = np.zeros(2)  # The closed curve's crossing is exact.
        return (position, k*np.array([a*c, 2*b*c2]),
                -k*k*np.array([a*s, 4*b*s2]),
                -k**3*np.array([a*c, 8*b*c2]))

    def tangent_heading(self, u):
        """Continuous unwrapped planned angle, independent of sample history."""
        _, first, _, _ = self.shape_derivatives(u)
        if self.route_id == 'straight':
            local = 0.0
        elif self.route_id == 'arc':
            local = (np.pi/2)*float(u)
        else:
            local = float(np.arctan2(first[1], first[0]))
            # The Gerono eight's velocity curve retraces a parabola.  Its
            # negative-x/positive-y branch continues below -pi, rather than
            # jumping to +pi.  This formula uses only the current planned u.
            if first[0] < 0.0 and local > 0.0:
                local -= 2*np.pi
        return float(np.deg2rad(self.geometry_heading_deg) + local)

    def derivative_certificate(self):
        """Global geometric derivative bounds, not sampled extrema."""
        if self.route_id == 'straight':
            return dict(minimum_tangent_norm=1.2, maximum_second_norm=0.0,
                        maximum_third_norm=0.0,
                        maximum_heading_parameter_derivative=0.0,
                        maximum_heading_parameter_second_derivative=0.0,
                        proof='Constant nonzero tangent.')
        if self.route_id == 'arc':
            radius, k = .6, np.pi/2
            return dict(minimum_tangent_norm=radius*k,
                        maximum_second_norm=radius*k*k,
                        maximum_third_norm=radius*k**3,
                        maximum_heading_parameter_derivative=k,
                        maximum_heading_parameter_second_derivative=0.0,
                        proof='Circular tangent angle equals (pi/2)*u exactly.')
        a, b, k = .5, .25, 2*np.pi
        # |c1|²/k² = a²*z + 4*b²*(2*z-1)², z=cos²(2*pi*u).
        z = float(np.clip((16*b*b-a*a)/(32*b*b), 0.0, 1.0))
        minimum = k*np.sqrt(a*a*z + 4*b*b*(2*z-1)**2)
        second = k*k*np.sqrt(a*a + (4*b)**2)
        third = k**3*np.sqrt(a*a + (8*b)**2)
        first_angle = second/minimum
        second_angle = third/minimum + 2*first_angle**2
        return dict(minimum_tangent_norm=float(minimum),
                    maximum_second_norm=float(second),
                    maximum_third_norm=float(third),
                    maximum_heading_parameter_derivative=float(first_angle),
                    maximum_heading_parameter_second_derivative=float(second_angle),
                    proof='Exact quadratic minimum in z=cos²(theta); componentwise norm upper bounds; |psi_u|<=|c2|/min|c1| and |psi_uu|<=|c3|/min|c1|+2*(|c2|/min|c1|)^2.')

    def __call__(self, time_s):
        u, du, ddu, _ = self.progress(time_s)
        curve, first, second, _ = self.shape_derivatives(u)
        return (self.origin + self.rotation @ curve,
                self.rotation @ first * du,
                self.rotation @ (second*du*du + first*ddu))

    def spec(self):
        geometries = dict(straight='[1.2*u,0]',
                          arc='[.6*sin(pi*u/2),.6*(1-cos(pi*u/2))]',
                          eight='[.5*sin(2*pi*u),.25*sin(4*pi*u)]')
        return dict(kind=self.kind, route_id=self.route_id,
                    anchor_origin=self.anchor_origin.tolist(),
                    planned_local_offset=self.planned_local_offset.tolist(),
                    origin=self.origin.tolist(), geometry_heading_deg=self.geometry_heading_deg,
                    geometry='origin + R113deg @ ' + geometries[self.route_id],
                    progress='u=10*r^3-15*r^4+6*r^5; r=(t-3)/duration clipped to [0,1]',
                    start_s=self.start_s, duration_s=self.duration_s,
                    goal_hold_s=self.goal_hold_s, total_duration_s=self.total_duration_s,
                    initial_heading_deg=self.heading_deg,
                    local_footprint_m=dict(straight=[0, 1.2, 0, 0],
                                           arc=[0, .6, 0, .6],
                                           eight=[-.5, .5, -.25, .25])[self.route_id],
                    endpoint_policy='Zero temporal velocity/acceleration during both holds; geometric endpoint tangent retained.',
                    crossing_policy='Eight starts/ends at its crossing with nonzero geometric tangent; no nearest-path progress selection.',
                    origin_policy='Fixed per-route planned origin = shared site anchor + R113deg @ planned_local_offset; no measured-state reanchor or reset.')


def make_route(route_id):
    return AnalyticRoute(route_id)


def samples(route_id, count=1601, corridor_radius_m=.15, corridor_count=25):
    """Deterministic geometry only, for registered map/fixture coverage."""
    if int(count) != count or count < 2 or int(corridor_count) != corridor_count or corridor_count < 3:
        raise ValueError('At least two route samples and three corridor samples required')
    if not np.isfinite(corridor_radius_m) or corridor_radius_m < 0:
        raise ValueError('Finite nonnegative corridor radius required')
    path = make_route(route_id)
    times = np.linspace(path.start_s, path.start_s+path.duration_s, int(count))
    values = [path(t) for t in times]
    xy, velocity, acceleration = (np.array([v[k] for v in values]) for k in range(3))
    angles = np.linspace(0, 2*np.pi, int(corridor_count), endpoint=False)
    offsets = corridor_radius_m*np.column_stack((np.cos(angles), np.sin(angles)))
    corridor = np.concatenate([xy[:, None, :], xy[:, None, :] + offsets], axis=1)
    return dict(time_s=times, xy_m=xy, velocity_m_s=velocity,
                acceleration_m_s2=acceleration, corridor_xy_m=corridor.reshape(-1, 2))


def world_bounds(route_id=None, margin_m=1.5):
    if not np.isfinite(margin_m) or margin_m < 0:
        raise ValueError('Finite nonnegative world margin required')
    ids = ROUTE_IDS if route_id is None else (route_id,)
    xy = np.concatenate([samples(item)['xy_m'] for item in ids])
    return (float(xy[:, 0].min()-margin_m), float(xy[:, 0].max()+margin_m),
            float(xy[:, 1].min()-margin_m), float(xy[:, 1].max()+margin_m))
