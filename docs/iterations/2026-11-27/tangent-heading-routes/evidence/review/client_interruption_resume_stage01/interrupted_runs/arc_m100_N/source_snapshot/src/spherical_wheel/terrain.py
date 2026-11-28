"""Smooth analytic off-road terrain: height, slope, curvature and friction.

The ground is a height field ``z = h(x, y)`` built from closed-form features,
so height, gradient and Hessian are exact everywhere and no grid resolution
enters the physics. A separate friction map supplies a local Coulomb
coefficient. All lengths are metres and all angles in specifications degrees.

The companion plant represents the ground under the wheel by the tangent plane
at the surface point closest to the wheel centre. That is exact for a sphere
resting on a smooth surface whose concave curvature radius exceeds the wheel
radius, and wrong for anything that needs two simultaneous contact points
(a step, a rut narrower than the wheel, a rock field). ``Terrain.audit``
measures that condition; it is not assumed.
"""
from __future__ import annotations

import copy
import math

import numpy as np


def _sigmoid(value):
    if value >= 0.:
        e = math.exp(-value)
        return 1./(1.+e)
    e = math.exp(value)
    return e/(1.+e)


def _softplus(value):
    return max(value, 0.)+math.log1p(math.exp(-abs(value)))


def _band(value, limits, edge):
    """Smooth indicator of ``limits`` with its first two derivatives."""
    if limits is None:
        return 1., 0., 0.
    low, high = limits
    a, b = _sigmoid((value-low)/edge), _sigmoid((value-high)/edge)
    da, db = a*(1.-a)/edge, b*(1.-b)/edge
    dda, ddb = da*(1.-2.*a)/edge, db*(1.-2.*b)/edge
    return a-b, da-db, dda-ddb


def _window(spec, x, y):
    """Product window ``wx(x)*wy(y)`` as (W, Wx, Wy, Wxx, Wxy, Wyy)."""
    edge = float(spec.get('edge', .05))
    wx, dwx, ddwx = _band(x, spec.get('x'), edge)
    wy, dwy, ddwy = _band(y, spec.get('y'), edge)
    return wx*wy, dwx*wy, wx*dwy, ddwx*wy, dwx*dwy, wx*ddwy


def _plane(spec, x, y):
    sx, sy = spec.get('slope', (0., 0.))
    return spec.get('offset', 0.)+sx*x+sy*y, sx, sy, 0., 0., 0.


def _ramp(spec, x, y):
    """Constant grade between two stations, flat outside, blended over ``blend``."""
    heading = math.radians(spec.get('heading_deg', 0.))
    dx, dy = math.cos(heading), math.sin(heading)
    grade, blend = float(spec['grade']), float(spec.get('blend', .05))
    station = dx*x+dy*y
    ua, ub = (station-spec['start'])/blend, (station-spec['end'])/blend
    sa, sb = _sigmoid(ua), _sigmoid(ub)
    height = grade*blend*(_softplus(ua)-_softplus(ub))
    first = grade*(sa-sb)
    second = grade/blend*(sa*(1.-sa)-sb*(1.-sb))
    return height, first*dx, first*dy, second*dx*dx, second*dx*dy, second*dy*dy


def _waves(spec, x, y):
    amplitude, kx, ky, phase = (spec['_arrays'][key] for key in ('amplitude', 'kx', 'ky', 'phase'))
    angle = kx*x+ky*y+phase
    sine, cosine = amplitude*np.sin(angle), amplitude*np.cos(angle)
    return (float(sine.sum()), float(cosine@kx), float(cosine@ky),
            -float(sine@(kx*kx)), -float(sine@(kx*ky)), -float(sine@(ky*ky)))


def _gaussian(spec, x, y):
    x0, y0 = spec['center']
    sx, sy = spec['sigma']
    angle = math.radians(spec.get('angle_deg', 0.))
    c, s = math.cos(angle), math.sin(angle)
    u, v = c*(x-x0)+s*(y-y0), -s*(x-x0)+c*(y-y0)
    g = spec['amplitude']*math.exp(-.5*(u*u/(sx*sx)+v*v/(sy*sy)))
    gu, gv = -g*u/(sx*sx), -g*v/(sy*sy)
    guu = g*(u*u/sx**4-1./(sx*sx))
    guv = g*u*v/(sx*sx*sy*sy)
    gvv = g*(v*v/sy**4-1./(sy*sy))
    return (g, c*gu-s*gv, s*gu+c*gv,
            c*c*guu-2.*c*s*guv+s*s*gvv,
            c*s*guu+(c*c-s*s)*guv-c*s*gvv,
            s*s*guu+2.*c*s*guv+c*c*gvv)


_FEATURES = {'plane': _plane, 'ramp': _ramp, 'waves': _waves, 'gaussian': _gaussian}


class Terrain:
    """A sum of analytic height features plus a friction map.

    ``features`` are dictionaries with a ``type`` of ``plane``, ``ramp``,
    ``waves`` or ``gaussian`` and an optional ``window`` restricting them to a
    box. ``friction_patches`` are boxes with their own ``mu``. ``yaw_deg`` and
    ``origin`` lay the whole description out along another heading, so one
    course can be driven along either wheel axis. ``spec()`` round-trips
    through JSON and is stored with every run.
    """

    def __init__(self, features=(), friction_patches=(), base_friction=None,
                 name='custom', description='', yaw_deg=0., origin=(0., 0.)):
        self.name, self.description = str(name), str(description)
        self.features = [copy.deepcopy(dict(feature)) for feature in features]
        self.friction_patches = [copy.deepcopy(dict(patch)) for patch in friction_patches]
        self.base_friction = None if base_friction is None else float(base_friction)
        self.yaw_deg = float(yaw_deg)
        self.origin = (float(origin[0]), float(origin[1]))
        self._cos, self._sin = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        # A half turn has a zero sine too, so test the heading itself.
        self._rotated = self.yaw_deg % 360. != 0. or self.origin != (0., 0.)
        self._evaluators = []
        for feature in self.features:
            kind = feature.get('type')
            if kind not in _FEATURES:
                raise ValueError(f'Unknown terrain feature type: {kind!r}')
            if kind == 'waves':
                arrays = {key: np.atleast_1d(np.asarray(feature[key], float))
                          for key in ('amplitude', 'kx', 'ky', 'phase')}
                if len({array.shape for array in arrays.values()}) != 1:
                    raise ValueError('Wave components need equal-length amplitude, kx, ky and phase')
                feature['_arrays'] = arrays
            if kind == 'gaussian' and min(feature['sigma']) <= 0:
                raise ValueError('Gaussian widths must be positive')
            if kind == 'ramp' and (feature.get('blend', .05) <= 0 or feature['end'] <= feature['start']):
                raise ValueError('A ramp needs end > start and a positive blend length')
            self._evaluators.append((_FEATURES[kind], feature, feature.get('window')))
        for patch in self.friction_patches:
            if not patch['mu'] > 0:
                raise ValueError('Friction coefficients must be positive')
        self.is_flat = not self.features

    # -- geometry ---------------------------------------------------------
    def _local(self, x, y):
        dx, dy = x-self.origin[0], y-self.origin[1]
        return self._cos*dx+self._sin*dy, -self._sin*dx+self._cos*dy

    def evaluate(self, x, y):
        """Return ``(h, hx, hy, hxx, hxy, hyy)`` at one world point."""
        if self.is_flat:
            return 0., 0., 0., 0., 0., 0.
        u, v = self._local(x, y) if self._rotated else (x, y)
        h = hu = hv = huu = huv = hvv = 0.
        for function, feature, window in self._evaluators:
            f, fu, fv, fuu, fuv, fvv = function(feature, u, v)
            if window is not None:
                w, wu, wv, wuu, wuv, wvv = _window(window, u, v)
                f, fu, fv, fuu, fuv, fvv = (f*w, fu*w+f*wu, fv*w+f*wv,
                                            fuu*w+2.*fu*wu+f*wuu,
                                            fuv*w+fu*wv+fv*wu+f*wuv,
                                            fvv*w+2.*fv*wv+f*wvv)
            h += f; hu += fu; hv += fv; huu += fuu; huv += fuv; hvv += fvv
        if not self._rotated:
            return h, hu, hv, huu, huv, hvv
        c, s = self._cos, self._sin
        return (h, c*hu-s*hv, s*hu+c*hv,
                c*c*huu-2.*c*s*huv+s*s*hvv,
                c*s*huu+(c*c-s*s)*huv-c*s*hvv,
                s*s*huu+2.*c*s*huv+c*c*hvv)

    def height(self, x, y):
        return self.evaluate(x, y)[0]

    def normal_at(self, x, y):
        _, gx, gy, *_ = self.evaluate(x, y)
        return np.array([-gx, -gy, 1.])/math.sqrt(1.+gx*gx+gy*gy)

    def friction(self, x, y, default):
        """Local Coulomb coefficient; ``default`` applies off every patch."""
        base = default if self.base_friction is None else self.base_friction
        if not self.friction_patches:
            return base
        u, v = self._local(x, y) if self._rotated else (x, y)
        value = base
        for patch in self.friction_patches:
            value += (patch['mu']-base)*_window(patch, u, v)[0]
        return max(value, 1e-3)

    def closest_point(self, center, guess=None, tolerance=1e-12, max_iterations=40):
        """Surface point nearest ``center``; returns ``(point, normal, signed_distance)``.

        Damped Newton on the squared distance. ``guess`` warm-starts from the
        previous contact so a rolling wheel follows one branch continuously.
        """
        cx, cy, cz = (float(value) for value in center)
        if self.is_flat:
            return np.array([cx, cy, 0.]), np.array([0., 0., 1.]), cz
        x, y = (cx, cy) if guess is None else (float(guess[0]), float(guess[1]))
        if (x-cx)**2+(y-cy)**2 > 1.:
            x, y = cx, cy
        h, gx, gy, hxx, hxy, hyy = self.evaluate(x, y)
        cost = .5*((x-cx)**2+(y-cy)**2+(h-cz)**2)
        for _ in range(max_iterations):
            dz = h-cz
            fx, fy = (x-cx)+dz*gx, (y-cy)+dz*gy
            if fx*fx+fy*fy < tolerance*tolerance:
                break
            a, b, c = 1.+gx*gx+dz*hxx, gx*gy+dz*hxy, 1.+gy*gy+dz*hyy
            # Away from a minimum the Hessian can lose definiteness; shift it.
            smallest = .5*(a+c)-math.sqrt(.25*(a-c)**2+b*b)
            if smallest < 1e-3:
                shift = 1e-3-smallest
                a, c = a+shift, c+shift
            det = a*c-b*b
            sx, sy = -(c*fx-b*fy)/det, -(a*fy-b*fx)/det
            scale = 1.
            while True:
                trial = self.evaluate(x+scale*sx, y+scale*sy)
                trial_cost = .5*((x+scale*sx-cx)**2+(y+scale*sy-cy)**2+(trial[0]-cz)**2)
                if trial_cost <= cost or scale < 1e-6:
                    break
                scale *= .5
            x, y, cost = x+scale*sx, y+scale*sy, trial_cost
            h, gx, gy, hxx, hxy, hyy = trial
            if scale*math.hypot(sx, sy) < tolerance:
                break
        normal = np.array([-gx, -gy, 1.])/math.sqrt(1.+gx*gx+gy*gy)
        point = np.array([x, y, h])
        return point, normal, float(normal@(np.array([cx, cy, cz])-point))

    # -- sampling and checks ----------------------------------------------
    def grid(self, xs, ys):
        """Heights on a rectangular grid, shaped ``(len(ys), len(xs))``."""
        return np.array([[self.evaluate(float(x), float(y))[0] for x in xs] for y in ys])

    def audit(self, bounds, wheel_radius, samples=121):
        """Slope and curvature limits over ``bounds = (x0, x1, y0, y1)``.

        ``min_concave_radius_m`` is the tightest valley the surface contains.
        The single-tangent-plane contact is valid only while it exceeds the
        wheel radius; ``single_contact_valid`` keeps a 15% margin.
        """
        x0, x1, y0, y1 = bounds
        steepest = curvature = 0.
        low, high = math.inf, -math.inf
        for y in np.linspace(y0, y1, samples):
            for x in np.linspace(x0, x1, samples):
                h, gx, gy, hxx, hxy, hyy = self.evaluate(float(x), float(y))
                low, high = min(low, h), max(high, h)
                norm = math.sqrt(1.+gx*gx+gy*gy)
                steepest = max(steepest, math.hypot(gx, gy))
                # Shape operator (I + g g^T)^-1 H / |n|; its largest eigenvalue
                # is the tightest upward-concave normal curvature.
                metric = np.array([[1.+gx*gx, gx*gy], [gx*gy, 1.+gy*gy]])
                shape = np.linalg.solve(metric, np.array([[hxx, hxy], [hxy, hyy]]))/norm
                curvature = max(curvature, float(np.max(np.linalg.eigvals(shape).real)))
        radius = math.inf if curvature <= 1e-12 else 1./curvature
        return {'bounds_m': [float(v) for v in bounds], 'samples_per_axis': int(samples),
                'height_range_m': [float(low), float(high)],
                'max_slope_deg': float(math.degrees(math.atan(steepest))),
                'min_concave_radius_m': float(radius),
                'wheel_radius_m': float(wheel_radius),
                'single_contact_valid': bool(radius > 1.15*wheel_radius)}

    def spec(self):
        def clean(item):
            return {key: (value.tolist() if isinstance(value, np.ndarray) else value)
                    for key, value in item.items() if not key.startswith('_')}
        return {'name': self.name, 'description': self.description,
                'yaw_deg': self.yaw_deg, 'origin': list(self.origin),
                'base_friction': self.base_friction,
                'features': [clean(feature) for feature in self.features],
                'friction_patches': [clean(patch) for patch in self.friction_patches]}

    @classmethod
    def from_spec(cls, spec):
        return cls(spec.get('features', ()), spec.get('friction_patches', ()),
                   spec.get('base_friction'), spec.get('name', 'custom'),
                   spec.get('description', ''), spec.get('yaw_deg', 0.), spec.get('origin', (0., 0.)))


def random_waves(seed, rms_m, wavelength_range_m, components=28, spectral_slope=1.5):
    """Band-limited random roughness as one ``waves`` feature.

    Wavelengths are log-uniform in the band, directions isotropic, and
    amplitudes fall as ``wavelength**spectral_slope`` before being scaled to
    the requested RMS height.
    """
    rng = np.random.default_rng(seed)
    low, high = wavelength_range_m
    wavelength = np.exp(rng.uniform(np.log(low), np.log(high), components))
    direction = rng.uniform(0., 2.*np.pi, components)
    amplitude = wavelength**spectral_slope*rng.uniform(.6, 1., components)
    amplitude *= rms_m/np.sqrt(.5*np.sum(amplitude**2))
    k = 2.*np.pi/wavelength
    return {'type': 'waves', 'amplitude': amplitude.tolist(), 'kx': (k*np.cos(direction)).tolist(),
            'ky': (k*np.sin(direction)).tolist(), 'phase': rng.uniform(0., 2.*np.pi, components).tolist()}


def make_terrain(name='flat', heading_deg=0., seed=7, **options):
    """Named terrains laid out along ``heading_deg`` (0 = +x, 90 = +y).

    Every preset starts with roughly 0.25 m of level ground at the origin so a
    run begins from the ordinary flat equilibrium. Distances are along the
    course heading; ``lateral`` means perpendicular to it.
    """
    layout = {'yaw_deg': heading_deg}
    if name == 'flat':
        return Terrain(name='flat', description='Level ground, nominal friction.')
    if name == 'slope':
        pitch, roll = options.get('grade_deg', 8.), options.get('cross_deg', 0.)
        return Terrain([{'type': 'plane', 'slope': [math.tan(math.radians(pitch)), math.tan(math.radians(roll))]}],
                       name='slope', description=f'Uniform plane rising {pitch:g} deg along the course and {roll:g} deg across it.', **layout)
    if name == 'ramp':
        grade, length = options.get('grade_deg', 8.), options.get('length_m', .6)
        return Terrain([{'type': 'ramp', 'start': .25, 'end': .25+length, 'grade': math.tan(math.radians(grade)), 'blend': .06}],
                       name='ramp', description=f'Level, then a {grade:g} deg grade for {length:g} m, then a plateau.', **layout)
    if name == 'cross_slope':
        grade = options.get('grade_deg', 6.)
        # Banked about the course centreline, so the path itself stays level
        # while the ground under it tilts sideways.
        return Terrain([{'type': 'plane', 'slope': [0., math.tan(math.radians(grade))],
                         'window': {'x': [.4, 1.5], 'edge': .1}}],
                       name='cross_slope', description=f'A {grade:g} deg side slope across the course between 0.4 and 1.5 m.', **layout)
    if name == 'hills':
        amplitude = options.get('amplitude_m', .012)
        waves = {'type': 'waves', 'amplitude': [amplitude, .55*amplitude, .4*amplitude],
                 'kx': [2*np.pi/.85, 2*np.pi/.55*np.cos(.6), 2*np.pi/1.3*np.cos(2.0)],
                 'ky': [0., 2*np.pi/.55*np.sin(.6), 2*np.pi/1.3*np.sin(2.0)],
                 'phase': [np.pi, 1.1, -.4], 'window': {'x': [.3, 30.], 'edge': .07}}
        return Terrain([waves], name='hills', description='Rolling undulations, 0.55-1.3 m wavelengths.', **layout)
    if name == 'washboard':
        amplitude, wavelength = options.get('amplitude_m', .003), options.get('wavelength_m', .16)
        waves = {'type': 'waves', 'amplitude': [amplitude], 'kx': [2*np.pi/wavelength], 'ky': [0.], 'phase': [0.],
                 'window': {'x': [.3, 1.5], 'edge': .05}}
        return Terrain([waves], name='washboard', description=f'{amplitude*1000:g} mm ripples every {wavelength*1000:g} mm between 0.3 and 1.5 m.', **layout)
    if name == 'bumps':
        rng = np.random.default_rng(seed)
        features = []
        for index in range(options.get('count', 9)):
            station = .4+.18*index+rng.uniform(-.03, .03)
            sign = -1. if index % 3 == 2 else 1.
            features.append({'type': 'gaussian', 'amplitude': sign*rng.uniform(.008, .014),
                             'center': [station, rng.uniform(-.06, .06)],
                             'sigma': [rng.uniform(.08, .11), rng.uniform(.09, .14)],
                             'angle_deg': rng.uniform(-30., 30.)})
        return Terrain(features, name='bumps', description='Isolated 8-14 mm mounds and potholes every 0.18 m.', **layout)
    if name == 'rough':
        rms = options.get('rms_m', .008)
        waves = random_waves(seed, rms, options.get('wavelengths_m', (.30, 1.4)))
        waves['window'] = {'x': [.3, 30.], 'edge': .07}
        return Terrain([waves], name='rough', description=f'Band-limited random ground, {rms*1000:g} mm RMS, seed {seed}.', **layout)
    if name == 'mud':
        mu = options.get('mu', .18)
        return Terrain([], [{'mu': mu, 'x': [.35, 1.0], 'edge': .01}], name='mud',
                       description=f'Level ground with a mu={mu:g} strip between 0.35 and 1.0 m.', **layout)
    if name == 'slick_ramp':
        grade, mu = options.get('grade_deg', 8.), options.get('mu', .15)
        return Terrain([{'type': 'ramp', 'start': .25, 'end': .85, 'grade': math.tan(math.radians(grade)), 'blend': .06}],
                       [{'mu': mu, 'x': [.45, .75], 'edge': .01}], name='slick_ramp',
                       description=f'A {grade:g} deg grade with a mu={mu:g} strip on it between 0.45 and 0.75 m; '
                                   f'standing still there uses {math.tan(math.radians(grade))/mu:.0%} of the friction limit.',
                       **layout)
    if name == 'course':
        grade = math.tan(math.radians(options.get('grade_deg', 7.)))
        rough = random_waves(seed, options.get('rms_m', .005), (.30, 1.2))
        rough['window'] = {'x': [1.05, 1.85], 'edge': .06}
        ripples = {'type': 'waves', 'amplitude': [.0028], 'kx': [2*np.pi/.16], 'ky': [0.], 'phase': [0.],
                   'window': {'x': [2.0, 2.6], 'edge': .05}}
        features = [{'type': 'ramp', 'start': .30, 'end': .85, 'grade': grade, 'blend': .06},
                    rough, ripples,
                    {'type': 'gaussian', 'amplitude': .014, 'center': [2.85, .02], 'sigma': [.10, .13]},
                    {'type': 'ramp', 'start': 3.05, 'end': 3.60, 'grade': -grade, 'blend': .06}]
        patches = [{'mu': options.get('mu', .25), 'x': [1.35, 1.75], 'edge': .01}]
        return Terrain(features, patches, name='course',
                       description='Climb, rough plateau with a slippery strip, washboard, a mound, then a descent.', **layout)
    raise ValueError(f'Unknown terrain preset: {name!r}')


PRESETS = ('flat', 'slope', 'ramp', 'cross_slope', 'hills', 'washboard', 'bumps', 'rough', 'mud', 'slick_ramp',
           'course')
