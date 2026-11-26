import numpy as np
import pytest

from spherical_wheel.mobility import MobilityPath, terrain_for, BOUNDS, CASES, SEEDS, evaluate


@pytest.mark.parametrize('kind', ['circle', 'figure8', 'lateral', 'square', 'spokes'])
def test_path_derivatives_and_closed_endpoints(kind):
    path = MobilityPath(kind)
    for t in (-1., path.start_s, path.start_s+path.duration_s, 100.):
        xy, velocity, acceleration = path(t)
        np.testing.assert_allclose(xy, path.origin, atol=1e-12)
        np.testing.assert_allclose(velocity, 0., atol=1e-12)
        np.testing.assert_allclose(acceleration, 0., atol=1e-12)
    h = 1e-4
    for t in np.linspace(1.13, 36.87, 43):
        if kind not in ('circle', 'figure8'):
            segment = path.duration_s/(len(path.waypoints())-1)
            if abs((t-path.start_s)/segment-round((t-path.start_s)/segment)) < h:
                # Jerk changes at these C2 joins: centred differences are only O(h).
                for side in (-1., 1.):
                    np.testing.assert_allclose(path(t+side*1e-8)[1:], 0., atol=1e-7)
                continue
        xy, velocity, acceleration = path(t)
        np.testing.assert_allclose((path(t+h)[0]-path(t-h)[0])/(2*h), velocity, atol=1e-7)
        np.testing.assert_allclose((path(t+h)[1]-path(t-h)[1])/(2*h), acceleration, atol=1e-7)


def test_circles_turn_in_both_directions_and_cover_all_quadrants():
    for clockwise in (False, True):
        path = MobilityPath(clockwise=clockwise)
        points = np.array([path(t)[0] for t in np.linspace(1., 37., 1000)])
        np.testing.assert_allclose(np.linalg.norm(points, axis=1), path.radius_m)
        angle = np.unwrap(np.arctan2(points[:, 1], points[:, 0]))
        np.testing.assert_allclose(angle[-1]-angle[0], (-1 if clockwise else 1)*2*np.pi)


@pytest.mark.parametrize('seed', SEEDS)
@pytest.mark.parametrize('kind', sorted({case[0] for case in CASES.values()}))
def test_arena_curvature_is_valid(kind, seed):
    assert terrain_for(kind, seed).audit(BOUNDS, .12, samples=51)['single_contact_valid']


def test_a_stationary_wheel_cannot_pass_a_circle():
    path = MobilityPath()
    times = np.linspace(0., 40., 401)
    reference = np.array([path(t)[0] for t in times])
    actual = np.tile(path.origin, (len(times), 1))
    errors = np.linalg.norm(actual-reference, axis=1)
    summary = {'completed': True, 'tracking': {'rms_error_while_moving_m': float(np.sqrt(np.mean(errors**2))),
               'max_error_m': float(errors.max()), 'final_error_m': 0.},
               'attitude': {'max_error_from_reference_deg': 0.},
               'actuation': {'saturated_time_fraction': 0.}, 'settled_at_end': True}
    log = {'time_s': times, 'qpos': actual, 'reference_xy': reference, 'heading_rad': np.zeros(len(times))}
    result = evaluate(log, summary, path)['mobility']
    assert not result['passed']
    assert not result['gates']['full_circle']
    assert result['signed_laps'] == 0.
