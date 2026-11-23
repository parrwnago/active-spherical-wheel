"""Command line: python -m spherical_wheel.gazebo {parity,mars} --help."""
from __future__ import annotations

import argparse
from pathlib import Path

from ..model import REPO_ROOT


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python -m spherical_wheel.gazebo',
                                     description='Gazebo Harmonic simulation of the active spherical wheel.')
    sub = parser.add_subparsers(dest='command', required=True)
    parity = sub.add_parser('parity', help='Off-road scenarios in Gazebo, compared with the saved MuJoCo suite')
    parity.add_argument('--scenarios', nargs='*', default=None)
    parity.add_argument('--controllers', nargs='*', default=['terrain_aware', 'flat_baseline'])
    parity.add_argument('--workers', type=int, default=4)
    parity.add_argument('--output', default=str(REPO_ROOT/'results'/'gazebo'/'parity'))
    parity.add_argument('--mujoco-reference', action='store_true',
                        help='Also run the MuJoCo suite on this machine and compare against it')
    mars = sub.add_parser('mars', help='Trajectories on the ERC Mars yard (marsyard2021)')
    mars.add_argument('--routes', nargs='*', default=None)
    mars.add_argument('--controllers', nargs='*', default=['terrain_aware', 'flat_baseline'],
                      help='terrain_aware, flat_baseline, terrain_aware_heading_hold and/or terrain_aware_fall_line')
    mars.add_argument('--workers', type=int, default=4)
    mars.add_argument('--output', default=str(REPO_ROOT/'results'/'gazebo'/'mars'))
    mars.add_argument('--phases', nargs='*', type=float, default=None,
                      help='Wheel starting phases (rad) to run instead of the default 0.30')
    lqr = sub.add_parser('mars-lqr', help='Original local LQR against the contact-aware controller on the Mars yard, '
                                          'matched starts, with side-by-side replay videos')
    lqr.add_argument('--routes', nargs='*', default=None)
    lqr.add_argument('--workers', type=int, default=5)
    lqr.add_argument('--render-workers', type=int, default=6)
    lqr.add_argument('--no-render', action='store_true')
    lqr.add_argument('--output', default=str(REPO_ROOT/'results'/'gazebo'/'mars_local_lqr'))
    view = sub.add_parser('view', help='Run one Mars route at real-time pace for the Gazebo GUI (gz sim -g)')
    view.add_argument('--route', default='plain_traverse')
    view.add_argument('--controller', default='terrain_aware',
                      help='terrain_aware, flat_baseline, terrain_aware_heading_hold or terrain_aware_fall_line')
    view.add_argument('--rate', type=float, default=1., help='Simulated seconds per wall-clock second')
    view.add_argument('--output', default=str(REPO_ROOT/'results'/'gazebo'/'view'))
    args = parser.parse_args(argv)
    if args.command == 'parity':
        from .parity import run_mujoco_reference, run_suite
        run_suite(args.output, args.scenarios, tuple(args.controllers), args.workers)
        if args.mujoco_reference:
            run_mujoco_reference(args.output, args.scenarios, tuple(args.controllers), args.workers)
    elif args.command == 'mars':
        from .mars import run_routes
        run_routes(args.output, args.routes, tuple(args.controllers), args.workers, args.phases)
    elif args.command == 'mars-lqr':
        from .mars_compare import run
        run(args.output, args.routes, args.workers, not args.no_render, args.render_workers)
    elif args.command == 'view':
        view_route(args)


def view_route(args):
    """Pace one route to real time so ``gz sim -g`` (another terminal) can follow it.

    The server runs in this process and its scene broadcaster publishes the
    world over gz-transport; start the GUI client with ``gz sim -g`` in the
    same environment. The run is saved like any other Mars route.
    """
    import time
    from .mars import ROUTES, run_route
    from . import plant as plant_module
    original = plant_module.GazeboPlant.step

    def paced(self, u, disturbance=None):
        state = original(self, u, disturbance)
        start = getattr(self, '_view_start', None)
        if start is None:
            self._view_start = start = time.monotonic()-self._time/args.rate
        lag = self._time/args.rate-(time.monotonic()-start)
        if lag > 0:
            time.sleep(lag)
        return state
    plant_module.GazeboPlant.step = paced
    print('World loading. In another terminal (asw-gz environment) run:  gz sim -g', flush=True)
    result = run_route({'route': args.route, 'controller': args.controller, 'output': args.output,
                        'workspace': str(Path(args.output)/'_workspaces'), 'all_routes': list(ROUTES)})
    print({key: value for key, value in result.items() if key != 'summary_path'})


if __name__ == '__main__':
    main()
