"""Off-road runs in Gazebo with the unchanged controller and runner.

``GazeboWorkspace`` is the MuJoCo :class:`~spherical_wheel.offroad_experiments.Workspace`
with one difference: ``plant(terrain)`` returns a :class:`GazeboPlant`. The
flat-floor gain library is the MuJoCo one, loaded from the same fingerprinted
cache, so both simulators run the same controller with the same gains.
``run_offroad``, ``place_on_ground`` and ``summarize`` are reused as they are.
"""
from __future__ import annotations

import json
import time

from ..offroad_experiments import DEFAULT_WORKSPACE, Workspace, run_offroad
from .plant import GazeboPlant

GAZEBO_VERSIONS = {'simulator': 'Gazebo Harmonic (gz-sim 8.10.0)', 'physics': 'DART 6.19.5 via gz-physics 7.5.0',
                   'collision_detector': 'ode', 'step_s': .001}


class GazeboWorkspace(Workspace):
    """Same configuration and gain library as ``Workspace``; plants run in Gazebo.

    The bridge plugin's interface is one per process, so only one Gazebo plant
    is alive at a time; asking for another terrain replaces it.
    """

    def __init__(self, config=None, directory=None, seed_cache=None, ground='auto', heightmap_cell_m=.01,
                 heightmap_size_m=None, plant_options=None):
        super().__init__(config, directory or DEFAULT_WORKSPACE, seed_cache, plant_limit=1)
        self.ground = ground
        self.heightmap_cell_m = heightmap_cell_m
        self.heightmap_size_m = heightmap_size_m
        self.plant_options = dict(plant_options or {})
        self._current = None

    def plant(self, terrain):
        key = json.dumps(terrain.spec(), sort_keys=True)
        if self._current is not None and self._current[0] == key:
            return self._current[1]
        if self._current is not None:
            self._current[1].close()
        options = dict(ground=self.ground, heightmap_cell_m=self.heightmap_cell_m,
                       heightmap_size_m=self.heightmap_size_m)
        options.update(self.plant_options)
        plant = GazeboPlant(self.config, terrain, self.directory/'gazebo'/'mirror',
                            world_dir=self.directory/'gazebo'/'world', **options)
        self._current = (key, plant)
        return plant


def run_gazebo(workspace, terrain, path, params=None, **options):
    """``run_offroad`` on a Gazebo plant; the summary records the Gazebo engine."""
    start = time.time()
    log, summary = run_offroad(workspace, terrain, path, params, **options)
    plant = workspace.plant(terrain)
    summary['engine'] = dict(GAZEBO_VERSIONS, ground=plant.ground_kind, world_info=plant.world_info)
    summary['engine_version'] = GAZEBO_VERSIONS['simulator']
    summary['ground_model'] = ('Gazebo collision with the ' + plant.ground_kind +
                               ('' if plant.ground_kind == 'plane' else
                                f' (grid {plant.world_info.get("cell_m", float("nan")):.4f} m)') +
                               '; contacts solved by DART. The MuJoCo mirror only evaluates kinematics.')
    summary['wall_time_s'] = time.time()-start
    summary['bridge_counters'] = plant.sim.counters()
    return log, summary
