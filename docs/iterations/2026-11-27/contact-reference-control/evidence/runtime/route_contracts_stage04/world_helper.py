"""I73 world fixture factory. Called only by the authorized physics runner.

Map geometry selects/builds the physical fixture and verifies its mesh. It does
not enter the StateMirror, rolling selector, or controller inputs.
"""
import numpy as np
from route import SmoothSPath, world_bounds


def world_options():
    path=SmoothSPath()
    times=np.linspace(path.start_s,path.start_s+path.duration_s,9)
    points=[path(t)[0].tolist() for t in times]
    return dict(ground='marsyard2021_dem',visual_bounds=world_bounds(),
                ground_check_points=points)


def create_workspace(directory):
    from spherical_wheel.gazebo.experiments import GazeboWorkspace
    from spherical_wheel.gazebo.mars import MarsYard
    from spherical_wheel.gazebo.parity import SEED_CACHE
    workspace=GazeboWorkspace(directory=directory,seed_cache=SEED_CACHE,
                              plant_options=world_options())
    return workspace,MarsYard(),SmoothSPath()
