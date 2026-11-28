"""Gazebo Harmonic (gz-sim 8, DART) reconstruction of the active spherical wheel.

``model_sdf`` writes the wheel as an SDFormat model from the same configuration
and mass properties as the MuJoCo model. ``worlds`` writes worlds: a level
plane, the analytic test terrains as height maps and the ERC Mars yards.
``bridge`` runs the Gazebo server in this process in lockstep with the
SphericalWheelBridge system plugin, and ``plant`` puts the unchanged off-road
controller on top of it. See GAZEBO.md.
"""
