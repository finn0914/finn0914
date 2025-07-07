"""Configuration for `fly_clutter_replan.py`.

Variables in CAPS are configs, and should be treated as constants when imported from elsewhere.
Those not in CAPS are intermediate variables.

Some configs are derived from others (You see stuff in CAPS on the right side of `=`).
As a general rule, do not edit derived config.
Other configs you can freely edit.
"""

import numpy as np

import os
import sys
sys.path.append(os.path.abspath('../gym-pybullet-drones'))
from gym_pybullet_drones.utils.enums import DroneModel, Physics


## Output

OUTPUT_FOLDER = "results"
SHOULD_PLOT = True


## PyBullet

VISION = True

PYBULLET_DRONE_MODEL = DroneModel("cf2x")
PYBULLET_PHYSICS = Physics("pyb")
PYBULLET_GUI = True
PYBULLET_SIM_FREQ_HZ = 240
PYBULLET_AGGREGATE_PHY_STEP = 5 # Aggregate n physics simulation step between control. Control runs at (sim_freq/aggr_step) Hz.
PYBULLET_RECORD_DRONE_VISION = False
PYBULLET_ADD_OBSTACLES = True
PYBULLET_USER_DEBUG_UI = True

MAX_SIMULATION_TIME_SEC = 3000 # Abort if reached. Simulation time (not run time).


## Formation

FORMATION_PATTERN = np.array(
    [[0.7,0.7,0],
    [-0.7,0.7,0],
    [0.7,-0.7,0],
    [-0.7,-0.7,0]]
)
FORMATION_START_POINT = np.array([-5, 0, 1])
FORMATION_GOAL_POINT = np.array([8, 0, 1])

DRONE_N = len(FORMATION_PATTERN)
DRONES_INIT_XYZ = FORMATION_START_POINT + FORMATION_PATTERN

init_moving_direction = (FORMATION_GOAL_POINT - FORMATION_START_POINT)[:2]
init_yaw = np.arctan2(init_moving_direction[1], init_moving_direction[0])
DRONES_INIT_RPY = np.array([[0, 0,  init_yaw] for i in range(DRONE_N)]) # [[0, 0,  init_yaw], ... , [0, 0,  init_yaw]]

## Environment

# [x, y, z, rx, ry, rz]
# Used properly in `similarity_calculator.py`.
# In `fly_clutter_replan.py`, currently only determines (x, y) position.
# Shape from `.urdf` files under `obstacle_model`.
OBSTACLES = np.array([   
    [  1,  1.5,  8, 0.4, 0.4, 20],
    [ -1,    3,  8, 0.4, 0.4, 20],
    [ -2,    1,  8, 0.4, 0.4, 20],
    [  2,    0,  8, 0.4, 0.4, 20],
    [  3, -1.5,  8, 0.4, 0.4, 20],
    [0.5,   -2,  8, 0.4, 0.4, 20],
    [ -3, -0.5,  8, 0.4, 0.4, 20],
    [  0,    0,  8, 0.4, 0.4, 20],
    [  0,    0,  8, 0.4, 0.4, 20],
])
# seed = 4255
# obstacle = generate_obstacles(2,26,-6,6,70,seed)


## Mapping

VOXEL_SIZE = 0.1 # meter
WAYPOINT_VOXEL_INTERVAL = 1 # number of voxels between waypoints
CAMERA_FOV_DEG = 60
CAMERA_RANGE = 4 # meter


## Planning

PSO_RAYCAST_ANGLE_INTERVAL_DEG = 10 #the gap between angles when calculating costs
PSO_RAYCAST_DISTANCE_INTERVAL_VOXEL = 3 # voxel length #the gap between angles when calculating costs
