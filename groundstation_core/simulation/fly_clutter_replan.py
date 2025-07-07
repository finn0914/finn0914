"""Script demonstrating the joint use of simulation and control.

The simulation is run by a `CtrlAviary` or `VisionAviary` environment.
The control is given by the PID implementation in `DSLPIDControl`.
"""

import os
import time
import argparse
from datetime import datetime
import pdb
import math
import random
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
import open3d as o3d
import sys

from pathlib import Path
simulation_path = Path(__file__).parent
sys.path.append(os.path.join(simulation_path, "gym-pybullet-drones"))

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.envs.VisionAviary import VisionAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.control.SimplePIDControl import SimplePIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

import cv2
from MPC_formation import Full_controller,fill_up_obstacle,calculate_waypoint
from occupied_map import known_map_establish,from_map_to_real,get_boundary,from_real_to_map,from_real_to_map_pattern
from obstacle_process import Obstacle_Processor
from berstein import points2spline_points3D,points2spline_angle
from minimum_deform_A_star_replan import minimum_deform_search,generate_obstacles

from O3D_Visualizer import o3d_visualizer,ellipsoid_geometry_from_params

import util
import config


np.set_printoptions(precision = 2,suppress=True)

# Based on https://stackoverflow.com/questions/59128880/getting-world-coordinates-from-opengl-depth-buffer
def get_point_cloud(depth_image, width, height, view_matrix, proj_matrix):
    
    # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
    proj_matrix = np.asarray(proj_matrix).reshape([4, 4], order="F")
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
    tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))

    # create a grid with pixel coordinates and depth values
    y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
    y *= -1.
    x, y, z = x.reshape(-1), y.reshape(-1), depth_image.reshape(-1)
    h = np.ones_like(z)

    # print(np.min(z),np.max(z))

    pixels = np.stack([x, y, z, h], axis=1)
    # filter out "infinite" depths
    pixels = pixels[(z < 0.99) & (z > 0.40)]
    # pixels = pixels[z > 0.20]
    pixels[:, 2] = 2 * pixels[:, 2] - 1

    # turn pixels to world coordinates
    points = np.matmul(tran_pix_world, pixels.T).T
    points /= points[:, 3: 4]
    points = points[:, :3]

    return points

def enlarge_obstacle(obstacle,safety_distance = 0.5):
    output = obstacle.copy()
    output[:,-3:] += safety_distance
    return output

# +x is 0
# +y is 1/2 pi
def yaw_rad_to_unit_vector(yaw):
    return np.array((np.cos(yaw), np.sin(yaw)))

def run(
        drone=config.PYBULLET_DRONE_MODEL,
        num_drones=config.DRONE_N,
        physics=config.PYBULLET_PHYSICS,
        vision=config.VISION,
        gui=config.PYBULLET_GUI,
        record_drone_vision=config.PYBULLET_RECORD_DRONE_VISION,
        should_plot=config.SHOULD_PLOT,
        user_debug_gui=config.PYBULLET_USER_DEBUG_UI,
        add_obstacles=config.PYBULLET_ADD_OBSTACLES,
        simulation_freq_hz=config.PYBULLET_SIM_FREQ_HZ,
        aggregate_phy_step_count=config.PYBULLET_AGGREGATE_PHY_STEP,
        max_simulation_time_sec=config.MAX_SIMULATION_TIME_SEC,
        output_folder=config.OUTPUT_FOLDER
        ):

    #### Create the environment with or without video capture ##
    if vision: 
        env = VisionAviary(drone_model=drone,
                           num_drones=num_drones,
                           initial_xyzs=config.DRONES_INIT_XYZ,
                           initial_rpys=config.DRONES_INIT_RPY,
                           physics=physics,
                           neighbourhood_radius=10,
                           freq=simulation_freq_hz,
                           aggregate_phy_steps=aggregate_phy_step_count,
                           gui=gui,
                           record=record_drone_vision,
                           obstacles=add_obstacles,
                           user_debug_gui=user_debug_gui
                           )
    else: 
        env = CtrlAviary(drone_model=drone,
                         num_drones=num_drones,
                         initial_xyzs=config.DRONES_INIT_XYZ,
                         initial_rpys=config.DRONES_INIT_RPY,
                         physics=physics,
                         neighbourhood_radius=10,
                         freq=simulation_freq_hz,
                         aggregate_phy_steps=aggregate_phy_step_count,
                         gui=gui,
                         record=record_drone_vision,
                         obstacles=add_obstacles,
                         user_debug_gui=user_debug_gui
                         )

    #### Obtain the PyBullet Client ID from the environment ####
    PYB_CLIENT = env.getPyBulletClient()

    #### Initialize the logger #################################
    logger = Logger(logging_freq_hz=int(simulation_freq_hz/aggregate_phy_step_count),
                    num_drones=num_drones,
                    output_folder=output_folder,
                    )

    #### Initialize the controllers ############################
    if drone in [DroneModel.CF2X, DroneModel.CF2P]:
        ctrl = [DSLPIDControl(drone_model=drone) for i in range(num_drones)]
    elif drone in [DroneModel.HB]:
        ctrl = [SimplePIDControl(drone_model=drone) for i in range(num_drones)]

    ### Load obstacles #########################################
    max_obstacle_count = 100
    min_u = -4
    max_u = 4
    empty_obstacle = np.array([[-50,-50,-50,1,1,1]])
    dt = 0.4
    #
    controller = Full_controller(
        formation_pattern=config.FORMATION_PATTERN,
        formation_center=config.FORMATION_START_POINT,
        formation_target=config.FORMATION_GOAL_POINT,
        obs=fill_up_obstacle(enlarge_obstacle(config.OBSTACLES, 0.1), max_obstacle_count),
        n_obstacle=max_obstacle_count,
        max_u=max_u,
        min_u = min_u,
        horizon=10,
        t_step=dt
    )

    for i in range(config.OBSTACLES.shape[0]):
        ### 003 is fat cylinder, 004 is thin
        # 000 -> pillar, 001 -> sphere, 002 -> big pillar
        p.loadURDF(os.path.join(simulation_path, "obstacle_model/obs003.urdf"),
                    [config.OBSTACLES[i,0], config.OBSTACLES[i,1], 0],
                    physicsClientId=PYB_CLIENT,
                    globalScaling = 0.4
                    )
        # p.loadURDF(os.path.join(simulation_path, "obstacle_model/obs004.urdf"),
        #             [config.OBSTACLES[i,0], config.OBSTACLES[i,1], -2],
        #             physicsClientId=PYB_CLIENT,
        #             globalScaling = 0.4
        #             )
    # for i in range(config.OBSTACLES.shape[0]):
    #     p.loadURDF(os.path.join(simulation_path, "obstacle_model/obs004.urdf"),
    #                 [config.OBSTACLES[i,0], config.OBSTACLES[i,1], -3.5],
    #                 physicsClientId=PYB_CLIENT,
    #                 globalScaling = 0.5
    #                 )
    # p.loadURDF(os.path.join(simulation_path, "obstacle_model/obs002.urdf"),
    #             [5,5,0],
    #             physicsClientId=PYB_CLIENT,
    #             globalScaling = 0.3
    #             )
    
        
    controller.setup_controller() # change parameter before setup controller #
    idx = None

    viz = o3d_visualizer()

    ### planning setting ###

    #### enlarge the obstacle ####
    obs_buffer = 0.1
    plan_obstacle = config.OBSTACLES.copy()
    plan_obstacle[:,-3:-1] += obs_buffer
    obs_ = np.empty((0,5)) # Currently, only used for `boundary_2D`.
    for point in plan_obstacle:
        x, y, z, dx, dy, dz = point[:6]
        obs_ = np.append(obs_, [[x, y, dx, dy, 0]], axis=0) ### the fifth element decides the shape of the obstacle, 0 is cylinder, 1 is cube(no related urdf file)

    boundary_2D = get_boundary(obs_, config.FORMATION_START_POINT, config.FORMATION_GOAL_POINT, config.FORMATION_PATTERN, buffer = config.CAMERA_RANGE)
    x_min,x_max,y_min,y_max = boundary_2D
    ###change if you want the modify the z coordinate
    z_max = 3
    z_min = 0
    x_slice_size = int((x_max-x_min)/config.VOXEL_SIZE)
    y_slice_size = int((y_max-y_min)/config.VOXEL_SIZE)
    z_slice_size = int((z_max-z_min)/config.VOXEL_SIZE)
    map_shape = (x_slice_size,y_slice_size)
    boundary_3D = boundary_2D + (z_min, z_max) 
    map_shape_3D = (x_slice_size,y_slice_size,z_slice_size)
    ###2D mapping
    known_map = np.zeros((y_slice_size,x_slice_size))
    ###change the parameters from real coordinate to map coordinate
    map_start_point = [from_real_to_map(config.FORMATION_START_POINT[0],x_min,x_max,x_slice_size),from_real_to_map(config.FORMATION_START_POINT[1],y_min,y_max,y_slice_size)]
    map_end_point = [from_real_to_map(config.FORMATION_GOAL_POINT[0],x_min,x_max,x_slice_size),from_real_to_map(config.FORMATION_GOAL_POINT[1],y_min,y_max,y_slice_size)]
    map_camera_range = int(config.CAMERA_RANGE / config.VOXEL_SIZE)
    map_formation_pattern = from_real_to_map_pattern(config.FORMATION_PATTERN,boundary_2D,[x_slice_size,y_slice_size])
    heading = np.zeros(len(map_formation_pattern))

    execute_center_map, execute_path_angle = minimum_deform_search(
        known_map, map_start_point, map_end_point, heading, map_formation_pattern,
        config.WAYPOINT_VOXEL_INTERVAL, config.CAMERA_FOV_DEG, config.PSO_RAYCAST_ANGLE_INTERVAL_DEG, map_camera_range, config.PSO_RAYCAST_DISTANCE_INTERVAL_VOXEL
    )
    
    obstacle_processor = Obstacle_Processor(map_shape_3D, config.VOXEL_SIZE, boundary_3D, config.CAMERA_FOV_DEG, config.CAMERA_RANGE)
    ### extract path and heading angle
    execute_center = []
    execute_path = [[] for i in range(num_drones)]
    execute_angle = [[] for i in range(num_drones)]
    
    ### doing smoothness to both path and angle sequence
    smooth_center = []
    smooth_paths = []
    smooth_angles = []
    ### the number of points between waypoints can be modified to get the same number of waypoints for each agent
    n = 10
    angle_n = int(n*2/len(execute_path_angle[0]))
    
    colors = ["r", "g", "b", "c","r", "g", "b", "c","r", "g", "b", "c"]

    for ID in range(num_drones):
        for item in execute_path_angle[ID]:
            sublist = [from_map_to_real(item[0][0],x_min,x_max,x_slice_size),from_map_to_real(item[0][1],y_min,y_max,y_slice_size)]
            three_d_points = [sublist[0], sublist[1], 1]
            execute_path[ID].append(three_d_points)
            execute_angle[ID].append(math.radians(item[1]))
        # plt.plot([point[0] for point in execute_path[ID]],[point[1] for point in execute_path[ID]],color = colors[ID])
    print("execute_path",execute_path)
    # plt.show()
    
    for item in execute_center_map:
        sublist = [from_map_to_real(item[0],x_min,x_max,x_slice_size),from_map_to_real(item[1],y_min,y_max,y_slice_size)]
        three_d_points = [sublist[0], sublist[1], 1]
        execute_center.append(three_d_points)
    print("execute_center",execute_center)
    t,smooth_center = points2spline_points3D(execute_center,angle_n)

    # print("execute_angle",execute_angle)

    for ID in range(num_drones):
        t,smooth_points = points2spline_points3D(execute_path[ID],n)
        t,smooth_angle = points2spline_angle(execute_angle[ID],angle_n)
        smooth_paths.append(smooth_points)
        smooth_angles.append(smooth_angle)
        # plt.plot([point[0] for point in smooth_points],[point[1] for point in smooth_points],color = colors[ID])
        print("smooth_angles_len",len(smooth_angles[ID]))
    print("smooth_paths = ",smooth_paths)
    print("smooth_angles = ",smooth_angles)
    # plt.show() 
    
    waypoint_index_offset = 0
    waypoint_index = 0
    # print(path)
    all_waypoint = (config.DRONES_INIT_XYZ).reshape(num_drones*3)

    #### Run the simulation ####################################
    simulation_start_clock_time = time.time()

    action = {str(i): np.array([0,0,0,0]) for i in range(num_drones)} # Control input of pybullet model.
    state = np.ones(shape=(controller.dim*2, controller.n_agent)) # shape = (dim*2,num_agent) = (pos+vel,num_agent)
    current_formation_center = np.mean(state[:controller.dim,:], axis = 1)
    for ID in range(controller.n_agent):
        mpc_obj = controller.mpc_objs[ID]
        state[:,ID] = mpc_obj.simulator.x0.cat.full()[:,0]

    last_waypoint_update_k = 0
    for k_sim in range(0, int(max_simulation_time_sec*env.SIM_FREQ), aggregate_phy_step_count):
        #### Make it rain rubber ducks #############################
        # if k_sim/env.SIM_FREQ>5 and k_sim%10==0 and k_sim/env.SIM_FREQ<10: p.loadURDF("duck_vhacd.urdf", [0+random.gauss(0, 0.3),-0.5+random.gauss(0, 0.3),3], p.getQuaternionFromEuler([random.randint(0,360),random.randint(0,360),random.randint(0,360)]), physicsClientId=PYB_CLIENT)

        #### Step the simulation ###################################
        obs, reward, done, info = env.step(action)

        target_rpys = config.DRONES_INIT_RPY
        #### run proposed planner and MPC controller ###############
        # 1. get state and observation ##
        if k_sim % (20*aggregate_phy_step_count) == (10*aggregate_phy_step_count): # Run at arbitrary interval (20*aggregate_phy_step_count).
            mapping_time_start = time.time()
            last_state = state
                
            state = np.ones(shape=(controller.dim*2,controller.n_agent)) # shape = (dim*2,num_agent) = (pos+vel,num_agent)
            # [[-0.04  1.46 -1.53] x1,x2,x3
            # [ 1.12 -0.38 -0.38] y1,y2,y3
            # [ 1.    1.    1.  ] z1,z2,z3
            # [ 0.01 -0.03 -0.01] vx1, vx2, vx3 
            # [ 0.02  0.06  0.01] vy1, vy2, vy3
            # [ 0.    0.    0.  ]] vz1, vz2, vz3
            # if k_sim < 300 or k_sim %
            heading = []
            for j in range(num_drones):
                agentstate = obs[str(j)]["state"] #X,Y,Z,Q1,Q2,Q3,Q4,R,P,Y,VX,VY,VZ,WX,WY,WZ,P0,P1,P2,P3
                state[:3,j] = agentstate[:3]    # position
                state[-3:,j] = agentstate[10:13]# velocity
                # print("current position",state[:3,j])
                

                rgb = obs[str(j)]["rgb"].astype(np.uint8)
                depth = obs[str(j)]["dep"].astype(np.float32)
                points = get_point_cloud(depth, 320, 240, info[str(j)]['ViewMatrix'], info[str(j)]['ProjectionMatrix'])
                pcd =   o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points)
                outlier_remove = False #True if (k_sim % 2200 == 50 and j == 0) else False
                # print('update PCD')
                # if j in [0,3]:
                print("####################update PCD#####################",j)
                time1 = time.time()
                ### this map can be constructed in 3d, but it cost a lot of the computation time
                map_2d = obstacle_processor.update_point_cloud_observation(j, pcd,agentstate[:3],agentstate[9], config.VOXEL_SIZE)
                # print('update PCD finish')
                time2 = time.time()
                print("pcd update time", time2-time1)
                heading.append(np.degrees(agentstate[9]))### record the current agents' heading
                print("heading",heading)
                # if j == 0:
                #     # print('state',state)
                #     # print(np.max(depth),np.min(depth))
                #     rgb = rgb[:,:,:3]
                #     cv2.imshow('rgb',rgb[:,:,::-1])
                #     cv2.imshow('depth',np.power(depth/np.max(depth),5))
                #     cv2.waitKey(1)
                    
            # print(np.array(obstacle_processor.pcd.points).shape)
            mapping_time_end = time.time()
            print("mapping_time:",mapping_time_end-mapping_time_start)

            # 2. run controller and planner ########################
            current_average_velocity = np.linalg.norm(np.mean(state[-controller.dim:,:],axis = 1))
            print('current_average_velocity',current_average_velocity)

            ### update waypoints
            if waypoint_index >= len(smooth_paths[0]):
                target_positions = [sublist[-1] for sublist in smooth_paths]
                old_target_positions = [sublist[-2] for sublist in smooth_paths]
            else:
                target_positions = [sublist[waypoint_index] for sublist in smooth_paths]
                old_target_positions = [sublist[waypoint_index-1] for sublist in smooth_paths]

            reference_idx  = min(len(smooth_center)-1, 2+(k_sim - waypoint_index_offset)//900)
            for j in range(num_drones):
                print("state",state[:3,j],target_positions[j],np.linalg.norm(target_positions[j] - state[:3,j]),"waypoint index:",waypoint_index)
                ###update waypoints
                if np.linalg.norm(target_positions[j] - state[:3,j]) < 0.1 or (k_sim - last_waypoint_update_k)%500==0:# or np.linalg.norm(target_positions[j] - state[:3,j]) > 2:# and reference_idx > waypoint_index:
                    waypoint_index += 1
                    if waypoint_index == len(smooth_center): ###replanning criteria
                        map_start_point = [from_real_to_map(smooth_center[-1][0],x_min,x_max,x_slice_size),from_real_to_map(smooth_center[-1][1],y_min,y_max,y_slice_size)]
                        print("heading",heading)
                        execute_center_map,execute_path_angle = minimum_deform_search(
                            map_2d, map_start_point, map_end_point, heading, map_formation_pattern,
                            config.WAYPOINT_VOXEL_INTERVAL, config.CAMERA_FOV_DEG, config.PSO_RAYCAST_ANGLE_INTERVAL_DEG, map_camera_range, config.PSO_RAYCAST_DISTANCE_INTERVAL_VOXEL
                        )
                        ### extract path and heading angle
                        execute_center = []
                        execute_path = [[] for i in range(num_drones)]
                        execute_angle = [[] for i in range(num_drones)]
                        
                        ### doing smoothness to both path and angle sequence
                        smooth_center = []
                        smooth_paths = []
                        smooth_angles = []

                        for ID in range(num_drones):
                            for item in execute_path_angle[ID]:
                                sublist = [from_map_to_real(item[0][0],x_min,x_max,x_slice_size),from_map_to_real(item[0][1],y_min,y_max,y_slice_size)]
                                three_d_points = [sublist[0], sublist[1], 1]
                                execute_path[ID].append(three_d_points)
                                execute_angle[ID].append(math.radians(item[1]))
                        
                        for item in execute_center_map:
                            sublist = [from_map_to_real(item[0],x_min,x_max,x_slice_size),from_map_to_real(item[1],y_min,y_max,y_slice_size)]
                            three_d_points = [sublist[0], sublist[1], 1]
                            execute_center.append(three_d_points)
                        t,smooth_center = points2spline_points3D(execute_center,angle_n)
                        for ID in range(num_drones):
                            t,smooth_points = points2spline_points3D(execute_path[ID],angle_n)
                            t,smooth_angle = points2spline_angle(execute_angle[ID],angle_n)
                            smooth_paths.append(smooth_points)
                            smooth_angles.append(smooth_angle)
                        waypoint_index = 0
                    last_waypoint_update_k = k_sim
                    break
            ### draw the center path
            p.addUserDebugLine(
                lineFromXYZ=smooth_center[waypoint_index-1,:],
                lineToXYZ=smooth_center[waypoint_index,:],
                lineColorRGB=[1, 1, 1],
                lineWidth=5
                )
            # print("smooth center",smooth_center)
            # print("smooth paths",smooth_paths)
            # print("smooth angles",smooth_angles)


            ##### deviation tracking ######
            # if waypoint_index >= len(smooth_paths[0]):
            #     target_positions = [sublist[-1] for sublist in smooth_paths]
            #     old_target_positions = [sublist[-2] for sublist in smooth_paths]
            # else:
            #     target_positions = [sublist[waypoint_index] for sublist in smooth_paths]
            #     old_target_positions = [sublist[waypoint_index-1] for sublist in smooth_paths]

            # reference_idx  = min(len(smooth_center)-1, 2+(k_sim-waypoint_index_offset)//900)
            # target_deviation = 0
            # for j in range(num_drones):
            #     target_deviation += np.linalg.norm(target_positions[j] - state[:3,j])/num_drones
            #     print("state",state[:3,j],target_positions[j],np.linalg.norm(target_positions[j] - state[:3,j]),"waypoint index:",waypoint_index)
            # if target_deviation < 0.6:# or np.linalg.norm(target_positions[j] - state[:3,j]) > 2:# and reference_idx > waypoint_index:
            #     waypoint_index += 1
            #     last_waypoint_update_k = k_sim
            #     p.addUserDebugLine(
            #     lineFromXYZ=smooth_center[waypoint_index-1,:],
            #     lineToXYZ=smooth_center[waypoint_index,:],
            #     lineColorRGB=[1, 1, 1],
            #     lineWidth=5
            #     )


            # ###### center tracking ######
            # target_positions = []
            # if waypoint_index >= len(smooth_center):
            #     target_positions = smooth_center[-1]
            # else:
            #     target_positions = smooth_center[waypoint_index]

            
            
            # print("state",current_formation_center,target_positions,np.linalg.norm(target_positions - current_formation_center),"waypoint index:",waypoint_index)
            # if np.linalg.norm(target_positions - current_formation_center) < 0.3:# or np.linalg.norm(target_positions[j] - state[:3,j]) > 2:# and reference_idx > waypoint_index:
            #     waypoint_index += 1
            #     last_waypoint_update_k = k_sim
                
                # p.addUserDebugLine(
                # lineFromXYZ=smooth_center[waypoint_index-1,:],
                # lineToXYZ=smooth_center[waypoint_index,:],
                # lineColorRGB=[1, 1, 1],
                # lineWidth=5
                # )

                # print("old",path[waypoint_index-1,:],)
                # print("old",path[waypoint_index,:])
                # print("new",[sublist[waypoint_index-1] for sublist in execute_path_angle])
                # print("new",target_positions)

                # p.addUserDebugLine(
                # lineFromXYZ=old_target_positions[0],
                # lineToXYZ=target_positions[0],
                # lineColorRGB=[0, 1, 0],
                # lineWidth=5
                # )
            
            # print("target",target_positions)
            # print("old",path[waypoint_index,:])
            # current_formation_target = path[waypoint_index,:]
            current_formation_target = [sublist[waypoint_index] for sublist in smooth_paths]

            #### plot the pink line between each pair of agents
            temp_line = []
            for line_idx_i in range(num_drones):
                for line_idx_j in range(line_idx_i+1,num_drones):
                    print("idx",line_idx_i,line_idx_j)
                    line_id = p.addUserDebugLine(
                        lineFromXYZ=state[:3,line_idx_i],
                        lineToXYZ=state[:3,line_idx_j], 
                        lineColorRGB=[1, 0, 1],
                        lineWidth=3,
                        lifeTime = 2
                        )
                    line_id_1 = p.addUserDebugLine(
                        lineFromXYZ=state[:3,line_idx_i],
                        lineToXYZ=state[:3,line_idx_j], 
                        lineColorRGB=[1, 0, 1],
                        lineWidth=3,
                        lifeTime = 3
                        )

            ###draw agents path !!!!!!!!!!!!!!!!!!Remember to change here if the number of agents changes!!!!!!!!!!!!!!!!!!!!!!!!!
            p.addUserDebugPoints(
                last_state[:3,:].T,
                [[1, 0, 0],
                 [0, 0, 1],
                 [0, 1, 1],
                 [1, 1, 0],
                #  [0, 1, 0],     # 綠色
                #  [1, 0, 1],     # 紫色
                #  [1, 0.5, 0],   # 橙色
                #  [0.5, 0, 0.5], # 紫紅色
                #  [0.5, 0.5, 0.5], # 灰色
                #  [0.5, 0.5, 1],
                 ],
                pointSize=3,
            )

            # print('current waypoint',current_formation_target)
            obs_time_start = time.time()
            controller.update_data(current_formation_target,current_formation_center)
            obs_time_end = time.time()
            print("obs time",obs_time_end-obs_time_start)


            time_mpc_start = time.time()
            all_u0 = controller.run(state)
            time_mpc_end = time.time()
            print("MPC execute time", time_mpc_end-time_mpc_start)
            
            ### control agents heading ###
            for ID in range(num_drones):
                if waypoint_index >= len(smooth_angles[ID]):
                    current_yaw = smooth_angles[ID][-1]
                else:
                    current_yaw = smooth_angles[ID][waypoint_index]
                # print("current yaw",current_yaw,"waypoint index",waypoint_index,smooth_angles[ID][waypoint_index])
                target_rpys[ID] = [0, 0, current_yaw]


                ### Draw the field of view coverage for each agent
                next_position = []
                for edge_angle in [-int(config.CAMERA_FOV_DEG / 2), int(config.CAMERA_FOV_DEG / 2)]:
                    current_pos = all_waypoint[3*(ID):3*(ID+1)-1]
                    next_point = current_pos + config.CAMERA_RANGE * yaw_rad_to_unit_vector(current_yaw+np.radians(edge_angle))
                    next_position.append(np.array([next_point[0], next_point[1], all_waypoint[3*(ID)+2]]))
                    line_id = p.addUserDebugLine(
                        lineFromXYZ=all_waypoint[3*(ID):3*(ID+1)],
                        lineToXYZ=next_position[len(next_position)-1],
                        lineColorRGB=[0, 0, 0],
                        lineWidth=30,
                        lifeTime = 1,
                        )
                    # print("target", next_position)
                line_id = p.addUserDebugLine(
                        lineFromXYZ=next_position[0],
                        lineToXYZ=next_position[1],
                        lineColorRGB=[0, 0, 0],
                        lineWidth=30,
                        lifeTime = 1,
                        )

            # print('waypoint',path[waypoint_index,:],waypoint_index)
            # print('ellipsoid_obs',ellipsoid_obs.shape,'\n',ellipsoid_obs)
            # print('state',state)
        

            # 3. send control command and run simulation ##
            waypoint_time_start = time.time()
            all_waypoint = calculate_waypoint(state,all_u0,dt = dt) #all_waypoint.shape =(dim*n_agent,)
            # print("waypoint",all_waypoint)
            controller.post_update_data()
            waypoint_time_end = time.time()
            print("waypoint time:",waypoint_time_end-waypoint_time_start)
            
        

        #### Compute control at the desired frequency ##############
        if k_sim%aggregate_phy_step_count == 0:

            #### Compute control for the current way point #############
            control_time_start = time.time()
            for j in range(num_drones):
                target_pos = all_waypoint[j*3:(j+1)*3]
                action[str(j)], _, _ = ctrl[j].computeControlFromState(control_timestep=aggregate_phy_step_count*env.TIMESTEP,
                                                                       state=obs[str(j)]["state"],
                                                                       target_pos=target_pos,
                                                                       target_rpy=target_rpys[j, :]
                                                                       )
            control_time_end = time.time()
            # print("control time:",control_time_end-control_time_start)

            
        #### Log the simulation ####################################
        for j in range(num_drones):
            all_waypoint[j*3:(j+1)*3]
            logger.log(drone=j,
                       timestamp=k_sim/env.SIM_FREQ,
                       state=obs[str(j)]["state"],
                       control=np.hstack([all_waypoint[j*3:(j+1)*3], target_rpys[j, :], np.zeros(6)])
                       )
        #### Printout ##############################################
        # if k_sim%env.SIM_FREQ == 0:
            env.render()
            #### Print matrices with the images captured by each drone #
            # if vision:
            #     for j in range(num_drones):
            #         print(obs[str(j)]["rgb"].shape, np.average(obs[str(j)]["rgb"]),
            #               obs[str(j)]["dep"].shape, np.average(obs[str(j)]["dep"]),
            #               obs[str(j)]["seg"].shape, np.average(obs[str(j)]["seg"])
            #               )

        #### Sync the simulation ###################################
        if gui:
            sync(k_sim, simulation_start_clock_time, env.TIMESTEP)

        current_formation_center = np.mean(state[:controller.dim,:],axis = 1)
        if util.euclidean_distance(current_formation_center, config.FORMATION_GOAL_POINT) < 1.5:
            print("IT'S TIME TO GO TO BED !!!")
            # break
            #### Close the environment #################################
            env.close()

            #### Save the simulation results ###########################
            logger.save()
            logger.save_as_csv("pid") # Optional CSV save

            #### Plot the simulation results ###########################
            if should_plot:
                logger.plot()
                obstacle_processor.plot_occupancy_map_2d(1)
                obstacle_processor.plot_pcd()
                obstacle_processor.plot_occupancy_grid()

if __name__ == "__main__":
    #### Define and parse (optional) arguments for the script ##
    parser = argparse.ArgumentParser(description='Helix flight script using CtrlAviary or VisionAviary and DSLPIDControl')
    parser.add_argument('--drone',              default=config.PYBULLET_DRONE_MODEL,     type=DroneModel,    help='Drone model (default: CF2X)', metavar='', choices=DroneModel)
    parser.add_argument('--num_drones',         default=config.DRONE_N,          type=int,           help='Number of drones (default: 3)', metavar='')
    parser.add_argument('--physics',            default=config.PYBULLET_PHYSICS,      type=Physics,       help='Physics updates (default: PYB)', metavar='', choices=Physics)
    parser.add_argument('--vision',             default=config.VISION,      type=str2bool,      help='Whether to use VisionAviary (default: False)', metavar='')
    parser.add_argument('--gui',                default=config.PYBULLET_GUI,       type=str2bool,      help='Whether to use PyBullet GUI (default: True)', metavar='')
    parser.add_argument('--record_video',       default=config.PYBULLET_RECORD_DRONE_VISION,      type=str2bool,      help='Whether to record a video (default: False)', metavar='')
    parser.add_argument('--should_plot',        default=config.SHOULD_PLOT,       type=str2bool,      help='Whether to plot the simulation results (default: True)', metavar='')
    parser.add_argument('--add_obstacles',      default=config.PYBULLET_ADD_OBSTACLES,       type=str2bool,      help='Whether to add obstacles to the environment (default: True)', metavar='')
    parser.add_argument('--simulation_freq_hz', default=config.PYBULLET_SIM_FREQ_HZ,        type=int,           help='Simulation frequency in Hz (default: 240)', metavar='')
    parser.add_argument('--output_folder',      default=config.OUTPUT_FOLDER, type=str,           help='Folder where to save logs (default: "results")', metavar='')
    ARGS = parser.parse_args()

    # run(**vars(ARGS))
    run()