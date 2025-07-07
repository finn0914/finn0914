### This is used to convert pcd to the occupancy map ###
### update_point_cloud_observation is for simulation ###

#! /usr/bin/env python3
import numpy as np
import open3d as o3d
import cv2
import threading
import matplotlib.pyplot as plt
import pandas as pd
import math
import time
from occupied_map import known_map_establish,get_boundary
import datetime
import csv
import os
from concurrent.futures import ThreadPoolExecutor


from point_cloud2_to_o3d import PCDGenerator
import queue

class Obstacle_Processor():
    def __init__(self,map_size,voxel_size,boundary,FOV_degree,visible_distance) -> None:
        self.path = './results/kgr/simulation1/2/centerNotPassing/'
        self.n = 0
        self.map_size = [(map_size[0]),(map_size[1]),(map_size[2])]
        self.voxel_size = voxel_size
        self.pcd = o3d.geometry.PointCloud()
        self.occupancy_grid = np.full((map_size[1], map_size[0], self.map_size[2]), -1, dtype=int)
        self.real_occupancy_grid = np.full((map_size[1], map_size[0], self.map_size[2]), 0, dtype=int)
        self.score_map = np.full((map_size[1], map_size[0], self.map_size[2]), 0, dtype=int)
        self.boundary = boundary
        print("boundary",self.boundary)
        self.FOV = FOV_degree
        self.visible_distance = visible_distance

        # The queue automatically fills up with PCD.
        # You can periodically empty the data for processing.
        self.fig = plt.figure()
        # self.fig1 = plt.figure()
        # self.ax = self.fig1.add_subplot(111, projection='3d')
        self.pcd_queue = queue.Queue(maxsize=10)

        # Call `start` and `stop` to start and stop.
        self.pcd_generator = PCDGenerator(self.pcd_queue)
    def save_msg(self,queue):
        self.pcd_generator.start()
        # 創建保存數據的目錄
        os.makedirs('pcd_files/0702/pcd1', exist_ok=True)

        # 打開 CSV 文件以寫入數據
        with open('pcd_files/0702/data1.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            
            # 寫入 CSV 標題
            writer.writerow(['x', 'y', 'z', 'yaw', 'pcd_filename'])
            while True:
                if not queue.empty(): 
                    x,y,z,yaw,pcd  = queue.get()
                                    
                    # 生成 PCD 文件名
                    pcd_filename = f'pcd_{x}_{y}_{z}_{yaw}.pcd'
                    
                    pcd_filepath = os.path.join('pcd_files/0702/pcd1', pcd_filename)
                    o3d.io.write_point_cloud(pcd_filepath, pcd)
                    
                    # 將 x, y, z, yaw 和 PCD 文件名寫入 CSV
                    writer.writerow([x, y, z, yaw, pcd_filename])
    def calculate_visible_coordinates(self, map, center_x, center_y, center_z, angle_degrees):
        # print("map shape",map.shape)
        # 将角度转换为弧度
        angle_radians = math.radians(angle_degrees)
        camera_FOV_radians = math.radians(self.FOV)/2

        # 计算视野范围内的角度边界
        min_angle = angle_radians - camera_FOV_radians
        max_angle = angle_radians + camera_FOV_radians
        fov_radians = math.radians(self.FOV)  # 将角度转换为弧度
        # z_max = round(center_z + visible_distance * math.tan(fov_radians / 2))
        # z_min = round(center_z - visible_distance * math.tan(fov_radians / 2))
        # z_max = max(min(z_max, self.map_size - 1), 0)
        # z_min = max(min(z_min, self.map_size - 1), 0)

        # 初始化视野范围内的坐标列表
        visible_coordinates = []
        obs_coordinates = []

        # 遍历视野范围内的每个角度
        for current_angle in range(int(math.degrees(min_angle)), int(math.degrees(max_angle)) + 1):
            # 将当前角度转换为弧度
            current_angle_radians = math.radians(current_angle)
            # print("current_angle_radians", current_angle_radians)
            weight = 0
            x_min,x_max,y_min,y_max,z_min,z_max = self.boundary
            z_layer = int((center_z-z_min)/(z_max-z_min)*self.map_size[2])
            while weight < self.visible_distance:
                # 根据视距计算坐标增量
                delta_x = (weight * math.cos(current_angle_radians))
                delta_y = (weight * math.sin(current_angle_radians))

                # 计算坐标
                # print("update voxels", center_x,center_y,delta_x,delta_y)
                coordinate_x = center_x + delta_x
                coordinate_y = center_y + delta_y
                i,j = int((coordinate_x-x_min)/(x_max-x_min)*self.map_size[0]),int((coordinate_y-y_min)/(y_max-y_min)*self.map_size[1])
                # print("map",i,j)
                if (-1 < j < map.shape[0] and -1 < i < map.shape[1]):
                #     print("map",map[j][i])
                    if map[j][i][z_layer] == True:                    
                        visible_coordinates.append([i, j])
                #         # for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1), (0 ,0)]:
                #         #     nx, ny = coordinate_x + dx, coordinate_y + dy
                #         #     obs_coordinates.append([nx, ny])
                #         # for j in range(1):
                #         #     coordinate_x = center_x + round((i+j) * math.cos(current_angle_radians))
                #         #     coordinate_y = center_y + round((i+j) * math.sin(current_angle_radians))
                #         # print("coordinate",coordinate_x,coordinate_y)
                        break
                # #     # 添加坐标到列表中            
                    if([i, j] not in visible_coordinates):
                        visible_coordinates.append([i, j])
                    # print("visible", visible_coordinates)
                weight+=0.1
        # plt.imshow(known_map[:,:,20])
        # for pos in visible_coordinates:
        # plt.plot([point[0] for point in visible_coordinates],[point[1] for point in visible_coordinates])
        # plt.show()
        return visible_coordinates,obs_coordinates
    def start(self):
        # self.occupancy_map_thread = threading.Thread(
        #     target=self.save_msg, args=(self.pcd_queue, ), daemon=True)
        # self.occupancy_map_thread.start()
        self.occupancy_map_thread = threading.Thread(
            target=self.occupancy_map_establish, args=(self.pcd_queue, ), daemon=True)
        self.occupancy_map_thread.start()

    def occupancy_map_establish(self,pcd_queue):
        print("Occupancy_map_establish")
        directions = [(0,0,0),(1,0,0),(0,1,0),(-1,0,0),(0,-1,0),(0,0,1),(0,0,-1)]
        self.pcd_generator.start()
        top = [(0,0,0),(self.occupancy_grid.shape[1]-1,self.occupancy_grid.shape[0]-1,self.occupancy_grid.shape[2]-1)]
        for point in top:
            self.occupancy_grid[point[1]][point[0]][point[2]] = 1
        n = 0
        while True:
            # print(self.pcd_queue.qsize())
            if pcd_queue.qsize()>0:
                start = time.time()
                # print("start mapping")
                x,y,z,yaw,new_pcd = pcd_queue.get() #x,y,z,yaw,pcd

                self.pcd = self.pcd + new_pcd
                self.pcd = self.pcd.voxel_down_sample(voxel_size=voxel_size)
                # self.pcd_queue.queue.clear()
                # print("pose and pcd",x,y,z,yaw,len(new_pcd.points))
                downpcd = new_pcd.voxel_down_sample(voxel_size=0.5)
                x_min,x_max,y_min,y_max,z_min,z_max = self.boundary
                occupancy_grid = np.zeros((self.map_size[1],self.map_size[0],self.map_size[2]), dtype=bool)
                i = 0
                for point in downpcd.points:
                    i+=1
                    if point[2]>self.boundary[5] or point[2]<self.boundary[4] or point[0]<self.boundary[0] or point[0]>self.boundary[1] or point[1]<self.boundary[2] or point[1]>self.boundary[3]:
                        continue
                    # print("point",point)
                    voxel_index = int((point[0]-x_min)/(x_max-x_min)*self.map_size[0]),int((point[1]-y_min)/(y_max-y_min)*self.map_size[1]),int((point[2]-z_min)/(z_max-z_min)*self.map_size[2])
                    voxel_index = list(voxel_index)
                    
                    # for direction in directions:
                    #     update_index = [0,0,0]
                    #     update_index[0] = max(min(voxel_index[0]+direction[0], self.map_size[0] - 1), 0)
                    #     update_index[1] = max(min(voxel_index[1]+direction[1], self.map_size[1] - 1), 0)
                    #     update_index[2] = max(min(voxel_index[2]+direction[2], self.map_size[2] - 1), 0)
                    self.real_occupancy_grid[voxel_index[1], voxel_index[0], voxel_index[2]] = 1
                yaw_deg = float(yaw)/np.pi*180
                # update_positions,obstacle_positions = self.calculate_visible_coordinates(occupancy_grid,float(x),float(y),float(z),yaw_deg)
                time3 = time.time()
                # print("update index",time3-time2)
                # # print("len",len(update_positions))

                # z_layer = 1
                # z_layer = int((z_layer-z_min)/(z_max-z_min)*self.map_size[2])

                # for pos in update_positions:
                #     i,j = pos[0],pos[1]
                #     if occupancy_grid[j][i][z_layer]==1:
                #         self.score_map[j][i][z_layer] += 4
                #     elif occupancy_grid[j][i][z_layer]==0:
                #         self.score_map[j][i][z_layer] -= 4
                #     if self.score_map[j][i][z_layer] >=12:
                #         for z in range(0,self.map_size[2]):                          
                #             self.occupancy_grid[j][i][z] = 1
                #     elif self.score_map[j][i][z_layer] <-3:
                #         for z in range(0,self.map_size[2]):                          
                #             self.occupancy_grid[j][i][z] = 0
                # time4 = time.time()
                # # print("global map update",time4-time3)

                # if self.n%2==0:
                #     self.plot_occupancy_grid(self.occupancy_grid)
                #     # self.plot_occupancy_map_2d(1)
                #     # o3d.io.write_point_cloud(file, self.pcd)
                    
                #     # plt.imshow(z_slice, cmap='binary', origin='lower')
                # self.n+=1
                # end = time.time()
                # print("end mapping",end-start)
    def calculate_visible_coordinates_3d(self, map, center_x, center_y, center_z, yaw_degrees):
        # 将角度转换为弧度
        yaw_radians = math.radians(yaw_degrees)
        camera_FOV_radians = math.radians(self.FOV) / 2

        # 计算视野范围内的角度边界
        min_yaw = yaw_radians - camera_FOV_radians
        max_yaw = yaw_radians + camera_FOV_radians
        min_pitch = -camera_FOV_radians
        max_pitch = camera_FOV_radians

        # 初始化视野范围内的坐标列表
        visible_coordinates = []
        obs_coordinates = []

        # 预计算三角函数结果
        yaw_angles = [math.radians(yaw) for yaw in range(int(math.degrees(min_yaw)), int(math.degrees(max_yaw)) + 1)]
        pitch_angles = [math.radians(pitch) for pitch in range(int(math.degrees(min_pitch)), int(math.degrees(max_pitch)) + 1)]
        cos_yaw = {angle: math.cos(angle) for angle in yaw_angles}
        sin_yaw = {angle: math.sin(angle) for angle in yaw_angles}
        cos_pitch = {angle: math.cos(angle) for angle in pitch_angles}
        sin_pitch = {angle: math.sin(angle) for angle in pitch_angles}

        # 线程池并行计算
        def compute_coordinates(current_yaw, current_pitch):
            local_visible_coordinates = []
            weight = 0
            x_min, x_max, y_min, y_max, z_min, z_max = self.boundary
            while weight < self.visible_distance:
                # 根据视距计算坐标增量
                delta_x = weight * cos_pitch[current_pitch] * cos_yaw[current_yaw]
                delta_y = weight * cos_pitch[current_pitch] * sin_yaw[current_yaw]
                delta_z = weight * sin_pitch[current_pitch]

                # 计算坐标
                coordinate_x = center_x + delta_x
                coordinate_y = center_y + delta_y
                coordinate_z = center_z + delta_z

                i = int((coordinate_x - x_min) / (x_max - x_min) * self.map_size[0])
                j = int((coordinate_y - y_min) / (y_max - y_min) * self.map_size[1])
                k = int((coordinate_z - z_min) / (z_max - z_min) * self.map_size[2])

                if (-1 < j < map.shape[0] and -1 < i < map.shape[1] and -1 < k < map.shape[2]):
                    if map[j][i][k] == True:
                        local_visible_coordinates.append([i, j, k])
                        break
                    if [i, j, k] not in local_visible_coordinates:
                        local_visible_coordinates.append([i, j, k])
                weight += 0.1
            return local_visible_coordinates

        # 使用线程池并行处理
        with ThreadPoolExecutor() as executor:
            results = []
            for current_yaw in yaw_angles:
                for current_pitch in pitch_angles:
                    results.append(executor.submit(compute_coordinates, current_yaw, current_pitch))
            
            for result in results:
                visible_coordinates.extend(result.result())

        return visible_coordinates, obs_coordinates
    def update_point_cloud_observation_3d(self,agent_index,new_pcd,position,yaw,voxel_size = 0.15):
        self.n+=1
        directions = [(0,0,0),(1,0,0),(0,1,0),(-1,0,0),(0,-1,0),(0,0,1),(0,0,-1)]
        # new_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        
        # if self.n%10==0:
        #     df = pd.DataFrame([[agent_index, agent_position[0], agent_position[1], agent_position[2], yaw]])
        #     df.to_csv(self.path+'state.csv',mode = 'a',header=False ,index=False)
        #     file = self.path+f'{self.n}.pcd'
        #     o3d.io.write_point_cloud(file, new_pcd)
        # self.n+=1
        time0 = time.time()

        self.pcd = self.pcd + new_pcd
        pcd_filepath = self.path + f"{self.n}.pcd"
        # o3d.io.write_point_cloud(pcd_filepath, new_pcd)
        self.pcd = self.pcd.voxel_down_sample(voxel_size=voxel_size)
        # if self.n%10==0:
        downpcd = new_pcd.voxel_down_sample(voxel_size=self.voxel_size)
        x_min,x_max,y_min,y_max,z_min,z_max = self.boundary
        occupancy_grid = np.zeros((self.map_size[1],self.map_size[0],self.map_size[2]), dtype=bool)
        append_x_min,append_y_min,append_z_min  = self.pcd.get_min_bound()
        append_x_max,append_y_max, append_z_max = self.pcd.get_max_bound()
        # print(append_x_min,append_x_max,append_y_min,append_y_max,append_z_min, append_z_max )
        time1 = time.time()
        # print("pre",time1-time0)
        for point in downpcd.points:
            # print("point",point)
            if point[2]>self.boundary[5] or point[2]<self.boundary[4] or point[0]<self.boundary[0] or point[0]>self.boundary[1] or point[1]<self.boundary[2] or point[1]>self.boundary[3]:
                continue
            voxel_index = int((point[0]-x_min)/(x_max-x_min)*self.map_size[0]),int((point[1]-y_min)/(y_max-y_min)*self.map_size[1]),int((point[2]-z_min)/(z_max-z_min)*self.map_size[2])
            voxel_index = list(voxel_index)
            # print("voxel index",voxel_index)
            # voxel_index[0] = max(min(voxel_index[0], self.map_size[0] - 1), 0)
            # voxel_index[1] = max(min(voxel_index[1], self.map_size[1] - 1), 0)
            # voxel_index[2] = max(min(voxel_index[2], self.map_size[2] - 1), 0)
            # # voxel_index = ((point - downpcd.get_min_bound()) / self.voxel_size).astype(int)
            # # print(voxel_index)
            # occupancy_grid[voxel_index[1], voxel_index[0], voxel_index[2]] = 1
            for direction in directions:
                update_index = [0,0,0]
                update_index[0] = max(min(voxel_index[0]+direction[0], self.map_size[0] - 1), 0)
                update_index[1] = max(min(voxel_index[1]+direction[1], self.map_size[1] - 1), 0)
                update_index[2] = max(min(voxel_index[2]+direction[2], self.map_size[2] - 1), 0)
                occupancy_grid[update_index[1], update_index[0], update_index[2]] = 1
            self.real_occupancy_grid[voxel_index[1], voxel_index[0], voxel_index[2]] = 1
        time2 = time.time()
        # print("occupancy establish",time2-time1)
        # 将occupancy_grid的值复制到self.occupancy_grid中
        agent_position = int((float(position[0])-x_min)/(x_max-x_min)*self.map_size[0]),int((float(position[1])-y_min)/(y_max-y_min)*self.map_size[1]),int((float(position[2])-z_min)/(z_max-z_min)*self.map_size[2])
        yaw_deg = float(yaw)/np.pi*180
        # print("yaw",yaw_deg,"y", yaw)
        # print("position",agent_position,"pos",position)

        update_positions,obstacle_positions = self.calculate_visible_coordinates_3d(occupancy_grid,float(position[0]),float(position[1]),float(position[2]),yaw_deg)
        time3 = time.time()
        print("update index",time3-time2)
        # print("len",len(update_positions))

        for pos in update_positions:
            i,j,k = pos[0],pos[1],pos[2]
            if occupancy_grid[j][i][k]==1:
                self.score_map[j][i][k] += 4
            elif occupancy_grid[j][i][k]==0:
                self.score_map[j][i][k] -= 4
            if self.score_map[j][i][k] >=12:                         
                self.occupancy_grid[j][i][k] = 1
            elif self.score_map[j][i][k] <-3:                         
                self.occupancy_grid[j][i][k] = 0
        time4 = time.time()
        # self.plot_real_occupancy_grid()

        z_layer = 1
        z_layer = int((z_layer-z_min)/(z_max-z_min)*self.map_size[2])

        z_slice = self.occupancy_grid[:, :, z_layer]
        if self.n%12==0:
            self.plot_occupancy_map_2d(1)

        return z_slice
        
    def update_point_cloud_observation(self,agent_index,new_pcd,position,yaw,voxel_size = 0.15):
        self.n+=1
        directions = [(0,0,0),(1,0,0),(0,1,0),(-1,0,0),(0,-1,0),(0,0,1),(0,0,-1)]
        # new_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        
        # if self.n%10==0:
        #     df = pd.DataFrame([[agent_index, agent_position[0], agent_position[1], agent_position[2], yaw]])
        #     df.to_csv(self.path+'state.csv',mode = 'a',header=False ,index=False)
        #     file = self.path+f'{self.n}.pcd'
        #     o3d.io.write_point_cloud(file, new_pcd)
        # self.n+=1
        time0 = time.time()

        self.pcd = self.pcd + new_pcd
        pcd_filepath = self.path + f"{self.n}.pcd"
        # o3d.io.write_point_cloud(pcd_filepath, new_pcd)
        self.pcd = self.pcd.voxel_down_sample(voxel_size=voxel_size)
        # if self.n%10==0:
        downpcd = new_pcd.voxel_down_sample(voxel_size=self.voxel_size)
        x_min,x_max,y_min,y_max,z_min,z_max = self.boundary

        ### transform pcd to occupied voxel 
        occupancy_grid = np.zeros((self.map_size[1],self.map_size[0],self.map_size[2]), dtype=bool)
        append_x_min,append_y_min,append_z_min  = self.pcd.get_min_bound()
        append_x_max,append_y_max, append_z_max = self.pcd.get_max_bound()
        # print(append_x_min,append_x_max,append_y_min,append_y_max,append_z_min, append_z_max )
        time1 = time.time()
        # print("pre",time1-time0)
        for point in downpcd.points:
            # print("point",point)
            if point[2]>self.boundary[5] or point[2]<self.boundary[4] or point[0]<self.boundary[0] or point[0]>self.boundary[1] or point[1]<self.boundary[2] or point[1]>self.boundary[3]:
                continue
            voxel_index = int((point[0]-x_min)/(x_max-x_min)*self.map_size[0]),int((point[1]-y_min)/(y_max-y_min)*self.map_size[1]),int((point[2]-z_min)/(z_max-z_min)*self.map_size[2])
            voxel_index = list(voxel_index)
            # print("voxel index",voxel_index)
            # voxel_index[0] = max(min(voxel_index[0], self.map_size[0] - 1), 0)
            # voxel_index[1] = max(min(voxel_index[1], self.map_size[1] - 1), 0)
            # voxel_index[2] = max(min(voxel_index[2], self.map_size[2] - 1), 0)
            # # voxel_index = ((point - downpcd.get_min_bound()) / self.voxel_size).astype(int)
            # # print(voxel_index)
            # occupancy_grid[voxel_index[1], voxel_index[0], voxel_index[2]] = 1

            ### enlarge the obstacle
            for direction in directions:
                update_index = [0,0,0]
                update_index[0] = max(min(voxel_index[0]+direction[0], self.map_size[0] - 1), 0)
                update_index[1] = max(min(voxel_index[1]+direction[1], self.map_size[1] - 1), 0)
                update_index[2] = max(min(voxel_index[2]+direction[2], self.map_size[2] - 1), 0)
                occupancy_grid[update_index[1], update_index[0], update_index[2]] = 1
            self.real_occupancy_grid[voxel_index[1], voxel_index[0], voxel_index[2]] = 1
        time2 = time.time()
        # print("occupancy establish",time2-time1)
        agent_position = int((float(position[0])-x_min)/(x_max-x_min)*self.map_size[0]),int((float(position[1])-y_min)/(y_max-y_min)*self.map_size[1]),int((float(position[2])-z_min)/(z_max-z_min)*self.map_size[2])
        yaw_deg = float(yaw)/np.pi*180
        # print("yaw",yaw_deg,"y", yaw)
        # print("position",agent_position,"pos",position)

        ### find out the update space
        update_positions,obstacle_positions = self.calculate_visible_coordinates(occupancy_grid,float(position[0]),float(position[1]),float(position[2]),yaw_deg)
        time3 = time.time()
        # print("update index",time3-time2)
        # print("len",len(update_positions))

        z_layer = 1
        k = int((z_layer-z_min)/(z_max-z_min)*self.map_size[2])
        
        for pos in update_positions:
            i,j= pos[0],pos[1]
            ### according to the update space, do the scoring
            if occupancy_grid[j][i][k]==1:
                self.score_map[j][i][k] += 4
            elif occupancy_grid[j][i][k]==0:
                self.score_map[j][i][k] -= 4

            ### check the scoring 
            if self.score_map[j][i][k] >=12:                         
                self.occupancy_grid[j][i][k] = 1
            elif self.score_map[j][i][k] <-3:                         
                self.occupancy_grid[j][i][k] = 0
        # for pos in update_positions:
        #     i,j = pos[0],pos[1]
        #     if occupancy_grid[j][i][z_layer]==1:
        #         self.score_map[j][i][z_layer] += 4
        #     elif occupancy_grid[j][i][z_layer]==0:
        #         self.score_map[j][i][z_layer] -= 4
        #     if self.score_map[j][i][z_layer] >=12:
        #         for z in range(0,self.map_size[2]):                          
        #             self.occupancy_grid[j][i][z] = 1
        #     elif self.score_map[j][i][z_layer] <-3:
        #         for z in range(0,self.map_size[2]):                          
        #             self.occupancy_grid[j][i][z] = 0
        time4 = time.time()
        # print("global map update",time4-time3)

        z_slice = self.occupancy_grid[:, :, k]
        # if self.n%12==0:
        #     # self.plot_real_occupancy_grid()
        #     self.plot_occupancy_map_2d(1)
        # o3d.visualization.draw_geometries([downpcd])
        # plt.figure()
        # plt.imshow(z_slice, cmap='binary', origin='lower')
        # plt.show()
        # file = self.path+f'{self.n}.pcd'
        # o3d.io.write_point_cloud(file, downpcd)
        # if self.n%12==0:
        #     self.plot_occupancy_map_2d(1)
            # o3d.io.write_point_cloud(file, self.pcd)
            
        #     # plt.imshow(z_slice, cmap='binary', origin='lower')
        # self.n+=1


        # # 绘制特定z层
        # plt.imshow(z_slice, cmap='binary', origin='lower')
        # plt.show()
        # plt.pause(0.001)

        ### return 2D map
        return z_slice
    
    def plot_occupancy_grid(self, color_map='viridis'):
        if self.occupancy_grid.ndim == 2:
            # 二维布尔数组的可视化
            plt.imshow(self.occupancy_grid, cmap='binary', origin='lower')
            plt.colorbar()
            plt.title('Occupancy Grid')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.show()
        elif self.occupancy_grid.ndim == 3:
            # 三维布尔数组的可视化
            # fig = plt.figure()
            ax = self.fig.add_subplot(111, projection='3d')

            y, x, z = np.where(self.occupancy_grid)# 创建颜色映射
            cmap = plt.get_cmap(color_map)
            norm = plt.Normalize(z.min(), z.max())
            colors = cmap(norm(z))

            ax.scatter(x, y, z, c=colors, cmap=color_map)
            ax.set_title('Occupancy Grid')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            plt.show()
            
        else:
            print("Unsupported occupancy grid dimensions.")
    # def plot_real_occupancy_grid(self, color_map='viridis'):
    #     x_min,x_max,y_min,y_max,z_min,z_max = self.boundary
    #     if self.occupancy_grid.ndim == 2:
    #         # 二维布尔数组的可视化
    #         plt.imshow(self.real_occupancy_grid, cmap='binary', origin='lower')
    #         plt.colorbar()
    #         plt.title('Occupancy Grid')
    #         plt.xlabel('X')
    #         plt.ylabel('Y')
    #         plt.show()
    #     elif self.occupancy_grid.ndim == 3:
    #         # 三维布尔数组的可视化
    #         # fig = plt.figure()
    #         # edgecolor = (0, 0, 0, 0.5)
    #         # self.ax.voxels(self.real_occupancy_grid, edgecolor=edgecolor)
    #         # self.ax.set_title('Occupancy Grid')
    #         # self.ax.set_xlabel('X')
    #         # self.ax.set_ylabel('Y')
    #         # self.ax.set_zlabel('Z')
    #         # self.ax.x_lim(x_min,x_max)
    #         # self.ax.y_lim(y_min,y_max)
    #         y, x, z = np.where(self.real_occupancy_grid)# 创建颜色映射
    #         cmap = plt.get_cmap(color_map)
    #         norm = plt.Normalize(z.min(), z.max())
    #         colors = cmap(norm(z))

    #         self.ax.scatter(x, y, z, c=colors, cmap=color_map)
    #         self.ax.set_xlim(self.ax.get_xlim()[::-1])
    #         self.ax.set_title('Occupancy Grid')
    #         self.ax.set_xlabel('X')
    #         self.ax.set_ylabel('Y')
    #         self.ax.set_zlabel('Z')
    #         self.ax.set_xlim(0,self.map_size[1])
    #         self.ax.set_ylim(0,self.map_size[0])
    #         self.ax.set_zlim(0,self.map_size[2])
            
    #         now = datetime.datetime.now()

    #         # Format the date and time as a string
    #         timestamp = now.strftime("%Y%m%d_%H%M%S")

    #         # Create a file name with the timestamp
    #         filename = self.path + f"occupancy_map_{timestamp}.png"

    #         plt.savefig(filename)
            
            
    #     else:
    #         print("Unsupported occupancy grid dimensions.")
    def plot_real_occupancy_grid(self, color_map='viridis'):
        if self.real_occupancy_grid.ndim == 2:
            # 二维布尔数组的可视化
            plt.imshow(self.real_occupancy_grid, cmap='binary', origin='lower')
            plt.colorbar()
            plt.title('Occupancy Grid')
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.show()
        elif self.real_occupancy_grid.ndim == 3:
            print("processing......")
            # 三维布尔数组的可视化
            # fig = plt.figure()
            ax = self.fig.add_subplot(111, projection='3d')

            
            y, x, z = np.where(self.real_occupancy_grid)# 创建颜色映射
            cmap = plt.get_cmap(color_map)
            # norm = plt.Normalize(0,1)
            # norm = plt.Normalize(z.min(), z.max())
            # colors = cmap(norm(z))
            print("z:",z.min(),z.max())
            sc = ax.scatter(x, y, z, c=z, cmap=color_map)
            # ax.set_title('Occupancy Grid')
            # ax.set_xlabel('X')
            # ax.set_ylabel('Y')
            # ax.set_zlabel('Z')
            ax.set_axis_off()
            ax.set_xlim(0,120)
            ax.set_ylim(50,160)
            # ax.set_xlim(0,self.map_size[0])
            # ax.set_ylim(0,self.map_size[1])
            ax.set_zlim(0,self.map_size[2])
            cbar = self.fig.colorbar(sc, ax=ax, shrink=0.5, aspect=5)
            # cbar.set_ticks([0, 1, 14])
            cbar.set_label('Color Bar')
            ax.grid(False)
            print("finish processing!!")
            plt.show()
            # edgecolor = (0, 0, 0, 0.5)
            # ax.voxels(self.real_occupancy_grid, edgecolor=edgecolor)
            # ax.set_title('Occupancy Grid')
            # ax.set_xlabel('X')
            # ax.set_ylabel('Y')
            # ax.set_zlabel('Z')
            # plt.show()
            
        else:
            print("Unsupported occupancy grid dimensions.")
        
    def plot_pcd(self):
        o3d.visualization.draw_geometries([self.pcd])

    def plot_occupancy_map_2d(self,z_layer):
        """
        Plot a specified Z layer of a 3D grid map using Matplotlib.

        Args:
        - self.occupancy_grid (numpy.ndarray): 3D grid map array.
        - z_layer (int): Index of the Z layer to plot.

        Returns:
        - None
        """
        print("z layer",z_layer)
        x_min,x_max,y_min,y_max,z_min,z_max = self.boundary
        print("z layer",self.boundary,self.map_size[2])
        plt.title(f"Z Layer {z_layer} of 3D Grid Map")
        print("z layer",z_layer-z_min)
        z_layer = int((z_layer-z_min)/(z_max-z_min)*self.map_size[2])
        # 提取指定 Z 层的数据
        z_slice = self.occupancy_grid[:, :, z_layer]

        # 使用 Matplotlib 绘制指定 Z 层的数据
        plt.imshow(z_slice, cmap='binary', origin='lower')
        plt.xlabel("X")
        plt.ylabel("Y")
        # # plt.colorbar(label="Occupancy")
        now = datetime.datetime.now()

        # Format the date and time as a string
        timestamp = now.strftime("%Y%m%d_%H%M%S")

        # Create a file name with the timestamp
        filename = self.path + f"execute_path_{timestamp}.png"

        # save_result = plt.savefig(filename)
        # if save_result is None:
        #     print("Save operation completed successfully.")
        # else:
        #     print(f"Save operation returned: {save_result}")
        plt.show()

    def plot_real_occupancy_map_2d(self,z_layer):
        """
        Plot a specified Z layer of a 3D grid map using Matplotlib.

        Args:
        - self.occupancy_grid (numpy.ndarray): 3D grid map array.
        - z_layer (int): Index of the Z layer to plot.

        Returns:
        - None
        """
        print("z layer",z_layer)
        x_min,x_max,y_min,y_max,z_min,z_max = self.boundary
        print("z layer",self.boundary,self.map_size[2])
        plt.title(f"Z Layer {z_layer} of 3D Grid Map")
        print("z layer",z_layer-z_min)
        z_layer = int((z_layer-z_min)/(z_max-z_min)*self.map_size[2])
        # 提取指定 Z 层的数据
        z_slice = self.real_occupancy_grid[:, :, z_layer]

        # 使用 Matplotlib 绘制指定 Z 层的数据
        plt.imshow(z_slice, cmap='binary', origin='lower')
        plt.xlabel("X")
        plt.ylabel("Y")
        # # plt.colorbar(label="Occupancy")
        # now = datetime.datetime.now()

        # # Format the date and time as a string
        # timestamp = now.strftime("%Y%m%d_%H%M%S")

        # # Create a file name with the timestamp
        # filename = "./results/" + f"execute_path_{timestamp}.png"

        # plt.savefig(filename)
        plt.pause(1)
    
if __name__ == "__main__":
    formation_center = np.array([1,6,1])
    formation_target = np.array([12,4,1])  
    formation_pattern = np.array([[1,0,0],
                                  [0,1,0],
                                  [0,-1,0],
                         ])
    obstacle = np.array([   
    [  4.6,  7.8,  8,1,1,20],
    [ 7.3,    4.6,  8,0.4,0.4,20],
    [ 13,    0,  8,0.01,0.01,20],
                ])

    # formation_center = np.array([1,5,1])
    # formation_target = np.array([8,5,1])
    # obstacle = np.array([   [3,6,4,0.4,0.4,20],
    #                         [6,4,4,0.4,0.4,20],
    #                 ])

    plan_obstacle = obstacle.copy()
    plan_obstacle[:,-3:-1] += 0.1
    obs = np.empty((0,5))
    for point in plan_obstacle:
        x, y, z, dx, dy, dz = point[:6]
        # 需要保证追加的数据的形状为(1, 5)
        obs = np.append(obs, [[x, y, dx, dy, 0]], axis=0)
    # planning ###
    y_slice_size = 150#35
    # y_slice_size = 35#35
    # voxel_size = 0.15
    iteration = 100
    shortest_path_num = 3
    waypoint_distance = 1 # round(0.6/voxel_size)
    FOV_degrees = 60
    ### visible distance is 3 m
    visible_distance = 4
    boundary_distance = 15
    angle_step = 10
    # origin_map = np.full((slice_size, slice_size), 0)
    # known_map = np.full((slice_size, slice_size), 0)
    boundary_2D = get_boundary(obs,formation_center,formation_target,formation_pattern,buffer = visible_distance)
    print("boundary",boundary_2D)
    x_min,x_max,y_min,y_max = boundary_2D
    z_max = 3
    z_min = 0
    voxel_size = round((y_max-y_min)/y_slice_size,2)
    x_slice_size = int((x_max-x_min)/voxel_size)
    z_slice_size = int((z_max-z_min)/voxel_size)
    map_shape = (x_slice_size,y_slice_size)
    print("voxel size",voxel_size)
    vision_step = 3
    origin_map,known_map,start_formation_pos,end_formation_pos = known_map_establish(obs,0,map_shape,formation_center[:2],formation_target[:2],formation_pattern[:,:2],boundary_2D,show_image=False)
    print("map size",known_map.shape,z_slice_size)
    boundary_3D = boundary_2D + (z_min, z_max)
    map_shape_3D = (x_slice_size,y_slice_size,z_slice_size)
    obstacle_processor = Obstacle_Processor(map_shape_3D,voxel_size,boundary_3D,FOV_degrees,visible_distance)
    while True:
        user_input = input("Enter command (e.g., 'o 3'): ")

        if user_input.lower() == 'exit':
            break
        
        # 解析用户输入
        command_parts = user_input.split()
        
        if command_parts[0] == 'o':
            print("Occupancy map establish start")
            obstacle_processor.start()
        elif command_parts[0] == 'p':
            obstacle_processor.plot_occupancy_map_2d(float(command_parts[1]))
        elif command_parts[0] == '3d':
            obstacle_processor.plot_occupancy_grid()
        elif command_parts[0] == 'r3d':
            obstacle_processor.plot_real_occupancy_grid()
        elif command_parts[0] == 'pcd':
            obstacle_processor.plot_pcd()
        elif command_parts[0] == 'rp':
            obstacle_processor.plot_real_occupancy_map_2d(float(command_parts[1]))





            
            
        
