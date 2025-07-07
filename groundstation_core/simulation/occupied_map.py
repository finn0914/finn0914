import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.colors import ListedColormap
from concurrent.futures import ThreadPoolExecutor

def from_real_to_map_pattern(formation_pattern,boundary,slice_size):
    if len(boundary) == 4: # 2D
        x_min,x_max,y_min,y_max = boundary
        transformed = []
        for i in range(len(formation_pattern)):
            print(float(formation_pattern[i][0]) )
            transformed_x = int(float(formation_pattern[i][0]) * slice_size[0] / (x_max - x_min))
            transformed_y = int(float(formation_pattern[i][1]) * slice_size[1] / (y_max - y_min))
            transformed.append([transformed_x,transformed_y])
        print("transformed formation", transformed)
    else: # 3D
        x_min,x_max,y_min,y_max,z_min,z_max = boundary
        transformed = []
        for i in range(len(formation_pattern)):
            print(float(formation_pattern[i][0]) )
            transformed_x = int(float(formation_pattern[i][0]) * slice_size[0] / (x_max - x_min))
            transformed_y = int(float(formation_pattern[i][1]) * slice_size[1] / (y_max - y_min))
            transformed_z = int(float(formation_pattern[i][2]) * slice_size[2] / (z_max - z_min))
            transformed.append([transformed_x,transformed_y,transformed_z])
        print("transformed formation", transformed)
    return transformed

def from_real_to_map(point,min,max,slice_size):
    transformed = int((point - min) * slice_size / (max - min))
    return transformed
def from_map_to_real(transformed,min,max,slice_size):
    point = transformed*(max-min)/slice_size+min
    return point

### Find information gain ###
def count_free_space(map, position, start_angle_degrees, FOV_degrees , visible_distance, angle_step, vision_step):
    # 角度步长转换为弧度
    center_x, center_y = position
    if type(start_angle_degrees) == float:
        start_angle_degrees = int(math.degrees(start_angle_degrees))
    
    # 初始化最大自由空间数量和对应的最佳角度
    max_free_space = -1
    best_angle = 0

    # 遍历可能的起始角度范围
    for start_angle in range(start_angle_degrees, start_angle_degrees + 360, angle_step):

        # 初始化当前视野范围内的自由空间数量
        current_free_space = 0

        # 遍历当前视野范围内的每个角度
        for current_angle_degrees in range(start_angle, start_angle + FOV_degrees, angle_step):
            # 将当前角度转换为弧度
            current_angle_radians = math.radians(current_angle_degrees)

            # 检查当前角度对应的射线上的像素
            for dist in range(1, visible_distance + 1, vision_step):
                # 计算射线上的像素坐标
                pixel_x = int(center_x + dist * math.cos(current_angle_radians))
                pixel_y = int(center_y + dist * math.sin(current_angle_radians))

                # 检查是否超出地图范围
                if not (0 <= pixel_x < len(map[0]) and 0 <= pixel_y < len(map)):
                    break  # 超出地图范围，停止射线检测

                # 检查是否遇到障碍物
                if map[pixel_y][pixel_x] == 1:
                    break  # 遇到障碍物，停止射线检测
                elif map[pixel_y][pixel_x] == -1:
                # 没有遇到障碍物，增加当前视野范围内的自由空间数量
                    current_free_space += 1
        # print("current angle:",start_angle,current_free_space)
        # 更新最大自由空间数量和对应的最佳角度
        if current_free_space > max_free_space:
            max_free_space = current_free_space
            best_angle = start_angle

    return best_angle

### find update voxels ###
def calculate_visible_coordinates_3d(map, center_x, center_y, center_z, heading, FOV_degrees,visible_distance):
    # 将角度转换为弧度
    yaw_radians = math.radians(heading)
    camera_FOV_radians = math.radians(FOV_degrees) / 2

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
        while weight < visible_distance:
            # 根据视距计算坐标增量
            delta_x = int(weight * cos_pitch[current_pitch] * cos_yaw[current_yaw])
            delta_y = int(weight * cos_pitch[current_pitch] * sin_yaw[current_yaw])
            delta_z = int(weight * sin_pitch[current_pitch])

            # 计算坐标
            coordinate_x = center_x + delta_x
            coordinate_y = center_y + delta_y
            coordinate_z = center_z + delta_z

            if (-1 < coordinate_y < map.shape[0] and -1 < coordinate_x < map.shape[1] and -1 < coordinate_z < map.shape[2]):
                if map[coordinate_y][coordinate_x][coordinate_z] == True:
                    local_visible_coordinates.append([coordinate_x, coordinate_y, coordinate_z])
                    break
                if [coordinate_x, coordinate_y, coordinate_z] not in local_visible_coordinates:
                    local_visible_coordinates.append([coordinate_x, coordinate_y, coordinate_z])
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
def calculate_visible_coordinates(known_map,center_x, center_y, angle_degrees, camera_FOV, visible_distance):
    # 将角度转换为弧度
    angle_radians = math.radians(angle_degrees)
    camera_FOV_radians = math.radians(camera_FOV)/2

    # 计算视野范围内的角度边界
    min_angle = angle_radians - camera_FOV_radians
    max_angle = angle_radians + camera_FOV_radians

    # 初始化视野范围内的坐标列表
    visible_coordinates = []
    obs_coordinates = []

    # 遍历视野范围内的每个角度
    for current_angle in range(int(math.degrees(min_angle)), int(math.degrees(max_angle)) + 1):
        # 将当前角度转换为弧度
        current_angle_radians = math.radians(current_angle)
        # print("current_angle_radians", current_angle_radians)
        for i in range(1,visible_distance+1):
            # 根据视距计算坐标增量
            delta_x = int(i * math.cos(current_angle_radians))
            delta_y = int(i * math.sin(current_angle_radians))

            # 计算坐标
            # print("update voxels", center_x,center_y,delta_x,delta_y)
            coordinate_x = center_x + delta_x
            coordinate_y = center_y + delta_y
            if -1 < coordinate_y < known_map.shape[0] and -1 < coordinate_x < known_map.shape[1]:
                if int(known_map[coordinate_y][coordinate_x]) == 1:                    
                    visible_coordinates.append([coordinate_x, coordinate_y])
                    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1), (0 ,0)]:
                        nx, ny = coordinate_x + dx, coordinate_y + dy
                        obs_coordinates.append([nx, ny])
                    # for j in range(1):
                    #     coordinate_x = center_x + round((i+j) * math.cos(current_angle_radians))
                    #     coordinate_y = center_y + round((i+j) * math.sin(current_angle_radians))
                    # obs_coordinates.append([coordinate_x, coordinate_y])
                    # print("coordinate",coordinate_x,coordinate_y)
                    break
            # 添加坐标到列表中            
                if([coordinate_x, coordinate_y] not in visible_coordinates):
                    visible_coordinates.append([coordinate_x, coordinate_y])
            # print("visible", visible_coordinates)
    
    return visible_coordinates,obs_coordinates


def get_boundary(obs,start,end,formation_pattern,buffer = 10):
    
    if (obs.shape[1])==5:
        form_start = [[x + y for x, y in zip(sublist, start)] for sublist in formation_pattern]
        form_end = [[x + y for x, y in zip(sublist, end)] for sublist in formation_pattern]
        print("start",form_start)
        print("end",form_end)
        x_min = min(min(obs[:, 0] - obs[:, 2]),start[0],end[0],min(i[0] for i in form_start),min(i[0] for i in form_end)) - buffer
        x_max = max(max(obs[:, 0] + obs[:, 2]),start[0],end[0],max(i[0] for i in form_start),max(i[0] for i in form_end)) + buffer
        y_min = min(min(obs[:, 1] - obs[:, 3]),start[1],end[1],min(i[1] for i in form_start),min(i[1] for i in form_end)) - buffer
        y_max = max(max(obs[:, 1] + obs[:, 3]),start[1],end[1],max(i[1] for i in form_start),max(i[1] for i in form_end)) + buffer
        return x_min,x_max,y_min,y_max
    elif (obs.shape[1])==7:
        # Calculate axis ranges
        x_min = min(min(obs[:, 0] - obs[:, 3]), start[0], end[0]) - buffer
        x_max = max(max(obs[:, 0] + obs[:, 3]), start[0], end[0]) + buffer
        y_min = min(min(obs[:, 1] - obs[:, 4]), start[1], end[1]) - buffer
        y_max = max(max(obs[:, 1] + obs[:, 4]), start[1], end[1]) + buffer
        z_min = min(min(obs[:, 2] - obs[:, 5]), start[2], end[2]) - buffer
        z_max = max(max(obs[:, 2] + obs[:, 5]), start[2], end[2]) + buffer
        return x_min,x_max,y_min,y_max,z_min,z_max
    else:
        print('wrong obstacle dimension!')

def known_map_establish(obs,buffer,slice_size,start,end,formation_pattern,boundary, show_image = True):
    if (obs.shape[1])==5:
        # 計算橫軸和縱軸的範圍
        x_min,x_max,y_min,y_max = boundary
        x = np.linspace(x_min, x_max, slice_size[0])
        y = np.linspace(y_min, y_max, slice_size[1])
        xx, yy = np.meshgrid(x, y)

        # Transform formation_pattern
        # Add transformed pattern to initial_pos
        start_formation_pos = []
        end_formation_pos = []
        transformed_formation_pattern = []
        for point in formation_pattern:
            x_transformed = from_real_to_map((start[0]+point[0]),x_min,x_max,slice_size[0])
            y_transformed = from_real_to_map((start[1]+point[1]),y_min,y_max,slice_size[1])
            start_formation_pos.append((x_transformed, y_transformed))

            x_transformed = from_real_to_map((end[0]+point[0]),x_min,x_max,slice_size[0])
            y_transformed = from_real_to_map((end[1]+point[1]),y_min,y_max,slice_size[1])
            end_formation_pos.append((x_transformed, y_transformed))
        # for point in transformed_pattern:
        #     start_formation_pos.append((initial_pos[0] + point[0], initial_pos[1] + point[1]))
        #     end_formation_pos.append((target_pos[0] + point[0], target_pos[1] + point[1]))
        # print(start)
        # print("start",start_formation_pos,"end",end_formation_pos)
        # Set to 0 if within ellipse.
        known_map = np.zeros_like(xx, dtype=int)
        origin_map = np.zeros_like(xx, dtype=int)
        for i in range(obs.shape[0]):
            if(obs[i,4]==0):
                ellipse = ((xx - obs[i, 0]) ** 2 / obs[i, 2] ** 2 + (yy - obs[i, 1]) ** 2 / obs[i, 3] ** 2) <= 1
                origin_map[ellipse]=1
                ellipse = ((xx - obs[i, 0]) ** 2 / (obs[i, 2]+buffer) ** 2 + (yy - obs[i, 1]) ** 2 / (obs[i, 3]+buffer) ** 2) <= 1
                known_map[ellipse] = 1
            elif(obs[i,4]==1):
                x_cent, y_cent, width, height, shape = obs[i] 
                rectangle = (xx >= x_cent-width) & (xx < x_cent + width) & (yy >= y_cent-height) & (yy < y_cent + height)
                origin_map[rectangle]=1
                rectangle = (xx >= x_cent-(width+buffer)) & (xx < x_cent + (width+buffer)) & (yy >= y_cent-(height+buffer)) & (yy < y_cent + (height+buffer))
                known_map[rectangle] = 1
        if show_image:
            viridis = plt.cm.viridis
            newcolors = viridis(np.linspace(0, 1, 256))
            newcolors[:1, :] = [1, 1, 1, 1]  # 设置第一种颜色为白色 (R, G, B, Alpha)
            white_viridis = ListedColormap(newcolors)
            plt.imshow(known_map, extent=[x_min, x_max, y_min, y_max], origin = 'lower', cmap=white_viridis)
            plt.plot(end[0],end[1],marker = '^',c = 'b')
            plt.plot(start[0],start[1],marker = '^',c = 'r')
            plt.xlim([x_min-1,x_max+1])
            plt.ylim([y_min-1,y_max+1])
            # plt.axis('off')
            plt.show()
    elif (obs.shape[1])==7:
        real_occupancy_grid = np.full((slice_size[1], slice_size[0], slice_size[2]), 0, dtype=int)
        # Calculate axis ranges
        x_min,x_max,y_min,y_max,z_min,z_max = boundary
        print("obs:", obs)
        x, xstep = np.linspace(x_min, x_max, slice_size[0], retstep=True)
        y, ystep = np.linspace(y_min, y_max, slice_size[1], retstep=True)
        z, zstep = np.linspace(z_min, z_max, slice_size[2], retstep=True)
        xx, yy, zz = np.meshgrid(x, y, z)

        # Plotting voxels require 1 extra index in each direction.
        xp = np.append(x, x[slice_size[0]-1] + xstep)
        yp = np.append(y, y[slice_size[1]-1] + ystep)
        zp = np.append(z, z[slice_size[2]-1] + zstep)
        xxp, yyp, zzp = np.meshgrid(xp, yp, zp)
        
        initial_pos = [
            from_real_to_map(start[0],x_min,x_max,slice_size[0]),
            from_real_to_map(start[1],y_min,y_max,slice_size[1]),
            from_real_to_map(start[2],z_min,z_max,slice_size[2]),
        ]
        target_pos = [
            from_real_to_map(end[0],x_min,x_max,slice_size[0]),
            from_real_to_map(end[1],y_min,y_max,slice_size[1]),
            from_real_to_map(end[2],z_min,z_max,slice_size[2]),
        ]
        start_formation_pos = []
        end_formation_pos = []
        transformed_formation_pattern = []
        for point in formation_pattern:
            x_transformed = from_real_to_map((start[0]+point[0]),x_min,x_max,slice_size[0])
            y_transformed = from_real_to_map((start[1]+point[1]),y_min,y_max,slice_size[1])
            z_transformed = from_real_to_map((start[2]+point[2]),z_min,z_max,slice_size[2])
            start_formation_pos.append((x_transformed, y_transformed,z_transformed))

            x_transformed = from_real_to_map((end[0]+point[0]),x_min,x_max,slice_size[0])
            y_transformed = from_real_to_map((end[1]+point[1]),y_min,y_max,slice_size[1])
            z_transformed = from_real_to_map((end[2]+point[2]),z_min,z_max,slice_size[2])
            end_formation_pos.append((x_transformed, y_transformed,z_transformed))

        # Set to 0 if within ellipse.
        known_map = np.zeros_like(xx, dtype=bool)
        origin_map = np.zeros_like(xx, dtype=bool)
        # for i in range(obs.shape[0]):
        #     ellipse = ((xx - obs[i, 0]) ** 2 / obs[i, 3] ** 2 +
        #                (yy - obs[i, 1]) ** 2 / obs[i, 4] ** 2 +
        #                (zz - obs[i, 2]) ** 2 / obs[i, 5] ** 2) <= 1
        #     result |= ellipse
        for i in range(obs.shape[0]):
            if(obs[i,6]==0):
                print("ellipse")
                ellipse = ((xx - obs[i, 0]) ** 2 / obs[i, 3] ** 2 + 
                           (yy - obs[i, 1]) ** 2 / obs[i, 4] ** 2 +
                           (zz - obs[i, 2]) ** 2 / obs[i, 5] ** 2) <= 1
                origin_map[ellipse] = 1
                ellipse = ((xx - obs[i, 0]) ** 2 / (obs[i, 3]+buffer) ** 2 + 
                           (yy - obs[i, 1]) ** 2 / (obs[i, 4]+buffer) ** 2 +
                           (zz - obs[i, 2]) ** 2 / (obs[i, 5]+buffer) ** 2) <= 1
                known_map[ellipse] = 1
            elif(obs[i,6]==1):
                print("rectangle")
                x_cent, y_cent, z_cent, length, width, height, shape = obs[i] 
                rectangle = (xx >= x_cent-length) & (xx < x_cent + length) & \
                            (yy >= y_cent-width) & (yy < y_cent + width) & \
                            (zz >= z_cent-height) & (zz < z_cent + height)
                origin_map[rectangle] = 1
                rectangle = (xx >= x_cent-(length+buffer)) & (xx < x_cent + (length+buffer)) & \
                            (yy >= y_cent-(width+buffer)) & (yy < y_cent + (width+buffer)) &  \
                            (zz >= z_cent-(height+buffer)) & (zz < z_cent + (height+buffer))
                known_map[rectangle] = 1
            elif(obs[i,6]==2):
                print("hole")
                ellipse = ((xx - obs[i, 0]) ** 2 / obs[i, 3] ** 2 + 
                           (yy - obs[i, 1]) ** 2 / obs[i, 4] ** 2 +
                           (zz - obs[i, 2]) ** 2 / obs[i, 5] ** 2) <= 1
                origin_map[ellipse] = 1
                ellipse = ((xx - obs[i, 0]) ** 2 / (obs[i, 3]+buffer) ** 2 + 
                           (yy - obs[i, 1]) ** 2 / (obs[i, 4]+buffer) ** 2 +
                           (zz - obs[i, 2]) ** 2 / (obs[i, 5]+buffer) ** 2) <= 1
                known_map[ellipse] = 1
            
        # print(result)
        # Plot 1: Occupancy voxel map
        # print("1234")
        # show_image = False
        if show_image:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            y, x, z = np.where(known_map>0)# 创建颜色映射
            # chunk_size = 1000
            # start,end = i, min(i + chunk_size, len(x))
            # ax.scatter(x[start:end], y[start:end], z[start:end], color=colors[start:end], alpha=0.05, zorder=2)
            cmap = plt.get_cmap('coolwarm')
            norm = plt.Normalize(z.min(), z.max())
            colors = cmap(norm(z))
            ax.scatter(x, y, z, color = colors, alpha=0.5, zorder=2)
            
            ax.set_xlim(0,known_map.shape[1])
            ax.set_ylim(0,known_map.shape[0])
            # ax.set_xlim(0,self.map_size[0])
            # ax.set_ylim(0,self.map_size[1])
            ax.set_zlim(0,known_map.shape[2])
            ax.set_xticklabels([])  # 隐藏x轴标签
            ax.set_yticklabels([])  # 隐藏y轴标签
            ax.set_zticklabels([])  # 隐藏z轴标签
            # plt.imshow(known_map[:,:,round(known_map.shape[2]/2)])
        #     color_map='viridis'
        #     ax = plt.figure().add_subplot(projection='3d')
        #     y, x, z = np.where(known_map)# 创建颜色映射
        #     cmap = plt.get_cmap(color_map)
        #     norm = plt.Normalize(z.min(), z.max())
        #     colors = cmap(norm(z))

        #     ax.scatter(x, y, z, c=colors, cmap=color_map)
        #     ax.set_title('Occupancy Grid')
        #     ax.set_xlabel('X')
        #     ax.set_ylabel('Y')
        #     ax.set_zlabel('Z')
        #     # plt.show()
        #     print("showing voxel map")
            plt.show()
        # print(known_map.shape)
        # print(list(initial_pos),list(target_pos))
        return origin_map,known_map,initial_pos,target_pos,start_formation_pos,end_formation_pos
    return origin_map,known_map,start_formation_pos,end_formation_pos

# def known_map_establish(map_size,start,end,formation_pattern,obs,buffer):
    
#     # map generation
#     known_map = np.zeros((map_size, map_size))
#     origin_map = np.zeros((map_size, map_size))


#     # 定義障礙物的位置和大小

#     # 在地圖上填充障礙物，occupied為1
#     xx, yy = np.meshgrid(np.arange(map_size), np.arange(map_size))
#     for i in range(obs.shape[0]):
#         if(obs[i,4]==0):
#             ellipse = ((xx - obs[i, 0]) ** 2 / obs[i, 2] ** 2 + (yy - obs[i, 1]) ** 2 / obs[i, 3] ** 2) <= 1
#             origin_map[ellipse]=1
#             ellipse = ((xx - obs[i, 0]) ** 2 / (obs[i, 2]+buffer) ** 2 + (yy - obs[i, 1]) ** 2 / (obs[i, 3]+buffer) ** 2) <= 1
#             known_map[ellipse] = 1
#         elif(obs[i,4]==1):
#             x_cent, y_cent, width, height, shape = obs[i] 
#             rectangle = (xx >= x_cent-width) & (xx < x_cent + width) & (yy >= y_cent-height) & (yy < y_cent + height)
#             origin_map[rectangle]=1
#             rectangle = (xx >= x_cent-(width+buffer)) & (xx < x_cent + (width+buffer)) & (yy >= y_cent-(height+buffer)) & (yy < y_cent + (height+buffer))
#             known_map[rectangle] = 1
#         # x, y = obstacle["x"], obstacle["y"]
#         # width, height = obstacle["width"], obstacle["height"]
#         # known_map[y:y+height, x:x+width] = 1
    
#     ################################ see the original map #################################
#     # plt.imshow(origin_map, interpolation='nearest')
#     # plt.colorbar()  # 添加颜色条
#     # plt.show()

#     # 初始化使用者地圖
#     user_map = np.full((map_size, map_size), -1)

#     # 定義使用者的位置
#     return origin_map,known_map

# 更新使用者地圖，保留已知的地圖資訊
def update_user_map(user_map, obstacle_list, known_map, user_position, visible_distance, FOV_angle):
    # print("user pos", user_position)
    visible_distance = visible_distance
    pos, heading = user_position
    if len(pos) == 3:
        x, y, z = pos
        update_positions,obs_positions = calculate_visible_coordinates_3d(known_map,x,y,z,heading,FOV_angle,visible_distance)
        for pos in update_positions:
            if -1 < pos[1] < known_map.shape[0] and -1 < pos[0] < known_map.shape[1]:
                user_map[pos[1], pos[0], pos[2]] = known_map[pos[1], pos[0],pos[2]]
    else:
        x, y = pos
        # print("visible_distance",visible_distance)
        update_positions,obs_positions = calculate_visible_coordinates(known_map,x,y,heading,FOV_angle,visible_distance)
        for pos in update_positions:
            if -1 < pos[1] < known_map.shape[0] and -1 < pos[0] < known_map.shape[1]:
                user_map[pos[1], pos[0]] = known_map[pos[1], pos[0]]
            
        # for pos in obs_positions:
        #     if -1 < pos[1] < known_map.shape[0] and -1 < pos[0] < known_map.shape[1] and pos not in obstacle_list:
        #         obstacle_list.append(pos)
        #         user_map[pos[1]][pos[0]] = 1
    return user_map, obstacle_list

# if __name__ == '__main__':
#     map_size = 50
#     formaiton_distance = 5
#     x = map_size/2
#     y = formaiton_distance
#     start = [50,5]
#     end = [50,map_size-1-5]
#     formation_pattern = [[5,5],
#                         [-5,5],
#                         [-5,-5],
#                         [5,-5]
#                         ]
#     # map_size = 20
#     # formaiton_distance = 5
#     # x = map_size/2
#     # y = formaiton_distance
#     # start = [10,5]
#     # end = [10,map_size-1-5]
#     # formation_pattern = [[5,5],
#     #                     [-5,5],
#     #                     [-5,-5],
#     #                     [5,-5]
#     #                     ]
    
#     obs = np.array([
#         # [50,50, 10, 10,0]
#         #h-shape
#         # [50,50,25,1,1],
#         # [25,50,5,25,1],
#         # [75,50,5,25,1],

#         # tunnel
#         [0,50,43,40,0],
#         [99,50,43,40,0],

#         #middle obstacles
#         # [50,50,20,30,0], 
#                     ])
#     # map safety extention
#     buffer=0
#     visible_step = 2
#     visible_distance = visible_step*5
#     FOV_angle = 90
#     origin_map,known_map = known_map_establish(map_size,start,end,formation_pattern,obs,buffer)
#     map = np.full((map_size, map_size), -1)
#     obstacle_list = []
#     for i in range(0,200,3):
#         user_position = [[50,i],90]
#         fake_map = np.full((map_size, map_size), -1)
#         max_perception_angle = count_free_space(map, user_position[0] , 90, 90, visible_distance,10,visible_step)
#         print("max",max_perception_angle)
#         user_position[1] = max_perception_angle
#         map,obstacle_list = update_user_map(map,obstacle_list, known_map,user_position,visible_distance,FOV_angle)
#         fake_map,_ = update_user_map(fake_map,obstacle_list, known_map,user_position,visible_distance,FOV_angle)
#         x_values = [obs[0] for obs in obstacle_list]
#         y_values = [obs[1] for obs in obstacle_list]
#         plt.plot(x_values,y_values,marker = '^',c = 'r')
#         plt.show()
#         # x_values = user_position[0][0]
#         # y_values = user_position[0][1]
#         # plt.plot(x_values,y_values,marker = '^',c = 'r',linestyle='-')
#         # plt.imshow(fake_map, interpolation='nearest')
#         # plt.colorbar()  # 添加颜色条
#         # plt.show()
#         # plt.imshow(map, interpolation='nearest')
#         # plt.colorbar()  # 添加颜色条
#         # plt.show()