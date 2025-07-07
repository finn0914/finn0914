### This is used for particle swarm optimization algorithm to find the corresponding headings on the waypoints ###

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from FOV_overlapping import calculate_fov_overlap
import time
import util


def vector_to_angle(vector):
    u1, u2 = vector
    if u1 == 0:
        if u2>0:
            return 90
        else:
            return -90 

    theta_rad = np.arctan2(u2 , u1)
    # print("rad",theta_rad)

    # 将弧度值转换为角度值（度）
    theta_deg = math.degrees(theta_rad)
    # print("deg",theta_deg)

    return theta_deg

### Find information gain ###
def count_free_space(map, position, start_angle_degrees, FOV_degrees , visible_distance, angle_step, vision_step):
    # 角度步长转换为弧度
    # print("start_angle",start_angle_degrees)
    center_x, center_y = position
    start_angle_degrees = int(start_angle_degrees)
   # 初始化当前视野范围内的自由空间数量
    current_unknown_space = 0

    # 遍历当前视野范围内的每个角度
    for current_angle_degrees in range(start_angle_degrees-int(FOV_degrees/2), start_angle_degrees + int(FOV_degrees/2), angle_step):
        # 将当前角度转换为弧度
        current_angle_radians = math.radians(current_angle_degrees)
        # print("current angle",current_angle_radians,current_angle_degrees)
        # 检查当前角度对应的射线上的像素
        for dist in range(1, visible_distance + 1, vision_step):
            # print("dist",dist,current_angle_degrees,position)
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
            # 没有遇到障碍物，增加当前视野范围内的未知空间数量
                current_unknown_space += 1
        # print("current_unknown_space: " , current_unknown_space)
        
    return current_unknown_space

### find update voxels ###
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
                    # for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1), (0 ,0)]:
                    #     nx, ny = coordinate_x + dx, coordinate_y + dy
                    #     obs_coordinates.append([nx, ny])
                    # for j in range(1):
                    #     coordinate_x = center_x + round((i+j) * math.cos(current_angle_radians))
                    #     coordinate_y = center_y + round((i+j) * math.sin(current_angle_radians))
                    obs_coordinates.append([coordinate_x, coordinate_y])
                    # print("coordinate",coordinate_x,coordinate_y)
                    break
            # 添加坐标到列表中            
                if([coordinate_x, coordinate_y] not in visible_coordinates):
                    visible_coordinates.append([coordinate_x, coordinate_y])
            # print("visible", visible_coordinates)
    
    return visible_coordinates,obs_coordinates

### assistive awareness ###
def calculate_assist_reward(heading,pf,agent_pos,FOV_degree,visible_distance):
    num = len(agent_pos)
    gamma = 1000
    C = 0
    result = 0
    for i in range(len(agent_pos)):
        for j in range(len(pf)):
            if pf[j] == None:
                num-=1
                continue
            if util.euclidean_distance(agent_pos[i],pf[j])>visible_distance:
                continue
            
            vector = np.array([pf[j][0] - agent_pos[i][0], pf[j][1] - agent_pos[i][1]])
            vector_length = np.linalg.norm(vector)
            # print("vector",vector,vector_length)
            if vector_length == 0:
                C+=1
                # print("0")
                continue
            else:
                # print("1")
                vector = vector/(vector_length)
            # print("vector1",vector1)
            # if vector != 
            angle = vector_to_angle(vector)
            bounds = [heading[i]-FOV_degree/2,heading[i]+FOV_degree/2]
            for bound in bounds:
                # print("bound",bound)
                if bound>180:
                    bound -= 360
                elif bound<-180:
                    bound += 360
                
            if bounds[0]<angle<bounds[1]:
                C+=1
    if C<4:
        return result
    else:
        result = gamma * np.exp(-2/gamma*np.sqrt((C-num)**2))
    return result

def particle_swarm_optimization(agent_paths,current_heading,pf,horizon,FOV_degrees,visible_distance,angle_step,vision_step, num_particles=15, num_iterations=30, w=0.5, c1=1.5, c2=1.5, map = None):
    class Agent:
        def __init__(self, position, orientation):
            self.position = np.array(position)
            self.next_position = np.array(position)
            self.orientation = orientation

    class Particle:
        def __init__(self, num_agents, previous_position):
            self.agent_pos = np.zeros((num_agents, 2))
            self.agent_next_pos = np.zeros((num_agents, 2))
            self.position = np.random.uniform(-180, 180, num_agents)
            self.velocity = np.random.uniform(-0.1, 0.1, num_agents)
            self.previos_pos = previous_position
            self.best_position = self.position.copy()
            self.best_score = float('inf')
            
        def update_velocity(self, global_best_position, w, c1, c2):
            r1 = np.random.uniform(0, 1, len(self.position)) 
            r2 = np.random.uniform(0, 1, len(self.position))
            cognitive_velocity = c1 * r1 * (self.best_position - self.position)
            social_velocity = c2 * r2 * (global_best_position - self.position)
            self.velocity = w * self.velocity + cognitive_velocity + social_velocity

        ### update agents positions for finding the heading ###
        def update_agent(self,num, agent_pos,agent_next_pos):
            self.agent_pos[num] = agent_pos
            self.agent_next_pos[num] = agent_next_pos

        ### update particles' positions ###
        def update_position(self):
            self.position += self.velocity
            for i in range(len(self.position)):
                # print("position",self.position)
                while self.position[i]>180:
                    self.position[i] = self.position[i] - 360
                while self.position[i]<-180:
                    self.position[i] = self.position[i] + 360
                
                self.position[i]=max(self.previos_pos[i] - FOV_degrees /10, min(self.position[i], self.previos_pos[i] + FOV_degrees /10))
                
                # print("pos",pos)
            # self.position = np.clip(self.position, -180, 180)
            current_score = self.fitness()
            if current_score < self.best_score:
                self.best_score = current_score
                self.best_position = self.position.copy()

        ### calculate the cost of the active heading planner
        def fitness(self):
            # print("pos",self.position,"pre",self.previos_pos)
            # print("pos",self.agent_pos)
            time1 = time.time()
            cost1 = 0
            ##### 1. information gain #####
            for i in range(self.agent_pos.shape[0]):
                cost1 += count_free_space(map,self.agent_pos[i],self.position[i],FOV_degrees,visible_distance,angle_step,vision_step)
            time2 = time.time()
            # print("cost1",cost1)
            # print("cost1 time",time2-time1)
            # if cost1 != 0:
            #     print("cost1",cost1)
            ##### 2. heading smoothness #####
            cost2 = [self.previos_pos[i] for i in range(len(self.previos_pos))]
            for i in range(len(cost2)):
                if cost2[i]>180:
                    cost2[i]-=360
            time3 = time.time()
            # print("cost2",np.sum(np.abs(self.position-cost2)))
            # print("cost2 time",time3-time2)
            ##### 3. assistive awareness #####
            cost3 = calculate_assist_reward(self.position,pf,self.agent_pos,FOV_degrees,visible_distance)
            # angle = [vector_to_angle(np.array([self.agent_next_pos[i][0] - self.agent_pos[i][0], self.agent_next_pos[i][1] - self.agent_pos[i][1]])) for i in range(len(self.agent_pos))]
            
            time4 = time.time()
            # print("cost3: ",cost3)
            # print("cost3 time",time4-time3)
            # print("pos",self.agent_pos,"heading",self.position)
            ##### 4. overlapping penalty #####
            intersection = calculate_fov_overlap(self.agent_pos, self.position, visible_distance, FOV_degrees)
            time5 = time.time()
            # print("cost4",intersection.area)
            # print("cost4 time",time5-time4)
            # return 1*np.sum(np.abs(self.position-cost2))-10*cost1
            # return np.sum(np.abs(self.position-cost2)**2)
            cost = 5*cost1#+cost3-1*intersection.area#-2*np.sum(np.abs(self.position-cost2))

            cost = cost1+10*cost3-1*intersection-0.1*np.sum(np.abs(self.position-cost2))
            # cost = 1*cost1-1*intersection.area-5*np.sum(np.abs(self.position-angle))
            return -cost
    num_agents = len(agent_paths)
    global_best_position = current_heading  # 初始朝向角度
    print("initial heading",global_best_position)
    agent_positions = []
    for i in range(len(agent_paths)):
        agent_positions.append(agent_paths[i][0])
    agents = [Agent(position, orientation) for position, orientation in zip(agent_positions, global_best_position)]
    

    heading_path = [[global_best_position[i]] for i in range(len(agent_paths))]
    min_len = math.inf
    for i in range(len(agent_paths)):
        if len(agent_paths[i]) < min_len:
            min_len = len(agent_paths[i])
    ### planning the same number of headings according to the executing horizon ###
    for time_step in range(1,horizon+1):
        particles = [Particle(num_agents,global_best_position) for _ in range(num_particles)]
        # print("iteration:",time_step)
        global_best_score = float('inf')
        for i in range(len(agents)):
            agents[i].position = agent_paths[i][time_step]
            agents[i].next_position = agent_paths[i][time_step+1]
            
        ### PSO iteration
        for iteration in range(num_iterations):
            # print("iteration:",iteration)
            for particle in particles:
                for i in range(len(agents)):
                    particle.update_agent(i,agents[i].position,agents[i].next_position)
                particle.update_velocity(global_best_position, w, c1, c2)
                particle.update_position()
                if particle.best_score < global_best_score:
                    global_best_score = particle.best_score
                    global_best_position = particle.best_position.copy()
                    for i in range(len(global_best_position)):
                        if global_best_position[i]>180:
                            global_best_position[i] = global_best_position[i] - 360
                
            for i, agent in enumerate(agents):
                agent.orientation = global_best_position[i]
                # print("best",global_best_position[i])
            # 清除之前的箭头

            
            plot_process = False
            # 绘制代理位置和朝向
            if plot_process == True:
                fig, ax = plt.subplots()
                ax.clear()
                ax.set_xlim(0, map.shape[1])
                ax.set_ylim(0, map.shape[0])
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                for i in range(len(agents)):
                    agent = agents[i]
                    x, y = agent.position
                    dx = np.cos(np.radians(agent.orientation))  # 弧度转换为度数
                    dy = np.sin(np.radians(agent.orientation))  # 弧度转换为度数
                    ax.arrow(x, y, dx, dy, head_width=5, head_length=5, fc='b', ec='b')
                    heading_path[i].append(agent.orientation)
                ax.set_title(f"Iteration {iteration + 1}/{num_iterations}, Best Score: {global_best_score:.2f}")
                plt.imshow(map, cmap="Greys")
                plt.pause(0.001)
                # plt.show()
            
        for i in range(len(agents)):
            agent = agents[i]
            heading_path[i].append(agent.orientation)
        plot = False
        if plot == True:
            ax.clear()
            ax.set_xlim(0, map.shape[1])
            ax.set_ylim(0, map.shape[0])
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            for i in range(len(agents)):
                agent = agents[i]
                x, y = agent.position
                dx = np.cos(np.radians(agent.orientation))  # 弧度转换为度数
                dy = np.sin(np.radians(agent.orientation))  # 弧度转换为度数
                ax.arrow(x, y, dx, dy, head_width=5, head_length=5, fc='b', ec='b')
                heading_path[i].append(agent.orientation)
            plt.imshow(map)
            # plt.pause(0.001)
            plt.show()
    print("PSO end")
    # plt.show()
    # print("heading path",heading_path)
    return heading_path

# if __name__=="__main__":
#     # 使用示例
#     agent_positions = [[5, 15], [5, -5], [-5, 5], [-5, -5]]
#     initial_orientations = [np.pi/4, -np.pi/4, np.pi/4, -np.pi/4]  # 初始朝向角度
#     history = particle_swarm_optimization(agent_positions, initial_orientations)
