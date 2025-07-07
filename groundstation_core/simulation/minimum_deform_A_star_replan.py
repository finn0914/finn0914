import heapq
import math
import matplotlib.pyplot as plt
import numpy as np
from occupied_map import known_map_establish,from_map_to_real,from_real_to_map,get_boundary,from_real_to_map_pattern,count_free_space,update_user_map
from PSO import particle_swarm_optimization
from collections import deque
import time
from scipy.interpolate import make_interp_spline, BSpline, interp1d
import util


def vector_to_angle(vector):
    u1, u2 = vector
    # theta = arctan(u2 / u1)，make sure u1!=0
    if u1 == 0:
        if u2>0:
            return 90
        else:
            return -90 

    theta_rad = np.arctan2(u2 , u1)

    # rad to degree
    theta_deg = math.degrees(theta_rad)

    return theta_deg

def angle_to_vector(angle_degrees):
    angle_radians = math.radians(angle_degrees)
    
    vector_x = np.cos(angle_radians)
    vector_y = np.sin(angle_radians)
    
    vector = np.array([vector_x, vector_y])
    
    return vector


def generate_obstacles(x_min, x_max, y_min, y_max, num_obstacles, seed=None):
    """
    Generate specified number of obstacles randomly within the specified x and y limits,
    using a specified seed for reproducibility.

    Args:
    - x_min (float): Minimum x-coordinate value.
    - x_max (float): Maximum x-coordinate value.
    - y_min (float): Minimum y-coordinate value.
    - y_max (float): Maximum y-coordinate value.
    - num_obstacles (int): Number of obstacles to generate.
    - seed (int or None): Seed for random number generator. If None, no seed is set.

    Returns:
    - numpy array: Array of obstacles with format [x, y, z, width, height, depth].
    """
    if seed is not None:
        np.random.seed(seed)  # Set seed for reproducibility
    
    obstacles = []
    for _ in range(num_obstacles):
        x = np.random.uniform(x_min, x_max)
        y = np.random.uniform(y_min, y_max)
        z = 8  # Fixed z-coordinate
        width = 0.2
        height = 0.2
        depth = 20  # Fixed depth
        
        obstacles.append([x, y, z, width, height, depth])
    
    return np.array(obstacles)

def find_team_center(pos_list):
    ### take average
    center = np.int(np.mean(pos_list, axis=0)).astype(int)
    return center

def separate_agent_paths(path):
    num_agents = len(path[1])  # Get the number of agents
    agent_paths = [[] for _ in range(num_agents)]  # Create empty lists for each agent

    for step in path:
        for agent_index, agent_position in enumerate(step):
            agent_paths[agent_index].append(agent_position)
    # for i in range(num_agents):
        # print("agent path",agent_paths[i])
    return agent_paths

def plot_path(agent_paths,map):
    plt.imshow(map)
    colors = ['r', 'g', 'b', 'c','m', 'w', 'k','r', 'g', 'b', 'c','m', 'w', 'k']
    for i, agent_path in enumerate(agent_paths):
        if i%2==1:
            continue
        x_values = [pos[0] for pos in agent_path]
        y_values = [pos[1] for pos in agent_path]
        plt.plot(x_values, y_values, label=f"Agent {i}",marker = "o", markersize = 3, color = colors[i])

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Agent Paths")
    plt.legend()
    # plt.grid(True)
    # plt.show()
    plt.pause(0.001)

def plot_obstacle_clusters(map,obstacle_clusters):
    for cluster in obstacle_clusters:
        cluster = list(zip(*cluster))  # Transpose the cluster points
        plt.plot(cluster[0], cluster[1], 'ro')
    plt.gca().invert_yaxis()
    plt.imshow(map)
    plt.show()


def plot_agent_paths_with_heading(execute,known_map):
    colors = ['r', 'g', 'b', 'm']  # Define colors for each agent

    for i, agent_data in enumerate(execute):
        x_values = [pos[0] for pos, _ in agent_data]
        y_values = [pos[1] for pos, _ in agent_data]
        headings = [heading for _, heading in agent_data]

        # Plot the path
        plt.plot(x_values, y_values, label=f"Agent {i}", color=colors[i % len(colors)])

        # Plot the heading direction at each point
        for (x, y), heading in agent_data:
            dx = np.cos(np.radians(heading))
            dy = np.sin(np.radians(heading))
            plt.arrow(x, y, dx, dy, head_width=1, head_length=1, fc=colors[i % len(colors)], ec=colors[i % len(colors)])

    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Agent Paths with Headings")
    plt.legend()
    # plt.grid(True)
    plt.imshow(known_map)
    plt.show()


class Node:
    def __init__(self, position, formation_pattern, parent=None):
        self.position = tuple(position)
        self.parent = parent
        self.g = 0
        self.h = 0
        self.FD = 0
        self.f = 0
        self.path = [[position[0]+formation_pattern[i][0], position[1]+formation_pattern[i][1]] for i in range(len(formation_pattern))]

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

    def __lt__(self, other):
        return self.f < other.f
    

def find_obstacle_clusters(map_grid,min_boundary_distance):
    rows, cols = len(map_grid), len(map_grid[0])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1),(-2, 0), (2, 0), (0, -2), (0, 2)]
    boundary = [math.inf,0,math.inf,0]
    def is_valid(x, y):
        return 0 <= x < rows and 0 <= y < cols

    def flood_fill(start):
        cluster = []
        min_y, max_y = start[1], start[1]  # Initialize min and max y coordinates
        queue = deque([start])
        visited = set()
        visited.add(tuple(start))
        
        while queue:
            x, y = queue.popleft()
            cluster.append([x, y])
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                
                if is_valid(nx, ny) and (nx, ny) not in visited and map_grid[ny][nx] == 1:
                    queue.append((nx, ny))
                    visited.add((nx, ny))
        
        return cluster, max_y - min_y + 1

    obstacle_clusters = []
    for y in range(rows):
        for x in range(cols):
            if map_grid[y][x] == 1:
                if x<boundary[0]:
                    boundary[0] = x-1000
                elif x>boundary[1]:
                    boundary[1] = x+1000
                if y<boundary[2]:
                    boundary[2] = y-1000
                elif y>boundary[3]:
                    boundary[3] = y+1000
                if ((x, y) not in cluster for cluster in obstacle_clusters):
                    cluster, boundary_size = flood_fill((x, y))
                    # print("boundary_size",boundary_size,min_boundary_distance)
                    if boundary_size >= min_boundary_distance:
                        obstacle_clusters.append(cluster)
    return obstacle_clusters,boundary


def find_closest_free_space(map_grid, start_point):
    rows, cols = len(map_grid), len(map_grid[0])
    # print("rows,cols",rows,cols)
    directions = [(1, 0), (0, -1), (-1, 0), (0, 1)]
    
    def is_valid(x, y):
        return 0 <= x < cols and 0 <= y < rows

    def bfs(start):
        # print(start)
        queue = deque([start])
        visited = set()
        visited.add(tuple(start))  # Convert start_point to tuple
        i = 0
        while queue:
            # print("iteration",i,"queue",queue)
            x, y = queue.popleft()
            # print("map",map_grid[y][x],x,y)
            # Check if the current cell is a free space
            if map_grid[y][x] == 0:
                return (x, y)
            
            # Explore neighbors
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                # print("aa",nx,ny,is_valid(nx,ny),(nx, ny) not in visited)
                if is_valid(nx, ny) and (nx, ny) not in visited:
                    # print("bb")
                    queue.append((nx, ny))
                    visited.add(tuple((nx, ny)))  # Convert (nx, ny) to tuple
            i+=1
        return None  # If no free space is found

    return bfs(start_point)

#############################Connect two point through line###################################
def bresenham_line(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    steep = dy > dx
    if steep:
        x0, y0 = y0, x0
        x1, y1 = y1, x1
        dx, dy = dy, dx
    if x0 > x1:
        x0, x1 = x1, x0
        y0, y1 = y1, y0
    error = dx / 2
    ystep = 1 if y0 < y1 else -1
    y = y0
    coords = []
    for x in range(x0, x1 + 1):
        coords.append((y, x) if steep else (x, y))
        error -= dy
        if error < 0:
            y += ystep
            error += dx
    return np.array(coords, dtype=int)

################################check visible between two points########################################
def checkVisible(known_map,p1,p2):
    x1,y1 = p1
    x2,y2 = p2
    i = []
    coor = bresenham_line(x1,y1,x2,y2)
    for i in range(len(coor)):
        if(known_map[coor[i][1]][coor[i][0]]==1):
            return False,coor,coor[i] 
    return True,coor,i

def interpolate_and_adjust(paths_list):
    """
    將多個航路點列表進行線性插值並調整為最小長度的函數
    
    Args:
    - paths_list (list of list of tuples): 包含多個航路點列表的列表
    
    Returns:
    - list of list of tuples: 插值並調整後的航路點列表
    """
    # 找到最短的航路點列表的長度
    min_length = min(len(path) for path in paths_list)
    
    # 進行線性插值，並截斷或填充到最短列表的長度
    interpolated_and_adjusted_paths = []
    for path in paths_list:
        x = np.linspace(0, 1, num=len(path))
        f = interp1d(x, path, axis=0, kind='linear')
        
        x_new = np.linspace(0, 1, num=min_length)
        interpolated_path = f(x_new)
        interpolated_and_adjusted_paths.append(interpolated_path.astype(int))  # 將插值後的結果轉換為整數
    
    return interpolated_and_adjusted_paths

def astar(start, end, formation_pattern, grid):
    start_node = Node(start,formation_pattern)
    end_node = Node(end,formation_pattern)
     # 初始化開放列表和閉合列表
    open_list = []
    closed_list = set()
    heapq.heappush(open_list, start_node)
    time_start = time.time()
    while open_list:

        # 從開放列表中取出F值最小的節點
        current_node = heapq.heappop(open_list)

        # 將該節點添加到閉合列表中
        closed_list.add(current_node)

        # 如果當前節點是目標節點，則找到了最短路徑
        time_end = time.time()
        if current_node == end_node:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            # print("append",path)
            return path[::-1]
        elif (time_end-time_start)>0.005:
            return []

        # 搜尋當前節點的鄰居節點
        # print("map size",grid.shape)
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            # print("current position:",current_node.position)
            # 取得鄰居節點的座標
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # 確保鄰居節點在地圖範圍內
            if node_position[0] < 0 or node_position[1] < 0 or node_position[0] >= grid.shape[1] or node_position[1] >= grid.shape[0]:
                continue
            
            # 確保鄰居節點是可以通行的
            if grid[node_position[1]][node_position[0]] == 1:
                continue

            # 創建鄰居節點
            neighbor = Node(node_position, formation_pattern, current_node)

            # 確保鄰居節點不在閉合列表中
            if neighbor in closed_list:
                continue

            # 計算鄰居節點的G值、H值和F值
            neighbor.g = current_node.g + 1
            neighbor.h = abs(neighbor.position[0] - end_node.position[0])**2 + abs(neighbor.position[1] - end_node.position[1])**2
                            
            neighbor.f = neighbor.g + neighbor.h
            # 如果鄰居節點已經在開放列表中且有更好的路徑，則更新它的父節點和路徑成本
            for open_node in open_list:
                if neighbor == open_node:
                    if neighbor.g < open_node.g:
                        open_list.remove(open_node)
                        heapq.heappush(open_list, neighbor)
                    break
            else:
                # 將鄰居節點添加到開放列表中
                heapq.heappush(open_list, neighbor)

    # 如果開放列表為空，則找不到路徑
    print("nono")
    return []

def minimum_deform_astar_search(grid, plan_map, start, end, formation_pattern, planning_rate):
    # 初始化起點和目標節點
    start_node = Node(start,formation_pattern)
    end_node = Node(end,formation_pattern)
    # print("start",start)
    # 初始化開放列表和閉合列表
    open_list = []
    closed_list = set()

    # 將起點添加到開放列表
    heapq.heappush(open_list, start_node)
    # print("open_list",open_list)

    # 開始搜索
    i = 0
    if planning_rate<1:
        rigidity = planning_rate
    else:
        rigidity = 1
    # print("rigidity",rigidity)
    time_start = time.time()
    while open_list:
        i+=1
        # print(i, boundary)

        # 從開放列表中取出F值最小的節點
        current_node = heapq.heappop(open_list)

        # 將該節點添加到閉合列表中
        closed_list.add(current_node)

        # 如果當前節點是目標節點，則找到了最短路徑

        time_current =  time.time()
        if current_node == end_node:
            center_path = []
            path = []
            while current_node:
                center_path.append(current_node.position)
                path.append(current_node.path)
                current_node = current_node.parent
            # print("path",path[::-1])
            return center_path[::-1],path[::-1]

        # 搜尋當前節點的鄰居節點
        # print("map size",grid.shape)
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            # print("current position:",current_node.position)
            # 取得鄰居節點的座標
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # 確保鄰居節點在地圖範圍內
            if node_position[0] < 0 or node_position[1] < 0 or node_position[0] >= grid.shape[1] or node_position[1] >= grid.shape[0]:
                continue
            
            # 確保鄰居節點是可以通行的
            if plan_map[node_position[1]][node_position[0]] == 1:
                continue

            # 創建鄰居節點
            neighbor = Node(node_position, formation_pattern, current_node)

            # 確保鄰居節點不在閉合列表中
            if neighbor in closed_list:
                continue

            # 計算鄰居節點的G值、H值和F值
            neighbor.g = current_node.g + 1
            neighbor.h = math.sqrt(abs(neighbor.position[0] - end_node.position[0])**2 + abs(neighbor.position[1] - end_node.position[1])**2)
            
            neighbor.path  = []
            current_agent_pos = current_node.path
            # print("currrent position",current_agent_pos,new_position)
            cost = 0
            for i in range(len(formation_pattern)):
                x = neighbor.position[0]+formation_pattern[i][0]
                y = neighbor.position[1]+formation_pattern[i][1]
                # x = current_agent_pos[i][0]+new_position[0]
                # y = current_agent_pos[i][1]+new_position[1]
                if grid[y][x] == 1:
                    pos = find_closest_free_space(grid,[x,y])
                    # print("pos",pos,x,y)
                    neighbor.path.append(pos)
                    cost += util.euclidean_distance(pos,[x,y])
                else:
                    neighbor.path.append([x,y])
            neighbor.FD = current_node.FD + cost
            neighbor.f = neighbor.g + neighbor.h/rigidity + cost*100
            # 如果鄰居節點已經在開放列表中且有更好的路徑，則更新它的父節點和路徑成本
            for open_node in open_list:
                if neighbor == open_node:
                    if neighbor.FD/neighbor.g < open_node.FD/neighbor.g:
                        open_list.remove(open_node)
                        heapq.heappush(open_list, neighbor)
                    break
            # for open_node in open_list:
            #     if neighbor == open_node and neighbor.g > open_node.g:
            #         break
            else:
                # 將鄰居節點添加到開放列表中
                heapq.heappush(open_list, neighbor)

    # 如果開放列表為空，則找不到路徑
    return [],[]
def minimum_deform_search(known_map,start_formation_pos,end_formation_pos,heading,map_formation_pattern,waypoint_distance,FOV_angle,angle_step,visible_distance,vision_step):
    colors = ['r', 'g', 'b', 'c','m', 'w', 'k','r', 'g', 'b', 'c','m', 'w', 'k']
    file_path = './graph/'
    
    Dynamic_obstacle = False
    Heading_control = True
    ### if the occupancy map is provided, this Known_map should be true
    Known_map = True
    Center_passing = True
    execute_horizon = 2
    
    
    start_pos_heading = []
    obstacle_list = []
    user_map = np.full((known_map.shape), -1)
    if Known_map == True:
        user_map = known_map
    agent_start_pos = [[] for i in range(len(map_formation_pattern))]
    for i in range(len(map_formation_pattern)):
        agent_start_pos[i] = [start_formation_pos[0]+map_formation_pattern[i][0],start_formation_pos[1]+map_formation_pattern[i][1]]
        start_pos_heading.append([agent_start_pos[i],heading[i]])
    execute_path = [[point] for point in start_pos_heading]   
    execute_center_path = []
    memory_center_pos = []
    # plot_obstacle_clusters(known_map,obstacle_clusters)
    
    print("map size",known_map.shape)
    # plt.imshow(user_map)
    # plt.show()
    # plt.imshow(known_map)
    # plt.show()
    finish = [False for i in range(len(map_formation_pattern))]
    safety = [True for i in range(len(map_formation_pattern))]
    timer = 0
    planning_rate = 1
    planning_path = []
    dynamic_obstacle = []
    n = 0
    start_time= time.time()

    if Dynamic_obstacle:
        if -1<100-2*timer<known_map.shape[1]:
            obs_center = [150,100-timer]
        else:
            obs_center = [99,50]
                
        for pos in dynamic_obstacle:
            known_map[pos[1]][pos[0]] = 0
        dynamic_obstacle = []
        stack = [obs_center]
        visited = set()
        
        while stack:
            x, y = stack.pop()
            visited.add((x, y))
            # 檢查當前點是否為 free space 或 unknown space
            # print("x,y",x,y)
            if (known_map[y][x] == 0 or known_map[y][x] == -1) and (x,y) not in dynamic_obstacle:
                dynamic_obstacle.append([x,y])
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < len(known_map[0]) and 0 <= ny < len(known_map):
                    if (nx,ny) not in visited and util.euclidean_distance((nx,ny),obs_center)<=5:
                        stack.append((nx, ny))
        for pos in dynamic_obstacle:
            known_map[pos[1]][pos[0]] = 1
        # plt.imshow(known_map)
        # plt.pause(0.1)

    # for i in range(len(map_formation_pattern)):
    #     user_map, obstacle_list = update_user_map(user_map,obstacle_list,known_map,execute_path[i][-1],visible_distance,FOV_angle)
    print("time",timer)
    if timer!=0:
        agent_start_pos = []  
        for i in range(len(map_formation_pattern)):
            agent_start_pos.append(execute_path[i][-1][0])
    else:
        memory_center_pos = start_formation_pos     
    time1 = time.time()
    # print("center",memory_center_pos)

    if Center_passing:
        plan_map = np.zeros((known_map.shape))
    else:
        plan_map = user_map
    # for cluster in (obstacle_clusters):
    #     print("cluster",cluster)
    #     for pos in cluster:
    #         plan_map[pos[1]][pos[0]]=1
    # plt.imshow(plan_map)
    # plt.title('plan map')
    # plt.show()

    ### formation path planning 
    center_path,path = minimum_deform_astar_search(user_map, plan_map, memory_center_pos, end_formation_pos, map_formation_pattern, planning_rate)
    time2 = time.time()
    planning_rate = 1/(time2-time1)
    # print("planning Hz", planning_rate)
    # print("path",path)
    if len(path)>1:
        agent_paths = separate_agent_paths(path)
        # print("agent paths",agent_paths)
        # plot_path(agent_paths,user_map)
        refine_path = []
        planning_path = []
        
        for i in range(len(agent_paths)):
            temp_path = []
            temp_path.append(agent_start_pos[i])
            # print("start",agent_start_pos[i],agent_paths[i][0])
            # temp_path.append(agent_paths[i][0])
            for index in range(0, len(agent_paths[i])):
                # print("index",index,temp_path[-1], agent_paths[i][index])
                if util.euclidean_distance(temp_path[-1],agent_paths[i][index])>waypoint_distance:
                    # print("warn")
                    # single_topo = single_topo_path(user_map, agent_paths[i][index - 1],agent_paths[i][index],visible_distance,waypoint_distance,100,1)
                    # print("single_topo_path",single_topo)
                    traditional_astar_path = astar(temp_path[-1], agent_paths[i][index],map_formation_pattern, user_map)
                    if len(traditional_astar_path)<1:
                        print("some problem exists")
                        continue
                    for pos in range(1,len(traditional_astar_path)):
                        if pos%waypoint_distance*2==0:
                            temp_path.append(traditional_astar_path[pos])
                    # RRT_path = build_rrt(user_map,temp_path[-1],agent_paths[i][index])
                    # for pos in RRT_path:
                    #     temp_path.append(pos)
                else:
                    # print("normal")
                    temp_path.append(agent_paths[i][index])

            refine_path.append(temp_path)

            temp = []
            waypoint_count = 0
            # print("agent:", i, "", refine_path[i])
            while waypoint_count < len(refine_path[i]):
                if waypoint_count % waypoint_distance == 0:
                    temp.append(refine_path[i][waypoint_count])
                waypoint_count += 1
            planning_path.append(temp)
        time3 = time.time()
        # print("refine time",time3-time2)
    heading_path = [[] for i in range(len(map_formation_pattern))]
    pf = []
    ### find out the intersection point of each agent
    for i in range(len(map_formation_pattern)):
        if len(planning_path[i])==0:
            continue
        # print(len(planning_path[i]))
        for time_step in range(len(planning_path[i])):
            # print("pos",i,planning_path[i])
            if user_map[planning_path[i][time_step][1]][planning_path[i][time_step][0]] == -1:
                pf.append(planning_path[i][time_step])
                break
            elif time_step==len(planning_path[i])-1:
                pf.append(None)
    print("pf",pf)
    
    ########################### Acitve Heading Planner #################################
    # min_len = math.inf
    # for i in range(len(map_formation_pattern)):
    #     if len(planning_path[i]) < min_len:
    #         min_len = len(planning_path[i])

    ### With heading control
    if Heading_control == True:
        #pso
        time_PSO_start = time.time()
        print("memory heading",heading,"visible distance",visible_distance)
        heading_path = particle_swarm_optimization(planning_path,heading,pf,execute_horizon,FOV_angle,visible_distance,angle_step,vision_step, map = user_map)
        time_PSO_end = time.time()
        print("heading path",heading_path)
        print("PSO execute time",(time_PSO_end-time_PSO_start))
    else:
        for i in range(len(map_formation_pattern)):
            if len(planning_path[i])==0:
                continue
            for time_step in range(len(planning_path[i])):
                # print("time_step",time_step)
                if time_step+1<len(planning_path[i]):
                    # print("1")
                    # print("planning",planning_path[i][time_step+1])
                    vector = np.array([planning_path[i][time_step+1][0] - planning_path[i][time_step][0], planning_path[i][time_step+1][1] - planning_path[i][time_step][1]])
                    vector_length = np.linalg.norm(vector)
                    if vector_length == 0:
                        if time_step > 0:
                            heading_path[i].append(heading_path[i][time_step-1])
                            continue
                        else:
                            heading_path[i].append(0)
                            continue
                    else:
                        vector= vector/vector_length
                    heading_path[i].append(vector_to_angle(vector))
                else:
                    heading_path[i].append(heading_path[i][time_step-1])
                    continue
                # print(heading_path[i])
    # for i in range(len(map_formation_pattern)):
    #     print("ohoh", planning_path[i][0] , execute_path[i][-1][0], agent_start_pos[i])
    #     if planning_path[i][0] != execute_path[i][-1][0]:
    #         print("nonono")
    # print("planning path",planning_path)
    # print("state",heading_path,memory_heading)
    end_time = time.time()
    print("System planning time:", (end_time-start_time))
    for execute_time in range(1,execute_horizon+1):
    # for execute_time in range(1,len(planning_path[0])):
        # print("execute time",execute_time)
        print("finish",finish,len(center_path))

        #### Terminal condition ####
        if any(finish) == True or len(center_path)<waypoint_distance*1.5:
            for i in range(len(map_formation_pattern)):
                print("num",i)
                if finish[i] == True:
                    continue
                else:
                    planning_path[i] = planning_path[i][1:]
            execute_time = execute_horizon
        
        
        #     for step in range(len(planning_path[i])):
        #         if user_map[planning_path[i][step][1]][planning_path[i][step][0]]==1:
        #             safety[i] = False
                    
        # print("safety",safety)
        # if not all(safety):
        #     safety = [True for i in range(len(map_formation_pattern))]
        #     break

        ### map IS THE CURRENT SEEING ENVIRONMENT
        map = np.full((known_map.shape), -1)
        update_formation_position = [planning_path[i][execute_time] for i in range(len(map_formation_pattern))]
        if len(center_path)>execute_horizon:
            memory_center_pos = center_path[execute_time]
            # memory_center_pos = find_team_center(update_formation_position)
        # print("center path",memory_center_pos,center_path[execute_time])
        # print("update_formation",update_formation_position)
        for i in range(len(map_formation_pattern)):
            # execute_path[i].append([update_formation_position[i], heading_path[i][execute_time]])
            # coor = bresenham_line(update_formation_position[i][0],update_formation_position[i][1],agent_start_pos[i][0],agent_start_pos[i][1])
            # for point in coor:
            #     if user_map[point[1]][point[0]]!=0:
            #         print(execute_path[i][-1][0])
            #         update_formation_position[i] = agent_start_pos[i]
            #         break
            # execute_check,coor_idx,coor = checkVisible(user_map,update_formation_position[i], execute_path[i][-1][0])

            #### This is the dangerous condition, only prevent fail when planning 
            if user_map[update_formation_position[i][1]][update_formation_position[i][0]]!=0:# or not execute_check:
                update_formation_position[i] = agent_start_pos[i]
                if not None in pf:
                # vec = np.array([update_formation_position[i][0] - agent_start_pos[i][0] , update_formation_position[i][1]-agent_start_pos[i][1]])
                    vec = np.array([pf[i][0] - agent_start_pos[i][0], pf[i][1] - agent_start_pos[i][1]])
                    ang = vector_to_angle(vec)
                    heading_path[i][execute_time] = ang
            
            # print("1",execute_path,"2",update_formation_position,"3",heading_path)
            print("heading path",heading_path)
            execute_path[i].append([update_formation_position[i], heading_path[i][execute_time]]) ### heading_path[i][execute_time] for with heading control, 90 for non heading control
            map,_ = update_user_map(map,obstacle_list,known_map,execute_path[i][-1],visible_distance,FOV_angle)
            user_map,_ = update_user_map(user_map,obstacle_list,known_map,execute_path[i][-1],visible_distance,FOV_angle)

            ### terminal condition check for each agent
            if util.euclidean_distance(list(execute_path[i][-1][0]),list((end_formation_pos[0]+map_formation_pattern[i][0],end_formation_pos[1]+map_formation_pattern[i][1])))>(waypoint_distance)*1.5:
                finish[i] = False
            else:
                finish[i] = True
            # print("update:",execute_path[i][-1][0])
            # print("end:",end_pos[i])
        execute_center_path.append(memory_center_pos)
        # print("finish:",finish)
        # print("exe",execute_path)
        # print("plan",planning_path)
        ###############plot the map for single timestep###################
        plt.clf()
        ###@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  Plot the planning horizen when planning
        # # for i in range(len(map_formation_pattern)):
        # #     if pf[i] != None:
        # #         plt.plot(pf[i][0],pf[i][1],color = colors[i],marker = "^")
        # #     x,y = execute_path[i][-1][0]
        # #     heading = execute_path[i][-1][1]
        # #     dx = np.cos(np.radians(heading))
        # #     dy = np.sin(np.radians(heading))
        # #     plt.arrow(x, y, dx, dy, head_width=5, head_length=1, fc=colors[i % len(colors)], ec=colors[i % len(colors)])
        # # plt.imshow(map, cmap = "Greys", interpolation='nearest')
        # # filename = file_path + f'{n}_single_frame.png'
        # # plt.savefig(filename)
        # # plt.show()
        

        # # ### Plotting Each Step ###
        # n+=1
        # for i in range(len(map_formation_pattern)-1,-1,-1):
        #     for j in range(1,len(planning_path[i]) - 1):  # 循环遍历每个点，并绘制点之间的连线
        #         point1 = planning_path[i][j]  # 当前点
        #         point2 = planning_path[i][j + 1]  # 下一个点

        #         # 提取点的坐标
        #         x_values = [point1[0], point2[0]]
        #         y_values = [point1[1], point2[1]]
                
        #         # 绘制点之间的连线
        #         # if i == 0 or i == 2:
        #         #     plt.plot(x_values, y_values, c=colors[i],marker = "o", linestyle='-')
        #         # else:
        #         plt.plot(x_values, y_values, c=colors[i], linestyle='-')#,marker = 'o',markersize = 3)
        #     # 绘制中心路径
        #     for j in range(1,len(center_path) - 1):
        #         point1 = center_path[j]
        #         point2 = center_path[j + 1]

        #         # 提取点的坐标
        #         x_values = [point1[0], point2[0]]
        #         y_values = [point1[1], point2[1]]

        #         # 绘制点之间的连线
        #         plt.plot(x_values, y_values, c='w', linestyle='-')#,marker = 'o',markersize = 3)
        #     # x,y = execute_path[i][-1][0]
        #     # # print("x,y",x,y)
        #     # heading = execute_path[i][-1][1]
        #     # dx = np.cos(np.radians(heading))
        #     # dy = np.sin(np.radians(heading))
        #     # plt.arrow(x, y, dx, dy, head_width=5, head_length=1, fc=colors[i % len(colors)], ec=colors[i % len(colors)])
        #     # for i in range(len(update_formation_position)):
        #     plot_list = []
        #     angle = execute_path[i][-1][1]
        #     vector = angle_to_vector(angle)
        #     dx,dy = update_formation_position[i]+vector
        #     pend_point1 = [update_formation_position[i][0]+vector[1],update_formation_position[i][1]-vector[0]]
        #     pend_point2 = [update_formation_position[i][0]-vector[1],update_formation_position[i][1]+vector[0]]
        #     plot_list.append([dx,dy])
        #     plot_list.append(pend_point1)
        #     plot_list.append(pend_point2)
        #     plot_list.append(update_formation_position[i])
        #     x_values = [point[0] for point in plot_list]
        #     y_values = [point[1] for point in plot_list]
        #     # print(plot_list)
            
        #     plt.plot(x_values,y_values,marker = 'o',c = colors[i], markersize=3 )
        #     if pf[i] != None:
        #         plt.plot(pf[i][0],pf[i][1],color = colors[i],marker = "^")
        #### change from user_map to map to see the FOV coverage for each frame
        # plt.imshow(user_map,cmap='Greys')
        # plt.title("Formation Path Planning")
        # plt.gca().invert_yaxis()
        # filename1 = file_path + f'{n}.png'
        # # plt.savefig(filename1)
        # plt.pause(0.001)
        # plt.show()



        ###@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    # print("executed",execute_path,"center",execute_center_path)
    return execute_center_path,execute_path
        
if __name__ == "__main__":
    formation_center = np.array([0,0,1])
    formation_target = np.array([30,4,1])  
    formation_center = np.array([-8,0,1])
    formation_target = np.array([8,0,1]) 
    formation_pattern = np.array([[0.7,0.7,0],
                                  [-0.7,0.7,0],
                                  [0.7,-0.7,0],
                                [-0.7,-0.7,0],
                         ])
    obstacle = np.array([   
    [  1,  1.5,  8,0.4,0.4,20],
    [ -1,    3,  8,0.4,0.4,20],
    [ -2,    1,  8,0.4,0.4,20],
    [  2,    0,  8,0.4,0.4,20],
    [  3, -1.5,  8,0.4,0.4,20],
    [0.5,   -2,  8,0.4,0.4,20],
    [ -3, -0.5,  8,0.4,0.4,20],
    [  0,    0,  8,0.4,0.4,20]
                ])
    # formation_center = np.array([1,6,1])
    # formation_target = np.array([12,4,1])  
    ### diamond shape ###
    # formation_center = np.array([0,0,1])
    # formation_target = np.array([30,4,1])  
    # formation_pattern = np.array([[0.7,0,0],
    #                               [-0.7,0,0],
    #                               [0,0.7,0],
    #                             [0,-0.7,0],
    #                      ])
    ### square shape ###
    # formation_center = np.array([0,0,1])
    # formation_target = np.array([30,4,1])  
    # formation_pattern = np.array([[0.7,0.7,0],
    #                               [-0.7,0.7,0],
    #                               [0.7,-0.7,0],
    #                             [-0.7,-0.7,0],
    #                      ])
    ###line shape###
    # formation_center = np.array([0,0,1])
    # formation_target = np.array([30,4,1])  
    # formation_pattern = np.array([[1.5,0,0],
    #                               [0.5,0,0],
    #                               [-0.5,0,0],
    #                             [-1.5,0,0],
    #                      ])
    ### rectangle shape ###
    # formation_center = np.array([0,0,1])
    # formation_target = np.array([30,4,1])  
    # formation_pattern = np.array([[1.7,0.7,0],
    #                               [-1.7,0.7,0],
    #                               [1.7,-0.7,0],
    #                             [-1.7,-0.7,0],
    #                      ])
    ###triangle shape###
    formation_center = np.array([0,0,1])
    formation_target = np.array([30,4,1])  
    formation_pattern = np.array([[0,0,0],
                                  [1.5,0,0],
                                  [0,1.5,0],
                                  [0,-1.5,0],
                         ])
    ###experiment shape###
    # formation_center = np.array([0,0,1])
    # formation_target = np.array([30,4,1])  
    # formation_pattern = np.array([[1,0,0],
    #                               [0,1,0],
    #                               [0,-1,0],
    #                      ])
    # formation_pattern = np.array([
    #                 [0, 0],
    #                 [1.0, 1.0],
    #                 [1.0, -1.0],
    #                 [0, 1.6],
    #                 [0, -1.6],
    #                 [-0.6, 1.3],
    #                 [-0.6, -1.3],
    #                 [2.0, 0],
    #                 [-0.6, 0.6],
    #                 [-0.6, -0.6]
    #                 ]) 
    # formation_pattern = np.array([[0.5,0,0],
    #                               [1.5,0,0],
    #                               [-0.5,0,0],
    #                               [-1.5,0,0],
    #                      ])
    # formation_pattern = np.array([
    #                     [-0.5, 0],
    #                     [2.0, 0.0],
    #                     [-1.0, 1.0],
    #                     [0, 2],
    #                     [1, 1],
    #                     [-1, -1],
    #                     [0, -2],
    #                     [1, -1],
    #                     # [-0.6, 0.6],
    #                     # [-0.6, -0.6]
    #                   ]) 
    # obstacle = np.array([   
    #     [  1,  1.5,  8,0.4,0.4,20],
    #     [ -1,    3,  8,0.4,0.4,20],
    #     [ -2,    1,  8,0.6,0.6,20],
    #     [  2,    0,  8,0.4,0.4,20],
    #     [  3, -1.5,  8,0.6,0.8,20],
    #     [0.5,   -2,  8,0.4,0.4,20],
    #     [ -3, -0.5,  8,0.4,0.4,20],
    #     [  0,    0,  8,0.4,0.4,20],
    #     [  0,    0,  8,1,0.4,20],
    #     # [1,0,8,1,3,20]
    #                 ])
    # obstacle = np.array([   
    #     [  1,  1.5,  8,0.4,0.4,20],
    #     [ -1,    3,  8,0.4,0.4,20],
    #     [ -2,    1,  8,0.4,0.4,20],
    #     [ -2,    1.4,  8,0.4,0.4,20],
    #     [ -1.6,    1,  8,0.4,0.4,20],
    #     [  3,    0.5,  8,0.4,0.4,20],
    #     [  3, -1.5,  8,0.4,0.4,20],
    #     [  3, -1.9,  8,0.4,0.4,20],
    #     [  2.8, -1.5,  8,0.4,0.4,20],
    #     [0.5,   -2,  8,0.4,0.4,20],
    #     [ -3, -0.5,  8,0.4,0.4,20],
    #     [  0,    0,  8,0.4,0.4,20],
    #     [  0.4,    0,  8,0.4,0.4,20],
    #     [  0.8,    0,  8,0.4,0.4,20],
    #     # [  0,    0,  8,0.8,0.8,20],
    #     # [1,0,8,1,3,20]
    #                 ])
    # obstacle = np.array([   
    #     [  1,  1.5,  8,0.4,0.4,20],
    #     [ -1,    3,  8,0.4,0.8,20],
    #     [ -2,    1,  8,0.4,0.4,20],
    #     [  2,    0,  8,0.4,0.4,20],
    #     [  3, -1.5,  8,0.4,0.4,20],
    #     [0.5,   -2,  8,0.4,0.8,20],
    #     [ -3, -0.5,  8,0.4,0.4,20],
    #     [  0,    0,  8,0.4,0.4,20],
    #     [  0,    0.5,  8,0.4,0.8,20],
    #     # [1,0,8,1,3,20]
    #                 ])
    # obstacle = np.array([   
    #     [  1,  0,  8,4,0.5,20],
    #     [ 1,    1.5,  8,4,0.4,20],
    #     [ 1,    -1.5,  8,6,0.4,20],
    #     # [  2,    0,  8,0.4,0.4,20],
    #     # [  3, -0.7,  8,0.6,0.1,20],
    #     # [  3, -2.3,  8,0.6,0.1,20],
    #     # [  2.4, -1.5,  8,0.1,0.8,20],
    #     # [  3.6, -1.5,  8,0.1,0.8,20],
    #     # [0.5,   -2,  8,0.4,0.4,20],
    #     # [ -3, -0.5,  8,0.4,0.4,20],
    #     # [  0,    0,  8,0.4,0.4,20],
    #     # [  0,    0,  8,1,0.4,20],
    #     # [1,0,8,1,3,20]
    #                 ])
    ### experiment setting
    # obstacle = np.array([   
    #     [  4.6,  7.8,  8,1,1,20],
    #     [ 7.3,    4.6,  8,0.4,0.4,20],
    #     [ 13,    0,  8,0.01,0.01,20],
    #                 ])
    # seed = 1830564
    seed = 4255
    obstacle = generate_obstacles(2,26,-6,6,70,seed)
    print(obstacle)
    # formation_center = np.array([1,5,1])
    # formation_target = np.array([8,5,1])
    # obstacle = np.array([   [3,-6,4,0.1,0.1,20],
    #                         [3,6,4,0.1,0.1,20],
    #                 ])

    formation_center = np.array([0,0,1])
    formation_target = np.array([30,4,1])  
    formation_pattern = np.array([[1.5,0,0],
                                  [0.5,0,0],
                                  [-0.5,0,0],
                                [-1.5,0,0],
                         ])
    obstacle = np.array([   [15,-6,4,0.1,0.1,20],
                            [15,6,4,0.1,0.1,20],
                    ])
    plan_obstacle = obstacle.copy()
    plan_obstacle[:,-3:-1] += 0.1
    obs = np.empty((0,5))
    for point in plan_obstacle:
        x, y, z, dx, dy, dz = point[:6]
        # 需要保证追加的数据的形状为(1, 5)
        obs = np.append(obs, [[x, y, dx, dy, 0]], axis=0)
    # y_slice_size = 150#35
    voxel_size = 0.1
    iteration = 100
    shortest_path_num = 3
    waypoint_distance = 1 # round(0.6/voxel_size)
    FOV_angle = 60
    ### visible distance is 3 m
    visible_distance = 4
    boundary_distance = 15
    angle_step = 10
    # origin_map = np.full((slice_size, slice_size), 0)
    # known_map = np.full((slice_size, slice_size), 0)
    boundary_2D = get_boundary(obs,formation_center,formation_target,formation_pattern,buffer = visible_distance)
    print("boundary",boundary_2D)
    x_min,x_max,y_min,y_max = boundary_2D
    # voxel_size = round((y_max-y_min)/y_slice_size,2)

    x_slice_size = int((x_max-x_min)/voxel_size)
    y_slice_size = int((y_max-y_min)/voxel_size)
    map_shape = (x_slice_size,y_slice_size)
    print("voxel size",voxel_size)
    vision_step = 3
    origin_map,known_map,start_formation_pos,end_formation_pos = known_map_establish(obs,0,map_shape,formation_center[:2],formation_target[:2],formation_pattern[:,:2],boundary_2D)
    print("map size",known_map.shape)
    start = [from_real_to_map(formation_center[0],x_min,x_max,x_slice_size),from_real_to_map(formation_center[1],y_min,y_max,y_slice_size)]
    end = [from_real_to_map(formation_target[0],x_min,x_max,x_slice_size),from_real_to_map(formation_target[1],y_min,y_max,y_slice_size)]
    visible_distance = int(visible_distance/voxel_size)
    print("visible_distance",visible_distance)
    map_formation_pattern = from_real_to_map_pattern(formation_pattern,boundary_2D,[x_slice_size,y_slice_size])
    #astar
    a,b = minimum_deform_search(known_map,start,end,[0 for agt in range(len(formation_pattern))],map_formation_pattern,waypoint_distance,FOV_angle,angle_step,visible_distance,vision_step)
    print("ab",a,b)
    