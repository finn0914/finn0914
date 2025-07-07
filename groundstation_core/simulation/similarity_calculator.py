from occupied_map import known_map_establish,from_real_to_map,get_boundary,from_real_to_map_pattern
import numpy as np
import util
import os
import csv
import matplotlib.pyplot as plt
import math
from scipy.interpolate import make_interp_spline, BSpline
import config


def compute_similarity_cost(first_pos, A_desired):
    N = len(first_pos)

    # 计算权重矩阵
    A = np.zeros((N, N))
    for i in range(N):
        for j in range(i+1, N):
            dist = util.euclidean_distance(first_pos[i], first_pos[j])
            # print(dist,first_pos[i], first_pos[j])
            A[i][j] = dist
            A[j][i] = dist

    # 计算度矩阵
    D = np.diag(np.sum(A, axis=1))
    if np.all(D == 0):
        return math.inf,math.inf
    # 计算拉普拉斯矩阵
    L = D - A
    # print("A:",A,'\n',"D:",D,'\n',"L:",L)

    # 规范化拉普拉斯矩阵
    D_sqrt_inv = np.linalg.inv(np.sqrt(np.diag(np.diag(L))))
    L_hat = np.dot(np.dot(D_sqrt_inv, L), D_sqrt_inv)

    # 计算目标规范化拉普拉斯矩阵
    D_desired = np.diag(np.sum(A_desired, axis=1))
    L_desired = D_desired - A_desired
    # print("A_desired:",A_desired,'\n',"D_desired:",D_desired,'\n',"L_desired:",L_desired)
    D_desired_sqrt_inv = np.linalg.inv(np.sqrt(np.diag(np.diag(L_desired))))
    L_hat_desired = np.dot(np.dot(D_desired_sqrt_inv, L_desired), D_desired_sqrt_inv)

    # fs with scaling
    # fs_ without scaling
    fs = np.linalg.norm(L_hat - L_hat_desired, 'fro') ** 2
    fs_ = np.linalg.norm(L - L_desired, 'fro') ** 2
    # print("fs,fs_",fs,fs_)
    return (fs,fs_)


data_folder = './results/save-flight-pid-08.10.2024_11.47.59'

plan_obstacle = config.OBSTACLES.copy()
plan_obstacle[:,-3:-1] += 0
obs = np.empty((0,5))
for point in plan_obstacle:
    x, y, z, dx, dy, dz = point[:6]
    # 需要保证追加的数据的形状为(1, 5)
    obs = np.append(obs, [[x, y, dx, dy, 0]], axis=0)
# y_slice_size = 150#35

# origin_map = np.full((slice_size, slice_size), 0)
# known_map = np.full((slice_size, slice_size), 0)
boundary_2D = get_boundary(obs, config.FORMATION_START_POINT, config.FORMATION_GOAL_POINT, config.FORMATION_PATTERN, buffer = config.CAMERA_RANGE)
print("boundary", boundary_2D)
x_min,x_max,y_min,y_max = boundary_2D
# config.VOXEL_SIZE = round((y_max-y_min)/y_slice_size,2)

x_slice_size = int((x_max-x_min) / config.VOXEL_SIZE)
y_slice_size = int((y_max-y_min) / config.VOXEL_SIZE)
map_shape = (x_slice_size,y_slice_size)
origin_map, known_map, start_formation_pos, end_formation_pos = known_map_establish(
    obs, 0, map_shape, config.FORMATION_START_POINT[:2], config.FORMATION_GOAL_POINT[:2], config.FORMATION_PATTERN[:,:2], boundary_2D)

def plot_positions_combined(formation_positions, formation_pattern, rotated_pattern):
    formation_positions = np.array(formation_positions)
    formation_pattern = np.array(formation_pattern)
    rotated_pattern = np.array(rotated_pattern)

    fig, ax = plt.subplots()

    # Plot initial formation positions
    ax.scatter(formation_positions[:, 0], formation_positions[:, 1], c='b', marker='o', label='Formation Positions')

    # Plot rotated pattern
    ax.scatter(rotated_pattern[:, 0], rotated_pattern[:, 1], c='g', marker='x', label='Rotated Pattern')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Positions Comparison')
    ax.legend()

    plt.show()

# # Example usage
# formation_positions = [[54, 94, 0], [37, 92, 0], [59, 82, 0], [38, 79, 0]]
# formation_pattern = [[2, 2, 0], [2, -2, 0], [-2, 2, 0], [-2, -2, 0]]

# rotated_pattern = find_best_rotation(formation_positions, formation_pattern)

# plot_positions_combined(formation_positions, formation_pattern, rotated_pattern)

def read_csv(filename):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # 跳過表頭行
        data = [float(row[0]) for row in reader]
    return data

def read_csv_first_column(filename):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # 跳過表頭行
        data = [float(row[0]) for row in reader]  # Read second column (index 1)
    return data

def read_csv_second_column(filename):
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # 跳過表頭行
        data = [float(row[1]) for row in reader]  # Read second column (index 1)
    return data

def load_paths(folder):
    selected_paths = []
    execute_time = []
    plot_paths = [[] for i in range(len(config.FORMATION_PATTERN))]

    for agent_id in range(len(config.FORMATION_PATTERN)):
        x_file = os.path.join(folder, f'x{agent_id}.csv')
        y_file = os.path.join(folder, f'y{agent_id}.csv')

        # Print file paths for debugging
        print(f'Reading x_file: {x_file}')
        print(f'Reading y_file: {y_file}')

        # Check if files exist
        if not os.path.exists(x_file) or not os.path.exists(y_file):
            print(f'File not found: {x_file} or {y_file}')
            continue

        x_data = read_csv_second_column(x_file)
        y_data = read_csv_second_column(y_file)
        execute_time_step = read_csv_first_column(x_file)

        path = list(zip(x_data, y_data))
        selected_paths.append(path)
        for point in path:
            plot_paths[agent_id].append((from_real_to_map(point[0],x_min,x_max,x_slice_size),from_real_to_map(point[1],y_min,y_max,y_slice_size)))

    return selected_paths, plot_paths, execute_time_step

def find_best_transformation(formation_position, formation_pattern):
    # 转换为numpy数组
    formation_position = np.array(formation_position)
    formation_pattern = np.array(formation_pattern)

    # 计算质心
    center_pos = np.mean(formation_position, axis=0)
    center_pat = np.mean(formation_pattern, axis=0)

    # 中心化点集
    formation_position_centered = formation_position - center_pos
    formation_pattern_centered = formation_pattern - center_pat

    # 计算缩放因子
    norm_pos = np.linalg.norm(formation_position_centered)
    norm_pat = np.linalg.norm(formation_pattern_centered)
    scale = norm_pos / norm_pat

    # 缩放 pattern
    formation_pattern_centered *= scale

    # 计算旋转矩阵
    H = np.dot(formation_pattern_centered.T, formation_position_centered)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # 如果旋转矩阵行列式为负，调整旋转矩阵
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)

    # 应用旋转矩阵
    rotated_pattern = np.dot(formation_pattern_centered, R)

    # 恢复原点位置
    rotated_pattern += center_pos

    return rotated_pattern

selected_paths, plan_paths, execute_time_step = load_paths(data_folder)
current_time = 0
start_index = 0
if execute_time_step[0]>1000:
    for time_step in range(len(execute_time_step)):
        if util.euclidean_distance(selected_paths[0][time_step],selected_paths[0][time_step+1])>0:
            start_index = time_step
            current_time = execute_time_step[time_step]
            break

    for time_step in range(len(execute_time_step)):
        execute_time_step[time_step] -= current_time
        execute_time_step[time_step] /= 10**9
# Print the selected_paths for verification
# print(selected_paths)

# Plotting the paths
colors = ['r', 'g', 'b', 'm','r', 'g', 'b', 'm']
markers = ['o', 's', '^', 'D','o', 's', '^', 'D']

plt.figure(figsize=(10, 8))
for idx, path in enumerate(plan_paths):
    x = [point[0] for point in path]
    y = [point[1] for point in path]
    plt.plot(x, y, color=colors[idx], marker=markers[idx], label=f'Path {idx + 1}',markersize = 2)

# Adding labels and legend
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.title('Selected Paths')
plt.legend()
plt.imshow(known_map)
plt.gca().invert_yaxis()
plt.show()

def calculate_new_positions(center, formation_pattern):
    # 根据 formation_pattern 计算新的位置
    new_positions = [list(center + np.array(pattern)) for pattern in formation_pattern]
    return new_positions
def find_closest_indices(sample_pos,formation_pattern):
    center = find_team_center(sample_pos)
    new_positions = calculate_new_positions(center, formation_pattern)
    num_agents = len(new_positions)
    closest_indices = []
    used_indices = set()

    for i in range(num_agents):
        min_distance = float('inf')
        closest_index = None

        for j in range(num_agents):
            if j not in used_indices:
                # distance = util.euclidean_distance(new_positions[i], sample_pos[j])
                distance = util.euclidean_distance(sample_pos[i], new_positions[j])
                if distance < min_distance:
                    min_distance = distance
                    closest_index = j

        closest_indices.append(closest_index)
        used_indices.add(closest_index)

    return closest_indices
def find_team_center(pos_list):
    # 计算队伍中心（所有位置的平均值）
    center = np.mean(pos_list, axis=0)
    return center

record_y_max = 0
record_y_min = math.inf
similarity_formation = []
e_sim = []
e_sim_scale = []
path_length = 0
min_len = math.inf
N = len(config.FORMATION_PATTERN)
A_desired = np.zeros((N, N))
for i in range(N):
    for j in range(i+1, N):
        desired_dist = util.euclidean_distance(config.FORMATION_PATTERN[i], config.FORMATION_PATTERN[j])
        A_desired[i][j] = desired_dist
        A_desired[j][i] = desired_dist
for idx in range(len(selected_paths)):
    if len(selected_paths[idx])<min_len:
        min_len = len(selected_paths[idx])

for j in range(start_index,min_len):
    a=0
    formation_position = []
    ideal_formation_position = []
    e_cost = 0
    for i in range(len(config.FORMATION_PATTERN)):
        formation_position.append(selected_paths[i][j])
        if j!=0:
            path_length += util.euclidean_distance(selected_paths[i][j], selected_paths[i][j-1]) / len(config.FORMATION_PATTERN)
    for i in range(len(config.FORMATION_PATTERN)-1):
        for k in range(i, len(config.FORMATION_PATTERN)):
            e_cost += abs(
                util.euclidean_distance(formation_position[i], formation_position[k]) - util.euclidean_distance(config.FORMATION_PATTERN[i], config.FORMATION_PATTERN[k]))
    center = find_team_center(formation_position)
    index = find_closest_indices(formation_position,config.FORMATION_PATTERN[:,:2])
    for ID in range(len(config.FORMATION_PATTERN)):
        ideal_formation_position.append(np.array(config.FORMATION_PATTERN[index[ID],:2])+np.array(center))
    fs,_ = compute_similarity_cost(formation_position,A_desired)
    e_sim.append(e_cost)
    e_sim_scale.append(fs)

    # ideal_formation_position_o = find_best_rotation(formation_position,ideal_formation_position)
    # ideal_formation_position = find_best_transformation(formation_position,ideal_formation_position)
    # print("w scale:",ideal_formation_position,"\n","w/o scale",ideal_formation_position_o)

    for i in range(len(config.FORMATION_PATTERN)):
        a+=util.euclidean_distance(formation_position[i], ideal_formation_position[i])
    # a = compute_similarity_cost(formation_position,A_desired)

    similarity_formation.append(a)
    if a>record_y_max:
        record_y_max = a
    elif a<record_y_min:
        record_y_min = a
    # print("print info", formation_position, ideal_formation_position)
    # plot_positions_combined(formation_position, config.FORMATION_PATTERN, ideal_formation_position)

# print("Similarity Estimate:",similarity_formation)
print("Average Deformation Value:",sum(similarity_formation)/len(similarity_formation))
print("Average Travelled Distance:",path_length)
plt.figure()
xnew = np.linspace(0, 1, len(similarity_formation))
# spl = make_interp_spline(range(len(similarity_formation)), similarity_formation, k=3)  # type: BSpline
# power_smooth = spl(xnew)
# plt.plot(xnew, power_smooth)
print("len",len(execute_time_step),"start",start_index,"min_len",min_len)
plt.plot(xnew,similarity_formation)
print("Time:", execute_time_step[min_len-1])
plt.ylim(0, 5)
# plt.ylim(0, record_y_max+1)
plt.xlabel('Normalized Time')
plt.ylabel('Formation Deformation Value (m)')
# plt.title('Formation Deform Value vs. Time')
plt.show()
plt.plot(xnew,e_sim)
print("Average similarity error:",sum(e_sim)/len(e_sim))
plt.ylim(0, 5)
# plt.ylim(0, )
plt.xlabel('Normalized Time')
plt.ylabel('Formation Similarity Error (m)')
# plt.title('Similarity Error vs. Time')
plt.show()
plt.plot(xnew,e_sim_scale)
print("Average similarity error with scaling:",sum(e_sim_scale)/len(e_sim_scale))
plt.ylim(0, 0.001)
# plt.ylim(0, )
plt.xlabel('Normalized Time')
plt.ylabel('Formation Similarity Error with Scaling')
# plt.title('Similarity Error vs. Time')
plt.show()
