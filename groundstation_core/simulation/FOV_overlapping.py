### This is used to calculate the overlapping region of field of view between agents. ###

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon, MultiPolygon
from matplotlib.patches import Polygon as MplPolygon

def create_fov_sector(agent_pos, orientation, range_radius, fov_angle):
    """
    Create a field of view (FOV) sector represented as a Polygon.

    Parameters:
    - agent_pos: list of tuples, [(x1, y1), (x2, y2), ...], coordinates of agent positions.
    - orientation: list of floats, [orientation1, orientation2, ...], orientation angles of agents in degrees.
    - range_radius: float, radius of the FOV range (assumed same for all agents).
    - fov_angle: float, total FOV angle in degrees (assumed same for all agents).

    Returns:
    - List of Polygons representing the FOV sectors for each agent.
    """
    fov_sectors = []
    num_points = 50
    half_fov_rad = np.deg2rad(fov_angle / 2)

    for pos, orient in zip(agent_pos, orientation):
        orientation_rad = np.deg2rad(orient)
        sector_points = [pos]

        for i in range(num_points + 1):
            angle = orientation_rad - half_fov_rad + (2 * half_fov_rad * i / num_points)
            x = pos[0] + range_radius * np.cos(angle)
            y = pos[1] + range_radius * np.sin(angle)
            sector_points.append((x, y))

        fov_sectors.append(Polygon(sector_points))

    return fov_sectors

def calculate_fov_overlap(agent_pos, orientation, range_radius, fov_angle):
    """
    Calculate the total overlapping area of FOVs for multiple agents.

    Parameters:
    - agent_pos: list of tuples, [(x1, y1), (x2, y2), ...], coordinates of agent positions.
    - orientation: list of floats, [orientation1, orientation2, ...], orientation angles of agents in degrees.
    - range_radius: float, radius of the FOV range (assumed same for all agents).
    - fov_angle: float, total FOV angle in degrees (assumed same for all agents).

    Returns:
    - Polygon representing the total overlapping area of FOVs.
    """
    fov_sectors = create_fov_sector(agent_pos, orientation, range_radius, fov_angle)
    total_intersection = MultiPolygon([])

    num_agents = len(agent_pos)
    for i in range(num_agents):
        for j in range(num_agents):
            if i!=j:
                intersection = fov_sectors[i].intersection(fov_sectors[j])
                total_intersection = total_intersection.union(intersection)

    return total_intersection.area/2

def plot_fovs_and_overlap(agent_pos, orientation, range_radius, fov_angle, grid_map=None):
    fig, ax = plt.subplots()
    
    # Calculate FOV sectors
    fov_sectors = create_fov_sector(agent_pos, orientation, range_radius, fov_angle)
    total_intersection = calculate_fov_overlap(agent_pos, orientation, range_radius, fov_angle)
    
    # Plot each FOV sector
    for i, fov in enumerate(fov_sectors):
        color = 'blue' if i == 0 else 'red'  # Different colors for different agents
        patch = MplPolygon(list(fov.exterior.coords), closed=True, edgecolor=color, alpha=0.3)
        ax.add_patch(patch)
    
    # Plot total intersection
    if total_intersection.geom_type == 'Polygon':
        patch_intersection = MplPolygon(list(total_intersection.exterior.coords), closed=True, edgecolor='green', alpha=0.5)
        ax.add_patch(patch_intersection)
    elif total_intersection.geom_type == 'MultiPolygon' or total_intersection.geom_type == 'GeometryCollection':
        for geom in total_intersection.geoms:
            if geom.geom_type == 'Polygon':
                patch_intersection = MplPolygon(list(geom.exterior.coords), closed=True, edgecolor='green', alpha=0.5)
                ax.add_patch(patch_intersection)
    
    # Plot grid map
    grid_map = np.zeros((30,30))
    if grid_map is not None:
        for i in range(grid_map.shape[0]):
            for j in range(grid_map.shape[1]):
                if grid_map[i, j] == 1:
                    rect = plt.Rectangle((j, i), 1, 1, color='black', alpha=0.5)
                    ax.add_patch(rect)
                elif grid_map[i, j] == 0:
                    rect = plt.Rectangle((j, i), 1, 1, color='white', edgecolor='gray', alpha=0.3)
                    ax.add_patch(rect)
    
    ax.set_xlim(0,30)
    ax.set_ylim(0,30)
    ax.set_aspect('equal', 'box')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Field of View and Overlapping Area')
    plt.legend(['Agent 1 FOV', 'Agent 2 FOV', 'Overlap Area', 'Obstacle'])
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    # Example usage:
    agent_pos = [(10, 10),(10, 10),(10, 10),(10, 10)]
    orientation = [0,0,0, 0]
    range_radius = 10
    fov_angle = 90  # degrees

    # Calculate intersection of FOVs
    intersection = calculate_fov_overlap(agent_pos, orientation, range_radius, fov_angle)
    print("interception",intersection.area)
    # Plot FOVs and intersection
    plot_fovs_and_overlap(agent_pos, orientation, range_radius, fov_angle, intersection)
