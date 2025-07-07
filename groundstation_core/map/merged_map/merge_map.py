# # examples/Python/Basic/working_with_numpy.py

import copy
import numpy as np
import open3d as o3d
import cv2              
from MapCreator3D import Map_Creator,slice_map
import sys
sys.path.append("simulation_folder")
from O3D_Visualizer import *

if __name__ == "__main__":
    voxelSize = 0.1

    x_translate = 1
    y_translate = 0
    z_translate = 0

    n = 20
    map = o3d.geometry.PointCloud()
    R_translate = np.array([[1, 0, 0, x_translate],
                [0, 1, 0, y_translate],
                [0, 0, 1, z_translate],
                [0, 0, 0, 1]])
    for i in range(1):
        
        file_l = './pcd/2024-1-14/left/'+f'{n}.pcd'
        file_r = './pcd/2024-1-14/right/'+f'{n}.pcd'
        
        n+=1
        # viz = o3d_visualizer()
        pcd_load_l = o3d.io.read_point_cloud(file_l)
        pcd_load_r = o3d.io.read_point_cloud(file_r)
        pcd_l = pcd_load_l.voxel_down_sample(voxel_size=0.005)
        pcd_r = pcd_load_r.voxel_down_sample(voxel_size=0.005)
        pcd_r = pcd_r.transform(R_translate)
        map= map + pcd_l
        map = map + pcd_r
        voxelgrid = o3d.geometry.VoxelGrid.create_from_point_cloud(map, voxel_size = voxelSize)
        o3d.visualization.draw_geometries([pcd_l])
        o3d.visualization.draw_geometries([pcd_r])        
        o3d.visualization.draw_geometries([voxelgrid])
        # viz.update_canvas(pcd_load)
        # for i in range(100):
        #     maze = slice_map(voxelgrid,i*0.1-2,100)

        #     cv2.imshow('slice',maze)
        #     cv2.waitKey(0)
        #     cv2.destroyAllWindows()
        
# import time
# import numpy as np
# import open3d as o3d

# def main():
#     frame = o3d.geometry.TriangleMesh.create_coordinate_frame(1.5)
#     mesh = o3d.geometry.TriangleMesh.create_sphere()
#     mesh.compute_vertex_normals()

#     vis = o3d.visualization.Visualizer()
#     vis.create_window(width=640, height=480)
#     vis.add_geometry(frame)
#     vis.add_geometry(mesh)

#     ctr = vis.get_view_control()
#     ctr.set_lookat([0,0,0])
#     ctr.set_front([1,1,1])
#     ctr.set_up([0,0,1])
#     ctr.set_zoom(0.5)

#     i = np.tile(np.arange(len(mesh.vertices)),(3,1)).T # (8,3)
#     while True:
#         # Deform mesh vertices
#         vert = mesh.vertices + np.sin(i)*0.02
#         mesh.vertices = o3d.utility.Vector3dVector(vert)
#         i += 1

#         vis.update_geometry(mesh)
#         vis.update_renderer()
#         vis.poll_events()

#         time.sleep(0.05)
#         if i[0,0]>1000:
#             break

#     vis.run()

# if __name__ == "__main__":
#     main()