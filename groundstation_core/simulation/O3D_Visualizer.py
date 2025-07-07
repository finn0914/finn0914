
import numpy as np
import open3d as o3d
import time

def ellipsoid_geometry_from_params(ellipsoids):
    ellipse_list = []
    for i in range(ellipsoids.shape[0]):
        centroid = ellipsoids[i,:3]
        rx,ry,rz = ellipsoids[i,-3:]

        transform_matrix = np.eye(4)
        np.fill_diagonal(transform_matrix,[rx,ry,rz,1])
        transform_matrix2 = np.eye(4)
        # transform_matrix2[:3,:3] = V.T
        transform_matrix2[:3,-1] = centroid
        transform_matrix2 = transform_matrix2 @ transform_matrix
            
        sphere = o3d.geometry.TriangleMesh.create_sphere()
        sphere.compute_vertex_normals()
        sphere.transform(transform_matrix2)
        ellipse_list.append(sphere)
    return ellipse_list

class o3d_visualizer():
    def __init__(self) -> None:
        self.vis = o3d.visualization.Visualizer()
        self.have_window = False
        self.window_width = 1280
        self.window_height = 960
    def update_canvas(self,geometries):
        if self.have_window:
            self.vis.clear_geometries()
            for geo in geometries:
                self.vis.add_geometry(geo,reset_bounding_box = False)
            self.vis.update_renderer()
            self.vis.poll_events()
        else:
            self.have_window = True
            self.vis.create_window(width = self.window_width, height = self.window_height)
            for geo in geometries:
                self.vis.add_geometry(geo,reset_bounding_box = True)

            self.ctrl = self.vis.get_view_control()
            # self.ctrl.change_field_of_view(90)
            self.ctrl.translate(0,800)
            self.ctrl.rotate(0,-450)
            self.ctrl.rotate(-500,0)
            self.ctrl.rotate(0,500)
            self.ctrl.rotate(100,0)
            self.ctrl.rotate(0,-100)
            
            
            self.ctrl.scale(0.01)
            self.ctrl.scale(0.4)

            
