#!/usr/bin/python3

"""
A ROS2 node to generate pose-pcd pairs.

It converts `PointCloud2` to `o3d.geometry.PointCloud`,
pairs it to the closest `XyzrpyStamped`,
and put the pair into a queue given at construction.

Use this class with your own code.
See `main` for example usage.
"""

import threading
import rclpy
from rclpy.executors import ExternalShutdownException
import queue
from station_flight.msg import XyzrpyStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d

import time # Only used in `main`.


class IndexableQueue(queue.Queue):
    def __getitem__(self, index):
        with self.mutex:
            return self.queue[index]

class PCDGenerator():
    @staticmethod
    def spin_and_catch(node):
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass

    def __init__(self, output_queue):
        rclpy.init()
        self.node = rclpy.create_node("point_cloud2_to_o3d")

        self.pose_queue_1 = IndexableQueue(maxsize=20)
        self.pose_queue_2 = IndexableQueue(maxsize=20)
        self.pose_queue_3 = IndexableQueue(maxsize=20)
        self.sub_pose_1 = self.node.create_subscription(
            XyzrpyStamped, "station/drone101/xyzrpy_stamped", self._cb_pose_1, 10)
        self.sub_pose_1 = self.node.create_subscription(
            XyzrpyStamped, "station/drone102/xyzrpy_stamped", self._cb_pose_2, 10)
        self.sub_pose_1 = self.node.create_subscription(
            XyzrpyStamped, "station/drone103/xyzrpy_stamped", self._cb_pose_3, 10)

        self.output_queue = output_queue # Tuple of x, y, z, yaw, o3d_cloud.
                                         # See `_cb_point_cloud2`.
        self.sub_point_cloud_1 = self.node.create_subscription(
            PointCloud2, "station/drone101/point_cloud2", self._cb_point_cloud2_1, 10)
        self.sub_point_cloud_2 = self.node.create_subscription(
            PointCloud2, "station/drone102/point_cloud2", self._cb_point_cloud2_2, 10)
        self.sub_point_cloud_3 = self.node.create_subscription(
            PointCloud2, "station/drone103/point_cloud2", self._cb_point_cloud2_3, 10)

    def start(self):
        self.spin_thread = threading.Thread(
            target=PCDGenerator.spin_and_catch, args=(self.node, ), daemon=True)
        self.spin_thread.start()

    def stop(self):
        rclpy.shutdown()
        self.spin_thread.join()

    def _cb_pose_1(self, msg):
        if self.pose_queue_1.qsize() == 20:
            self.pose_queue_1.get()
        self.pose_queue_1.put(msg)

    def _cb_pose_2(self, msg):
        if self.pose_queue_2.qsize() == 20:
            self.pose_queue_2.get()
        self.pose_queue_2.put(msg)

    def _cb_pose_3(self, msg):
        if self.pose_queue_3.qsize() == 20:
            self.pose_queue_3.get()
        self.pose_queue_3.put(msg)

    @staticmethod
    def _to_o3d_point_cloud(msg_point_cloud2):
        points = point_cloud2.read_points(
            msg_point_cloud2, field_names=("x", "y", "z"), skip_nans=True)
        points_np = np.empty((points.size, 3))
        for i in range(points.size):
            points_np[i][0] = points[i][0]
            points_np[i][1] = points[i][1]
            points_np[i][2] = points[i][2]
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(points_np)
        return o3d_cloud
    
    @staticmethod
    def _to_nanoseconds(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec

    @staticmethod
    def _find_nearest_pose_msg(stamp, pose_queue):
        if pose_queue.empty():
            return None
        t0 = PCDGenerator._to_nanoseconds(stamp)

        # Keep comparing 0-th and 1-th element in queue.
        best_pose_msg = pose_queue[0]
        best_dt = abs(PCDGenerator._to_nanoseconds(pose_queue[0].stamp) - t0)
        while pose_queue.qsize() >= 2:
            dt = abs(PCDGenerator._to_nanoseconds(pose_queue[1].stamp) - t0)
            if best_dt < dt:
                # Found. No need to continue popping queue.
                break
            # 1-th is closer in time than the 0-th.
            best_pose_msg = pose_queue[1] # Record 1-th.
            best_dt = dt
            pose_queue.get()              # Discard 0-th.

        return best_pose_msg

    def _cb_point_cloud2_1(self, msg):
        self._cb_point_cloud2(msg, self.pose_queue_1)

    def _cb_point_cloud2_2(self, msg):
        self._cb_point_cloud2(msg, self.pose_queue_2)

    def _cb_point_cloud2_3(self, msg):
        self._cb_point_cloud2(msg, self.pose_queue_3)

    def _cb_point_cloud2(self, cloud_msg, pose_queue):
        o3d_cloud = PCDGenerator._to_o3d_point_cloud(cloud_msg)
        nearest_pose_msg = PCDGenerator._find_nearest_pose_msg(cloud_msg.header.stamp, pose_queue)
        if nearest_pose_msg is None:
            return
        
        try:
            self.output_queue.put_nowait((
                nearest_pose_msg.x, nearest_pose_msg.y, nearest_pose_msg.z,
                nearest_pose_msg.yaw, o3d_cloud
            ))
        except queue.Full:
            pass

def main():
    q = queue.Queue(maxsize=10)
    pcd_generator = PCDGenerator(q)
    pcd_generator.start()

    t0 = time.perf_counter_ns()
    while time.perf_counter_ns() - t0 < 2 * 1_000_000_000:
        try:
            x = q.get(timeout=0.1)
            print("x: %.1f, y: %.1f, z: %.1f, yaw: %.1f"%(x[0], x[1], x[2], x[3]))
            print(repr(x[4]))
        except:
            pass
    
    print("done")
    pcd_generator.stop()

if __name__=="__main__":
    main()
