#!/usr/bin/python3
import threading
from my_tcp import TCPServer
import config
import util.coordinate_transform as ct
import trajectory.run as traj
from util.log_no_ros import *
import traceback

import rclpy
from rclpy.executors import ExternalShutdownException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import QuaternionStamped
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import TrajectorySetpoint
from stationui_frontend_rviz_plugin_msgs.msg import RecvState
from stationui_frontend_rviz_plugin_msgs.msg import RecvArmed
from stationui_frontend_rviz_plugin_msgs.msg import SendState
from stationui_frontend_rviz_plugin_msgs.msg import SendArmed
from stationui_frontend_rviz_plugin_msgs.msg import SendSetpointMode
from stationui_frontend_rviz_plugin_msgs.msg import SendOrigin
from functools import partial
import numpy as np

from scipy.spatial.transform import Rotation
def quat_to_euler(x, y, z, w):
    """
    From real-last Hamilton quaterions
    to intrinsic x-y-z Euler angles (roll-pitch yaw), in radians.
    """
    return Rotation.from_quat((x, y, z, w)).as_euler('xyz')
def euler_to_quat(roll_radian, pitch_radian, yaw_radian):
    """
    From intrinsic x-y-z Euler angles (roll-pitch yaw), in radians,
    to real-last Hamilton quaterions.
    """
    return Rotation.from_euler('xyz', (roll_radian, pitch_radian, yaw_radian)).as_quat('xyz')

def us(sec, nanosec):
    return sec*1000_000 + nanosec//1000

def spin_and_catch(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


# sanitization, error if invalid
def to_drone_id(s):
    if s in ["1", "2", "3"]:
        return s
    raise RuntimeError("Unknown drone_id %s"%s)

def drone_id_to_host(id):
    if id == "1":
        return config.DRONE1_HOST
    elif id == "2":
        return config.DRONE2_HOST
    elif id == "3":
        return config.DRONE3_HOST
    raise Exception("Unexpected drone_id %s"%id)

def host_to_drone_id(host):
    if host == config.DRONE1_HOST:
        return "1"
    elif host == config.DRONE2_HOST:
        return "2"
    elif host == config.DRONE3_HOST:
        return "3"
    raise Exception("Unexpected host %s"%host)


ros_node = None
class ROSNode():
    def __init__(self):
        rclpy.init()

        NODE_NAME = "stationui_backend"
        self.node = rclpy.create_node(NODE_NAME)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        AGENT_COUNT = 3

        self.pubs = dict()
        self.pubs["state"] = []
        self.pubs["armed"] = []
        self.pubs["pose"] = []
        self.pubs["position_raw"] = []
        self.pubs["setpoint_xyz"] = []
        self.pubs["setpoint_xyzyaw"] = []
        self.pubs["setpoint_vxvyvz"] = []
        for i in range(1, AGENT_COUNT+1):
            self.pubs["state"].append(self.node.create_publisher(RecvState, "~/drone10{}/recv_state".format(i), 10))
            self.pubs["armed"].append(self.node.create_publisher(RecvArmed, "~/drone10{}/recv_armed".format(i), 10))
            self.pubs["pose"].append(self.node.create_publisher(VehicleOdometry, "~/drone10{}/recv_pose".format(i), 10))
            self.pubs["position_raw"].append(self.node.create_publisher(VehicleOdometry, "~/drone10{}/recv_position_raw".format(i), 10))
            self.pubs["setpoint_xyz"].append(self.node.create_publisher(TrajectorySetpoint, "~/drone10{}/recv_setpoint_xyz".format(i), 10))
            self.pubs["setpoint_xyzyaw"].append(self.node.create_publisher(TrajectorySetpoint, "~/drone10{}/recv_setpoint_xyzyaw".format(i), 10))
            self.pubs["setpoint_vxvyvz"].append(self.node.create_publisher(TrajectorySetpoint, "~/drone10{}/recv_setpoint_vxvyvz".format(i), 10))
        
        FRONTEND_NODE_NAME = "rviz"
        self.subs = dict()
        self.subs["state"] = []
        self.subs["armed"] = []
        self.subs["setpoint_mode"] = []
        self.subs["setpoint_xyz"] = []
        self.subs["setpoint_xyzyaw"] = []
        self.subs["setpoint_xyzautoyaw"] = []
        self.subs["setpoint_vxvyvz"] = []
        self.subs["origin"] = []
        for i in range(1, AGENT_COUNT+1):
            self.subs["state"].append(self.node.create_subscription(SendState,"{}/drone10{}/send_state".format(FRONTEND_NODE_NAME, i),
                partial(self.on_send_state, str(i)), 10))
            self.subs["armed"].append(self.node.create_subscription(SendArmed, "{}/drone10{}/send_armed".format(FRONTEND_NODE_NAME, i),
                partial(self.on_send_armed, str(i)), 10))
            self.subs["setpoint_mode"].append(self.node.create_subscription(SendSetpointMode, "{}/drone10{}/send_setpoint_mode".format(FRONTEND_NODE_NAME, i),
                partial(self.on_send_setpoint_mode, str(i)), 10))
            self.subs["setpoint_xyz"].append(self.node.create_subscription(TrajectorySetpoint, "{}/drone10{}/send_setpoint_xyz".format(FRONTEND_NODE_NAME, i),
                partial(self.on_send_setpoint_xyz, str(i)), 10))
            self.subs["setpoint_xyzyaw"].append(self.node.create_subscription(TrajectorySetpoint, "{}/drone10{}/send_setpoint_xyzyaw".format(FRONTEND_NODE_NAME, i),
                partial(self.on_send_setpoint_xyzyaw, str(i)), 10))
            self.subs["setpoint_vxvyvz"].append(self.node.create_subscription(TrajectorySetpoint, "{}/drone10{}/send_setpoint_vxvyvz".format(FRONTEND_NODE_NAME, i),
                partial(self.on_send_setpoint_vxvyvz, str(i)), 10))
            self.subs["origin"].append(self.node.create_subscription(SendOrigin, "{}/drone10{}/send_origin".format(FRONTEND_NODE_NAME, i),
                partial(self.on_send_origin, str(i)), 10))


        self.spin_thread = threading.Thread(target=spin_and_catch, args=(self.node, ), daemon=True)
        self.spin_thread.start()

        self.server_ref = None

    def close(self):
        self.spin_thread.join()
    
    def set_server_ref(self, server_ref):
        self.server_ref = server_ref

    def _ct_xyz(self, from_x, from_y, from_z, from_frame_id, to_frame_id):
        """May throw."""
        point_from = PointStamped()
        point_from.header.frame_id = from_frame_id
        point_from.point[:] = from_x, from_y, from_z
        point_to = tf_buffer.transform(point_from, to_frame_id)
        return point_to.point
    def _ct_rpy(self, from_roll_radian, from_pitch_radian, from_yaw_radian, from_frame_id, to_frame_id):
        """May throw."""
        attitude_from = QuaternionStamped()
        attitude_from.header.frame_id = from_frame_id
        attitude_from.quaternion = Quaternion(*euler_to_quat(roll, pitch, yaw))
        attitude_to = tf_buffer.transform(attitude_from, to_frame_id)
        return quat_to_euler(attitude_to.quaternion)

    def enu_to_px4(self, enu_x, enu_y, enu_z):
        return self._ct_xyz(enu_x, enu_y, enu_z, from_frame_id="map_enu", to_frame_id="map_px4")
    def px4_to_enu(self, px4_x, px4_y, px4_z):
        return self._ct_xyz(px4_x, px4_y, px4_z, from_frame_id="map_px4", to_frame_id="map_enu")
    def enu_to_px4_rpy(self, enu_roll_radian, enu_pitch_radian, enu_yaw_radian):
        return self._ct_rpy(enu_roll_radian, enu_pitch_radian, enu_yaw_radian,
            from_frame_id="map_enu", to_frame_id="map_px4")
    def px4_to_enu_rpy(self, px4_roll_radian, px4_pitch_radian, px4_yaw_radian):
        return self._ct_rpy(px4_roll_radian, px4_pitch_radian, px4_yaw_radian,
            from_frame_id="map_px4", to_frame_id="map_enu")

    def on_recv_state(self, drone_id, state_string):
        msg = RecvState()
        msg.state_string = state_string
        self.pubs["state"][int(drone_id) - 1].publish(msg)
    def on_recv_armed(self, drone_id, armed):
        msg = RecvArmed()
        msg.armed = armed
        self.pubs["armed"][int(drone_id) - 1].publish(msg)
    def on_recv_pose(self, drone_id, t, x, y, z, vx, vy, vz, roll, pitch, yaw):
        msg = VehicleOdometry()
        msg.timestamp = us(*t)
        msg.pose_frame = 1 # NED (PX4)
        msg.velocity_frame = 1 # NED (PX4)
        msg.position[:] = x, y, z
        msg.velocity[:] = vx, vy, vz
        msg.q[:] = euler_to_quat(roll, pitch, yaw)
        self.pubs["pose"][int(drone_id) - 1].publish(msg)
    def on_recv_raw_position(self, drone_id, t, x, y, z):
        msg = VehicleOdometry()
        msg.timestamp = us(*t)
        msg.pose_frame = 0 # UNKNOWN (ENU)
        msg.position[:] = x, y, z
        self.pubs["position_raw"][int(drone_id) - 1].publish(msg)
    def on_recv_setpoint_xyz(self, drone_id, enu_x, enu_y, enu_z):
        msg = TrajectorySetpoint()
        try:
            msg.position[:] = self.enu_to_px4(enu_x, enu_y, enu_z)
        except:
            return
        self.pubs["setpoint_xyz"][int(drone_id) - 1].publish(msg)
    def on_recv_setpoint_xyzyaw(self, drone_id, enu_x, enu_y, enu_z, yaw_degree):
        msg = TrajectorySetpoint()
        try:
            msg.position[:] = self.enu_to_px4(enu_x, enu_y, enu_z)
            msg.yaw = self.enu_to_px4_rpy(0.0, 0.0, np.deg2rad(yaw_degree))[2]
        except:
            return
        self.pubs["setpoint_xyzyaw"][int(drone_id) - 1].publish(msg)
    def on_recv_setpoint_vxvyvz(self, drone_id, enu_vx, enu_vy, enu_vz):
        msg = TrajectorySetpoint()
        try:
            msg.velocity[:] = self.enu_to_px4(enu_vx, enu_vy, enu_vz)
        except:
            return
        self.pubs["setpoint_vxvyvz"][int(drone_id) - 1].publish(msg)
    def on_send_state(self, drone_id, msg):
        if self.server_ref is None:
            return
        if msg.state_string == "REBOOT_FCU":
            self.server_ref.send_to_drone(drone_id, ("reboot_fcu",))
        else:
            self.server_ref.send_to_drone(drone_id, ("set_mode", msg.state_string))
    def on_send_armed(self, drone_id, msg):
        if self.server_ref is None:
            return
        self.server_ref.send_to_drone(drone_id, ("set_armed", msg.armed))
    def on_send_setpoint_mode(self, drone_id, msg):
        if self.server_ref is None:
            return
        self.server_ref.send_to_drone(drone_id, ("set_setpoint_mode", msg.setpoint_mode))
    def on_send_setpoint_xyz(self, drone_id, msg):
        if self.server_ref is None:
            return
        self.server_ref.send_to_drone(drone_id, ("set_setpoint_xyz", *msg.position))
    def on_send_setpoint_xyzyaw(self, drone_id, msg):
        if self.server_ref is None:
            return
        self.server_ref.send_to_drone(drone_id, ("set_setpoint_xyzyaw", *msg.position, int(np.rad2deg(msg.yaw))))
    def on_send_setpoint_vxvyvz(self, drone_id, msg):
        if self.server_ref is None:
            return
        self.server_ref.send_to_drone(drone_id, ("set_setpoint_vxvyvz", *msg.position))
    def on_send_origin(self, drone_id, msg):
        if self.server_ref is None:
            return
        self.server_ref.send_to_drone(drone_id, ("set_origin",))
    def on_pong(self, drone_id, s0, ns0):
        s1, ns1 = ros_node.node.get_clock().now().seconds_nanoseconds()
        dt = (s1 - s0) * 1_000_000_000 + ns1 - ns0
        loginfo("Recevied pong from drone %s, RTT=%dms."%(drone_id, dt / 1_000_000))


class TrajectorySender:
    def __init__(self, drones):
        self.drone_id = None
        self.trajectory = None
        self.drones = drones
        self.stop_event = threading.Event()
        self.running_event = threading.Event()
    
    def load(self, drone_id, filename):
        self.drone_id = drone_id
        self.trajectory = traj.load_trajectory(filename)
        if self.trajectory is None:
            logerr("Load trajactory %s failed"%filename)
            return
        loginfo("Loaded trajactory %s"%filename)

    def run(self):
        if self.trajectory is None:
            logerr("No trajectory loaded")
            return
        if self.running_event.is_set():
            print("still running")
            return
        self.stop_event.clear()
        self.thread = threading.Thread(
            target=traj.play_trajectory,
            args=(self.stop_event, self.running_event, self.trajectory, self._callback))
        self.thread.start()

    def stop(self):
        self.stop_event.set()

    def _callback(self, x, y, z, yaw_degree):
        ned_x, ned_y, ned_z = ct.enu_to_ned(x, y, z)
        self.drones.send_to_drone(self.drone_id, ("set_setpoint", ned_x, ned_y, ned_z, yaw_degree))


class DroneState:
    def __init__(self):
        self.is_armed = None
        self.mode = None


class Server:
    def __init__(self, ros_node):
        self.tcp_server = TCPServer()
        # self.recv_should_run = False
        self.recv_stop_event = threading.Event()
        self.ros_node = ros_node
        self.drone_states = dict()

    def send_to_drone(self, drone_id, payload):
        self._clear_closed_clients()
        host = drone_id_to_host(drone_id)
        try:
            self.tcp_server.clients[host].send(payload)
        except Exception as e:
            logerr(e)
            logerr("Not connected to drone %s!"%drone_id)

    def _clear_closed_clients(self):
        to_clear = [host for (host, client) in self.tcp_server.clients.items() if client.is_closed()]
        for host in to_clear:
            self.tcp_server.remove_client(host)
            del self.drone_states[host_to_drone_id(host)]
            loginfo("[TCP] Removed connection from %s"%host)

    def run(self):
        host = "10.0.1.100"
        port = 9000
        if not self.tcp_server.try_bind(host, port):
            raise Exception("Binding to %s:%d failed."%(host, port))
        self.tcp_server.run()

        self.recv_thread = threading.Thread(target = self._recv_thread_target)
        self.recv_thread.start()

    def _recv_thread_target(self):
        while not self.recv_stop_event.is_set():
            try:
                for host, client in self.tcp_server.clients.items():
                    msg = client.recv()
                    if msg is None:
                        continue

                    id = host_to_drone_id(host)
                    if id not in self.drone_states.keys():
                        self.drone_states[id] = DroneState()
                    
                    try:
                        if msg[0] == "state":
                            self.ros_node.on_recv_state(id, msg[1])
                        elif msg[0] == "is_armed":
                            self.ros_node.on_recv_armed(id, msg[1])
                        elif msg[0] == "raw_position":
                            self.ros_node.on_recv_raw_position(id, msg[1], msg[2], msg[3], msg[4])
                        elif msg[0] == "pose":
                            self.ros_node.on_recv_pose(id, msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7], msg[8], msg[9], msg[10])
                        elif msg[0] == "setpoint_xyz":
                            self.ros_node.on_recv_setpoint_xyz(id, msg[1], msg[2], msg[3])
                        elif msg[0] == "setpoint_xyzyaw":
                            self.ros_node.on_recv_setpoint_xyzyaw(id, msg[1], msg[2], msg[3], msg[4])
                        elif msg[0] == "setpoint_vxvyvz":
                            self.ros_node.on_recv_setpoint_vxvyvz(id, msg[1], msg[2], msg[3])
                        elif msg[0] == "pong":
                            self.ros_node.on_pong(id, msg[1], msg[2])
                        else:
                            loginfo((id, msg))
                    except Exception as e:
                        logerr(e)
            except Exception as e:
                # self.tcp_server.clients might change size during iteration
                # e.g. drone join/leave
                # Just ignoring it and move on without creating a copy.
                if str(e) == "dictionary changed size during iteration":
                    pass
                else:
                    raise(e)
                

    def stop(self):
        self.recv_stop_event.set()
        self.tcp_server.close()


def to_bool(s):
    b = s.lower() in ['true', '1']
    loginfo("Interpret as %r"%b)
    return b

def main():
    global ros_node
    ros_node = ROSNode()

    server = Server(ros_node)
    server.run()
    ros_node.set_server_ref(server)

    trajectory_senders = dict()

    group = []
    def get_drone_ids(token_1):
        drone_ids = []
        if token_1 == "g":
            drone_ids = group
        else:
            drone_ids.append(to_drone_id(token_1))
        if len(drone_ids) == 0:
            logwarn("No target drones!")
        return drone_ids
    while rclpy.ok():
        # 1. Send commands according to input.
        try:
            s = input("Running..." )
        # 2. Handle exceptions.
        except KeyboardInterrupt:
            print() # Go down one line in the console (for aesthetics).
            print("Closing drone connections...")
            server.stop()
            print("Quit...")
            break
        except KeyError as e:
            logerr(traceback.format_exc())
            logwarn("Perhaps drone is not connected?")
        except IndexError as e:
            logerr(traceback.format_exc())
            logwarn("Perhaps command is missing parameters?")
        except OSError as e:
            logerr(traceback.format_exc())
            if e.errno == 113: # No route to host
                logwarn("Perhaps drone is not on, or host address is wrong?")
        except Exception as e:
            logerr(traceback.format_exc())

    server.stop()
    ros_node.close()

if __name__=="__main__":
    main()
