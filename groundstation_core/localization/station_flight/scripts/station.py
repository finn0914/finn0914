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
from geometry_msgs.msg import Vector3Stamped
from station_flight.msg import XyzrpyStamped


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

        NODE_NAME = "station_flight"
        self.node = rclpy.create_node(NODE_NAME)
        self.pub_raw_position_1 = self.node.create_publisher(Vector3Stamped, "station/drone101/raw_xyz_stamped", 10)
        self.pub_raw_position_2 = self.node.create_publisher(Vector3Stamped, "station/drone102/raw_xyz_stamped", 10)
        self.pub_raw_position_3 = self.node.create_publisher(Vector3Stamped, "station/drone103/raw_xyz_stamped", 10)
        self.pub_pose_1 = self.node.create_publisher(XyzrpyStamped, "station/drone101/xyzrpy_stamped", 10)
        self.pub_pose_2 = self.node.create_publisher(XyzrpyStamped, "station/drone102/xyzrpy_stamped", 10)
        self.pub_pose_3 = self.node.create_publisher(XyzrpyStamped, "station/drone103/xyzrpy_stamped", 10)

        self.spin_thread = threading.Thread(target=spin_and_catch, args=(self.node, ), daemon=True)
        self.spin_thread.start()

    def close(self):
        self.spin_thread.join()

    def on_raw_position(self, drone_id, t, x, y, z):
        msg = Vector3Stamped()
        msg.header.stamp.sec, msg.header.stamp.nanosec = t[0], t[1]
        msg.vector.x, msg.vector.y, msg.vector.z = x, y, z
        if drone_id == "1":
            self.pub_raw_position_1.publish(msg)
        elif drone_id == "2":
            self.pub_raw_position_2.publish(msg)
        elif drone_id == "3":
            self.pub_raw_position_3.publish(msg)
    def on_pose(self, drone_id, t, x, y, z, roll, pitch, yaw):
        msg = XyzrpyStamped()
        msg.stamp.sec, msg.stamp.nanosec = t[0], t[1]
        msg.x, msg.y, msg.z = x, y, z
        msg.roll, msg.pitch, msg.yaw = roll, pitch, yaw
        if drone_id == "1":
            self.pub_pose_1.publish(msg)
        elif drone_id == "2":
            self.pub_pose_2.publish(msg)
        elif drone_id == "3":
            self.pub_pose_3.publish(msg)
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
        self.drones.send_to_drone(self.drone_id, ("set_setpoint", x, y, z, yaw_degree))


class DroneState:
    def __init__(self):
        self.is_armed = None
        self.mode = None


print_setpoint_counter = 0
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
                    
                    if msg[0] == "raw_position":
                        self.ros_node.on_raw_position(id, msg[1], msg[2], msg[3], msg[4])
                    elif msg[0] == "pose":
                        self.ros_node.on_pose(id, msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], msg[7])
                    elif msg[0] == "is_armed":
                        self.drone_states[id].is_armed = msg[1]
                    elif msg[0] == "mode":
                        self.drone_states[id].mode = msg[1]
                    elif msg[0] == "pong":
                        self.ros_node.on_pong(id, msg[1], msg[2])
                    elif msg[0] == "state":
                        loginfo((id, msg))
                    elif msg[0] in ["setpoint_xyz", "setpoint_xyzyaw", "setpoint_vxvyvz"]:
                        global print_setpoint_counter
                        print_setpoint_counter += 1
                        if print_setpoint_counter % 10 == 0:
                            loginfo(msg[0])
                            loginfo((id, msg))
                    else:
                        loginfo((id, msg))
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
            s = input(">>> " )
            tokens = s.split() # Arbitrary number of whitespaces
            loginfo(tokens)

            # Use pass (instead of continue) such that 2. and 3. are handled.
            if len(tokens) == 0 or tokens[0] == "":
                pass
            elif tokens[0] == "ros_node":
                ros_node.run()
            elif tokens[0] in ["group", "g"]:
                loginfo("group: %s"%repr(group))
            elif tokens[0] in ["group_add", "ga"]:
                drone_id = to_drone_id(tokens[1])
                if drone_id in group:
                    logwarn("Drone %s is already in group!"%drone_id)
                    pass
                group.append(drone_id)
                loginfo("Drone %s added to group."%drone_id)
            elif tokens[0] in ["group_clear", "gc"]:
                group.clear()
                loginfo("Group cleared.")
            elif tokens[0] in ["status"]:
                lines = []
                for id, state in server.drone_states.items():
                    lines.append("---")
                    lines.append("drone %s:"%id)
                    lines.append("  ARMED:  %s"%state.is_armed)
                    lines.append("  MODE:   %s"%state.mode)
                loginfo("\n".join(lines))
            elif tokens[0] in ["reboot_fcu", "r"]:
                drone_ids = get_drone_ids(tokens[1])
                for id in drone_ids:
                    server.send_to_drone(id, ("reboot_fcu",))
            elif tokens[0] in ["set_origin", "o"]:
                drone_ids = get_drone_ids(tokens[1])
                for id in drone_ids:
                    server.send_to_drone(id, ("set_origin",))
            elif tokens[0] in ["set_mode", "m"]:
                drone_ids = get_drone_ids(tokens[1])
                for id in drone_ids:
                    server.send_to_drone(id, ("set_mode", tokens[2]))
            elif tokens[0] in ["set_armed", "a"]:
                drone_ids = get_drone_ids(tokens[1])
                for id in drone_ids:
                    server.send_to_drone(id, ("set_armed", to_bool(tokens[2])))
            elif tokens[0] in ["takeoff", "t"]:
                drone_id = to_drone_id(tokens[1])
                h = float(tokens[2])
                server.send_to_drone(drone_id, ("takeoff", h))
            elif tokens[0] in ["set_setpoint_mode", "sm"]:
                drone_id = to_drone_id(tokens[1])
                if tokens[2] in ["off", "xyz", "xyzyaw", "vxvyvz", "land"]:
                    server.send_to_drone(drone_id, ("set_setpoint_mode", tokens[2]))
                else:
                    logerr("Invalid setpoint type %s"%tokens[2])
            elif tokens[0] in ["set_setpoint", "s"]:
                drone_id = to_drone_id(tokens[1])
                x = float(tokens[2])
                y = float(tokens[3])
                z = float(tokens[4])
                # Check xyz bounds.
                SETPOINT_MIN_X, SETPOINT_MAX_X = 0, 13
                SETPOINT_MIN_Y, SETPOINT_MAX_Y = 0, 12
                SETPOINT_MIN_Z, SETPOINT_MAX_Z = 0.5, 2
                is_out_of_bound = False
                if x < SETPOINT_MIN_X or x > SETPOINT_MAX_X:
                    logerr("x: %+.2f is out of bounds!"%x)
                    is_out_of_bound = True
                if y < SETPOINT_MIN_Y or y > SETPOINT_MAX_Y:
                    logerr("y: %+.2f is out of bounds!"%y)
                    is_out_of_bound = True
                if z < SETPOINT_MIN_Z or z > SETPOINT_MAX_Z:
                    logerr("z: %+.2f is out of bounds!"%z)
                    is_out_of_bound = True
                if not is_out_of_bound:
                    if len(tokens) >= 6:
                        yaw_degree = float(tokens[5])
                        server.send_to_drone(drone_id, ("set_setpoint_xyzyaw", x, y, z, yaw_degree))
                    else:
                        server.send_to_drone(drone_id, ("set_setpoint_xyz", x, y, z))
            elif tokens[0] in ["set_setpoint_vxvyvz", "sv"]:
                drone_id = to_drone_id(tokens[1])
                vx = float(tokens[2])
                vy = float(tokens[3])
                vz = float(tokens[4])
                # Check xyz bounds.
                SETPOINT_MIN_VX, SETPOINT_MAX_VX = -1, 1
                SETPOINT_MIN_VY, SETPOINT_MAX_VY = -1, 1
                SETPOINT_MIN_VZ, SETPOINT_MAX_VZ = -1, 1
                is_out_of_bound = False
                if vx < SETPOINT_MIN_VX or vx > SETPOINT_MAX_VX:
                    logerr("vx: %+.2f is out of bounds!"%vx)
                    is_out_of_bound = True
                if vy < SETPOINT_MIN_VY or vy > SETPOINT_MAX_VY:
                    logerr("vy: %+.2f is out of bounds!"%vy)
                    is_out_of_bound = True
                if vz < SETPOINT_MIN_VZ or vz > SETPOINT_MAX_VZ:
                    logerr("vz: %+.2f is out of bounds!"%vz)
                    is_out_of_bound = True
                if not is_out_of_bound:
                    server.send_to_drone(drone_id, ("set_setpoint_vxvyvz", vx, vy, vz))
            elif tokens[0] in ["request_read_setpoint", "rrs"]:
                drone_id = to_drone_id(tokens[1])
                server.send_to_drone(drone_id, ("request_read_setpoint"))
            elif tokens[0] in ["trajectory_load", "tl"]:
                drone_id = to_drone_id(tokens[1])
                if drone_id not in trajectory_senders.keys():
                    trajectory_senders[drone_id] = TrajectorySender(server)
                trajectory_senders[drone_id].load(drone_id, tokens[2])
            elif tokens[0] in ["trajectory_run", "tr"]:
                drone_ids = get_drone_ids(tokens[1])
                for id in drone_ids:
                    trajectory_senders[id].run()
            elif tokens[0] in ["trajectory_stop", "ts"]:
                drone_ids = get_drone_ids(tokens[1])
                for id in drone_ids:
                    trajectory_senders[id].stop()
            elif tokens[0] == "ping":
                s, ns = ros_node.node.get_clock().now().seconds_nanoseconds()
                drone_ids = get_drone_ids(tokens[1])
                for id in drone_ids:
                    server.send_to_drone(id, ("ping", s, ns))
            # elif tokens[0] in ["debug_string"]:
            #     drones.send(("debug_string", tokens[1]))
            else:
                logerr("unknown command %s!"%tokens[0])

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
