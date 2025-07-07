////////////////////////////////////////////////////////////////////////////////
//******************************************************************************************************* 
//*******This composable node is responsible to force followers to follow thier leader ********
//******************************************************************************************************* 
// Copyright (c) 2024/12 NTU UAV lab Inc.
// Auther :Morteza Aliyari
// Email: mortezaliyari@gmail.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <std_msgs/msg/string.hpp>
#include <atomic>

# include <px4_msgs/srv/traj_trigger.hpp>
# include <px4_msgs/srv/ready_receive_traj.hpp>
#include "controllers_msg/msg/pid_info.hpp"

# include <std_msgs/msg/bool.hpp>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <queue>

#include "controllers/controllers.hpp"
#include <cmath>            // for std::isnan, etc.
#include <limits>
using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace controllers_msg::msg;


class KeepDist:public rclcpp::Node{
    public:
    KeepDist(const rclcpp::NodeOptions & opt);
    void init(int num_agents);
    ~KeepDist();
    VehicleCommand publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t target_system_id=2);
    OffboardControlMode offboard_control_mode();
    // std::vector<rclcpp::Subscription<Xstatus>::SharedPtr>   initpose_sub_;
    TrajectorySetpoint trajectory_setpoint(float x=0.0, float y=0.0, float z=0.0,
                                            float vx=0.0, float vy=0.0, float vz=0.0, 
                                            float yaw=0.0);
    std::shared_ptr<VehicleOdometry> set_traj_vel;
    double dt,kpx,kix,kdx,kpy,kiy,kdy,kpz,kiz,kdz;

    private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
    std::string controller_name;
    Controllers::PID pid_x_;
    Controllers::PID pid_y_;
    Controllers::PID pid_z_;
    std::atomic<bool> start_following;
    void vehicle_odom_unbiase_callback(const VehicleOdometry & msg);
    void vehicle_odom_unbiase_callback_2(const VehicleOdometry & msg);

    void take_off_done_callback(const std_msgs::msg::Bool msg);
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_publisher_;
    rclcpp::Publisher<PidInfo>::SharedPtr controller_info_publisher_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odom_unbiase_sub_,vehicle_odom_unbiase_sub2_;
    double distance;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr take_off_done_sub_;
    std::shared_ptr<VehicleOdometry> follower_odom_;
};

