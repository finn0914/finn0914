////////////////////////////////////////////////////////////////////////////////
//******************************************************************************************************* 
//*******This composable node is responsible to generate trajectory for leader********
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
#include "rclcpp_components/register_node_macro.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include<std_msgs/msg/string.hpp>
#include <atomic>

# include <px4_msgs/srv/ready_receive_traj.hpp>
#include <memory>
#include <unordered_map>
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
// using SendSetpointXyz = stationui_frontend_rviz_plugin_msgs::msg::SendSetpointXyz;
using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;

class PlanningForLeader:public rclcpp::Node {

    public:
    PlanningForLeader(const rclcpp::NodeOptions & options);
    ~PlanningForLeader();
    void init_commons();
    void init_algorithm();
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_publisher_sim_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_publisher_real_drone101_xyz_,setpoint_publisher_real_drone101_xyzyaw_,setpoint_publisher_real_drone101_, setpoint_publisher_real_drone102_,setpoint_publisher_real_drone103_;
    private:
            float sum_sin_               = 0.0f;
        float sum_cos_               = 0.0f;
        unsigned counter_yaw_filt_   = 0u;
        float  yaw_radian_drone101_  = 0.0f;   // filtered instantaneous yaw
        float  yaw_radian_drone101_avg_ = 0.0f; // 20-sample circular mean

    double yaw_radian_drone101_sum=0;
    bool is_thread_publish_generated_setpoints_drone101_active, is_thread_publish_generated_setpoints_drone102_active, is_thread_publish_generated_setpoints_drone103_active;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr pose_subscription_real_drone101_;

    std::string drone101_path_shape,drone102_path_shape,drone103_path_shape,sim_real_;
    int drone101_resolution,drone102_resolution,drone103_resolution, setpoint_delay;
    double drone101_ratio,drone102_ratio,drone103_ratio;
    std::vector<int64_t> drone101_only_line,drone102_only_line,drone103_only_line; 
    std::vector<double> drone101_start_pose,drone102_start_pose,drone103_start_pose;


    std::atomic<bool> ready_send_traj;
    rclcpp::Client<px4_msgs::srv::ReadyReceiveTraj>::SharedPtr ready_send_traj_client_;
    std::shared_ptr<std::vector<VehicleOdometry>> generate_circle_trajectory(const VehicleOdometry& start_pose, double radius, int num_points,std::vector<int64_t> only_line);
    std::shared_ptr<std::vector<VehicleOdometry>> generate_square_trajectory(const VehicleOdometry& start_pose, double radius, int num_points,std::vector<int64_t> only_line) ;

    void publish_generated_setpoints_drone101();
    void publish_generated_setpoints_drone102();
    void publish_generated_setpoints_drone103();
    rclcpp::Service<px4_msgs::srv::ReadyReceiveTraj>::SharedPtr ready_send_traj_service_;
    void handle_traj_request(const std::shared_ptr<px4_msgs::srv::ReadyReceiveTraj::Request> request,
    std::shared_ptr<px4_msgs::srv::ReadyReceiveTraj::Response> response);
    bool drone101_yaw_align_velocity;


    void on_recv_pose_drone101(const VehicleOdometry& msg);



};
RCLCPP_COMPONENTS_REGISTER_NODE(PlanningForLeader)