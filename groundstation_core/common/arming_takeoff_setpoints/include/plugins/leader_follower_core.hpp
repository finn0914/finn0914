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

# include <px4_msgs/srv/traj_trigger.hpp>
# include <px4_msgs/srv/ready_receive_traj.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <unordered_map>
#include <mutex>
#include <queue>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;



class LeaderFollower:public rclcpp::Node{
    public:
    LeaderFollower(const rclcpp::NodeOptions & opt);
    void init(int num_agents);
    ~LeaderFollower();
    VehicleCommand publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t target_system_id=2);
    OffboardControlMode offboard_control_mode(int target_system_id);
    // std::vector<rclcpp::Subscription<Xstatus>::SharedPtr>   initpose_sub_;
    TrajectorySetpoint trajectory_setpoint(float x=0.0, float y=0.0, float z=0.0,
                                            float vx=0.0, float vy=0.0, float vz=0.0, 
                                            float yaw=0.0);

    private:
    void vehicle_odometry_callback(int target_system_id,const VehicleOdometry & msg);
    void unbiase_vehicle_odometry_callback(int target_system_id,const VehicleOdometry & msg);
    void leader_fol_traj_callback(int target_system_id,const TrajectorySetpoint & msg);

    void vehicle_status_callback(int target_system_id,const VehicleStatus & msg);
    void handle_traj_trigger_request(const std::shared_ptr<px4_msgs::srv::TrajTrigger::Request> request,
    std::shared_ptr<px4_msgs::srv::TrajTrigger::Response> response,int num_agents);

    bool all_values_true(const std::vector<bool>& vec);
    std::atomic<bool> traj_start,ready_receive_traj;
    std::vector<bool> initialpose_received_;
    std::vector<bool> leader_fol_traj_rec_;
    std::vector<VehicleStatus> vehicle_status_;
    std::vector<VehicleOdometry> vehicle_pose_;
    
    std::shared_ptr<std::unordered_map<int,std::shared_ptr<std::queue<VehicleOdometry>>>> leader_fol_traj_setpoint_;
    std::vector<rclcpp::Subscription<VehicleOdometry>::SharedPtr> vehicle_odometry_sub_vect_;
    std::vector<rclcpp::Subscription<VehicleOdometry>::SharedPtr> unbiase_vehicle_odometry_sub_vect_;
    std::vector<rclcpp::Subscription<TrajectorySetpoint>::SharedPtr> leader_follower_traj_setpoint_sub_vect_;//leader_follower

    std::vector<rclcpp::Subscription<VehicleStatus>::SharedPtr> vehicle_status_sub_vect_;
    std::vector<rclcpp::Publisher<VehicleCommand>::SharedPtr> arm_disarm_publshers_;

    std::vector<rclcpp::Publisher<TrajectorySetpoint>::SharedPtr> follow_traj_publishers_;
    std::vector<rclcpp::Publisher<OffboardControlMode>::SharedPtr> control_mode_publishers_;
    rclcpp::Service<px4_msgs::srv::TrajTrigger>::SharedPtr traj_trigger_service_;
    rclcpp::Client<px4_msgs::srv::ReadyReceiveTraj>::SharedPtr ready_rec_traj_client_;

    void follow_traj(int num_agents);
    std::shared_ptr<std::vector<VehicleOdometry>> generate_circle_trajectory(const VehicleOdometry& start_pose, double radius, int num_points);
    rclcpp::TimerBase::SharedPtr traj_timer_;
    bool is_inside_sphere(VehicleOdometry &drone_pose, VehicleOdometry &traj, double radius);

    std::vector<std::unique_ptr<std::mutex>> element_mutexes_;
    std::shared_ptr<double> sphare_radius;

};

// Register as a component
RCLCPP_COMPONENTS_REGISTER_NODE(LeaderFollower)