////////////////////////////////////////////////////////////////////////////////
//
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

////////////////////////////////////////////////////////////////////////////////
//
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
#include<std_msgs/msg/bool.hpp>

#include <atomic>
# include <px4_msgs/srv/traj_trigger.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;



class TakeOff:public rclcpp::Node{
    public:
    TakeOff(const rclcpp::NodeOptions & opt);
    void init(int num_agents);
    ~TakeOff();
    void take_off_all(int num_agents);
    VehicleCommand publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t target_system_id=2);
    OffboardControlMode offboard_control_mode();
    TrajectorySetpoint trajectory_setpoint(float x, float y, float z, float yaw);
    // std::vector<rclcpp::Subscription<Xstatus>::SharedPtr>   initpose_sub_;

    private:
    void vehicle_odometry_callback(int target_system_id,const VehicleOdometry & msg);
    void vehicle_status_callback(int target_system_id,const VehicleStatus & msg);
    bool check_arming_mode();
    void check_takeoff();
    bool all_values_true(const std::vector<bool>& vec);
    std::atomic<bool> takeoff_done;
    std::vector<bool> initialpose_received_;
    std::vector<VehicleStatus> vehicle_status_;
    std::vector<VehicleOdometry> vehicle_pose_,unbiase_vehicle_pose_;
    std::vector<rclcpp::Subscription<VehicleOdometry>::SharedPtr> vehicle_odometry_sub_vect_;
    std::vector<rclcpp::Subscription<VehicleStatus>::SharedPtr> vehicle_status_sub_vect_;

    std::vector<rclcpp::Publisher<TrajectorySetpoint>::SharedPtr> take_off_publishers_;
    std::vector<rclcpp::Publisher<OffboardControlMode>::SharedPtr> control_mode_publishers_;
    std::vector<rclcpp::Publisher<VehicleOdometry>::SharedPtr> unbiase_z_publishers_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr take_off_done_publishers_ ;
    rclcpp::TimerBase::SharedPtr take_off_timer_;
    rclcpp::Client<px4_msgs::srv::TrajTrigger>::SharedPtr traj_trigger_client_;
    
    std::vector<int> counter_unbiase_;
    std::vector<float> sum_unbiase_;


};

// Register as a component
RCLCPP_COMPONENTS_REGISTER_NODE(TakeOff)