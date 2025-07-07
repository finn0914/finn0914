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
#include <px4_msgs/msg/vehicle_status.hpp>

#include <stdint.h>

#include <chrono>
#include <iostream>
#include<std_msgs/msg/string.hpp>
# include <px4_msgs/srv/vehicle_command.hpp>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;



class ARMDISARM:public rclcpp::Node{
    public:
    ARMDISARM(const rclcpp::NodeOptions & opt);
    ~ARMDISARM();
    void arm_all(int num_agents);
    VehicleCommand publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, uint8_t target_system_id=2);
    OffboardControlMode publish_offboard_control_mode();
    // TrajectorySetpoint ARMDISARM::publish_trajectory_setpoint(float x, float y, float z, float yaw);
    bool check_arming_mode(int target_system_id) ;
    void init(int num_agents);
    void vehicle_status_callback(int target_system_id,const VehicleStatus & msg);

    private:
        std::vector<rclcpp::Publisher<VehicleCommand>::SharedPtr> arm_disarm_publshers_;
        std::vector<rclcpp::Publisher<OffboardControlMode>::SharedPtr> control_mode_publishers_;
        std::vector<rclcpp::Subscription<VehicleStatus>::SharedPtr> vehicle_status_sub_vect_;

        std::vector<rclcpp::TimerBase::SharedPtr> arm_disarm_timer_;
        std::vector<VehicleStatus> vehicle_status_;
        std::vector<std::shared_ptr<std::atomic<bool>>> vehicle_status_received_;
        rclcpp::TimerBase::SharedPtr arm_all_timer_;
        rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr last_px4_service_;

};

// Register as a component
RCLCPP_COMPONENTS_REGISTER_NODE(ARMDISARM)