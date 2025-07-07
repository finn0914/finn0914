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
//K)
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

#include<plugins/arm_disarm_core.hpp>

ARMDISARM::ARMDISARM(const rclcpp::NodeOptions & options):Node("arm_disarm_core", options)
{

        this->declare_parameter<int>("num_agents", 1);
        int num_agents = this->get_parameter("num_agents").as_int();
        std::string last_px4_service_name= "/px4_"+std::to_string(num_agents)+"/fmu/vehicle_command";
        last_px4_service_ = this->create_client<px4_msgs::srv::VehicleCommand>(last_px4_service_name);
        // int i =0;
        while (rclcpp::ok() && !last_px4_service_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "last_px4_service not available.");
            // return;
        }
        this->init(num_agents);

        
        this->arm_all_timer_=this->create_wall_timer(3000ms,[this,num_agents](){this->arm_all(num_agents);});


}
ARMDISARM::~ARMDISARM(){

}
void ARMDISARM::init(int num_agents){
    try{
        
        this->vehicle_status_.resize(num_agents);
        this->vehicle_status_received_.resize(num_agents);
        this->vehicle_status_received_.resize(num_agents);
        for (auto& status : this->vehicle_status_received_) {
            status = std::make_shared<std::atomic<bool>>(false);
        }
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
        for (int target_system_id = 1; target_system_id <= num_agents; target_system_id++) {
            std::string topic_name_vehicle_status = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_status";
            std::string topic_name_vehicle_command = "/px4_" + std::to_string(target_system_id) + "/fmu/in/vehicle_command";
            std::string topic_name_offboard_control_mode = "/px4_" + std::to_string(target_system_id) + "/fmu/in/offboard_control_mode";
            
            rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
                topic_name_vehicle_status,qos, [this, target_system_id](const VehicleStatus &msg) 
                {this->vehicle_status_callback(target_system_id, msg);});
            rclcpp::Publisher<VehicleCommand>::SharedPtr  vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(topic_name_vehicle_command, 10);
            rclcpp::Publisher<OffboardControlMode>::SharedPtr  control_mode_publisher_ = this->create_publisher<OffboardControlMode>(topic_name_offboard_control_mode, 10);
           
            this->arm_disarm_publshers_.push_back(vehicle_command_publisher_);
            this->control_mode_publishers_.push_back(control_mode_publisher_);
            this->vehicle_status_sub_vect_.push_back(vehicle_status_sub_);
        }
        RCLCPP_INFO(this->get_logger(),"initialized!");
    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
void ARMDISARM::arm_all(int num_agents) {
    try {
        if (!std::all_of(vehicle_status_received_.begin(), vehicle_status_received_.end(),
                        [](const std::shared_ptr<std::atomic<bool>>& value) { return value->load(); })) {
            RCLCPP_WARN_STREAM(this->get_logger(), "Not received all topics");
                // rclcpp::sleep_for(std::chrono::seconds(1));
                return;
        }
        

        for (int target_system_id = 1; target_system_id <= num_agents; target_system_id++) {
            int retry_count = 0;
            const int max_retries = 50;

            while (rclcpp::ok() && !ARMDISARM::check_arming_mode(target_system_id)) {
                if (++retry_count > max_retries) {
                    RCLCPP_WARN(this->get_logger(), "Failed to arm drone %d after %d retries", target_system_id, max_retries);
                    break;
                }

                this->control_mode_publishers_[target_system_id - 1]->publish(ARMDISARM::publish_offboard_control_mode());
                // rclcpp::sleep_for(std::chrono::milliseconds(10));

                this->arm_disarm_publshers_[target_system_id - 1]->publish(ARMDISARM::publish_vehicle_command(
                    VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6, target_system_id+1));
                // rclcpp::sleep_for(std::chrono::milliseconds(10));

                this->arm_disarm_publshers_[target_system_id - 1]->publish(ARMDISARM::publish_vehicle_command(
                    VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, target_system_id+1));
                RCLCPP_INFO(this->get_logger(), "Publishing command for drone %d", target_system_id);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Arm command sent to all");
        this->arm_all_timer_->cancel();
    } catch (std::exception& ex) {
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}








VehicleCommand ARMDISARM::publish_vehicle_command(uint16_t command, float param1, float param2, uint8_t target_system_id)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = target_system_id;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    return msg;

	// vehicle_command_publisher_->publish(msg);
}
OffboardControlMode ARMDISARM::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	// offboard_control_mode_publisher_->publish(msg);
    return msg;
}
bool ARMDISARM::check_arming_mode(int target_system_id) {
    RCLCPP_INFO(this->get_logger(), "size of vehicle_status_ is %zu", vehicle_status_.size());
    if (vehicle_status_.empty())
        return false;

    // for (size_t i = 1; i <= this->vehicle_status_.size(); i++) 

    RCLCPP_INFO(this->get_logger(),"drone with id {%d} has ARMING_STATE: %d",target_system_id, this->vehicle_status_[target_system_id-1].arming_state);

    if(vehicle_status_[target_system_id-1].arming_state==VehicleStatus::ARMING_STATE_ARMED){
        RCLCPP_INFO(this->get_logger(),"drone with id {%d} is active and ARMING_STATE: %d",target_system_id, this->vehicle_status_[target_system_id-1].arming_state);
        this->vehicle_status_.clear();
        return true;
    }
    else{
        RCLCPP_INFO(this->get_logger(),"drone with id {%d} is deactive and ARMING_STATE: %d",target_system_id, this->vehicle_status_[target_system_id-1].arming_state);
        this->vehicle_status_.clear();
        return false;
    }
    
    

}
void ARMDISARM::vehicle_status_callback(int target_system_id,const VehicleStatus & msg){
    try{
        // std::atomic rec=true;
        *this->vehicle_status_received_[target_system_id - 1] = true;
        this->vehicle_status_[target_system_id-1]=msg;
        // RCLCPP_INFO(this->get_logger(),"vehicle_status_callback of drone with id {%d} is active and ARMING_STATE: %d",target_system_id, msg.arming_state);
    }
     catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}