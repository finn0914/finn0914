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

#include<plugins/take_off_core.hpp>
TakeOff::TakeOff(const rclcpp::NodeOptions & options):Node("take_off_core", options),takeoff_done{false}

{

        this->declare_parameter<int>("num_agents", 1);
        int num_agents = this->get_parameter("num_agents").as_int();
        init(num_agents);

}
TakeOff::~TakeOff(){

}
void TakeOff::init(int num_agents){
    try{
        traj_trigger_client_ = this->create_client<px4_msgs::srv::TrajTrigger>("/trajectory_trigger");

        this->initialpose_received_.resize(num_agents);
        this->vehicle_pose_.resize(num_agents);
        this->vehicle_status_.resize(num_agents);
        this->counter_unbiase_.resize(num_agents,0);
        this->unbiase_vehicle_pose_.resize(num_agents);
        this->sum_unbiase_.resize(num_agents,0.0);
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);

        std::string topic_name_take_off_done_pub="/px4/all/take_off_done";

        TakeOff::take_off_done_publishers_ = this->create_publisher<std_msgs::msg::Bool>(topic_name_take_off_done_pub, qos);
                                                

        for (int target_system_id = 1; target_system_id <= num_agents; target_system_id++) {
            std::string topic_name_vehicle_odometry = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_odometry";
            std::string topic_name_vehicle_status = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_status";
            std::string topic_name_traj_setpoint = "/px4_" + std::to_string(target_system_id) + "/fmu/in/trajectory_setpoint";
            std::string topic_name_offboard_control_mode = "/px4_" + std::to_string(target_system_id) + "/fmu/in/offboard_control_mode";
            std::string topic_name_unbiase_z = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_odometry_unbiase_z";

            rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_ = this->create_subscription<VehicleOdometry>(
                topic_name_vehicle_odometry,qos, [this, target_system_id](const VehicleOdometry &msg) 
                {this->vehicle_odometry_callback(target_system_id, msg);});
            
            rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
                topic_name_vehicle_status,qos, [this, target_system_id](const VehicleStatus &msg) 
                {this->vehicle_status_callback(target_system_id, msg);});
            rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_setpoint_publisher_ = 
                                                this->create_publisher<TrajectorySetpoint>(topic_name_traj_setpoint, 10);
            rclcpp::Publisher<OffboardControlMode>::SharedPtr  control_mode_publisher_ = 
                                                this->create_publisher<OffboardControlMode>(topic_name_offboard_control_mode, 10);
            rclcpp::Publisher<VehicleOdometry>::SharedPtr  unbiase_z_publisher_ = 
                                                this->create_publisher<VehicleOdometry>(topic_name_unbiase_z, 10);


            this->vehicle_odometry_sub_vect_.push_back(vehicle_odometry_sub_);
            this->take_off_publishers_.push_back(traj_setpoint_publisher_);
            this->control_mode_publishers_.push_back(control_mode_publisher_);
            this->vehicle_status_sub_vect_.push_back(vehicle_status_sub_);
            this->unbiase_z_publishers_.push_back(unbiase_z_publisher_);
        }
        while (rclcpp::ok() && !traj_trigger_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "trajectory_trigger service not available.");
            // return;
        }
        this->take_off_timer_=this->create_wall_timer(100ms,[this,num_agents](){TakeOff::take_off_all(num_agents);});
        RCLCPP_INFO(this->get_logger(), "take_off_timer_ started! ");
        std::thread(&TakeOff::check_takeoff,this).detach();
        RCLCPP_INFO(this->get_logger(), "Detached thread started ! ");


    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());

    }
}
void TakeOff::vehicle_odometry_callback(int target_system_id,const VehicleOdometry & msg){
    try{
        // std::atomic rec=true;
        this->counter_unbiase_[target_system_id-1]++;
        this->initialpose_received_[target_system_id-1]=true;
        this->vehicle_pose_[target_system_id-1]=msg;
        this->unbiase_vehicle_pose_[target_system_id-1]=msg;
        this->unbiase_vehicle_pose_[target_system_id-1].pose_frame = VehicleOdometry::POSE_FRAME_NED; //uint8 NED earth-fixed frame
        // RCLCPP_INFO(this->get_logger(),"drone id is [%d ] , this->counter_unbiase_[target_system_id-1]: %d",target_system_id,this->counter_unbiase_.at(target_system_id-1));
        if (this->counter_unbiase_[target_system_id-1] <= 50){
            this->sum_unbiase_[target_system_id-1]=(this->sum_unbiase_[target_system_id-1]+msg.position[2]);
            this->sum_unbiase_[target_system_id-1]=this->sum_unbiase_[target_system_id-1]/(this->counter_unbiase_[target_system_id-1]);
        }
        this->unbiase_vehicle_pose_[target_system_id-1].position[2]=msg.position[2]-this->sum_unbiase_[target_system_id-1];

        this->unbiase_z_publishers_[target_system_id-1]->publish(this->unbiase_vehicle_pose_[target_system_id-1]);
        
        // RCLCPP_INFO(this->get_logger(),"position of drone with id : %d is x: %f , y: %f, z: %f",target_system_id,msg.position[0],msg.position[1],msg.position[2]);
    }
     catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
void TakeOff::vehicle_status_callback(int target_system_id,const VehicleStatus & msg){
    try{
        // std::atomic rec=true;
        this->vehicle_status_[target_system_id-1]=msg;
        // RCLCPP_INFO(this->get_logger(),"position of drone with id : %d is x: %f , y: %f, z: %f",target_system_id,msg.position[0],msg.position[1],msg.position[2]);
    }
     catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
void TakeOff::take_off_all(const int num_agents){
    try{
        if(!TakeOff::check_arming_mode() || TakeOff::takeoff_done)
            return;
  
        for (int target_system_id = 1; target_system_id <= num_agents; target_system_id++) {
            // RCLCPP_INFO(this->get_logger(), "inside the loop");
            this->control_mode_publishers_[target_system_id-1]->publish(TakeOff::offboard_control_mode());
            rclcpp::sleep_for(std::chrono::milliseconds(10));

            this->take_off_publishers_[target_system_id-1]->publish(TakeOff::trajectory_setpoint(target_system_id+1, target_system_id+1, -1.0*(4.0), 0.0));
            rclcpp::sleep_for(std::chrono::milliseconds(10));

            this->initialpose_received_[target_system_id-1]=false;
            RCLCPP_INFO(this->get_logger(), "Publishing take off command for drone %d", target_system_id);               
        }
        
    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
VehicleCommand TakeOff::publish_vehicle_command(uint16_t command, float param1, float param2, uint8_t target_system_id)
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
}
OffboardControlMode TakeOff::offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    return msg;
}
TrajectorySetpoint TakeOff::trajectory_setpoint(float x, float y, float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    return msg;
}
bool TakeOff::all_values_true(const std::vector<bool>& vec) {
    return std::all_of(vec.begin(), vec.end(), [](bool value) { return value; });
}
bool TakeOff::check_arming_mode() {
    return std::all_of(
        vehicle_status_.begin(),
        vehicle_status_.end(),
        [](const VehicleStatus& agent_status) {
            return agent_status.arming_state == VehicleStatus::ARMING_STATE_ARMED;
        }
    );
}

void TakeOff::check_takeoff(){
    bool check_z_pose{false};
    while(rclcpp::ok()){
        // RCLCPP_INFO(this->get_logger(), "vehicle_pose_.size()! : %zu",vehicle_pose_.size() );

        check_z_pose=std::all_of(vehicle_pose_.begin(), vehicle_pose_.end(), [this](VehicleOdometry &pose) 
        { 
            RCLCPP_INFO(get_logger(), "pose.position[2]! : %f",pose.position[2] );

            return pose.position[2]<-3;});
        rclcpp::sleep_for(std::chrono::seconds(2));
        if(check_z_pose)
            break;
    }


    // Create the request
    auto request = std::make_shared<px4_msgs::srv::TrajTrigger::Request>();
    request->traj_trigger = true;

    // Send the request asynchronously with a callback -->lambda function here
    traj_trigger_client_->async_send_request(request, 
        [this,check_z_pose](rclcpp::Client<px4_msgs::srv::TrajTrigger>::SharedFuture future) {
            try {
                auto response = future.get();
                if (response->success) {
                    RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
                    // both below condition is needed since sometimes timer can't be cancelled!
                    rclcpp::sleep_for(std::chrono::milliseconds(2000));
                    std_msgs::msg::Bool done;
                    done.data=check_z_pose;
                    TakeOff::take_off_done_publishers_->publish(done);
                    TakeOff::takeoff_done.store(check_z_pose);
                    this->take_off_timer_->cancel(); 
                    RCLCPP_INFO(this->get_logger(), "take_off_timer_ cancelled!");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Service request failed: %s", response->message.c_str());
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
            }
        });

    RCLCPP_INFO(this->get_logger(), "Detached thread finished!");

};