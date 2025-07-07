////////////////////////////////////////////////////////////////////////////////
//******************************************************************************************************* 
//*******This composable node is responsible to force agents to follow distinguished trajectories********
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

#include<plugins/follow_traj_core.hpp>

FollowTraj::FollowTraj(const rclcpp::NodeOptions & options):Node("follow_traj_core", options),
traj_start{false}

{

        this->declare_parameter<int>("num_agents", 1);
        int num_agents = this->get_parameter("num_agents").as_int();
        init(num_agents);

}
FollowTraj::~FollowTraj(){

}
void FollowTraj::init(int num_agents){
    try{

        traj_trigger_service_ = this->create_service<px4_msgs::srv::TrajTrigger>(
            "trajectory_trigger",
            [this,num_agents](const std::shared_ptr<px4_msgs::srv::TrajTrigger::Request> request,
                   std::shared_ptr<px4_msgs::srv::TrajTrigger::Response> response) {
                this->handle_traj_trigger_request(request, response,num_agents);
            }
        );
        // RCLCPP_INFO(this->get_logger(), "0! ");

        this->initialpose_received_.resize(num_agents);
        this->vehicle_pose_.resize(num_agents);
        this->vehicle_status_.resize(num_agents);
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
        for (int target_system_id = 1; target_system_id <= num_agents; target_system_id++) {
            std::string topic_name_unbiase_vehicle_odometry = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_odometry_unbiase_z";
            std::string topic_name_vehicle_odometry = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_odometry";
            std::string topic_name_vehicle_status = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_status";
            std::string topic_name_vehicle_command = "/px4_" + std::to_string(target_system_id) + "/fmu/in/vehicle_command";
            std::string topic_name_traj_setpoint = "/px4_" + std::to_string(target_system_id) + "/fmu/in/trajectory_setpoint";
            std::string topic_name_offboard_control_mode = "/px4_" + std::to_string(target_system_id) + "/fmu/in/offboard_control_mode";
            
            rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
                topic_name_vehicle_status,qos, [this, target_system_id](const VehicleStatus &msg) 
                {this->vehicle_status_callback(target_system_id, msg);});
            rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_ = this->create_subscription<VehicleOdometry>(
                topic_name_vehicle_odometry,qos, [this, target_system_id](const VehicleOdometry &msg) 
                {this->vehicle_odometry_callback(target_system_id, msg);});
            rclcpp::Subscription<VehicleOdometry>::SharedPtr unbiase_vehicle_odometry_sub_ = this->create_subscription<VehicleOdometry>(
                topic_name_unbiase_vehicle_odometry,qos, [this, target_system_id](const VehicleOdometry &msg) 
                {this->unbiase_vehicle_odometry_callback(target_system_id, msg);});
            rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_setpoint_publisher_ = 
                                                this->create_publisher<TrajectorySetpoint>(topic_name_traj_setpoint, 50);
            rclcpp::Publisher<OffboardControlMode>::SharedPtr  control_mode_publisher_ = 
                                                this->create_publisher<OffboardControlMode>(topic_name_offboard_control_mode, 50);

            rclcpp::Publisher<VehicleCommand>::SharedPtr  vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(topic_name_vehicle_command, 50);

            vehicle_odometry_sub_vect_.push_back(vehicle_odometry_sub_);
            follow_traj_publishers_.push_back(traj_setpoint_publisher_);
            control_mode_publishers_.push_back(control_mode_publisher_);
            vehicle_status_sub_vect_.push_back(vehicle_status_sub_);
            arm_disarm_publshers_.push_back(vehicle_command_publisher_);
            unbiase_vehicle_odometry_sub_vect_.push_back(unbiase_vehicle_odometry_sub_);
        }
        std::thread([this, num_agents]() { this->follow_traj(num_agents); }).detach();
    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());

    }
}
void FollowTraj::handle_traj_trigger_request(const std::shared_ptr<px4_msgs::srv::TrajTrigger::Request> request,
                                std::shared_ptr<px4_msgs::srv::TrajTrigger::Response> response, int num_agents){
    try{
        if (request->traj_trigger) {
            RCLCPP_INFO(this->get_logger(), " handle_traj_trigger_request! excuted");
            this->traj_start=true;
            response->success = true;
            response->message = "Requested to stop takeoff mode from trajectory server.";
            RCLCPP_INFO(this->get_logger(), response->message.c_str());
            RCLCPP_INFO(this->get_logger(), "tracking mode is active now!");

        }
        else {
            response->success = false;
            response->message = "Request to stop takeoff mode failed from trajectory server.";
            RCLCPP_WARN(this->get_logger(), response->message.c_str());
        }
    }
    
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }

}
void FollowTraj::vehicle_odometry_callback(int target_system_id,const VehicleOdometry & msg){
    try{
        // std::atomic rec=true;
        // this->initialpose_received_[target_system_id-1]=true;
        // this->vehicle_pose_[target_system_id-1]=msg;
        RCLCPP_INFO(this->get_logger(),"position of drone with id : %d is x: %f , y: %f, z: %f",target_system_id,msg.position[0],msg.position[1],msg.position[2]);
    }
     catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
void FollowTraj::unbiase_vehicle_odometry_callback(int target_system_id,const VehicleOdometry & msg){
    try{
        // std::atomic rec=true;
        this->initialpose_received_[target_system_id-1]=true;
        this->vehicle_pose_[target_system_id-1]=msg;
        RCLCPP_INFO(this->get_logger(),"FollowTraj unbiase position of drone with id : %d is x: %f , y: %f, z: %f",target_system_id,msg.position[0],msg.position[1],msg.position[2]);
    }
     catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
void FollowTraj::vehicle_status_callback(int target_system_id,const VehicleStatus & msg){
    try{
        // std::atomic rec=true;
        this->vehicle_status_[target_system_id-1]=msg;
        // RCLCPP_INFO(this->get_logger(),"position of drone with id : %d is x: %f , y: %f, z: %f",target_system_id,msg.position[0],msg.position[1],msg.position[2]);
    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}

VehicleCommand FollowTraj::publish_vehicle_command(uint16_t command, float param1, float param2, uint8_t target_system_id){
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
OffboardControlMode FollowTraj::offboard_control_mode(){
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    return msg;
}
TrajectorySetpoint FollowTraj::trajectory_setpoint_pose_yaw(float x, float y, float z, float yaw)
{
	TrajectorySetpoint msg{};
	msg.position = {x, y, z};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    return msg;
}
TrajectorySetpoint FollowTraj::trajectory_setpoint_velocity_yaw(float vx, float vy, float vz, float yaw)
{
	TrajectorySetpoint msg{};
	msg.velocity = {vx, vy, vz};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    return msg;
}
bool FollowTraj::all_values_true(const std::vector<bool>& vec){
    return std::all_of(vec.begin(), vec.end(), [](bool value) { return value; });
}

bool FollowTraj::is_inside_sphere(VehicleOdometry &drone_pose, VehicleOdometry &traj, double radius) {
    try{
        double distance = std::sqrt(
            std::pow(drone_pose.position[0] - traj.position[0], 2) +
            std::pow(drone_pose.position[1] - traj.position[1], 2) +
            std::pow(drone_pose.position[2] - traj.position[2], 2)
        );

        // Check if the distance is within the radius
        // RCLCPP_INFO(this->get_logger(),"distance  %f",distance);

        return distance <= radius;
    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
void FollowTraj::follow_traj(int num_agents){

    try{
            while(!FollowTraj::traj_start){

                rclcpp::sleep_for(std::chrono::milliseconds(1000));
                RCLCPP_WARN(this->get_logger(), "trajectory is waiting for service respond"); 

            }
            std::vector<int> point_counter;
            std::vector<bool> traj_end;
            traj_end.resize(num_agents);
            point_counter.resize(num_agents);
            float radius=0.5;
            // traj_end={false,false,false};
            // std::shared_ptr<std::vector<VehicleOdometry>> traj_id1=std::make_shared<std::vector<VehicleOdometry>>();
            // std::shared_ptr<std::vector<VehicleOdometry>> traj_id2=std::make_shared<std::vector<VehicleOdometry>>();
            // std::shared_ptr<std::vector<VehicleOdometry>> traj_id3=std::make_shared<std::vector<VehicleOdometry>>();

            VehicleOdometry start_pose1,start_pose2,start_pose3,land_pose1,land_pose2,land_pose3;
            start_pose1.position={1.0,1.0,-6.0};
            start_pose2.position={2.0,2.0,-8.0};
            start_pose3.position={3.0,3.0,-10.0};

            // traj_id1->push_back(start_pose1);
            // traj_id2->push_back(start_pose2);
            // traj_id3->push_back(pose3);

            std::unordered_map<int, std::shared_ptr<std::vector<VehicleOdometry>>> drone_trajectories;
            // drone_trajectories[1]=traj_id1;
            // drone_trajectories[2]=traj_id2;
            // drone_trajectories[3]=traj_id3;

            drone_trajectories[1] = FollowTraj::generate_circle_trajectory(start_pose1, 10, 20);
            drone_trajectories[2] = FollowTraj::generate_circle_trajectory(start_pose2, 10, 20);
            drone_trajectories[3] = FollowTraj::generate_circle_trajectory(start_pose3, 10, 20);
            land_pose1.position={drone_trajectories[1]->back().position[0],
                                  drone_trajectories[1]->back().position[1],
                                  -0.1};
            land_pose2.position={drone_trajectories[2]->back().position[0],
                                  drone_trajectories[2]->back().position[1],
                                -0.1};
            land_pose3.position={drone_trajectories[3]->back().position[0],
                                  drone_trajectories[3]->back().position[1],
                                -0.1};

            drone_trajectories[1]->push_back(land_pose1);
            drone_trajectories[2]->push_back(land_pose2);
            drone_trajectories[3]->push_back(land_pose3);

            while (rclcpp::ok()){

                for (int target_system_id = 1; target_system_id <= num_agents; target_system_id++) {
                    if(traj_end[target_system_id-1]){

                        // RCLCPP_INFO(this->get_logger(), "End of traj for drone with id [%d]",target_system_id); 
                        continue;
                    }
                    std::shared_ptr<std::vector<VehicleOdometry>> trajectory_id=drone_trajectories[target_system_id];

                    // RCLCPP_INFO(this->get_logger(), "drone with id [%d] , traj size [%zu], point_counter size[%zu],point_counter[%d] ", target_system_id,trajectory_id->size(),point_counter.size(),point_counter[target_system_id-1]); 

                    // RCLCPP_INFO(this->get_logger(), "drone with id [%d] , goal.x: [%f],goal.y: [%f] ", target_system_id,trajectory_id->at(point_counter[target_system_id-1]).position[0],trajectory_id->at(point_counter[target_system_id-1]).position[1]); 


                    this->control_mode_publishers_[target_system_id-1]->
                                                publish(FollowTraj::offboard_control_mode());
                    this->follow_traj_publishers_[target_system_id-1]->
                                                publish(FollowTraj::trajectory_setpoint_pose_yaw(trajectory_id->at(point_counter[target_system_id-1]).position[0],
                                                                                        trajectory_id->at(point_counter[target_system_id-1]).position[1], 
                                                                                        trajectory_id->at(point_counter[target_system_id-1]).position[2], 0.0));
                    // RCLCPP_INFO(this->get_logger(), "Publishing trajectory to drone %d", target_system_id); 

                    if(is_inside_sphere(this->vehicle_pose_[target_system_id-1],trajectory_id->at(point_counter[target_system_id-1]),radius)) {
                        point_counter[target_system_id-1]++;
                        // RCLCPP_INFO(this->get_logger(), "drone with id [%d] reached to goal point ", target_system_id); 
                        if(point_counter[target_system_id-1]>=drone_trajectories[target_system_id]->size()){

                            // point_counter[target_system_id-1]=0;
                            point_counter[target_system_id-1]--;
                            traj_end[target_system_id-1]=true;
                            // RCLCPP_INFO(this->get_logger(), "End of traj for drone with id [%d] inside last if cond",target_system_id,trajectory_id->size()); 
                        }

                    }
                }

                if(FollowTraj::all_values_true(traj_end)){
                    RCLCPP_INFO(this->get_logger(), "**********LANDING Mode***********");               
                    break;
                }
                
            }
    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }

}
// Function to generate a circular trajectory
std::shared_ptr<std::vector<VehicleOdometry>> FollowTraj::generate_circle_trajectory(const VehicleOdometry& start_pose, double radius, int num_points) {
    auto trajectory = std::make_shared<std::vector<VehicleOdometry>>();

    // Generate `num_points` along the circle
    for (int i = 0; i < num_points; ++i) {
        double angle = 2 * M_PI * i / num_points; // Angle for the current point
        VehicleOdometry point;
        point.position = {
            start_pose.position[0] + radius * cos(angle), // x = center_x + r * cos(theta)
            start_pose.position[1] + radius * sin(angle), // y = center_y + r * sin(theta)
            start_pose.position[2]                        // z remains constant
        };
        trajectory->push_back(point);
    }
    // VehicleOdometry point;
    // point.position = {start_pose.position[0],start_pose.position[1],0.0};
    // trajectory->push_back(point);
    return trajectory;
}