////////////////////////////////////////////////////////////////////////////////
//******************************************************************************************************* 
//*******This composable node is responsible to force followers to follow thier leader ********
//******************************************************************************************************* 
// Copyright (c) 2024/12 NTU UAV lab Inc.
// Auther :Morteza Aliyari
// Email: mortezaliyari@gmail.com
//
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

#include<plugins/leader_follower_core.hpp>

LeaderFollower::LeaderFollower(const rclcpp::NodeOptions & options):Node("leader_follower_core", options),
traj_start{false},ready_receive_traj{false}

{

        this->declare_parameter<int>("num_agents", 1);
        int num_agents = this->get_parameter("num_agents").as_int();
        this->declare_parameter<double>("sphare_radius", 0.4);
        sphare_radius = std::make_shared<double>(this->get_parameter("sphare_radius").as_double());
        RCLCPP_WARN(this->get_logger(),"Yaml params: num_agents [%d], sphare_radius [%f]"
        ,num_agents,sphare_radius);

        init(num_agents);

}
LeaderFollower::~LeaderFollower(){

}
void LeaderFollower::init(int num_agents){
    try{

        this->initialpose_received_.resize(num_agents);
        this->leader_fol_traj_rec_.resize(num_agents);
        this->vehicle_pose_.resize(num_agents);
        this->vehicle_status_.resize(num_agents);
        this->element_mutexes_.resize(num_agents);
        for (size_t i = 0; i < num_agents; ++i) {
            this->element_mutexes_[i] = std::make_unique<std::mutex>();
        } 
               
        LeaderFollower::leader_fol_traj_setpoint_=std::make_shared<std::unordered_map<int, std::shared_ptr<std::queue<VehicleOdometry>>>>();

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        rmw_qos_profile_t qos_profile_traj = rmw_qos_profile_parameters;
        auto qos_traj = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_traj.history, 10), qos_profile_traj);
        LeaderFollower::traj_trigger_service_ = this->create_service<px4_msgs::srv::TrajTrigger>(
            "trajectory_trigger",
            [this,num_agents](const std::shared_ptr<px4_msgs::srv::TrajTrigger::Request> request,
                   std::shared_ptr<px4_msgs::srv::TrajTrigger::Response> response) {
                this->handle_traj_trigger_request(request, response,num_agents);
            }
        );
        LeaderFollower::ready_rec_traj_client_ = this->create_client<px4_msgs::srv::ReadyReceiveTraj>("/ready_run_gen_traj");

        while (rclcpp::ok() && !LeaderFollower::ready_rec_traj_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "ready_run_gen_traj service not available.");
        }


		
        for (int target_system_id = 1; target_system_id <= num_agents; target_system_id++) {
            std::string topic_name_unbiase_vehicle_odometry = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_odometry_unbiase_z";
            std::string topic_name_vehicle_odometry = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_odometry";
            std::string topic_name_vehicle_status = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_status";
            std::string topic_name_vehicle_command = "/px4_" + std::to_string(target_system_id) + "/fmu/in/vehicle_command";
            std::string topic_name_traj_setpoint = "/px4_" + std::to_string(target_system_id) + "/fmu/in/trajectory_setpoint";
            std::string topic_name_offboard_control_mode = "/px4_" + std::to_string(target_system_id) + "/fmu/in/offboard_control_mode";
            // std::string topic_name_leader_fol_traj = "/px4_" + std::to_string(target_system_id) + "/leader_follower/trajectory_setpoint";
            std::string topic_name_leader_fol_traj = "/rviz/drone10" + std::to_string(target_system_id) + "/send_setpoint_xyzyaw";

            rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
                topic_name_vehicle_status,qos, [this, target_system_id](const VehicleStatus &msg) 
                {this->vehicle_status_callback(target_system_id, msg);});
            // rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_ = this->create_subscription<VehicleOdometry>(
            //     topic_name_vehicle_odometry,qos, [this, target_system_id](const VehicleOdometry &msg) 
            //     {this->vehicle_odometry_callback(target_system_id, msg);});
            rclcpp::Subscription<VehicleOdometry>::SharedPtr unbiase_vehicle_odometry_sub_ = this->create_subscription<VehicleOdometry>(
                topic_name_unbiase_vehicle_odometry,qos, [this, target_system_id](const VehicleOdometry &msg) 
                {this->unbiase_vehicle_odometry_callback(target_system_id, msg);});
            rclcpp::Subscription<TrajectorySetpoint>::SharedPtr leader_follower_traj_sub_ = this->create_subscription<TrajectorySetpoint>(
                topic_name_leader_fol_traj,qos, [this, target_system_id](const TrajectorySetpoint &msg) 
                {this->leader_fol_traj_callback(target_system_id, msg);});
            rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_setpoint_publisher_ = 
                                                this->create_publisher<TrajectorySetpoint>(topic_name_traj_setpoint, 50);
            rclcpp::Publisher<OffboardControlMode>::SharedPtr  control_mode_publisher_ = 
                                                this->create_publisher<OffboardControlMode>(topic_name_offboard_control_mode, 50);

            rclcpp::Publisher<VehicleCommand>::SharedPtr  vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(topic_name_vehicle_command, 50);

            // vehicle_odometry_sub_vect_.push_back(vehicle_odometry_sub_);
            follow_traj_publishers_.push_back(traj_setpoint_publisher_);
            control_mode_publishers_.push_back(control_mode_publisher_);
            vehicle_status_sub_vect_.push_back(vehicle_status_sub_);
            arm_disarm_publshers_.push_back(vehicle_command_publisher_);
            unbiase_vehicle_odometry_sub_vect_.push_back(unbiase_vehicle_odometry_sub_);
            leader_follower_traj_setpoint_sub_vect_.push_back(leader_follower_traj_sub_);

            if ((*leader_fol_traj_setpoint_).find(target_system_id) == (*leader_fol_traj_setpoint_).end() ||
            (*leader_fol_traj_setpoint_)[target_system_id] == nullptr) {
            (*leader_fol_traj_setpoint_)[target_system_id] = std::make_shared<std::queue<VehicleOdometry>>();
        }
        }
        std::thread([this, num_agents]() { this->follow_traj(num_agents); }).detach();
    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());

    }
}
void LeaderFollower::handle_traj_trigger_request(const std::shared_ptr<px4_msgs::srv::TrajTrigger::Request> request,
                                std::shared_ptr<px4_msgs::srv::TrajTrigger::Response> response, int num_agents){
    try{
        if (request->traj_trigger) {
            RCLCPP_INFO(this->get_logger(), " handle_traj_trigger_request! excuted");
            this->traj_start=true;
                {
                    auto request_planning = std::make_shared<px4_msgs::srv::ReadyReceiveTraj::Request>();
                    request_planning->ready_receive_traj[0] = true;  // drone101 or leader
                    request_planning->ready_receive_traj[1] = false; // drone102
                    request_planning->ready_receive_traj[2] = false; // drone 103

                    // Send the request asynchronously with a callback -->lambda function here
                    ready_rec_traj_client_->async_send_request(request_planning, 
                        [this](rclcpp::Client<px4_msgs::srv::ReadyReceiveTraj>::SharedFuture future) {
                            try {
                                auto response_planning = future.get();
                                if (response_planning->success) {
                                    LeaderFollower::ready_receive_traj.store(true);
                                    RCLCPP_INFO(this->get_logger(), "Response: %s", response_planning->message.c_str());
                                } else {
                                    RCLCPP_WARN(this->get_logger(), "Service request failed: %s", response_planning->message.c_str());
                                }
                            } catch (const std::exception& e) {
                                RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
                            }
                    });
                }
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

void LeaderFollower::vehicle_odometry_callback(int target_system_id,const VehicleOdometry & msg){
    try{
        // std::atomic rec=true;
        // this->initialpose_received_[target_system_id-1]=true;
        // this->vehicle_pose_[target_system_id-1]=msg;
        // RCLCPP_INFO(this->get_logger(),"position of drone with id : %d is x: %f , y: %f, z: %f",target_system_id,msg.position[0],msg.position[1],msg.position[2]);
    }
     catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
void LeaderFollower::leader_fol_traj_callback(int target_system_id,const TrajectorySetpoint & msg){
    try{
        RCLCPP_DEBUG(this->get_logger(),"callback Position of drone with id : %d is x: %f , y: %f, z: %f vx: %f , vy: %f, vz: %f, yaw: %f"
        ,target_system_id,msg.position[0],msg.position[1],msg.position[2],msg.velocity[0],msg.velocity[1],msg.velocity[2],msg.yaw);
        // change the desire trajectory coordinate from ENU to NED. PX4 is in NED
        
        VehicleOdometry msg_ned;

        msg_ned.timestamp=msg.timestamp;
        msg_ned.position[0]=msg.position[0];
        msg_ned.position[1]=msg.position[1];
        msg_ned.position[2]=msg.position[2];

        msg_ned.velocity[0]=msg.velocity[0];
        msg_ned.velocity[1]=msg.velocity[1];
        msg_ned.velocity[2]=msg.velocity[2];

        if (target_system_id<2){
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, msg.yaw);
            msg_ned.q[0]=q.x();
            msg_ned.q[1]=q.y();
            msg_ned.q[2]=q.z();
            msg_ned.q[3]=q.w();

        }

        
        // std::lock_guard<std::mutex> lock(*element_mutexes_[target_system_id-1]);

        // this->leader_fol_traj_rec_[target_system_id-1]=true;
        if(target_system_id==1){

            (*leader_fol_traj_setpoint_)[target_system_id]->push(msg_ned);     // its initialized in init() fc   
            // RCLCPP_WARN(this->get_logger(),"-----1");
        }
        else{
            if (!(*leader_fol_traj_setpoint_)[target_system_id]->empty()) {
            // Overwrite the front element with the new message
            (*leader_fol_traj_setpoint_)[target_system_id]->front() = msg_ned;
            // RCLCPP_WARN(this->get_logger(),"-----2");
            } else {
            // If it's empty, just push
            (*leader_fol_traj_setpoint_)[target_system_id]->push(msg_ned);
            // RCLCPP_WARN(this->get_logger(),"-----3");
            }
        }

    }
     catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
void LeaderFollower::unbiase_vehicle_odometry_callback(int target_system_id,const VehicleOdometry & msg){
    try{
        // std::atomic rec=true;
        this->initialpose_received_[target_system_id-1]=true;
        this->vehicle_pose_[target_system_id-1]=msg;
        // RCLCPP_INFO(this->get_logger(),"LeaderFollower unbiase position of drone with id : %d is x: %f , y: %f, z: %f",target_system_id,msg.position[0],msg.position[1],msg.position[2]);
    }
     catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
void LeaderFollower::vehicle_status_callback(int target_system_id,const VehicleStatus & msg){
    try{
        // std::atomic rec=true;
        this->vehicle_status_[target_system_id-1]=msg;
        // RCLCPP_INFO(this->get_logger(),"position of drone with id : %d is x: %f , y: %f, z: %f",target_system_id,msg.position[0],msg.position[1],msg.position[2]);
    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}

VehicleCommand LeaderFollower::publish_vehicle_command(uint16_t command, float param1, float param2, uint8_t target_system_id){
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
OffboardControlMode LeaderFollower::offboard_control_mode(int target_system_id){
	OffboardControlMode msg{};
    if (target_system_id>1){//follower velocity mode
	msg.position = false;
	msg.velocity = true;
    }
    else{//leader position mode
	msg.position = true;
	msg.velocity = false;
    }

	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    return msg;
}

TrajectorySetpoint LeaderFollower::trajectory_setpoint(float x, float y, float z,float vx, float vy, float vz, float yaw)
{
	TrajectorySetpoint msg{};
    msg.position = {x, y, z};
	msg.velocity = {vx, vy, vz};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    return msg;
}
bool LeaderFollower::all_values_true(const std::vector<bool>& vec){
    return std::all_of(vec.begin(), vec.end(), [](bool value) { return value; });
}

bool LeaderFollower::is_inside_sphere(VehicleOdometry &drone_pose, VehicleOdometry &traj, double radius) {
    try{// calculation is in NED coordinate system
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
void LeaderFollower::follow_traj(int num_agents){

    try{
            while(!LeaderFollower::traj_start && !LeaderFollower::ready_receive_traj){

                rclcpp::sleep_for(std::chrono::milliseconds(1000));
                RCLCPP_WARN(this->get_logger(), "trajectory is not published to gazebo.its waiting for service respond"); 

            }
            std::vector<bool> traj_end;
            traj_end.resize(num_agents);
            const auto find_yaw_setpoint=[](auto const &q)->double {
                    const double siny_cosp = 2.0 * (q[3] * q[2] + q[0] * q[1]);
                    const double cosy_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
                    return std::atan2(siny_cosp, cosy_cosp);

            };
            while (rclcpp::ok()){
                // RCLCPP_DEBUG(this->get_logger(), "while loop"); 
// 
                for (int target_system_id = 1; target_system_id <= num_agents; target_system_id++) {
                //  RCLCPP_DEBUG(this->get_logger(), "for loop"); 

                    if ((*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->empty()){

                        RCLCPP_DEBUG(this->get_logger(), "qeueu for drone with id [%d] is empty ", target_system_id); 
                        continue;

                    }
                    double set_yaw;
                    if (target_system_id>1)
                        set_yaw=0.0;
                    else 
                        set_yaw=find_yaw_setpoint(((*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().q));
                    RCLCPP_DEBUG(this->get_logger(), "drone id [%d] ,find_yaw_setpoint:  %f ", target_system_id,set_yaw);
                    this->control_mode_publishers_[target_system_id-1]->
                                                publish(LeaderFollower::offboard_control_mode(target_system_id));
                    this->follow_traj_publishers_[target_system_id-1]->
                                                publish(LeaderFollower::trajectory_setpoint(((*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().position[0]),
                                                                                            ((*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().position[1]), 
                                                                                            ((*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().position[2]),
                                                                                            ((*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().velocity[0]),
                                                                                            ((*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().velocity[1]),
                                                                                            ((*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().velocity[2]), 
                                                                                            set_yaw));
                    rclcpp::sleep_for(std::chrono::milliseconds(1));

                    RCLCPP_DEBUG(this->get_logger(),"target_system_id[%d],size of q [%ld], x:%f, y:%f, z: %f, vx: %f, vy: %f, vz: %f",
                    target_system_id,(*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->size(),
                    (*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().position[0],
                    (*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().position[1],
                    (*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().position[2],
                    (*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().velocity[0],
                    (*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().velocity[1],
                    (*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front().velocity[2]);
                    if (target_system_id==1){//only leader track the generated path
                        // is_inside_sphere->calculation is in NED frame
                        if(is_inside_sphere(this->vehicle_pose_[target_system_id-1],(*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->front(),*sphare_radius)) {
                            RCLCPP_DEBUG(this->get_logger(), "leader_fol_traj_setpoint_ target_system_id is [%d] poped ",target_system_id);               

                            (*LeaderFollower::leader_fol_traj_setpoint_)[target_system_id]->pop();//remove the tracked point
                            
                            if(this->vehicle_pose_[target_system_id-1].position[2]>-0.5){ // calculation is in NED coordinate system

                                traj_end[target_system_id-1]=true;
                            }
                        }

                    }

                }

                if(LeaderFollower::all_values_true(traj_end)){
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
std::shared_ptr<std::vector<VehicleOdometry>> LeaderFollower::generate_circle_trajectory(const VehicleOdometry& start_pose, double radius, int num_points) {
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