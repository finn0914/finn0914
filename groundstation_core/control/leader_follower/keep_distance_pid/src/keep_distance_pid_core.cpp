////////////////////////////////////////////////////////////////////////////////
//******************************************************************************************************* 
//*******This node is responsible to force followers to follow thier leader ********
//******************************************************************************************************* 
// Copyright (c) 2025/1 NTU UAV lab Inc.
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

#include "keep_distance_pid/keep_distance_pid_core.hpp"

KeepDist::KeepDist(const rclcpp::NodeOptions & options):Node("keep_distance_pid_core", options) ,
start_following{false},pid_x_(1,0.1,0.0),pid_y_(1,0.1,0.0),pid_z_(1,0.1,0.0)
{

        this->declare_parameter<int>("num_agents", 1);
        int num_agents = this->get_parameter("num_agents").as_int();
        this->declare_parameter<double>("distance", 0.1);
        distance = this->get_parameter("distance").as_double();
        this->declare_parameter<std::string>("controller_name", "pid");
        controller_name = this->get_parameter("controller_name").as_string();

        this->declare_parameter<double>("kpx", 0.1);
        kpx = this->get_parameter("kpx").as_double();
        this->declare_parameter<double>("kix", 0.1);
        kix = this->get_parameter("kix").as_double();
        this->declare_parameter<double>("kdx", 0.1);
        kdx = this->get_parameter("kdx").as_double();

        this->declare_parameter<double>("kpy", 0.1);
        kpy = this->get_parameter("kpy").as_double();
        this->declare_parameter<double>("kiy", 0.1);
        kiy = this->get_parameter("kiy").as_double();
        this->declare_parameter<double>("kdy", 0.1);
        kdy = this->get_parameter("kdy").as_double();
        
        this->declare_parameter<double>("kpz", 0.1);
        kpz = this->get_parameter("kpz").as_double();
        this->declare_parameter<double>("kiz", 0.1);
        kiz = this->get_parameter("kiz").as_double();
        this->declare_parameter<double>("kdz", 0.1);
        kdz = this->get_parameter("kdz").as_double();
        
        RCLCPP_WARN(this->get_logger(),"num_agents [%d], distance [%f], controller_name[%s]",num_agents,distance,controller_name.c_str());
        // C++14 or C++17 style
        on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params)
            -> rcl_interfaces::msg::SetParametersResult
            {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto & param : params) {
                if (param.get_name() == "distance") {
                    double new_distance = param.as_double();
                    distance = new_distance;                
                }
                if (param.get_name() == "kpx") {
                    double new_kpx = param.as_double();
                    kpx = new_kpx;                
                }
                if (param.get_name() == "kix") {
                    double new_kix = param.as_double();
                    kix = new_kix;
                }
                if (param.get_name() == "kdx") {
                    double new_kdx = param.as_double();
                    kdx = new_kdx;
                }
                if (param.get_name() == "kpy") {
                    double new_kpy = param.as_double();
                    kpy = new_kpy;                
                }
                if (param.get_name() == "kiy") {
                    double new_kiy = param.as_double();
                    kiy = new_kiy;
                }
                if (param.get_name() == "kdy") {
                    double new_kdy = param.as_double();
                    kdy = new_kdy;
                }
                if (param.get_name() == "kpz") {
                    double new_kpz = param.as_double();
                    kpz = new_kpz;                
                }
                if (param.get_name() == "kiz") {
                    double new_kiz = param.as_double();
                    kiz = new_kiz;
                }
                if (param.get_name() == "kdz") {
                    double new_kdz = param.as_double();
                    kdz = new_kdz;
                }
            }

            return result;
            }
        );

        pid_x_.print_coefficients();
        init(num_agents);

}
KeepDist::~KeepDist(){

}
void KeepDist::init(int num_agents){
    try{
        
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        rmw_qos_profile_t qos_profile_traj = rmw_qos_profile_parameters;
        auto qos_traj = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_traj.history, 10), qos_profile_traj);

        KeepDist::set_traj_vel=std::make_shared<VehicleOdometry>();


        std::string topic_name_take_off_done_pub="/px4/all/take_off_done";
        std::string topic_name_unbiase1_z = "/px4_1/fmu/out/vehicle_odometry_unbiase_z/enu";
        std::string topic_name_unbiase2_z = "/px4_2/fmu/out/vehicle_odometry_unbiase_z/enu";

        std::string topic_name_traj_setpoint = "/rviz/drone102/send_setpoint_xyzyaw/enu";
        std::string topic_name_controller_info = "/px4_2/leader_follower/"+controller_name+"_controller/info";

        take_off_done_sub_=this->create_subscription<std_msgs::msg::Bool>(topic_name_take_off_done_pub,qos,
        std::bind(&KeepDist::take_off_done_callback, this, std::placeholders::_1));
        vehicle_odom_unbiase_sub_=this->create_subscription<VehicleOdometry>(topic_name_unbiase1_z,qos,
        std::bind(&KeepDist::vehicle_odom_unbiase_callback, this, std::placeholders::_1));
        vehicle_odom_unbiase_sub2_=this->create_subscription<VehicleOdometry>(topic_name_unbiase2_z,qos,
        std::bind(&KeepDist::vehicle_odom_unbiase_callback_2, this, std::placeholders::_1));
        KeepDist::setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(topic_name_traj_setpoint, qos);
        KeepDist::controller_info_publisher_ = this->create_publisher<PidInfo>(topic_name_controller_info, qos);

        KeepDist::follower_odom_=std::make_shared<VehicleOdometry>();
        

        
    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());

    }
}

void KeepDist::vehicle_odom_unbiase_callback_2(const VehicleOdometry & msg){
    try{
        RCLCPP_DEBUG(this->get_logger(),"follower position is x: %f , y: %f, z: %f",msg.position[0],msg.position[1],msg.position[2]);

        *KeepDist::follower_odom_=msg;
    }     
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
void KeepDist::vehicle_odom_unbiase_callback(const VehicleOdometry & msg){
    try{
        RCLCPP_DEBUG(this->get_logger(),"leader position is x: %f , y: %f, z: %f",msg.position[0],msg.position[1],msg.position[2]);

        if(!KeepDist::start_following.load()){
            RCLCPP_WARN(this->get_logger(), "[vehicle_odom_unbiase_callback] start_following result is [%d]",KeepDist::start_following.load());
            return;
        }        
        dt=0.01;//100hz
        // distance from leader on x axis is 1
        // RCLCPP_INFO(this->get_logger(),"-1");
        set_traj_vel->position[0]=std::numeric_limits<double>::quiet_NaN();
        set_traj_vel->position[1]=std::numeric_limits<double>::quiet_NaN();
        set_traj_vel->position[2]=std::numeric_limits<double>::quiet_NaN();

        pid_x_.set_coefficients(kpx,kix,kdx);
        pid_y_.set_coefficients(kpy,kiy,kdy);
        pid_z_.set_coefficients(kpz,kiz,kdz);
        PidInfo pid_info; 
        pid_info.distance_error_x=(-msg.position[0]+KeepDist::follower_odom_->position[0]);
        pid_info.distance_error_y=(-msg.position[1]+KeepDist::follower_odom_->position[1]);
        pid_info.distance_error_z=(-msg.position[2]+KeepDist::follower_odom_->position[2]);
        pid_info.pid_error_x=distance-pid_info.distance_error_x;
        pid_info.pid_error_y=distance-pid_info.distance_error_y;
        pid_info.pid_error_z=0.0-pid_info.distance_error_z;
        
        set_traj_vel->velocity[0]=pid_x_.compute(distance,pid_info.distance_error_x,dt);
        set_traj_vel->velocity[1]=pid_y_.compute(distance,pid_info.distance_error_y,dt);
        set_traj_vel->velocity[2]=pid_z_.compute(0,pid_info.distance_error_z,dt); // in ENU frame

        // mode-> velocity,so follower track must velocity profile
        KeepDist::controller_info_publisher_->publish(pid_info);
        
        std::shared_ptr<TrajectorySetpoint> leader_traj_real_sim = std::make_shared<TrajectorySetpoint>();
        leader_traj_real_sim->position = {NAN, NAN, NAN};
        leader_traj_real_sim->acceleration = {NAN, NAN, NAN};
        leader_traj_real_sim->jerk = {NAN, NAN, NAN};
        leader_traj_real_sim->yaw = NAN;
        leader_traj_real_sim->yawspeed = NAN;
        leader_traj_real_sim->velocity[0]=set_traj_vel->velocity[0];
        leader_traj_real_sim->velocity[1]=set_traj_vel->velocity[1];
        leader_traj_real_sim->velocity[2]=set_traj_vel->velocity[2];

        KeepDist::setpoint_publisher_->publish(*leader_traj_real_sim);                                                                                                                                             
        RCLCPP_DEBUG(this->get_logger(),"controller output is: x:%f, y:%f, z: %f, vx: %f, vy: %f, vz: %f",0.0,0.0,0.0,
        set_traj_vel->velocity[0],
        set_traj_vel->velocity[1],
        set_traj_vel->velocity[2]);
        // RCLCPP_INFO(this->get_logger(),"leader position is x: %f , y: %f, z: %f",msg.position[0],msg.position[1],msg.position[2]);
    }     
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}

VehicleCommand KeepDist::publish_vehicle_command(uint16_t command, float param1, float param2, uint8_t target_system_id){
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
OffboardControlMode KeepDist::offboard_control_mode(){
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    return msg;
}

TrajectorySetpoint KeepDist::trajectory_setpoint(float x, float y, float z,float vx, float vy, float vz, float yaw)
{
	TrajectorySetpoint msg{};
    msg.position = {x, y, z};
	msg.velocity = {vx, vy, vz};
	msg.yaw = yaw; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    return msg;
}
void KeepDist::take_off_done_callback(const std_msgs::msg::Bool msg){
    try{
        
        if (!msg.data){
            RCLCPP_WARN(this->get_logger(), "take_off_done_callback result is [%d]",msg.data );
            return;
        }

        KeepDist::start_following.store(true);

    }
    catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }


}
