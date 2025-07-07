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

#include<plugins/planning_for_leader_core.hpp>
#include <rcutils/logging_macros.h>

PlanningForLeader::PlanningForLeader(const rclcpp::NodeOptions & options):Node("planning_for_leader_core",options),
                                                                          ready_send_traj{false},
                                                                          is_thread_publish_generated_setpoints_drone101_active{false},
                                                                          is_thread_publish_generated_setpoints_drone102_active{false},
                                                                          is_thread_publish_generated_setpoints_drone103_active{false}
{
    try{

        init_commons();

        init_algorithm();
    }
    catch(std::exception &ex){
        RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());

    }
}
PlanningForLeader::~PlanningForLeader(){}
void PlanningForLeader::init_commons() {
    try {
        // Common parameters
        setpoint_delay = this->declare_parameter<int>("setpoint_delay");
        // sim_real_ = this->declare_parameter<std::string>("sim_real");
        std::string log_level = this->declare_parameter<std::string>("log_level", "debug");

        // Drone101 parameters
        drone101_path_shape = this->declare_parameter<std::string>("drones.drone101.path.shape", "square");
        drone101_resolution = this->declare_parameter<int>("drones.drone101.path.resolution", 1000);
        drone101_ratio = this->declare_parameter<double>("drones.drone101.path.ratio", 1.0);
        drone101_only_line = this->declare_parameter<std::vector<int64_t>>("drones.drone101.path.only_line", std::vector<int64_t>{1, 1, 0});
        drone101_start_pose = this->declare_parameter<std::vector<double>>("drones.drone101.start_pose", std::vector<double>{2.0, 2.0, 1.0});
        drone101_yaw_align_velocity = this->declare_parameter<bool>("drones.drone101.yaw_align_velocity", false);
        RCLCPP_DEBUG(this->get_logger(), "drone101_yaw_align_velocity: [%d]",drone101_yaw_align_velocity);

        // Drone102 parameters
        drone102_path_shape = this->declare_parameter<std::string>("drones.drone102.path.shape", "circle");
        drone102_resolution = this->declare_parameter<int>("drones.drone102.path.resolution", 1000);
        drone102_ratio = this->declare_parameter<double>("drones.drone102.path.ratio", 1.0);
        drone102_only_line = this->declare_parameter<std::vector<int64_t>>("drones.drone102.path.only_line", std::vector<int64_t>{1, 1, 0});
        drone102_start_pose = this->declare_parameter<std::vector<double>>("drones.drone102.start_pose", std::vector<double>{3.0, 3.0, 1.0});

        // Drone103 parameters
        drone103_path_shape = this->declare_parameter<std::string>("drones.drone103.path.shape", "triangle");
        drone103_resolution = this->declare_parameter<int>("drones.drone103.path.resolution", 3);
        drone103_ratio = this->declare_parameter<double>("drones.drone103.path.ratio", 1.5);
        drone103_only_line = this->declare_parameter<std::vector<int64_t>>("drones.drone103.path.only_line", std::vector<int64_t>{1, 1, 0});
        drone103_start_pose = this->declare_parameter<std::vector<double>>("drones.drone103.start_pose", std::vector<double>{4.0, 4.0, 1.0});

        // Logging common parameters
        RCLCPP_WARN(this->get_logger(),
                    "Config info:\n\tsetpoint_delay: [%d]\n\tlog_level: [%s]",
                    setpoint_delay, log_level.c_str());

        // Log Drone101 configuration
        RCLCPP_INFO(this->get_logger(),
                    "Drone101 -> path: {shape: %s, resolution: %d, ratio: %f, only_line: [%ld, %ld, %ld]}, "
                    "start_pose: [%f, %f, %f]",
                    drone101_path_shape.c_str(), drone101_resolution, drone101_ratio,
                    drone101_only_line[0], drone101_only_line[1], drone101_only_line[2],
                    drone101_start_pose[0], drone101_start_pose[1], drone101_start_pose[2]);

        // Log Drone102 configuration
        RCLCPP_INFO(this->get_logger(),
                    "Drone102 -> path: {shape: %s, resolution: %d, ratio: %f, only_line: [%ld, %ld, %ld]}, "
                    "start_pose: [%f, %f, %f]",
                    drone102_path_shape.c_str(), drone102_resolution, drone102_ratio,
                    drone102_only_line[0], drone102_only_line[1], drone102_only_line[2],
                    drone102_start_pose[0], drone102_start_pose[1], drone102_start_pose[2]);

        // Log Drone103 configuration
        RCLCPP_INFO(this->get_logger(),
                    "Drone103 -> path: {shape: %s, resolution: %d, ratio: %f, only_line: [%ld, %ld, %ld]}, "
                    "start_pose: [%f, %f, %f]",
                    drone103_path_shape.c_str(), drone103_resolution, drone103_ratio,
                    drone103_only_line[0], drone103_only_line[1], drone103_only_line[2],
                    drone103_start_pose[0], drone103_start_pose[1], drone103_start_pose[2]);




                    
    } catch (std::exception &ex) {
        RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
    }
}

void PlanningForLeader::init_algorithm(){
    try{
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        rmw_qos_profile_t qos_profile_traj = rmw_qos_profile_parameters;
        auto qos_traj = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile_traj.history, 10), qos_profile_traj);
        PlanningForLeader::ready_send_traj_service_ = this->create_service<px4_msgs::srv::ReadyReceiveTraj>(
            "/ready_run_gen_traj",
            [this](const std::shared_ptr<px4_msgs::srv::ReadyReceiveTraj::Request> request,
                            std::shared_ptr<px4_msgs::srv::ReadyReceiveTraj::Response> response) {
                this->handle_traj_request(request, response);
            });
        RCLCPP_DEBUG(this->get_logger(), "ready_generate_traj started!");
        std::string topic_name_traj_setpoint_drone101_xyz ="/rviz/drone101/send_setpoint_xyz/enu";
        std::string topic_name_traj_setpoint_drone101_xyzyaw ="/rviz/drone101/send_setpoint_xyzyaw/enu";

        std::string topic_name_traj_setpoint_drone102 ="/rviz/drone102/send_setpoint_xyz/enu";
        std::string topic_name_traj_setpoint_drone103 ="/rviz/drone103/send_setpoint_xyz/enu";
        std::string topic_name_pose_drone101 ="/px4_1/fmu/out/vehicle_odometry_unbiase_z/enu";

        RCLCPP_DEBUG(this->get_logger(), "This is a real world scenario");

            PlanningForLeader::pose_subscription_real_drone101_ = this->create_subscription<VehicleOdometry>(topic_name_pose_drone101, 10, std::bind(&PlanningForLeader::on_recv_pose_drone101, this, std::placeholders::_1));

        // std::string topic_name_traj_setpoint = "/px4_" + std::to_string(1) + "/leader_follower/trajectory_setpoint";
            PlanningForLeader::setpoint_publisher_real_drone101_xyz_ = this->create_publisher<TrajectorySetpoint>(topic_name_traj_setpoint_drone101_xyz, qos_traj);
            PlanningForLeader::setpoint_publisher_real_drone101_xyzyaw_ = this->create_publisher<TrajectorySetpoint>(topic_name_traj_setpoint_drone101_xyzyaw, qos_traj);
        PlanningForLeader::setpoint_publisher_real_drone102_ = this->create_publisher<TrajectorySetpoint>(topic_name_traj_setpoint_drone102, qos_traj);
        PlanningForLeader::setpoint_publisher_real_drone103_ = this->create_publisher<TrajectorySetpoint>(topic_name_traj_setpoint_drone103, qos_traj);

 

        RCLCPP_DEBUG(this->get_logger(), "ready_send_traj is true!");

        RCLCPP_DEBUG(this->get_logger(), "thread detached!");
        sum_sin_               = 0.0f;
        sum_cos_               = 0.0f;
        counter_yaw_filt_   = 0u;
        yaw_radian_drone101_  = 0.0f;   // filtered instantaneous yaw
        yaw_radian_drone101_avg_ = 0.0f;

    }
    catch(std::exception &ex){
        RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
    }
}

// Clamp to [-pi, pi)
float wrap(float theta_radian) {
    while(theta_radian >= M_PI) {
        theta_radian -= 2*M_PI;
    }
    while(theta_radian < -M_PI) {
        theta_radian += 2*M_PI;
    }
    return theta_radian;
}
const float generate_yaw_velocity_threshold = 0.05; // (m/s)
void PlanningForLeader::on_recv_pose_drone101(const VehicleOdometry &msg)
{
    /* ---------- 0. reject low-speed samples ---------- */
    const float vx = msg.velocity[0];
    const float vy = msg.velocity[1];

    if (vx * vx + vy * vy <
        generate_yaw_velocity_threshold * generate_yaw_velocity_threshold) {
        return;                       // keep counter & sums unchanged
    }

    /* ---------- 1. first-order yaw filter ------------ */
    const float new_yaw = std::atan2(vy, vx);          // raw heading
    const float err     = wrap(yaw_radian_drone101_ - new_yaw);

    constexpr float alpha = 0.1f / (0.1f + 1.0f);      // dT / (dT + τ)
    yaw_radian_drone101_  = wrap(yaw_radian_drone101_ - alpha * err);

    /* ---------- 2. accumulate for 20-sample mean ----- */
    sum_sin_ += std::sin(yaw_radian_drone101_);
    sum_cos_ += std::cos(yaw_radian_drone101_);
    ++counter_yaw_filt_;                               // increment **after** acceptance

    if (counter_yaw_filt_ == 20)                       // exactly 20 good samples
    {
        yaw_radian_drone101_avg_ = std::atan2(sum_sin_, sum_cos_); // circular mean

        // reset for next window
        sum_sin_         = 0.0f;
        sum_cos_         = 0.0f;
        counter_yaw_filt_ = 0u;
    }
}


void PlanningForLeader::publish_generated_setpoints_drone101(){

    try{
        RCLCPP_DEBUG_STREAM(this->get_logger(),"drone101 is activated");

        VehicleOdometry start_pose1,start_pose2;
        start_pose1.position={  drone101_start_pose[0],
                                drone101_start_pose[1],
                                drone101_start_pose[2] };
        std::shared_ptr<std::vector<VehicleOdometry>>   leader_traj = std::make_shared<std::vector<VehicleOdometry>>();
        std::shared_ptr<TrajectorySetpoint>             leader_traj_real = std::make_shared<TrajectorySetpoint>();
        std::shared_ptr<TrajectorySetpoint>             leader_traj_real_sim = std::make_shared<TrajectorySetpoint>();

        leader_traj->resize(drone101_resolution+1);
        if (drone101_path_shape=="circle")
            leader_traj= PlanningForLeader::generate_circle_trajectory(start_pose1,drone101_ratio,drone101_resolution,drone101_only_line);//its included land position
        else 
            leader_traj= PlanningForLeader::generate_square_trajectory(start_pose1,drone101_ratio,drone101_resolution,drone101_only_line);//its included land position
        int counter=0;

        while(rclcpp::ok() && counter<leader_traj->size()){
            RCLCPP_DEBUG(this->get_logger(), "drone101's counter is [%d]",counter);
            rclcpp::sleep_for(std::chrono::milliseconds(setpoint_delay));
            leader_traj_real->velocity = {NAN, NAN, NAN};
            leader_traj_real->acceleration = {NAN, NAN, NAN};
            leader_traj_real->jerk = {NAN, NAN, NAN};
            leader_traj_real->yaw = NAN;
            leader_traj_real->yawspeed = NAN;
            leader_traj_real->position[0]=leader_traj->at(counter).position[0];
            leader_traj_real->position[1]=leader_traj->at(counter).position[1];
            leader_traj_real->position[2]=leader_traj->at(counter).position[2];
            if (drone101_yaw_align_velocity) {
                leader_traj_real->yaw = yaw_radian_drone101_avg_;
                RCLCPP_DEBUG(this->get_logger(), "yaw_radian_drone101_avg [%f]",yaw_radian_drone101_avg_);

                PlanningForLeader::setpoint_publisher_real_drone101_xyzyaw_->publish(*leader_traj_real);
            }
            else {
                PlanningForLeader::setpoint_publisher_real_drone101_xyz_->publish(*leader_traj_real);
            }
            // PlanningForLeader::setpoint_publisher_real_drone101_->publish(*leader_traj_real);
            counter++;

        }
    }
    catch(std::exception &ex){
        RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
    }
}
void PlanningForLeader::publish_generated_setpoints_drone102(){

    try{
        RCLCPP_DEBUG_STREAM(this->get_logger(),"drone102 is activated");
        VehicleOdometry start_pose1,start_pose2;
        start_pose1.position={  drone102_start_pose[0],
                                drone102_start_pose[1],
                                drone102_start_pose[2] };
        std::shared_ptr<std::vector<VehicleOdometry>>   leader_traj = std::make_shared<std::vector<VehicleOdometry>>();
        std::shared_ptr<TrajectorySetpoint>             leader_traj_real = std::make_shared<TrajectorySetpoint>();

        leader_traj->resize(drone102_resolution+1);
        if (drone102_path_shape=="circle")
            leader_traj= PlanningForLeader::generate_circle_trajectory(start_pose1,drone102_ratio,drone102_resolution,drone102_only_line);//its included land position
        else 
            leader_traj= PlanningForLeader::generate_square_trajectory(start_pose1,drone102_ratio,drone102_resolution,drone102_only_line);//its included land position
        int counter=0;

        while(rclcpp::ok() && counter<leader_traj->size()){
            RCLCPP_DEBUG(this->get_logger(), "drone102's counter is [%d]",counter);
            rclcpp::sleep_for(std::chrono::milliseconds(setpoint_delay));
            leader_traj_real->velocity = {NAN, NAN, NAN};
            leader_traj_real->acceleration = {NAN, NAN, NAN};
            leader_traj_real->jerk = {NAN, NAN, NAN};
            leader_traj_real->yaw = NAN;
            leader_traj_real->yawspeed = NAN;
            leader_traj_real->position[0]=leader_traj->at(counter).position[0];
            leader_traj_real->position[1]=leader_traj->at(counter).position[1];
            leader_traj_real->position[2]=leader_traj->at(counter).position[2];

            PlanningForLeader::setpoint_publisher_real_drone102_->publish(*leader_traj_real);
            counter++;

        }
    }
    catch(std::exception &ex){
        RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
    }
}
void PlanningForLeader::publish_generated_setpoints_drone103(){

    try{
        RCLCPP_DEBUG_STREAM(this->get_logger(),"drone103 is activated");
        VehicleOdometry start_pose1,start_pose2;
        start_pose1.position={  drone103_start_pose[0],
                                drone103_start_pose[1],
                                drone103_start_pose[2] };
        std::shared_ptr<std::vector<VehicleOdometry>>   leader_traj = std::make_shared<std::vector<VehicleOdometry>>();
        std::shared_ptr<TrajectorySetpoint>             leader_traj_real = std::make_shared<TrajectorySetpoint>();

        leader_traj->resize(drone103_resolution+1);
        if (drone103_path_shape=="circle")
            leader_traj= PlanningForLeader::generate_circle_trajectory(start_pose1,drone103_ratio,drone103_resolution,drone103_only_line);//its included land position
        else 
            leader_traj= PlanningForLeader::generate_square_trajectory(start_pose1,drone103_ratio,drone103_resolution,drone103_only_line);//its included land position
        int counter=0;

        while(rclcpp::ok() && counter<leader_traj->size()){
            RCLCPP_DEBUG(this->get_logger(), "drone103's counter is [%d]",counter);
            rclcpp::sleep_for(std::chrono::milliseconds(setpoint_delay));
            leader_traj_real->velocity = {NAN, NAN, NAN};
            leader_traj_real->acceleration = {NAN, NAN, NAN};
            leader_traj_real->jerk = {NAN, NAN, NAN};
            leader_traj_real->yaw = NAN;
            leader_traj_real->yawspeed = NAN;
            leader_traj_real->position[0]=leader_traj->at(counter).position[0];
            leader_traj_real->position[1]=leader_traj->at(counter).position[1];
            leader_traj_real->position[2]=leader_traj->at(counter).position[2];

            PlanningForLeader::setpoint_publisher_real_drone103_->publish(*leader_traj_real);
            counter++;

        }
    }
    catch(std::exception &ex){
        RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
    }
}
std::shared_ptr<std::vector<VehicleOdometry>> PlanningForLeader::generate_circle_trajectory(const VehicleOdometry& start_pose, double radius, int num_points,std::vector<int64_t> only_line) {
    auto trajectory = std::make_shared<std::vector<VehicleOdometry>>();
    VehicleOdometry point;

    for (int i = 0; i < num_points; ++i) {
        double angle = 2 * M_PI * i / num_points; // Angle for the current point
        point.position = {
            (only_line[0])*(start_pose.position[0] + radius * cos(angle)), // x = center_x + r * cos(theta)
            (only_line[1])*(start_pose.position[1] + radius * sin(angle)), // y = center_y + r * sin(theta)
            start_pose.position[2]                        // z remains constant
        };
        trajectory->push_back(point);
    }
    // last XY pose is the last XY land pose
    VehicleOdometry land_pose;
    land_pose=point;
    land_pose.position[2]=0.1;
    trajectory->push_back(land_pose);
    return trajectory;
}
void PlanningForLeader::handle_traj_request(const std::shared_ptr<px4_msgs::srv::ReadyReceiveTraj::Request> request,
                                std::shared_ptr<px4_msgs::srv::ReadyReceiveTraj::Response> response){
    try{

        if (request->ready_receive_traj[0] && !is_thread_publish_generated_setpoints_drone101_active ) {
            response->success = true;
            response->message = "server is sending setpoints to drone101 or leader";
            RCLCPP_INFO_STREAM(this->get_logger(), response->message.c_str());
            is_thread_publish_generated_setpoints_drone101_active=true;
            std::thread([this]() { this->publish_generated_setpoints_drone101(); }).detach();
        }
        else {
            if (!is_thread_publish_generated_setpoints_drone101_active){

                response->success = false;
                response->message = "server is NOT sending the generated traj to the drone101 or leader.";
                RCLCPP_WARN_STREAM(this->get_logger(), response->message.c_str());
            }
        }
        if (request->ready_receive_traj[1] && !is_thread_publish_generated_setpoints_drone102_active) {
            response->success = true;
            response->message = "server is sending setpoints to drone102";
            RCLCPP_INFO_STREAM(this->get_logger(), response->message.c_str());
            is_thread_publish_generated_setpoints_drone102_active=true;
            std::thread([this]() { this->publish_generated_setpoints_drone102(); }).detach();
        }
        else {
            if (!is_thread_publish_generated_setpoints_drone102_active){
                response->success = false;
                response->message = "server is NOT sending the generated traj to the drone102.";
                RCLCPP_WARN_STREAM(this->get_logger(), response->message.c_str());
            }

        }
        if (request->ready_receive_traj[2] && !is_thread_publish_generated_setpoints_drone103_active) {
            response->success = true;
            response->message = "server is sending setpoints to drone103";
            RCLCPP_INFO_STREAM(this->get_logger(), response->message.c_str());
            is_thread_publish_generated_setpoints_drone103_active=true;
            std::thread([this]() { this->publish_generated_setpoints_drone103(); }).detach();
        }
        else {
            if (!is_thread_publish_generated_setpoints_drone103_active){
                response->success = false;
                response->message = "server is NOT sending the generated traj to the drone103.";
                RCLCPP_WARN_STREAM(this->get_logger(), response->message.c_str());
            }
        }
    }
    
    catch(std::exception &ex){
        RCLCPP_ERROR_STREAM(this->get_logger(), ex.what());
    }

}
std::shared_ptr<std::vector<VehicleOdometry>> PlanningForLeader::generate_square_trajectory(const VehicleOdometry& start_pose, double radius, int num_points,std::vector<int64_t> only_line) {
    auto trajectory = std::make_shared<std::vector<VehicleOdometry>>();
    VehicleOdometry point;

    // Define the 4 corners of the square:
    // Top-left, Top-right, Bottom-right, Bottom-left
    std::vector<std::array<double, 2>> corners = {
        {start_pose.position[0] , start_pose.position[1] },
        {start_pose.position[0] + radius, start_pose.position[1]},
        {start_pose.position[0] + radius, start_pose.position[1] + radius},
        {start_pose.position[0]         , start_pose.position[1] + radius}
    };

    // Generate points along the square's perimeter
    for (int i = 0; i < num_points; ++i) {
        // Map the index i to a parameter t in the range [0, 4) (4 segments)
        double t = (static_cast<double>(i) / num_points) * 4.0;
        int segment = static_cast<int>(t);       // Determines which side of the square we are on
        double local = t - segment;              // Local interpolation factor on that side

        // Interpolate between the current corner and the next corner
        double x = (1.0 - local) * corners[segment % 4][0] + local * corners[(segment + 1) % 4][0];
        double y = (1.0 - local) * corners[segment % 4][1] + local * corners[(segment + 1) % 4][1];

        point.position = {
            (only_line[0]) * x,
            (only_line[1]) * y,
            start_pose.position[2]  // z remains constant
        };

        trajectory->push_back(point);
    }

    //Add a landing pose at the end (adjust z for landing)
    // VehicleOdometry land_pose = point;
    // land_pose.position[2] = 0.1;
    // trajectory->push_back(land_pose);

    return trajectory;
}
