#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Vector3.h>

using namespace px4_msgs::msg;
using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;

class TopicsConversion: public rclcpp::Node{

public:
//TopicsConversion() : TopicsConversion(rclcpp::NodeOptions{})
TopicsConversion(const rclcpp::NodeOptions & options):Node("topics_conversion_node", options)
,tf_buffer_(this->get_clock())
,tf_buffer_traj_(this->get_clock())
,tf_listener_(tf_buffer_)
,tf_listener_traj_(tf_buffer_traj_)
,tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))

{

  this->declare_parameter<int>("num_agents", 1);
  int num_agents = this->get_parameter("num_agents").as_int();
  rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
	auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
  run_mode = this->declare_parameter<std::string>("run_mode", "sim");
  std::string topic_name_unbiase_vehicle_odometry_ned;
  std::string topic_name_unbiase_vehicle_odometry_enu;
  std::string topic_name_trajectory_setpoint_enu;
  std::string topic_name_trajectory_setpoint_ned;
  for (int target_system_id = 1; target_system_id <= num_agents; target_system_id++) {
    if (run_mode=="sim"){
      topic_name_unbiase_vehicle_odometry_ned = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_odometry_unbiase_z";
      topic_name_unbiase_vehicle_odometry_enu = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_odometry_unbiase_z/enu";

      topic_name_trajectory_setpoint_enu = "/rviz/drone10" + std::to_string(target_system_id) + "/send_setpoint_xyzyaw/enu";
      topic_name_trajectory_setpoint_ned = "/rviz/drone10" + std::to_string(target_system_id) + "/send_setpoint_xyzyaw";
      RCLCPP_DEBUG(this->get_logger(),"Run Mode: sim");

    }
    else{
      topic_name_unbiase_vehicle_odometry_ned = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_odometry_unbiase_z";
      topic_name_unbiase_vehicle_odometry_enu = "/px4_" + std::to_string(target_system_id) + "/fmu/out/vehicle_odometry_unbiase_z/enu";

      topic_name_trajectory_setpoint_enu = "/rviz/drone10" + std::to_string(target_system_id) + "/send_setpoint_xyzyaw/enu";
      topic_name_trajectory_setpoint_ned = "/rviz/drone10" + std::to_string(target_system_id) + "/send_setpoint_xyzyaw";
      RCLCPP_DEBUG(this->get_logger(),"Run Mode: real");

    }

    rclcpp::Subscription<TrajectorySetpoint>::SharedPtr traj_setpoint_enu_sub_ = this->create_subscription<TrajectorySetpoint>(
    topic_name_trajectory_setpoint_enu,qos, [this, target_system_id](const TrajectorySetpoint &msg) 
    {trajectory_setpoint_enu_callback(target_system_id, msg);});

    rclcpp::Subscription<VehicleOdometry>::SharedPtr unbiase_vehicle_odometry_ned_sub_ = this->create_subscription<VehicleOdometry>(
    topic_name_unbiase_vehicle_odometry_ned,qos, [this, target_system_id](const VehicleOdometry &msg) 
    {unbiase_vehicle_odometry_callback(target_system_id, msg);});


    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr  traj_setpoint_ned_pub_ = this->create_publisher<TrajectorySetpoint>(topic_name_trajectory_setpoint_ned, 50);
    rclcpp::Publisher<VehicleOdometry>::SharedPtr  unbiase_vehicle_odometry_enu_pub_ = this->create_publisher<VehicleOdometry>(topic_name_unbiase_vehicle_odometry_enu, 50);

    unbiase_vehicle_odometry_ned_sub_vect_.push_back(unbiase_vehicle_odometry_ned_sub_);
    unbiase_vehicle_odometry_enu_pub_vect_.push_back(unbiase_vehicle_odometry_enu_pub_);

    traj_setpoint_enu_sub_ect_.push_back(traj_setpoint_enu_sub_);
    traj_setpoint_ned_pub_vect_.push_back(traj_setpoint_ned_pub_);
  }
}
~TopicsConversion(){}
void trajectory_setpoint_enu_callback(int target_system_id,const TrajectorySetpoint & msg){
    try{
      double yaw_enu,roll_ned,pitch_ned,yaw_ned;
      rclcpp::Time stamp_ros(static_cast<uint64_t>(msg.timestamp) * 1000ULL,RCL_SYSTEM_TIME);   // <- same clock type RViz uses
      std::shared_ptr<TrajectorySetpoint>   traj_setpoint_ned = std::make_shared<TrajectorySetpoint>();

      if(target_system_id==1){
        
        yaw_enu=msg.yaw;
        tf2::Quaternion q,q_ned;
        q.setRPY(0.0, 0.0, yaw_enu);
        geometry_msgs::msg::PoseStamped pose_enu,pose_ned;
        pose_enu.header.stamp = stamp_ros;
        pose_enu.header.frame_id = "map_enu";
        pose_enu.pose.position.x = msg.position[0];
        pose_enu.pose.position.y = msg.position[1];
        pose_enu.pose.position.z = msg.position[2];
        pose_enu.pose.orientation.x = q[1];
        pose_enu.pose.orientation.y = q[2];
        pose_enu.pose.orientation.z = q[3];
        pose_enu.pose.orientation.w = q[0];
        std::string target_frame="drone10" + std::to_string(target_system_id) + "/base_link";

        auto map2base = tf_buffer_traj_.lookupTransform(
                                                        "station_ned",         // target
                                                        "map_enu", // source ("map_enu")
                                                         tf2::TimePointZero,
                                                        tf2::durationFromSec(0.05));
        tf2::doTransform(pose_enu, pose_ned, map2base);


        // pose_ned = tf_buffer_traj_.transform(pose_enu, target_frame, tf2::durationFromSec(0.05));

        geometry_msgs::msg::Quaternion q_msg;

        q_msg.x=pose_ned.pose.orientation.x;
        q_msg.y=pose_ned.pose.orientation.y;
        q_msg.z=pose_ned.pose.orientation.z;
        q_msg.w=pose_ned.pose.orientation.w;
        tf2::fromMsg(q_msg, q_ned);
        tf2::Matrix3x3(q_ned).getRPY(roll_ned, pitch_ned, yaw_ned);
        traj_setpoint_ned->velocity = {NAN, NAN, NAN};
        traj_setpoint_ned->acceleration = {NAN, NAN, NAN};
        traj_setpoint_ned->jerk = {NAN, NAN, NAN};
        traj_setpoint_ned->yaw = yaw_ned;
        traj_setpoint_ned->yawspeed = NAN;
        traj_setpoint_ned->position[0]=pose_ned.pose.position.x;
        traj_setpoint_ned->position[1]=pose_ned.pose.position.y;
        traj_setpoint_ned->position[2]=pose_ned.pose.position.z;
      }
      else{

        yaw_enu=0.0;
        tf2::Quaternion q,q_ned;
        q.setRPY(0.0, 0.0, yaw_enu);
        geometry_msgs::msg::PoseStamped vel_enu,vel_ned;

        vel_enu.header.stamp = stamp_ros;
        vel_enu.header.frame_id = "map_enu";
        vel_enu.pose.position.x = msg.velocity[0];
        vel_enu.pose.position.y = msg.velocity[1];
        vel_enu.pose.position.z = msg.velocity[2];
        vel_enu.pose.orientation.x = q[1];
        vel_enu.pose.orientation.y = q[2];
        vel_enu.pose.orientation.z = q[3];
        vel_enu.pose.orientation.w = q[0];
        std::string target_frame="drone10" + std::to_string(target_system_id) + "/base_link";
        auto map2base = tf_buffer_traj_.lookupTransform(
                                                        "station_ned",         // target
                                                        "map_enu", // source ("map_enu")
                                                         tf2::TimePointZero,
                                                        tf2::durationFromSec(0.05));
        tf2::doTransform(vel_enu, vel_ned, map2base);

        geometry_msgs::msg::Quaternion q_msg;
        traj_setpoint_ned->timestamp = msg.timestamp;

        traj_setpoint_ned->acceleration = {NAN, NAN, NAN};
        traj_setpoint_ned->jerk = {NAN, NAN, NAN};
        traj_setpoint_ned->yaw = NAN;
        traj_setpoint_ned->yawspeed = NAN;
        traj_setpoint_ned->position = {NAN, NAN, NAN};
        traj_setpoint_ned->velocity[0]=vel_ned.pose.position.x;
        traj_setpoint_ned->velocity[1]=vel_ned.pose.position.y;
        traj_setpoint_ned->velocity[2]=vel_ned.pose.position.z;

      }   

      traj_setpoint_ned_pub_vect_[target_system_id-1]->publish(*traj_setpoint_ned);                                               

      RCLCPP_DEBUG(this->get_logger(),"topics_conversion node: trajectory_setpoint_enu_callback of drone with id : %d is x: %f , y: %f, z: %f",target_system_id,msg.position[0],msg.position[1],msg.position[2]);
    }
     catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
void unbiase_vehicle_odometry_callback(int target_system_id,const VehicleOdometry & msg){
    try{

      geometry_msgs::msg::TransformStamped tf_base2station_ned;
      rclcpp::Time stamp_ros(static_cast<uint64_t>(msg.timestamp) * 1000ULL,RCL_SYSTEM_TIME);   // <- same clock type RViz uses
      tf_base2station_ned.header.stamp    = stamp_ros;
      tf_base2station_ned.header.frame_id = "station_ned";
      std::string source_frame="drone10" + std::to_string(target_system_id) + "/base_link";
      tf_base2station_ned.child_frame_id  = source_frame;

      tf_base2station_ned.transform.translation.x = msg.position[0];
      tf_base2station_ned.transform.translation.y = msg.position[1];
      tf_base2station_ned.transform.translation.z = msg.position[2];
      tf_base2station_ned.transform.rotation.x    = msg.q[1];
      tf_base2station_ned.transform.rotation.y    = msg.q[2];
      tf_base2station_ned.transform.rotation.z    = msg.q[3];
      tf_base2station_ned.transform.rotation.w    = msg.q[0];

      tf_broadcaster_->sendTransform(tf_base2station_ned);
      tf_buffer_traj_.setTransform(tf_base2station_ned, "topics_conversion_node");

      //Transform pose into map_enu
      geometry_msgs::msg::PoseStamped pose_ned,pose_map;
      pose_ned.header.stamp = stamp_ros;
      pose_ned.header.frame_id = "station_ned";
      pose_ned.pose.position.x = msg.position[0];
      pose_ned.pose.position.y = msg.position[1];
      pose_ned.pose.position.z = msg.position[2];
      pose_ned.pose.orientation.x = msg.q[1];
      pose_ned.pose.orientation.y = msg.q[2];
      pose_ned.pose.orientation.z = msg.q[3];
      pose_ned.pose.orientation.w = msg.q[0];
      // auto pose_map = tf_buffer_.transform(pose_ned, "map_enu", tf2::durationFromSec(0.05));
      auto base2map = tf_buffer_traj_.lookupTransform(
                                                        "map_enu",          // target
                                                        "station_ned", // source ("baselink")
                                                        tf2::TimePointZero,
                                                        tf2::durationFromSec(0.05));
      tf2::doTransform(pose_ned, pose_map, base2map);
      //Transform twist into map_enu
      geometry_msgs::msg::PoseStamped vel_ned,vel_map; // the tf_buffer_.transform is not accepting twist_msg

      vel_ned.header = pose_ned.header;
      vel_ned.pose.position.x = msg.velocity[0];
      vel_ned.pose.position.y = msg.velocity[1];
      vel_ned.pose.position.z = msg.velocity[2];
      vel_ned.pose.orientation.x = msg.q[1];
      vel_ned.pose.orientation.y = msg.q[2];
      vel_ned.pose.orientation.z = msg.q[3];
      vel_ned.pose.orientation.w = msg.q[0];
      //geometry_msgs::msg::TwistStamped twist_map;
      tf2::doTransform(vel_ned, vel_map, base2map);
      VehicleOdometry odom_enu = msg;  // copy all other fields
      odom_enu.position[0] = pose_map.pose.position.x;
      odom_enu.position[1] = pose_map.pose.position.y;
      odom_enu.position[2] = pose_map.pose.position.z;
      odom_enu.q[0] = pose_map.pose.orientation.w;
      odom_enu.q[1] = pose_map.pose.orientation.x;
      odom_enu.q[2] = pose_map.pose.orientation.y;
      odom_enu.q[3] = pose_map.pose.orientation.z;
      // override linear velocity
      odom_enu.velocity[0] = vel_map.pose.position.x;//twist.linear.x
      odom_enu.velocity[1] = vel_map.pose.position.y;//twist.linear.y
      odom_enu.velocity[2] = vel_map.pose.position.z;//twist.linear.z
      unbiase_vehicle_odometry_enu_pub_vect_[target_system_id-1]->publish(odom_enu);
                                                

      RCLCPP_DEBUG(this->get_logger(),"topics_conversion node: unbiase position of drone with id : %d is x: %f , y: %f, z: %f",target_system_id,msg.position[0],msg.position[1],msg.position[2]);
    }
     catch(std::exception &ex){
        RCLCPP_WARN_STREAM(this->get_logger(), ex.what());
    }
}
private:
std::string run_mode;
std::vector<rclcpp::Subscription<VehicleOdometry>::SharedPtr> unbiase_vehicle_odometry_ned_sub_vect_;
std::vector<rclcpp::Subscription<TrajectorySetpoint>::SharedPtr> traj_setpoint_enu_sub_ect_;


std::vector<rclcpp::Publisher<VehicleOdometry>::SharedPtr> unbiase_vehicle_odometry_enu_pub_vect_;
std::vector<rclcpp::Publisher<TrajectorySetpoint>::SharedPtr> traj_setpoint_ned_pub_vect_;
tf2_ros::Buffer                      tf_buffer_, tf_buffer_traj_;
tf2_ros::TransformListener           tf_listener_, tf_listener_traj_;
std::shared_ptr<tf2_ros::TransformBroadcaster>               tf_broadcaster_;
};
int main(int argc,char** argv)
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<TopicsConversion>(rclcpp::NodeOptions{});

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
