#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <Eigen/Geometry>  // for Isometry3d
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class DroneRviz : public rclcpp::Node
{
public:
  DroneRviz();
  ~DroneRviz();
  void init_commons();
  void lateInit();

  double accum_error_101_=0,accum_error_102_=0,accum_error_103_=0;    
  rviz_visual_tools::RvizVisualToolsPtr vt_;

private:
  // pubs & subs
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_ref_101_pub_, path_flown_101_pub_,path_ref_102_pub_, path_flown_102_pub_,path_ref_103_pub_, path_flown_103_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr metric_101_pub_,metric_102_pub_,metric_103_pub_,marker_101_pub_,marker_102_pub_,marker_103_pub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_101_sub_,odom_102_sub_,odom_103_sub_;
  rclcpp::Subscription<TrajectorySetpoint>::SharedPtr traj_setpoints_101_sub_, traj_setpoints_102_sub_, traj_setpoints_103_sub_;

  nav_msgs::msg::Path ref_path_101_,ref_path_102_,ref_path_103_;
  nav_msgs::msg::Path flown_101_,flown_102_,flown_103_;
  void vehicle_odom_unbiase_101_callback(const VehicleOdometry & msg);
  void vehicle_odom_unbiase_102_callback(const VehicleOdometry & msg);
  void vehicle_odom_unbiase_103_callback(const VehicleOdometry & msg);

  void traj_setpoins_101_callback(const TrajectorySetpoint & msg);
  void traj_setpoins_102_callback(const TrajectorySetpoint & msg);
  void traj_setpoins_103_callback(const TrajectorySetpoint & msg);

  // in class DroneRviz
  rclcpp::TimerBase::SharedPtr error_timer_,post_init_timer_;
  visualization_msgs::msg::Marker text_instant_err_101_,text_instant_err_102_,text_instant_err_103_,text_accum_error_101_,text_accum_error_102_,text_accum_error_103_;     // reused every tick
  void publish_error();
  double tracking_error(nav_msgs::msg::Path ref_path, const geometry_msgs::msg::Pose &cur) const;

  
};
