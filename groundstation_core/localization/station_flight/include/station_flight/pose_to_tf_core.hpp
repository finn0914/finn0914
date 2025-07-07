#pragma once
#include "rclcpp/rclcpp.hpp"
#include "station_flight/msg/xyzrpy_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
class PoseToTF:public rclcpp::Node {

    public:
    PoseToTF();
    ~PoseToTF();
    rclcpp::Subscription<station_flight::msg::XyzrpyStamped>::SharedPtr sub_xyzrpy_stamped_101_;
    rclcpp::Subscription<station_flight::msg::XyzrpyStamped>::SharedPtr sub_xyzrpy_stamped_102_;
    rclcpp::Subscription<station_flight::msg::XyzrpyStamped>::SharedPtr sub_xyzrpy_stamped_103_;
    void xyzrpy_stamped_101_cb(const station_flight::msg::XyzrpyStamped::SharedPtr msg);
    void xyzrpy_stamped_102_cb(const station_flight::msg::XyzrpyStamped::SharedPtr msg);
    void xyzrpy_stamped_103_cb(const station_flight::msg::XyzrpyStamped::SharedPtr msg);

    private:
    void handle_drone_tf(const station_flight::msg::XyzrpyStamped msg,std::string drone_name );
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;



};