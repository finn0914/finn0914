#ifndef TF_LISTENER_H
#define TF_LISTENER_H

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace stationui_frontend_rviz_plugin
{

class TfListener
{
public:
    TfListener(rclcpp::Node::SharedPtr rosnode = nullptr);

public:
    using PointStamped = geometry_msgs::msg::PointStamped;
    using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;
    PointStamped px4_to_enu(PointStamped& msg) const;
    PointStamped enu_to_px4(PointStamped& msg) const;
    QuaternionStamped px4_to_enu_quat(QuaternionStamped& msg) const;
    QuaternionStamped enu_to_px4_quat(QuaternionStamped& msg) const;

    static void my_getRPY(tf2::Quaternion q, double& roll_out, double& pitch_out, double& yaw_out);

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}

#endif // TF_LISTENER_H
