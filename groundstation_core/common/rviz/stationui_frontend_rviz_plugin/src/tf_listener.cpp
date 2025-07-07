#include "tf_listener.hpp"

namespace stationui_frontend_rviz_plugin
{

TfListener::TfListener(rclcpp::Node::SharedPtr rosnode)
{
    node_ = rosnode;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    rclcpp::sleep_for(std::chrono::seconds(1));  // Ensure listener has time to receive transforms.
}

using PointStamped = geometry_msgs::msg::PointStamped;
using QuaternionStamped = geometry_msgs::msg::QuaternionStamped;
PointStamped TfListener::px4_to_enu(PointStamped& msg) const
{
    msg.header.frame_id = "map_px4";
    return tf_buffer_->transform(msg, "map_enu"); // assume nothrow
}
PointStamped TfListener::enu_to_px4(PointStamped& msg) const
{
    msg.header.frame_id = "map_enu";
    return tf_buffer_->transform(msg, "map_px4"); // assume nothrow
}
// void MainPanel::px4_to_enu_rpy(const float* in_rpy_px4, const float* out_rpy_enu) const
QuaternionStamped TfListener::px4_to_enu_quat(QuaternionStamped& msg) const
{
    msg.header.frame_id = "map_px4";
    return tf_buffer_->transform(msg, "map_enu"); // assume nothrow
}
// void MainPanel::enu_to_px4_rpy(const float* in_rpy_enu, const float* out_rpy_px4) const
QuaternionStamped TfListener::enu_to_px4_quat(QuaternionStamped& msg) const
{
    msg.header.frame_id = "map_enu";
    return tf_buffer_->transform(msg, "map_px4"); // assume nothrow
}

void TfListener::my_getRPY(tf2::Quaternion q, double& roll_out, double& pitch_out, double& yaw_out)
{
    tf2::Matrix3x3 m(q);
    m.getRPY(roll_out, pitch_out, yaw_out);
}

}
