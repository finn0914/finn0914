#include "station_flight/pose_to_tf_core.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

PoseToTF::~PoseToTF() {}

PoseToTF::PoseToTF() : Node("pose_to_tf_node")
{
    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize subscriptions
    sub_xyzrpy_stamped_101_ = this->create_subscription<station_flight::msg::XyzrpyStamped>(
        "/station/drone101/xyzrpy_stamped",
        10, std::bind(&PoseToTF::xyzrpy_stamped_101_cb, this, std::placeholders::_1)
    );

    sub_xyzrpy_stamped_102_ = this->create_subscription<station_flight::msg::XyzrpyStamped>(
        "/station/drone102/xyzrpy_stamped",
        10, std::bind(&PoseToTF::xyzrpy_stamped_102_cb, this, std::placeholders::_1)
    );

    sub_xyzrpy_stamped_103_ = this->create_subscription<station_flight::msg::XyzrpyStamped>(
        "/station/drone103/xyzrpy_stamped",
        10, std::bind(&PoseToTF::xyzrpy_stamped_103_cb, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "initialized!");
}

void PoseToTF::xyzrpy_stamped_101_cb(const station_flight::msg::XyzrpyStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "drone101: msg.x: %f, msg.y: %f, msg.z: %f, msg.roll: %f, msg.pitch: %f, msg.yaw: %f ",
                                    msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw);
    handle_drone_tf(*msg, "drone101");
}

void PoseToTF::xyzrpy_stamped_102_cb(const station_flight::msg::XyzrpyStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "drone102: msg.x: %f, msg.y: %f, msg.z: %f, msg.roll: %f, msg.pitch: %f, msg.yaw: %f ",
                                    msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw);
    handle_drone_tf(*msg, "drone102");
}

void PoseToTF::xyzrpy_stamped_103_cb(const station_flight::msg::XyzrpyStamped::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "drone103: msg.x: %f, msg.y: %f, msg.z: %f, msg.roll: %f, msg.pitch: %f, msg.yaw: %f ",
                                    msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw);
    handle_drone_tf(*msg, "drone103");
}

void PoseToTF::handle_drone_tf(const station_flight::msg::XyzrpyStamped msg, std::string drone_name)
{
    try{
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map_neu";
        t.child_frame_id = "/"+drone_name+"/base_link";

        t.transform.translation.x = msg.x;
        t.transform.translation.y = msg.y;
        t.transform.translation.z = msg.z;

        tf2::Quaternion q;
        q.setRPY(msg.roll, msg.pitch, msg.yaw);

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);

        RCLCPP_INFO(this->get_logger(), "Transform sent for %s", drone_name.c_str());
    }
    catch(tf2::TransformException &ex){
          RCLCPP_ERROR(this->get_logger(),"[station_flight][handle_drone_tf] tf from drone to base_link is not set: %s", ex.what());   
}
}
