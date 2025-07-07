#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>

class DroneTFBroadcaster : public rclcpp::Node {
public:
    DroneTFBroadcaster() : Node("drone_tf_broadcaster") {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/stationui_backend/drone101/recv_pose", 10,
            std::bind(&DroneTFBroadcaster::pose_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Started TF broadcaster for drone101");
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_;

    void pose_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform;

        // 將 PX4 timestamp 轉換為 ROS2 的 builtin_interfaces::msg::Time
        builtin_interfaces::msg::Time stamp_ros;
        stamp_ros.sec = static_cast<int32_t>(msg->timestamp / 1000000ULL);
        stamp_ros.nanosec = static_cast<uint32_t>((msg->timestamp % 1000000ULL) * 1000);
        transform.header.stamp = stamp_ros;

        transform.header.frame_id = "map_px4";
        transform.child_frame_id = "drone101/base_link";

        // 位移直接從 PX4 傳回的 position[] 使用
        transform.transform.translation.x = msg->position[0];
        transform.transform.translation.y = msg->position[1];
        transform.transform.translation.z = msg->position[2];

        // 將 PX4 的 NED frame 的 quaternion 轉為 ROS ENU frame
        tf2::Quaternion q_ned(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);

        // NED → ENU: 對應的是 roll=180°, yaw=180°
        tf2::Quaternion ned_to_enu;
        ned_to_enu.setRPY(M_PI, 0, M_PI);

        tf2::Quaternion q_enu = ned_to_enu * q_ned;
        q_enu.normalize();

        transform.transform.rotation.x = q_enu.x();
        transform.transform.rotation.y = q_enu.y();
        transform.transform.rotation.z = q_enu.z();
        transform.transform.rotation.w = q_enu.w();

        tf_broadcaster_->sendTransform(transform);
        RCLCPP_INFO(this->get_logger(), "✅ Sent TF from map_px4 → drone101/base_link");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}
