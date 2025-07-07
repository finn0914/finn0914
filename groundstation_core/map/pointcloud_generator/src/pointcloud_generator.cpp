#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class RGBDToPointCloud : public rclcpp::Node {
public:
    RGBDToPointCloud() : Node("rgbd_to_pointcloud") {
        RCLCPP_INFO(this->get_logger(), "Node started and waiting for synchronized messages...");

        rgb_sub_.subscribe(this, "/station/drone101/camera/color/image");
        depth_sub_.subscribe(this, "/station/drone101/camera/depth/image");

        sync_ = std::make_shared<message_filters::Synchronizer<ApproxSyncPolicy>>(ApproxSyncPolicy(10), rgb_sub_, depth_sub_);
        sync_->registerCallback(std::bind(&RGBDToPointCloud::sync_callback, this, std::placeholders::_1, std::placeholders::_2));

        pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
    }

private:
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    using ApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    std::shared_ptr<message_filters::Synchronizer<ApproxSyncPolicy>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

    void sync_callback(const sensor_msgs::msg::Image::ConstSharedPtr rgb_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr depth_msg) {
        RCLCPP_INFO(this->get_logger(), "✅ sync_callback triggered");

        auto rgb_img = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;
        auto depth_img = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;

        int width = depth_img.cols;
        int height = depth_img.rows;

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->get_clock()->now();  // ✅ 用 ROS 時間，避免 TF 錯誤
        cloud_msg.header.frame_id = "drone101/base_link";   // ✅ 這要對應 TF 中 child_frame_id
        cloud_msg.height = height;
        cloud_msg.width = width;
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(width * height);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

        // 相機內參（建議用你的真實值）
        float fx = 615.0;
        float fy = 615.0;
        float cx = 320.0;
        float cy = 240.0;

        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
                uint16_t depth = depth_img.at<uint16_t>(v, u);
                if (depth == 0) {
                    *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
                    *iter_r = *iter_g = *iter_b = 0;
                    continue;
                }

                float z = depth * 0.001f; // mm → m
                float x = (u - cx) * z / fx;
                float y = (v - cy) * z / fy;

                *iter_x = x;
                *iter_y = y;
                *iter_z = z;

                cv::Vec3b color = rgb_img.at<cv::Vec3b>(v, u);
                *iter_r = color[2];
                *iter_g = color[1];
                *iter_b = color[0];
            }
        }

        RCLCPP_INFO(this->get_logger(), "📡 Publishing PointCloud2");
        pc_pub_->publish(cloud_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RGBDToPointCloud>());
    rclcpp::shutdown();
    return 0;
}
