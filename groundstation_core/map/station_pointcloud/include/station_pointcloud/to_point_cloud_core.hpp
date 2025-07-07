#pragma once

#include <rclcpp/rclcpp.hpp>

#include <station_flight/msg/xyzrpy_stamped.hpp>
#include <deque>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <limits>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <atomic>
// #include <thread.hpp>

class ToPointCloud : public rclcpp::Node
{
public:
    ToPointCloud();
    ~ToPointCloud();
        /*
    threads
    */
    std::atomic<bool> pcd_101_ready_,pcd_102_ready_,pcd_103_ready_;
    void concatenated_pointcloud();

private:
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::string target_frame="map";
    /*
        Loaded parameters
    */    
    // sensor_msgs::msg::CameraInfo::SharedPtr realsense_intrinsic;
    float max_depth_; // For point cloud. In meters.
    float fx_, fy_, cx_, cy_; // Camara intrinsic parameters.
    float threshold_color_depth_;
    /*
        Subscriber and publisher callbacks
    */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_101_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_102_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_color_103_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_101_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_102_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_103_;
    rclcpp::Subscription<station_flight::msg::XyzrpyStamped>::SharedPtr sub_xyzrpy_stamped_101_;
    rclcpp::Subscription<station_flight::msg::XyzrpyStamped>::SharedPtr sub_xyzrpy_stamped_102_;
    rclcpp::Subscription<station_flight::msg::XyzrpyStamped>::SharedPtr sub_xyzrpy_stamped_103_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_101_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_102_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud_103_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_concatenated_point_cloud_;

    
    std::deque<sensor_msgs::msg::Image::SharedPtr> color_msgs_101_;
    std::deque<sensor_msgs::msg::Image::SharedPtr> color_msgs_102_;
    std::deque<sensor_msgs::msg::Image::SharedPtr> color_msgs_103_;
    std::deque<sensor_msgs::msg::Image::SharedPtr> depth_msgs_101_;
    std::deque<sensor_msgs::msg::Image::SharedPtr> depth_msgs_102_;
    std::deque<sensor_msgs::msg::Image::SharedPtr> depth_msgs_103_;
    // cv::Mat depth_image_;
    // cv::Mat rgb_image_;
    station_flight::msg::XyzrpyStamped xyzrpy_stamped_103_,xyzrpy_stamped_102_,xyzrpy_stamped_101_;


    std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloud_101_msg_,pointcloud_102_msg_,pointcloud_103_msg_;
/* callbacks for image and depth from realsense*/
    void color_101_cb(const sensor_msgs::msg::Image::SharedPtr msg);
    void depth_101_cb(const sensor_msgs::msg::Image::SharedPtr msg);
    void color_102_cb(const sensor_msgs::msg::Image::SharedPtr msg);
    void depth_102_cb(const sensor_msgs::msg::Image::SharedPtr msg);
    void color_103_cb(const sensor_msgs::msg::Image::SharedPtr msg);
    void depth_103_cb(const sensor_msgs::msg::Image::SharedPtr msg);
        /*
        Functions
    */
    long long int nanoseconds(builtin_interfaces::msg::Time stamp);
    void fail_to_load_param(const char* param_name);

    // Returns false on failure.
    bool find_nearest_pose_msg (
        std::deque<station_flight::msg::XyzrpyStamped>& pose_queue,
        const builtin_interfaces::msg::Time stamp,
        station_flight::msg::XyzrpyStamped& out);

    // Returns false on failure.
    bool find_nearest_color_msg (
        std::deque<sensor_msgs::msg::Image::SharedPtr>& color_queue,
        const builtin_interfaces::msg::Time stamp,
        sensor_msgs::msg::Image::SharedPtr& out);

    std::shared_ptr<sensor_msgs::msg::PointCloud2> convert_to_point_cloud(
                                                                const std::deque<sensor_msgs::msg::Image::SharedPtr>& color_msg,
                                                                const std::deque<sensor_msgs::msg::Image::SharedPtr>& depth_msg,
                                                                const station_flight::msg::XyzrpyStamped xyzrpy_stamped, std::string drone_name);
    bool transform_pointcloud_to_frame_id(const std::shared_ptr<sensor_msgs::msg::PointCloud2> in_pointcloud_msg,
                                                    std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> out_pointcloud,
                                                    const std::string &fromFrameRel,const std::string &toFrameRel);
};
