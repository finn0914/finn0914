#include "station_pointcloud/to_point_cloud_core.hpp"

ToPointCloud::~ToPointCloud(){}
ToPointCloud::ToPointCloud() : Node("point_cloud_converter"),pcd_101_ready_(false),pcd_102_ready_(false),pcd_103_ready_(false)
{
    // Load parameters
    declare_parameter<float>("max_depth");
    declare_parameter<float>("fx");
    declare_parameter<float>("fy");
    declare_parameter<float>("cx");
    declare_parameter<float>("cy");
    declare_parameter<float>("threshold_color_depth_topics");

    if (!get_parameter("max_depth", max_depth_)) fail_to_load_param("max_depth");
    if (!get_parameter("fx", fx_)) fail_to_load_param("fx");
    if (!get_parameter("fy", fy_)) fail_to_load_param("fy");
    if (!get_parameter("cx", cx_)) fail_to_load_param("cx");
    if (!get_parameter("cy", cy_)) fail_to_load_param("cy");
    if (!get_parameter("threshold_color_depth_topics", threshold_color_depth_)) fail_to_load_param("threshold_color_depth_topics");

    xyzrpy_stamped_101_.x=5.0;
    xyzrpy_stamped_101_.y=5.0;
    xyzrpy_stamped_101_.z=0.0;
    xyzrpy_stamped_101_.roll=0.0;
    xyzrpy_stamped_101_.pitch=0.0;
    xyzrpy_stamped_101_.yaw=0.0;
    xyzrpy_stamped_102_.x=-5.0;
    xyzrpy_stamped_102_.y=-5.0;
    xyzrpy_stamped_102_.z=0.0;
    xyzrpy_stamped_102_.roll=0.0;
    xyzrpy_stamped_102_.pitch=0.0;
    xyzrpy_stamped_102_.yaw=0.0;
    xyzrpy_stamped_103_.x=0.0;
    xyzrpy_stamped_103_.y=0.0;
    xyzrpy_stamped_103_.z=0.0;
    xyzrpy_stamped_103_.roll=0.0;
    xyzrpy_stamped_103_.pitch=0.0;
    xyzrpy_stamped_103_.yaw=0.0;
    RCLCPP_INFO_STREAM(this->get_logger(),"threshold_color_depth: "<< threshold_color_depth_ << ", x:"<<xyzrpy_stamped_103_.x<<", y:"<<xyzrpy_stamped_103_.y<<", z:"<<xyzrpy_stamped_103_.z);
    // Subscribe to RGB and depth image
    sub_color_101_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/station/drone101/camera/color/image",
        10, std::bind(&ToPointCloud::color_101_cb, this, std::placeholders::_1)
    );
    sub_depth_101_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/station/drone101/camera/depth/image",
        10, std::bind(&ToPointCloud::depth_101_cb, this, std::placeholders::_1)
    );
    sub_color_102_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/station/drone102/camera/color/image",
        10, std::bind(&ToPointCloud::color_102_cb, this, std::placeholders::_1)
    );
    sub_depth_102_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/station/drone102/camera/depth/image",
        10, std::bind(&ToPointCloud::depth_102_cb, this, std::placeholders::_1)
    );
    sub_color_103_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/station/drone103/camera/color/image",
        10, std::bind(&ToPointCloud::color_103_cb, this, std::placeholders::_1)
    );
    sub_depth_103_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/station/drone103/camera/depth/image",
        10, std::bind(&ToPointCloud::depth_103_cb, this, std::placeholders::_1)
    );

    // Subscribe to xyzrpy_stamped
    // sub_xyzrpy_stamped_101 = this->create_subscription<station_flight::msg::XyzrpyStamped>(
    //     "/station/drone101/xyzrpy_stamped",
    //     10, std::bind(&ToPointCloud::xyzrpy_stamped_101_cb, this, std::placeholders::_1)
    // );
    // sub_xyzrpy_stamped_102 = this->create_subscription<station_flight::msg::XyzrpyStamped>(
    //     "/station/drone102/xyzrpy_stamped",
    //     10, std::bind(&ToPointCloud::xyzrpy_stamped_102_cb, this, std::placeholders::_1)
    // sub_xyzrpy_stamped_103 = this->create_subscription<station_flight::msg::XyzrpyStamped>(
    //     "/station/drone103/xyzrpy_stamped",
    //     10, std::bind(&ToPointCloud::xyzrpy_stamped_103_cb, this, std::placeholders::_1)
    // );
    
    // Publisher for the point cloud
    pub_point_cloud_101_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/station/drone101/point_cloud2", 10
    );
    pub_point_cloud_102_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/station/drone102/point_cloud2", 10
    );
    pub_point_cloud_103_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/station/drone103/point_cloud2", 10
    );
    pub_concatenated_point_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/station/concatenated/point_cloud2", 10
    );
    tf_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    std::thread(&ToPointCloud::concatenated_pointcloud,this).detach();
}


void ToPointCloud::fail_to_load_param(const char* param_name) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load parameter %s", param_name);
    rclcpp::shutdown();
    throw std::runtime_error("Failed to load parameter");
}

/* callbacks for image and depth from realsense*/
void ToPointCloud::color_101_cb(const sensor_msgs::msg::Image::SharedPtr msg){
    try{
        color_msgs_101_.push_back(msg);
        if (depth_msgs_101_.empty())
            return;
        else{

            pointcloud_101_msg_=ToPointCloud::convert_to_point_cloud(color_msgs_101_,depth_msgs_101_,xyzrpy_stamped_101_,"drone101");
            pointcloud_101_msg_->header.frame_id="drone101/camera_depth_tf";
            color_msgs_101_.clear();
            depth_msgs_101_.clear();
            pcd_101_ready_=true;
            pointcloud_101_msg_ ? pub_point_cloud_101_->publish(*pointcloud_101_msg_): void();  
        }

    }
    catch(std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"color_101_cb-> Exception in Callback : %s",e.what());
    }

} 
void ToPointCloud::depth_101_cb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        depth_msgs_101_.push_back(msg);
        if (color_msgs_101_.empty())
            return;
        else{

            pointcloud_101_msg_=ToPointCloud::convert_to_point_cloud(color_msgs_101_,depth_msgs_101_,xyzrpy_stamped_101_,"drone101");
            pointcloud_101_msg_->header.frame_id="drone101/camera_depth_tf";
            color_msgs_101_.clear();
            depth_msgs_101_.clear();
            pcd_101_ready_=true;
            pointcloud_101_msg_ ? pub_point_cloud_101_->publish(*pointcloud_101_msg_): void();   
        }
    }
    catch(const std::exception& e){

        RCLCPP_ERROR(this->get_logger(),"depth_101_cb-> Exception in Callback : %s", e.what());

    }
}
void ToPointCloud::color_102_cb(const sensor_msgs::msg::Image::SharedPtr msg){
    try{
        color_msgs_102_.push_back(msg);
        if (depth_msgs_102_.empty())
            return;
        else{
            pointcloud_102_msg_=ToPointCloud::convert_to_point_cloud(color_msgs_102_,depth_msgs_102_,xyzrpy_stamped_102_,"drone102");
            pointcloud_102_msg_->header.frame_id="drone102/camera_depth_tf";
            color_msgs_102_.clear();
            depth_msgs_102_.clear();
            pcd_102_ready_=true;
            pointcloud_102_msg_ ? pub_point_cloud_102_->publish(*pointcloud_102_msg_): void();    
        }

    }
    catch(std::exception& e){
        RCLCPP_ERROR(this->get_logger(),"color_102_cb-> Exception in Callback : %s",e.what());
    }

}
void ToPointCloud::depth_102_cb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        depth_msgs_102_.push_back(msg);
        if (color_msgs_102_.empty())
            return;
        else{

            pointcloud_102_msg_=ToPointCloud::convert_to_point_cloud(color_msgs_102_,depth_msgs_102_,xyzrpy_stamped_102_,"drone102");
            pointcloud_102_msg_->header.frame_id="drone102/camera_depth_tf";
            color_msgs_102_.clear();
            depth_msgs_102_.clear();
            pcd_102_ready_=true;
            pointcloud_102_msg_ ? pub_point_cloud_102_->publish(*pointcloud_102_msg_): void();   
        }

    }
    catch(const std::exception& e){

        RCLCPP_ERROR(this->get_logger(),"depth_102_cb-> Exception in Callback : %s", e.what());

    }
}
void ToPointCloud::color_103_cb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        color_msgs_103_.push_back(msg);
        if (depth_msgs_103_.empty())
            return;
        else{

            pointcloud_103_msg_=ToPointCloud::convert_to_point_cloud(color_msgs_103_,depth_msgs_103_,xyzrpy_stamped_103_,"drone103");
            pointcloud_103_msg_->header.frame_id="drone103/camera_depth_tf";
            color_msgs_103_.clear();
            depth_msgs_103_.clear();
            pcd_103_ready_=true;
            pointcloud_103_msg_ ? pub_point_cloud_103_->publish(*pointcloud_103_msg_): void();     

        }

    }
    catch(const std::exception& e){

        RCLCPP_ERROR(this->get_logger(),"color_103_cb-> Exception in Callback : %s", e.what());

    }
}
void ToPointCloud::depth_103_cb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {
        depth_msgs_103_.push_back(msg);
        if (color_msgs_103_.empty())
            return;
        else{

            pointcloud_103_msg_=ToPointCloud::convert_to_point_cloud(color_msgs_103_,depth_msgs_103_,xyzrpy_stamped_103_,"drone103");
            pointcloud_103_msg_->header.frame_id="drone103/camera_depth_tf";
            color_msgs_103_.clear();
            depth_msgs_103_.clear();
            pcd_103_ready_=true;
            pointcloud_103_msg_ ? pub_point_cloud_103_->publish(*pointcloud_103_msg_): void();    
               
        }

    }
    catch(const std::exception& e){

        RCLCPP_ERROR(this->get_logger(),"depth_103_cb-> Exception in Callback : %s", e.what());

    }
}

/*
    Functions
*/

std::shared_ptr<sensor_msgs::msg::PointCloud2> ToPointCloud::convert_to_point_cloud(
    const std::deque<sensor_msgs::msg::Image::SharedPtr>& color_msg,
    const std::deque<sensor_msgs::msg::Image::SharedPtr>& depth_msg,
    const station_flight::msg::XyzrpyStamped xyzrpy_stamped, std::string drone_name)
{
    try {
        if(color_msg.empty() || depth_msg.empty() ) return nullptr;
        // RCLCPP_WARN(this->get_logger(),"color_msg.size() %d depth_msg %d",color_msg.size(),depth_msg.size());
        auto color_stamp = color_msg.at(color_msg.size()-1)->header.stamp;
        auto depth_stamp = depth_msg.at(depth_msg.size()-1)->header.stamp;
        rclcpp::Time color_time(color_stamp);
        rclcpp::Time depth_time(depth_stamp);
        auto time_difference = std::abs((color_time - depth_time).seconds());
        const double color_delay_time_sec = (this->now() - color_time).seconds();
        const double depth_delay_time_sec = (this->now() - depth_time).seconds();
        // RCLCPP_WARN(
        //         this->get_logger(),
        //         "The color_time is %lf[sec] the this->now() is %lf[sec](the tolerance is "
        //         "%lf[sec] and %lf[sec])",
        //         color_time.seconds(),this->now().seconds(), threshold_color_depth_,threshold_color_depth_);
        //TODO   |
        //      \ /
        // if (color_delay_time_sec >= threshold_color_depth_ || depth_delay_time_sec >= threshold_color_depth_)
        // {
        //     RCLCPP_WARN(
        //         this->get_logger(),
        //         "convert_to_point_cloud->The camera topics are experiencing latency. The color_delay_time_sec is %lf[sec] the depth_delay_time_sec is %lf[sec](the tolerance is "
        //         "%lf[sec] and %lf[sec])",
        //         color_delay_time_sec,depth_delay_time_sec, threshold_color_depth_,threshold_color_depth_);
        //     // return nullptr;
        // }

        // // RCLCPP_INFO_STREAM(this->get_logger(),"color and depth time_difference is :" << time_difference);
        // if(time_difference > threshold_color_depth_){
        //     RCLCPP_WARN_STREAM(this->get_logger(),"delay between depth and color topics");
        //     // return nullptr;
        // }
        cv_bridge::CvImagePtr cv_color_ptr=cv_bridge::toCvCopy(color_msg.at(color_msg.size()-1),color_msg.at(color_msg.size()-1)->encoding);
        cv_bridge::CvImagePtr cv_depth_ptr=cv_bridge::toCvCopy(depth_msg.at(depth_msg.size()-1),depth_msg.at(depth_msg.size()-1)->encoding);

        cv::Mat color_image=cv_color_ptr->image;
        cv::Mat depth_image=cv_depth_ptr->image;
        // RCLCPP_WARN(this->get_logger(),"color_image.rows %d depth_image.cols %d",depth_image.rows,depth_image.cols);

        std::shared_ptr<sensor_msgs::msg::PointCloud2>     pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // sensor_msgs::msg::PointCloud2 pointcloud_msg;
        pointcloud_msg->header.stamp=this->now();
        pointcloud_msg->header.frame_id=drone_name;
        pointcloud_msg->header = depth_msg.at(depth_msg.size()-1)->header;
        pointcloud_msg->height = depth_image.rows;
        pointcloud_msg->width = depth_image.cols;
        pointcloud_msg->is_dense = false;
        pointcloud_msg->is_bigendian = false;
        sensor_msgs::PointCloud2Modifier modifier(*pointcloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pointcloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pointcloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pointcloud_msg, "b");

        for (int v = 0; v < depth_image.rows; ++v) {
            for (int u = 0; u < depth_image.cols; ++u) {
                float depth = depth_image.at<uint16_t>(v, u) * 0.001f;  // Convert from mm to meters

                if (depth > 0.0f && depth < max_depth_) {
                    // Calculate the 3D point using the intrinsic parameters
                    *iter_x = (u - cx_) * depth / fx_;
                    *iter_y = (v - cy_) * depth / fy_;
                    *iter_z = depth;
                    // Get the color from the color image
                    cv::Vec3b color = color_image.at<cv::Vec3b>(v, u);
                    *iter_r = color[2];  // Red
                    *iter_g = color[1];  // Green
                    *iter_b = color[0];  // Blue
                } else {
                    // Invalid depth, mark as NaN
                    *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
                    *iter_r = *iter_g = *iter_b = 0;
                }
                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_r;
                ++iter_g;
                ++iter_b;
            }
        }
        return pointcloud_msg;
        // pub_point_cloud_103->publish(pointcloud_msg);
        // RCLCPP_INFO_STREAM(this->get_logger(),"after publishing the pointcloud!");
    }
    catch(const std::exception& e){

        RCLCPP_ERROR(this->get_logger(),"convert_to_point_cloud-> Exception in FC : %s", e.what());

    }
}

long long int ToPointCloud::nanoseconds(builtin_interfaces::msg::Time stamp) {
return (long long int)stamp.sec * 1000000000 + (long long int)stamp.nanosec;
}

// Returns false on failure.
bool ToPointCloud::find_nearest_pose_msg (
    std::deque<station_flight::msg::XyzrpyStamped>& pose_queue,
    const builtin_interfaces::msg::Time stamp,
    station_flight::msg::XyzrpyStamped& out) {

    if (pose_queue.empty()) {
        return false;
    }

    long long int t0 = nanoseconds(stamp);

    // Keep comparing 0-th and 1-th element in queue.
    station_flight::msg::XyzrpyStamped best_pose_msg = pose_queue[0];
    long long int best_dt = abs(nanoseconds(pose_queue[0].stamp) - t0);
    while (pose_queue.size() >= 2) {
        long long int dt = abs(nanoseconds(pose_queue[1].stamp) - t0);
        if (best_dt < dt) {
            // Found. No need to continue popping queue.
            break;
        }
        // 1-th is closer in time than the 0-th.
        best_pose_msg = pose_queue[1]; // Record 1-th.
        best_dt = dt;
        pose_queue.pop_front();        // Discard 0-th.
    }
    out = best_pose_msg;
    return true;
}

// Returns false on failure.
bool ToPointCloud::find_nearest_color_msg (
    std::deque<sensor_msgs::msg::Image::SharedPtr>& color_queue,
    const builtin_interfaces::msg::Time stamp,
    sensor_msgs::msg::Image::SharedPtr& out) {

    if (color_queue.empty()) {
        return false;
    }

    long long int t0 = nanoseconds(stamp);

    // Keep comparing 0-th and 1-th element in queue.
    sensor_msgs::msg::Image::SharedPtr best_color_msg_ptr = color_queue[0];
    long long int best_dt = abs(nanoseconds(color_queue[0]->header.stamp) - t0);
    while (color_queue.size() >= 2) {
        long long int dt = abs(nanoseconds(color_queue[1]->header.stamp) - t0);
        if (best_dt < dt) {
            // Found. No need to continue popping queue.
            break;
        }
        // 1-th is closer in time than the 0-th.
        best_color_msg_ptr = color_queue[1]; // Record 1-th.
        best_dt = dt;
        color_queue.pop_front();        // Discard 0-th.
    }
    out = best_color_msg_ptr;
    return true;
}
 
void ToPointCloud::concatenated_pointcloud(){
    try{
        while (rclcpp::ok()){
            if( pcd_101_ready_ && pcd_102_ready_ && pcd_103_ready_ ){
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud101=std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud102=std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud103=std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
                std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> concat_cloud=std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

                // transform_pointcloud_to_frame_id(pointcloud_101_msg_,cloud101,pointcloud_101_msg_->header.frame_id,target_frame);
                // transform_pointcloud_to_frame_id(pointcloud_102_msg_,cloud102,pointcloud_102_msg_->header.frame_id,target_frame);
                // transform_pointcloud_to_frame_id(pointcloud_103_msg_,cloud103,pointcloud_103_msg_->header.frame_id,target_frame);
                if (!transform_pointcloud_to_frame_id(pointcloud_101_msg_, cloud101, pointcloud_101_msg_->header.frame_id, target_frame) ||
                    !transform_pointcloud_to_frame_id(pointcloud_102_msg_, cloud102, pointcloud_102_msg_->header.frame_id, target_frame) ||
                    !transform_pointcloud_to_frame_id(pointcloud_103_msg_, cloud103, pointcloud_103_msg_->header.frame_id, target_frame)) {
                    RCLCPP_ERROR(this->get_logger(), "Error transforming point clouds.");
                    return;
                }
                *concat_cloud += *cloud101;
                *concat_cloud += *cloud102;
                *concat_cloud += *cloud103;

                std::shared_ptr<sensor_msgs::msg::PointCloud2> output=std::make_shared<sensor_msgs::msg::PointCloud2>();
                pcl::toROSMsg(*concat_cloud, *output);
                output->header.frame_id="map";
                pub_concatenated_point_cloud_->publish(*output);
                pcd_101_ready_ = pcd_102_ready_ = pcd_103_ready_ =false;
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }
    }
    catch(const std::exception & e){
        RCLCPP_ERROR(this->get_logger(),"concatenated_pointcloud-> Exception in thread : %s  ", e.what());
    }

}
bool ToPointCloud::transform_pointcloud_to_frame_id(const std::shared_ptr<sensor_msgs::msg::PointCloud2> in_pointcloud_msg,
                                                          std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> out_pointcloud,
                                                    const std::string &fromFrameRel,const std::string &toFrameRel){
         
    try {
        geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,tf2::TimePointZero);
        // in_pointcloud_msg->header.frame_id=fromFrameRel;
        sensor_msgs::msg::PointCloud2 transformed_pointcloud_ros;
        tf2::doTransform(*in_pointcloud_msg, transformed_pointcloud_ros, transform_stamped);
        pcl::fromROSMsg(transformed_pointcloud_ros, *out_pointcloud);

            // Publish the transformed point cloud
        // transformed_pointcloud_pub_->publish(transformed_pointcloud);
        return true;

    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(
        this->get_logger(), "[station_poincloud][transform_pointcloud_to_frame_id]Could not transform %s to %s: %s",
        fromFrameRel.c_str(),toFrameRel.c_str() , ex.what());
        return false;
    }
}