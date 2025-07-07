#include "station_pointcloud/to_point_cloud_core.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto topointcloud=std::make_shared<ToPointCloud>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(topointcloud);
    RCLCPP_INFO(topointcloud->get_logger(),"ToPointCloud node is started!");
    exec.spin();
    return 0;
}
