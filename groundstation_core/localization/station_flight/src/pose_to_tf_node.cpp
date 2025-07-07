#include "station_flight/pose_to_tf_core.hpp"



int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<PoseToTF>());
    rclcpp::shutdown();
    return 0;

}