#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include<thread>
#include "sensor_msgs/msg/image.hpp"

class ImageReceiver : public rclcpp::Node {
public:
    ImageReceiver()
    : Node("image_receiver") {
        // Use a timer to defer initialization of the publisher and TCP connection
        initialize();
    }

private:
    void initialize() {

        drone103_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/station/drone103/camera/color/image", 10);
        drone102_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/station/drone102/camera/color/image", 10);
        drone101_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/station/drone101/camera/color/image", 10);
        drone101_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/station/drone101/camera/depth/image", 10);
        drone102_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/station/drone102/camera/depth/image", 10);
        drone103_depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/station/drone103/camera/depth/image", 10);

        RCLCPP_INFO(this->get_logger(), "1..");
        // Proceed to receive image
        thread_101 = std::thread(&ImageReceiver::receive_image_101, this);       
        thread_102 = std::thread(&ImageReceiver::receive_image_102, this);
        thread_103 = std::thread(&ImageReceiver::receive_image_103, this);

    }

    void receive_image_101() {
        try {
                    // RCLCPP_INFO(this->get_logger(), "12..");

            boost::asio::io_service io_service;
            boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), 12341);
            boost::asio::ip::tcp::acceptor acceptor(io_service, endpoint);
            while(rclcpp::ok()){            
                boost::asio::ip::tcp::socket socket(io_service);
                acceptor.accept(socket);
                size_t header;
                uchar message_type_depth=2,message_type_color=1;
                boost::asio::read(socket, boost::asio::buffer(&header, sizeof(header)));
                if(header==0)
                {
                    RCLCPP_WARN(this->get_logger(),"drone101-> tcp is empty ");
                    continue;
                }
                std::vector<uchar> buf(header);
                boost::asio::read(socket, boost::asio::buffer(buf));
                // for (int i=0;i<3;i++){
                //     std::cout << "received Buffer: " << static_cast<int>(buf[i]) << std::endl;
                // }
                if (static_cast<int>(buf[0])==message_type_depth){
                    uint32_t sec = *reinterpret_cast<uint32_t*>(&buf[1]);//buf[0] message type buf[1,9] is time , rest is camera info 
                    uint32_t nsec = *reinterpret_cast<uint32_t*>(&buf[1+sizeof(uint32_t)]);
                    buf.erase(buf.begin(), buf.begin() + 9);


                    RCLCPP_INFO(this->get_logger(), "drone101->Depth Timestamp received: '%u.%u'", sec, nsec);

                    // std::cout << "message_type_depth->received Buffer after erase: " << static_cast<int>(buf[0]) << std::endl;
                    cv::Mat image = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
                    // RCLCPP_INFO(this->get_logger(), "drone103->Received image with type: %d and channels: %d", image.type(), image.channels());
                    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", image).toImageMsg();
                    msg->header.stamp.sec = sec;
                    msg->header.stamp.nanosec = nsec;
                    // msg->header.stamp=this->now();
                    msg->header.frame_id="drone101/camera_link";
                    drone101_depth_pub_->publish(*msg);
                }
                else if (static_cast<int>(buf[0])==message_type_color)
                {
                    uint32_t sec = *reinterpret_cast<uint32_t*>(&buf[1]);//buf[0] message type buf[1,9] is time , rest is camera info 
                    uint32_t nsec = *reinterpret_cast<uint32_t*>(&buf[1+sizeof(uint32_t)]);
                    buf.erase(buf.begin(), buf.begin() + 9);                    // std::cout << "message_type_color->received Buffer after erase: " << static_cast<int>(buf[0]) << std::endl;
                    
                    
                    RCLCPP_INFO(this->get_logger(), "drone101->color Timestamp received: '%u.%u'", sec, nsec);

                    
                    cv::Mat image = cv::imdecode(buf, cv::IMREAD_COLOR);
                    // RCLCPP_INFO(this->get_logger(), "drone103->Received image with type: %d and channels: %d", image.type(), image.channels());
                    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                    msg->header.stamp.sec = sec;
                    msg->header.stamp.nanosec = nsec;
                    // msg->header.stamp=this->now();
                    msg->header.frame_id="drone101/camera_link";
                    drone101_image_pub_->publish(*msg);  
                }
            else{
                RCLCPP_WARN(this->get_logger(), "drone101->message_type is wrong");
            }

        }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "drone101->Exception: %s", e.what());
        }
    }
    void receive_image_102() {
        try {
            boost::asio::io_service io_service;
            boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), 12342);
            boost::asio::ip::tcp::acceptor acceptor(io_service, endpoint);
            while(rclcpp::ok()){            
                boost::asio::ip::tcp::socket socket(io_service);
                acceptor.accept(socket);
                size_t header;
                uchar message_type_depth=2,message_type_color=1;
                boost::asio::read(socket, boost::asio::buffer(&header, sizeof(header)));
                if(header==0)
                {
                    RCLCPP_WARN(this->get_logger(),"drone102-> tcp is empty ");
                    continue;
                }
                std::vector<uchar> buf(header);
                boost::asio::read(socket, boost::asio::buffer(buf));
                std::cout << "Image size: " << sizeof(header) << " bytes" <<", message_type: "<<message_type_depth<< std::endl;
                // for (int i=0;i<3;i++){
                //     std::cout << "received Buffer: " << static_cast<int>(buf[i]) << std::endl;
                // }
                if (static_cast<int>(buf[0])==message_type_depth){
                    uint32_t sec = *reinterpret_cast<uint32_t*>(&buf[1]);//buf[0] message type buf[1,9] is time , rest is camera info 
                    uint32_t nsec = *reinterpret_cast<uint32_t*>(&buf[1+sizeof(uint32_t)]);
                    buf.erase(buf.begin(), buf.begin() + 9);


                    RCLCPP_INFO(this->get_logger(), "drone102->Depth Timestamp received: '%u.%u'", sec, nsec);

                    // std::cout << "message_type_depth->received Buffer after erase: " << static_cast<int>(buf[0]) << std::endl;
                    cv::Mat image = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
                    // RCLCPP_INFO(this->get_logger(), "drone103->Received image with type: %d and channels: %d", image.type(), image.channels());
                    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", image).toImageMsg();
                    msg->header.stamp.sec = sec;
                    msg->header.stamp.nanosec = nsec;
                    // msg->header.stamp=this->now();
                    msg->header.frame_id="drone102/camera_link";
                    drone102_depth_pub_->publish(*msg);
                }
                else if (static_cast<int>(buf[0])==message_type_color)
                {
                    uint32_t sec = *reinterpret_cast<uint32_t*>(&buf[1]);//buf[0] message type buf[1,9] is time , rest is camera info 
                    uint32_t nsec = *reinterpret_cast<uint32_t*>(&buf[1+sizeof(uint32_t)]);
                    buf.erase(buf.begin(), buf.begin() + 9);                    // std::cout << "message_type_color->received Buffer after erase: " << static_cast<int>(buf[0]) << std::endl;
                    
                    
                    RCLCPP_INFO(this->get_logger(), "drone102->color Timestamp received: '%u.%u'", sec, nsec);

                    
                    cv::Mat image = cv::imdecode(buf, cv::IMREAD_COLOR);
                    // RCLCPP_INFO(this->get_logger(), "drone103->Received image with type: %d and channels: %d", image.type(), image.channels());
                    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                    msg->header.stamp.sec = sec;
                    msg->header.stamp.nanosec = nsec;
                    // msg->header.stamp=this->now();
                    msg->header.frame_id="drone102/camera_link";
                    drone102_image_pub_->publish(*msg);  
                }
            else{
                RCLCPP_WARN(this->get_logger(), "drone102->message_type wrong");
            }

        }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "drone102->Exception: %s", e.what());
        }
    }
    void receive_image_103() {
        try {
                    // RCLCPP_INFO(this->get_logger(), "12..");

            boost::asio::io_service io_service;
            boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), 12343);
            boost::asio::ip::tcp::acceptor acceptor(io_service, endpoint);
            while(rclcpp::ok()){            
                boost::asio::ip::tcp::socket socket(io_service);
                acceptor.accept(socket);
                size_t header;
                uchar message_type_depth=2,message_type_color=1;
                boost::asio::read(socket, boost::asio::buffer(&header, sizeof(header)));
                if(header==0)
                {
                    RCLCPP_WARN(this->get_logger(),"drone103-> tcp is empty ");
                    continue;
                }
                std::vector<uchar> buf(header);
                boost::asio::read(socket, boost::asio::buffer(buf));
                // for (int i=0;i<3;i++){
                //     std::cout << "received Buffer: " << static_cast<int>(buf[i]) << std::endl;
                // }
                if (static_cast<int>(buf[0])==message_type_depth){
                    uint32_t sec = *reinterpret_cast<uint32_t*>(&buf[1]);//buf[0] message type buf[1,9] is time , rest is camera info 
                    uint32_t nsec = *reinterpret_cast<uint32_t*>(&buf[1+sizeof(uint32_t)]);
                    buf.erase(buf.begin(), buf.begin() + 9);


                    RCLCPP_INFO(this->get_logger(), "drone103->Depth Timestamp received: '%u.%u'", sec, nsec);

                    // std::cout << "message_type_depth->received Buffer after erase: " << static_cast<int>(buf[0]) << std::endl;
                    cv::Mat image = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
                    // RCLCPP_INFO(this->get_logger(), "drone103->Received image with type: %d and channels: %d", image.type(), image.channels());
                    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1", image).toImageMsg();
                    msg->header.stamp.sec = sec;
                    msg->header.stamp.nanosec = nsec;
                    // msg->header.stamp=this->now();
                    msg->header.frame_id="drone103/camera_link";
                    drone103_depth_pub_->publish(*msg);
                }
                else if (static_cast<int>(buf[0])==message_type_color)
                {
                    uint32_t sec = *reinterpret_cast<uint32_t*>(&buf[1]);//buf[0] message type buf[1,9] is time , rest is camera info 
                    uint32_t nsec = *reinterpret_cast<uint32_t*>(&buf[1+sizeof(uint32_t)]);
                    buf.erase(buf.begin(), buf.begin() + 9);                    // std::cout << "message_type_color->received Buffer after erase: " << static_cast<int>(buf[0]) << std::endl;
                    
                    
                    RCLCPP_INFO(this->get_logger(), "drone103->color Timestamp received: '%u.%u'", sec, nsec);

                    
                    cv::Mat image = cv::imdecode(buf, cv::IMREAD_COLOR);
                    // RCLCPP_INFO(this->get_logger(), "drone103->Received image with type: %d and channels: %d", image.type(), image.channels());
                    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
                    msg->header.stamp.sec = sec;
                    msg->header.stamp.nanosec = nsec;
                    // msg->header.stamp=this->now();
                    msg->header.frame_id="drone103/camera_link";
                    drone103_image_pub_->publish(*msg);  
                }
            else{
                RCLCPP_WARN(this->get_logger(), "drone103->message_type is wrong");
            }

        }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "drone103->Exception: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr drone103_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr drone102_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr drone101_image_pub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr drone101_depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr drone102_depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr drone103_depth_pub_;

    std::thread thread_103;
    std::thread thread_102;
    std::thread thread_101;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageReceiver>());
    rclcpp::shutdown();
    return 0;
}
