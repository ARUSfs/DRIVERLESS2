/**
 * @file camera_perception_node.cpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @date 08-03-2025
 * @brief 
 * 
 */

#include "camera_perception/camera_perception_node.hpp"

CameraPerception::CameraPerception() : Node("camera_perception")
{
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->get_parameter("image_topic", kImageTopic);
    this->declare_parameter<int>("camera_period", 100);
    this->get_parameter("camera_period", kCameraPeriod);
    this->declare_parameter<int>("camera_id", 0);
    this->get_parameter("camera_id", kCameraID);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 20);
    camera_timer_ = this->create_wall_timer(std::chrono::milliseconds(kCameraPeriod),
                    std::bind(&CameraPerception::camera_callback, this));
    camera.open(kCameraID, cv::CAP_V4L2);
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    camera.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    camera.set(cv::CAP_PROP_FPS, 15);
    std::cout << camera.get(cv::CAP_PROP_ZOOM) << std::endl;
}

void CameraPerception::camera_callback(){
    if (!camera.isOpened()){
        RCLCPP_ERROR(this->get_logger(), "Camera not opened");
        return;
    }
    cv::Mat frame;
    camera >> frame;
    if (frame.empty()){
        RCLCPP_ERROR(this->get_logger(), "Empty frame");
        return;
    }
    cv::Mat frame_resized;
    cv::resize(frame, frame_resized, cv::Size(640, 480));
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_resized).toImageMsg();
    image_pub_->publish(*msg);
    
        
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPerception>());
    rclcpp::shutdown();
    return 0;
}