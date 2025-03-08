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
    this->declare_parameter<bool>("show_image", false);
    this->get_parameter("show_image", kShowImage);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    camera_timer_ = this->create_wall_timer(std::chrono::milliseconds(kCameraPeriod),
                    std::bind(&CameraPerception::camera_callback, this));
    camera.open(0);
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
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    image_pub_->publish(*msg);
    if (kShowImage){
        imshow("Camera", frame);
        cv::waitKey();
    }
        
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPerception>());
    rclcpp::shutdown();
    return 0;
}