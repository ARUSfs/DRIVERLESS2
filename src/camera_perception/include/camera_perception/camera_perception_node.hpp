/**
 * @file camera_perception_node.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @date 08-03-2025
 * @brief 
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>



class CameraPerception : public rclcpp::Node
{
    public:
        CameraPerception();
    private:
        std::string kImageTopic;
        int kCameraPeriod;
        //rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::TimerBase::SharedPtr camera_timer_;

        cv::VideoCapture camera;

        void camera_callback();


    };