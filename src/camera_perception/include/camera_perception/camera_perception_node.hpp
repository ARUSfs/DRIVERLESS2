/**
 * @file camera_perception_node.hpp
 * @author Ignacio Sánchez Isidro (igsais12@gmail.com)
 * @date 08-03-2025
 * @brief Header file for the Camera Perception node, which main purpose is to detect cones through 
 * a camera
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <DarkHelp.hpp>


/**
 * @brief Class containing the Camera Perception node
 * 
 */
class CameraPerception : public rclcpp::Node
{
    public:
        CameraPerception();
    private:
        // Config file
        std::string kImageTopic;
        int kCameraPeriod;
        int kCameraID;
        
        // Darknet params
        std::string kConfigFile;
        std::string kWeightsFile;
        std::string kNamesFile;

        DarkHelp::NN nn_;

        // ROS2 publishers and subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::TimerBase::SharedPtr camera_timer_;
        
        // Camera
        cv::VideoCapture camera;
        
        // Callbacks
        void camera_callback();
        void image_callback(sensor_msgs::msg::Image::SharedPtr msg);

    };