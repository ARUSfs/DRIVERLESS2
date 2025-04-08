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
#include "ConeXYZColorScore.h"
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
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

        // Package path
        std::string kPkgPath;
        
        // Darknet params
        std::string kConfigFile;
        std::string kWeightsFile;
        std::string kNamesFile;

        DarkHelp::NN nn_;

        // ROS2 publishers and subscribers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr perception_pub_;
        rclcpp::TimerBase::SharedPtr camera_timer_;
        
        // Camera
        cv::VideoCapture camera;
        
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        // cv::Mat homography_matrix_;
        cv::Mat rot_mat_, tvec_, left_side_, right_side_;
        
        // Callbacks
        void camera_callback();
        void image_callback(sensor_msgs::msg::Image::SharedPtr msg);

        cv::Mat load_mat_from_file(std::string file_path, std::string matrix_name);

    };