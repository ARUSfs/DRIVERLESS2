/**
 * @file calibration.hpp
 * @author Ignacio Sánchez (igsais12@gmail.com)
 * @brief Header file for camera calibration
 * @version 0.1
 * @date 2025-03-29
 * 
 */
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>

#include <chrono>

class CameraCalibration : public rclcpp::Node
{
    public:
        CameraCalibration();

    private:
        std::string kImageTopic;
        bool kDebug;
        
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

        std::vector<std::vector<cv::Point2f>> image_points_ = {};
        std::vector<std::vector<cv::Point3f>> object_points_ = {};

        cv::Mat camera_matrix_, dist_coeffs_, rvecs_, tvecs_;
        std::string kMatrixFile, kDistFile, kRvecsFile, kTvecsFile;

        cv::Size pattern_size_ = cv::Size(7, 6);
        cv::TermCriteria term_criteria_ = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);
        void camera_calibration_callback(const sensor_msgs::msg::Image::SharedPtr msg);

        std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
};
