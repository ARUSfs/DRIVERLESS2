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
#include <ament_index_cpp/get_package_prefix.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ConeXYZColorScore.h"

#include <DarkHelp.hpp>
#include <cmath>

class CameraCalibration : public rclcpp::Node
{
    public:
        CameraCalibration();

    private:
        // Config file
        bool kDebug;
        bool kIntrinsicCalibration;
        bool kExtrinsicCalibration;
        bool kTrapezoidalLayout;
        std::string kImageTopic;
        std::string kLidarTopic;
        double kCamX;
        double kCamY;
        double kCamZ;
        double kLidarX;
        double kLidarY;
        double kLidarZ;
        double kSquareSize;

        // ROS2 suscribers and publishers
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_perception_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

        // Points for calibration
        std::vector<std::vector<cv::Point2f>> image_points_ = {};
        std::vector<std::vector<cv::Point3f>> object_points_ = {};
        cv::Mat last_image_;

        // Intrinsic calibration
        bool intrinsic_finished_ = false;
        double last_detection_time_;
        cv::Mat camera_matrix_, dist_coeffs_, rvecs_, tvecs_;
        std::string kMatrixFile, kDistFile, kRvecsFile, kTvecsFile, kHomographyFile;

        // Extrinsic calibration
        bool extrinsic_finished_ = false;
        DarkHelp::NN nn_;

        cv::Size pattern_size_ = cv::Size(9, 6);
        cv::TermCriteria term_criteria_ = cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1);
        void camera_calibration_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void lidar_perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

        void save_mat_to_file(cv::Mat matrix, std::string matrix_name, std::string file_path, int mode = 1);
        cv::Mat load_mat_from_file(std::string file_path, std::string matrix_name);

};
