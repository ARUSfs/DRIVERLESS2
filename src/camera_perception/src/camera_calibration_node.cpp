#include "camera_perception/camera_calibration_node.hpp"


CameraCalibration::CameraCalibration() : Node("camera_calibration")
{
    // Declare and get parameters
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->get_parameter("image_topic", kImageTopic);

    this->declare_parameter<bool>("debug", false);
    this->get_parameter("debug", kDebug);

    kMatrixFile = ament_index_cpp::get_package_share_directory("camera_perception") + "/resources/calibration/camera_matrix.yaml";
    kDistFile = ament_index_cpp::get_package_share_directory("camera_perception") + "/resources/calibration/dist_coeffs.yaml";
    kRvecsFile = ament_index_cpp::get_package_share_directory("camera_perception") + "/resources/calibration/rvecs.yaml";
    kTvecsFile = ament_index_cpp::get_package_share_directory("camera_perception") + "/resources/calibration/tvecs.yaml";

    // Suscriber options to match the camera's publisher
    rclcpp::QoS qos(10);
    qos.best_effort();

    // Subscribers and publishers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        kImageTopic, qos, std::bind(&CameraCalibration::camera_calibration_callback, this, std::placeholders::_1));

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/calibration", 10);

    // Initialize the clock
    start_time_ = std::chrono::high_resolution_clock::now();
}


void CameraCalibration::camera_calibration_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    if (image.empty()) return;

    // Check if we have enough images for calibration
    if (image_points_.size() >= 20) {
        cv::calibrateCamera(object_points_, image_points_, image.size(), camera_matrix_, dist_coeffs_, rvecs_, tvecs_);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time_).count();
        std::cout << "Calibration time: " << duration << " seconds" << std::endl;
        std::cout << "Camera matrix: " << camera_matrix_ << std::endl;
        std::cout << "Distortion coefficients: " << dist_coeffs_ << std::endl;
        std::cout << "Rvecs: " << rvecs_ << std::endl;
        std::cout << "Tvecs: " << tvecs_ << std::endl;
        std::cout << "Calibration done!" << std::endl;
        std::cout << "Writing to file..." << std::endl;

        cv::FileStorage mat_file(kMatrixFile, cv::FileStorage::WRITE);
        mat_file << "Camera matrix" << camera_matrix_;
        mat_file.release();
        cv::FileStorage dist_file(kDistFile, cv::FileStorage::WRITE);
        dist_file << "Distorsion coefficients" << dist_coeffs_;
        dist_file.release();
        cv::FileStorage rvecs_file(kRvecsFile, cv::FileStorage::WRITE);
        rvecs_file << "Rvecs" << rvecs_;
        rvecs_file.release();
        cv::FileStorage tvecs_file(kTvecsFile, cv::FileStorage::WRITE);
        tvecs_file << "Tvecs" << tvecs_;
        tvecs_file.release();
        std::cout << "Files written!" << std::endl; 
        std::cout << "Exiting..." << std::endl;
        rclcpp::shutdown();
    }

    // Create the object points for the chessboard pattern
    std::vector<cv::Point3f> object_points;
    for (int i = 0; i < pattern_size_.height; i++) {
        for (int j = 0; j < pattern_size_.width; j++) {
            object_points.push_back(cv::Point3f(j, i, 0));
        }
    }


    // Convert the image to grayscale
    cv::Mat gray_image;
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

    // Find the chessboard corners in the image
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray_image, pattern_size_, corners,
                            cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK + cv::CALIB_CB_NORMALIZE_IMAGE);
    if (found) {
        cv::cornerSubPix(gray_image, corners, cv::Size(11, 11), cv::Size(-1, -1), term_criteria_);
        image_points_.push_back(corners);
        object_points_.push_back(object_points);
        if (kDebug){
            std::cout << "Chessboard corners found!" << std::endl;
            cv::drawChessboardCorners(image, pattern_size_, cv::Mat(corners), found);
            sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
            image_pub_->publish(*msg_out);
        }
    }
    
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraCalibration>());
    rclcpp::shutdown();
    return 0;
}