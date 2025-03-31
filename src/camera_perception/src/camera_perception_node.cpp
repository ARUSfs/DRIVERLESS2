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

    kPkgPath = ament_index_cpp::get_package_share_directory("camera_perception");
    
    // Subscribers and publishers
    rclcpp::QoS qos(20);
    qos.best_effort();
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(kImageTopic, qos,
        std::bind(&CameraPerception::image_callback, this, std::placeholders::_1));
        
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 20);

    // Darknet params and initialization
    kConfigFile = kPkgPath + "/resources/darknet/cfg.cfg";
    kWeightsFile = kPkgPath + "/resources/darknet/weights";
    kNamesFile = kPkgPath + "/resources/darknet/names";

    nn_.init(kConfigFile, kWeightsFile, kNamesFile);
    nn_.config.threshold = 0.1;
    nn_.config.include_all_names = true;
    nn_.config.names_include_percentage = true;
    nn_.config.annotation_include_duration = false;
    nn_.config.annotation_include_timestamp = false;
    nn_.config.sort_predictions = DarkHelp::ESort::kAscending;

    // Calibration params
    std::string matrix_file_path = kPkgPath + "/resources/calibration/camera_matrix.yaml";
    std::string dist_file_path = kPkgPath + "/resources/calibration/dist_coeffs.yaml";
    std::string rvecs_file_path = kPkgPath + "/resources/calibration/rvecs.yaml";
    std::string tvecs_file_path = kPkgPath + "/resources/calibration/tvecs.yaml";
    cv::FileStorage mat_file(matrix_file_path, cv::FileStorage::READ);
    cv::FileStorage dist_file(dist_file_path, cv::FileStorage::READ);
    cv::FileStorage rvecs_file(rvecs_file_path, cv::FileStorage::READ);
    cv::FileStorage tvecs_file(tvecs_file_path, cv::FileStorage::READ);
    if (!mat_file.isOpened() || !dist_file.isOpened() || !rvecs_file.isOpened() || !tvecs_file.isOpened()){
        RCLCPP_ERROR(this->get_logger(), "Calibration files not found");
        return;
    }
    mat_file["Camera matrix"] >> camera_matrix_;
    dist_file["Distorsion coefficients"] >> dist_coeffs_;
    rvecs_file["Rvecs"] >> rvecs_;
    tvecs_file["Tvecs"] >> tvecs_;
    mat_file.release();
    dist_file.release();
    rvecs_file.release();
    tvecs_file.release();
    std::cout << "Camera matrix: " << camera_matrix_ << std::endl;
    std::cout << "Distortion coefficients: " << dist_coeffs_ << std::endl;
    std::cout << "Rvecs: " << rvecs_ << std::endl;
    std::cout << "Tvecs: " << tvecs_ << std::endl;
}

void CameraPerception::camera_callback(){
    return;
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

void CameraPerception::image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    // Return if the NN is not initialized
    if (!nn_.is_initialized()) return;

    // Process the image
    cv::Mat image_raw = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat image_undistorted;
    cv::Mat image_resized;
    cv::undistort(image_raw, image_undistorted, camera_matrix_, dist_coeffs_);
    cv::resize(image_undistorted, image_resized, cv::Size(1920, 1088));
    
    // Perform the prediction
    const auto result = nn_.predict(image_resized);
    if (result.size() == 0){
        RCLCPP_INFO(this->get_logger(), "No predictions found in frame");
        return;
    }

    // Results visualization
    std::cout << result << std::endl;
    cv::Mat output = nn_.annotate();
    sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", output).toImageMsg();
    image_pub_->publish(*msg_out);

}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPerception>());
    rclcpp::shutdown();
    return 0;
}