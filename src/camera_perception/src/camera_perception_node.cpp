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
    perception_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/map", 20);

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
    std::string homography_file_path = kPkgPath + "/resources/calibration/homography.yaml";
    camera_matrix_ = load_mat_from_file(matrix_file_path, "Camera matrix");
    dist_coeffs_ = load_mat_from_file(dist_file_path, "Distorsion coefficients");
    rot_mat_ = load_mat_from_file(homography_file_path, "Inverse rotation matrix");
    tvec_ = load_mat_from_file(homography_file_path, "Translation vector");
    if (camera_matrix_.empty() || dist_coeffs_.empty() || rot_mat_.empty() || tvec_.empty()){
        RCLCPP_ERROR(this->get_logger(), "Calibration files not found");
        return;
    }

    // Calculate parameters to reproject the points of the image
    left_side_ = rot_mat_ * camera_matrix_.inv();
    right_side_ = rot_mat_ * tvec_;    
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
    if (image_raw.empty()) return;
    cv::Mat image_resized, image;
    cv::resize(image_raw, image, cv::Size(1920, 1088));
    //cv::undistort(image_resized, image, camera_matrix_, dist_coeffs_);
    
    // Perform the prediction
    const auto result = nn_.predict(image);
    if (result.size() == 0){
        RCLCPP_INFO(this->get_logger(), "No predictions found in frame");
        return;
    }

    // Results visualization
    std::cout << result << std::endl;
    cv::Mat output = nn_.annotate();
    sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", output).toImageMsg();
    image_pub_->publish(*msg_out);

    pcl::PointCloud<ConeXYZColorScore> perception_map;
    for (const auto det : result){
        int cone_color;
        if (det.best_class == 0) {
            cone_color = 1;
        } else if (det.best_class == 1) {
            cone_color = 0;
        } else if (det.best_class == 2) {
            cone_color = 2;
        } else {
            cone_color = 3;
        }
        cv::Mat vertex = (cv::Mat_<double>(3,1) <<  det.rect.x + det.rect.width / 2, det.rect.y, 1);

        cv::Mat left_side = left_side_ * vertex;

        double s = (0.32 + right_side_.at<double>(2,0))/left_side.at<double>(2,0);

        cv::Mat cone_loc = s * left_side - right_side_;
        ConeXYZColorScore cone(cone_loc.at<double>(0,0), cone_loc.at<double>(1,0), 0.32, cone_color, det.best_probability);
        std::cout << "Cone: " << cone.x << ", " << cone.y << ", " << cone.z << std::endl;
        perception_map.push_back(cone);
    }

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(perception_map, map_msg);
    map_msg.header.frame_id = "rslidar";
    perception_pub_->publish(map_msg);
}

cv::Mat CameraPerception::load_mat_from_file(std::string file_path, std::string matrix_name){
    cv::Mat matrix;
    cv::FileStorage mat_file(file_path, cv::FileStorage::READ);
    mat_file[matrix_name] >> matrix;
    mat_file.release();
    return matrix;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPerception>());
    rclcpp::shutdown();
    return 0;
}