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

    // Darknet params
    std::string pkg_path = ament_index_cpp::get_package_share_directory("camera_perception");
    kConfigFile = pkg_path + "/resources/darknet/cfg.cfg";
    kWeightsFile = pkg_path + "/resources/darknet/weights";
    kNamesFile = pkg_path + "/resources/darknet/names";
    
    rclcpp::QoS qos(20);
    qos.best_effort();
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(kImageTopic, qos,
                    std::bind(&CameraPerception::image_callback, this, std::placeholders::_1));

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("image", 20);
    // camera_timer_ = this->create_wall_timer(std::chrono::milliseconds(kCameraPeriod),
    //                 std::bind(&CameraPerception::camera_callback, this)); 
    
    // Darknet
    nn_.init(kConfigFile, kWeightsFile, kNamesFile);
    nn_.config.threshold = 0.1;
    nn_.config.include_all_names = true;
    nn_.config.names_include_percentage = true;
    nn_.config.annotation_include_duration = false;
    nn_.config.annotation_include_timestamp = false;
    nn_.config.sort_predictions = DarkHelp::ESort::kAscending;
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
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::Mat image_resized;
    cv::resize(image, image_resized, cv::Size(1920, 1088));
    const auto result = nn_.predict(image_resized);
    
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