#include "camera_perception/camera_calibration_node.hpp"

#define READ_FLAG 0
#define WRITE_FLAG 1
#define APPEND_FLAG 2

CameraCalibration::CameraCalibration() : Node("camera_calibration")
{
    // Declare and get parameters
    this->declare_parameter<bool>("debug", false);
    this->get_parameter("debug", kDebug);

    this->declare_parameter<bool>("intrinsic_calibration", true);
    this->get_parameter("intrinsic_calibration", kIntrinsicCalibration);
    this->declare_parameter<bool>("extrinsic_calibration", true);
    this->get_parameter("extrinsic_calibration", kExtrinsicCalibration);
    this->declare_parameter<bool>("trapezoidal_layout", true);
    this->get_parameter("trapezoidal_layout", kTrapezoidalLayout);

    this->declare_parameter<std::string>("image_topic", "/camera/color/image_raw");
    this->get_parameter("image_topic", kImageTopic);
    this->declare_parameter<std::string>("lidar_topic", "/perception/map");
    this->get_parameter("lidar_topic", kLidarTopic);

    this->declare_parameter<double>("cam_x", 0.0);
    this->get_parameter("cam_x", kCamX);
    this->declare_parameter<double>("cam_y", 0.0);
    this->get_parameter("cam_y", kCamY);
    this->declare_parameter<double>("cam_z", 0.0);
    this->get_parameter("cam_z", kCamZ);

    this->declare_parameter<double>("lidar_x", 0.0);
    this->get_parameter("lidar_x", kLidarX);
    this->declare_parameter<double>("lidar_y", 0.0);
    this->get_parameter("lidar_y", kLidarY);
    this->declare_parameter<double>("lidar_z", 0.0);
    this->get_parameter("lidar_z", kLidarZ);

    this->declare_parameter<double>("square_size", 0.024);
    this->get_parameter("square_size", kSquareSize);

    // Get src package path for calibration files
    std::string src_pkg_path = ament_index_cpp::get_package_prefix("camera_perception");
    int pos = src_pkg_path.find("install");
    src_pkg_path.resize(pos);
    src_pkg_path += "src/DRIVERLESS2/src/camera_perception";

    kMatrixFile = src_pkg_path + "/resources/calibration/camera_matrix.yaml";
    kDistFile = src_pkg_path + "/resources/calibration/dist_coeffs.yaml";
    kRvecsFile = src_pkg_path + "/resources/calibration/rvecs.yaml";
    kTvecsFile = src_pkg_path + "/resources/calibration/tvecs.yaml";
    kHomographyFile = src_pkg_path + "/resources/calibration/homography.yaml";
    
    // Darknet params and initialization
    std::string share_pkg_path = ament_index_cpp::get_package_share_directory("camera_perception");
    std::string config_file = share_pkg_path + "/resources/darknet/cfg.cfg";
    std::string weights_file = share_pkg_path + "/resources/darknet/weights";
    std::string names_file = share_pkg_path + "/resources/darknet/names";

    nn_.init(config_file, weights_file, names_file);
    nn_.config.threshold = 0.1;
    nn_.config.include_all_names = true;
    nn_.config.names_include_percentage = true;
    nn_.config.annotation_include_duration = false;
    nn_.config.annotation_include_timestamp = false;
    nn_.config.sort_predictions = DarkHelp::ESort::kAscending;

    // Suscriber options to match the camera's publisher
    rclcpp::QoS qos(10);
    qos.best_effort();

    // Subscribers and publishers
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        kImageTopic, qos, std::bind(&CameraCalibration::camera_calibration_callback, this, std::placeholders::_1));
    lidar_perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kLidarTopic, 10, std::bind(&CameraCalibration::lidar_perception_callback, this, std::placeholders::_1));
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/calibration", 10);

    // Initialize the clock
    last_detection_time_ = this->now().seconds();
}


void CameraCalibration::camera_calibration_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    if (image.empty()) return;
    cv::resize(image, image, cv::Size(1920, 1088));
    last_image_ = image;
    if (!kIntrinsicCalibration || intrinsic_finished_) return;

    // Check if last detection was at least 1 second ago
    double time_diff = this->now().seconds() - last_detection_time_;
    if (time_diff < 1.0) return;

    // Check if we have enough images for calibration
    if (image_points_.size() > 10) {
        cv::calibrateCamera(object_points_, image_points_, image.size(), camera_matrix_, dist_coeffs_, rvecs_, tvecs_);
        std::cout << "Camera matrix: " << camera_matrix_ << std::endl;
        std::cout << "Distortion coefficients: " << dist_coeffs_ << std::endl;
        std::cout << "Rvecs: " << rvecs_ << std::endl;
        std::cout << "Tvecs: " << tvecs_ << std::endl;
        std::cout << "Calibration done!" << std::endl;
        std::cout << "Writing to file..." << std::endl;

        this->save_mat_to_file(camera_matrix_, "Camera matrix", kMatrixFile);
        this->save_mat_to_file(dist_coeffs_, "Distorsion coefficients", kDistFile);
        this->save_mat_to_file(rvecs_, "Rvecs", kRvecsFile);
        this->save_mat_to_file(tvecs_, "Tvecs", kTvecsFile);
        std::cout << "Files written!" << std::endl; 
        std::cout << "Exiting..." << std::endl;
        intrinsic_finished_ = true;
        return;
    }

    // Create the object points for the chessboard pattern
    std::vector<cv::Point3f> object_points;
    for (int i = 0; i < pattern_size_.height; i++) {
        for (int j = 0; j < pattern_size_.width; j++) {
            object_points.push_back(cv::Point3f(j*kSquareSize, i*kSquareSize, 0));
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
        last_detection_time_ = this->now().seconds();

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

void CameraCalibration::lidar_perception_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    
    if (!kExtrinsicCalibration) return;
    pcl::PointCloud<ConeXYZColorScore> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);
    if (pcl_cloud.size() == 0) return;
    if (last_image_.empty()) return;

    cv::Mat image_undistorted, camera_matrix, dist_coeffs;
    camera_matrix = this->load_mat_from_file(kMatrixFile, "Camera matrix");
    dist_coeffs = this->load_mat_from_file(kDistFile, "Distorsion coefficients");
    //cv::undistort(last_image_, image_undistorted, camera_matrix, dist_coeffs);
    const auto result = nn_.predict(last_image_);
    
    if (result.size() != pcl_cloud.size()) return;

    std::vector<cv::Point3f> world_points;
    std::vector<cv::Point2f> image_points;
    
    for (int i = 0; i < pcl_cloud.size(); i++) {
        if (std::abs(pcl_cloud.points[i].x) > 6 || std::abs(pcl_cloud.points[i].y) > 6) return;
        cv::Point3f point(pcl_cloud.points[i].x + kCamX, pcl_cloud.points[i].y, 0.32);
        world_points.push_back(point);
    }
    
    // Sort the points assuming a trapezoidal layout (with closer cones to the camera closer in y coordinate)
    if (kTrapezoidalLayout){
        std::sort(world_points.begin(), world_points.end(), [](const cv::Point3f &a, const cv::Point3f &b) {
            return a.y < b.y;
        });

        for (const auto & cone : result){
            image_points.push_back(cv::Point2f(cone.rect.x + cone.rect.width / 2, cone.rect.y));
        }
        std::sort(image_points.begin(), image_points.end(), [](const cv::Point2f &a, const cv::Point2f &b) {
            return a.x < b.x;
        });
    }
    
    if (kDebug) {
        for (int i = 0; i < pcl_cloud.size(); i++) {
            std::cout << "Cone " << i << ": " << world_points[i].x << ", " << world_points[i].y << ", " << world_points[i].z << std::endl;
            std::cout << "Cone " << i << ": " << image_points[i].x << ", " << image_points[i].y << std::endl;
        }
        cv::Mat output = nn_.annotate();
        sensor_msgs::msg::Image::SharedPtr msg_out = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", output).toImageMsg();
        image_pub_->publish(*msg_out);
    }

    // Calculate the homography parameters
    cv::Mat rvec, tvec, rot_mat, empty_vec;
    cv::solvePnP(world_points, image_points, camera_matrix, empty_vec, rvec, tvec, false);
    cv::Rodrigues(rvec, rot_mat);
    cv::Mat rot_mat_inv = rot_mat.inv();
    if (kDebug) std::cout << "Rotation matrix: " << rot_mat << std::endl;
    if (kDebug) std::cout << "Translation vector: " << tvec << std::endl;
    this->save_mat_to_file(rot_mat_inv, "Inverse rotation matrix", kHomographyFile, WRITE_FLAG);
    this->save_mat_to_file(tvec, "Translation vector", kHomographyFile, APPEND_FLAG);
    rclcpp::shutdown();
    
}

void CameraCalibration::save_mat_to_file(cv::Mat matrix, std::string matrix_name, std::string file_path, int mode){
    cv::FileStorage mat_file(file_path, mode);
    mat_file << matrix_name << matrix;
    mat_file.release();
}

cv::Mat CameraCalibration::load_mat_from_file(std::string file_path, std::string matrix_name){
    cv::Mat matrix;
    cv::FileStorage mat_file(file_path, cv::FileStorage::READ);
    mat_file[matrix_name] >> matrix;
    mat_file.release();
    return matrix;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraCalibration>());
    rclcpp::shutdown();
    return 0;
}