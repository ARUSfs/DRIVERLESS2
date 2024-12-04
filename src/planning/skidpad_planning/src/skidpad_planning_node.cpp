#include "skidpad_planning_node.hpp"



SkidpadPlanning::SkidpadPlanning() : Node("skidpad_planning_node"), trajectory_calculated_(false) 
{

    this->declare_parameter<std::string>("perception_topic", "/arussim/perception");
    this->declare_parameter<std::string>("trajectory_topic", "/skidpad_planning/trajectory");
    this->get_parameter("perception_topic", kPerceptionTopic);
    this->get_parameter("trajectory_topic", kTrajectoryTopic);

    // Publish resulting trajectory
    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>(kTrajectoryTopic, 10);
    // Subscribe to the perception point cloud topic
    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kPerceptionTopic, 10, std::bind(&SkidpadPlanning::perception_callback, this, std::placeholders::_1));

    // Inicialize the template
    double d = 0.5; // Distance between points in the straight
    double r = 9.125;
    int N = 100; // Number of points in the circle

    for (int i = 0; i <= 20 / d; ++i) {
        template_.emplace_back(-20 + d * i, 0);
    }
    for (int i = 0; i < N; ++i) {
        template_.emplace_back(r * std::sin(2 * M_PI * i / N), -9.125 + r * std::cos(2 * M_PI * i / N));
    }
    for (int i = 0; i < N; ++i) {
        template_.emplace_back(r * std::sin(2 * M_PI * i / N), -9.125 + r * std::cos(2 * M_PI * i / N));
    }
    for (int i = 0; i < N; ++i) {
        template_.emplace_back(r * std::sin(2 * M_PI * i / N), 9.125 - r * std::cos(2 * M_PI * i / N));
    }
    for (int i = 0; i < N; ++i) {
        template_.emplace_back(r * std::sin(2 * M_PI * i / N), 9.125 - r * std::cos(2 * M_PI * i / N));
    }
    for (int i = 0; i <= 20 / d; ++i) {
        template_.emplace_back(d * i, 0);
    }
}

void SkidpadPlanning::perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg) {
    
  if (!trajectory_calculated_) {
        cones_ = SkidpadPlanning::convert_ros_to_pcl(per_msg);

        if (cones_.points.empty()) {
            std::cout << "No points in the PointCloud after conversion to PCL." << std::endl;
        } else {
            std::cout << "PointCloud contains " << cones_.points.size() << " points." << std::endl;
        }

        SkidpadPlanning::generate_planning();
        SkidpadPlanning::publish_trajectory();

        trajectory_calculated_ = true;

        std::cout << "Trajectory calculated and published." << std::endl;
   } else {
        SkidpadPlanning::publish_trajectory();
   }
}


std::tuple<double, double, double> SkidpadPlanning::find_circle_center(
    const ConeXYZColorScore& p1, const ConeXYZColorScore& p2, const ConeXYZColorScore& p3) {
    const double radius_target1 = 7.625;
    const double radius_target2 = 10.625;
    // Get middle points
    double mid_x1 = (p1.x + p2.x) / 2.0;
    double mid_y1 = (p1.y + p2.y) / 2.0;
    
    double mid_x2 = (p2.x + p3.x) / 2.0;
    double mid_y2 = (p2.y + p3.y) / 2.0;

    // Get mediatrix lines slopes (m)
    double slope1 = (p2.y - p1.y) != 0 ? -(p2.x - p1.x) / (p2.y - p1.y) : std::numeric_limits<double>::infinity();
    double slope2 = (p3.y - p2.y) != 0 ? -(p3.x - p2.x) / (p3.y - p2.y) : std::numeric_limits<double>::infinity();

    // Compute mediatrix equations (b) (y=mx+b)
    double intercept1 = slope1 != std::numeric_limits<double>::infinity() ? mid_y1 - slope1 * mid_x1 : mid_x1;
    double intercept2 = slope2 != std::numeric_limits<double>::infinity() ? mid_y2 - slope2 * mid_x2 : mid_x2;

    double center_x, center_y;

    // Solve the equations to find the center
    if (slope1 != slope2) {
        center_x = (intercept2 - intercept1) / (slope1 - slope2);
        center_y = slope1 * center_x + intercept1;
    } else {
        center_x = (mid_x1 + mid_x2) / 2.0;
        center_y = (mid_y1 + mid_y2) / 2.0;
    }

    // Compute the radius
    radius = std::sqrt(std::pow(p1.x - center_x, 2) + std::pow(p1.y - center_y, 2));
    return{center_x, center_y, radius};
    
}

void SkidpadPlanning::generate_planning() {
    const int N_iterations = 500; // RANSAC iterations
    const double threshold = 0.2; // RANSAC threshold
    const double radius_target1 = 7.625;
    const double radius_target2 = 10.625;
    const double distance_between_centers = 18.25;

    int max_inliers1 = 0, max_inliers2 = 0;
    best_center = {0.0, 0.0};
    second_best_center = {0.0, 0.0};

    pcl::PointCloud<ConeXYZColorScore> remaining_cones;

    // RANSAC fot first straight
    for (int iter = 0; iter < N_iterations; ++iter) {
        int i = rand() % cones_.points.size();
        int j = rand() % cones_.points.size();
        int k = rand() % cones_.points.size();

        const auto& p1 = cones_.points[i];
        const auto& p2 = cones_.points[j];
        const auto& p3 = cones_.points[k];

        auto [x_center, y_center, r] = find_circle_center(p1, p2, p3);
   
   
        
           if ((std::abs(r - radius_target1) < 2.0 || std::abs(r - radius_target2) < 2.0) && x_center > 13.0) {
            int inliers = 0;
            std::vector<bool> is_inlier(cones_.points.size(), true);

            for (size_t idx = 0; idx < cones_.points.size(); ++idx) {
                const auto& cone = cones_.points[idx];
                double d = std::sqrt(std::pow(cone.x - x_center, 2) + std::pow(cone.y - y_center, 2));

                if (std::abs(d - radius_target1) < threshold || std::abs(d - radius_target2) < threshold) {
                    ++inliers;
                    is_inlier[idx] = false;
                }
            }

            if (inliers > max_inliers1) {
                best_center = {x_center, y_center};
                max_inliers1 = inliers;

                // Update remaining cones
                remaining_cones.clear();
                for (size_t idx = 0; idx < cones_.points.size(); ++idx) {
                    if (is_inlier[idx]) {
                        remaining_cones.push_back(cones_.points[idx]);
                    }
                }
            }
        }
    }

    // RANSAC for second straight
    if (remaining_cones.size() >= 3) {
        for (int iter = 0; iter < N_iterations; ++iter) {
            int i = rand() % remaining_cones.points.size();
            int j = rand() % remaining_cones.points.size();
            int k = rand() % remaining_cones.points.size();

            const auto& p1 = remaining_cones.points[i];
            const auto& p2 = remaining_cones.points[j];
            const auto& p3 = remaining_cones.points[k];

            auto [x_center, y_center, r] = find_circle_center(p1, p2, p3);
            if ((std::abs(r - radius_target1) < 2.0 || std::abs(r - radius_target2) < 2.0) && x_center > 13.0 &&
                std::abs(distance_between_centers - std::sqrt(std::pow(best_center.first - x_center, 2) + std::pow(best_center.second - y_center, 2))) < 2.0) {
                int inliers = 0;

                for (const auto& cone : remaining_cones.points) {
                    double d = std::sqrt(std::pow(cone.x - x_center, 2) + std::pow(cone.y - y_center, 2));
                    if (std::abs(d - radius_target1) < threshold || std::abs(d - radius_target2) < threshold) {
                        ++inliers;
                    }
                }

                if (inliers > max_inliers2) {
                    second_best_center = {x_center, y_center};
                    max_inliers2 = inliers;
                }
            }
        }
    }
    
    std::cout << "Best center: (" << best_center.first << ", " << best_center.second << ")" << std::endl;
    std::cout << "Second best center: (" << second_best_center.first << ", " << second_best_center.second << ")" << std::endl;
    std::cout << "Radius: " << radius << std::endl;
}




void SkidpadPlanning::publish_trajectory() {
    common_msgs::msg::Trajectory trajectory_msg;

    if (best_center.first == 0.0 || best_center.second == 0.0 || 
        second_best_center.first == 0.0 || second_best_center.second == 0.0) {
        std::cerr << "Error: Invalid centers detected. Trajectory will not be published." << std::endl;
        return;
    }

    // Compute the skidpad center
    double mid_x = (best_center.first + second_best_center.first) / 2.0;
    double mid_y = (best_center.second + second_best_center.second) / 2.0;
    double orientation = - M_PI/2 + std::atan2(second_best_center.second - best_center.second, 
                                    second_best_center.first - best_center.first);

    if(orientation < - M_PI/2) {
        orientation +=  M_PI;
    } else if (orientation > M_PI/2) {
        orientation -=  M_PI;
    }

    // Transform the template
    for (const auto& point : template_) {
        Eigen::Vector2d transformed_point;
        transformed_point.x() = std::cos(orientation) * point.x() - std::sin(orientation) * point.y() + mid_x;
        transformed_point.y() = std::sin(orientation) * point.x() + std::cos(orientation) * point.y() + mid_y;

        common_msgs::msg::PointXY traj_point;
        traj_point.x = transformed_point.x();
        traj_point.y = transformed_point.y();
        trajectory_msg.points.push_back(traj_point);
    }

    trajectory_pub_->publish(trajectory_msg);
}

pcl::PointCloud<ConeXYZColorScore> SkidpadPlanning::convert_ros_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud) {
    pcl::PointCloud<ConeXYZColorScore> pcl_cloud;  
    pcl::fromROSMsg(*ros_cloud, pcl_cloud); 
    return pcl_cloud;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SkidpadPlanning>());
    rclcpp::shutdown();
    return 0;
}