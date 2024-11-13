#include "acc_planning_node.hpp"



AccPlanning::AccPlanning() : Node("acc_planning_node")
{

    this->declare_parameter<std::string>("perception_topic", "/arussim/perception");
    this->declare_parameter<std::string>("trajectory_topic", "/acc_planning/trajectory");
    this->get_parameter("perception_topic", kPerceptionTopic);
    this->get_parameter("trajectory_topic", kTrajectoryTopic);

    // Publish resulting trajectory
    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>(kTrajectoryTopic, 10);
    // Subscribe to the perception point cloud topic
    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kPerceptionTopic, 10, std::bind(&AccPlanning::perception_callback, this, std::placeholders::_1));

}

void AccPlanning::perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg) {
    
    cones_ = AccPlanning::convert_ros_to_pcl(per_msg);

  
    if (cones_.points.empty()) {
        std::cout << "No points in the PointCloud after conversion to PCL." << std::endl;
    } else {
        std::cout << "PointCloud contains " << cones_.points.size() << " points." << std::endl;
    }

    AccPlanning::generate_planning();


    if (a_ == 0.0 && b_ == 0.0) {
        std::cout << "No valid line found during planning." << std::endl;
    } else {
        std::cout << "Best line found with coefficients: a = " << a_ << ", b = " << b_ << std::endl;
    }

    AccPlanning::publish_trajectory();
}


void AccPlanning::generate_planning() {
    int max_iterations = 500;
    double ransac_threshold = 0.2;
    int min_inliers_required = 3; 
    std::random_device rd;
    std::mt19937 gen(rd());
    double best_a = 0.0;
    double best_b = 0.0;


    int max_inliers = 0;

    std::uniform_int_distribution<> distrib(0, cones_.points.size() - 1);


    for (int iter = 0; iter < max_iterations; ++iter) {

        int i = distrib(gen);
        int j = distrib(gen);
        int k = distrib(gen);
        auto& c1 = cones_.points[i];
        auto& c2 = cones_.points[j];
        auto& c3 = cones_.points[k];


        if (std::abs(c1.x - c2.x) < 0.1 || (std::abs(c1.y - c3.y) < 0.1 && std::abs(c2.y - c3.y) < 0.1)) {
            continue;
        }


        double a = (c2.y - c1.y) / (c2.x - c1.x);
        double b1 = c1.y - c1.x * a;
        double b2 = c3.y - a * c3.x;

        
        int inliers = 0;
        std::vector<int> inliers_indices;
        for (int idx = 0; idx < cones_.points.size(); ++idx) {
            const auto& cone = cones_.points[idx];
            double d1 = std::abs(a * cone.x + b1 - cone.y) / std::sqrt(a * a + 1);
            double d2 = std::abs(a * cone.x + b2 - cone.y) / std::sqrt(a * a + 1);
            if (std::min(d1, d2) < ransac_threshold) {
                ++inliers;
                inliers_indices.push_back(idx);
            }
        }


        if (inliers > max_inliers) {
            best_a = a;
            best_b = (b1 + b2)/2.0;
            max_inliers = inliers;
        }
    }

    if (max_inliers >= min_inliers_required) {
        a_ = best_a;
        b_ = best_b;
    }
    
}


void AccPlanning::publish_trajectory(){
    common_msgs::msg::Trajectory trajectory_msg;

    double step = 0.1;  
    double max_distance = 150.0; 

    for (double x = 0.0; x <= max_distance; x += step) {
        common_msgs::msg::PointXY point;
        point.x = x;
        point.y = a_ * x + b_;  
        trajectory_msg.points.push_back(point);
    }

    trajectory_pub_->publish(trajectory_msg);
}


pcl::PointCloud<ConeXYZColorScore> AccPlanning::convert_ros_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud) {
    pcl::PointCloud<ConeXYZColorScore> pcl_cloud;  
    pcl::fromROSMsg(*ros_cloud, pcl_cloud); 
    return pcl_cloud;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AccPlanning>());
    rclcpp::shutdown();
    return 0;
}