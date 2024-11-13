#include "acc_planning_node.hpp"



AccPlanning::AccPlanning() : Node("acc_planning_node")
{

    kPerceptionTopic = "/arussim/perception";  
    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>("/acc/planning/trajectory", 10);
    // Subscribe to the perception point cloud topic
    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kPerceptionTopic, 10, std::bind(&AccPlanning::perception_callback, this, std::placeholders::_1));

    // Publisher for planning results
    
}
std::pair<double, double> best_coef;  
common_msgs::msg::Trajectory trajectory_msg;
void AccPlanning::perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg) {
    
    std::cout << "Received PointCloud2 message." << std::endl;
    std::cout << "Width: " << per_msg->width << ", Height: " << per_msg->height << std::endl;
    std::cout << "PointCloud2 data size: " << per_msg->data.size() << " bytes." << std::endl;


    std::cout << "Converting PointCloud2 to PCL format..." << std::endl;
    cones_ = AccPlanning::convert_ros_to_pcl(per_msg);

  
    if (cones_.points.empty()) {
        std::cout << "No points in the PointCloud after conversion to PCL." << std::endl;
    } else {
        std::cout << "PointCloud contains " << cones_.points.size() << " points." << std::endl;
    }


    std::cout << "Generating planning results..." << std::endl;
    AccPlanning::generate_planning();


    if (best_coef.first == 0.0 && best_coef.second == 0.0) {
        std::cout << "No valid line found during planning." << std::endl;
    } else {
        std::cout << "Best line found with coefficients: a = " << best_coef.first << ", b = " << best_coef.second << std::endl;
    }

    std::cout << "Publishing trajectory..." << std::endl;
    AccPlanning::publish_trajectory();
    std::cout << "Trajectory published." << std::endl;
}


void AccPlanning::generate_planning() {
    int max_iterations = 500;
    double ransac_threshold = 0.2;
    int min_inliers_required = 3; 
    best_coef = {0.0, 0.0};
    std::pair<double, double> current_best_coef{0.0, 0.0}; 
    std::random_device rd;
    std::mt19937 gen(rd());


    while (cones_.points.size() >= min_inliers_required) {
        int max_inliers = 0;
        std::pair<double, double> best_coef{0.0, 0.0};
        std::vector<int> best_inliers_indices;

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
                
                best_coef = {a, (b1 + b2) / 2.0};
                max_inliers = inliers;
                best_inliers_indices = inliers_indices;
            }
        }


        if (max_inliers < min_inliers_required) {
            break;
        }
        best_coef = current_best_coef;
   
        std::cout << "Best line coefficients: a = " << best_coef.first << ", b = " << best_coef.second << "\n";
        std::cout << "Number of inliers for this line: " << max_inliers << "\n";

        // Elimina los puntos inliers encontrados de `cones_.points`
        std::sort(best_inliers_indices.rbegin(), best_inliers_indices.rend()); 
        for (int idx : best_inliers_indices) {
            cones_.points.erase(cones_.points.begin() + idx);
        }
    }
    


}

    


void AccPlanning::publish_trajectory(){

    double step = 0.1;  
    double max_distance = 150.0; 
    double a = best_coef.first; 
    double b = best_coef.second;

    for (double x = 0.0; x <= max_distance; x += step) {
        common_msgs::msg::PointXY point;
        point.x = x;
        point.y = a * x + b;  
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