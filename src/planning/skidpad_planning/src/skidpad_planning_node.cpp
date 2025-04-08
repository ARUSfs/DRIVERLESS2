#include "skidpad_planning_node.hpp"



SkidpadPlanning::SkidpadPlanning() : Node("skidpad_planning_node") 
{

    this->declare_parameter<std::string>("perception_topic", "/slam/map");
    this->declare_parameter<std::string>("trajectory_topic", "/skidpad_planning/trajectory");
    this->declare_parameter<double>("planning_time", 5.0);
    this->declare_parameter<double>("target_first_lap", 5.0);
    this->declare_parameter<double>("target_second_lap", 10.0);
    this->declare_parameter<double>("route_spacing", 0.5);
    this->declare_parameter<double>("top_accx_forwards", 5.0);
    this->declare_parameter<double>("top_accx_backwards", 5.0);
    this->declare_parameter<double>("top_accy", 5.0);
    this->declare_parameter<double>("step_width_1", 2);
    this->declare_parameter<double>("step_width_2", 2);


    this->get_parameter("perception_topic", kPerceptionTopic);
    this->get_parameter("trajectory_topic", kTrajectoryTopic);
    this->get_parameter("planning_time", kPlanningTime);
    this->get_parameter("target_first_lap", kTargetFirstLap);
    this->get_parameter("target_second_lap", kTargetSecondLap);
    this->get_parameter("route_spacing", kRouteSpacing);
    this->get_parameter("top_accx_forwards", kMaxXAccForwards);
    this->get_parameter("top_accx_backwards", kMaxXAccBackwards);
    this->get_parameter("top_accy", kMaxYAcc);
    this->get_parameter("step_width_1", kStepWidth1);
    this->get_parameter("step_width_2", kStepWidth2);

    start_time_ = this->now();
    initialize_skidpad(kRouteSpacing, 9.125, kTargetFirstLap, kTargetSecondLap);

    // Publish resulting trajectory
    trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>(kTrajectoryTopic, 10);
    // Subscribe to the perception point cloud topic
    perception_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        kPerceptionTopic, 10, std::bind(&SkidpadPlanning::perception_callback, this, std::placeholders::_1));


}


void SkidpadPlanning::initialize_skidpad(double spacing, double circle_radius, 
                                         double first_lap_speed, double second_lap_speed) {
    template_.clear();
    speed_profile_.clear();

    int circle_points = 2*M_PI*circle_radius/spacing;
    double ax1 = kMaxXAccForwards;
    double ax2 = kMaxXAccBackwards;
    double ay = kMaxYAcc;
    double n1 = kStepWidth1/spacing;
    double n2 = kStepWidth2/spacing;
    
    // Initialize straight section
    for (int i = 0; i <= 20 / spacing; ++i) {
        template_.emplace_back(-20 + spacing * i, 0);
        speed_profile_.push_back(first_lap_speed);
        k_.push_back(0.0);
    }

    // Initialize first circle section
    for (int i = 0; i < circle_points - n1; ++i) { 
        template_.emplace_back(circle_radius * std::sin(2 * M_PI * i / circle_points),
                               -circle_radius + circle_radius * std::cos(2 * M_PI * i / circle_points));
        speed_profile_.push_back(min(first_lap_speed, ay * circle_radius));
        k_.push_back(-1/circle_radius);
    }

    // Initialize second circle section
    for (int i = -n1; i < circle_points + n2; ++i) { 
        template_.emplace_back(circle_radius * std::sin(2 * M_PI * i / circle_points),
                               -circle_radius + circle_radius * std::cos(2 * M_PI * i / circle_points));
        speed_profile_.push_back(min(second_lap_speed, ay * circle_radius));
        k_.push_back(-1/circle_radius);
    }

    // Initialize third circle section
    for (int i = n2; i < circle_points - n1; ++i) { 
        template_.emplace_back(circle_radius * std::sin(2 * M_PI * i / circle_points),
                               circle_radius - circle_radius * std::cos(2 * M_PI * i / circle_points));
        speed_profile_.push_back(min(first_lap_speed, ay * circle_radius));
        k_.push_back(1/circle_radius);
    }

    // Initialize fourth circle section
    for (int i = -n1; i < circle_points; ++i) { 
        template_.emplace_back(circle_radius * std::sin(2 * M_PI * i / circle_points),
                               circle_radius - circle_radius * std::cos(2 * M_PI * i / circle_points));
        speed_profile_.push_back(min(second_lap_speed, ay * circle_radius));
        k_.push_back(1/circle_radius);
    }

    // Initialize final straight section
    for (int i = 0; i <= 8 / spacing; ++i) {
        template_.emplace_back(spacing * i, 0);
        speed_profile_.push_back(first_lap_speed);
        k_.push_back(0.0);
    }
    for (int i = 0; i <= 12 / spacing; ++i) {
        template_.emplace_back(8 + spacing * i, 0);
        speed_profile_.push_back(0.0);
        k_.push_back(0.0);
    }

    double s_acum = 0.0;
    for (int i = 0; i < template_.size(); ++i) {
        s_.push_back(s_acum);
        s_acum += spacing;
    }


    // Compute speed profile
    // Smooth deccelerations
    for(int i = 0; i< speed_profile_.size(); i++){
        if(speed_profile_[i-1]>speed_profile_[i]){
            speed_profile_[i] = max(speed_profile_[i], sqrt(pow(speed_profile_[i-1],2)-2*ax2*spacing));
        }
    }
    // Smooth accelerations
    for(int i = speed_profile_.size()-1; i>0; i--){
        if(speed_profile_[i] < speed_profile_[i+1]){
            speed_profile_[i] = max(speed_profile_[i], sqrt(pow(speed_profile_[i+1],2)-2*ax1*spacing));
        }
    }
    // Smooth start
    for (int i = 0; i <= 4/spacing; ++i) {
        speed_profile_[i] = 1.0;
        
    }
    for (int i = 4/spacing; i <= 20/spacing; ++i) {
        if(speed_profile_[i-1]<first_lap_speed){
            speed_profile_[i] = min(first_lap_speed, sqrt(pow(speed_profile_[i-1],2)+ax1*spacing));
        }
    }

    // Compute acceleration profile
    for (int i = 0; i<speed_profile_.size()-1; i++){
            acc_profile_.push_back((pow(speed_profile_[i+1], 2)-pow(speed_profile_[i], 2))/(2*spacing));
        
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

void SkidpadPlanning::perception_callback(sensor_msgs::msg::PointCloud2::SharedPtr per_msg) {

    cones_ = SkidpadPlanning::convert_ros_to_pcl(per_msg);

    if(this->now().seconds() - start_time_.seconds() < kPlanningTime ){

        if (cones_.points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No points in the PointCloud.");
            return;
        }

        const int N_iterations = 100; // RANSAC iterations
        const double threshold = 0.2; // RANSAC threshold
        const double radius_target1 = 7.625;
        const double radius_target2 = 10.625;
        const double distance_between_centers = 18.25;

        int max_inliers = 0;
        // Point struct from dbscan.h
        Point best_center = Point();

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
    

        
            if ((std::abs(r - radius_target1) < 2.0 || std::abs(r - radius_target2) < 2.0) && x_center > 10.0) {
                int inliers = 0;

                for (size_t idx = 0; idx < cones_.points.size(); ++idx) {
                    const auto& cone = cones_.points[idx];
                    double d = std::sqrt(std::pow(cone.x - x_center, 2) + std::pow(cone.y - y_center, 2));

                    if (std::abs(d - radius_target1) < threshold || std::abs(d - radius_target2) < threshold) {
                        ++inliers;
                    }
                }

                if (inliers > max_inliers) {
                    best_center.x = x_center;
                    best_center.y = y_center;
                    max_inliers = inliers;
                }
            }
        }

        // Update remaining cones
        for (size_t idx = 0; idx < cones_.points.size(); ++idx) {
            double dx = cones_.points[idx].x - best_center.x;
            double dy = cones_.points[idx].y - best_center.y;
            double d = std::sqrt(dx * dx + dy * dy);
            if (std::abs(d - radius_target1) > threshold && std::abs(d - radius_target2) > threshold) {
                remaining_cones.push_back(cones_.points[idx]);
            }
        }

        // first_center_candidates.emplace_back(best_center.first, best_center.second);
        best_center.z = 0.0;
        best_center.clusterID = UNCLASSIFIED;
        if (best_center.x != 0.0){
            centers.push_back(best_center);
        }
        std::cout << "First center: (" << best_center.x << ", " << best_center.y << ")" << std::endl;

        Point best_center2 = Point();
        max_inliers = 0;
        // RANSAC for second center
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
                    std::abs(distance_between_centers - std::sqrt(std::pow(best_center.x - x_center, 2) + std::pow(best_center.y - y_center, 2))) < 2.0) {
                    int inliers = 0;

                    for (const auto& cone : remaining_cones.points) {
                        double d = std::sqrt(std::pow(cone.x - x_center, 2) + std::pow(cone.y - y_center, 2));
                        if (std::abs(d - radius_target1) < threshold || std::abs(d - radius_target2) < threshold) {
                            ++inliers;
                        }
                    }

                    if (inliers > max_inliers) {
                        best_center2.x = x_center;
                        best_center2.y = y_center;
                        max_inliers = inliers;
                    }
                }
            }
            // second_center_candidates.emplace_back(best_center.first, best_center.second);
            best_center2.z = 0.0;
            best_center2.clusterID = UNCLASSIFIED;
            if (best_center2.x != 0.0){
                centers.push_back(best_center2);
            }
            std::cout << "Second center: (" << best_center2.x << ", " << best_center2.y << ")" << std::endl;
        }

        
    } else {
        if (!trajectory_calculated_) {

            DBSCAN ds(3, 2, centers);
            ds.run();

            int cluster1 = 0;
            int cluster2 = 0;
            int max_s = 0;
            for(int i = 0; i < ds.n_clusters; i++){
                if(ds.clusters[i].size() > max_s){
                    max_s = ds.clusters[i].size();
                    cluster1 = i;
                }
            }

            max_s = 0;
            for(int i = 0; i < ds.n_clusters; i++){
                if(ds.clusters[i].size() > max_s && i != cluster1){
                    max_s = ds.clusters[i].size();
                    cluster2 = i;
                }
            }

            for (int i = 0; i < ds.m_points.size(); i++) {
                if (ds.m_points[i].clusterID == cluster1) {
                    left_center = {ds.m_points[i].x, ds.m_points[i].y};
                } else if (ds.m_points[i].clusterID == cluster2) {
                    right_center = {ds.m_points[i].x, ds.m_points[i].y};
                }
            }

            if(left_center.second < right_center.second){
                std::swap(left_center, right_center);
            }

            RCLCPP_INFO(this->get_logger(), "Left center: (%f, %f)", left_center.first, left_center.second);
            RCLCPP_INFO(this->get_logger(), "Right center: (%f, %f)", right_center.first, right_center.second);
        
            // Check if the final centers are valid. 
            // Centers should be 18.25 meters apart and not too close to the origin
            if(std::abs(std::sqrt(pow(left_center.first - right_center.first, 2) + 
                    pow(left_center.second - right_center.second, 2)) - 18.25) > 5.0 ||
                    left_center.first < 5.0 || right_center.first < 5.0){
                RCLCPP_ERROR(this->get_logger(), "Error: Invalid centers detected. Restarting planning.");
                start_time_ = this->now();
                return;
            }

            trajectory_calculated_ = true;

            
            publish_trajectory();
            
        } else {
            publish_trajectory();
        }
    }  
}


void SkidpadPlanning::publish_trajectory() {
    common_msgs::msg::Trajectory trajectory_msg;

    if (left_center.first == 0.0 || left_center.second == 0.0 || 
        right_center.first == 0.0 || right_center.second == 0.0) {
        RCLCPP_ERROR(this->get_logger(), "Error: Invalid centers detected. Trajectory will not be published.");
        return;
    }

    // Compute the skidpad center
    double mid_x = (left_center.first + right_center.first) / 2.0;
    double mid_y = (left_center.second + right_center.second) / 2.0;
    double orientation = - M_PI/2 + std::atan2(right_center.second - left_center.second, 
                                    right_center.first - left_center.first);

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

    for (const auto& v : speed_profile_) {
        trajectory_msg.speed_profile.push_back(v);
    }
    for (const auto& a : acc_profile_) {
        trajectory_msg.acc_profile.push_back(a);
    }
    for (const auto& s : s_) {
        trajectory_msg.s.push_back(s);
    }
    for (const auto& k : k_) {
        trajectory_msg.k.push_back(k);
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