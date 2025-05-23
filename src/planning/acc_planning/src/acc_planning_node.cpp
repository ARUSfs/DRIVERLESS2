#include "acc_planning_node.hpp"



AccPlanning::AccPlanning() : Node("acc_planning_node")
{
    this->declare_parameter<std::string>("perception_topic", "/arussim/perception");
    this->declare_parameter<std::string>("trajectory_topic", "/acc_planning/trajectory");
    this->declare_parameter<double>("target_speed", 10.0);
    this->declare_parameter<double>("max_acc", 5.0);
    this->declare_parameter<double>("max_dec", 5.0);
    this->declare_parameter<double>("track_length", 75.0);
    this->get_parameter("perception_topic", kPerceptionTopic);
    this->get_parameter("trajectory_topic", kTrajectoryTopic);
    this->get_parameter("target_speed", kTargetSpeed);
    this->get_parameter("max_acc", kMaxXAcc);
    this->get_parameter("max_dec", kMaxDec);
    this->get_parameter("track_length", kTrackLength);

    calculate_profiles();

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
        return;
    }

    AccPlanning::generate_planning();


    if (a_ == 0.0 && b_ == 0.0) {
        RCLCPP_ERROR(this->get_logger(), "No valid line found during planning.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Best line found with coefficients: a = %f, b = %f", a_, b_);
    }

    AccPlanning::publish_trajectory();
}

void AccPlanning::calculate_profiles() {
    double step = 0.1;  

    s_.clear();
    speed_profile_.clear();
    acc_profile_.clear();

    for (int i=0; i<2*kTrackLength/step; i++){ 
        s_.push_back(i*step);
        speed_profile_.push_back(0.0);
    }

    speed_profile_[0] = 1;

    for (size_t i = 1; i < s_.size(); ++i) {
        double ds = step;
        if(s_[i] < kTrackLength){
            speed_profile_[i] = std::min(kTargetSpeed, sqrt(pow(speed_profile_[i - 1], 2) + 2 * kMaxXAcc * ds));
        } else {
            speed_profile_[i] = std::max(0.0, sqrt(pow(speed_profile_[i - 1], 2) - 2 * kMaxDec * ds));
        }
    }

    for (size_t i = 0; i < speed_profile_.size(); ++i) {
        double ds = step;
        acc_profile_.push_back((pow(speed_profile_[i + 1], 2) - pow(speed_profile_[i], 2)) / (2 * ds));
    }
}


void AccPlanning::generate_planning() {
    int max_iterations = 500;
    double ransac_threshold = 0.5;
    int min_inliers_required = 3; 
    double max_allowed_deviation = 10;
    std::random_device rd;
    std::mt19937 gen(rd());

    double a_initial = 0.0;
    double b_initial = 0.0;

    double best_a1 = 0.0, best_b1 = 0.0;
    double best_a2 = 0.0, best_b2 = 0.0;

    double max_score = 0.0;

    std::uniform_int_distribution<> distrib(0, cones_.points.size() - 1);

    std::vector<int> inliers_indices1;

    for (int iter = 0; iter < max_iterations; ++iter) {
        int i = distrib(gen);
        int j = distrib(gen);
        auto& c1 = cones_.points[i];
        auto& c2 = cones_.points[j];

        if (std::abs(c1.x - c2.x) < 0.1) {
            continue;
        }

        double a1 = (c2.y - c1.y) / (c2.x - c1.x);
        double b1_1 = c1.y - c1.x * a1;

        double score = 0.0;
        std::vector<int> temp_inliers_indices;

        for (int idx = 0; idx < cones_.points.size(); ++idx) {
            const auto& cone = cones_.points[idx];
            double d = std::abs(a1 * cone.x + b1_1 - cone.y) / std::sqrt(a1 * a1 + 1);
            if (d < ransac_threshold) {
                score += 1.0 - (d / ransac_threshold);
                temp_inliers_indices.push_back(idx);
            }
        }
        
        if (score > max_score) {
            best_a1 = a1;
            best_b1 = b1_1;
            max_score = score;
            inliers_indices1 = temp_inliers_indices;
        }
    }
    a_initial = best_a1;
    b_initial = best_b1;

    pcl::PointCloud<ConeXYZColorScore> remaining_cones;
    for (int idx = 0; idx < cones_.points.size(); ++idx) {
        if (std::find(inliers_indices1.begin(), inliers_indices1.end(), idx) == inliers_indices1.end()) {
            remaining_cones.push_back(cones_.points[idx]);
        }
    }

    if (remaining_cones.size() < 2) {
        return;
    }

    max_score = 0;
    std::uniform_int_distribution<> distrib_remaining(0, remaining_cones.size() - 1);

    for (int iter2 = 0; iter2 < max_iterations; ++iter2) {
        int i = distrib_remaining(gen), j = distrib_remaining(gen);
        const auto& c1 = remaining_cones.points[i];
        const auto& c2 = remaining_cones.points[j];

        if (std::abs(c1.x - c2.x) < 0.1) {
            continue;
        }

        double a2 = (c2.y - c1.y) / (c2.x - c1.x);
        double b2_1 = c1.y - c1.x * a2;

        double score = 0.0;
        for (const auto& cone : remaining_cones.points) {
            double d = std::abs(a2 * cone.x + b2_1 - cone.y) / std::sqrt(a2 * a2 + 1);
            if (d<ransac_threshold) {
                score += 1.0 - (d / ransac_threshold);
            }
        }

        if (score > max_score) {
            best_a2 = a2;
            best_b2 = b2_1;
            max_score = score;
        }
    }


    double a_temp = (best_a1 + best_a2) / 2;
    double b_temp = (best_b1 + best_b2) / 2;
    if (std::abs(a_temp - a_initial) > max_allowed_deviation || std::abs(b_temp - b_initial) > max_allowed_deviation) {
        std::cout << "Deviation too high. Keeping initial trajectory." << std::endl;
        return;
    }

    a_ = a_temp;
    b_ = b_temp;

    if (a_history_.size() >= history_size_) {
        sum_a_ -= a_history_.front();
        a_history_.erase(a_history_.begin());
    }
    if (b_history_.size() >= history_size_) {
        sum_b_ -= b_history_.front();
        b_history_.erase(b_history_.begin());
    }

    a_history_.push_back(a_);
    b_history_.push_back(b_);

    sum_a_ += a_;
    sum_b_ += b_;

    a_ = sum_a_ / a_history_.size();
    b_ = sum_b_ / b_history_.size();

}





void AccPlanning::publish_trajectory() {
    common_msgs::msg::Trajectory trajectory_msg;
    double step = 0.1;

    for (size_t i = 0; i < s_.size(); ++i) {
        common_msgs::msg::PointXY point;
        point.x = i * step;
        point.y = a_ * point.x + b_;  // Updates the slope of the trajectory
        trajectory_msg.points.push_back(point);

        trajectory_msg.speed_profile.push_back(speed_profile_[i]);
        trajectory_msg.acc_profile.push_back(acc_profile_[i]);
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