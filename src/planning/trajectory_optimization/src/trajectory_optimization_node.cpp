/**
 * @file trajectory_optimization_node.cpp
 * @author José Manuel Landero Plaza (josemlandero05@gmail.com)
 * @brief Trajectory Optimization node implementation for ARUS Team Driverless pipeline
 */
#include "trajectory_optimization/trajectory_optimization_node.hpp"
#include "trajectory_optimization/min_curvature_path.hpp"


TrajectoryOptimization::TrajectoryOptimization() : Node("trajectory_optimization")
{   
    this->declare_parameter<int>("n_iter", 3);
    this->declare_parameter<int>("n_seg", 3);
    this->declare_parameter<double>("d_min", 1.2);
    this->get_parameter("n_iter", kNIter);
    this->get_parameter("d_min", kMinDist);
    this->get_parameter("n_seg", kNSeg);    
    
    this->declare_parameter<double>("v_max", 8.);
    this->declare_parameter<double>("mu_y", 1.1);
    this->declare_parameter<double>("mu_throttle", 0.6);
    this->declare_parameter<double>("mu_brake", 0.9);
    this->get_parameter("v_max", kVMax);
    this->get_parameter("mu_y", kMuY);
    this->get_parameter("mu_throttle", kMuXThrottle);
    this->get_parameter("mu_brake", kMuxBrake);
    
    this->declare_parameter<double>("g", 9.81);
    this->declare_parameter<double>("m", 270.0);
    this->declare_parameter<double>("c_L", 0.5*1.2*3.5);
    this->declare_parameter<double>("c_D", 0.5*1.2*1.2);
    this->get_parameter("g", kG);
    this->get_parameter("m", kMass);
    this->get_parameter("c_L", kCLift);
    this->get_parameter("c_D", kCDrag);

    this->declare_parameter<std::string>("optimized_trajectory_topic", "/trajectory_optimization/trajectory");
    this->declare_parameter<std::string>("track_limits_topic","/path_planning/track_limits");
    this->get_parameter("optimized_trajectory_topic", kOptimizedTrajectoryTopic);
    this->get_parameter("track_limits_topic", kTrackLimitsTopic);

    this->declare_parameter<bool>("debug", true);
    this->get_parameter("debug", kDebug);

    track_limits_sub_ = this->create_subscription<common_msgs::msg::TrackLimits>(
        kTrackLimitsTopic, 10, std::bind(&TrajectoryOptimization::trajectory_callback, this, std::placeholders::_1));
    optimized_trajectory_pub_ = this->create_publisher<common_msgs::msg::Trajectory>(kOptimizedTrajectoryTopic, 10);
}


void TrajectoryOptimization::trajectory_callback(common_msgs::msg::TrackLimits::SharedPtr track_limits_msg){
    // Extract track limits and trajectory from trajectory message
    track_limit_right_ = track_limits_msg->right_limit;
    track_limit_left_ = track_limits_msg->left_limit;

    common_msgs::msg::Trajectory trajectory = track_limits_msg-> trajectory;
    std::vector<common_msgs::msg::PointXY> track_xy = trajectory.points;
    
    //Convert trajectory message to vectors
    int n = track_xy.size();
    VectorXd x(n), y(n);
    for(int i = 0; i < n; i++){
        x(i) = track_xy[i].x;
        y(i) = track_xy[i].y;
    }
    
    if(!(track_limit_left_.empty())){
        //Generate track width vectors
        VectorXd twr = TrajectoryOptimization::generate_track_width(x, y, track_limit_right_);
        VectorXd twl = TrajectoryOptimization::generate_track_width(x, y, track_limit_left_);

        //Get minimal curvature path iteratively        
        MatrixXd optimized_trajectory(n,2);

        for(int i=0; i<kNIter; i++){
            double t0 = this->now().seconds();
            auto opt_out = MinCurvaturepath::get_min_curvature_path(x, y, twr, twl, kNSeg);
            optimized_trajectory = opt_out.first;
            auto status = opt_out.second;

            if(status != qpmad::Solver::OK){
                if (kDebug) RCLCPP_ERROR(this->get_logger(), "qpmad solver didn't find a solution at iteration %d!", i+1);
            } else {
                x = optimized_trajectory.col(0);
                y = optimized_trajectory.col(1);

                twr = TrajectoryOptimization::generate_track_width(x, y, track_limit_right_);
                twl = TrajectoryOptimization::generate_track_width(x, y, track_limit_left_);

                if (kDebug) {
                    RCLCPP_INFO(this->get_logger(), "Trajectory correctly optimized at iteration %d.", i+1);
                    RCLCPP_INFO(this->get_logger(), "Trajectory optimization time: %f", this->now().seconds()-t0);
                } 
            }
            
        }

        //Get accumulated distance and curvature at each point
        MatrixXd optimized_s_k = TrajectoryOptimization::get_distance_and_curvature_values(x, y);
        VectorXd optimized_s = optimized_s_k.col(0);
        VectorXd optimized_k = optimized_s_k.col(1);

        //Generate speed and acceletation profile
        MatrixXd profile = TrajectoryOptimization::generate_speed_and_acc_profile(optimized_s, optimized_k);
        VectorXd speed_profile = profile.col(0);
        VectorXd acc_profile = profile.col(1);    

        //Create and publish trajectory message
        common_msgs::msg::Trajectory optimized_traj_msg = TrajectoryOptimization::create_trajectory_msg(x, y, optimized_s, optimized_k, speed_profile, acc_profile);
        optimized_trajectory_pub_ -> publish(optimized_traj_msg);

        if (kDebug) RCLCPP_INFO(this->get_logger(), "Optimized trajectory correctly published. \n");

    } else {
        if (kDebug) RCLCPP_ERROR(this->get_logger(), "Track limits empty! \n");
    }
}


VectorXd TrajectoryOptimization::generate_track_width(VectorXd x, VectorXd y, std::vector<common_msgs::msg::PointXY> track_limit){
    int n = x.size();
    int m = track_limit.size();
    VectorXd track_width = VectorXd::Zero(n);

    for(int i = 0; i < n; i++){
        // Calculate point's distance² to track limits
        double min_dist = 100;
        for(int j = 0; j < m; j++){
            double dx = x(i) - track_limit[j].x;
            double dy = y(i) - track_limit[j].y;
            double dist_iter = dx*dx + dy*dy;
            if (dist_iter < min_dist){min_dist = dist_iter;}
        }
        // Ensure minimal distance
        track_width(i) = std::max(std::sqrt(min_dist) - kMinDist, 0.);
    }

    return track_width;
}

MatrixXd TrajectoryOptimization::get_distance_and_curvature_values(VectorXd traj_x, VectorXd traj_y){
    // Get the accumulated distance at each point of the trajectory (s)
    double acum = 0;
    int n = traj_x.size();
    VectorXd s(n), xp(n), yp(n);
    s(0) = 0.;

    for(int i = 0; i < n-1; i++){
        xp(i) = traj_x(i+1) - traj_x(i);    
        yp(i) = traj_y(i+1) - traj_y(i);    
        acum += sqrt(xp(i)*xp(i) + yp(i)*yp(i));    
        s(i+1) = acum;      
    }

    // Ensure s, xp and yp have the same size by repeating the last X and Y differences
    xp(n-1) = xp(n-2);       
    yp(n-1) = yp(n-2);       

    // Calculate the curvature at each point of the trajectory (k)
    VectorXd xpp(n), ypp(n), k(n);

    for(int i = 0; i < xp.size()-1; i++){
        xpp(i) = xp(i+1) - xp(i);
        ypp(i) = yp(i+1) - yp(i);
    }

    xpp(n-1) = xpp(n-2);
    ypp(n-1) = ypp(n-2);

    k = VectorXd::Zero(n);

    for(int i = 0; i < xpp.size() -1; i++){
        if(xp(i) != yp(i)){
            k(i) = (xp(i)*ypp(i) - xpp(i)*yp(i)) / pow((xp(i)*xp(i) + yp(i)*yp(i)), 1.5);
        } else {
            k(i) = 0.;
        }
    }

    MatrixXd res(n,2);
    res << s, k;

    return res;
}


MatrixXd TrajectoryOptimization::generate_speed_and_acc_profile(VectorXd s, VectorXd k){
    int m = s.size();

    VectorXd speed_profile = VectorXd::Zero(m);
    speed_profile(0) = kVMax;   // Initial max speed for first iteration
    VectorXd v_grip(m), ds(m);
    double v_max_braking;

    for(int i = 0; i < m; i++){
        v_grip(i) = std::min(kVMax, calculate_apex(k(i)));  // Max speed to maintain grip
    }

    // Forwards loop. First iteration. Calculate speed profile and limit by grip
    for(int i = 1; i < m; i++){
        ds(i) = s(i) - s(i-1);
        double ax_max = ggv_ax_throttle(speed_profile(i-1),k(i-1));
        speed_profile(i) = sqrt(speed_profile(i-1)*speed_profile(i-1) + 2*ax_max*ds(i));
        if (speed_profile(i) > v_grip(i)){                      
            speed_profile(i) = v_grip(i);                       
        }
    }
    // Close loop
    speed_profile(0) = speed_profile(m-1);

    // Forwards loop. Second iteration
    for(int i = 1; i < m; i++){
        ds(i) = s(i) - s(i-1);
        double ax_max = ggv_ax_throttle(speed_profile(i-1),k(i-1));
        speed_profile(i) = sqrt(speed_profile(i-1)*speed_profile(i-1) + 2*ax_max*ds(i));
        if (speed_profile(i) > v_grip(i)){                      
            speed_profile(i) = v_grip(i);                       
        }
    }

    // Backwards loop. First iteration. Limit speed by max braking
    for(int j = m-1; j > 0; j--){
        double ax_max_braking = ggv_ax_brake(speed_profile(j),k(j));
        v_max_braking = sqrt(speed_profile(j)*speed_profile(j) + 2*ax_max_braking*ds(j-1));
        if (speed_profile(j-1) > v_max_braking){
            speed_profile(j-1) = v_max_braking;
        }
    }
    // Close loop
    speed_profile(m-1) = speed_profile(1);

    // Backwards loop. Second iteration
    for(int j = m-1; j > 0; j--){
        double ax_max_braking = ggv_ax_brake(speed_profile(j),k(j));
        v_max_braking = sqrt(speed_profile(j)*speed_profile(j) + 2*ax_max_braking*ds(j-1));
        if (speed_profile(j-1) > v_max_braking){
            speed_profile(j-1) = v_max_braking;
        }
    }

    // Generate acceleration profile
    VectorXd acc_profile = VectorXd::Zero(m);

    for(int i = 1; i < m; i++){
        acc_profile(i) = (speed_profile(i)*speed_profile(i) - speed_profile(i-1)*speed_profile(i-1)) / (2*ds(i-1));
    }
    // Close loop
    acc_profile(1) = acc_profile(m-1);

    MatrixXd res(m,2);
    res << speed_profile, acc_profile;

    return res;
}


double TrajectoryOptimization::calculate_apex(double k){

    if (std::abs(k) < 0.01) {k = 0.01;}
    double v_grip = std::sqrt(kMuY * kG / (std::abs(k) - kMuY * kCLift / kMass));
    return v_grip;
}


double TrajectoryOptimization::ggv_ax_throttle(double v, double k){
    double ay_max_v = kMuY * (kG + kCLift/kMass * v*v);
    double ax_max_v = kMuXThrottle * kG + (kMuXThrottle * kCLift - kCDrag) / kMass *v*v;

    double ay = (std::abs(v*v*k) < ay_max_v) ? std::abs(v*v*k) : ay_max_v;
    double ax_max = ax_max_v * sqrt(1 - std::pow((ay / ay_max_v),2));
    return ax_max;
}


double TrajectoryOptimization::ggv_ax_brake(double v, double k){
    double ay_max_v = kMuY * (kG + kCLift/kMass * v*v);
    double ax_max_v = kMuxBrake * kG + (kMuXThrottle * kCLift + kCDrag) / kMass *v*v;

    double ay = (std::abs(v*v*k) < ay_max_v) ? std::abs(v*v*k) : ay_max_v;
    double ax_max = ax_max_v - ax_max_v * ay / ay_max_v;
    return ax_max;
}


common_msgs::msg::Trajectory TrajectoryOptimization::create_trajectory_msg(VectorXd traj_x, VectorXd traj_y, 
    VectorXd s, VectorXd k, VectorXd speed_profile, VectorXd acc_profile){
    
    common_msgs::msg::Trajectory traj_msg;

    for(int i = 0; i < traj_x.size(); i++){
        common_msgs::msg::PointXY p;
        p.x = traj_x(i);
        p.y = traj_y(i);
        traj_msg.points.push_back(p);
        traj_msg.s.push_back(s(i));
        traj_msg.k.push_back(k(i));
        traj_msg.speed_profile.push_back(speed_profile(i));
        traj_msg.acc_profile.push_back(acc_profile(i));
    }

    return traj_msg;
}



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryOptimization>());
    rclcpp::shutdown();
    return 0;
}