#include "car_state/car_state.hpp"

CarState::CarState()
: Node("car_state_node"),   x_(0.0), 
                            y_(0.0),
                            vx_(0.0), 
                            vy_(0.0),
                            ax_(0.0),
                            ay_(0.0),
                            yaw_(0.0), 
                            r_(0.0),
                            delta_(0.0)
{
    sub_extensometer_ = this->create_subscription<std_msgs::msg::Float32>(
        "/sensors/extensometer", 1, std::bind(&CarState::extensometer_callback, this, std::placeholders::_1));

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/sensors/imu", 1, std::bind(&CarState::imu_callback, this, std::placeholders::_1));

    sub_motor_speed_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motor_speed", 1, std::bind(&CarState::motor_speed_callback, this, std::placeholders::_1));

    pub_state_ = this->create_publisher<common_msgs::msg::State>(
        "/car_state/state", 1);

        // Configure timer once in the constructor based on the selected controller and frequency
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / 10)),
        [this]() {
            process_data();
        });

    RCLCPP_INFO(this->get_logger(), "CarState node initialized");
}

void CarState::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
        ax_ = msg-> linear_acceleration.x;
        ay_ = msg-> linear_acceleration.y;
        r_ = msg->angular_velocity.z;

        // Convert orientation (quaternion) to Euler angles to get yaw (Z-axis rotation)
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw); // Extract roll, pitch, yaw from the quaternion
        yaw_ = yaw; // Store yaw value
}

void CarState::extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    delta_ = msg->data;
}

void CarState::motor_speed_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    vx_ = msg->data;
}

void CarState::process_data()
{
    // Create a State message and populate the fields with aggregated data
    auto state_msg = common_msgs::msg::State();
    
    // Fill the state message with sensor data
    state_msg.vx = vx_;    
    state_msg.vy = vy_;    
    state_msg.yaw = yaw_;  
    state_msg.x = x_;      
    state_msg.y = y_;      
    state_msg.r = r_;      
    state_msg.ax = ax_;
    state_msg.ay = ay_;
    state_msg.delta = delta_;

    // Publish the message to the /car_state/state topic
    pub_state_->publish(state_msg);

    // Print information to the console
    RCLCPP_INFO(this->get_logger(), "Publishing state: vx = %f, vy = %f, yaw = %f, x = %f, y = %f, r = %f, ax = %f, ay = %f, delta = %f",
            vx_, vy_, yaw_, x_, y_, r_, ax_, ay_, delta_);

}


int main(int argc, char * argv[])
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create an instance of the CarState class
    auto car_state_node = std::make_shared<CarState>();

    // Run the node
    rclcpp::spin(car_state_node);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}

