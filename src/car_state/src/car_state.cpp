#include "car_state/car_state.hpp"

CarState::CarState()
: Node("car_state_node"), x_(0.0), y_(0.0), yaw_(0.0), vx_(0.0), vy_(0.0), r_(0.0),
                        extensometer_received_(false),
                        imu_received_(false)
{
    sub_extensometer_ = this->create_subscription<std_msgs::msg::Float32>(
        "/sensors/extensometer", 1, std::bind(&CarState::extensometer_callback, this, std::placeholders::_1));

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/sensors/imu", 1, std::bind(&CarState::imu_callback, this, std::placeholders::_1));

    pub_state_ = this->create_publisher<common_msgs::msg::State>(
        "/car_state/state", 1);

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "CarState node initialized");
}

void CarState::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // Get the current time
    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds(); // Time delta in seconds

    if (dt > 0) {
        // Integrate acceleration to calculate velocity
        vx_ += msg->linear_acceleration.x * dt;
        vy_ += msg->linear_acceleration.y * dt;

        // Integrate velocity to calculate position
        x_ += vx_ * dt;
        y_ += vy_ * dt;

        // Get angular velocity around the Z-axis directly
        //r_ = msg->angular_velocity.z;

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

        // Log the position and orientation values
        RCLCPP_INFO(this->get_logger(), "Position: x = %f, y = %f, yaw = %f", x_, y_, yaw_);
        RCLCPP_INFO(this->get_logger(), "Velocity: vx = %f, vy = %f, r = %f", vx_, vy_, r_);
    }
    
    last_time_ = current_time;

    if (extensometer_received_ && imu_received_) {
        process_data();
    }
}

void CarState::extensometer_callback(const std_msgs::msg::Float32::SharedPtr msg)
{

    r_ = msg->data;
    extensometer_received_ = true;

    if (extensometer_received_ && imu_received_) {
        process_data();
    }
}

void CarState::process_data()
{
    // Create a State message and populate the fields with aggregated data
    auto state_msg = common_msgs::msg::State();
    
    // Fill the state message with sensor data
    state_msg.vx = vx_;    // Velocity in X direction
    state_msg.vy = vy_;    // Velocity in Y direction
    state_msg.yaw = yaw_;  // Orientation (yaw)
    state_msg.x = x_;      // Position X (if applicable)
    state_msg.y = y_;      // Position Y (if applicable)
    state_msg.r = r_;      // Rotation or some other parameter (if applicable)

    // Publish the message to the /car_state/state topic
    pub_state_->publish(state_msg);

    // Print information to the console
    RCLCPP_INFO(this->get_logger(), "Publishing state: vx = %f, vy = %f, yaw = %f, x = %f, y = %f, r = %f",
                vx_, vy_, yaw_, x_, y_, r_);

    // Reset the flags to wait for new data from all three sensors
    extensometer_received_ = false;
    imu_received_ = false;
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

