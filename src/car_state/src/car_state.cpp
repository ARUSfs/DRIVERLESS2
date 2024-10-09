#include "car_state/car_state.hpp"

CarState::CarState()
: Node("car_state_node"), x_(0.0), y_(0.0), yaw_(0.0), vx_(0.0), vy_(0.0), r_(0.0)
{
    // Initialize the subscriber to receive Float32MultiArray messages
    sub_sim_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/arussim/state", 1, std::bind(&CarState::sim_state_callback, this, std::placeholders::_1));

    sub_can_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/can", 1, std::bind(&CarState::can_state_callback, this, std::placeholders::_1));
    
    sub_imu_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/imu", 1, std::bind(&CarState::imu_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "CarState node initialized with std_msgs");
}

void CarState::sim_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // Check if the message contains sufficient data before accessing it
    if (msg->data.size() >= 6) {
        // Store the received data
        x_ = msg->data[0];
        y_ = msg->data[1];
        yaw_ = msg->data[2];
        vx_ = msg->data[3];
        vy_ = msg->data[4];
        r_ = msg->data[5];

        RCLCPP_INFO(this->get_logger(), "Received data - x: %f, y: %f, yaw: %f, vx: %f, vy: %f, r: %f",
                    x_, y_, yaw_, vx_, vy_, r_);

        // Call the data processing function
        process_data();
    } else {
        RCLCPP_WARN(this->get_logger(), "Received insufficient data in Float32MultiArray");
    }
}

void CarState::can_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{

}
void CarState::imu_state_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{

}

void CarState::process_data()
{
    // Example: compute total speed from the velocity components
    float speed = std::sqrt(vx_ * vx_ + vy_ * vy_);  // Example: calculate total speed

    RCLCPP_INFO(this->get_logger(), "Processed speed: %f", speed);

    // You can continue processing other data as needed
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

