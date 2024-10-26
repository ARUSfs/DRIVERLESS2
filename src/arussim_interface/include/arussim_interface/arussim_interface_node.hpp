/**
 * @file arussim_interface_node.hpp
 * @brief ARUSSim Interface node header for ARUS Team Driverless pipeline
 */

#include <rclcpp/rclcpp.hpp>
#include "common_msgs/msg/cmd.hpp"
#include "common_msgs/msg/cmd4_wd.hpp"
#include "common_msgs/msg/four_wheel_drive.hpp"
#include "common_msgs/msg/point_xy.hpp"
#include "common_msgs/msg/trajectory.hpp"
#include "arussim_msgs/msg/cmd.hpp"
#include "arussim_msgs/msg/cmd4_wd.hpp"
#include "arussim_msgs/msg/four_wheel_drive.hpp"
#include "arussim_msgs/msg/trajectory.hpp"


/**
 * @class ARUSSimInterface
 * @brief ARUSSimInterface class 
 * 
 * This class translates the custom messages between the simulator and the pipeline
 */
class ARUSSimInterface : public rclcpp::Node
{
    public:
        /**
         * @brief Constructor for the ARUSSimInterface class.
         */
        ARUSSimInterface();
    
    private:
        //Callbacks
        void cmd_callback(const common_msgs::msg::Cmd::SharedPtr msg);
        void cmd4wd_callback(const common_msgs::msg::Cmd4WD::SharedPtr msg);
        void wheel_speeds_callback(const arussim_msgs::msg::FourWheelDrive::SharedPtr msg);
        void trajectory_callback(const arussim_msgs::msg::Trajectory::SharedPtr msg);

        // Publishers
        rclcpp::Publisher<arussim_msgs::msg::Cmd>::SharedPtr cmd_pub_;
        rclcpp::Publisher<arussim_msgs::msg::Cmd4WD>::SharedPtr cmd4wd_pub_;
        rclcpp::Publisher<common_msgs::msg::FourWheelDrive>::SharedPtr wheel_speeds_pub_;
        rclcpp::Publisher<common_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;

        // Subscribers
        rclcpp::Subscription<common_msgs::msg::Cmd>::SharedPtr cmd_sub_;
        rclcpp::Subscription<common_msgs::msg::Cmd4WD>::SharedPtr cmd4wd_sub_;
        rclcpp::Subscription<arussim_msgs::msg::FourWheelDrive>::SharedPtr wheel_speeds_sub_;
        rclcpp::Subscription<arussim_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;

        // Topics names
        std::string kSimCmdTopic;
        std::string kSimCmd4WDTopic;
        std::string kSimWheelSpeedsTopic;
        std::string kSimTrajectoryTopic;
        std::string kPipelineCmdTopic;
        std::string kPipelineCmd4WDTopic;
        std::string kPipelineWheelSpeedsTopic;
        std::string kPipelineTrajectoryTopic;


         // Load parameters from the config file
        void get_config_parameters(){
            this->declare_parameter("sim_cmd_topic", "/arussim/cmd");
            this->declare_parameter("sim_cmd4wd_topic", "/arussim/cmd4wd");
            this->declare_parameter("sim_wheel_speeds_topic", "/arussim/wheel_speeds");
            this->declare_parameter("sim_trajectory_topic", "/arussim/fixed_trajectory");
            this->declare_parameter("pipeline_cmd_topic", "/controller/cmd");
            this->declare_parameter("pipeline_cmd4wd_topic", "/controller/cmd4wd");
            this->declare_parameter("pipeline_wheel_speeds_topic", "/arussim_interface/wheel_speeds");
            this->declare_parameter("pipeline_trajectory_topic", "/arussim_interface/fixed_trajectory");

            this->get_parameter("sim_cmd_topic", kSimCmdTopic);
            this->get_parameter("sim_cmd4wd_topic", kSimCmd4WDTopic);
            this->get_parameter("sim_wheel_speeds_topic", kSimWheelSpeedsTopic);
            this->get_parameter("sim_trajectory_topic", kSimTrajectoryTopic);
            this->get_parameter("pipeline_cmd_topic", kPipelineCmdTopic);
            this->get_parameter("pipeline_cmd4wd_topic", kPipelineCmd4WDTopic);
            this->get_parameter("pipeline_wheel_speeds_topic", kPipelineWheelSpeedsTopic);
            this->get_parameter("pipeline_trajectory_topic", kPipelineTrajectoryTopic);
        }
};