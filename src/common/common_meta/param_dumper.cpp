#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <cstdio>
#include <memory>
#include <array>
#include <string>
#include <vector>

class ParamDumper : public rclcpp::Node
{
public:
    ParamDumper()
    : Node("param_dumper")
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("param_dump", 10);

        dump_all_once();
        rclcpp::shutdown();
    }

private:
    void dump_all_once()
    {
        std::vector<std::string> nodes_to_check = {"can_interface",
                                                    "car_state",
                                                    "controller",
                                                    "epos_interface",
                                                    "graph_slam",
                                                    "path_planning",
                                                    "skidpad_planning",
                                                    "acc_planning",
                                                    "perception",
                                                    "trajectory_optimization",
                                                    "visualization"};


        for (const auto& node : nodes_to_check)
        {
            std::string cmd = "ros2 param dump " + node;
            std::string output = exec_command(cmd.c_str());

            std_msgs::msg::String msg;
            msg.data = "[" + node + "] node:\n" + output;
            publisher_->publish(msg);
        }

    }

    std::string exec_command(const char* cmd)
    {
        std::array<char, 256> buffer;
        std::string result;
        FILE* pipe = popen(cmd, "r");
        if (!pipe)
            return "Error executing the command.";

        while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
            result += buffer.data();

        pclose(pipe);
        return result;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParamDumper>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
