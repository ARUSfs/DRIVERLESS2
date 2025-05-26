#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>


#include <std_msgs/msg/string.hpp>

#include <vector>
#include <cstring>



void param_export(const std::string &rosbag_path, std::string file_type)
{

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = rosbag_path; //Path to the rosbag

    // Detect file extension and set storage_id accordingly
    if (file_type == "db3") {
        storage_options.storage_id = "sqlite3";
    } else {
        storage_options.storage_id = "mcap";
    }

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr"; //ROS 2 Humble uses cdr serialization
    converter_options.output_serialization_format = "cdr";

    try {
        auto reader = std::make_unique<rosbag2_cpp::Reader>();
        reader->open(storage_options, converter_options);

        while (reader->has_next()) {
            auto bag_message = reader->read_next();
            
            if (bag_message->topic_name == "/param_dump") {
                std_msgs::msg::String param_msg;
                rclcpp::Serialization<std_msgs::msg::String> serializer;

                //Convert serialized data to rclcpp::SerializedMessage
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);

                //Deserialize the message
                serializer.deserialize_message(&serialized_msg, &param_msg);
                
                std::cout << param_msg.data << std::endl;
                

            }
            
        }
    } catch (const std::exception & e) {
        std::cerr << "Failed to open rosbag: " << e.what() << std::endl;
    }
}



int main(int argc, char * argv[])
{
    std::string rosbag_path;
    if (argc > 1) {
        rosbag_path = argv[1];
    } 

    std::string file_type_ = "mcap";

    if (argc > 2) {
        file_type_ = argv[2];
    }

    param_export(rosbag_path, file_type_);
    return 0;
}