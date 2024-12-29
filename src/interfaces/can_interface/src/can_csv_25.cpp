/**
 * @file can_csv_25.cpp
 * @author Álvaro Galisteo Bermúdez (galisbermo03@gmail.com)
 * @brief Main file for the CAN interface node. Contains the main function to read 
 * the CAN lines and parse the messages according to the csv data. A new implementation
 * and concept to archive modularity and be easily modified when there is a change or a new ID.
 * @version 0.1
 * @date 25-12-2024
 * 
 */
#include "can_interface/can_csv_25.hpp"

bool DEBUG = true;

CanInterface::CanInterface() : Node("can_interface"){
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("can_interface");

    std::string path_can = package_share_directory + "/can.csv";
    csvdata_main = readCSV(path_can);

    std::string path_can_aux = package_share_directory + "/can_aux.csv";
    csvdata_aux = readCSV(path_can_aux);

    // Create publishers from the csv file and store them in a key-vector pair
    for (const auto& [key, vector] : csvdata_main) {
        std::string topic = vector[4];
        publishers[key] = this->create_publisher<std_msgs::msg::Float32>(topic, 10);
    }

    if (DEBUG) {
        std::cout << "Main CSV Data Loaded:\n";
        for (const auto &par : csvdata_main) {
            std::cout << "Key: " << par.first << "\nValues:";
            for (const auto &valor : par.second) {
                std::cout << " " << valor;
            }
            std::cout << "\n\n";
        }

        std::cout << "Aux CSV Data Loaded:\n";
        for (const auto &par : csvdata_aux) {
            std::cout << "Key: " << par.first << "\nValues:";
            for (const auto &valor : par.second) {
                std::cout << " " << valor;
            }
            std::cout << "\n\n";
        }
    }

    // Configure socketCan0 for the CAN bus 0.
    const char *can_interface0 = "can0"; 

    socketCan0 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketCan0 < 0) {
        perror("Error while opening can0 socket");
        return;
    } else{
        std::cout << "can0 enabled for writing" << std::endl;
    }

    std::strncpy(ifr.ifr_name, can_interface0, IFNAMSIZ - 1);
    if (ioctl(socketCan0, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting can0 interface index");
        return ;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socketCan0, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Error in binding socketCan0");
            return;
    }


    // Configure socketCan1 for the CAN bus 1.
    const char *can_interface1 = "can1"; 

    socketCan1 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketCan1 < 0) {
        perror("Error while opening can1 socket");
        return;
    } else{
        std::cout << "can1 enabled for writing" << std::endl;
    }

    std::strncpy(ifr.ifr_name, can_interface1, IFNAMSIZ - 1);
    if (ioctl(socketCan1, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error getting can1 interface index");
        return ;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socketCan1, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in binding socketCan1");
        return;
    }

    std::thread thread_0(&CanInterface::readCan, this, socketCan0);
    std::thread thread_1(&CanInterface::readCan, this, socketCan1);

    thread_0.detach();
    thread_1.detach();
}

std::map<std::string, std::vector<std::string>> CanInterface::readCSV(const std::string &filepath)
{
    std::map<std::string, std::vector<std::string>> localCsvData;

    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Error opening the csv: " << filepath << std::endl;
        return localCsvData; // return an empty map if file opening fails
    }

    std::string line;

    // Skip the first line (header)
    std::getline(file, line);

    // Read the file line by line
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string element;
        std::vector<std::string> row;

        // Separate the line by commas
        while (std::getline(ss, element, ',')) {
            row.push_back(element);
        }

        if (!row.empty()) {
            // The first element is the key
            std::string key = row[0];
            // The other elements are stored in the vector
            std::vector<std::string> values(row.begin() + 1, row.end());
            localCsvData[key] = values;
        }
    }

    file.close();
    return localCsvData;
}

void CanInterface::readCan(int socketCan) 
{
    struct can_frame frame;
    while (rclcpp::ok()) {
        int nbytes = read(socketCan, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("CAN read error");
            continue;
        }

        // Convert frame ID to string (hex format)
        std::ostringstream id;
        id << "0x" << std::hex << frame.can_id;
        std::string frame_id = id.str();
        
        // Check if frame_id exists in csvdata_aux
        auto aux_iter = csvdata_aux.find(frame_id);
        if (aux_iter == csvdata_aux.end()) {
            std::cerr << "No matching key in csvdata_aux for key: " << frame_id << std::endl;
            break;
        }

        // Process auxiliary vector
        const auto &aux_vector = aux_iter->second;

        // Check if there is a subID to filter
        if (filterSubID(frame, aux_vector[0])) {
            // Build multiple dynamic keys
            int num_keys = std::stoi(aux_vector[1]);
            for (int i = 1; i <= num_keys; ++i) {
                std::string dynamic_key = frame_id;
                dynamic_key += std::to_string(i);

                // Check for matching entry in csvdata_main
                auto main_iter = csvdata_main.find(dynamic_key);
                if (main_iter == csvdata_main.end()) {
                    std::cerr << "No matching key in csvdata_main for key: " << dynamic_key << std::endl;
                    continue;
                }

                // Convert configuration from main_iter->second to CANParseConfig
                const auto &main_vector = main_iter->second;
                CANParseConfig config;
                config.startByte = std::stoi(main_vector[0]);
                config.endByte = std::stoi(main_vector[1]);
                config.scale = std::stof(main_vector[2]);
                config.offset = std::stof(main_vector[3]);
                config.key = dynamic_key;
                    
                // Call parseMsg with the populated CANParseConfig
                parseMsg(frame, config);
            
                if (DEBUG) {
                std::cout << "Matched CAN frame ID: " << frame_id 
                        << " with dynamic key: " << dynamic_key 
                        << " Data: ";
                for (const auto &val : main_iter->second) {
                    std::cout << val << " ";
                }
                std::cout << std::endl;
                }
            }
        } else {
            continue;
        }
    }
}


void CanInterface::parseMsg(const struct can_frame& frame, const CANParseConfig& config) {
    float rawValue = 0;

    // Calculate the number of bytes
    uint8_t numBytes = config.endByte - config.startByte + 1;

    if (numBytes == 1) { 
        rawValue = frame.data[config.startByte];
    } 
    else if (numBytes == 2) { 
        rawValue = (frame.data[config.endByte] << 8) | frame.data[config.startByte];
    } 
    else if (numBytes == 3) { 
        rawValue = (frame.data[config.startByte + 2] << 16) | 
                       (frame.data[config.startByte + 1] << 8) | 
                       frame.data[config.startByte];
    } 
    else if (numBytes == 4) { 
        rawValue = (frame.data[config.startByte + 3] << 24) |
                       (frame.data[config.startByte + 2] << 16) |
                       (frame.data[config.startByte + 1] << 8) |
                       frame.data[config.startByte];
    } 
    else {
        printf("Unsupported byte range: %d bytes\n", numBytes);
    }

    float scaledValue = rawValue * config.scale + config.offset;

    // Find the associated publisher, create the message and publish the scaled value.
    auto pub_iter = publishers.find(config.key);
    if (pub_iter != publishers.end()) {
        std_msgs::msg::Float32 msg;
        msg.data = scaledValue;

        pub_iter->second->publish(msg);

        if (DEBUG) {
            std::cout << "Published value: " << scaledValue << " on topic associated with key: " 
                      << config.key << std::endl;
        }
    } else {
            std::cerr << "No matching publisher in publishers for key: " << config.key << std::endl;
    }
}

bool CanInterface::filterSubID(const struct can_frame& frame, const std::string& aux_vector_subID) {
    if (aux_vector_subID != "no") {
        // Convert the subID in aux_vector to hex
        int subID = std::stoi(aux_vector_subID, nullptr, 16);
        if (frame.data[0] != subID) {
            return false;
        } else {
            return true;
        }
    } else {
        return true;
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CanInterface>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}