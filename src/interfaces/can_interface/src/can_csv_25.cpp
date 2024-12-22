#include "can_interface/can_csv_25.hpp"

bool DEBUG = false;

CanInterface::CanInterface() : Node("can_interface"){
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("can_interface");

    std::string path_can = package_share_directory + "/can.csv";
    csvdata_main = readCSV(path_can);

    std::string path_can_aux = package_share_directory + "/can_aux.csv";
    csvdata_aux = readCSV(path_can_aux);

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

    // Configure socketCan0
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


    // Configure socketCan1
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

    motorSpeedPub = this->create_publisher<std_msgs::msg::Float32>("/can/inv_speed", 10);
    ASStatusPub = this->create_publisher<std_msgs::msg::Int16>("/can/AS_status", 10);
    GPSPub = this->create_publisher<sensor_msgs::msg::NavSatFix>("can/gps", 10);
    GPSSpeedPub = this->create_publisher<geometry_msgs::msg::Vector3>("can/gps_speed", 10);
    IMUPub = this->create_publisher<sensor_msgs::msg::Imu>("can/IMU", 10);
    steeringAnglePub = this->create_publisher<std_msgs::msg::Float32>("can/steering_angle", 10);
    RESRangePub = this->create_publisher<std_msgs::msg::Float32>("/can/RESRange", 10);
    PCTempPub = this->create_publisher<std_msgs::msg::Float32>("/pc_temp", 10);
    DL500Pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/can/DL500", 10);
    DL501Pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/can/DL501", 10);
    DL502Pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/can/DL502", 10);

    std::thread thread_0(&CanInterface::readCan(socketCan0), this);
    std::thread thread_1(&CanInterface::readCan(socketCan1), this);

    thread_0.detach();
    thread_1.detach();
}

//################################################# READ FUNCTIONS #################################################

//--------------------------------------------- CSV -------------------------------------------------------------------   

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
        std::ostringstream stream;
        stream << "0x" << std::hex << frame.can_id;
        std::string frame_id = stream.str();
        
        // Check if frame_id exists in csvdata_aux
        auto aux_iter = csvdata_aux.find(frame_id);
        if (aux_iter == csvdata_aux.end()) {
            std::cerr << "No matching key in csvdata_aux for key: " << frame_id << std::endl;
            break;
        }

        // Process auxiliary vector
        const auto &aux_vector = aux_iter->second;

        // Check if it match the subID
        if (frame.can_dlc == aux_vector[0]) {
            continue;
        } else {
            std::cerr << "No matching subID for the ID: " << frame_id << std::endl;
            break;
        }

        // Build multiple dynamic keys
        int num_keys = std::stoi(aux_vector[1]);
        for (int i = 1; i < num_keys; ++i) {
            std::string dynamic_key = frame_id;
            if (aux_vector[0] == "yes") {
                dynamic_key += std::to_string(frame.data[0]);
            }
            dynamic_key += aux_vector[1] + std::to_string(i);

            // Check for matching entry in csvdata_main
            auto main_iter = csvdata_main.find(dynamic_key);
            if (main_iter == csvdata_main.end()) {
                std::cerr << "No matching key in csvdata_main for key: " << dynamic_key << std::endl;
                continue;
            }

            // Convert configuration from main_iter->second to CANParseConfig
            for (const auto& config_data : main_iter->second) {
                CANParseConfig config;
                config.startByte = std::stoi(config_data[0]);
                config.endByte = std::stoi(config_data[1]);
                config.endianness = config_data[2];
                config.scale = std::stof(config_data[3]);
                config.offset = std::stof(config_data[4]);
                config.variable_name = config_data[5];
                
                // Call parseMsg with the populated CANParseConfig
                parseMsg(frame, config);
            }
        }

        // Search in aux_vector for nomPublisher
        if (aux_vector[2] == "IMUData" && aux_vector[3] == "no") {
            IMUData.header.stamp = this->get_clock()->now();
        } else {
            auto pub_iter = publisherMap.find(aux_vector[3]);
            if (pub_iter != publisherMap.end()) {
                auto var_iter = variableMap.find(aux_vector[2]);
                auto var = var_iter->second;
                auto pub = pub_iter->second;
                pub->publish(var)
            } else {
                std::cerr << "Publisher not found for: " << publisher_name << std::endl;
            }
        }
    
        // Debug: Print matched data
        if (true) {
            std::cout << "Matched CAN frame ID: " << frame_id 
                      << " with dynamic key: " << dynamic_key 
                      << " Data: ";
            for (const auto &val : main_iter->second) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
    }
}


void parseMsg(const struct can_frame& frame, const CANParseConfig& config) {
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
        // Unsupported number of bytes
        printf("Unsupported byte range: %d bytes\n", numBytes);
    }

    float scaledValue = rawValue * config.scale + config.offset;
    auto var_iter = variableMap.find(config.target);
    if (var_iter != variableMap.end()) {
        // Update the value via the pointer
        *var_iter->second = scaledValue;
    } else {
        std::cerr << "No matching variable in variableMap for key: " << config.target << std::endl;
    }
}

//--------------------------------------------- CAN 1 -------------------------------------------------------------------   

void CanInterface::readCan1()
{   
    struct can_frame frame;
    while (rclcpp::ok()) {
        int nbytes = read(socketCan1, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("can1 read error");
            continue;
        }

        // Debug: Print the received CAN frame
        if(DEBUG){
            std::cout << "Received CAN frame on can1: ID=0x" 
                    << std::hex << frame.can_id 
                    << " DLC=" << std::dec << static_cast<int>(frame.can_dlc) 
                    << " Data=";

            for (int i = 0; i < frame.can_dlc; i++) {
                std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
            }
            std::cout << std::endl;
        }

        // Process the frame
        switch (frame.can_id) {
            case 0x181:
                if(frame.data[0] == 0x30) parseInvSpeed(frame.data);
                break;
            case 0x182:
                switch (frame.data[0]) {
                    case 0x01:
                        parseASStatus(frame.data);
                        break;
                    case 0x04: // Brake pressure
                        parseBrakeHydr(frame.data);
                        break;
                    case 0x05: // Pneumatic pressure
                        parsePneumatic(frame.data);
                        break;
                    default:
                        break;
                }
                break;
            case 0x18B:
                parseRES(frame.data);
                break;
            default:
                break;
        }
    }
}


//--------------------------------------------- CAN 0 -------------------------------------------------------------------

void CanInterface::readCan0()
{   
    struct can_frame frame;
    while (rclcpp::ok()) {
        int nbytes = read(socketCan0, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("can0 read error");
            continue;
        } else if (nbytes == 0) {
             std::cerr << "No data read from can0." << std::endl;
            continue;
        }

        // Debug: Print the received CAN frame
        if(DEBUG){
            std::cout << "Received CAN frame on can0: ID=0x" 
                    << std::hex << frame.can_id 
                    << " DLC=" << std::dec << static_cast<int>(frame.can_dlc) 
                    << " Data=";

            for (int i = 0; i < frame.can_dlc; i++) {
                std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
            }
            std::cout << std::endl;
        }

        // Process the frame
        switch (frame.can_id) {
            case 0x380:
                parseAcc(frame.data);
                break;
            case 0x394:
                parseGPS(frame.data);
                break;
            case 0x392:
                parseGPSVel(frame.data);
                break;
            case 0x384:
                parseEulerAngles(frame.data);
                break;
            case 0x382:
                parseAngularVelocity(frame.data);
                break;
            case 0x187:
                switch (frame.data[0]) {
                    case 0x01:
                        parseSteeringAngle(frame.data);
                        break;
                    default:
                        break;
                }
                break;
            case 0x185:
                switch (frame.data[0]) {
                    case 0x01:
                        parseMission(frame.data);
                        break;
                    default:
                        break;
                }
                break;
            default:
                break;
        }
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