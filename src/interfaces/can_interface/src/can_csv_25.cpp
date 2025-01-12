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

bool DEBUG = false;

CanInterface::CanInterface() : Node("can_interface"){
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("can_interface");

    std::string path_can = package_share_directory + "/can24.csv";
    csvdata_main = read_csv(path_can);

    std::string path_can_aux = package_share_directory + "/can_aux24.csv";
    csvdata_aux = read_csv(path_can_aux);

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
        RCLCPP_ERROR(
            this->get_logger(),
            "Error while opening can0 socket: %s",
            strerror(errno)
        );
        return;
    } else{
        RCLCPP_INFO(this->get_logger(), "can0 enabled for writing");
    }

    std::strncpy(ifr.ifr_name, can_interface0, IFNAMSIZ - 1);
    if (ioctl(socketCan0, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error getting can0 interface index: %s",
            strerror(errno)
        );
        return;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socketCan0, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error in binding socketCan0: %s",
            strerror(errno)
        );
        return;
    }


    // Configure socketCan1 for the CAN bus 1.
    const char *can_interface1 = "can1"; 

    socketCan1 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketCan1 < 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error while opening can1 socket: %s",
            strerror(errno)
        );
        return;
    } else{
        RCLCPP_INFO(this->get_logger(), "can1 enabled for writing");
    }

    std::strncpy(ifr.ifr_name, can_interface1, IFNAMSIZ - 1);
    if (ioctl(socketCan1, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error getting can1 interface index: %s",
            strerror(errno)
        );
        return ;
    }
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socketCan1, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Error in binding socketCan1: %s",
            strerror(errno)
        );
        return;
    }

    control_sub_ = this->create_subscription<common_msgs::msg::Cmd>(
        "/controller/cmd", 1, std::bind(&CanInterface::
            control_callback, this, std::placeholders::_1));

    car_info_sub_ = this->create_subscription<common_msgs::msg::CarInfo>(
        "/car_state/car_info", 1, std::bind(&CanInterface::
            car_info_callback, this, std::placeholders::_1));
    
    run_check_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/car_state/run_check", 1, std::bind(&CanInterface::
            run_check_callback, this, std::placeholders::_1));


    heart_beat_timer_ = this->create_wall_timer(0.1s, std::bind(&CanInterface::heart_beat_callback, this));
    dl_timer_ = this->create_wall_timer(0.1s, std::bind(&CanInterface::dl_timer_callback, this));


    std::thread thread_0(&CanInterface::read_CAN, this, socketCan0);
    std::thread thread_1(&CanInterface::read_CAN, this, socketCan1);

    thread_0.detach();
    thread_1.detach();
}

std::map<std::string, std::vector<std::string>> CanInterface::read_csv(const std::string &filepath)
{
    std::map<std::string, std::vector<std::string>> localCsvData;

    std::ifstream file(filepath);
    if (!file.is_open()) {
        RCLCPP_INFO(
        this->get_logger(), 
        "Error opening the CSV file: %s", 
        filepath.c_str()
        );
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

void CanInterface::read_CAN(int socketCan) 
{
    struct can_frame frame;
    while (rclcpp::ok()) {
        int nbytes = read(socketCan, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            RCLCPP_ERROR(
                this->get_logger(),
                "CAN read error (socket: %d): %s",
                socketCan,
                strerror(errno)
            );
            continue;
        }

        // Convert frame ID to string (hex format)
        std::ostringstream id;
        id << "0x" << std::hex << frame.can_id;
        std::string frame_id = id.str();
        
        // Check if frame_id exists in csvdata_aux
        auto aux_iter = csvdata_aux.find(frame_id);
        if (aux_iter == csvdata_aux.end()) {
            if (DEBUG) {
            RCLCPP_INFO(
                this->get_logger(),
                "No matching key in csvdata_aux for key: %s",
                frame_id.c_str()
            );
            }
            continue;
        }

        // Process auxiliary vector
        const auto &aux_vector = aux_iter->second;

        // Check if there is a subID to filter
        if (filter_subID(frame, aux_vector[0])) {
            // Build multiple dynamic keys
            int num_keys = std::stoi(aux_vector[1]);
            for (int i = 1; i <= num_keys; ++i) {
                std::string dynamic_key = frame_id;
                if (num_keys != 1) {
                    dynamic_key += std::to_string(i);
                }

                // Check for matching entry in csvdata_main
                auto main_iter = csvdata_main.find(dynamic_key);
                if (main_iter == csvdata_main.end()) {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "No matching key in csvdata_main for key: %s",
                        dynamic_key.c_str()
                    );
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
                    
                // Call parse_msg with the populated CANParseConfig
                parse_msg(frame, config);
            
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


void CanInterface::parse_msg(const struct can_frame& frame, const CANParseConfig& config) {
    int16_t rawValue = 0;

    // Calculate the number of bytes
    uint8_t numBytes = config.endByte - config.startByte + 1;

    if (numBytes == 1) { 
        rawValue = static_cast<int8_t>(frame.data[config.startByte]);
    } 
    else if (numBytes == 2) { 
        rawValue = static_cast<int16_t>(
            (frame.data[config.endByte] << 8) | frame.data[config.startByte]
        );
    } 
    else if (numBytes == 3) { 
        rawValue = static_cast<int32_t>(
            (frame.data[config.startByte + 2] << 16) |
            (frame.data[config.startByte + 1] << 8) |
             frame.data[config.startByte]
        );

        // Sign-extend 24-bit value to 32-bit
        if (rawValue & 0x00800000) { // Check if the sign bit (23rd bit) is set
            rawValue |= 0xFF000000;  // Extend the sign to the upper bits
        }
    } 
    else if (numBytes == 4) { 
        rawValue = static_cast<int32_t>(
            (frame.data[config.startByte + 3] << 24) |
            (frame.data[config.startByte + 2] << 16) |
            (frame.data[config.startByte + 1] << 8) |
             frame.data[config.startByte]
        );
    } 
    else {
        RCLCPP_ERROR(
                this->get_logger(),
                "Unsupported byte range: %d bytes",
                numBytes
        );
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

bool CanInterface::filter_subID(const struct can_frame& frame, const std::string& aux_vector_subID) {
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

void intToBytes(int16_t val, int8_t* bytes)
{
    std::memcpy(bytes, &val, sizeof(val));
}           

void CanInterface::control_callback(common_msgs::msg::Cmd msg)
{   
    float acc = msg.acc;
    int16_t intValue = static_cast<int16_t>(acc * (1<<15))-1;
    this->motor_moment_target_ = intValue;

    int8_t bytesCMD[2];
    intToBytes(intValue, bytesCMD);
    int8_t cabecera = 0x90;

    struct can_frame frame;
    frame.can_id = 0x201;             
    frame.can_dlc = 3;                
    frame.data[0] = cabecera;
    frame.data[1] = bytesCMD[0];
    frame.data[2] = bytesCMD[1];
    write(socketCan1, &frame, sizeof(struct can_frame));  
}

void CanInterface::car_info_callback(const common_msgs::msg::CarInfo msg)
{
    speed_actual_ = msg.vx;
    speed_target_ = msg.target_speed;
    steering_angle_actual_ = msg.delta;
    steering_angle_target_ = msg.target_delta;
    brake_hydr_actual_ = msg.brake_hydr_pressure;
    brake_hydr_target_ = msg.brake_hydr_pressure;
    motor_moment_actual_ = msg.torque_actual;
    motor_moment_target_ = msg.torque_target;
 
    ax_ = msg.ax;
    ay_ = msg.ay;
    yaw_rate_ = msg.r;

    as_status_ = msg.as_status;
    asb_ebs_state_ = 0;
    ami_state_ = msg.ami;
    steering_state_ = msg.steering_state;
    asb_redundancy_state_ = 0;
    lap_counter_ = msg.lap_count;
    cones_count_actual_ = msg.cones_count_actual;
    cones_count_all_ = msg.cones_count_all;
}

void CanInterface::run_check_callback(const std_msgs::msg::Bool msg)
{
    run_check_ = msg.data;
}

void CanInterface::heart_beat_callback()
{
    struct can_frame frame;
    frame.can_id = 0x140;             
    frame.can_dlc = 1;                
    frame.data[0] = 0x00;

    write(socketCan0, &frame, sizeof(struct can_frame));
}

void CanInterface::dl_timer_callback()
{
    send_dl500();
    send_dl501();
    send_dl502();
}

void CanInterface::send_dl500()
{
    struct can_frame frame;
    frame.can_id = 0x500;             
    frame.can_dlc = 8;
    
    float clamped_speed_actual_ = std::clamp(static_cast<float>(speed_actual_ *3.6), 0.0f, 255.0f);           
    frame.data[0] = static_cast<uint8_t>(clamped_speed_actual_);

    float clamped_speed_target_ = std::clamp(static_cast<float>(speed_target_ *3.6), 0.0f, 255.0f);
    frame.data[1] = static_cast<uint8_t>(clamped_speed_target_);

    float clamped_steering_angle_actual_ = std::clamp(static_cast<float>((steering_angle_actual_ *57.2958)*0.5), -128.0f, 127.0f);
    frame.data[2] = static_cast<int8_t>(clamped_steering_angle_actual_);

    float clamped_steering_angle_target_ = std::clamp(static_cast<float>((steering_angle_target_ *57.2958)*0.5), -128.0f, 127.0f);
    frame.data[3] = static_cast<int8_t>(clamped_steering_angle_target_);

    float clamped_brake_hydr_actual_ = std::clamp(brake_hydr_actual_, 0.0f, 255.0f);
    frame.data[4] = static_cast<uint8_t>(clamped_brake_hydr_actual_);

    float clamped_brake_hydr_target_ = std::clamp(brake_hydr_target_, 0.0f, 255.0f);
    frame.data[5] = static_cast<uint8_t>(clamped_brake_hydr_target_);

    float clamped_motor_moment_actual_ = std::clamp(motor_moment_actual_, -128.0f, 127.0f);
    frame.data[6] = static_cast<int8_t>(clamped_motor_moment_actual_);

    float clamped_motor_moment_target_ = std::clamp(motor_moment_target_, -128.0f, 127.0f);
    frame.data[7] = static_cast<int8_t>(clamped_motor_moment_target_);

    write(socketCan1, &frame, sizeof(struct can_frame));
}

void CanInterface::send_dl501()
{
    struct can_frame frame;
    frame.can_id = 0x501;             
    frame.can_dlc = 6;      
    
    int16_t clamped_ax_ = static_cast<int16_t>(std::clamp(static_cast<float>(ax_ *1/512 ), -32768.0f, 32767.0f));
    // Convert to little-endian (break into 2 bytes)
    frame.data[0] = clamped_ax_ & 0xFF;       
    frame.data[1] = (clamped_ax_ >> 8) & 0xFF; 

    int16_t clamped_ay_ = static_cast<int16_t>(std::clamp(static_cast<float>(ay_ *1/512 ), -32768.0f, 32767.0f));
    // Convert to little-endian (break into 2 bytes)
    frame.data[2] = clamped_ay_ & 0xFF;       
    frame.data[3] = (clamped_ay_ >> 8) & 0xFF; 

    int16_t clamped_yaw_rate_ = static_cast<int16_t>(std::clamp(static_cast<float>(yaw_rate_ *1/512 ), -32768.0f, 32767.0f));
    // Convert to little-endian (break into 2 bytes)
    frame.data[4] = clamped_yaw_rate_ & 0xFF;    
    frame.data[5] = (clamped_yaw_rate_ >> 8) & 0xFF; 

    write(socketCan1, &frame, sizeof(struct can_frame));
}

void CanInterface::send_dl502()
{   
    struct can_frame frame;
    frame.can_id = 0x502;             
    frame.can_dlc = 5;        

    uint64_t packed_message = 0;

    // Pack each field into the correct bit positions
    packed_message |= (as_status_ & 0x07) << 0;             // Bits 0-2: AS_status (3 bits)
    packed_message |= (asb_ebs_state_ & 0x07) << 3;         // Bits 3-4: ASB_EBS_state (3 bits)
    packed_message |= (ami_state_ & 0x07) << 5;             // Bits 5-7: AMI_state (3 bits)
    packed_message |= (steering_state_ & 0x01) << 8;        // Bit 8: Steering state (1 bit)
    packed_message |= (asb_redundancy_state_ & 0x07) << 9;  // Bits 9-10: ASB_Redundancy_state (3 bits)
    packed_message |= (lap_counter_ & 0x0F) << 11;          // Bits 11-14: Lap counter (4 bits)
    packed_message |= (cones_count_actual_ & 0xFF) << 15;   // Bits 15-22: Cones_count_actual (8 bits)
    packed_message |= (cones_count_all_ & 0xFFFF) << 23;    // Bits 23-39: Cones_count_all (16 bits)

    // Extract the packed message into the frame data bytes (Little-Endian)
    frame.data[0] = packed_message & 0xFF;         
    frame.data[1] = (packed_message >> 8) & 0xFF;  
    frame.data[2] = (packed_message >> 16) & 0xFF; 
    frame.data[3] = (packed_message >> 24) & 0xFF; 
    frame.data[4] = (packed_message >> 32) & 0xFF;         

    write(socketCan1, &frame, sizeof(struct can_frame));
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