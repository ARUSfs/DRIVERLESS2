#include "can_interface/can_interface_node.hpp"


CanInterface::CanInterface() : Node("can_interface"){
    // Declare and get parameters
    //  Topics
    this->declare_parameter<std::string>("cmd_topic", "/controller/cmd");
    this->declare_parameter<std::string>("car_info_topic", "/car_state/car_info");
    this->declare_parameter<std::string>("run_check_topic", "/car_state/run_check");

    this->get_parameter("cmd_topic", kCmdTopic);
    this->get_parameter("car_info_topic", kCarInfoTopic);
    this->get_parameter("run_check_topic", kRunCheckTopic);

    //  Files Path
    this->declare_parameter<std::string>("csv_main_file", "/can24.csv");
    this->declare_parameter<std::string>("csv_aux_file", "/can_aux24.csv");
    this->declare_parameter<std::string>("killer_script_file", "/killer.sh");

    this->get_parameter("csv_main_file", kCsvMainFile);
    this->get_parameter("csv_aux_file", kCsvAuxFile);
    this->get_parameter("killer_script_file", kKillerScriptFile);

    // Car parameters
    this->declare_parameter<double>("car_mass", 250);
    this->declare_parameter<double>("wheel_radius", 0.225);
    this->declare_parameter<double>("transmission_ratio", 0.24444);
    this->declare_parameter<double>("max_inv_torque", 230);

    this->get_parameter("car_mass", kCarMass);
    this->get_parameter("wheel_radius", kWheelRadius);
    this->get_parameter("transmission_ratio", kTransmissionRatio);
    this->get_parameter("max_inv_torque", kMaxInvTorque);

    //  Debug
    this->declare_parameter<bool>("debug", true);

    this->get_parameter("debug", kDebug);


    // Read the csv files
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("can_interface");

    std::string path_can = package_share_directory + kCsvMainFile;
    csvdata_main_ = read_csv(path_can);

    std::string path_can_aux = package_share_directory + kCsvAuxFile;
    csvdata_aux_ = read_csv(path_can_aux);


    // Create publishers_ from the csv file and store them in a key-vector pair
    for (const auto& [key, vector] : csvdata_main_) {
        std::string topic = vector[6];
        publishers_[key] = this->create_publisher<std_msgs::msg::Float32>(topic, 10);
    }
    if (kDebug) {
        std::stringstream ss;

        ss << "Main CSV Data Loaded:\n";
        for (const auto &par : csvdata_main_) {
            ss << "Key: " << par.first << "\nValues:";
            for (const auto &valor : par.second) {
                ss << " " << valor;
            }
            ss << "\n\n";
        }

        ss << "Aux CSV Data Loaded:\n";
        for (const auto &par : csvdata_aux_) {
            ss << "Key: " << par.first << "\nValues:";
            for (const auto &valor : par.second) {
                ss << " " << valor;
            }
            ss << "\n\n";
        }

        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    // Configure socket_can0_ for the CAN bus 0.
    const char *can_interface0 = "can0"; 

    socket_can0_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_can0_ < 0) {
        if (kDebug) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Error while opening can0 socket: %s",
                strerror(errno)
            );
        }
        return;
    } else{
        RCLCPP_INFO(this->get_logger(), "can0 enabled for writing");
    }

    std::strncpy(ifr_.ifr_name, can_interface0, IFNAMSIZ - 1);
    if (ioctl(socket_can0_, SIOCGIFINDEX, &ifr_) < 0) {
        if (kDebug) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Error getting can0 interface index: %s",
                strerror(errno)
            );
        }
        return;
    }
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (bind(socket_can0_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
        if (kDebug) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Error in binding socket_can0_: %s",
                strerror(errno)
            );
        }
        return;
    }


    // Configure socket_can1_ for the CAN bus 1.
    const char *can_interface1 = "can1"; 

    socket_can1_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_can1_ < 0) {
        if (kDebug) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Error while opening can1 socket: %s",
                strerror(errno)
            );
        }
        return;
    } else{
        if (kDebug) {
            RCLCPP_INFO(this->get_logger(), "can1 enabled for writing");
        }
    }

    std::strncpy(ifr_.ifr_name, can_interface1, IFNAMSIZ - 1);
    if (ioctl(socket_can1_, SIOCGIFINDEX, &ifr_) < 0) {
        if (kDebug) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Error getting can1 interface index: %s",
                strerror(errno)
            );
        }
        return ;
    }
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    if (bind(socket_can1_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0) {
        if (kDebug) {
            RCLCPP_ERROR(
                this->get_logger(),
                "Error in binding socket_can1_: %s",
                strerror(errno)
            );
        }
        return;
    }

    control_sub_ = this->create_subscription<common_msgs::msg::Cmd>(
        kCmdTopic, 1, std::bind(&CanInterface::
            control_callback, this, std::placeholders::_1));

    car_info_sub_ = this->create_subscription<common_msgs::msg::CarInfo>(
        kCarInfoTopic, 1, std::bind(&CanInterface::
            car_info_callback, this, std::placeholders::_1));
    
    run_check_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        kRunCheckTopic, 1, std::bind(&CanInterface::
            run_check_callback, this, std::placeholders::_1));


    heart_beat_timer_ = this->create_wall_timer(0.1s, std::bind(&CanInterface::heart_beat_callback, this));
    dl_timer_ = this->create_wall_timer(0.1s, std::bind(&CanInterface::dl_timer_callback, this));


    std::thread thread_0_(&CanInterface::read_CAN, this, socket_can0_);
    std::thread thread_1_(&CanInterface::read_CAN, this, socket_can1_);

    thread_0_.detach();
    thread_1_.detach();
}

std::map<std::string, std::vector<std::string>> CanInterface::read_csv(const std::string &filepath)
{
    std::map<std::string, std::vector<std::string>> localCsvData;

    std::ifstream file(filepath);
    if (!file.is_open()) {
        if (kDebug) {
            RCLCPP_ERROR(
                this->get_logger(), 
                "Error opening the CSV file: %s", 
                filepath.c_str()
            );
        }
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
        auto aux_iter = csvdata_aux_.find(frame_id);
        if (aux_iter == csvdata_aux_.end()) {
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
                auto main_iter = csvdata_main_.find(dynamic_key);
                if (main_iter == csvdata_main_.end()) {
                    continue;
                }

                // Convert configuration from main_iter->second to CANParseConfig
                const auto &main_vector = main_iter->second;
                CANParseConfig config;
                config.startByte = std::stoi(main_vector[0]);
                config.endByte = std::stoi(main_vector[1]);
                config.isSigned = main_vector[2];
                config.power = std::stoi(main_vector[3]);
                config.scale = std::stof(main_vector[4]);
                config.offset = std::stof(main_vector[5]);
                config.key = dynamic_key;
                    
                // Call parse_msg with the populated CANParseConfig
                parse_msg(frame, config);
            }
        } else {
            continue;
        }
    }
}


void CanInterface::parse_msg(const struct can_frame& frame, const CANParseConfig& config) {
    int32_t rawValue = 0;

    // Calculate the number of bytes
    uint8_t numBytes = config.endByte - config.startByte + 1;

    if (numBytes == 1) { 
        if (config.isSigned == "yes") {
            rawValue = static_cast<int8_t>(frame.data[config.startByte]);
        } else {
            rawValue = static_cast<uint8_t>(frame.data[config.startByte]);
        }
    } 
    else if (numBytes == 2) { 
        if (config.isSigned == "yes") {
            rawValue = static_cast<int16_t>(
            (frame.data[config.endByte] << 8) | frame.data[config.startByte]);
        } else {
            rawValue = static_cast<uint16_t>(
            (frame.data[config.endByte] << 8) | frame.data[config.startByte]);
        }
        
    } 
    else if (numBytes == 3) { 
        if (config.isSigned == "yes") {
            rawValue = static_cast<int32_t>(
            (frame.data[config.startByte + 2] << 16) |
            (frame.data[config.startByte + 1] << 8) |
             frame.data[config.startByte]);

            // Sign-extend 24-bit value to 32-bit
            if (rawValue & 0x00800000) { // Check if the sign bit (23rd bit) is set
            rawValue |= 0xFF000000;  // Extend the sign to the upper bits
            }
        } else {
            rawValue = static_cast<uint32_t>(
            (frame.data[config.startByte + 2] << 16) |
            (frame.data[config.startByte + 1] << 8) |
             frame.data[config.startByte]);
        }
    } 
    else if (numBytes == 4) { 
        if (config.isSigned == "yes") {
            rawValue = static_cast<int32_t>( 
            (frame.data[config.startByte + 3] << 24) |
            (frame.data[config.startByte + 2] << 16) |
            (frame.data[config.startByte + 1] << 8) |
             frame.data[config.startByte]);
        } else {
            rawValue = static_cast<uint32_t>(
            (frame.data[config.startByte + 3] << 24) |
            (frame.data[config.startByte + 2] << 16) |
            (frame.data[config.startByte + 1] << 8) |
             frame.data[config.startByte]);
        }   
    } 
    else if (kDebug) {
        RCLCPP_ERROR(
                this->get_logger(),
                "Unsupported byte range: %d bytes",
                numBytes
        );
    }

    float scaledValue = std::pow(static_cast<float>(rawValue), config.power) * config.scale + config.offset;

    // Find the associated publisher, create the message and publish the scaled value.
    auto pub_iter = publishers_.find(config.key);
    if (pub_iter != publishers_.end()) {
        std_msgs::msg::Float32 msg;
        msg.data = scaledValue;
        pub_iter->second->publish(msg);
    } else if (kDebug) {
        RCLCPP_ERROR(this->get_logger(), 
                    "No matching publisher in publishers_ for key: %s", 
                    config.key.c_str());
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
    if(run_check_){
        float torque_ = msg.acc * kCarMass * kWheelRadius * kTransmissionRatio / kMaxInvTorque;
        this->motor_moment_target_ = torque_;
        
        int16_t intValue = static_cast<int16_t>(torque_ * (1<<15))-1;
        int8_t bytesCMD[2];
        intToBytes(intValue, bytesCMD);

        struct can_frame frame;
        frame.can_id = 0x201;             
        frame.can_dlc = 3;                
        frame.data[0] = 0x90;
        frame.data[1] = bytesCMD[0];
        frame.data[2] = bytesCMD[1];
        write(socket_can1_, &frame, sizeof(struct can_frame));  
    }
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

    if(as_status_ == 5){ // Finished
        struct can_frame frame;
        frame.can_id = 0x202;             
        frame.can_dlc = 3;                
        frame.data[0] = 0x01;
        frame.data[1] = 0x01;
        frame.data[2] = 0x05;

        write(socket_can1_, &frame, sizeof(struct can_frame));  

        // Define the killer command
        std::string package_path = ament_index_cpp::get_package_share_directory("common_meta");
        std::string kill_command = package_path + kKillerScriptFile;
        int ret1 = system(kill_command.c_str());

    }else if(as_status_ == 4){ // Emergency 
        struct can_frame frame;
        frame.can_id = 0x202;             
        frame.can_dlc = 3;                
        frame.data[0] = 0x01;
        frame.data[1] = 0x01;
        frame.data[2] = 0x04;

        write(socket_can1_, &frame, sizeof(struct can_frame));   
        
        // Define the killer command
        std::string package_path = ament_index_cpp::get_package_share_directory("common_meta");
        std::string kill_command = package_path + kKillerScriptFile;
        int ret2 = system(kill_command.c_str());
    }
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

    write(socket_can0_, &frame, sizeof(struct can_frame));
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

    float clamped_steering_angle_actual_ = std::clamp(static_cast<float>((steering_angle_actual_ *57.2958)*2), -128.0f, 127.0f);
    frame.data[2] = static_cast<int8_t>(clamped_steering_angle_actual_);

    float clamped_steering_angle_target_ = std::clamp(static_cast<float>((steering_angle_target_ *57.2958)*2), -128.0f, 127.0f);
    frame.data[3] = static_cast<int8_t>(clamped_steering_angle_target_);

    float clamped_brake_hydr_actual_ = std::clamp(brake_hydr_actual_, 0.0f, 255.0f);
    frame.data[4] = static_cast<uint8_t>(clamped_brake_hydr_actual_);

    float clamped_brake_hydr_target_ = std::clamp(brake_hydr_target_, 0.0f, 255.0f);
    frame.data[5] = static_cast<uint8_t>(clamped_brake_hydr_target_);

    float clamped_motor_moment_actual_ = std::clamp(motor_moment_actual_, -128.0f, 127.0f);
    frame.data[6] = static_cast<int8_t>(clamped_motor_moment_actual_);

    float clamped_motor_moment_target_ = std::clamp(motor_moment_target_, -128.0f, 127.0f);
    frame.data[7] = static_cast<int8_t>(clamped_motor_moment_target_);

    write(socket_can1_, &frame, sizeof(struct can_frame));
}

void CanInterface::send_dl501()
{
    struct can_frame frame;
    frame.can_id = 0x501;             
    frame.can_dlc = 6;      
    
    int16_t clamped_ax_ = static_cast<int16_t>(std::clamp(static_cast<float>(ax_ *512 ), -32768.0f, 32767.0f));
    // Convert to little-endian (break into 2 bytes)
    frame.data[0] = clamped_ax_ & 0xFF;       
    frame.data[1] = (clamped_ax_ >> 8) & 0xFF; 

    int16_t clamped_ay_ = static_cast<int16_t>(std::clamp(static_cast<float>(ay_ *512 ), -32768.0f, 32767.0f));
    // Convert to little-endian (break into 2 bytes)
    frame.data[2] = clamped_ay_ & 0xFF;       
    frame.data[3] = (clamped_ay_ >> 8) & 0xFF; 

    int16_t clamped_yaw_rate_ = static_cast<int16_t>(std::clamp(static_cast<float>(yaw_rate_ *128 ), -32768.0f, 32767.0f));
    // Convert to little-endian (break into 2 bytes)
    frame.data[4] = clamped_yaw_rate_ & 0xFF;    
    frame.data[5] = (clamped_yaw_rate_ >> 8) & 0xFF; 

    write(socket_can1_, &frame, sizeof(struct can_frame));
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

    write(socket_can1_, &frame, sizeof(struct can_frame));
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
