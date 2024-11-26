#include "can_interface/can_interface_node.hpp"

int velMax = 5500;
float wheelRadius = 0.2;
float transmissionRatio = 0.24444444444444444;//11/45;


CanInterface::CanInterface() : Node("can_interface"){
   // controlsSub = this->create_subscription<common_msgs::msg::Controls>("/controls", 10, std::bind(&CanInterface::controlsCallback, this, std::placeholders::_1));
    ASStatusSub = this->create_subscription<std_msgs::msg::Int16>("/can/AS_status", 10, std::bind(&CanInterface::ASStatusCallback, this, std::placeholders::_1));
    steeringInfoSub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/steering/epos_info", 10, std::bind(&CanInterface::steeringInfoCallback, this, std::placeholders::_1));
    lapCounterSub = this->create_subscription<std_msgs::msg::Int16>("/lap_counter", 10, std::bind(&CanInterface::lapCounterCallback, this, std::placeholders::_1));
    conesCountSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/perception_map", 10, std::bind(&CanInterface::conesCountCallback, this, std::placeholders::_1));
    conesCountAllSub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/mapa_icp", 10, std::bind(&CanInterface::conesCountAllCallback, this, std::placeholders::_1));
    targetSpeedSub = this->create_subscription<std_msgs::msg::Float32>("/target_speed", 10, std::bind(&CanInterface::targetSpeedCallback, this, std::placeholders::_1));
    brakeLightSub = this->create_subscription<std_msgs::msg::Int16>("/brake_light", 10, std::bind(&CanInterface::brakeLightCallback, this, std::placeholders::_1));
    
    motorSpeedPub = this->create_publisher<std_msgs::msg::Float32>("/motor_speed", 10);
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

    pcTempTimer = this->create_wall_timer(0.1s, std::bind(&CanInterface::pcTempCallback, this));
    heartBeatTimer = this->create_wall_timer(0.1s, std::bind(&CanInterface::pubHeartBeat, this));
    DL500Timer = this->create_wall_timer(0.1s, std::bind(&CanInterface::DL500Callback, this));
    DL501Timer = this->create_wall_timer(0.1s, std::bind(&CanInterface::DL501Callback, this));
    DL502Timer = this->create_wall_timer(0.1s, std::bind(&CanInterface::DL502Callback, this));

    std::thread thread_0(&CanInterface::readCan0, this);
    std::thread thread_1(&CanInterface::readCan1, this);

    // Configure socketCan0
    socketCan0 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketCan0 < 0) {
        perror("Error while opening can0 socket");
        return;
    } else{
        printf("can0 enabled for writing \n");
    }
    strcpy(ifr.ifr_name, "can0");
    ioctl(socketCan0, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketCan0, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Error in binding socketCan0");
            return;
        }

    // Configure socketCan1
    socketCan1 = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socketCan1 < 0) {
        perror("Error while opening can1 socket");
        return;
    } else{
        printf("can1 enabled for writing \n");
    }
    strcpy(ifr.ifr_name, "can1");
    ioctl(socketCan1, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socketCan1, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            perror("Error in binding socketCan1");
            exit(EXIT_FAILURE);
        }

    // ros::waitForShutdown();

    thread_0.join();
    thread_1.join();

    this->brake_hydr_actual = 100;
    this->brake_hydr_target = 100;
    this->service_brake_state = 0;
    this->cones_count_all = 0;
    this->EBS_state = 0;
    this->motor_moment_actual = 0;
}

//void CanInterface::check_can(canStatus stat)
//{
//    if(stat != canOK){
//        char buf[50];
//        buf[0] = '\0';
//        canGetErrorText(stat, buf, sizeof(buf));
//        printf("failed, stat=%d (%s) \n", (int)stat, buf);
//    }
//}

//################################################# PARSE FUNCTIONS #################################################

//--------------------------------------------- INV SPEED -------------------------------------------------------------
void CanInterface::parseInvSpeed(uint8_t msg[8])
{      
    int16_t val = (msg[2] << 8) | msg[1];
    float angV = val / pow(2, 15) * velMax;
    float invSpeed = -angV * 2 * M_PI * wheelRadius * transmissionRatio / 60;
    std_msgs::msg::Float32 x;
    x.data = invSpeed;
    this->motorSpeedPub->publish(x);
    this->actual_speed = invSpeed*3.6;
}

//-------------------------------------------- AS -------------------------------------------------------------------------
//1111
//133
void CanInterface::parseASStatus(uint8_t msg[8])
{
    int16_t val = (msg[2]);

    if(val == 0x02)
    {
        this->brake_hydr_actual = 0;
        this->brake_hydr_target = 0;
    }else{
        this->brake_hydr_actual = 100;
        this->brake_hydr_target = 100;
    }

    this->AS_state = val+1;

    std_msgs::msg::Int16 x;
    x.data = val;
    this->ASStatusPub->publish(x);
}

void CanInterface::parseBrakeHydr(uint8_t msg[8])
{
    uint16_t b = (msg[2]<<8) | msg[1];
    this -> brake_hydr_actual = ((b-1111)/(133-1111))*100;
}

void CanInterface::parsePneumatic(uint8_t msg[8])
{
    uint16_t p1 = (msg[2]<<8) | msg[1];
    uint16_t p2 = (msg[4]<<8) | msg[3];
    std::cout << "pneumatic pressure: " << p1 << " " << p2 << std::endl;
}

//-------------------------------------------- IMU -----------------------------------------------------------------------
void CanInterface::parseAcc(uint8_t msg[8])
{
    int16_t intX = (msg[1] << 8) | msg[0];
    float accX = intX*0.01;

    int16_t intY = (msg[3]  << 8) | msg[2];
    float accY = intY*0.01;

    int16_t intZ = (msg[5] << 8) | msg[4];
    float accZ = intZ*0.01;


    IMUData.linear_acceleration.x = accX;
    IMUData.linear_acceleration.y = accY;
    IMUData.linear_acceleration.z = accZ;
    IMUData.header.stamp = this->get_clock()->now();

    this->IMUPub->publish(IMUData);  
}

void CanInterface::parseEulerAngles(uint8_t msg[8])
{
    int16_t intRoll = (msg[1] << 8) | msg[0];
    float roll = intRoll*0.0001;

    int16_t intPitch = (msg[3] << 8) | msg[2];
    float pitch = intPitch*0.0001;

    int16_t intYaw = (msg[5] << 8) | msg[4];
    float yaw = intYaw*0.0001;

    float qx = sin(roll*0.5) * cos(pitch/2) * cos(yaw*0.5) - cos(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5);
    float qy = cos(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5) + sin(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5);
    float qz = cos(roll*0.5) * cos(pitch*0.5) * sin(yaw*0.5) - sin(roll*0.5) * sin(pitch*0.5) * cos(yaw*0.5);
    float qw = cos(roll*0.5) * cos(pitch*0.5) * cos(yaw*0.5) + sin(roll*0.5) * sin(pitch*0.5) * sin(yaw*0.5);

    IMUData.orientation.x = qx;
    IMUData.orientation.y = qy;
    IMUData.orientation.z = qz;
    IMUData.orientation.w = qw;
    IMUData.header.stamp = this->get_clock()->now();
}

void CanInterface::parseAngularVelocity(uint8_t msg[8])
{
    int16_t intX = (msg[1] << 8) | msg[0];
    float angVelX = intX*0.001;

    int16_t intY = (msg[3] << 8) | msg[2];
    float angVelY = intY*0.001;

    int16_t intZ = (msg[5] << 8) | msg[4];
    float angVelZ = intZ*0.001;

    IMUData.angular_velocity.x = angVelX;
    IMUData.angular_velocity.y = angVelY;
    IMUData.angular_velocity.z = angVelZ;
    IMUData.header.stamp = this->get_clock()->now(); 
}

void CanInterface::parseGPS(uint8_t msg[8])
{
    int32_t lat = (msg[3] << 24) | (msg[2] << 16) | (msg[1] << 8) | msg[0];
    int32_t lon = (msg[7] << 24) | (msg[6] << 16) | (msg[5] << 8) | msg[4];

    sensor_msgs::msg::NavSatFix x;
    x.latitude = lat;
    x.longitude = lon;
    this->GPSPub->publish(x);
}

void CanInterface::parseGPSVel(uint8_t msg[8])
{
    int16_t inxN = (msg[1] << 8) | msg[0];
    float velN = inxN/100;

    int16_t inxE = (msg[3] << 8) | msg[2];
    float velE = inxE/100;

    int16_t inxD = (msg[5] << 8) | msg[4];
    float velD = inxD/100;

    geometry_msgs::msg::Vector3 x;
    x.x = velN;
    x.y = velE;
    x.z = velD;
    this->GPSSpeedPub->publish(x);
}

//-------------------------------------------------------- ACQUISITION -------------------------------------------------
void CanInterface::parseSteeringAngle(uint8_t msg[8])
{
    int16_t val = (msg[2] << 8) | msg[1];

    this->actual_steering_angle = val * 2;

    std_msgs::msg::Float32 x;
    x.data = val;
    this->steeringAnglePub->publish(x);
}


//---------------------------------------------RES---------------------------------------------------------------
void CanInterface::parseRES(uint8_t msg[8])
{
    uint8_t val = msg[6];
    std_msgs::msg::Float32 x;
    x.data = val;
    this->RESRangePub->publish(x);
}

//---------------------------------------------DASHBOARD---------------------------------------------------------------
void CanInterface::parseMission(uint8_t msg[8])
{
    uint8_t val = msg[1];

    switch (val)
    {
    case 0x01:
        this->AMI_state = 1;
        break;
    case 0x02:
        this->AMI_state = 2;
        break;
    case 0x03:
        this->AMI_state = 6;
        break;
    case 0x04:
        this->AMI_state = 3;
        break;
    case 0x05:
        this->AMI_state = 4;
        break;
    case 0x06:
        this->AMI_state = 5;
        break;
    default:
        break;
    }
}

//################################################# READ FUNCTIONS #################################################

//--------------------------------------------- CAN 0 -------------------------------------------------------------------   

void CanInterface::readCan0()
{   
    struct can_frame frame;
    std::cout << "readCan0 is called" << std::endl;
    while (true) {
        int nbytes = read(socketCan0, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("can0 read error");
            continue;
        }

        // Debug: Print the received CAN frame
        std::cout << "Received CAN frame on can0: ID=0x" 
                  << std::hex << frame.can_id 
                  << " DLC=" << std::dec << static_cast<int>(frame.can_dlc) 
                  << " Data=";

        for (int i = 0; i < frame.can_dlc; i++) {
            std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << std::endl;

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
                std::cerr << "Unhandled can0 ID: 0x" << std::hex << frame.can_id << std::endl;
                break;
        }
    }
}


//--------------------------------------------- CAN 1 -------------------------------------------------------------------

void CanInterface::readCan1()
{   
    struct can_frame frame;
    std::cout << "readCan1 is called" << std::endl;
    while (true) {
        int nbytes = read(socketCan1, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("can1 read error");
            continue;
        } else if (nbytes == 0) {
             std::cerr << "No data read from can1." << std::endl;
            continue;
        }

        // Debug: Print the received CAN frame
        std::cout << "Received CAN frame on can1: ID=0x" 
                  << std::hex << frame.can_id 
                  << " DLC=" << std::dec << static_cast<int>(frame.can_dlc) 
                  << " Data=";

        for (int i = 0; i < frame.can_dlc; i++) {
            std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << std::endl;

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
                std::cerr << "Unhandled can1 ID: 0x" << std::hex << frame.can_id << std::endl;
                break;
        }
    }
}

//################################################# CALLBACKS ###########################################################
void intToBytes(int16_t val, int8_t* bytes)
{
    std::memcpy(bytes, &val, sizeof(val));
}           

// void CanInterface::controlsCallback(common_msgs::Controls msg)
// {
//     // std::cout << "llega" << std::endl;
//     float acc = msg.accelerator;
//     int16_t intValue = static_cast<int16_t>(acc * (1<<15))-1;
//     this->motor_moment_target = intValue;

//     int8_t bytesCMD[2];
//     intToBytes(intValue, bytesCMD);
//     int8_t cabecera = 0x90;

//     int8_t data[3] = {cabecera, bytesCMD[0], bytesCMD[1]};

//     canWrite(hndW0, 0x201, data, 3, canMSG_STD);
// }

void CanInterface::ASStatusCallback(std_msgs::msg::Int16 msg)
{
    if(msg.data == 3){
        struct can_frame frame;
        frame.can_id = 0x202;             
        frame.can_dlc = 3;                
        frame.data[0] = 0x01;
        frame.data[1] = 0x01;
        frame.data[2] = 0x03;

        write(socketCan0, &frame, sizeof(struct can_frame));           
    }else if(msg.data==4){
        struct can_frame frame;
        frame.can_id = 0x202;             
        frame.can_dlc = 3;                
        frame.data[0] = 0x01;
        frame.data[1] = 0x01;
        frame.data[2] = 0x04;

        write(socketCan0, &frame, sizeof(struct can_frame));   
    }
}

void CanInterface::steeringInfoCallback(std_msgs::msg::Float32MultiArray msg)
{
    int8_t pMovementState = msg.data[0];
    this->steering_state = pMovementState;

    int16_t pPosition = msg.data[1]*2;
    int8_t pPositionBytes[3];
    this->actual_steering_angle = pPosition;
    intToBytes(pPosition, pPositionBytes);

    int16_t pTargetPosition = msg.data[2]*2;
    int8_t pTargetPositionBytes[3];
    this->target_steering_angle = pTargetPosition;
    intToBytes(pTargetPosition, pTargetPositionBytes);

    struct can_frame frame;
    frame.can_id = 0x183;             
    frame.can_dlc = 8;                
    frame.data[0] = 0x02;
    frame.data[1] = pMovementState;
    frame.data[2] = pPositionBytes[0];
    frame.data[3] = pPositionBytes[1];
    frame.data[4] = pPositionBytes[2];
    frame.data[5] = pTargetPositionBytes[0];
    frame.data[6] = pTargetPositionBytes[1];
    frame.data[7] = pTargetPositionBytes[2];

    write(socketCan1, &frame, sizeof(struct can_frame));

    int16_t pVelocity = msg.data[3]*100;
    int8_t pVelocityBytes[3];
    intToBytes(pVelocity, pVelocityBytes);

    int16_t pVelocityAvg = msg.data[4]*100;
    int8_t pVelocityAvgBytes[3];
    intToBytes(pVelocityAvg, pVelocityAvgBytes);

    frame.can_id = 0x183;             
    frame.can_dlc = 7;                
    frame.data[0] = 0x03;
    frame.data[1] = pVelocityBytes[0];
    frame.data[2] = pVelocityBytes[1];
    frame.data[3] = pVelocityBytes[2];
    frame.data[4] = pVelocityAvgBytes[0];
    frame.data[5] = pVelocityAvgBytes[1];
    frame.data[6] = pVelocityAvgBytes[2];

    write(socketCan1, &frame, sizeof(struct can_frame));

    int16_t pTorque = msg.data[5]*100;
    int8_t pTorqueBytes[2];
    intToBytes(pTorque, pTorqueBytes);

    frame.can_id = 0x183;             
    frame.can_dlc = 3;                
    frame.data[0] = 0x04;
    frame.data[1] = pTorqueBytes[0];
    frame.data[2] = pTorqueBytes[1];

    write(socketCan1, &frame, sizeof(struct can_frame));    
}

void CanInterface::pubHeartBeat()
{
    struct can_frame frame;
    frame.can_id = 0x183;             
    frame.can_dlc = 1;                
    frame.data[0] = 0x00;

    write(socketCan1, &frame, sizeof(struct can_frame));
}

void CanInterface::lapCounterCallback(std_msgs::msg::Int16 msg)
{
    this->lap_counter = msg.data;
}

void CanInterface::conesCountCallback(sensor_msgs::msg::PointCloud2 msg)
{
    this->cones_count_actual = msg.width;
}

void CanInterface::conesCountAllCallback(sensor_msgs::msg::PointCloud2 msg)
{   
    if(this->cones_count_all<msg.width && msg.width < 500){
        this->cones_count_all = msg.width;
    }
}

void CanInterface::DL500Callback()
{
    std_msgs::msg::Float32MultiArray x;

    x.data.push_back(motor_moment_target);
    x.data.push_back(this->motor_moment_actual);
    x.data.push_back(this->brake_hydr_target);
    x.data.push_back(this->brake_hydr_actual);
    x.data.push_back(this->target_steering_angle);
    x.data.push_back(this->actual_steering_angle);
    x.data.push_back(this->target_speed);
    x.data.push_back(this->actual_speed);

    this->DL500Pub->publish(x);

    struct can_frame frame;
    frame.can_id = 0x500;             
    frame.can_dlc = 8;                
    frame.data[0] = motor_moment_target;
    frame.data[1] = this->motor_moment_actual;
    frame.data[2] = this->brake_hydr_target;
    frame.data[3] = this->brake_hydr_actual;
    frame.data[4] = this->target_steering_angle;
    frame.data[5] = this->actual_steering_angle;
    frame.data[6] = this->target_speed;
    frame.data[7] = this->actual_speed;

    write(socketCan0, &frame, sizeof(struct can_frame));
}

void CanInterface::DL501Callback()
{
    std_msgs::msg::Float32MultiArray x;

    int16_t long_acc = IMUData.linear_acceleration.x*512;
    int8_t long_acc_bytes[2];
    intToBytes(long_acc, long_acc_bytes);
    int8_t long_acc_bytes_le[2] = {long_acc_bytes[1], long_acc_bytes[0]};
    x.data.push_back(IMUData.linear_acceleration.x);

    int16_t lat_acc = IMUData.linear_acceleration.y*512;
    int8_t lat_acc_bytes[2];
    intToBytes(lat_acc, lat_acc_bytes);
    int8_t lat_acc_bytes_le[2] = {lat_acc_bytes[1], lat_acc_bytes[0]};
    x.data.push_back(IMUData.linear_acceleration.y);

    int16_t yaw_rate = IMUData.angular_velocity.z*(180/M_PI)*128;
    int8_t yaw_rate_bytes[2];
    intToBytes(yaw_rate, yaw_rate_bytes);
    int8_t yaw_rate_bytes_le[2] = {yaw_rate_bytes[1], yaw_rate_bytes[0]};
    x.data.push_back(IMUData.angular_velocity.z);

    this->DL501Pub->publish(x);

    struct can_frame frame;
    frame.can_id = 0x501;             
    frame.can_dlc = 6;                
    frame.data[0] = yaw_rate_bytes_le[0];
    frame.data[1] = yaw_rate_bytes_le[1];
    frame.data[2] = lat_acc_bytes_le[0];
    frame.data[3] = lat_acc_bytes_le[1];
    frame.data[4] = long_acc_bytes_le[0];
    frame.data[5] = long_acc_bytes_le[1];

    write(socketCan0, &frame, sizeof(struct can_frame));
}

void CanInterface::DL502Callback()
{   
    std_msgs::msg::Float32MultiArray x;
    x.data.push_back(this->AMI_state);
    x.data.push_back(this->EBS_state);
    x.data.push_back(this->AS_state);
    x.data.push_back(this->cones_count_actual);
    x.data.push_back(this->lap_counter);
    x.data.push_back(this->service_brake_state);
    x.data.push_back(this->steering_state);
    x.data.push_back(this->cones_count_all);

    struct can_frame frame;
    frame.can_id = 0x502;             
    frame.can_dlc = 5;                
    frame.data[0] = (this->cones_count_all & 0xFE00)>>9;
    frame.data[1] = (this->cones_count_all & 0x01FE)>>1;
    frame.data[2] = (this->cones_count_all & 0x0001)|((this->cones_count_actual & 0xFE)>>1);
    frame.data[3] = ((((((this->cones_count_actual & 0x01)<<4)|this->lap_counter)<<2)|this->service_brake_state)<<1)|steering_state;
    frame.data[4] = (((this->AMI_state <<2) | this->EBS_state)<<3) | this->AS_state;

    write(socketCan0, &frame, sizeof(struct can_frame));

    this->DL502Pub->publish(x);
}

void CanInterface::targetSpeedCallback(std_msgs::msg::Float32 msg)
{
    this->target_speed = msg.data;
}

void CanInterface::pcTempCallback()
{
    this->getPcTemp();
    std_msgs::msg::Float32 x;
    x.data = this->pc_temp;
    this->PCTempPub->publish(x);

    int8_t bytes[2];
    int16_t temp = this->pc_temp*100;
    intToBytes(temp, bytes);

    struct can_frame frame;
    frame.can_id = 0x183;             
    frame.can_dlc = 3;                
    frame.data[0] = 0x01;
    frame.data[1] = bytes[0];
    frame.data[2] = bytes[1];

    write(socketCan1, &frame, sizeof(struct can_frame));
}

void CanInterface::brakeLightCallback(std_msgs::msg::Int16 msg)
{
    struct can_frame frame;
    frame.can_id = 0x208;             
    frame.can_dlc = 2;                
    frame.data[0] = 0x01;
    frame.data[1] = msg.data;

    write(socketCan1, &frame, sizeof(struct can_frame));
}

void CanInterface::getPcTemp()
{
    float temp = 0.0;
    FILE* fp = popen("sensors", "r");
    if (fp == NULL) {
        RCLCPP_ERROR(this->get_logger(), "This is an error message!");
    }

    char path[1035];
    while (fgets(path, sizeof(path), fp) != NULL) {
        std::string line(path);
        if (line.find("Core 0:") != std::string::npos) { // Ajusta esto según tu salida de 'sensors'
            std::istringstream iss(line);
            std::string token;
            while (iss >> token) {
                if (token[0] == '+') {
                    token = token.substr(1);
                    token.pop_back();
                    temp = std::stof(token);
                    break;
                }
            }
            break;
        }
    }
    pclose(fp);
    this->pc_temp = temp;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanInterface>());
  rclcpp::shutdown();
  return 0;
}