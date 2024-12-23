/**
 * @file VSC_funtions.hpp
 * 
 * @author Francis Rojas (frarojram@gmail.com)
 * 
 * @brief VSC interface, epos Handle for ARUS Team Driverless pipeline
 * 
 * @date 22-12-2024
 */
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <bitset>
#include <stdint.h>

class VSC
{
public:
    const char* EPOS_LIB_PATH = "libEposCmd.so";
    int NodeID = 4;

    VSC(){
        max_acc_ = 0;
        max_dec_ = 0;
        prof_vel_ = 0;

        _is_enable_ = false;
        _is_centered_ = false;
        _is_connected_ = false;

        epos_lib_ = dlopen(EPOS_LIB_PATH, RTLD_LAZY);
        
        loadFunctionVSC(VCS_OpenDevice,"VCS_OpenDevice");
        loadFunctionVSC(VCS_ActivateProfilePositionMode,"VCS_ActivateProfilePositionMode");
        loadFunctionVSC(VCS_SetPositionProfile,"VCS_SetPositionProfile");
        loadFunctionVSC(VCS_CloseDevice,"VCS_CloseDevice");
        loadFunctionVSC(VCS_SetEnableState,"VCS_SetEnableState");
        loadFunctionVSC(VCS_SetDisableState,"VCS_SetDisableState");
        loadFunctionVSC(VCS_MoveToPosition,"VCS_MoveToPosition");
        loadFunctionVSC(VCS_GetMovementState,"VCS_GetMovementState");
        loadFunctionVSC(VCS_GetPositionIs,"VCS_GetPositionIs");
        loadFunctionVSC(VCS_GetTargetPosition,"VCS_GetTargetPosition");
        loadFunctionVSC(VCS_GetVelocityIs,"VCS_GetVelocityIs");
        loadFunctionVSC(VCS_GetVelocityIsAveraged,"VCS_GetVelocityIsAveraged");
        loadFunctionVSC(VCS_GetObject,"VCS_GetObject");
        loadFunctionVSC(VCS_SetObject,"VCS_SetObject");
        
    }

    void set_params(int max_acceleration, int max_deceleration, int prof_velocity){
        max_acc_ = max_acceleration;
        max_dec_ = max_deceleration;
        prof_vel_ = prof_velocity;
    }

    void connect_to_device() {
        uint32_t pErrorCode = 0;
        
        if (!_is_connected_) {

            keyhandle_ = VCS_OpenDevice("EPOS4", "MAXON SERIAL V2", "USB", "USB0", &pErrorCode);
            // Alternative connection: VCS_OpenDevice("EPOS4", "CANopen", "CAN_kvaser_usb 0", "CAN1", &pErrorCode)

            if (keyhandle_ == nullptr) {
                std::cerr << "Error: Failed to connect to EPOS4 device." << std::endl;
                return;
            } else if (pErrorCode != 0) {
                std::cerr << "Connection error: Error code " << pErrorCode << std::endl;
                return;
            }

            _is_connected_ = true;
            std::cout << "Connection established with EPOS4." << std::endl;

            // Activate position mode
            int ret = VCS_ActivateProfilePositionMode(keyhandle_, NodeID, &pErrorCode);
            if (ret == 0) {
                std::cerr << "Error activating position mode: Error code " << pErrorCode << std::endl;
            } else {
                std::cout << "Position mode successfully activated." << std::endl;
            }

            // Configure the position profile
            ret = VCS_SetPositionProfile(keyhandle_, NodeID, prof_vel_, max_acc_, max_dec_, &pErrorCode);
            if (ret == 0) {
                std::cerr << "Error configuring position profile: Error code " << pErrorCode << std::endl;
            } else {
                std::cout << "Position profile successfully configured." << std::endl;
            }
        }
    }

    void disconnect_device() {
        uint32_t pErrorCode = 0;
        if (_is_connected_) {
            int ret = VCS_CloseDevice(keyhandle_, &pErrorCode);

            if (ret == 0) {
                std::cerr << "Error: Failed to disconnect from EPOS4." << std::endl;
            } else if (pErrorCode != 0) {
                std::cerr << "Error: EPOS CloseDevice returned error code " << pErrorCode << std::endl;
            } else {
                _is_connected_ = false;
                std::cout << "EPOS4 successfully disconnected." << std::endl;
            }
        }
    }

    void enable() {
        uint32_t pErrorCode = 0;

        if (!_is_connected_) {
            std::cerr << "Error: EPOS4 not connected. Cannot enable." << std::endl;
        } else if (!_is_enable_) {
            int return_code = VCS_SetEnableState(keyhandle_, NodeID, &pErrorCode);
            if (return_code == 0) {
                std::cerr << "Error: Couldn't enable EPOS4 with error code " << pErrorCode << std::endl;
            } else {
                std::cout << "EPOS4 enabled." << std::endl;
                _is_enable_ = true;
            }
        } else {
            std::cerr << "Warning: EPOS4 already enabled." << std::endl;
        }
    }

    void disable() {
        uint32_t pErrorCode = 0;

        if (!_is_connected_) {
            
            std::cerr << "Error: EPOS4 not connected. Cannot disable." << std::endl;
        } else if (_is_enable_) {
            
            int return_code = VCS_SetDisableState(keyhandle_, NodeID, &pErrorCode);
            if (return_code == 0) {
                
                std::cerr << "Error: Couldn't disable EPOS4 with error code " << pErrorCode << std::endl;
            } else {
                
                std::cout << "EPOS4 disabled." << std::endl;
                _is_enable_ = false;
            }
        } else {
            
            std::cerr << "Warning: EPOS4 already disabled." << std::endl;
        }
    }

    void move_to(double wheel_angle) {
        uint32_t pErrorCode = 0;

        int motor_position = static_cast<int>(2 * wheel_angle * (2048 * 5 * 66 / 360));

        if (_is_enable_) {

            int ret = VCS_MoveToPosition(keyhandle_, NodeID, motor_position, 1, 1, &pErrorCode);

            if (ret == 0) {
               
                std::cerr << "Warning: MoveToPosition error with code " << pErrorCode 
                        << ". Disabling controller." << std::endl;
                disable();
            }
        } else {
            
            std::cerr << "Error: Cannot move to position with disabled controller." << std::endl;
        }
    }

    std::vector<double> get_epos_info() {
        uint32_t pErrorCode = 0;
        uint32_t pMovementState = 0;
        int32_t pPosition = 0;
        int32_t pTargetPosition = 0;
        int32_t pVelocity = 0;
        int32_t pVelocityAvg = 0;
        int16_t pTorque = 0;
        uint32_t pBytesRead = 0;

        std::vector<double> epos_info;

        if (_is_enable_) {
            // Get Movement State
            if (VCS_GetMovementState(keyhandle_, NodeID, &pMovementState, &pErrorCode) == 0) {
                std::cerr << "WARNING: getMovementState error with code " << pErrorCode << ". Disabling controller." << std::endl;
                disable();
            }
            epos_info.push_back(static_cast<double>(pMovementState));

            // Get Position
            if (VCS_GetPositionIs(keyhandle_, NodeID, &pPosition, &pErrorCode) == 0) {
                std::cerr << "WARNING: getPosition error with code " << pErrorCode << ". Disabling controller." << std::endl;
                disable();
            }
            if (std::bitset<32>(pPosition).count() == 34) {
                pPosition -= (1LL << 32);  // Adjust for overflow
            }
            epos_info.push_back(pPosition * 360.0 / (2 * 2048 * 5 * 66));

            // Get Target Position
            if (VCS_GetTargetPosition(keyhandle_, NodeID, &pTargetPosition, &pErrorCode) == 0) {
                std::cerr << "WARNING: getTargetPosition error with code " << pErrorCode << ". Disabling controller." << std::endl;
                disable();
            }
            if (std::bitset<32>(pTargetPosition).count() == 34) {
                pTargetPosition -= (1LL << 32);
            }
            epos_info.push_back(pTargetPosition * 360.0 / (2 * 2048 * 5 * 66));

            // Get Velocity
            if (VCS_GetVelocityIs(keyhandle_, NodeID, &pVelocity, &pErrorCode) == 0) {
                std::cerr << "WARNING: getVelocity error with code " << pErrorCode << ". Disabling controller." << std::endl;
                disable();
            }
            epos_info.push_back(static_cast<double>(pVelocity));

            // Get Averaged Velocity
            if (VCS_GetVelocityIsAveraged(keyhandle_, NodeID, &pVelocityAvg, &pErrorCode) == 0) {
                std::cerr << "WARNING: getVelocityAvg error with code " << pErrorCode << ". Disabling controller." << std::endl;
                disable();
            }
            epos_info.push_back(static_cast<double>(pVelocityAvg));

            // Get Torque
            if (VCS_GetObject(keyhandle_, NodeID, 0x6077, 0x00, &pTorque, 2, &pBytesRead, &pErrorCode) == 0) {
                std::cerr << "WARNING: getTorque error with code " << pErrorCode << ". Disabling controller." << std::endl;
                disable();
            }
            epos_info.push_back(static_cast<double>(pTorque));

            return epos_info;
        } else {
            std::cerr << "ERROR: Cannot get epos info with disabled controller :(" << std::endl;
            return {};
        }
    }

    void set_position_offset(double initial_position) {
        uint32_t pErrorCode = 0;
        float pPositionOffset = static_cast<float>(2 * initial_position * (2048 * 5 * 66 / 360));
        uint32_t pBytesWritten = 0;

        if (_is_enable_) {
            if (VCS_SetObject(keyhandle_, NodeID, 0x60B0, 0x00, &pPositionOffset, sizeof(pPositionOffset), &pBytesWritten, &pErrorCode) == 0) {
                std::cerr << "WARNING: SetPositionOffset error with code " << pErrorCode << ". Disabling controller." << std::endl;
                disable();
            }
        } else {
            std::cerr << "ERROR: Cannot set position offset with disabled controller :(" << std::endl;
        }
    }

/*
    void zero_position_protocol(){

    }
    void set_zero_position(){

    }
*/
    
private:
    //connect_to_device()
    using VCS_OpenDevice_t = void*(*)(const char*, const char*, const char*, const char*, uint32_t*);
    using VCS_ActivateProfilePositionMode_t = int(*)(void*, int, uint32_t*);
    using VCS_SetPositionProfile_t = int(*)(void*, int, int, int, int, uint32_t*);
    //disconnect_device()
    using VCS_CloseDevice_t = int(*)(void*,uint32_t*);
    //enable()
    using VCS_SetEnableState_t = int(*)(void*, int, uint32_t*);
    //disble()
    using VCS_SetDisableState_t = int(*)(void*, int, uint32_t*);
    //move_to()
    using VCS_MoveToPosition_t = int(*)(void*, int, int, int, int, uint32_t*);
    // get_epos_info()
    using VCS_GetMovementState_t = int(*)(void*, int, uint32_t*, uint32_t*);
    using VCS_GetPositionIs_t = int(*)(void*, int, int32_t*, uint32_t*);
    using VCS_GetTargetPosition_t = int(*)(void*, int, int32_t*, uint32_t*);
    using VCS_GetVelocityIs_t = int(*)(void*, int, int32_t*, uint32_t*);
    using VCS_GetVelocityIsAveraged_t = int(*)(void*, int, int32_t*, uint32_t*);
    using VCS_GetObject_t = int(*)(void*, int, uint16_t, uint8_t, void*, uint16_t, uint32_t*, uint32_t*);
    // set_position_offset()
    using VCS_SetObject_t = int(*)(void*, int, uint16_t, uint8_t, void*, uint16_t, uint32_t*, uint32_t*);

    VCS_OpenDevice_t VCS_OpenDevice;
    VCS_ActivateProfilePositionMode_t VCS_ActivateProfilePositionMode;
    VCS_SetPositionProfile_t VCS_SetPositionProfile;
    VCS_CloseDevice_t VCS_CloseDevice;
    VCS_SetEnableState_t VCS_SetEnableState;
    VCS_SetDisableState_t VCS_SetDisableState;
    VCS_MoveToPosition_t VCS_MoveToPosition;
    VCS_GetMovementState_t VCS_GetMovementState;
    VCS_GetPositionIs_t VCS_GetPositionIs;
    VCS_GetTargetPosition_t VCS_GetTargetPosition;
    VCS_GetVelocityIs_t VCS_GetVelocityIs;
    VCS_GetVelocityIsAveraged_t VCS_GetVelocityIsAveraged;
    VCS_GetObject_t VCS_GetObject;
    VCS_SetObject_t VCS_SetObject;

    int max_acc_;
    int max_dec_;
    int prof_vel_;

    bool _is_enable_;
    bool _is_centered_;
    bool _is_connected_;

    void* keyhandle_ = nullptr;
    void* epos_lib_ = nullptr;

    template<typename Func>
    void loadFunctionVSC(Func& func, const char* name) {
        func = reinterpret_cast<Func>(dlsym(epos_lib_, name));
        if (!func) {
            throw std::runtime_error(std::string("Failed to load function: ") + name);
        }
    }
};