/**
 * @file VSC_funtions.hpp
 * @author Francis Rojas (frarojram@gmail.com)
 * @brief VSCfuntions implementation for ARUS Team Driverless pipeline
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
    rclcpp::Logger logger_ = rclcpp::get_logger("VSC");

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


    /**
     * @brief Set the parameters for the EPOS4 controller.
     */
    void set_params(int max_acceleration, int max_deceleration, int prof_velocity){
        max_acc_ = max_acceleration;
        max_dec_ = max_deceleration;
        prof_vel_ = prof_velocity;
    }


    /**
     * @brief Connect to EPOS4 and activate position profile mode.
     */
    void connect_to_device() {
        uint32_t pErrorCode = 0;
        
        if (!_is_connected_) {

            //keyhandle_ = VCS_OpenDevice("EPOS4", "MAXON SERIAL V2", "USB", "USB0", &pErrorCode);
            keyhandle_ = VCS_OpenDevice("EPOS4", "CANopen", "CAN_kvaser_usb 0", "CAN1", &pErrorCode);

            if (keyhandle_ == nullptr) {
                RCLCPP_ERROR(logger_, "Error: Failed to open EPOS4 device.");
                return;
            } else if (pErrorCode != 0) {
                RCLCPP_ERROR(logger_, "Connection error: Error code  %d", pErrorCode);
                return;
            }

            _is_connected_ = true;
            RCLCPP_INFO(logger_, "Connection established with EPOS4.");

            // Activate position mode
            int ret = VCS_ActivateProfilePositionMode(keyhandle_, NodeID, &pErrorCode);
            if (ret == 0) {
                RCLCPP_ERROR(logger_, "Error activating position mode: Error code %d", pErrorCode);
            } else {
                RCLCPP_INFO(logger_, "Position mode successfully activated.");
            }

            // Configure the position profile
            ret = VCS_SetPositionProfile(keyhandle_, NodeID, prof_vel_, max_acc_, max_dec_, &pErrorCode);
            if (ret == 0) {
                RCLCPP_ERROR(logger_, "Error configuring position profile: Error code %d", pErrorCode);
            } else {
                RCLCPP_INFO(logger_, "Position profile successfully configured.");
            }
        }
    }


    /**
     * @brief Disconnect EPOS4 controller.
     */
    void disconnect_device() {
        uint32_t pErrorCode = 0;
        if (_is_connected_) {
            int ret = VCS_CloseDevice(keyhandle_, &pErrorCode);

            if (ret == 0) {
                RCLCPP_ERROR(logger_, "Error: Failed to disconnect from EPOS4 device.");
            } else if (pErrorCode != 0) {
                RCLCPP_ERROR(logger_, "Error: EPOS CloseDevice returned error code %d", pErrorCode);
            } else {
                _is_connected_ = false;
                RCLCPP_INFO(logger_, "EPOS4 successfully disconnected.");
            }
        }
    }


    /**
     * @brief Enable EPOS4 controller.
     */
    void enable() {
        uint32_t pErrorCode = 0;

        if (!_is_connected_) {
            RCLCPP_ERROR(logger_, "Error: EPOS4 not connected. Cannot enable.");
        } else if (!_is_enable_) {
            int return_code = VCS_SetEnableState(keyhandle_, NodeID, &pErrorCode);
            if (return_code == 0) {
                RCLCPP_ERROR(logger_, "Error: Couldn't enable EPOS4 with error code %d", pErrorCode);
            } else {
                RCLCPP_INFO(logger_, "EPOS4 successfully enabled.");
                _is_enable_ = true;
            }
        } else {
            RCLCPP_WARN(logger_, "Warning: EPOS4 already enabled.");
        }
    }


    /**
     * @brief Disable EPOS4 controller.
     */
    void disable() {
        uint32_t pErrorCode = 0;

        if (!_is_connected_) {
            RCLCPP_ERROR(logger_, "Error: EPOS4 not connected. Cannot disable.");
        } else if (_is_enable_) {
            
            int return_code = VCS_SetDisableState(keyhandle_, NodeID, &pErrorCode);
            if (return_code == 0) {
                RCLCPP_ERROR(logger_, "Error: Couldn't disable EPOS4 with error code %d", pErrorCode);
            } else {
                RCLCPP_INFO(logger_, "EPOS4 successfully disabled.");
                _is_enable_ = false;
            }
        } else {
            RCLCPP_WARN(logger_, "Warning: EPOS4 already disabled.");
        }
    }


    /**
     * @brief Move EPOS4 to a specified position. Position is given in radians and transformed to increments.
     */
    void move_to(double wheel_angle) {
        uint32_t pErrorCode = 0;

        // 4096 incs = 2pi rad
        // 5:1 steering ratio
        // 66:1 motor reduction
        int motor_position = static_cast<int>(wheel_angle*(4096 * 5 * 66)/(2*M_PI));

        if (_is_enable_) {

            int ret = VCS_MoveToPosition(keyhandle_, NodeID, motor_position, 1, 1, &pErrorCode);

            if (ret == 0) {
                RCLCPP_ERROR(logger_, "Error: MoveToPosition failed with error code %d. Disabling controller", pErrorCode);
                disable();
            }
        } else {
            RCLCPP_ERROR(logger_, "Error: Cannot move to position with disabled controller.");
        }
    }


    /**
     * @brief Get EPOS4 information such as movement state, position, target position, 
     * velocity, averaged velocity, and torque.
     */
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
                RCLCPP_WARN(logger_, "WARNING: getMovementState error with code %d. Disabling controller.", pErrorCode);
                disable();
            }
            epos_info.push_back(static_cast<double>(pMovementState));

            // Get Position
            if (VCS_GetPositionIs(keyhandle_, NodeID, &pPosition, &pErrorCode) == 0) {
                RCLCPP_WARN(logger_, "WARNING: getPosition error with code %d. Disabling controller.", pErrorCode);
                disable();
            }
            if (std::bitset<32>(pPosition).count() == 34) {
                pPosition -= (1LL << 32);  // Adjust for overflow
            }
            epos_info.push_back((pPosition*2*M_PI)/(4096 * 5 * 66));

            // Get Target Position
            if (VCS_GetTargetPosition(keyhandle_, NodeID, &pTargetPosition, &pErrorCode) == 0) {
                RCLCPP_WARN(logger_, "WARNING: getTargetPosition error with code %d. Disabling controller.", pErrorCode);
                disable();
            }
            if (std::bitset<32>(pTargetPosition).count() == 34) {
                pTargetPosition -= (1LL << 32);
            }
            epos_info.push_back((pTargetPosition*2*M_PI)/(4096 * 5 * 66));

            // Get Velocity
            if (VCS_GetVelocityIs(keyhandle_, NodeID, &pVelocity, &pErrorCode) == 0) {
                RCLCPP_WARN(logger_, "WARNING: getVelocity error with code %d. Disabling controller.", pErrorCode);
                disable();
            }
            epos_info.push_back(static_cast<double>(pVelocity));

            // Get Averaged Velocity
            if (VCS_GetVelocityIsAveraged(keyhandle_, NodeID, &pVelocityAvg, &pErrorCode) == 0) {
                RCLCPP_WARN(logger_, "WARNING: getVelocityAvg error with code %d. Disabling controller.", pErrorCode);
                disable();
            }
            epos_info.push_back(static_cast<double>(pVelocityAvg));

            // Get Torque
            if (VCS_GetObject(keyhandle_, NodeID, 0x6077, 0x00, &pTorque, 2, &pBytesRead, &pErrorCode) == 0) {
                RCLCPP_WARN(logger_, "WARNING: getTorque error with code %d. Disabling controller.", pErrorCode);
                disable();
            }
            epos_info.push_back(static_cast<double>(pTorque));

            return epos_info;
        } else {
            RCLCPP_ERROR(logger_, "ERROR: Cannot get epos info with disabled controller :(");
            return {};
        }
    }

    
    
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

    // EPOS4 parameters
    int max_acc_;
    int max_dec_;
    int prof_vel_;

    // EPOS4 state
    bool _is_enable_;
    bool _is_centered_;
    bool _is_connected_;

    void* keyhandle_ = nullptr;
    void* epos_lib_ = nullptr;


    /**
     * @brief Load a function from the EPOS4 library.
     */
    template<typename Func>
    void loadFunctionVSC(Func& func, const char* name) {
        func = reinterpret_cast<Func>(dlsym(epos_lib_, name));
        if (!func) {
            throw std::runtime_error(std::string("Failed to load function: ") + name);
        }
    }
};