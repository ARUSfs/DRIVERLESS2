import threading
import time
from ctypes import *

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os


class EPOSHandle(Node):
    share_directory = get_package_share_directory('epos_interface')
    EPOS_LIB_PATH = os.path.join(share_directory,'include/epos_interface/lib/libEposCmd.so.6.8.1.0')
    
    NodeID = 4

    def __init__(self, max_acc, max_dec, prof_vel):
        super().__init__('epos_handle')
        self.max_acc = max_acc
        self.max_dec = max_dec
        self.prof_vel = prof_vel
        cdll.LoadLibrary(self.EPOS_LIB_PATH)
        self.epos = CDLL(self.EPOS_LIB_PATH)

        self._is_centered = False
        self._is_enabled = False
        self._is_connected = False

    def connect_to_device(self):
        pErrorCode = c_uint()
        if not self._is_connected:
            self.keyhandle = self.epos.VCS_OpenDevice(b'EPOS4', b'CANopen', b'CAN_kvaser_usb 0',b'CAN0', byref(pErrorCode))
            if self.keyhandle == 0:
                self.get_logger().error('Failed to connect to EPOS4. Is it on and connected?')
            elif pErrorCode.value != 0:
                self.get_logger().error(f'EPOS OpenDevice returned error code {pErrorCode.value}')
            else:
                self._is_connected = True
                self.get_logger().info('EPOS4 Successfully connected')

            ret = self.epos.VCS_ActivateProfilePositionMode(self.keyhandle, self.NodeID, byref(pErrorCode))

            if ret == 0:
                self.get_logger().error(f'EPOS ActivatePositionMode returned error code {pErrorCode.value}')
            else:
                self.get_logger().info('Successfully activated position mode')

            ret = self.epos.VCS_SetPositionProfile(self.keyhandle, self.NodeID, self.prof_vel, self.max_acc, self.max_dec, byref(pErrorCode))

            if ret == 0:
                self.get_logger().error(f'EPOS SetPositionProfile returned error code {pErrorCode.value}')
            else:
                self.get_logger().info('Successfully set position profile')

    def disconnect_device(self):
        pErrorCode = c_uint()
        if self._is_connected:
            ret = self.epos.VCS_CloseDevice(self.keyhandle, byref(pErrorCode))

            if ret == 0:
                self.get_logger().error('Failed to disconnect from EPOS4.')
            elif pErrorCode.value != 0:
                self.get_logger().error(f'EPOS CloseDevice returned error code {pErrorCode.value}')
            else:
                self._is_connected = False
                self.get_logger().info('EPOS4 Successfully disconnected')

    def enable(self):
        pErrorCode = c_uint()
        if not self._is_connected:
            self.get_logger().error('EPOS4 not connected. Cannot enable')
        elif not self._is_enabled:
            return_code = self.epos.VCS_SetEnableState(self.keyhandle, self.NodeID, byref(pErrorCode))
            if return_code == 0:
                self.get_logger().error(f'Couldn\'t enable EPOS4 with error code {pErrorCode.value}')
            else:
                self.get_logger().info('EPOS4 enabled')
                self._is_enabled = True
        else:
            self.get_logger().warn('EPOS already enabled')

    def disable(self):
        pErrorCode = c_uint()
        if not self._is_connected:
            self.get_logger().error('EPOS4 not connected. Cannot disable')
        elif self._is_enabled:
            return_code = self.epos.VCS_SetDisableState(self.keyhandle, self.NodeID, byref(pErrorCode))
            if return_code == 0:
                self.get_logger().error(f'Couldn\'t disable EPOS4 with error code {pErrorCode.value}')
            else:
                self.get_logger().info('EPOS4 disabled')
                self._is_enabled = False
        else:
            self.get_logger().warn('EPOS already disabled')

    def move_to(self, wheel_angle):
        pErrorCode = c_uint()
        motor_position = int(2 * wheel_angle * (2048 * 5 * 66 / 360))
        if self._is_enabled:
            ret = self.epos.VCS_MoveToPosition(self.keyhandle, self.NodeID, motor_position, 1, 1, byref(pErrorCode))

            if ret == 0:
                self.get_logger().warn(f'MoveToPosition error with code {pErrorCode.value}\nDisabling controller.')
                self.disable()
        else:
            self.get_logger().error('Cannot move to position with disabled controller :(')

    def get_epos_info(self):
        pErrorCode = c_uint()
        pMovementState = c_uint()
        pPosition = c_long()
        pTargetPosition = c_long()
        pVelocity = c_long()
        pVelocityAvg = c_long()
        pTorque = c_int16()
        pBytesRead = c_uint()

        epos_info = []

        if self._is_enabled:
            ret = self.epos.VCS_GetMovementState(self.keyhandle, self.NodeID, byref(pMovementState), byref(pErrorCode))
            if ret == 0:
                self.get_logger().warn(f'getMovementState error with code {pErrorCode.value}\nDisabling controller.')
                self.disable()
            epos_info.append(pMovementState.value)

            ret = self.epos.VCS_GetPositionIs(self.keyhandle, self.NodeID, byref(pPosition), byref(pErrorCode))
            if ret == 0:
                self.get_logger().warn(f'getPosition error with code {pErrorCode.value}\nDisabling controller.')
                self.disable()
            if len(bin(pPosition.value)) == 34:
                pPosition.value = pPosition.value - 2 ** 32
            epos_info.append(pPosition.value * 360 / (2 * 2048 * 5 * 66))

            ret = self.epos.VCS_GetTargetPosition(self.keyhandle, self.NodeID, byref(pTargetPosition), byref(pErrorCode))
            if ret == 0:
                self.get_logger().warn(f'getTargetPosition error with code {pErrorCode.value}\nDisabling controller.')
                self.disable()
            if len(bin(pTargetPosition.value)) == 34:
                pTargetPosition.value = pTargetPosition.value - 2 ** 32
            epos_info.append(pTargetPosition.value * 360 / (2 * 2048 * 5 * 66))

            ret = self.epos.VCS_GetVelocityIs(self.keyhandle, self.NodeID, byref(pVelocity), byref(pErrorCode))
            if ret == 0:
                self.get_logger().warn(f'getVelocity error with code {pErrorCode.value}\nDisabling controller.')
                self.disable()
            if len(bin(pVelocity.value)) == 34:
                pVelocity.value = pVelocity.value - 2 ** 32
            epos_info.append(pVelocity.value)

            ret = self.epos.VCS_GetVelocityIsAveraged(self.keyhandle, self.NodeID, byref(pVelocityAvg), byref(pErrorCode))
            if ret == 0:
                self.get_logger().warn(f'getVelocityAvg error with code {pErrorCode.value}\nDisabling controller.')
                self.disable()
            if len(bin(pVelocityAvg.value)) == 34:
                pVelocityAvg.value = pVelocityAvg.value - 2 ** 32
            epos_info.append(pVelocityAvg.value)

            ret = self.epos.VCS_GetObject(self.keyhandle, self.NodeID, 0x6077, 0x00, byref(pTorque), 2, byref(pBytesRead), byref(pErrorCode))
            if ret == 0:
                self.get_logger().warn(f'getTorque error with code {pErrorCode.value}\nDisabling controller.')
                self.disable()
            epos_info.append(pTorque.value)

            return epos_info
        else:
            self.get_logger().error('Cannot get epos info with disabled controller :(')

    def set_position_offset(self, initial_position):
        pErrorCode = c_uint()
        pPositionOffset = c_float()
        pBytesWritten = c_uint()
        pPositionOffset.value = int(2 * initial_position * (2048 * 5 * 66 / 360))

        if self._is_enabled:
            ret = self.epos.VCS_SetObject(self.keyhandle, self.NodeID, 0x60B0, 0x00, byref(pPositionOffset), 4, byref(pBytesWritten), byref(pErrorCode))

            if ret == 0:
                self.get_logger().warn(f'SetPositionOffset error with code {pErrorCode.value}\nDisabling controller.')
                self.disable()
        else:
            self.get_logger().error('Cannot set position offset with disabled controller :(')

    def zero_position_protocol(self):
        pass

    def set_zero_position(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    epos_handle = EPOSHandle(max_acc=1000, max_dec=1000, prof_vel=1000)
    rclpy.spin(epos_handle)
    epos_handle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
