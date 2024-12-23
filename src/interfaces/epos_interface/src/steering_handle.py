#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from common_msgs.msg import Cmd
# from common_msgs.msg import Controls, CarState
from eposhandle import EPOSHandle
from std_msgs.msg import Float32MultiArray
import math

class SteeringHandle(Node):
    def __init__(self):
        super().__init__('epos_interface')
        
        self.declare_parameter('MAX_ACCELERATION', 6000)
        self.declare_parameter('MAX_DECELERATION', 6000)
        self.declare_parameter('PROFILE_VELOCITY', 6000)
        
        MAX_ACCELERATION = self.get_parameter('MAX_ACCELERATION').get_parameter_value().integer_value
        MAX_DECELERATION = self.get_parameter('MAX_DECELERATION').get_parameter_value().integer_value
        PROFILE_VELOCITY = self.get_parameter('PROFILE_VELOCITY').get_parameter_value().integer_value

        self.epos = EPOSHandle(MAX_ACCELERATION, MAX_DECELERATION, PROFILE_VELOCITY)
        self.epos.connect_to_device()
        self.epos.enable()

        self._is_shutdown = False

        self.create_subscription(Cmd, '/controller/cmd', self.command_callback, 1)
        self.info_pub = self.create_publisher(Float32MultiArray, '/epos_interface/epos_info', 10)

    def command_callback(self, msg: Float32):
        angle = msg.delta*180/math.pi
        assert angle<=20 and angle>=-20, "Angle out of range"
        if not self._is_shutdown:
            self.epos.move_to(angle)

        epos_info = self.epos.get_epos_info()
        msg = Float32MultiArray()
        for i in epos_info:
            msg.data.append(i)
        self.info_pub.publish(msg)

    def clean_and_close(self):
        self._is_shutdown = True
        self.epos.disable()
        self.epos.disconnect_device()

def main(args=None):
    rclpy.init(args=args)
    steering_handle = SteeringHandle()
    rclpy.spin(steering_handle)
    steering_handle.clean_and_close()
    steering_handle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
