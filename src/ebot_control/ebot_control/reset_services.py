#!/usr/bin/env python3

#############################FOR ARDUINO RESET##########################################

#ros2 service call /usb_relay_sw usb_relay/srv/RelaySw "{relaychannel: 0, relaystate: true}"
# Provide delay for 1 second
#ros2 service call /usb_relay_sw usb_relay/srv/RelaySw "{relaychannel: 0, relaystate: false}"

############################################################################################


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import Float32MultiArray, Float32
import math
from pymoveit2 import MoveIt2
from threading import Thread
from ur_msgs.srv import SetIO
from rclpy.task import Future


class ThemeImplementation(Node):
    def __init__(self):
        super().__init__('DOCKING_NODE')

        ###### Initializing variables used in the code ###### 

        #############################

        ####### Functions to be called only once #######         
        self.servo_trigger()
        self.reset_imu()                                    # Reset IMU data
        self.reset_odom()
        self.Magnet_Off(False)
                                                            # Reset Odom
        #################################################
        ####### Create a main loop where the main logic of the code will be implemented #######
        #######################################################################################

    def reset_odom(self):
        self.get_logger().info('Resetting Odometry. Please wait...')
        self.reset_odom_ebot = self.create_client(Trigger, 'reset_odom')
        while not self.reset_odom_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_odom service not available. Waiting for /reset_odom to become available.')

        self.request_odom_reset = Trigger.Request()
        self.odom_service_resp=self.reset_odom_ebot.call_async(self.request_odom_reset)
        rclpy.spin_until_future_complete(self, self.odom_service_resp)
        if(self.odom_service_resp.result().success== True):
            self.get_logger().info(self.odom_service_resp.result().message)
        else:
            self.get_logger().warn(self.odom_service_resp.result().message)

    def reset_imu(self):
        self.get_logger().info('Resetting IMU. Please wait...')
        self.reset_imu_ebot = self.create_client(Trigger, 'reset_imu')
        while not self.reset_imu_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_imu service not available. Waiting for /reset_imu to become available.')

        request_imu_reset = Trigger.Request()
        self.imu_service_resp=self.reset_imu_ebot.call_async(request_imu_reset)
        rclpy.spin_until_future_complete(self, self.imu_service_resp)
        if(self.imu_service_resp.result().success== True):
            self.get_logger().info(self.imu_service_resp.result().message)
        else:
            self.get_logger().warn(self.imu_service_resp.result().message)
    def servo_trigger(self):
        self.get_logger().info('Triggering servo node. Please wait...')
        self.trigger_servo_arm = self.create_client(Trigger, '/servo_node/start_servo')
        while not self.trigger_servo_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/servo_node/start_servo service not available. Waiting for /servo_node/start_servo to become available.')

        request_servo_trigger = Trigger.Request()
        self.servo_trigger_resp=self.trigger_servo_arm.call_async(request_servo_trigger)
        rclpy.spin_until_future_complete(self, self.servo_trigger_resp)
        if(self.servo_trigger_resp.result().success== True):
            self.get_logger().info('Servo triggered successfully')
        else:
            self.get_logger().warn(self.servo_trigger_resp.result().message)
    def Magnet_Off(self, state):
        gripper_control = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF Tool service not available, waiting again...')
        req         = SetIO.Request()
        req.fun     = 1
        req.pin     = 16
        req.state   = float(state)
        self.get_logger().info('Arm Magnet turned OFF')

        future=gripper_control.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Arm Magnet turned OFF')

         
def main(args=None):
    rclpy.init(args=args)
    theme=ThemeImplementation()
    
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(theme)
    executor.spin()

if __name__ == '__main__':
    main()