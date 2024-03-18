#!/usr/bin/env python3

###########################################################

# Team ID:          CL#1374
# Author List:		Rudranil Bose  , Arnab Mondal , Saptarshi Pal
# Filename:		    reset_services.py 
# Functions:        [reset_odom() , reset_imu() , servo trigger() , Magnet_Off() , call_ebot_attacher()]
# Nodes:		    [RESETTING_NODE]
#			        Services Called: [/io_and_status_controller/set_io , /reset_odom , /reset_imu,
#                                     /servo_node/start_servo , /usb_relay_sw ]


################### IMPORT MODULES #######################


import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from ur_msgs.srv import SetIO
from usb_relay.srv import RelaySw
from rclpy.task import Future


class ThemeImplementation(Node):
    '''
    ___CLASS___

    Description:    Class which servers purpose to reset all the necessary things before starting the run.
    
    '''
    def __init__(self):
        '''
        Description:    Initialization of class ThemeImplementation
        '''
        super().__init__('RESETTING_NODE')                  #registering node
        
        ############################ Constructor VARIABLES #########################
        
        self.servo_trigger()                                # Triger the servo node
        self.reset_imu()                                    # Reset IMU data
        self.reset_odom()                                   # Reset the Odom Data
        self.Magnet_Off(False)                              # Reset the arm magnet
        self.call_ebot_Attacher(1,0)                        # turning off the ebot magnet

    def reset_odom(self):
        '''
        Description:    Callback function for resetting the Odom. 

        Args:

        Returns:
        '''        
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
        '''
        Description:    Callback function for resetting the IMU. 

        Args:

        Returns:
        '''  
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
        '''
        Description:    Callback function for triggering the servo. 

        Args:

        Returns:
        '''  
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
        '''
        Description:    Callback function for turning the arm magnet off. 

        Args:
            state       (Int)       : Stores the state 0 to the service in order to turn the magnet off

        Returns:
        '''  
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
        
    def call_ebot_Attacher(self, relaychannel, relaystate):
        '''
        Description:    Callback function for turing the ebot magnet off. 

        Args:
        
            relaychannel       (Int)          : Stores the relay channel number you want to reset
            relaystate         (Int)          : Stores the state in which you wanna change the relay

        Returns:
        '''  
        ebot_attacher = self.create_client(RelaySw, '/usb_relay_sw')

        while not ebot_attacher.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Relay service not available, waiting again...')

        req = RelaySw.Request()
        req.relaychannel = bool(relaychannel)
        req.relaystate  = bool(relaystate)
        # Calling the service asynchronously
        future = ebot_attacher.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('EBOT Magnet turned OFF')
        
def main(args=None):
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''
    rclpy.init(args=args)                                           # initialisation
    
    theme=ThemeImplementation()                                     # creating a object from Theme Implementation
    
    executor = rclpy.executors.MultiThreadedExecutor(2)             #making a rclpy multithreaded executor for handling spin
    executor.add_node(theme)                                        #adding the object in executor for spinning
    executor.spin()                                                 #spining on the object to make it alive in ROS 2 DDS

if __name__ == '__main__':
    '''
    Description:    If the python interpreter is running that module (the source file) as the main program, 
    '''
    main()
