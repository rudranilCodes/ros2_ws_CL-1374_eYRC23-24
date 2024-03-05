#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray,Float32
from sensor_msgs.msg import Range,Imu
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
import pytest
import time

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 20)

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultra_sub = self.create_subscription(Float32MultiArray, '/ultrasonic_sensor_std_float', self.ultra_callback, 10)
        
        # Add another one here
        self.imu_sub = self.create_subscription(Float32, '/orientation', self.imu_callback, 100)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 20)    

        # Initialize all  flags and parameters here
        self.is_docking = False
        self.dock_aligned=False
        self.usrleft_value = 1.5
        self.usrright_value = 1.5        
        self.desired_distance = 0.1 # Replace with your desired distance
        self.kp = 0.25 # Adjust as needed  
        self.orientation_value = 0.0
        self.orientation_error = 0.0
        self.kp_orientation = 0.5
        self.kp_ultra = 0.7
        self.robot_pose = [0.0,0.0,0.0]
        self.distance1=0.0
        self.distance2=0.0
        self.linear_dock=False 
       

        # Initialize a timer for the main control loop
        self.timer=self.create_timer(1.0,self.controller_loop)
    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    # Callback function for the left ultrasonic sensor
    def ultra_callback(self,msg):
        self.usrleft_value= msg.data[4]/100.00
        self.usrright_value = msg.data[5]/100.00
    # Callback function for the right ultrasonic sensor
    #
    #
    def imu_callback(self, msg):
        yaw = msg.data
        if yaw >math.pi:
            yaw -= 2*math.pi
        self.orientation_error = self.normalize_angle(self.orientation_value - yaw)   #***
        #self.orientation_error = self.orientation_value - yaw    #have to put rack's orientation at self.orientation
        print("orientation error=",self.orientation_error)

    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
    def normalize_angle(self, angle):
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            return angle

    # Main control loop for managing docking behavior

    def controller_loop(self):
        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and executio
        #if self.is_docking:
        if self.linear_dock:
            if self.is_docking:
                if self.orientation_error > 0.20 or self.orientation_error < -0.20 :
                    angular_velocity = self.kp_orientation * self.orientation_error
                    cmd = Twist()
                    cmd.angular.z = -angular_velocity
                    print(f"Stopping with angular velocity: {cmd.angular.z}")
                    self.velocity_publisher.publish(cmd)
                    self.is_docking=True
                    self.dock_aligned=False

                else:
                    error1=abs(self.distance1-self.robot_pose[0])
                    error2=abs(self.distance2-self.robot_pose[1])
                    error = max(error1,error2)
                    if abs(error) > 0.5:
                        print(f"Linear Error: {error}")
                        linear_velocity = -self.kp * error
                        cmd = Twist()
                        cmd.linear.x = linear_velocity
                        print(f"Stopping with linear velocity: {cmd.linear.x}")
                        self.velocity_publisher.publish(cmd)
                        time.sleep(0.5)
                    else:
                        cmd = Twist()
                        cmd.linear.x = 0.0
                        cmd.linear.y = 0.0
                        cmd.angular.z = 0.0 
                        print(f"Boddi")
                        self.velocity_publisher.publish(cmd)
                        self.is_docking=False
                        self.dock_aligned=True
                    pass
        else:
            ultra_error = self.usrleft_value - self.usrright_value

            if self.is_docking:

                if self.orientation_error > 0.18 or self.orientation_error < -0.18 :
                    angular_velocity = self.kp_orientation * self.orientation_error
                    cmd = Twist()
                    cmd.angular.z = -angular_velocity
                    print(f"Stopping with angular velocity: {cmd.angular.z}")
                    self.velocity_publisher.publish(cmd)
                    self.is_docking=True
                    self.dock_aligned=False

                else:
                    """if ultra_error > 0.05 or ultra_error < -0.05:
                        print(f"Ultrasonic Orientation Error: {ultra_error}")
                        cmd = Twist()
                        cmd.angular.z = self.kp_ultra*ultra_error
                        print(f"Ultra angular Velocity:{cmd.angular.z}")
                        self.velocity_publisher.publish(cmd)
                        self.is_docking=True
                        self.dock_aligned=False
                    else:"""
                    if max(self.usrleft_value,self.usrright_value) > 0.40:
                        error_left = self.desired_distance - self.usrleft_value
                        
                        error_right = self.desired_distance - self.usrright_value
                        self.kp_docking = 0.2

                        print("error left:", error_left)
                        print("error right:", error_right)
                        linear_velocity_left = -self.kp_docking * self.usrleft_value
                        linear_velocity_right = -self.kp_docking * self.usrright_value

                        cmd = Twist()
                        cmd.linear.x = max(linear_velocity_left,linear_velocity_right)
                        print(f"Stopping with linear velocity: {cmd.linear.x}")
                        self.velocity_publisher.publish(cmd)
                        #time.sleep(0.05)
                        

                    else:
                        cmd = Twist()
                        cmd.linear.x=-0.06
                        self.velocity_publisher.publish(cmd)
                        print(f"docking in process")
                        time.sleep(6.0)
                        print(f"done")

                        cmd.linear.x = 0.0
                        cmd.linear.y = 0.0
                        cmd.angular.z = 0.0 
                        self.velocity_publisher.publish(cmd)
                        print(f"Bodda")
                        self.is_docking=False
                        self.dock_aligned=True
                        time.sleep(4.0)
                    pass




    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        #
        #
        self.is_docking=True
        self.distance1=request.distance1
        self.distance2=request.distance2
        self.orientation_value=request.orientation
        self.linear_dock=request.linear_dock
        # Reset flags and start the docking process
        #
        #


        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(1.0, self.get_clock())

        # Wait until the robot is aligned for docking
        while not self.dock_aligned:
            self.get_logger().info("Aligning")
            rate.sleep()

        # Set the service response indicating success
        response.success = True
        response.message = "Docking Completed"
        self.dock_aligned=False
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


