#!/usr/bin/env/python3
import math
from geometry_msgs.msg import TransformStamped
import tf2_ros
from std_msgs.msg import String
import ast
from math import cos, sin
import math, time
from copy import deepcopy
from pymoveit2 import MoveIt2
import rclpy
from rclpy.task import Future
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
#from linkattacher_msgs.srv import AttachLink
#from linkattacher_msgs.srv import DetachLink
from ur_msgs.srv import SetIO
from threading import Thread
from controller_manager_msgs.srv import SwitchController 

arm_manipulator=None
box1=[]
box2=[]
box3=[]
current_position=None
message=""

prev_box_name=""
flag=0
linear=[]
angular=[]
wrist_3_link_pose=[]
diff=[]
mag=0.0
unit_vector=[0.0,0.0,0.0]


"""
class MagnetON(Node):
    def __init__(self):
        super().__init__('GripperMagnetON_client')

    def runON(self, obj):
        # Create a thread for the service client.
        self.call_Attacher(obj, 'link', 'ur5', 'wrist_3_link')

    def call_Attacher(self, model1_name, link1_name, model2_name, link2_name):
        gripper_control = self.create_client(AttachLink, '/GripperMagnetON')

        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        req = AttachLink.Request()
        req.model1_name = str(model1_name)
        req.link1_name  = link1_name       
        req.model2_name = model2_name       
        req.link2_name  = link2_name 
        # Call the service asynchronously
        future = gripper_control.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

class MagnetOFF(Node):
    def __init__(self):
        super().__init__('GripperMagnetOFF_client')

    def runOFF(self, obj):
        self.call_Detacher(obj, 'link', 'ur5', 'wrist_3_link')

    def call_Detacher(self, model1_name, link1_name, model2_name, link2_name):
        gripper_control = self.create_client(DetachLink, '/GripperMagnetOFF')

        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF service not available, waiting again...')

        req = DetachLink.Request()
        req.model1_name = str(model1_name)
        req.link1_name  = link1_name       
        req.model2_name = model2_name       
        req.link2_name  = link2_name 
        future = gripper_control.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
"""
class Emagnet(Node):
    def __init__(self):
        super().__init__('Emagnet_client')
    def gripper_call(self, state):
        gripper_control = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF Tool service not available, waiting again...')
        req         = SetIO.Request()
        req.fun     = 1
        req.pin     = 16
        req.state   = float(state)
        future=gripper_control.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        #return state


class SwitchtoMoveIt(Node):
    def __init__(self):
        super().__init__('SwitchtoMoveIt_Client')
        self.__contolMSwitch = self.create_client(SwitchController, "/controller_manager/switch_controller")
    def change(self):
        switchParam = SwitchController.Request()
        switchParam.activate_controllers = ["scaled_joint_trajectory_controller"] # for normal use of moveit
        switchParam.deactivate_controllers = ["forward_position_controller"] # for servoing
        switchParam.strictness = 2
        switchParam.start_asap = False

        # calling control manager service after checking its availability
        while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Service control Manager is not yet available...")
        
        future=self.__contolMSwitch.call_async(switchParam)
        rclpy.spin_until_future_complete(self, future)
        print("[CM]: Switching Complete")

class SwitchtoServo(Node):
    def __init__(self):
        super().__init__('SwitchtoServo_Client')
        self.__contolMSwitch = self.create_client(SwitchController, "/controller_manager/switch_controller")
    def change(self):
        switchParam = SwitchController.Request()
        switchParam.deactivate_controllers = ["scaled_joint_trajectory_controller"] # for normal use of moveit
        switchParam.activate_controllers = ["forward_position_controller"] # for servoing
        switchParam.strictness = 2
        switchParam.start_asap = False

        # calling control manager service after checking its availability
        while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Service control Manager is not yet available...")
        
        future=self.__contolMSwitch.call_async(switchParam)
        rclpy.spin_until_future_complete(self, future)
        print("[CM]: Switching Complete")


class Arm_Manipulator(Node):
    def __init__(self):
        super().__init__("Arm_servoing")
        self.__twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.get_logger().info('Node created: Arm_Manipulator')
        self.callbackgroup=ReentrantCallbackGroup()
        # Initialize message based on passed arguments 
        self.__twist_msg = TwistStamped()
        self.__twist_msg.header.frame_id = ur5.base_link_name()
        self.__twist_msg.twist.linear.x = 1.0
        self.__twist_msg.twist.linear.y = 1.0
        self.__twist_msg.twist.linear.z = 1.0
        self.__twist_msg.twist.angular.x = 1.0
        self.__twist_msg.twist.angular.y = 1.0
        self.__twist_msg.twist.angular.z = 1.0
        self.timer1 = self.create_timer(0.008, self.servo_circular_motion)
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.callbakgroup=ReentrantCallbackGroup()
    def servo_circular_motion(self):
        box_position = current_position
        print(box_position)
        while True:
            try:
                self.t2 = self.tf_buffer.lookup_transform('base_link','wrist_3_link', rclpy.time.Time())
                global wrist_3_link_pose, diff, mag, unit_vector
                wrist_3_link_pose = [self.t2.transform.translation.x, self.t2.transform.translation.y, self.t2.transform.translation.z]
                diff = [box_position[0] - wrist_3_link_pose[0], box_position[1] - wrist_3_link_pose[1], box_position[2] - wrist_3_link_pose[2]]
                mag = math.sqrt((diff[0]**2 + diff[1]**2 + diff[2]**2))
                unit_vector = [(diff[0])/mag, diff[1]/mag, diff[2]/mag]
                self.get_logger().info(f'Successfully listened to tf unit vector :{unit_vector} distance : {mag}')
                break
            except tf2_ros.LookupException as e:
                self.get_logger().info(f"LookupException: {e}")
                break
            except tf2_ros.ConnectivityException as e2:
                self.get_logger().info(f"ConnectivityException: {e2}")
                break
        self.twist_msg = deepcopy(self.__twist_msg)
        self.twist_msg.header.stamp = self.get_clock().now().to_msg()
        global linear, angular
        linear=unit_vector
        angular=[0.0,0.0,0.0]
        if unit_vector==[0.0,0.0,0.0]:
            self.get_logger().info('Skipped')
            return
        if mag < 0.1:
            self.twist_msg.twist.linear.x *= 0.0
            self.twist_msg.twist.linear.y *= 0.0
            self.twist_msg.twist.linear.z *= 0.0
            self.twist_msg.twist.angular.x *= 0.0
            self.twist_msg.twist.angular.y *= 0.0
            self.twist_msg.twist.angular.z *= 0.0
            self.__twist_pub.publish(self.twist_msg)
            global flag
            flag=1
        else:
            self.twist_msg.twist.linear.x *= linear[0]
            self.twist_msg.twist.linear.y *= linear[1]
            self.twist_msg.twist.linear.z *= linear[2]
            self.twist_msg.twist.angular.x *= angular[0]
            self.twist_msg.twist.angular.y *= angular[1]
            self.twist_msg.twist.angular.z *= angular[2]
            self.__twist_pub.publish(self.twist_msg)
            self.get_logger().info('Successfully published to twist')
    def arm_run(self):
        #self.servo_circular_motion()
        rclpy.spin_once(self)


def main():
        try:
            rclpy.init()
            
            callback_group = ReentrantCallbackGroup()
            node= Node("task5")
            node.get_logger().info('Node created: Task_5')
            moveit=SwitchtoMoveIt()
            servo=SwitchtoServo()
            emagnet = Emagnet()
            arm_manipulator = Arm_Manipulator()

            node.declare_parameter(
            "home_pose",
            [
                -0.261799,
                -2.47837,
                2.40855,
                -3.14159,
                -1.58825,
                3.14159
            ],
            )

            node.declare_parameter(
            "drop_pose",
            [
                0.0,
                -1.81514,
                -1.3090,
                -3.07178,
                -1.58825,
                3.14159
            ],
            )
            moveit2 = MoveIt2(
                node=node,
                joint_names=ur5.joint_names(),
                base_link_name=ur5.base_link_name(),
                end_effector_name=ur5.end_effector_name(),
                group_name=ur5.MOVE_GROUP_ARM,
                callback_group=callback_group,
            )

            joint_positions2 = (
                node.get_parameter("home_pose").get_parameter_value().double_array_value
            )

            joint_positions3 = (
                node.get_parameter("drop_pose").get_parameter_value().double_array_value
            )
            while True:
                global box1, box2, box3, current_position, flag,prev_box_name
                box1=[]
                current_position=None
                def message_callback(msg):
                    try:
                        global message, box1, box2, box3,prev_box_name
                        message = msg.data  # Get the string data from the message
                        message = ast.literal_eval(message)
                        #print(message)

                        # Iterating through the main list and storing each sublist separately
                        for sublist in message:
                            if sublist is not None:
                                if isinstance(sublist, list):
                                    if len(sublist) == 0:
                                        continue
                                    if sublist == message[0]:
                                        box1 = list(sublist)
                                    elif sublist == message[1]:
                                        box2 = list(sublist)
                                    elif sublist == message[2]:
                                        box3 = list(sublist)

                        # Displaying the stored sublists separately
                        node.destroy_subscription(tf_subscription)
                    except Exception as e:
                        print("Error while converting message:", e)
                    # Create a TF subscriber within the main method
                tf_subscription = node.create_subscription(
                String,
                '/topic',
                message_callback,
                10
                )

                rclpy.spin_once(node)


                if box1:
                    if prev_box_name!=box1[0]:
                        node.declare_parameter(
                        box1[0],
                        [
                            math.radians(box1[7]),
                            -2.23402,
                            1.98968,
                            -2.93215,
                            -1.58825,
                            3.14159
                        ],
                        )
                    

                    # Create MoveIt 2 interface
                        try:
                            if box1:
                                joint_positions1 = (
                                    node.get_parameter(box1[0]).get_parameter_value().double_array_value
                                )
                        except Exception as e:
                            pass
                        # Move to joint configuration
                        moveit.change()
                        
                        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions1)}}}")
                        moveit2.move_to_configuration(joint_positions1)
                        #moveit2.wait_until_executed()
                        time.sleep(7.0)

                        current_position = box1[1]
                        
                        
                        servo.change()
                        emagnet.gripper_call(True)
                        while flag == 0:
                            arm_manipulator.arm_run()
                        flag=0
                        time.sleep(1.0)

                        current_position = box1[2]
                        while flag == 0:
                            arm_manipulator.arm_run()
                        flag=0
                        time.sleep(1.0)

                        current_position = box1[3]
                        while flag == 0:
                            arm_manipulator.arm_run()
                        flag=0
                        time.sleep(2.0)


                        #Emagnet = Emagnet()
                        
                        time.sleep(2.0)
                    
                        current_position = box1[2]
                        while flag == 0:
                            arm_manipulator.arm_run()
                        flag=0
                        time.sleep(1.0)

                        current_position = box1[1]
                        while flag == 0:
                            arm_manipulator.arm_run()
                        flag=0
                        time.sleep(1.0)
                        
                        moveit.change()
                        
                        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions2)}}}")
                        moveit2.move_to_configuration(joint_positions2)
                        time.sleep(3.0)

                        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions3)}}}")
                        moveit2.move_to_configuration(joint_positions3)
                        time.sleep(15.0)

                        emagnet.gripper_call(False)
                            
                        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions2)}}}")
                        moveit2.move_to_configuration(joint_positions2)
                        time.sleep(2.0)
                        prev_box_name=box1[0]



        # Spin the node in background thread(s)
                executor = rclpy.executors.MultiThreadedExecutor(2)
                executor.add_node(node)
                executor.spin_once()
                executor.remove_node(node)
        # Shutdown nodes and the executor when exiting
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt detected. Exiting...")
        except Exception as e:
            if 'node' in locals():
                node.get_logger().error(f"An exception occurred: {str(e)}")
        finally:
        # Cleanup operations
            if 'node' in locals():
                node.destroy_node()
            if 'Arm_servoing' in locals():
                arm_manipulator.destroy_node()
        rclpy.shutdown()
        exit(0)

        
if __name__ == '__main__':

    main()