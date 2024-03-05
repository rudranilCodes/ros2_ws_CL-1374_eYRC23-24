#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from std_msgs.msg import Float32MultiArray, Float32
import math
from pymoveit2 import MoveIt2
from threading import Thread
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from controller_manager_msgs.srv import SwitchController 

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


class arm_pose(Node):
    def __init__(self):
        super().__init__('arm_pose')
        self.timer=self.create_timer(0.01,self.arm_run())
    def arm_run(self):
        
        # Declare parameter for joint positions
        self.declare_parameter(
            "joint_positions",
            [  -0.261799,
                -2.47837,
                2.40855,
                -3.14159,
                -1.58825,
                3.14159
            ],
        )

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()
        change=SwitchtoMoveIt()
        change.change()
        # Create MoveIt 2 interface
        moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        # Get parameter
        joint_positions = (
            self.get_parameter("joint_positions").get_parameter_value().double_array_value
        )

        # Move to joint configuration
        self.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
        moveit2.move_to_configuration(joint_positions)
        moveit2.wait_until_executed()
        
def main(args=None):
    rclpy.init(args=args)
    arm=arm_pose()
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(arm)
    executor.spin_once()
    
if __name__ == '__main__':
    main()