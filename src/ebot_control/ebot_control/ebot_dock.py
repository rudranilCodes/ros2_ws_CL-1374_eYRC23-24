import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
#from geometry_msgs.msg import Twist  
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.parameter import ParameterValue
from rclpy.parameter import ParameterType 
import cmd
from std_msgs.msg import Int32  
from usb_relay.srv import RelaySw                        
import math, statistics
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import yaml
from scipy.spatial.transform import Rotation as R
import subprocess
flag=1
initial_pose1 = [0.0,0.0,0.0,0.0]

with open('/home/student/student_workspace/cl_1374/src/ebot_control/ebot_control/config.yaml', 'r') as read_file:
    contents = yaml.safe_load(read_file)

rack1_xy_yaw = contents['position'][0]['rack1']
rack2_xy_yaw = contents['position'][1]['rack2']
rack3_xy_yaw = contents['position'][2]['rack3']
# arm_pose=contents['position'][3]['arm']
rack2_dock_xy_yaw = contents['position'][5]['rack2_dock']
rack3_dock_xy_yaw = contents['position'][4]['rack3_dock']

print("Rack 1 xy, yaw:", rack1_xy_yaw)
print("Rack 2 xy, yaw:", rack2_xy_yaw)
print("Rack 3 xy, yaw:", rack3_xy_yaw)
#print("arm_pose yaw:",arm_pose)
print("Rack 2 dock xy, yaw:", rack2_dock_xy_yaw)
print("Rack 3 dock xy, yaw:", rack3_dock_xy_yaw)

r = R.from_euler('xyz', [0, 0,rack3_xy_yaw[2]], degrees=False)
r3=r.as_quat()
r = R.from_euler('xyz', [0, 0,rack2_xy_yaw[2]], degrees=False)
r2=r.as_quat()
# r = R.from_euler('xyz', [0, 0,arm_pose[2]], degrees=False)
# a=r.as_quat()
r = R.from_euler('xyz', [0, 0,rack2_dock_xy_yaw[2]], degrees=False)
rd2=r.as_quat()
r = R.from_euler('xyz', [0, 0,rack3_dock_xy_yaw[2]], degrees=False)
rd3=r.as_quat()


class USB_Relay(Node):
    def __init__(self):
        super().__init__('usb_relay_client')

    def run_usb(self, relaychannel, relaystate):
        self.call_ebot_Attacher(relaychannel, relaystate)

    def call_ebot_Attacher(self, relaychannel, relaystate):
        ebot_attacher = self.create_client(RelaySw, '/usb_relay_sw')

        while not ebot_attacher.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Relay service not available, waiting again...')

        req = RelaySw.Request()
        req.relaychannel = bool(relaychannel)
        req.relaystate  = bool(relaystate)
        # Call the service asynchronously
        future = ebot_attacher.call_async(req)
        rclpy.spin_until_future_complete(self, future)

class GParamchange(Node):
    def __init__(self):
        super().__init__('Global_Param_change_client')
    def call_param_change(self,mode):
        changer=self.create_client(SetParameters,"/global_costmap/global_costmap/set_parameters")
        request=SetParameters.Request()
        if mode == 0:
            x=Parameter(name="robot_radius",value=0.10)
        elif mode == 1:
            x=Parameter(name="robot_radius",value=0.30)
        request.parameters=[x.to_parameter_msg()]
        future=changer.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        response = future.result()
        print(response.results)

class Lparamchange(Node):
    def __init__(self):
        super().__init__('Local_Param_change_client')
    def call_param_change(self,mode):
        changer=self.create_client(SetParameters,"/local_costmap/local_costmap/set_parameters")
        request=SetParameters.Request()
        if mode == 0:
            x=Parameter(name="inflation_layer.inflation_radius",value=0.20)
        elif mode == 1:
            x=Parameter(name="inflation_layer.inflation_radius",value=0.45)
        request.parameters=[x.to_parameter_msg()]
        future=changer.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        response = future.result()
        print(response.results)

class Docking(Node):
    def __init__(self):
        super().__init__('Docking_client')

    def run_docker(self, distance1,distance2,orientaion,rack_no ,undocking):
        self.call_ebot_Docker(distance1, distance2,orientaion,rack_no,undocking)

    def call_ebot_Docker(self, distance1,distance2,orientation, rack_no,undocking):
        ebot_docker = self.create_client(DockSw, '/dock_control')

        while not ebot_docker.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Docking service not available, waiting again...')

        req = DockSw.Request()
        req.linear_dock = undocking
        req.orientation_dock  = True
        #req.undocking=undocking
        req.distance1 = distance1  
        req.distance2 = distance2    
        req.orientation =orientation 
        req.rack_no=str(rack_no)
        future = ebot_docker.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,)) 




# Define your new class MyBotNavigator
class MyBotNavigator(Node):
    def __init__(self):
        super().__init__('my_bot_navigator')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        

        
        global flag
        #self.success = 0
        self.publisher = self.create_publisher(Int32, 'flag', 10)
        self.publisher_function()
        ########################################INITIAL GOAL POSE PUBLISHING##########################################
        self.navigator = BasicNavigator()
        global initial_pose1
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = initial_pose1[0]
        initial_pose.pose.position.y = initial_pose1[1]
        initial_pose.pose.orientation.z = initial_pose1[2]
        initial_pose.pose.orientation.w = initial_pose1[3]
        self.navigator.setInitialPose(initial_pose)


        self.navigator.waitUntilNav2Active()


        ############################################# FOR RACK 3 DOCk #########################################################
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rack3_xy_yaw[0] +0.8 #0.5, 2.05, 0.0
        goal_pose.pose.position.y = rack3_xy_yaw[1] +0.10
        goal_pose.pose.orientation.z = r3[2]
        goal_pose.pose.orientation.w = r3[3]

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        self.navigator.goToPose(goal_pose)


        

        i = 0
        while not self.navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################
            self.publisher_function()
        # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    
                    self.navigator.goToPose(goal_pose)


                # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        attach_detach = USB_Relay()
        attach_detach.run_usb(1,1)

        gparam=GParamchange()
        
        gparam.call_param_change(mode=0)

        lparam=Lparamchange()
        lparam.call_param_change(mode=0)

        # command="ros2 param set /global_costmap/global_costmap robot_radius 0.20"
        # output = subprocess.check_output(command, shell=True, text=True)
        # print(output)

        docking=Docking()
        docking.run_docker(float(rack3_xy_yaw[0]),float(rack3_xy_yaw[1]+0.10),float(rack3_xy_yaw[2]+0.10),'rack3',False)

        #time.sleep(5.0)
        flag=0
        self.publisher_function()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rack3_xy_yaw[0] +1.0 #0.5, 2.05, 0.0
        goal_pose.pose.position.y = rack3_xy_yaw[1] +0.10
        goal_pose.pose.orientation.z = r3[2]
        goal_pose.pose.orientation.w = r3[3]

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        self.navigator.goToPose(goal_pose)


        

        i = 0
        while not self.navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################
            self.publisher_function()
        # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    
                    self.navigator.goToPose(goal_pose)


                # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')


        gparam.call_param_change(mode=1)
        lparam.call_param_change(mode=1)

         ############################################# FOR RACK 3 UNDOCk #########################################################


        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rack3_dock_xy_yaw[0]-0.85
        goal_pose.pose.position.y =rack3_dock_xy_yaw[1]-0.35
        goal_pose.pose.orientation.z = rd3[2]
        goal_pose.pose.orientation.w = rd3[3]
        self.navigator.goToPose(goal_pose)
        # command="ros2 param set /global_costmap/global_costmap robot_radius 0.30"
        # output = subprocess.check_output(command, shell=True, text=True)
        # print(output)
        i = 0
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            self.publisher_function()
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                
                    self.navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        # command="ros2 param set /global_costmap/global_costmap robot_radius 0.20"
        # output = subprocess.check_output(command, shell=True, text=True)
        # print(output)
        gparam.call_param_change(mode=0)

        docking.run_docker(float(rack3_dock_xy_yaw[0]+0.25),float(rack3_dock_xy_yaw[1]-0.20),float(rack3_dock_xy_yaw[2]+0.05),'rack3',True)
        attach_detach.run_usb(1,0)
        time.sleep(2.0)
        flag=1
        self.publisher_function()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rack3_dock_xy_yaw[0]-0.85
        goal_pose.pose.position.y =rack3_dock_xy_yaw[1]-0.20
        goal_pose.pose.orientation.z = rd3[2]
        goal_pose.pose.orientation.w = rd3[3]
        self.navigator.goToPose(goal_pose)
        # command="ros2 param set /global_costmap/global_costmap robot_radius 0.30"
        # output = subprocess.check_output(command, shell=True, text=True)
        # print(output)
        i = 0
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            self.publisher_function()
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                
                    self.navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        gparam.call_param_change(mode=1)

        # command="ros2 param set /global_costmap/global_costmap robot_radius 0.30"
        # output = subprocess.check_output(command, shell=True, text=True)
        # print(output)

        #################################################FOR RACK 2 DOCk##############################################################

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rack2_xy_yaw[0]+1.0
        goal_pose.pose.position.y = rack2_xy_yaw[1]-0.10
        goal_pose.pose.orientation.z = r2[2]
        goal_pose.pose.orientation.w = r2[3]

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        self.navigator.goToPose(goal_pose)


        

        i = 0
        while not self.navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################
            self.publisher_function()
        # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    
                    self.navigator.goToPose(goal_pose)


                # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        gparam.call_param_change(mode=0)
        lparam.call_param_change(mode=0)
    
        # command="ros2 param set /global_costmap/global_costmap robot_radius 0.20"
        # output = subprocess.check_output(command, shell=True, text=True)
        # print(output)

        attach_detach.run_usb(1,1)
        docking.run_docker(float(rack2_xy_yaw[0]), float(rack2_xy_yaw[1]-0.15),float(rack2_xy_yaw[2]-0.20),'rack2',False)   
        flag=0
        self.publisher_function()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rack2_xy_yaw[0]+0.8
        goal_pose.pose.position.y = rack2_xy_yaw[1]-0.10
        goal_pose.pose.orientation.z = r2[2]
        goal_pose.pose.orientation.w = r2[3]

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        self.navigator.goToPose(goal_pose)


        

        i = 0
        while not self.navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################
            self.publisher_function()
        # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    
                    self.navigator.goToPose(goal_pose)


                # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        gparam.call_param_change(mode=1)
        lparam.call_param_change(mode=1)


        #################################################FOR RACK 2 UNDOCk##############################################################
    
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rack2_dock_xy_yaw[0]-0.10
        goal_pose.pose.position.y = rack2_dock_xy_yaw[1]-1.2
        goal_pose.pose.orientation.z = rd2[2]
        goal_pose.pose.orientation.w = rd2[3]

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)
        # command="ros2 param set /global_costmap/global_costmap robot_radius 0.20"
        # output = subprocess.check_output(command, shell=True, text=True)
        # print(output)
        self.navigator.goToPose(goal_pose)


        

        i = 0
        while not self.navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################
            self.publisher_function()
        # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # Some navigation request change to demo preemption
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
    
                    self.navigator.goToPose(goal_pose)


                # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        gparam.call_param_change(mode=0)

        docking.run_docker(float(rack2_dock_xy_yaw[0]),float(rack2_dock_xy_yaw[1]+0.20),float(rack2_dock_xy_yaw[2]+0.15),'rack2',True)
        attach_detach.run_usb(1,0)
        time.sleep(2.0)
        flag=1
        self.publisher_function()

        gparam.call_param_change(mode=1)
        self.navigator.lifecycleShutdown()
        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)


        
    def publisher_function(self):
        global flag
        msg=Int32()
        msg.data=flag
        self.publisher.publish(msg)


    # Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)
    
    
    node= Node("task_5")
    def odometry_callback(msg):
        global initial_pose1
        # Extract and update robot pose information from odometry message
        initial_pose1[0] = msg.pose.pose.position.x
        initial_pose1[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        initial_pose1[2] = quaternion_array.z
        initial_pose1[3] = quaternion_array.w
        node.destroy_subscription(odom_sub)
    odom_sub=node.create_subscription(
            Odometry,
            'odom',
            odometry_callback,
            10
            )
    rclpy.spin_once(node)
    global initial_pose1
    print(initial_pose1)
    
    my_bot_navigator = MyBotNavigator()
    #script_name = 'ebot_docking_boilerplate.py'
    #subprocess.run(['python3', script_name])

    executor = MultiThreadedExecutor()
    executor.add_node(my_bot_navigator)

    executor.spin()

    my_bot_navigator.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
