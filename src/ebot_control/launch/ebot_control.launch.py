from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    docking_service= Node(
        package="ebot_control",
        name="my_robot_docking_controller",
        executable="hardware_docking.py",
        output="screen"
    )

    task_4C = Node(
        package="ebot_control",
        name="task_5",
        executable="ebot_dock",
        output="screen"
    )

    aruco_tf_process = Node(
        package="ebot_control",
        name="aruco_tf_process",
        executable="hardware_aruco.py",
        output="screen"
    )

    task_4A = Node(
        package="ebot_control",
        name="task5",
        executable="hardware_servo.py",
        output="screen"
    )

    ld.add_action(docking_service)
    ld.add_action(task_4C)
    ld.add_action(aruco_tf_process)
    ld.add_action(task_4A)


    return ld