import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 1. Load the MoveIt Configuration
    moveit_config = MoveItConfigsBuilder("robotic_arm", package_name="robotic_arm_config").to_moveit_configs()

    # 2. Define the Monitor Node
    monitor_node = Node(
        name="position_monitor_node",
        package="robotic_arm_config",
        executable="position_monitor_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([monitor_node])