from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    
    # Load MoveIt Config
    moveit_config = MoveItConfigsBuilder("robotic_arm", package_name="robotic_arm_config").to_moveit_configs()

    # The C++ Node
    move_arm_node = Node(
        name="move_arm_node",
        package="robotic_arm_config",
        executable="move_arm_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([move_arm_node])