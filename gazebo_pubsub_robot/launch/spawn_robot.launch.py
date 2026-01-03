from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('gazebo_pubsub_robot')
    urdf_path = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    return LaunchDescription([

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            additional_env={
                'GAZEBO_PLUGIN_PATH': '/opt/ros/humble/lib'
            },
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'pubsub_bot',
                '-file', urdf_path,
                '-z', '1.0'
            ],
            output='screen'
        )
    ])

