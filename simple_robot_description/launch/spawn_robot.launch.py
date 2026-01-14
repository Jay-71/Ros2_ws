from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('simple_robot_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'simple_robot.urdf')

    return LaunchDescription([

        # Start Gazebo WITH factory plugin AND explicit env
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '-s', 'libgazebo_ros_factory.so'
            ],
            additional_env={
                'GAZEBO_PLUGIN_PATH': '/opt/ros/humble/lib'
            },
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'simple_robot',
                '-file', urdf_path,
                '-x', '0',
                '-y', '0',
                '-z', '1.0'
            ],
            output='screen'
        )
    ])

