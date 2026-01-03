from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'gazebo_pubsub_robot'

setup(
    name=package_name,
    version='0.0.0',

    # Automatically find Python packages
    packages=find_packages(exclude=['test']),

    # Install non-Python files
    data_files=[
        # Required ROS index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # Package manifest
        ('share/' + package_name, ['package.xml']),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        # URDF files
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='jaydhake',
    maintainer_email='your@email.com',
    description='ROS 2 Gazebo publisher-subscriber robot',
    license='Apache License 2.0',

    entry_points={
        'console_scripts': [
            'cmd_publisher = gazebo_pubsub_robot.cmd_publisher:main',
        ],
    },
)

