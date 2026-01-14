from setuptools import setup
from glob import glob
import os

package_name = 'simple_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaydhake',
    maintainer_email='your@email.com',
    description='Simple robot description package',
    license='Apache License 2.0',
)

