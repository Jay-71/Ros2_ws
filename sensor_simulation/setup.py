from setuptools import find_packages, setup

package_name = 'sensor_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jaydhake',
    maintainer_email='jaydhake@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_reading = sensor_simulation.ultrasonic_reading:main',
            'distance_subscriber = sensor_simulation.distance_subscriber:main',
        ],
    },
)
