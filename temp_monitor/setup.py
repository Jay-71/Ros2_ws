from setuptools import find_packages, setup

package_name = 'temp_monitor'

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
    description='Temperature monitoring package',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'temp_alert = temp_monitor.temp_alert:main',
            'alert_service = temp_monitor.alert_service:main',
            'temp_publisher = temp_monitor.temp_publisher:main',
        ],
    },
)

