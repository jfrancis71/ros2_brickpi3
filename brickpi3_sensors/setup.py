import os
from glob import glob
from setuptools import setup

package_name = 'brickpi3_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "battery_node = brickpi3_sensors.battery_node",
            "color_sensor_node = brickpi3_sensors.color_sensor_node",
            "compass_node = brickpi3_sensors.compass_node",
            "differential_drive_node = brickpi3_sensors.differential_drive_node",
            "gyro_node = brickpi3_sensors.gyro_node",
            "infrared_distance_node = brickpi3_sensors.infrared_distance_node",
            "touch_sensor_node = brickpi3_sensors.touch_sensor_node",
            "ultrasonic_distance_node = brickpi3_sensors.ultrasonic_distance_node",
        ],
    },
)
