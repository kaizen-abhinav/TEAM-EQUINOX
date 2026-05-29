from setuptools import setup
import os
from glob import glob

package_name = 'lka'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhinav',
    maintainer_email='abhinav@todo.com',
    description='Lane Keeping Assist system using UFLDv2 and Stanley controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lane_detection_node = lka.lane_detection_node:main',
            'stanley_controller_node = lka.stanley_controller_node:main',
            'vehicle_control_node = lka.vehicle_control_node:main',
            'performance_logger_node = lka.performance_logger_node:main',
        ],
    },
)
