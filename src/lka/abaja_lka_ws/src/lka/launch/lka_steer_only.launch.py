"""
LKA Full Stack Launch File

Launches the COMPLETE LKA stack including:
- CarMaker bridge (carmakercamera.py)
- RSDS camera publisher
- Lane detection (UFLDv2)
- Stanley controller
- Vehicle control (steering relay)
- Performance logger
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('lka'),
        'config',
        'lka_params.yaml'
    )

    return LaunchDescription([
        # CarMaker bridge (telemetry + steering DVA)
        Node(
            package='pycarmaker',
            executable='carmakercamera',
            name='carmakercamera',
            output='screen',
        ),

        # RSDS camera stream publisher
        Node(
            package='camera_sensor',
            executable='rsds_camera_publisher',
            name='rsds_camera_publisher',
            output='screen',
        ),

        # Lane Detection Node (UFLDv2)
        Node(
            package='lka',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen',
            parameters=[config_file],
        ),

        # Stanley Controller Node
        Node(
            package='lka',
            executable='stanley_controller_node',
            name='stanley_controller_node',
            output='screen',
            parameters=[config_file],
        ),

        # Vehicle Control Node (steering relay)
        Node(
            package='lka',
            executable='vehicle_control_node',
            name='vehicle_control_node',
            output='screen',
            parameters=[config_file],
        ),

        # Performance Logger Node
        Node(
            package='lka',
            executable='performance_logger_node',
            name='performance_logger_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
