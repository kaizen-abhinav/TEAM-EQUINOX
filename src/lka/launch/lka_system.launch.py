"""
LKA System Launch File

Launches the complete LKA stack:
1. CarMaker Telemetry Bridge
2. RSDS Camera Publisher
3. Lane Detection (UFLDv2)
4. Stanley Controller
5. Vehicle Control Relay
6. Performance Logger
7. Visualization Window
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Load parameters from YAML
    config_file = os.path.join(
        get_package_share_directory('lka'),
        'config',
        'lka_params.yaml'
    )

    return LaunchDescription([
        # CarMaker Telemetry Bridge Node
        Node(
            package='pycarmaker',
            executable='carmakercamera',
            name='carmaker_complete_interface',
            output='screen',
        ),

        # RSDS Camera Publisher Node
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

        # Stanley Controller Node (computes steering from lane center)
        Node(
            package='lka',
            executable='stanley_controller_node',
            name='stanley_controller_node',
            output='screen',
            parameters=[config_file],
        ),

        # Vehicle Control Node (steering relay to CarMaker bridge)
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

        # Visualization Window
        Node(
            package='image_tools',
            executable='showimage',
            name='lane_visualization',
            remappings=[('image', '/lka/lane_image')],
            output='screen',
        )
    ])
