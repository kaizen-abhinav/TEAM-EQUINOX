"""
Endurance Event System Launch File

Combines:
1. Unified CarMaker Bridge (LKA + AEB interface)
2. AEB Fuzzy Controller (Longitudinal)
3. UFLDv2 Lane Detection (Lateral)
4. Stanley Controller (Lateral)
5. Camera/Sensors & Relays
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
        # --- UNIFIED CARMAKER BRIDGE ---
        Node(
            package='endurance',
            executable='carmakercamera.py',
            name='carmaker_complete_interface',
            output='screen',
        ),

        # --- AEB LONGITUDINAL CONTROLLER ---
        Node(
            package='aeb',
            executable='fuzzy_aeb_node',
            name='aeb_fuzzy_node',
            output='screen',
        ),

        # --- CAMERA SENSOR PUBLISHER ---
        Node(
            package='endurance',
            executable='rsds_camera_publisher.py',
            name='rsds_camera_publisher',
            output='screen',
        ),

        # --- LKA LATERAL CONTROLLER ---
        Node(
            package='lka',
            executable='lane_detection_node',
            name='lane_detection_node',
            output='screen',
            parameters=[config_file],
        ),

        Node(
            package='lka',
            executable='stanley_controller_node',
            name='stanley_controller_node',
            output='screen',
            parameters=[config_file],
        ),

        Node(
            package='lka',
            executable='vehicle_control_node',
            name='vehicle_control_node',
            output='screen',
            parameters=[config_file],
        ),

        Node(
            package='lka',
            executable='performance_logger_node',
            name='performance_logger_node',
            output='screen',
            parameters=[config_file],
        ),

        # --- VISUALIZATION ---
        Node(
            package='image_tools',
            executable='showimage',
            name='lane_visualization',
            remappings=[('image', '/lka/lane_image')],
            output='screen',
        )
    ])
