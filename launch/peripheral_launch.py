from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    kinova_vision_launch_file = os.path.join(
        get_package_share_directory('kinova_vision'),
        'launch',
        'kinova_vision.launch.py'
    )
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_MIN_SEVERITY_LEVEL', 'WARN'),
        Node(
            package='get_pic',
            executable='video_publisher',
            arguments=['--ros-args', '--log-level', 'WARN']
        ),
        Node(
            package='get_pic',
            executable='video_subscriber',
            arguments=['--ros-args', '--log-level', 'INFO']

        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinova_vision_launch_file),
            launch_arguments={'launch_color': 'false'}.items()
        ),

        Node(
            package='image_processing',
            executable='arm_location_service',
            arguments=['--ros-args', '--log-level', 'INFO']

        ),

        Node(
            package='get_pic',
            executable='zed_publisher',
            arguments=['--ros-args', '--log-level', 'WARN']
        ),

        Node(
            package='image_processing',
            executable='zed_location_service',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
        Node(
            package='arm_control',
            executable='arm_move_action',
            arguments=['--ros-args', '--log-level', 'WARN']

        ),
        Node(
            package='tank_control',
            executable='move_tank',
            arguments=['--ros-args', '--log-level', 'WARN']
        ),
        Node(
            package='xbox_controller',
            executable='controller_node',
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
    
    ])