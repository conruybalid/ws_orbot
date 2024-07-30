from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
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

        Node(
            package='image_processing',
            executable='arm_location_service',
            arguments=['--ros-args', '--log-level', 'WARN']

        ),

        Node(
            package='get_pic',
            executable='zed_publisher',
            arguments=['--ros-args', '--log-level', 'WARN']
        ),

        Node(
            package='image_processing',
            executable='zed_location_service',
            arguments=['--ros-args', '--log-level', 'WARN']
        ),
        Node(
            package='arm_control',
            executable='arm_move_action',
            arguments=['--ros-args', '--log-level', 'INFO']

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